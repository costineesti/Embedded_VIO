#include "bayes.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace periodics
{
   /**
    * @brief Class constructorbayes
    *
    * @param f_period              period for controller execution in seconds
    */
    CBayes::CBayes(
        uint32_t                      f_period,
        UnbufferedSerial&             f_serial
    )
    : utils::CTask(f_period)
    , m_serial(f_serial)
    , variance_predict(0.2f)
    , variance_update(0.5f)
    , variance_fused(0.0f)
    , flagMeasurements(false)
    , flagSpeed(false)
    , flagSteer(false)
    , flagUpdate(false)
    , dt(0.1f)
    , L(26.0f)
    , constant(360.0f*dt/(2.0f*M_PI*L))
    , Q_noise(0.001f)
    , R_noise(0.001f)

    {        
        this->state_fused["x"] = 0.0f;
        this->state_fused["y"] = 0.0f;
        this->state_fused["psi"] = 0.0f;
        this->state_measurements["x"] = 0.0f;
        this->state_measurements["y"] = 0.0f;
        this->state_measurements["psi"] = 0.0f;
        this->state_predictions["x"] = 0.0f;
        this->state_predictions["y"] = 0.0f;
        this->state_predictions["psi"] = 0.0f;
        this->state_updates["x"] = 0.0f;
        this->state_updates["y"] = 0.0f;
        this->state_updates["psi"] = 0.0f;
    }

    /** @brief  CBayes class destructor
     */
    CBayes::~CBayes()
    {
    };

    /** @brief setter for controls dictionary */
    void CBayes::setControl(const std::string& key, float value) {
        this->controls[key] = value;
    }
    
    /** @brief setter for prediction dictionary */
    void CBayes::setMeasurements(const std::string& key, float value) {
        this->state_measurements[key] = value;
    }

    /** @brief setter for controls flag */
    void CBayes::setSpeedFlag(bool value) {
        this->flagSpeed = value;
    }

    /** @brief setter for controls flag */
    void CBayes::setSteeerFlag(bool value) {
        this->flagSteer = value;
    }

        /** @brief setter for measurements flag */
    void CBayes::setMeasurementsFlag(bool value) {
        this->flagMeasurements = value;
    }

    float rad2angle(float rad){
        return rad*180.0f/M_PI;
    }

    float angle2rad(float angle){
        return angle*M_PI/180.0f;
    }

    /** @brief This function returns an angle between [0,360) */
    float normalize_angle(float angle){
        float angle_norm = fmod(angle, 360.0f);
        if(angle_norm < 0){
            angle_norm += 360.0f;
        }
        return angle_norm;
    }

    float convertToPiRange(float angle_deg) {
    // Convert to radians
    float  angle_rad =  angle2rad(angle_deg);

    // Adjust to [-π, +π] range
    if (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    } else if (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}


    /** @brief callback method to get the state update [x,y,psi] from the camera */
    void CBayes::serialCallbackStateUPDATE(char const * a, char * b)
    {
        float psi;

        uint32_t parsed = sscanf(a, "%f", &psi);
        if(parsed == 1)
        {
            this->state_updates["psi"] = psi;
            this->flagUpdate = true;
            sprintf(b,"ack");
        }
        else
        {
            sprintf(b, "something went wrong");
        }
    }

    void CBayes::ComputeIMUModel(char *buffer, size_t size){
        
        this->state_measurements["x"] += this->controls["speed"]*std::cos(convertToPiRange(this->state_measurements["psi"]))*this->dt;
        this->state_measurements["y"] += this->controls["speed"]*std::sin(convertToPiRange(this->state_measurements["psi"]))*this->dt;

        snprintf(buffer, size, "@6:%.3f;%.3f;%.3f;;\r\n",
                state_measurements["x"], state_measurements["y"], state_measurements["psi"]);
        m_serial.write(buffer,strlen(buffer));
    }

    void CBayes::ComputeCameraModel(char *buffer, size_t size){

        this->state_updates["x"] += this->controls["speed"]*std::cos(convertToPiRange(state_updates["psi"]))*this->dt;
        this->state_updates["y"] += this->controls["speed"]*std::sin(convertToPiRange(state_updates["psi"]))*this->dt;
        this->variance_update += this->R_noise; // Update covariance

        snprintf(buffer, size, "@5:%.3f;%.3f;%.3f;;\r\n",
            state_updates["x"], state_updates["y"], state_updates["psi"]);
        m_serial.write(buffer,strlen(buffer));
    }

    void CBayes::Predict(char *buffer, size_t size){
        // Prediction step
        // Use the controls and the previous state to predict the current state
        float control = this->controls["speed"]*this->constant;
        this->state_predictions["psi"] = this->state_measurements["psi"] + control*std::tan(convertToPiRange(this->controls["steering"]));
        this->state_predictions["psi"] = normalize_angle(this->state_predictions["psi"]);

        this->state_predictions["x"] += this->controls["speed"]*std::cos(convertToPiRange(this->state_predictions["psi"]))*this->dt;
        this->state_predictions["y"] += this->controls["speed"]*std::sin(convertToPiRange(this->state_predictions["psi"]))*this->dt;

        // Update the variance
        this->variance_predict += this->Q_noise; // Update covariance

        // Send the prediction to Raspberry PI
        snprintf(buffer, size, "@7:%.3f;%.3f;%.3f;;\r\n",
                state_predictions["x"], state_predictions["y"], state_predictions["psi"]);
        m_serial.write(buffer,strlen(buffer));
    }

    void CBayes::Update(char *buffer, size_t size){
        // Update step
        // Check for angles in 1st and 4th quadrant and bring them to same quadrant
        if (state_predictions["psi"] > 270.0f && this->state_updates["psi"] < 90.0f){
            state_predictions["psi"] = 360.0f - state_predictions["psi"];
        }
        else if (state_predictions["psi"] < 90.0f && this->state_updates["psi"] > 270.0f){
            this->state_updates["psi"] = 360.0f - this->state_updates["psi"];
        }
        
        // Use the camera odometry to correct the prediction
        float z = this->state_updates["psi"]; // Camera orientation
        float y = z - this->state_predictions["psi"]; // compute the output error
        float K = this->variance_predict/(this->variance_predict + this->variance_update); // Compute the Kalman gain
        
        // Update the state
        this->state_fused["psi"] = this->state_predictions["psi"] + K*y;
        this->state_fused["psi"] = normalize_angle(this->state_fused["psi"]);
        this->state_fused["x"] += this->controls["speed"]*std::cos(convertToPiRange(this->state_fused["psi"]))*this->dt;
        this->state_fused["y"] += this->controls["speed"]*std::sin(convertToPiRange(this->state_fused["psi"]))*this->dt;

        // Update the variance
        this->variance_fused = (1-K)*this->variance_predict; // Update covariance

        // Send the updated state to Raspberry PI
        snprintf(buffer, size, "@8:%.3f;%.3f;%.3f;;\r\n",
            state_fused["x"], state_fused["y"], state_fused["psi"]);
        m_serial.write(buffer,strlen(buffer));
    }

    void CBayes::_run()
    {
        char buffer[1024];
        if(this->flagSteer == true && this->flagSpeed == true && this->flagMeasurements == true && this->flagUpdate==true) // adauga inapoi flag_update
        {
            // if I have all the data, then I can start the KALMAN filter
            // ====================================    IMU    =====================================
            ComputeIMUModel(buffer, sizeof(buffer));
            // =====================================  CAMERA  =====================================
            ComputeCameraModel(buffer, sizeof(buffer));
            // =====================================  PREDICT =====================================
            Predict(buffer, sizeof(buffer));
            // =====================================  UPDATE  =====================================
            Update(buffer, sizeof(buffer));

            // Reset flags
            this->flagSteer = false;
            this->flagMeasurements = false;
            this->flagUpdate = false;
        }
    }

}; // namespace periodics