#ifndef BAYES_HPP
#define BAYES_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/task.hpp>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>
#include <map>
#include <string>

namespace periodics
{
   /**
    * @brief Class bayes
    *
    */
    class CBayes : public utils::CTask
    {
        public:
            /* Construnctor */
            CBayes(
                uint32_t f_period,
                UnbufferedSerial& f_serial              
            );
            /* Destructor */
            ~CBayes();
            void setControl(const std::string& key, float value);
            void setMeasurements(const std::string& key, float value);
            void setSpeedFlag(bool value);
            void setSteeerFlag(bool value);
            void setMeasurementsFlag(bool value);
            void serialCallbackStateUPDATE(const char* message, char* response);
        private:
            UnbufferedSerial&      m_serial;
            /** @brief dictionary with measurement values from imu */
            std::map<std::string, float>state_measurements;
            /** @brief dictionary with predictions values */
            std::map<std::string, float>state_predictions;
            /** @brief dictionary with update values */
            std::map<std::string, float>state_updates;
            /** @brief dictionary with fused values */
            std::map<std::string, float>state_fused;
            /** @brief command vector from robotstatemachine */
            std::map<std::string, float>controls;
            /** @brief variance for estimated orientation from IMU */
            float variance_predict;
            /** @brief variance for updated orientation fom CAMERA */
            float variance_update;
            /** @brief variance for fused orientation */
            float variance_fused;
            /** @brief flags */
            bool flagMeasurements;
            bool flagSpeed;
            bool flagSteer;
            bool flagUpdate;
            /** @brief sampling time */
            float dt;
            /** @brief wheelbase */
            float L;
            float constant;
            /** @brief noise */
            float Q_noise;
            float R_noise;
            /** @brief Run method */
            virtual void _run();
            /** @brief Predict step of kalman */
            void Predict(char *buffer, size_t size);
            /** @brief Update step of Kalman */
            void Update(char *buffer, size_t size);
            /** @brief Compute Camera Model */
            void ComputeCameraModel(char *buffer, size_t size);
            /** @brief Compute IMU Model */
            void ComputeIMUModel(char *buffer, size_t size);
    }; // class CBayes
}; // namespace periodics

#endif // BAYES_HPP