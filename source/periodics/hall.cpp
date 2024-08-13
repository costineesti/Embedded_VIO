#include "periodics/hall.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace periodics
{
   /**
    * @brief Class constructor
    * 
    * It initializes the task and the state of the Hall sensor.
    * 
    */
    CHall::CHall(
            uint32_t            f_period, 
            PinName             f_pin,
            UnbufferedSerial&   f_serial
        ) 
        : utils::CTask(f_period)
        , m_pin(f_pin)
        , m_serial(f_serial)
        , m_isActive(false)
        , m_counter(0)
        , m_poles(4.0f)
        , m_previousCounter(0)
    {
        // Attach an interrupt to the Hall sensor
        m_pin.fall(callback(this, &CHall::hallSensorRise));
    }

    /** @brief  CHall class destructor
     */
    CHall::~CHall()
    {
    };

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param message           input received string
     * @param response          output reponse message
     * 
     */
    void CHall::HallPublisherCommand(char const * message, char * response) {
        int l_isActivate=0;
        uint32_t l_res = sscanf(message,"%d",&l_isActivate);
        if(l_res==1){
            m_isActive = (l_isActivate>=1);
            sprintf(response,"ack;;");
        }else{
            sprintf(response,"sintax error;;");
        }
    }

    void CHall::hallSensorRise()
    {
        char buffer[256];
        this->m_counter++;
        // snprintf(buffer, sizeof(buffer), "@3:%d;;\r\n", this->m_counter);
        // m_serial.write(buffer, strlen(buffer));
    }

    /** \brief  Periodically applied method to read the Hall sensor's state
     * 
     */
    void CHall::_run()
    {   
        if(!m_isActive) return;
        // char buffer[256];

        // Calculate the speed of the wheel
        if (this->m_counter >= this->m_poles) {
            float revolutions = this->m_counter / this->m_poles;
            float distance = revolutions * this->m_wheelDiameter * M_PI;
            float speed = distance / 0.1; // As the code executes every 100ms
            this->m_counter = 0;
            // snprintf(buffer, sizeof(buffer), "@3:%f;;\r\n", speed);
            // m_serial.write(buffer, strlen(buffer));
        }
        else if (this->m_counter == this->m_previousCounter) {
            this->m_counter = 0;
        }
        this->m_previousCounter = this->m_counter;
    }

}; // namespace periodics