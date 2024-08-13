/* Include guard */
#ifndef HALL_HPP
#define HALL_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the task manager library, which applies periodically the fun function of its children */
#include <utils/task.hpp>

namespace periodics
{
   /**
    * @brief It is used for reading a Hall sensor.
    * 
    */
    class CHall : public utils::CTask
    {
        public:
            /* Constructor */
            CHall(
                uint32_t            f_period, 
                PinName             f_pin,
                UnbufferedSerial&   f_serial
            );
            /* Destructor */
            ~CHall();
            /* Serial callback implementation */
            void HallPublisherCommand(char const * message, char * response);
        private:
            /* Run method */
            virtual void        _run();
            /* Digital interrupt input pin connected to a Hall sensor */
            mbed::InterruptIn     m_pin;
            /* @brief Serial communication obj.  */
            UnbufferedSerial&   m_serial;
            /** @brief Active flag  */
            bool                m_isActive;
            /** @brief Pulses counter */
            int m_counter;
            /** @brief Previous pulses counter */
            int m_previousCounter;
            /** @brief Motor poles */
            float m_poles;
            /** @brief m_wheelDiameter */
            float m_wheelDiameter = 0.06;
            void hallSensorRise();
    }; // class CHall
}; // namespace periodics

#endif // HALL_HPP