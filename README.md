# Embedded_VIO
The Embedded code that I used for my bachelor's degree. It is mainly developed by the BFMC (Bosch Future Mobility Challenge) team. I only added what I needed (the control to the motors and the Kalman Filter).

* In order to flash NUCLEO please type in terminal "mbed-tools compile -m NUCLEO_F446RE -t GCC_ARM" inside the top folder. Of course change the version of the NUCLEO with yours.

* The project contains all the software present on the Nucleo board, together with the documentation on how to create new components and what are the features of the given one. Some of the feature are:
- Communication protocol between RPi and Nucleo,
- Motors control,
- IMU readings
- Notifications from Power Board
- Architecture prone to features addition

## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/embeddedplatform.html) 
