#ifndef STATE_H
#define STATE_H

// #include "../IMU/imu.h"

//Stores information on the airbrakes state
struct state_t
{
    // imu_data_t imu_data;

    enum status_t
    {
        PAD,
        ARMED,
        LAUNCH_DETECTED,
        ACTUATION,
        APOGEE_DETECTED
    } status;
};

#endif