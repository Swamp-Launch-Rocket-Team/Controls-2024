#ifndef STATE_H
#define STATE_H

#include "../IMU/imu.h"

//Stores information on the airbrakes state
struct state_t
{
    imu_data_t imu_data;

    //Altimeter altitude
    struct altimeter_t
    {
        float pressure4 = 0.0;    
        float pressure3 = 0.0;
        float pressure2 = 0.0;
        float pressure1 = 0.0;
        float pressure = 0.0;
        float temp = 0.0;
        float z4 = 0.0;
        float z3 = 0.0;
        float z2 = 0.0;
        float z1 = 0.0;
        float z = 0.0;
    } altimeter;

    //Velo struct
    struct velo_t
    {
        float xdot_4 = 0.0;
        float xdot_3 = 0.0;
        float xdot_2 = 0.0;
        float xdot_1 = 0.0;
        float xdot = 0.0;

        float zdot_4 = 0.0;
        float zdot_3 = 0.0;
        float zdot_2 = 0.0;
        float zdot_1 = 0.0;
        float zdot = 0.0;

        float Mach = 0.0;
    } velo;

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