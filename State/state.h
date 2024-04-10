#ifndef STATE_H
#define STATE_H

#include "../IMU/imu.h"
#include <vector>

//Stores information on the airbrakes state
struct state_t
{
    imu_data_t imu_data;

    float theta = 0.0;

    float prev_accelz = 0.0;

    
    vector<float> accel_window1{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        //For velo during launch detect
    vector<float> accel_window2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};        //For velo during launch detect
    vector<float> dt_window1{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};            //For velo during lauch detect
    vector<float> dt_window2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};            //For velo during launch detect
    vector<float> theta_window1{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};            //For velo during launch detect
    vector<float> theta_window2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};            //For velo during launch detect


    //Altimeter altitude
    struct altimeter_t
    {
        float pressure4 = 0.0;    
        float pressure3 = 0.0;
        float pressure2 = 0.0;
        float pressure1 = 0.0;
        float pressure = 0.0;

        float filt_pressure4 = 0.0;
        float filt_pressure3 = 0.0;
        float filt_pressure2 = 0.0;
        float filt_pressure1 = 0.0;
        float filt_pressure = 0.0;

        float temp2 = 0.0;
        float temp1 = 0.0;
        float temp = 0.0;

        float z_prev = 0.0;
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

        float integral_velox = 0.0;
        float prev_integral_velox = 0.0;

        float integral_veloz = 0.0;
        float prev_integral_veloz = 0.0;
    } velo;

    //Controller and dynamics model outputs
    struct airbrake_t
    {
        float apogee_expected = 0.0;
        float U_airbrake = 0.0;
        float prev_error = 0.0;
    } airbrake;

    enum status_t
    {
        PAD,
        ARMED,
        LAUNCH_DETECTED,
        ACTUATION,
        DESCENT,
        LAND,
    } status;
};

#endif