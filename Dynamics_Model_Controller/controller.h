#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cmath>
#include "PI.h"

using namespace std;

class controller
{
    private:
    struct parameters
    {
        float integral = 0.0;               //Integral of the error
        float prev_error = 0.0;             //Previous error
        float limMin = 345;                 //Min PWM signal
        float limMax = 625;                //Max PWM signal
        float limMin_Integrator = 0;        //Minimum integral value for dynamic clamping (anti windup)
        float limMax_Integrator = 1;        //Max integral value for dynamic clamping (anti windup)
        const float setpoint = 3048;        //10,000 ft in meters
        float slope_PWM = 280;              //slope for taking airbrake from 0->1 into PWM signal
        float b_PWM = 345;                   //y-intercept for taking airbrake from 0->1 into PWM signal
        float airbrake_output = 0;          //Output in PWM
    } parameters;

    public:
        controller();       //Defualt constructor
        void init_controller(float Pwm_home_value, float Pwm_max_value);        //Controller initialization method
        float controller_loop(double apogee_expected, double Mach, double altitude);      //Controller loop method, returns PWM signal airbrake_output
        float get_integral();           //getter for the integral
        float get_prev_error();         //getter for the previous error
        float get_limMin();             //getter for the minimum PWM signal
        float get_limMax();             //getter for the max PWM signal
        float get_limMin_Integrator();  //getter for the min integral value
        float get_limMax_Inetgrator();  //getter for the max integral value
        float get_setpoint();           //getter for the setpoint
        float get_slope_PWM();          //getter for the slope of the mapping from the U_airbrake [0->1] into PWM signal
        float get_b_PWM();              //getter for the y-intercept of the mapping from the U_airbrake [0->1] into PWM signal
        float get_airbrake_output();    //getter for the airbrake output PWM signal
};

#endif