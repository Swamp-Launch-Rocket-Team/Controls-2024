#include <iostream>
#include <cmath>
#include "controller.h"
#include "PI.h"

using namespace std;

//Defualt constructor
controller::controller()
{

}

void controller::init_controller(float Pwm_home_value, float Pwm_max_value) //This only gets called once in main
{
    //initialize the variables needed for the controller?? maybe make sure all of the variables in the controller header are correct
    //airbrake_output should be set to "home" pwm signal
    this->parameters.integral = 0;
    this->parameters.limMin = Pwm_home_value;
    this->parameters.limMax = Pwm_max_value;
    this->parameters.limMin_Integrator = 0;
    this->parameters.limMax_Integrator = 1;
    this->parameters.prev_error = 0;
    this->parameters.airbrake_output = Pwm_home_value;
    this->parameters.slope_PWM = (Pwm_max_value - Pwm_home_value)/(1.0 - 0.0);
    this->parameters.b_PWM = Pwm_home_value;
}
//was float as return type
float controller::controller_loop(double apogee_expected, double Mach, double altitude)       //This gets called at every time step in main during ACTUATION state
{
    //Get the current gains
    //calculate error
    //Get the proportional input term
    //do the integral stuff including the anti-windup limits
    //find proportional and integral error
    //set current stuff to previous
    //do other stuff
    //end? idk 

        //Read the IMU for velocity to get Mach number: JK this will be done outside of the controller, this is now an input
        //Read the altimeter for the altitude: JK this will be done outside of the controller, this is now an input

    //Get current gains
    PI cur_gains;
    // cout << "Current Kp Gain:" << "\t" << cur_gains.get_kp() << "\t" << "Current Ki Gain:" << "\t" << cur_gains.get_ki() << endl;

    //Calculate current error
    double cur_error = apogee_expected - parameters.setpoint;
    //cout << "Current error:" << "\t" << cur_error << endl;

    //Proportional
    double proportional = cur_gains.get_kp()*(cur_error);
    //cout << "Proportional Portion:" << "\t" << proportional << endl;

    // if(proportional < 0)     //Idk if we need this?????
    // {
    //     proportional = 0;
    // }

    //Inetgrator Limits: This is working.
    if(0 < proportional)
    {
        this->parameters.limMin_Integrator =  0 - proportional;
    } else
    {
        this->parameters.limMin_Integrator = 0;
    }

    if(1 > proportional)
    {
        this->parameters.limMax_Integrator = 1 - proportional;
    } else
    {
        this->parameters.limMax_Integrator = 0;
    }

    //Integral
    double T = 0.001;        //Loop time of the controller (this will be found using chrono on the Pi), in seconds
    this->parameters.integral += cur_gains.get_ki()*(T/2)*(cur_error + parameters.prev_error);
    //cout << "Integral portion:" << "\t" << parameters.integral << endl;

    //Clamp Integral: This is working.
    if(parameters.integral > parameters.limMax_Integrator)
    {
        this->parameters.integral = parameters.limMax_Integrator;
    } else if(parameters.integral < parameters.limMin_Integrator)
    {
        this->parameters.integral = parameters.limMin_Integrator;
    }

    //Total airbrake output (P + I)
    this->parameters.airbrake_output = proportional + parameters.integral; //This is not the PWM value, should be from 0 - 1 ish
    //cout << "Original airbrake output [0 -> 1] U_airbrake:" << "\t" << parameters.airbrake_output << endl;

    //Transform output to PWM signal using the linear mapping
    // this->parameters.airbrake_output = parameters.slope_PWM*parameters.airbrake_output + parameters.b_PWM;  //PWM value

    //cout << "Airbrake output in PWM:" << "\t" << parameters.airbrake_output << endl;
    
    //Check if the PWM signal is larger than the max and min range, determined by the PAD ARMING state
    // if(parameters.airbrake_output < parameters.limMin)
    // {
    //     this->parameters.airbrake_output = parameters.limMin;
    // } else if(parameters.airbrake_output > parameters.limMax)
    // {
    //     this->parameters.airbrake_output = parameters.limMax;
    // }
    if(parameters.airbrake_output < 0)
    {
        this->parameters.airbrake_output = 0;
    } else if(parameters.airbrake_output > 1)
    {
        this->parameters.airbrake_output = 1;
    }

    //cout << parameters.airbrake_output << endl;

    // cout << "Previous error:" << "\t" << parameters.prev_error << endl;

    //Set the previous error to the current error
    this->parameters.prev_error = cur_error;        //set previous error to the current error

    return this->parameters.airbrake_output;        //return the airbrake output [0->1]
}

float controller::get_airbrake_output()
{
    this->parameters.airbrake_output = parameters.slope_PWM*parameters.airbrake_output + parameters.b_PWM;  //PWM value
    if(parameters.airbrake_output < parameters.limMin)
    {
        this->parameters.airbrake_output = parameters.limMin;
    } else if(parameters.airbrake_output > parameters.limMax)
    {
        this->parameters.airbrake_output = parameters.limMax;
    }
    return this->parameters.airbrake_output;        //Output PWM signal
}

float controller::get_integral()
{
    return this->parameters.integral;
}

float controller::get_prev_error()
{
    return this->parameters.prev_error;
}

float controller::get_limMin()
{
    return this->parameters.limMin;
}

float controller::get_limMax()
{
    return this->parameters.limMax;
}

float controller::get_limMin_Integrator()  //getter for the min integral value
{
    return this->parameters.limMin_Integrator;
}

float controller::get_limMax_Inetgrator() //getter for the max integral value
{
    return this->parameters.limMax_Integrator;
}

float controller::get_setpoint()
{
    return this->parameters.setpoint;
}

float controller::get_slope_PWM()         //getter for the slope of the mapping from the U_airbrake [0->1] into PWM signal
{
    return this->parameters.slope_PWM;
}

float controller::get_b_PWM()             //getter for the y-intercept of the mapping from the U_airbrake [0->1] into PWM signal
{
    return this->parameters.b_PWM;
}