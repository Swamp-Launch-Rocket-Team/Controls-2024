#ifndef SERVO_ARMING 
#define SERVO_ARMING

#include <wiringPi.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

using namespace std;

class Servo
{
    //struct for the arming calibration
    private:
        struct arming_t
        {
            double Pwm_home_value;      //final home PWM value, this will be an output of the Arming state
            double Pwm_max_value;       //final max PWM value, this will be an output of the Arming state
        } arming;

    public:
        Servo();
        void Servo_cal(int Pwm_pin, float Pwm_init_val, vector<int> homing_limit_switches, vector<int> max_limit_switches);
        float get_Pwm_init_value();
        double get_Pwm_home_value();
        double get_Pwm_max_value();
};

#endif