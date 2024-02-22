#include <iostream>
#include "PI.h"
#include "controller.h"

using namespace std;

// class PID
// {
//     public:
//         double kp = 0.1;
//         double ki = 0.05;
        
//         PID()
//         {

//         }

//         PID(double Mach, double altitude)
//         {
//             this->kp = 0.1*Mach + 0.01*altitude;
//             this->ki = (0.1*Mach + 0.01*altitude)/10;
//         }
// };

int main()
{
    // double M = 0.5;
    // double z = 100;

    // PI gains(M,z);

    // cout << gains.get_kp() << "\t" << gains.get_ki() << endl;

    float Pwm_home_value = 35;
    float Pwm_max_value = 1250;
    double apogee_expected = 3085;
    double Mach = 0.6;
    double altitude = 1000;
    controller test;
    test.init_controller(Pwm_home_value, Pwm_max_value);
    for(int i = 0; i < 100; i++)        //This test the controller loop at constant Mach and altitude. Provides insight into how the controller is working and anti-windup
    {
    test.controller_loop(apogee_expected, Mach, altitude);

    apogee_expected -= 0.7*((test.get_airbrake_output() - test.get_b_PWM())/test.get_slope_PWM());      //THIS MULTIPLIES 0.7*U_airbrake [0->1], THAT PORTION INSIDE MAPS FROM PWM TO [0->1]
    //Mach -= 0.5;
    //altitude -= 10;

    cout << "Final controller output:" << "\t" << test.get_airbrake_output() << endl;

    cout << test.get_prev_error() << endl;
    cout << i << endl;
    }

    //This currently has the integral anti-windup working using dynamic clamping.

    return 0;
}

