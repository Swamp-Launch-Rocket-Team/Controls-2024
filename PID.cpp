#include "PID.h"
#include <iostream>

using namespace std;

PID::PID()
{

}

PID::PID(double Mach, double altitude)
{
    this->gains.kp = 0.1*Mach + 0.01*altitude;
    this->gains.ki = (0.1*Mach + 0.01*altitude)/10;
}

float PID::get_kp()
{
    return this->gains.kp;
}

float PID::get_ki()
{
    return this->gains.ki;
}

