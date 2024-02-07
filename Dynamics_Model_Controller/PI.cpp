#include "PI.h"
#include <iostream>

using namespace std;

//Defualt constructor
PI::PI()
{

}

//Gain-scheduled constructor
PI::PI(double Mach, double altitude)
{
    this->gains.kp = 0.1*Mach + 0.00001*altitude;       //This eqn is completely arbitrary rn, it is waiting for the gain-scheduling analysis to be completed
    this->gains.ki = (0.1*Mach + 0.01*altitude)/1;      //This eqn is completely arbitrary rn, it is waiting for the gain-scheduling analysis to be completed

    if(gains.kp < 0)
    {
        this->gains.kp = 0;
    } else if(gains.ki < 0)
    {
        this->gains.ki = 0;
    }
}

//proportional gain getter method
float PI::get_kp()
{
    return this->gains.kp;
}

//integral gain getter method
float PI::get_ki()
{
    return this->gains.ki;
}

