#ifndef PI_H
#define PI_H

#include <iostream>
#include <cmath>

using namespace std;

class PI
{
    //Gains structure, these are the defualt values when using the default constructor
    private:
        struct gains_t
        {
            float kp = 0.1;     //proportional gain, DEPENDENT      --
            float ki = 0.005;    //integral gain, DEPENDENT         --
        } gains;

    public:
        PI();                                   //defualt constructor
        PI(double Mach, double altitude);       //gain-scheduled constructor
        float get_kp();                         //getter method for the proportional gain
        float get_ki();                         //getter method for the integral gain
};

#endif