#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>

using namespace std;

class PID
{
    private:
        struct gains_t
        {
            float kp = 0.1;
            float ki = 0.05;
        } gains;

    public:
        PID();
        PID(double Mach, double altitude);
        float get_kp();
        float get_ki();
};
#endif