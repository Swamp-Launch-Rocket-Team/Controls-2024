#include <iostream>
#include <cmath>

using namespace std;

//Before implementing this in the dynamics model, make z the input and rho the output.
int main()
{
    double z = 3002.3;     //Altitude [m], THIS IS THE INPUT

    double T0 = 310;        //Temperature at ground level [K]
    double L = 0.0065;      //Lapse rate 

    double T = T0 - L*z;        //Temperature at current altitude [K]

    double rho0 = 1.13;     //Air density at ground level, THIS NEEDS TO CHANGE BASED ON FLORIDA OR SPACEPORT LAUNCH SITE
    double g = 9.81;        //Gravity [m/s^2]
    double R = 287.0531;    //Universal gas constant for air

    double rho = rho0*pow(T/T0, (((g/(L*R))) - 1));     //Air density at altitude [kg/m^3], THIS IS THE OUTPUT

    cout << rho << endl;

    return 0;
}