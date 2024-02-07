#include <iostream> 
#include <cmath>

using namespace std;

int main()
{
    double d_rocket = 6.14*0.0254;    //diameter of rocket in meters, 6.14 in = 0.1560 m
    double A_rocket;
    A_rocket = (0.25)*(M_PI)*pow(d_rocket, 2);

    float U_airbrake = 0.25;       //Input of the airbrake, this is from 0 -> 1, THIS WILL BE THE INPUT 
    double Servo_angle = 105*U_airbrake;        //Servo angle in degrees
    
    double A_airbrake;

    if(U_airbrake == 0)
    {
        A_airbrake = 0;
    }
    else
    {
        //Max cross-sectional area of airbrake during extension in [m^2]
        A_airbrake = 0.00064516*(8.48389446479259e-8*pow(Servo_angle, 4) - 0.0000348194172718423*pow(Servo_angle, 3) + 0.00388560760536394*pow(Servo_angle, 2) - 0.0629348277080075*Servo_angle + 0.148040926341214); 
    }

    double A_cross = A_rocket + A_airbrake;     //Cross sectional area of the rocket, THIS IS THE OUTPUT

    cout << A_cross << "\t" << M_PI << endl;

    return 0;
}