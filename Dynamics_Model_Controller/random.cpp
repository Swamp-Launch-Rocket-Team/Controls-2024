#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

#define R 287.058
#define g 9.81

float Altitude_calc(float pressure);

int main()
{
    float pressure = 101025.0/100.0;
    cout << pressure << endl;
    float z = Altitude_calc(pressure);
    cout << z << endl;
    return 0;
}

float Altitude_calc(float pressure)
{
    pressure = pressure*100;        //mbar to Pa
    float altitude = (288.15/0.0065)*(1-(pow(pressure/101325,0.0065*(R/g))));
    return altitude;
}