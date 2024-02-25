#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std;

#define R 287.058
#define g 9.81

float Altitude_calc(float pressure);

int main()
{
    // float pressure = 101025.0/100.0;
    // cout << pressure << endl;
    // float z = Altitude_calc(pressure);
    // cout << z << endl;

    auto t = chrono::high_resolution_clock::now();
    float res = 2000.0/50.0;
    for (int i = 0; i < 100; i++)
    {
        float x = i;
    }
    auto t2 = chrono::high_resolution_clock::now();
    cout << chrono::duration<double>(t2 - t).count() << endl;
    cout << chrono::duration_cast<chrono::nanoseconds>(t2 - t).count() << endl;
    for (int i = 0; i < 10000; i++)
    {
        float x = i;
    }
    auto t3 = chrono::high_resolution_clock::now();
    cout << chrono::duration<double>(t3 - t).count() << endl;
    cout << chrono::duration_cast<chrono::nanoseconds>(t3 - t).count() << endl;



    return 0;
}

float Altitude_calc(float pressure)
{
    pressure = pressure*100;        //mbar to Pa
    float altitude = (288.15/0.0065)*(1-(pow(pressure/101325,0.0065*(R/g))));
    return altitude;
}