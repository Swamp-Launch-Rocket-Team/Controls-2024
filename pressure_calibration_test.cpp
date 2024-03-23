#include <iostream>
#include <chrono>
#include "Altimiter/altimiter.h"
#include "IMU/imu.h"
#include "busynano/busynano.h"
#include "bitbang/bitbang.h"
#include "State/state.h"
#include <vector>
#include <fstream>
#include <numeric>

#define press_min 950.0
#define press_max 1200.0
#define press_expected 1017.21
#define temp_min 20.0
#define temp_max 25.0
#define temp_expected 23.0

using namespace std;

int main()
{
    spi_init_bitbang();

    alt_init();

    imu_init();

    ofstream file;
    file.open("Calibration Testing.csv");

    state_t state;

    float T0 = 0.0;
    float P0 = 0.0;

    vector<float> press_cal;
    vector<float> temp_cal;

    auto t_start = chrono::high_resolution_clock::now();
    auto t_cur = chrono::high_resolution_clock::now();

    while (chrono::duration<double>(t_cur - t_start).count() < 120)
    {
        t_cur = chrono::high_resolution_clock::now();

        altimiter_t alt_data = get_temp_and_pressure();
        state.altimeter.pressure = alt_data.pressure;
        state.altimeter.temp = alt_data.temp;

        if (state.altimeter.pressure > press_max || state.altimeter.pressure < press_min)
        {
            state.altimeter.pressure = press_expected; 
        }

        state.altimeter.pressure4 = state.altimeter.pressure3;
        state.altimeter.pressure3 = state.altimeter.pressure2;
        state.altimeter.pressure2 = state.altimeter.pressure1;
        state.altimeter.pressure1 = state.altimeter.pressure;

        if (state.altimeter.temp > temp_max || state.altimeter.temp < temp_min) //Check to make sure temp is in expected range
        {
            state.altimeter.temp = temp_expected;       //set temp to expected value if not in expected ranged
        }

        press_cal.push_back(state.altimeter.pressure);  //Add the pressure value to the pressure cal vector
        temp_cal.push_back(state.altimeter.temp);       //Add the temp value to the temp cal vector

        P0 = (accumulate(press_cal.begin(),press_cal.end(),0.0))/press_cal.size();  //Find the average pressure and set to ground pressure
        T0 = (accumulate(temp_cal.begin(),temp_cal.end(),0.0)/temp_cal.size());

        file << chrono::duration<double>(t_cur - t_start).count() << "," << state.altimeter.pressure4 << "," << state.altimeter.pressure3 << "," << state.altimeter.pressure2 << "," << state.altimeter.pressure1 << "," << state.altimeter.pressure << "," << state.altimeter.temp << "," << P0 << "," << T0 << endl;

    }
    cout << "Calibration Testing Completed" << endl;
    return 0;
}