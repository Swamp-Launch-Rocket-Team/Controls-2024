#include <iostream>
#include <chrono>
#include "Altimiter/altimiter.h"
#include "IMU/imu.h"
#include "fstream"

#include "busynano/busynano.h"
#include "bitbang/bitbang.h"

void print_all(imu_data_t *imu_data)
{
    printf("Heading: %.4f\t%.4f\t%.4f\tACCEL: %.4f\t%.4f\t%.4f\t\n",
        imu_data->heading.x,imu_data->heading.y,imu_data->heading.z,
        imu_data->accel.x, imu_data->accel.y, imu_data->accel.z);
}

#define R 287.058f
#define g 9.81f
float Altitude_calc(float pressure)
{
    pressure = pressure*100;        //mbar to Pa
    float altitude = (288.15/0.0065)*(1-(pow(pressure/101325,0.0065*(R/g))));
    return altitude;
}

int main()
{
    spi_init_bitbang();

    alt_init();

    imu_init();

    std::ofstream sensor_testing;
    sensor_testing.open("Sensor Testing");

    auto start = chrono::high_resolution_clock::now();

    int ii = 0;

    while (ii < 72000)
    {
        int a = 0;
        auto cur = chrono::high_resolution_clock::now();

        altimiter_t alt_data = get_temp_and_pressure();

        imu_data_t imu_data = imu_read_data();

        auto t_sample = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - cur).count();
        // std::cout << "Time to sample:\t" << t_sample << std::endl;

        // print_all(&imu_data);

        // float alt = Altitude_calc(alt_data.pressure);
        // std::cout << "TEMP: " + std::to_string(alt_data.temp) << "\tPRESS: " + std::to_string(alt_data.pressure) << "\tALT: "  + std::to_string(alt) << std::endl;

        auto t_elaps = chrono::duration<double>(chrono::high_resolution_clock::now() - start).count();
        // std::cout << "Total Elasped Time:\t" << t_elaps << "\t" << "seconds" << std::endl;
        // sensor_testing <<"%d, %d,%d,%d,%d,%d,%d,%d,%d,%d,%d", cur, alt_data.temp, alt_data.pressure, imu_data.heading.x, imu_data.heading.y, imu_data.heading.z, imu_data.accel.x, imu_data.accel.y, imu_data.accel.z, t_elaps << std::endl;;
        sensor_testing << chrono::duration<double>(chrono::high_resolution_clock::now() - start).count() << "\t" << alt_data.pressure << "\t" << alt_data.temp << "\t" << imu_data.heading.x << "\t" << imu_data.heading.y << "\t" << imu_data.heading.z << "\t" << imu_data.accel.x << "\t" << imu_data.accel.y << "\t" << imu_data.accel.z << std::endl;
        usleep(10000);
        ii += 1;
    }

    sensor_testing.close();
    return 0;
}