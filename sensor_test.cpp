#include <iostream>

#include "Altimiter/altimiter.h"
#include "IMU/imu.h"

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

    while (true)
    {
        altimiter_t alt_data = get_temp_and_pressure();

        imu_data_t imu_data = imu_read_data();

        print_all(&imu_data);

        float alt = Altitude_calc(alt_data.pressure);
        std::cout << "TEMP: " + std::to_string(alt_data.temp) << "\tPRESS: " + std::to_string(alt_data.pressure) << "\tALT: "  + std::to_string(alt) << std::endl;

        usleep(10000);
    }
}