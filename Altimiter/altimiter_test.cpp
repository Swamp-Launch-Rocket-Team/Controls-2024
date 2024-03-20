#include <iostream>

#include "altimiter.h"

#include "bitbang.h"

#define R 287.058f
#define g 9.81f
float Altitude_calc(float pressure)
{
    pressure = pressure*100;        //mbar to Pa
    float altitude = (288.15/0.0065)*(1-(pow(pressure/101325,0.0065*(R/g))));
    return altitude;
}

int main(void)
{    
    spi_init_bitbang();
    
    alt_init();

    uint16_t cal[6] = {0};
    alt_read_calibration(cal);

    for (int i = 0; i < 6; i++)
    {
        std::cout << cal[i] << std::endl;
    }

    altimiter_t alt_data = get_temp_and_pressure();

    float alt = Altitude_calc(alt_data.pressure);
    std::cout << "TEMP: " + std::to_string(alt_data.temp) << "\tPRESS: " + std::to_string(alt_data.pressure) << "\tALT: "  + std::to_string(alt) << std::endl;

    return 0;
}