#include <iostream>

#include "spi.h"
#include "altimiter.h"
#include <wiringPi.h>

int main(void)
{    
    alt_init();

    uint16_t cal[6] = {0};
    alt_read_calibration(cal);

    for (int i = 0; i < 6; i++)
    {
        std::cout << cal[i] << std::endl;
    }

    get_temp_and_pressure();

    return 0;
}