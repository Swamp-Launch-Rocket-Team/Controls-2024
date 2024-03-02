#include <iostream>

#include "spi.h"
#include "altimiter.h"

int main(void)
{
    int alt = alt_init();

    uint16_t cal[6] = {0};
    alt_read_calibration(alt, cal);

    for (int i = 0; i < 6; i++)
    {
        std::cout << cal[i] << std::endl;
    }

    get_temp_and_pressure(alt);

    spi_close(alt);

    return 0;
}