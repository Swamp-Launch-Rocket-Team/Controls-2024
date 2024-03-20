#pragma once

#include <stdlib.h>
#include <cstdint>
#include <iostream>

#include <chrono>
#include <thread>
#include <cmath>

// #include "spi.h"

#include "bitbang.h"

#define CMD_RESET 0x1E // Reset

#define CMD_CONVERT_PRESSURE_256 0x40  // Pressure, OSR=256
#define CMD_CONVERT_PRESSURE_512 0x42  // Pressure, OSR=512
#define CMD_CONVERT_PRESSURE_1024 0x44 // Pressure, OSR=1024
#define CMD_CONVERT_PRESSURE_2048 0x46 // Pressure, OSR=2048
#define CMD_CONVERT_PRESSURE_4096 0x48 // Pressure, OSR=4096

#define CMD_CONVERT_TEMPERATURE_256 0x50  // Temperature, OSR=256
#define CMD_CONVERT_TEMPERATURE_512 0x52  // Temperature, OSR=512
#define CMD_CONVERT_TEMPERATURE_1024 0x54 // Temperature, OSR=1024
#define CMD_CONVERT_TEMPERATURE_2048 0x56 // Temperature, OSR=2048
#define CMD_CONVERT_TEMPERATURE_4096 0x58 // Temperature, OSR=4096

#define CMD_ADC_READ 0x00 // ADC READ

#define CMD_PROM_RESERVED 0xA0 // PROM Reserved
#define CMD_PROM_C1 0xA2       // Pressure sensitivity | SENS T1
#define CMD_PROM_C2 0xA4       // Pressure offset | OFF T1
#define CMD_PROM_C3 0xA6       // Temperature coefficient of pressure sensitivity | TCS
#define CMD_PROM_C4 0xA8       // Temperature coefficient of pressure offset | TCO
#define CMD_PROM_C5 0xAA       // Reference temperature | T ref
#define CMD_PROM_C6 0xAC       // Temperature coefficient of the temperature | TEMPSENS
#define CMD_PROM_CRC 0xAE      // CRC value

#define CALIBRATION_C1 38084
#define CALIBRATION_C2 35453
#define CALIBRATION_C3 23313
#define CALIBRATION_C4 21911
#define CALIBRATION_C5 33371
#define CALIBRATION_C6 27363

// #define ALT_SPI_DEVICE "/dev/spidev0.1"
// #define ALT_SPI_MODE SPI_MODE_0

void alt_init()
{   
    // Reset
    char tx = CMD_RESET;
    char rx = 0;
    // spi_write(ALT_SPI_DEVICE, ALT_SPI_MODE, &tx, 1);

    spi_transfer_bitbang(&tx, &rx, 1, 0, 1);
    
    // Chip has 2.8ms reload timing
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
}

void alt_read_calibration(uint16_t *calibration)
{
    char commands[18] = {CMD_PROM_C1, 0x00, 0x00,
                        CMD_PROM_C2, 0x00, 0x00,
                        CMD_PROM_C3, 0x00, 0x00,
                        CMD_PROM_C4, 0x00, 0x00,
                        CMD_PROM_C5, 0x00, 0x00,
                        CMD_PROM_C6, 0x00, 0x00};

    char buff[18] = {0};

    for (uint8_t i = 0; i < 6; i++)
    {
        // spi_transact(ALT_SPI_DEVICE, ALT_SPI_MODE, commands + (3*i), buff + (3*i), 3);
        spi_transfer_bitbang(commands + (3*i), buff + (3*i), 3, 0, 1);
    }

    calibration[0] = buff[1] << 8 | buff[2];
    calibration[1] = buff[4] << 8 | buff[5];
    calibration[2] = buff[7] << 8 | buff[8];
    calibration[3] = buff[10] << 8 | buff[11];
    calibration[4] = buff[13] << 8 | buff[14];
    calibration[5] = buff[16] << 8 | buff[17];
}

uint32_t sample(char conv)
{    
    char tx = conv;
    char rx = 0;
    // spi_write(ALT_SPI_DEVICE, ALT_SPI_MODE, &tx, 1);
    spi_transfer_bitbang(&tx, &rx, 1, 0, 1);

    // Max conversion is 9.04ms for 4096 conversion
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    char command[4] = {CMD_ADC_READ, 0x00, 0x00, 0x00};
    char buff[4] = {0};

    // spi_transact(ALT_SPI_DEVICE, ALT_SPI_MODE, command, buff, 4);
    spi_transfer_bitbang(command, buff, 4, 0, 1);


    return buff[1] << 16 | buff[2] << 8 | buff[3];
}

template <class T>
T clamp(T val, T min, T max)
{
    if (val > max)
    {
        return max;
    }

    if (val < min)
    {
        return min;
    }

    return val;
}

#define R 287.058f
#define g 9.81f
float Altitude_calc(float pressure)
{
    pressure = pressure*100;        //mbar to Pa
    float altitude = (288.15/0.0065)*(1-(pow(pressure/101325,0.0065*(R/g))));
    return altitude;
}

void get_temp_and_pressure()
{
    uint32_t temp_sample = sample(CMD_CONVERT_TEMPERATURE_4096);
    uint32_t press_sample = sample(CMD_CONVERT_PRESSURE_4096);
    // std::cout << "SAMPLES: " << std::to_string(temp_sample) << ", " << std::to_string(press_sample) << std::endl;

    // Calculate temperature
    int32_t dT = temp_sample - (CALIBRATION_C5 * std::pow<int32_t, int32_t>(2, 8));
    dT = clamp<int32_t>(dT, -16776960, 16777216);
    int32_t temp = 2000 + (((dT * (float)CALIBRATION_C6)) / std::pow<int32_t, int32_t>(2, 23));

    std::cout << "TEMP: " + std::to_string(temp / 100.0) << std::endl;

    // Calculate pressure
    int64_t off = CALIBRATION_C2 * std::pow<int64_t, int64_t>(2, 17) + (CALIBRATION_C4 * dT) / std::pow<int64_t, int64_t>(2, 6);
    off = clamp<int64_t>(off, -17179344900, 25769410560);
    int64_t sens = CALIBRATION_C1 * std::pow<int64_t, int64_t>(2, 16) + (CALIBRATION_C3 * dT) / std::pow<int64_t, int64_t>(2, 7);
    sens = clamp<int64_t>(sens, -8589672450, 12884705280);
    int32_t p = (press_sample * sens / std::pow<int64_t, int64_t>(2, 21) - off) / std::pow<int64_t, int64_t>(2, 15);

    std::cout << "PRESS: " + std::to_string(p / 100.0f) << std::endl;

    float alt = Altitude_calc(p / 100.0f);

    std::cout << "ALT: "  + std::to_string(alt) << std::endl;
}
