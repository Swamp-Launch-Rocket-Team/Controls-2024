#pragma once

#include <stdlib.h>
#include <cstdint>
#include <iostream>

#include <chrono>
#include <thread>
#include <cmath>

#include "../spi/spi.h"

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

#define CALIBRATION_C1 25342
#define CALIBRATION_C2 16126
#define CALIBRATION_C3 35070
#define CALIBRATION_C4 52222
#define CALIBRATION_C5 11774
#define CALIBRATION_C6 29182

int alt_init()
{
    int file = spi_open();

    // Reset
    char tx = CMD_RESET;
    int res = spi_write(file, &tx, 1);

    // Chip has 2.8ms reload timing
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    return file;
}

int alt_read_calibration(int file, uint16_t *buff)
{
    char commands[6] = {CMD_PROM_C1,
                        CMD_PROM_C2,
                        CMD_PROM_C3,
                        CMD_PROM_C4,
                        CMD_PROM_C5,
                        CMD_PROM_C6};

    for (uint8_t i = 0; i < 6; i++)
    {
        // Send command
        int res = spi_write(file, commands + i, 1);

        // Read 16 bit result
        res = spi_read(file, (char *)(buff + i), 2);
    }

    return 0;
}

void sample(int file, char conv, uint32_t *sample)
{
    char tx = conv;
    spi_write(file, &tx, 1);

    // Max conversion is 9.04ms for 4096 conversion
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Trigger ADC read
    tx = CMD_ADC_READ;
    spi_write(file, &tx, 1);

    // Read Result
    spi_read(file, (char *)sample, 4);
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

void get_temp_and_pressure(int file)
{
    uint32_t temp_sample = 0;
    sample(file, CMD_CONVERT_TEMPERATURE_4096, &temp_sample);

    uint32_t press_sample = 0;
    sample(file, CMD_CONVERT_PRESSURE_4096, &press_sample);

    // temp_sample = temp_sample & 0x00FFFFFF;

    // temp_sample = 8077636;

    temp_sample = temp_sample >> 8;

    std::cout << "SAMPLES: " << std::to_string(temp_sample) << ", " << std::to_string(press_sample) << std::endl;

    // Calculate temperature
    int32_t dT = temp_sample - (CALIBRATION_C5 * std::pow<int32_t, int32_t>(2, 8));
    dT = clamp<int32_t>(dT, -16776960, 16777216);
    int32_t temp = 2000 + (((dT * (float)CALIBRATION_C6)) / std::pow<int32_t, int32_t>(2, 23));

    std::cout << "dt: " + std::to_string(dT) << std::endl;
    std::cout << "TEMP: " + std::to_string(temp / 100.0) << std::endl;

    // Calculate pressure
    int64_t off = CALIBRATION_C2 * std::pow<int64_t, int64_t>(2, 17) + (CALIBRATION_C4 * dT) / std::pow<int64_t, int64_t>(2, 6);
    off = clamp<int64_t>(off, -17179344900, 25769410560);
    int64_t sens = CALIBRATION_C1 * std::pow<int64_t, int64_t>(2, 16) + (CALIBRATION_C3 * dT) / std::pow<int64_t, int64_t>(2, 7);
    sens = clamp<int64_t>(sens, -8589672450, 12884705280);
    int32_t p = (press_sample * sens / std::pow<int64_t, int64_t>(2, 21) - off) / std::pow<int64_t, int64_t>(2, 15);

    std::cout << "PRESS: " + std::to_string(p / 100.0) << std::endl;
}