#pragma once

#include <wiringPi.h>

#include "../busynano/busynano.h"

#define MOSI_PIN 12 // GPIO pin for MOSI (Master Out Slave In)
#define MISO_PIN 13 // GPIO pin for MISO (Master In Slave Out)
#define SCLK_PIN 14 // GPIO pin for SCLK (Serial Clock)
#define CS0_PIN  10 // GPIO pin for CS0 (Chip Select)
#define CS1_PIN  11 // GPIP pin for CS1 (Chip Select)

int spi_init_bitbang();
void spi_transfer_bitbang(char *tx, char *rx, char length, char mode, char cs);