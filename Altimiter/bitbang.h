#pragma once

#include <wiringPi.h>

#define MOSI_PIN 23 // GPIO pin for MOSI (Master Out Slave In)
#define MISO_PIN 24 // GPIO pin for MISO (Master In Slave Out)
#define SCLK_PIN 25 // GPIO pin for SCLK (Serial Clock)
#define CS0_PIN  26 // GPIO pin for CS0 (Chip Select)
#define CS1_PIN  27 // GPIP pin for CS1 (Chip Select)

int spi_init_bitbang() {
    // Initialize the WiringPi library
    wiringPiSetup();

    // Set the GPIO pin modes
    pinMode(MOSI_PIN, OUTPUT);
    pinMode(MISO_PIN, INPUT);
    pinMode(SCLK_PIN, OUTPUT);
    pinMode(CS0_PIN, OUTPUT);
    pinMode(CS1_PIN, OUTPUT);

    // Set the initial state of the CS and SCLK pins
    digitalWrite(CS0_PIN, HIGH);
    digitalWrite(CS1_PIN, HIGH);
    digitalWrite(SCLK_PIN, LOW);
}

void spi_transfer_bitbang(char *tx, char *rx, char length, char mode, char cs) {
    char chipSel = cs == 0 ? CS0_PIN : CS1_PIN;
    
    // Set the CS pin low to select the slave device
    digitalWrite(chipSel, LOW);

    // Mode 3 wants a high idle clock
    if (mode == 3) {
        digitalWrite(SCLK_PIN, HIGH);
    }

    // Shift out the data byte
    for (int byte = 0; byte < length; byte++) {
        for (int i = 0; i < 8; i++) {
            rx[byte] <<= 1;
            digitalWrite(SCLK_PIN, LOW);
            digitalWrite(MOSI_PIN, (tx[byte] & 0x80) ? HIGH : LOW); // Send the MSB first
            rx[byte] |= digitalRead(MISO_PIN) == HIGH ? 0x01 : 0x00; // Read MISO
            digitalWrite(SCLK_PIN, HIGH);
            tx[byte] <<= 1; // Shift the data byte
        }
    }

    // Always set clock to low since hardware bug
    digitalWrite(SCLK_PIN, LOW);

    // Set the CS pin high to deselect the slave device
    digitalWrite(chipSel, HIGH);
}

