#include "bitbang.h"

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

    return 0;
}

void spi_transfer_bitbang(char *tx, char *rx, char length, char mode, char cs) {
    char chipSel = cs == 0 ? CS0_PIN : CS1_PIN;

     // Mode 3 wants a high idle clock
    if (mode == 3) {
        digitalWrite(SCLK_PIN, HIGH);
        busy10ns(20);
    }
    
    // Set the CS pin low to select the slave device
    digitalWrite(chipSel, LOW);
    busy10ns(20);

    if (mode == 3) {
        // while (digitalRead(MISO_PIN) == LOW) {
        //     std::cout << "WE ARE SO LOW RN FR" << std::endl;
        //     busy10ns(2);
        // }
    }

    // Shift out the data byte
    for (int byte = 0; byte < length; byte++) {
        for (int i = 0; i < 8; i++) {
            rx[byte] <<= 1;
            digitalWrite(SCLK_PIN, LOW);
            busy10ns(20);
            digitalWrite(MOSI_PIN, (tx[byte] & 0x80) ? HIGH : LOW); // Send the MSB first
            busy10ns(20);
            digitalWrite(SCLK_PIN, HIGH);
            busy10ns(20);
            
            rx[byte] |= digitalRead(MISO_PIN) == HIGH ? 0x01 : 0x00; // Read MISO
            tx[byte] <<= 1; // Shift the data byte
            busy10ns(500);

        }
    }

    // Always set clock to low since hardware bug
    digitalWrite(SCLK_PIN, LOW);

    // Set the CS pin high to deselect the slave device
    digitalWrite(chipSel, HIGH);
}
