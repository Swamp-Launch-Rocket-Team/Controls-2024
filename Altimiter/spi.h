#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cerrno>
#include <wiringPi.h>

// #define SL_SPI_DEVICE "/dev/spidev0.1"
// #define SL_SPI_MODE 0
#define SL_SPI_BITS_PER_WORD 8
#define SL_SPI_SPEED_HZ 1000000 // 10 MHz

int spi_open(std::string device, char mode)
{
    int spiFile;

    // Open the SPI file
    if ((spiFile = open(device.c_str(), O_RDWR)) < 0)
    {
        return -1;
    }

    // Configure SPI mode
    if (ioctl(spiFile, SPI_IOC_WR_MODE, &mode) < 0)
    {
        close(spiFile);
        return -2;
    }

    // Configure number of bits per word
    char spiBits = SL_SPI_BITS_PER_WORD;
    if (ioctl(spiFile, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0)
    {
        close(spiFile);
        return -3;
    }

    // Configure BAUD
    unsigned spiBaud = SL_SPI_SPEED_HZ;
    if (ioctl(spiFile, SPI_IOC_WR_MAX_SPEED_HZ, &spiBaud) < 0)
    {
        close(spiFile);
        return -4;
    }

    return spiFile;
}

int spi_close(int file)
{
    return close(file);
}

/*
* DO NOT MAKE A SPI READ FUNCTION such as
* int spi_read(int fd, char *buf, unsigned count)
* the linux SPI driver toggles a chip select if you
* split it up and do a spi_write then spi_read
* just do the call in the spi_transact function instead.
*/

int spi_write(const char *dev, char mode, char *buf, unsigned count)
{
    int err;
    struct spi_ioc_transfer spi;

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)buf;
    spi.rx_buf = (unsigned long)NULL;
    spi.len = count;
    spi.speed_hz = SL_SPI_SPEED_HZ;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    // digitalWrite(14, mode == SPI_MODE_0 ? LOW : HIGH);

    int file = spi_open(dev, mode);

    err = ioctl(file, SPI_IOC_MESSAGE(1), &spi);

    spi_close(file);

    // pinMode(14, OUTPUT);
    // pinMode(10, OUTPUT);
    // pinMode(11, OUTPUT);

    // digitalWrite(14, LOW);
    // digitalWrite(10, HIGH);
    // digitalWrite(11, HIGH);
    

    if (err < 0)
    {
        perror("SPI_IOC_MESSAGE failed");
        printf("Error: %s\n", strerror(errno));
    }

    return err;
}

int spi_transact(const char *dev, char mode, char *tx, char *rx, unsigned count)
{    
    int err;
    struct spi_ioc_transfer spi;

    memset(&spi, 0, sizeof(spi));

    spi.tx_buf = (unsigned long)tx;
    spi.rx_buf = (unsigned long)rx;
    spi.len = count;
    spi.speed_hz = SL_SPI_SPEED_HZ;
    spi.delay_usecs = 0;
    spi.bits_per_word = 8;
    spi.cs_change = 0;

    // digitalWrite(14, mode == SPI_MODE_0 ? LOW : HIGH);

    int file = spi_open(dev, mode);

    err = ioctl(file, SPI_IOC_MESSAGE(1), &spi);

    spi_close(file);

    // wiringPiSetup();

    // pinMode(14, OUTPUT);
    // pinMode(10, OUTPUT);
    // pinMode(11, OUTPUT);

    // digitalWrite(14, LOW);
    // digitalWrite(10, HIGH);
    // digitalWrite(11, HIGH);

    if (err < 0)
    {
        perror("SPI_IOC_MESSAGE failed");
        printf("Error: %s\n", strerror(errno));
    }

    return err;
}