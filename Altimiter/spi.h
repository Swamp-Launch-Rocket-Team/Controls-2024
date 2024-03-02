#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cerrno>

// #define SL_SPI_DEVICE "/dev/spidev0.1"
#define SL_SPI_MODE 0
#define SL_SPI_BITS_PER_WORD 8
#define SL_SPI_SPEED_HZ 10000000 // 10 MHz

int spi_open(std::string device)
{
    int spiFile;

    // Open the SPI file
    if ((spiFile = open(device.c_str(), O_RDWR)) < 0)
    {
        return -1;
    }

    // Configure SPI mode
    char spiMode = SL_SPI_MODE;
    if (ioctl(spiFile, SPI_IOC_WR_MODE, &spiMode) < 0)
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

int spi_write(int fd, char *buf, unsigned count)
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

    err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    if (err < 0)
    {
        perror("SPI_IOC_MESSAGE failed");
        printf("Error: %s\n", strerror(errno));
    }

    return err;
}

int spi_transact(int fd, char *tx, char *rx, unsigned count)
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

    err = ioctl(fd, SPI_IOC_MESSAGE(1), &spi);

    if (err < 0)
    {
        perror("SPI_IOC_MESSAGE failed");
        printf("Error: %s\n", strerror(errno));
    }

    return err;
}