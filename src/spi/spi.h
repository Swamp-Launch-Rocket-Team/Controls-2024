#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cerrno>

#define SL_SPI_DEVICE "/dev/spidev0.0"
#define SL_SPI_MODE 0
#define SL_SPI_BITS_PER_WORD 8
#define SL_SPI_SPEED_HZ 10000000 // 10 MHz

int spi_open(void);

int spi_close(int file);

int spi_read(int fd, char *buf, unsigned count);

int spi_write(int fd, char *buf, unsigned count);
