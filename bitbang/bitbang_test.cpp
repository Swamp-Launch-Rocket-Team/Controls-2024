#include "bitbang.h"
#include <stdio.h>

#include <iostream>

int main()
{
    spi_init_bitbang();

    std::cout << "Here" << std::endl;

    char tx = 0x38;
    char rx = 0x00;

    spi_transfer_bitbang(&tx, &rx, 1, 3, 1);

    std::cout << "Here2" << std::endl;
    std::cout << std::hex << int(rx) << std::endl;
}