#include "busynano.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")

void busy10ns(uint32_t periods)
{
    for(; periods > 0; periods--)
    {
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
        __asm volatile("nop\n");
    }
}

#pragma GCC pop_options
