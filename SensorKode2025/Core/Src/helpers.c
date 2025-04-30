/*
 * helpers.c
 *
 *  Created on: Apr 2, 2025
 */

#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdint.h>

#define NS_PER_S 1000000000

void delay_ms(volatile uint32_t delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t elapsed_time = 0;

    while(elapsed_time < delay)
    {
        elapsed_time = HAL_GetTick() - tickstart;
    }
}

/**
 * @brief Vent ns
 *
 * @note Ikke ment for nøyaktig timing
 *
 */

void delay_ns(volatile uint32_t time_ns)
{
    #ifndef MCU_FREQ_HZ
    #define MCU_FREQ_HZ 170000000UL
#endif

    uint32_t wait_cycles = ((time_ns * MCU_FREQ_HZ) + (NS_PER_S)) / NS_PER_S;

    for (uint32_t i = 0; i<= wait_cycles; i++)
    {
        asm volatile("nop");
    }
}
