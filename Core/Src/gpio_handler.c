/**
*    @file gpio.c
*
*    @author Håvard Syslak
*    @date 18.03.2023
*/

#include <stdbool.h>
#include <stdint.h>

#include "ICM20948.h"
#include "gpio_handler.h"
#include "can_handler.h"

extern bool TB_status;
extern uint8_t leak_status;


/**
 * @brief   Les status på testbryteren. Denne funksjonen må kalles 3 ganger for, og den må ha 3 høye avlesninger pårad
 *          for å sette det globale flagget TB_status
 */
void Read_TB1()
{
    static uint8_t debounce_counter = 0;
    if (((TB1_PORT->IDR & TB1_PIN) == 0x00U))
    {
        debounce_counter++;
    }
    else
    {
    	debounce_counter = 0;
    }
    if (debounce_counter >= 3)
    {
    	TB_status = true;
    	debounce_counter = 3;
    }
    else
    {
    	TB_status = false;
    }
} 

void Set_LD2(bool state)
{
    if (state)
    {
        GPIOB->BSRR |= LED1_Pin;
    }
    else
    {
        GPIOB->BRR |= LED1_Pin;
    }
}

void Check_Leak_Probes()
{
    uint8_t probe_1, probe_2, probe_3, probe_4;
    probe_1 = GPIOA->IDR & LEKK1_Pin ? 1 : 0;
    probe_2 = GPIOA->IDR & LEKK2_Pin ? 1 : 0;
    probe_3 = GPIOA->IDR & LEKK3_Pin ? 1 : 0;
    probe_4 = GPIOA->IDR & LEKK4_Pin ? 1 : 0;
    
    leak_status = (probe_4 << 3) | (probe_3 << 2) | (probe_2 << 1) | (probe_1);
}

