/**
*    @file gpio.c
*
*    @author Håvard Syslak
*    @date 18.03.2023
*/

#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdbool.h>
#include <stdint.h>


#define LEKK1_Pin GPIO_PIN_0
#define LEKK1_GPIO_Port GPIOA
#define LEKK2_Pin GPIO_PIN_1
#define LEKK2_GPIO_Port GPIOA
#define LEKK3_Pin GPIO_PIN_4
#define LEKK3_GPIO_Port GPIOA
#define LEKK4_Pin GPIO_PIN_7
#define LEKK4_GPIO_Port GPIOA
#define TB1_PIN GPIO_PIN_0
#define TB1_PORT GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB

void Read_TB1();
void Set_LD2(bool state);
void Check_Leak_Probes();


#endif /* GPIO_HANDLER_H */
