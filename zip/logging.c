/**
* @file: logging.c 
*
* @author: Håvard Syslak
* @date: 17.02.2023
*
* @brief 
*/


#include <string.h>

#include "logging.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_uart.h"



/**
 * @brief Send rå IMU data over Usart
 * 
 * @retval HAL_Status
 */



HAL_StatusTypeDef Log_temp_All(int16_t temp, UART_HandleTypeDef *huart,uint8_t error)
{

    HAL_StatusTypeDef retval = HAL_OK;

    static uint8_t time = 0;
    static uint16_t last_tick = 0;
    uint8_t usart_data[4]; // tredje verdi er tid!
    //int16_t temp_8pack = sensor->temp_raw;


    //temp er 16 bits verdi
    usart_data[0] = temp >> 8;        // temp
    usart_data[1] = temp & (0xFF);    // temp
    if (last_tick)
    {
        time = (HAL_GetTick() - last_tick);
    }


    usart_data[2] = time;
    usart_data[3] = error;


    printf(usart_data);
    retval = HAL_UART_Transmit(huart, usart_data, sizeof(usart_data), 200);
    last_tick = HAL_GetTick();
    return retval;
}

HAL_StatusTypeDef Log_pressure(MS5837 *sensor,UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef retval;
	uint16_t cm_dybde = sensor->cm_dybde;
	uint16_t cm_dybde_iir = sensor->cm_dybde_iir;
	uint32_t pmbar =sensor->P_mbar;
	uint8_t error =sensor->error;
	uint16_t vanntemp = sensor->vanntemp;



    static uint8_t time = 0;
    static uint16_t last_tick = 0;

    uint8_t usart_data[12];

    //int16_t temp_8pack = sensor->temp_raw;


    //trykk er 16 bits verdi


    //send cm uten iir
    usart_data[0] = cm_dybde >> 8;
    usart_data[1] = cm_dybde & (0xFF);



    // temp
    if (last_tick)
    {
        time = (HAL_GetTick() - last_tick);
    }


    usart_data[2] = time;
    usart_data[3] = error;

    //send iir cm,8:12
    usart_data[4] = cm_dybde_iir >> 8;
    usart_data[5] = cm_dybde_iir & (0xFF);

    //send pm bar raw data, 12:20
    usart_data[6] = pmbar >> 24;
    usart_data[7] = pmbar >>16;
    usart_data[8] = pmbar >>8;
    usart_data[9] = pmbar;


    // send temperatur ,20:24
    usart_data[10] = vanntemp >>8;
    usart_data[11] =vanntemp;


    retval = HAL_UART_Transmit(huart, usart_data, sizeof(usart_data), 400);
    last_tick = HAL_GetTick();

    return retval;
}
