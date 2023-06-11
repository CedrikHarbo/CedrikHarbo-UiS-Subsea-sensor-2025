/*
*    Project: logging
*         __ 
*     |_|(_
*     | |__)
*    Author: Håvard Syslak
*    Date: 17.02.2023
*/

#ifndef LOGGING_H
#define LOGGING_H

//#include "ICM20948.h"
#include "STTS75.h"
#include "MS5837.h"
//#include "orient.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_uart.h"

//HAL_StatusTypeDef Log_Imu_Raw_Data(const ICM20948 *imu, UART_HandleTypeDef *huart);

HAL_StatusTypeDef Log_temp_All(int16_t temp, UART_HandleTypeDef *huart,uint8_t error);
HAL_StatusTypeDef Log_pressure(MS5837 *sensor,UART_HandleTypeDef *huart);

#endif /* LOGGING_H */
