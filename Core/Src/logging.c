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


HAL_StatusTypeDef Log_Imu_Raw_Data(const ICM20948 *imu, UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef retval = HAL_OK;
    static uint32_t time = 0;
    static uint32_t last_tick = 0;

    uint8_t usart_data[13];
    int16_t gyro_x = imu->gyro_data_raw[0];
    int16_t gyro_y = imu->gyro_data_raw[1];
    int16_t gyro_z = imu->gyro_data_raw[2];

    int16_t accel_x = imu->accel_data_raw[0];
    int16_t accel_y = imu->accel_data_raw[1];
    int16_t accel_z = imu->accel_data_raw[2];

    usart_data[0] = gyro_x >> 8; //gyro x h
    usart_data[1] = gyro_x & (0xff); //gyro x l
    usart_data[2] = gyro_y >> 8; //gyro y h
    usart_data[3] = gyro_y & (0xff); //gyro y l
    usart_data[4] = gyro_z >> 8; //gyro z h
    usart_data[5] = gyro_z & (0xff); //gyro z l
    usart_data[6] = accel_x >> 8; //gyro x h
    usart_data[7] = accel_x & (0xff); //gyro x l
    usart_data[8] = accel_y >> 8; //gyro y h
    usart_data[9] = accel_y & (0xff); //gyro y l
    usart_data[10] = accel_z >> 8; //gyro z h
    usart_data[11] = accel_z & (0xff); //gyro z l

    if (last_tick) 
    {
        time = time + (HAL_GetTick() - last_tick);
    }
                                     
    usart_data[12] = time;
    char data[13];
    memcpy(data, (uint8_t *) usart_data, sizeof(usart_data));
    
    HAL_UART_Transmit(huart, (uint8_t *)data, sizeof(data), 1000);
    uint8_t tx[2] = {0x12, 0xF1};
// memcpy(tx, (uint8_t *) tx, sizeof(data));
    //HAL_UART_Transmit(huart, (uint8_t *) tx, sizeof(tx), 5000);
    last_tick = HAL_GetTick();
    return retval;
}

HAL_StatusTypeDef Log_Imu_All(const ICM20948 *imu, const struct orientation *orient, UART_HandleTypeDef *huart)
{

    HAL_StatusTypeDef retval = HAL_OK;
    static uint8_t time = 0;
    static uint32_t last_tick = 0;

    uint8_t usart_data[23];
    int16_t gyro_x = imu->gyro_data_raw[0];
    int16_t gyro_y = imu->gyro_data_raw[1];
    int16_t gyro_z = imu->gyro_data_raw[2];

    int16_t accel_x = imu->accel_data_raw[0];
    int16_t accel_y = imu->accel_data_raw[1];
    int16_t accel_z = imu->accel_data_raw[2];

    int16_t mag_x = imu->mag_data_raw[0];
    int16_t mag_y = imu->mag_data_raw[1];
    int16_t mag_z = imu->mag_data_raw[1];

    int16_t roll = (int16_t) (orient->roll_deg * 100.0);
    int16_t pitch = (int16_t) (orient->pitch_deg * 100.0);

    usart_data[0] = gyro_x >> 8;        // gyro x h
    usart_data[1] = gyro_x & (0xFF);    // gyro x l
    usart_data[2] = gyro_y >> 8;        // gyro y h
    usart_data[3] = gyro_y & (0xFF);    // gyro y l
    usart_data[4] = gyro_z >> 8;        // gyro z h
    usart_data[5] = gyro_z & (0xFF);    // gyro z l
    usart_data[6] = accel_x >> 8;       // accel x h
    usart_data[7] = accel_x & (0xFF);   // accel x l
    usart_data[8] = accel_y >> 8;       // accel y h
    usart_data[9] = accel_y & (0xFF);   // accel y l
    usart_data[10] = accel_z >> 8;      // accel z h
    usart_data[11] = accel_z & (0xFF);  // accel z l
    usart_data[12] = roll >> 8;         // roll h
    usart_data[13] = roll & 0xFF;       // roll l
    usart_data[14] = pitch >> 8;        // pitch h
    usart_data[15] = pitch & 0xFF;      // pitch l
    usart_data[16] = mag_x >> 8;        // mag x l
    usart_data[17] = mag_x & 0xFF;      // mag x h
    usart_data[18] = mag_y >> 8;        // mag y l
    usart_data[19] = mag_y & 0xFF;      // mag y h
    usart_data[20] = mag_z >> 8;        // mag z l
    usart_data[21] = mag_z & 0xFF;      // mag z h
    
    if (last_tick) 
    {
        time = (HAL_GetTick() - last_tick);
    }
                                     
    usart_data[22] = time;
    
    retval = HAL_UART_Transmit(huart, usart_data, sizeof(usart_data), 1000);
    //uint8_t tx[2] = {0x12, 0xF1};
// memcpy(tx, (uint8_t *) tx, sizeof(data));
    //HAL_UART_Transmit(huart, (uint8_t *) tx, sizeof(tx), 5000);
    last_tick = HAL_GetTick();
    return retval;
}


HAL_StatusTypeDef Log_Imu_Raw_Data_ASCII(const ICM20948 *imu, UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef retval = HAL_OK;
    static uint32_t time = 0;
    static uint32_t last_tick = 0;

    char usart_data[64];
    volatile int16_t gyro_x = imu->gyro_data_raw[0];
    volatile int16_t gyro_y = imu->gyro_data_raw[1];
    volatile int16_t gyro_z = imu->gyro_data_raw[2];

    volatile int16_t accel_x = imu->accel_data_raw[0];
    volatile int16_t accel_y = imu->accel_data_raw[1];
    volatile int16_t accel_z = imu->accel_data_raw[2];

    sprintf(usart_data, "%d;%d;%d;%d;%d;%d;%d;%d\n",
            0, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, time);

    if (last_tick)
    {
        time = time + (HAL_GetTick() - last_tick);
    }

    retval = HAL_UART_Transmit(huart, (uint8_t *)usart_data, strlen(usart_data), 1000);
    last_tick = HAL_GetTick();
    return retval;
}


