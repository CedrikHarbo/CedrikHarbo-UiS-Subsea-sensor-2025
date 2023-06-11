/**
 * @file    error_handler.c
 *
 * @author  Håvard Syslak
 * @data    23.03.23
 *
 * @brief   
 */

#include <stdint.h>
#include "can_handler.h"
#include "error_handler.h"
#include "ICM20948.h"
#include "STTS75.h"
#include "MS5837.h"

uint8_t leak_status = 0;
uint8_t IMU_err = 0;
uint8_t temp_err = 0;
uint8_t pres_error = 0;

void IMU_error_handler(ICM20948_StatusTypeDef err)
{
    IMU_err = 0;
    switch (err)
    {
        case ICM20948_HAL_ERROR: 
            IMU_err = (1 << 1);
            break;
        case ICM20948_HAL_BUSY:
            IMU_err = (1 << 2);
            break;
        case ICM20948_HAL_TIMEOUT:
            IMU_err = (1 << 3);
            break;
        case ICM20948_INIT_ERROR:
            IMU_err = (1 << 4);
            break;
        case ICM20948_WHO_AM_I_ERROR:
            IMU_err = (1 << 5);
            break;
        case ICM20948_MAG_ERROR:
                IMU_err = (1 << 6);
            break;
        case ICM20948_MAG_WHO_AM_I_ERROR:
            IMU_err = (1 << 7);
            break;
        default:
            return;
    }
    Send_Error_Code();
}

void Leak_error_handler(uint8_t leak_status)
{
    
}

void Temp_error_handler(STTS75_StatusTypeDef err)
{
    temp_err = 0;
    switch (err)
    {
        case STTS75_HAL_ERROR:
            temp_err = (1 << 1);
            break;
        case STTS75_HAL_BUSY:
            temp_err = (1 << 2);
        case STTS75_HAL_TIMEOUT:
            temp_err = (1 << 3);
            break;
        default:
            return;
    }
    Send_Error_Code();
}

void Pres_error_handler(MS5837_StatusTypeDef err)
{
    pres_error = 0;
    switch (err)
    {
        case MS5837_HAL_ERROR:
            pres_error = 1;
            break;
        case MS5837_HAL_BUSY:
            pres_error = (1 << 1);
        case MS5837_HAL_TIMEOUT:
            pres_error = (1 << 2);
            break;
        default:
            return;
    }
    Send_Error_Code();
}

void Send_Error_Code()
{
    // data = {IMU_err, temp_err, trykk_err, lekkasjestatus, 0, 0, 0, 0}
    uint8_t data[8] = {IMU_err, temp_err, pres_error, leak_status, 0, 0 , 0, 0};
    FDCAN_Send(TOP_ID_ERROR_CODES, &hfdcan1, data);
}

