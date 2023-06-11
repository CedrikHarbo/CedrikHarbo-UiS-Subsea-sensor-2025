/**
 * @file attutide.c
 *
 * @brief Estimerings metoder for vinkler
 *
 */

#include "ICM20948.h"
#include "orient.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include <math.h>
#include <stdint.h>
extern TIM_HandleTypeDef htim2;

void compute_orientation(ICM20948 *imu, struct orientation *orient)
{
    volatile uint32_t Ts = 0;

    float Ax = imu->accel_data[2];
    float Ay = imu->accel_data[1];
    float Az = -imu->accel_data[0];
    orient->accel_roll = atan2f(Ay, Az) * (180 / M_PI);
    orient->accel_pitch = atan2f(-Ax, sqrt(pow(Ay, 2) + pow(Az, 2))) * (180 / M_PI);
    float alpha = 0.9;
    
    static uint32_t last_tick = 0; 
    if (last_tick) 
    {
        Ts = HAL_GetTick() - last_tick;
        last_tick = HAL_GetTick();
    }
    else 
        {
    		last_tick = HAL_GetTick();
            orient->roll_deg = orient->accel_roll;
            orient->pitch_deg = orient->accel_pitch;
            return;
        }


    orient->gyro_roll  = imu->gyro_data[0] * ((float) Ts / 1000);
    orient->gyro_pitch = imu->gyro_data[1] * ((float) Ts / 1000);
    comp_filter(orient, alpha);

}

void comp_filter(struct orientation *orient, float alpha)
{
    orient->roll_deg = ((orient->roll_deg + orient->gyro_roll) * alpha) + (orient->accel_roll * (1-alpha));
    orient->pitch_deg = ((orient->pitch_deg + orient->gyro_pitch) * alpha) + (orient->accel_pitch * (1-alpha));
}


void IMU_cal(ICM20948 *imu, uint32_t n_samples)
{
    double accel_avg[3] = {0};
    double gyro_avg[3] = {0};
    
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    for(uint32_t i = 0; i <= n_samples; i++)
    {
        for (;;)
        {
            ICM20948_Read_Register(imu, ICM20948_DATA_RDY_STATUS, tx_buf, rx_buf);
            if (rx_buf[1] & 0x03U) 
            {
                ICM20948_Read_Accel(imu);
                ICM20948_Read_Gyro(imu);
                break;
            }

        }

        accel_avg[0] += imu->accel_data_raw[0] / imu->accel_scale_factor;
        accel_avg[1] += imu->accel_data_raw[1] / imu->accel_scale_factor;
        accel_avg[2] += imu->accel_data_raw[2] / imu->accel_scale_factor;

        gyro_avg[0] += imu->gyro_data_raw[0] / imu->accel_scale_factor;
        gyro_avg[1] += imu->gyro_data_raw[1] / imu->accel_scale_factor;
        gyro_avg[2] += imu->gyro_data_raw[2] / imu->accel_scale_factor;
    }

    //imu->accel_offs[0] = (int16_t) ((accel_avg[0] / n_samples) * imu->accel_scale_factor);
    //imu->accel_offs[1] = (int16_t) ((accel_avg[1] / n_samples) * imu->accel_scale_factor);
    //imu->accel_offs[2] = (int16_t) ((accel_avg[2] / n_samples) * imu->accel_scale_factor) - imu->accel_scale_factor;

    imu->gyro_offs[0] = (int16_t) ((gyro_avg[0] / n_samples) * imu->gyro_scale_factor);
    imu->gyro_offs[1] = (int16_t) ((gyro_avg[1] / n_samples) * imu->gyro_scale_factor);
    imu->gyro_offs[2] = (int16_t) ((gyro_avg[2] / n_samples) * imu->gyro_scale_factor);
     
}
