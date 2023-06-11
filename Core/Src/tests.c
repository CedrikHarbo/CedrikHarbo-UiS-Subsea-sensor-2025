/**
 * @file	tests.c
 *
 *  @date 22.03.23
 *  @author: Håvard Syslak
 */

#include "tests.h"
#include <stdint.h>
#include "ICM20948.h"
#include "gpio_handler.h"
#include <math.h>
#include "cordic.h"

extern ICM20948 imu;

void IMU_com_test()
{
    	uint8_t tx_buf[2];
    	uint8_t rx_buf[2];
       // while (!TB_status) {}
        ICM20948_Read_Register(&imu, ICM20948_WHO_AM_I, tx_buf, rx_buf);

        //while (TB_status) {}
        //while (!TB_status) {}
        ICM20948_Read_Gyro(&imu);
    	HAL_GPIO_TogglePin(GPIOB, TB1_PIN);
        //while (!TB_status) {}
        ICM20948_Read_Accel(&imu);
    	//HAL_GPIO_TogglePin(GPIOB, TB1_PIN);
        //while (!TB_status) {}
        ICM20948_Read_Mag(&imu);
    	HAL_GPIO_TogglePin(GPIOB, TB1_PIN);
}

void cordic_test_sqrt(uint32_t *cordic_cycles, uint32_t *cycles_m, float num, float *cordic_res, float *res)
{
    __disable_irq();
    DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);
    DWT->CYCCNT = 0;
    *cordic_res = CORDIC_Sqrt(num);
    *cordic_cycles = DWT->CYCCNT;
    DWT->CYCCNT = 0;
    *res = sqrt(num);
    *cycles_m = DWT->CYCCNT;
    __enable_irq();
}

void cordic_test_atan2f(uint32_t *cordic_cycles, uint32_t *cycles_m, float x, float y, float *cordic_res, float *res)
{
    __disable_irq();
    DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);
    DWT->CYCCNT = 0;
    *cordic_res = CORDIC_Atan2f(x, y);
    *cordic_cycles = DWT->CYCCNT;
    DWT->CYCCNT = 0;
    *res = atan2f(x, y);
    *cycles_m = DWT->CYCCNT;
    __enable_irq();

}
