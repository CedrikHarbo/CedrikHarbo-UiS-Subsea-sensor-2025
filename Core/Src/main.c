/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "logging.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_it.h"
#include "ICM20948.h"
#include "helpers.h"
#include "orient.h"
#include "can_handler.h"
#include "gpio_handler.h"
#include "cordic.h"
#include "STTS75.h"
#include "MS5837.h"
#include "stm32g4xx_it.h"
#include "error_handler.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

ICM20948 imu;
STTS75 temp_sensor = {0};
MS5837 pres_sensor = {0};
bool TB_status;
extern uint8_t init_flags;

struct orientation orient = {0};
MS5837_StatusTypeDef pres_status;
//extern struct tim_2_flags tim_flags;

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};  // CAN-Bus Transmit Buffer 
uint8_t RxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};                   // CAN-Bus Receive Buffer 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void oppstartCAN(FDCAN_HandleTypeDef *canPort);
//void sendCAN(uint16_t id, FDCAN_HandleTypeDef *canPort);
ICM20948_StatusTypeDef IMU_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	//HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
    FDCAN_Init(&hfdcan1);
    ICM20948_StatusTypeDef imu_status = IMU_Init();
   // IMU_cal(&imu, 500);
    /* Init temp sensor */
    STTS75_Init(&temp_sensor, &hi2c2, STTS75_12BIT);
    /* Init pres sensor */
    vanntype vanntype = FERSKVANN;
    pres_status = MS5837_Init(&pres_sensor, &hi2c2, &vanntype);
    pres_status = MS5837_Read_PROMS(&pres_sensor);
    MS5837_nullstill_trykk(&pres_sensor);
    STTS75_StatusTypeDef temp_status;
    //CORDIC_Init();
    //IMU_cal(&imu, 1000);

    HAL_TIM_Base_Start_IT(&htim2);
    NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    //IMU_com_test();
    static last_leak_status = 0;
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	if (tim_flags.topside)
    	{
            
            tim_flags.topside = 0;
            if (temp_status || pres_status) Temp_error_handler(temp_status);
            FDCAN_Send_Temp_Pres(&temp_sensor, &pres_sensor, TOP_ID_TEMP_PRES);
            //ICM20948_Read_Data_Poll(&imu);
            //compute_orientation(&imu, &orient);
            FDCAN_Send_Orientation_Data(&orient, TOP_ID_ORIENTATION_DATA);

            Check_Leak_Probes();
            //if (leak_status && (last_leak_status != leak_status)) Send_Error_Code();
            if (leak_status) Send_Error_Code();
            last_leak_status = leak_status;
        }

        if (tim_flags.pres_read)
        {
            tim_flags.pres_read = 0;
            pres_status = MS5837_Dybde(&pres_sensor);
            if (pres_status) Pres_error_handler(pres_status);
        }

        if (tim_flags.temp_read)
        {
    	    HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
            tim_flags.temp_read = 0;
            temp_status = STTS75_Read_Temp(&temp_sensor);
            if (temp_status) Temp_error_handler(temp_status);
        }
        if (tim_flags.reg_send) 
        {
            tim_flags.reg_send = 0;
            imu_status = ICM20948_Read_Data_Poll(&imu);
            compute_orientation(&imu, &orient);
            if (imu_status) IMU_error_handler(imu_status);
            else FDCAN_Send_Reg(&orient, &pres_sensor);
        }

        if (init_flags)
        {
            //__disable_irq();
            if (init_flags & 1) MS5837_nullstill_trykk(&pres_sensor);
           // if ((init_flags & (1 << 1)) || (init_flags & (1 << 2))) IMU_cal(&imu, 100);
            if (init_flags & (1 << 3)) vanntype = SALTVANN;
            init_flags = 0;
            //__enable_irq();

        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
ICM20948_StatusTypeDef IMU_Init()
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    delay_ms(100);
    ICM20948_InitTypeDef cfg = {0};
    cfg.clk_src = ICM20948_AUTO_SEL_CLK;

    /* Accel settings */
    cfg.accel_enable = true;
    cfg.accel_scale = ICM20948_2g;
    cfg.accel_lpf_enable = true;
    cfg.accel_break_freq = 196;

    /* Gyro settings */
    cfg.gyro_enable = true;
    cfg.gyro_scale = ICM20948_250dps;
    cfg.gyro_lpf_enable = true;
    cfg.gyro_break_freq = 20;

    /* Mag setteings TODO: */
    cfg.mag_enable = false;

    return ICM20948_Init(&imu, &hspi3, IMU_CS_Pin, IMU_CS_GPIO_Port, &cfg);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     * example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
