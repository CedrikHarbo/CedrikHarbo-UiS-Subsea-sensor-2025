/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "orient.h"
#include "ICM20948.h"
#include <stdint.h>
#include "gpio_handler.h"
#include "can_handler.h"
#include "error_handler.h"
#include "logging.h"

#include "STTS75.h"
#include "MS5837.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern ICM20948 imu;
extern struct orientation orient;
extern UART_HandleTypeDef huart2;
extern uint8_t leak_status;
extern STTS75 temp_sensor;
extern MS5837 pres_sensor;
struct tim_2_flags tim_flags = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    // Rread TB1
	uint32_t mau = HAL_GetTick();
    static uint8_t last_leak_status = 0;
    static uint32_t leak_cntr = 0;
    static uint32_t TB_ctnr = 0;
    static uint32_t imu_cntr = 0;
    static uint32_t temp_cntr = 0;
    static uint32_t pres_cntr = 0;
    static uint32_t topside_cntr = 0;
    static uint32_t reg_cntr = 0;
    static uint32_t imu_raw_cntr = 0;

    if (topside_cntr >= 10)
    {
        topside_cntr = 0;
        tim_flags.topside = 1;
    }
    if (imu_cntr >= 2)
    {
        imu_cntr = 0;
        tim_flags.imu_read = 1;
    }
    if (temp_cntr >= 100)
    {
        temp_cntr = 0;
        tim_flags.temp_read = 1;
    }
    if (pres_cntr >= 2)
    {
        pres_cntr = 0;
        tim_flags.pres_read = 1;
    }

    if (reg_cntr >= 5)
    {
        reg_cntr = 0;
        tim_flags.reg_send = 1;
    }
    if (imu_raw_cntr >= 5)
    {
        ICM20948_Read_Gyro(&imu);
        ICM20948_Read_Accel(&imu);
        Log_Imu_Raw_Data_ASCII(&imu, &huart2);
        imu_raw_cntr = 0;
    }
#ifdef ASDASD
    if (topside_cntr >= 200)
    {
        topside_cntr = 0;
        /* Send orietation data */
        if (leak_status && (last_leak_status != leak_status)) Send_Error_Code();
        FDCAN_Send_Orientation_Data(&orient);
        /* Send temp_data */
        STTS75_StatusTypeDef temp_status = STTS75_Read_Temp(&temp_sensor);
        if (temp_status) Temp_error_handler(temp_status);
        else FDCAN_Send_Temp_Pres(&temp_sensor, &pres_sensor);
    }

    if (leak_cntr >= 10)
    {
        leak_cntr = 0;
        Check_Leak_Probes();
        if (leak_status && (last_leak_status != leak_status)) Send_Error_Code();
        last_leak_status = leak_status;
    }

    if (TB_ctnr >= 10)
    {
        TB_ctnr = 0;
        Read_TB1();
    }
    
    if (orient_cntr >= 2)
    {
        ICM20948_Read_Accel(&imu);
        ICM20948_Read_Gyro(&imu);
        compute_orientation(&imu, &orient);
        i++;
        if (i >= 50)
        {
        	FDCAN_Send_Orientation_Data(&orient);
        	i = 0;
        }
     	orient_cntr = 0;
    }


    if (pres_cntr >= 2)
    {
    	pres_cntr = 0;
        MS5837_StatusTypeDef pres_status = MS5837_Dybde(&pres_sensor);
        if (pres_status) Pres_error_handler(pres_status);
        else FDCAN_Send_Temp_Pres(&temp_sensor, &pres_sensor);
    }

    if (imu_log_cntr >= 20)
    {
    	ICM20948_Read_Data_Poll(&imu);
    	Log_Imu_Raw_Data_ASCII(&imu, &huart2);
    	imu_log_cntr = 0;
    }
#endif

    leak_cntr++;
    TB_ctnr++;
    //orient_cntr++; //orient_cntr = 0;
    temp_cntr++;
    pres_cntr++;
    topside_cntr++;
    reg_cntr++;
    imu_raw_cntr++;
#ifdef mau
    temp_cntr = 0;
    pres_cntr = 0;
    topside_cntr = 0;
    reg_cntr = 0;
#else
    imu_raw_cntr = 0;
#endif

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */


  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
