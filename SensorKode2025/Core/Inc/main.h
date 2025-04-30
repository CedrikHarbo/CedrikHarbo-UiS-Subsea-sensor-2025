/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define LEKK1_Pin GPIO_PIN_0
#define LEKK1_GPIO_Port GPIOA
#define LEKK2_Pin GPIO_PIN_1
#define LEKK2_GPIO_Port GPIOA
#define LEKK3_Pin GPIO_PIN_4
#define LEKK3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LEKK4_Pin GPIO_PIN_7
#define LEKK4_GPIO_Port GPIOA
#define B1B0_Pin GPIO_PIN_0
#define B1B0_GPIO_Port GPIOB
#define DVL_TX_Pin GPIO_PIN_10
#define DVL_TX_GPIO_Port GPIOB
#define DVL_RX_Pin GPIO_PIN_11
#define DVL_RX_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define IMU_SPI_SCK_Pin GPIO_PIN_3
#define IMU_SPI_SCK_GPIO_Port GPIOB
#define IMU_SPI_MISO_Pin GPIO_PIN_4
#define IMU_SPI_MISO_GPIO_Port GPIOB
#define IMU_SPI_MOSI_Pin GPIO_PIN_5
#define IMU_SPI_MOSI_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define STATUS_LED_1_Pin GPIO_PIN_7
#define STATUS_LED_1_GPIO_Port GPIOB
#define TEST_BRYTER1_Pin GPIO_PIN_0
#define TEST_BRYTER1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
