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
#include "stm32f1xx_hal.h"

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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAX_LENGTH 128

struct dvl_wrz{
	char vx[MAX_LENGTH];
	char vy[MAX_LENGTH];
	char vz[MAX_LENGTH];
	char valid[MAX_LENGTH];
	char altitude[MAX_LENGTH];
	char fom[MAX_LENGTH];
	char covariance[MAX_LENGTH];
	char time_of_validity[MAX_LENGTH];
	char time_of_transmission[MAX_LENGTH];
	char time[MAX_LENGTH];
	char status[MAX_LENGTH];
};

struct dvl_wrx{
	char time[MAX_LENGTH];
	char vx[MAX_LENGTH];
	char vy[MAX_LENGTH];
	char vz[MAX_LENGTH];
	char fom[MAX_LENGTH];
	char altitude[MAX_LENGTH];
	char valid[MAX_LENGTH];
	char status[MAX_LENGTH];
};

struct dvl_wrp{
	char timestamp[MAX_LENGTH];
	char px[MAX_LENGTH];
	char py[MAX_LENGTH];
	char pz[MAX_LENGTH];
	char pos_std[MAX_LENGTH];
	char roll[MAX_LENGTH];
	char pitch[MAX_LENGTH];
	char yaw[MAX_LENGTH];
	char status[MAX_LENGTH];
};

extern UART_HandleTypeDef huart2;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
