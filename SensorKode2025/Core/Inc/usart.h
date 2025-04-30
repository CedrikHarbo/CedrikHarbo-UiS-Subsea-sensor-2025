/*
 * usart.h
 *
 *  Created on: Apr 2, 2025
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_USART_H_ */
