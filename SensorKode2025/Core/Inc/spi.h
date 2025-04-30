/*
 * spi.h
 *
 *  Created on: Apr 2, 2025
 *      Author: Martin
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern SPI_HandleTypeDef hspi3;

void MX_SPI3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SPI_H_ */
