/*
 * error_handler.h
 *
 *  Created on: Apr 3, 2025
 *      Author: Martin
 */

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

#include "ICM20948.h"
#include "STTS75.h"

extern uint8_t IMU_err;
extern uint8_t leak_status;
extern uint8_t temp_err;
extern uint8_t pres_error;

void IMU_error_handler(ICM20948_StatusTypeDef err);
void Send_Error_Code();
void Temp_error_handler(STTS75_StatusTypeDef err);
void Pres_error_handler(MS5837_StatusTypeDef err);

#endif /* INC_ERROR_HANDLER_H_ */
