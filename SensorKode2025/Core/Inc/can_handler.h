/*
 * can_handler.h
 *
 *  Created on: Apr 2, 2025
 */

#ifndef INC_CAN_HANDLER_H_
#define INC_CAN_HANDLER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "orient.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

#include "ICM20948.h"
#include "STTS75.h"
#include "MS5837.h"

/* IDer for å motta data */
#define ID_INIT_FLAGS 66
#define ID_MARCO 95

/* IDer for å sende data til topside */
#define TOP_ID_ACCEL_DATA (uint16_t) 135 //0x87
#define TOP_ID_GYRO_DATA (uint16_t) 136 //0x88
#define TOP_ID_ORIENTATION_DATA (uint16_t) 138 //0x8A
#define TOP_ID_TEMP_PRES (uint16_t) 139 //0x8B
#define TOP_ID_ERROR_CODES (uint16_t) 140 //0x8C
#define TEST_ID (uint16_t) 123

/* IDer for å sende data til regulering */
#define REG_ID 34				//0x22

extern uint8_t init_flags;

extern struct can_flags can_flags;


// extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_FilterTypeDef sFilterConfig;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern FDCAN_HandleTypeDef hfdcan1;

extern uint8_t TxData[8];
extern uint8_t RxData[8];


void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
void FDCAN_Send(uint16_t id, FDCAN_HandleTypeDef *hfdcan, uint8_t *data);
void FDCAN_Send_Accel_data(ICM20948 *imu, uint8_t ID);
void FDCAN_Send_Gyro_data(ICM20948 *imu, uint8_t ID);
void FDCAN_Send_Orientation_Data(struct orientation *orient, uint8_t ID);
void FDCAN_Send_Temp_Pres(STTS75 *temp_sensor, MS5837 *pres_sensor, uint8_t ID);
void FDCAN_Send_Reg(struct orientation *orient, MS5837 *pres_sensor);

#endif /* INC_CAN_HANDLER_H_ */
