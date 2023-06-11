/**
 * @file FDCAN.c
 *
 * @brief FDCAN Headder file
 *  
 */

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "orient.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

#include "ICM20948.h"
#include "STTS75.h"
#include "MS5837.h"

/* IDer for å motta data */
#define ID_INIT_FLAGS 66 
#define ID_MARCO 95

/* IDer for å sende data til topside */
#define TOP_ID_ACCEL_DATA (uint16_t) 135
#define TOP_ID_GYRO_DATA (uint16_t) 136
#define TOP_ID_ORIENTATION_DATA (uint16_t) 138
#define TOP_ID_TEMP_PRES (uint16_t) 139
#define TOP_ID_ERROR_CODES (uint16_t) 140

/* IDer for å sende data til regulering */
#define REG_ID 35

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

#endif /* ifndef CAN_HANDLER_H_ */
