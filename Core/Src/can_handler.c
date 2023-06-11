/**
 * @file FDCAN.c
 *
 * @brief FDCAN
 *  
 */

#include "can_handler.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_uart.h"
#include "usart.h"
uint8_t init_flags = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) 
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) { 
        /* Retreive Rx messages from RX FIFO0 */ 
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) 
        { 
            /* Reception Error */ 
            Error_Handler(); 
        } 

        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) 
        { 
            /* Notification Error */ 
            Error_Handler(); 
        }
        switch (RxHeader.Identifier) { // Leser ID på motatt melding. Casene må lages ut fra ID-er enn mottar. (Sjekk "Interface-Overview" under "Overordnet prosjekt") 
            case ID_INIT_FLAGS:
                {
                    init_flags = RxData[0];
                    break;
                }
            case ID_MARCO: 
            {

                uint8_t data[8] = {'p', 'o', 'l', 'o', '\n', '\0', '\0', '\0'};

                FDCAN_Send(156, hfdcan, data); // 155 => Regulering, 156 => Sensor, 157/158/159 => Kraft1/2/3.
                break;
            }
        } 
    } 
} 

void FDCAN_Init(FDCAN_HandleTypeDef *canPort) 
{ 
    // Configure filter for FDCAN1 
    sFilterConfig.IdType = FDCAN_STANDARD_ID; 
    sFilterConfig.FilterIndex = 0; 
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; 
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 
    sFilterConfig.FilterID1 = 0x40; // FilterID1 fra listen nedenfor settes her. Definerer filterID //
    sFilterConfig.FilterID2 = 0xE0; // FilterID2 fra listen nedenfor settes her. Definerer maskeID //0xE0

    /* ---------FilterID1 og FilterID2 for de ulike gruppene--------- */ 
    // Reguleringskort: FilterID1 = 0x20, FilterID2 = 0xE0 --> Slipper gjennom ideer mellom 32-63 (0x20 - 0x3F) 
    // Sensorskort: FilterID1 = 0x40, FilterID2 = 0xE0 --> Slipper gjennom ideer mellom 64-95 (0x40 - 0x5F) 
    // Kraftkort: FilterID1 = 0x60, FilterID2 = 0xE0 --> Slipper gjennom ideer mellom 96-127 (0x60 - 0x7F) 
    // Kommunikasjonskort: FilterID1 = 0x80, FilterID2 = 0xE0 --> Slipper gjennom ideer mellom 128-159 (0x80 - 0x9F) 
    // Sett inn rett ID-er for din gruppe 
    // Configure TX Header for FDCAN1 

    TxHeader.Identifier = 0x00; 
    TxHeader.IdType = FDCAN_STANDARD_ID; 
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; 
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // Antall byte som sendes 
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; 
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; 
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // Bruker CAN og ikke FDCAN 
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 
    TxHeader.MessageMarker = 0; 

    HAL_FDCAN_ConfigFilter(canPort, &sFilterConfig); 
    HAL_FDCAN_Start(canPort); 
    HAL_FDCAN_ActivateNotification(canPort, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
} 

void FDCAN_Send(uint16_t id, FDCAN_HandleTypeDef *hfdcan, uint8_t *data)
{
    TxHeader.Identifier = id; 
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

void FDCAN_Send_Accel_data(ICM20948 *imu, uint8_t ID)
{
    uint8_t data[8];
    int16_t scale_factor = (int16_t) imu->accel_scale_factor;
    data[0] = imu->accel_data_raw[0] >> 8; 
    data[1] = imu->accel_data_raw[0] & (0xFFU);
    data[2] = imu->accel_data_raw[1]  >> 8; 
    data[3] = imu->accel_data_raw[1] & (0xFFU);
    data[4] = imu->accel_data_raw[2] >> 8; 
    data[5] = imu->accel_data_raw[2] & (0xFFU); 
    data[6] = scale_factor >> 8;
    data[7] = scale_factor & (0xFFU);

    FDCAN_Send(ID, &hfdcan1, data);
}

void FDCAN_Send_Gyro_data(ICM20948 *imu, uint8_t ID)
{
    uint8_t data[8];
    int16_t scale_factor = (int16_t) (imu->gyro_scale_factor / 1000);
    data[0] = imu->accel_data_raw[0] >> 8; 
    data[1] = imu->accel_data_raw[0] & (0xFFU);
    data[2] = imu->accel_data_raw[1]  >> 8; 
    data[3] = imu->accel_data_raw[1] & (0xFFU);
    data[4] = imu->accel_data_raw[2] >> 8; 
    data[5] = imu->accel_data_raw[2] & (0xFFU); 
    data[6] = scale_factor >> 8;
    data[7] = scale_factor & (0xFFU);

    FDCAN_Send(ID, &hfdcan1, data);
}

void FDCAN_Send_Orientation_Data(struct orientation *orient, uint8_t ID)
{
    uint8_t data[8] = {0};
    // Vurder å slå av avbruddet som oppdaterer vinkler
    int16_t roll = (int16_t) (orient->roll_deg * 100);
    int16_t pitch = (int16_t) (orient->pitch_deg * 100);

    data[0] = roll & (0xFFU);
    data[1] = roll >> 8;
    data[2] = pitch & (0xFFU);
    data[3] = pitch >> 8;

    FDCAN_Send(ID, &hfdcan1, data);
}

void FDCAN_Send_Temp_Pres(STTS75 *temp_sensor, MS5837 *pres_sensor, uint8_t ID)
{
    uint8_t data[8] = {0};
    int16_t depth = (int16_t) pres_sensor->cm_dybde;
    int16_t water_temp = (int16_t) (pres_sensor->vanntemp);

    int16_t temp_pcb = (int16_t) (temp_sensor->temp_degc * 100);

    data[0] = depth & (0xFFU);
    data[1] = depth >> 8; 
    data[2] = water_temp & (0xFFU);
    data[3] = water_temp  >> 8; 
    data[4] = temp_pcb & (0xFFU); 
    data[5] = temp_pcb >> 8; 

    if (data[5] == 0x8a)
    {
    	volatile mau = 1;

    }
    if (temp_pcb <= 0){
    	volatile mau = 0;
    }

    FDCAN_Send(ID, &hfdcan1, data);
}

void FDCAN_Send_Reg(struct orientation *orient, MS5837 *pres_sensor)
{
    uint8_t data[8] = {0};
    // Vurder å slå av avbruddet som oppdaterer vinkler
    volatile int16_t roll = (int16_t) (orient->roll_deg * 100);
    volatile int16_t pitch = (int16_t) (orient->pitch_deg * 100);
    volatile int16_t depth = (int16_t) pres_sensor->cm_dybde;


    data[0] = roll & (0xFFU);
    data[1] = roll >> 8;
    data[2] = pitch & (0xFFU);
    data[3] = pitch >> 8;
    data[4] = depth & (0xFFU);
    data[5] = depth >> 8; 

    FDCAN_Send(REG_ID, &hfdcan1, data);

}

