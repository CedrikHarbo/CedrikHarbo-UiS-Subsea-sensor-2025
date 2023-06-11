/**
 * @file ICM20948.h
 *
 * @author Håvard Syslak
 * 
 * @brief Inneholder definisjoner og funksjoner for ICM20948 SPI-driveren. Laget for stm32g431 mikrokontrolleren.
 */


#ifndef SRC_ICM20948_H_
#define SRC_ICM20948_H_

#include <stdint.h>
#include <stdbool.h>

//#include "stm32_hal_legacy.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"

/**
 * @defgroup ICM20948 definisjoner
 * @{
 */
#define ICM20948_SPI_TIMEOUT 1000U
#define ICM20948_SPI_WRITE_MASK 0xFFU
#define ICM20948_SPI_READ_MASK 0x80U
#define ICM20948_SEL_BANK_0 0x00U
#define ICM20948_SEL_BANK_1 0x01U
#define ICM20948_SEL_BANK_2 0x02U
#define ICM20948_SEL_BANK_3 0x03U
#define ICM20948_RESET 0x80U | 0x41U
#define ICM20948_WAKEUP 0x01U
#define ICM20948_CS_HOLD_TIME_NS 500
#define ICM20948_CS_SETUP_TIME_NS 8
#define ICM20948_I2C_EN_SVL0 0x01U << 5
/* @} */


/*
 * Macro
 */
// #define ICM20948_OP_READ(reg) (reg | ICM20948_SPI_READ_MASK)
// #define ICM20948_OP_WRITE(reg) (reg & ICM20948_SPI_WRITE_MASK)


/**
 * @defgroup ICM20948 registerbank 0 definisjoner
 * @{
 */
#define ICM20948_WHO_AM_I 0x00U
#define ICM20948_USER_CTRL 0x3U
#define ICM20948_LP_CONFIG 0x05U
#define ICM20948_PWR_MGMT_1 0x06U
#define ICM20948_PWR_MGMT_2 0x07U
#define ICM20948_I2C_MST_STATUS 0x17U

#define ICM20948_ACCEL_XOUT_H 0x2DU
#define ICM20948_ACCEL_XOUT_L 0x2EU
#define ICM20948_ACCEL_YOUT_L 0x2FU
#define ICM20948_ACCEL_YOUT_H 0x30U
#define ICM20948_ACCEL_ZOUT_H 0x31U
#define ICM20948_ACCEL_ZOUT_L 0x32U

#define ICM20948_GYRO_XOUT_H 0x33U
#define ICM20948_GYRO_XOUT_L 0x34U
#define ICM20948_GYRO_YOUT_H 0x35U
#define ICM20948_GYRO_YOUT_L 0x36U
#define ICM20948_GYRO_ZOUT_H 0x37U
#define ICM20948_GYRO_ZOUT_L 0x38U

#define ICM20948_TEMP_OUT_H 0x39U
#define ICM20948_TEMP_OUT_HL 0x3AU

#define ICM20948_FIFO_EN_2 0x1FU
#define ICM20948_DATA_RDY_STATUS 0x74U
#define ICM20948_REG_BANK_SEL 0x7FU

/* Extern I2C slave data register */
#define ICM20948_EXT_SLV_SENS_DATA_00 0x39U
#define ICM20948_EXT_SLV_SENS_DATA_01 0x3AU
#define ICM20948_EXT_SLV_SENS_DATA_02 0x3BU
#define ICM20948_EXT_SLV_SENS_DATA_03 0x3CU
#define ICM20948_EXT_SLV_SENS_DATA_04 0x3DU
#define ICM20948_EXT_SLV_SENS_DATA_05 0x3EU
#define ICM20948_EXT_SLV_SENS_DATA_06 0x3FU
#define ICM20948_EXT_SLV_SENS_DATA_07 0x40U
#define ICM20948_EXT_SLV_SENS_DATA_08 0x41U
#define ICM20948_EXT_SLV_SENS_DATA_09 0x42U
#define ICM20948_EXT_SLV_SENS_DATA_10 0x43U
#define ICM20948_EXT_SLV_SENS_DATA_11 0x44U
#define ICM20948_EXT_SLV_SENS_DATA_12 0x45U
#define ICM20948_EXT_SLV_SENS_DATA_13 0x46U
#define ICM20948_EXT_SLV_SENS_DATA_14 0x47U
#define ICM20948_EXT_SLV_SENS_DATA_15 0x48U
#define ICM20948_EXT_SLV_SENS_DATA_16 0x49U
#define ICM20948_EXT_SLV_SENS_DATA_17 0x4AU
#define ICM20948_EXT_SLV_SENS_DATA_18 0x4BU
#define ICM20948_EXT_SLV_SENS_DATA_19 0x4CU
#define ICM20948_EXT_SLV_SENS_DATA_20 0x4DU
#define ICM20948_EXT_SLV_SENS_DATA_21 0x4EU
#define ICM20948_EXT_SLV_SENS_DATA_22 0x4FU
#define ICM20948_EXT_SLV_SENS_DATA_23 0x50U
/* @} */

/**
 * @defgroup ICM20948 registerbank 2 definisjoner
 * @{
 */
#define ICM20948_GYRO_CONFIG_1 0x01U
#define ICM20948_GYRO_CONFIG_2 0x02U
#define ICM20948_ACCEL_CONFIG 0x20U
#define ICM20948_ODR_ALIGN_ENABLE 0x01U
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10U // MSB for accel sample rate div
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11U // LSB for accel sample rate div
/* @} */

/**
 * @defgroup ICM20948 registerbank 3 definisjoner
 * @{
 */
#define ICM20948_I2C_MST_CTRL 0x01U
#define ICM20948_I2C_SLV0_ADDR 0x03U
#define ICM20948_I2C_SLV0_REG 0x04U
#define ICM20948_I2C_SLV0_CTRL 0x05U
#define ICM20948_I2C_SLV0_DO 0x06U // Data out when slave 0 is set to write.
/* @} */

/* AK09916 Registers*/
/** 
 * @defgroup AK09916 magnetometer definisjoner
 * @{
 */
#define AK09916_DEVICE_ID 0x01U
#define AK09916_STATUS_1 0x10U
#define AK09916_MAG_XOUT_H 0x11U
#define AK09916_MAG_XOUT_L 0x12U
#define AK09916_MAG_YOUT_H 0x13U
#define AK09916_MAG_YOUT_L 0x14U
#define AK09916_MAG_ZOUT_H 0x15U
#define AK09916_MAG_ZOUT_L 0x16U
#define AK09916_STATUS_2 0x18U
#define AK09916_CONTROL_2 0x31U

#define AK09916_I2C_ADDR (0x0CU << 1)
/* @} */

/* @defgroup ICM20948 
 * @{
 */

typedef enum 
{
    ENABLED = 0x00U,
    DISABLED = 0x01U,
    
} ICM20948_SPI_CS;


typedef enum 
{
    ICM20948_OK                 = 0x00U,
    ICM20948_HAL_ERROR          = 0x01U,
    ICM20948_HAL_BUSY           = 0x02U,
    ICM20948_HAL_TIMEOUT        = 0x03U,
    ICM20948_INIT_ERROR         = 0x04U,
    ICM20948_WHO_AM_I_ERROR     = 0x05U,
    ICM20948_MAG_ERROR          = 0x06U,
    ICM20948_MAG_WHO_AM_I_ERROR = 0x07U
} ICM20948_StatusTypeDef;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    uint8_t selected_user_bank;
    uint16_t CS_Pin;
    GPIO_TypeDef *CS_GPIO;

    float gyro_scale_factor;
    float accel_scale_factor;
    float mag_scale_factor;

    float gyro_data[3]; 
    float accel_data[3];
    float mag_data[3];

    int16_t gyro_data_raw[3]; 
    int16_t accel_data_raw[3];
    int16_t mag_data_raw[3];

    int16_t accel_offs[3];
    int16_t gyro_offs[3];

    

} ICM20948;

typedef enum 
{
    ICM20949_INTERNAL_CLK = 0x0U,
    ICM20948_AUTO_SEL_CLK = 0x01U,

} ICM20948_CLK_SELECTOR;

typedef enum
{
    ICM20948_2g = 0x00U,
    ICM20948_4g = 0x01U,
    ICM20948_8g = 0x02U,
    ICM20948_16g = 0x03U,
} ICM20948_Accel_Scale;

typedef enum
{
    ICM20948_250dps = 0x00U, 
    ICM20948_500dps = 0x01U, 
    ICM20948_1000dps = 0x02U, 
    ICM20948_2000dps = 0x03U, 
} ICM20948_Gyro_Scale;

typedef struct
{
    /*GENERAL*/
    uint8_t int_enable;
    ICM20948_CLK_SELECTOR clk_src;

    /* ACCEL CONFIG */
    bool accel_enable;
    ICM20948_Accel_Scale accel_scale;
    bool accel_lpf_enable;
    float accel_break_freq; // ref table 16 p. 60 in datasheet.

    /* GYRO CONFIG */
    bool gyro_enable;
    ICM20948_Gyro_Scale gyro_scale;
    bool gyro_lpf_enable;
    float gyro_break_freq;

    /* MAG CONFIG */
    bool mag_enable;
} ICM20948_InitTypeDef;



/* INIT */
ICM20948_StatusTypeDef ICM20948_Init(ICM20948 *imu,
        SPI_HandleTypeDef *hspi,
        uint16_t CS_pin,
        GPIO_TypeDef *CS_GPIOx,
        ICM20948_InitTypeDef *config);
ICM20948_StatusTypeDef ICM20948_Init_Accel(ICM20948 *imu, ICM20948_InitTypeDef *config);
ICM20948_StatusTypeDef ICM20948_Init_Gyro(ICM20948 *imu, ICM20948_InitTypeDef *config);
ICM20948_StatusTypeDef ICM20948_Init_Mag(ICM20948 *imu);
ICM20948_StatusTypeDef ICM20948_Read_Data_Poll(ICM20948 *imu);
void ICM20948_Set_Scale_Factor(ICM20948 *imu, 
        ICM20948_Gyro_Scale gyro_scale, 
        ICM20948_Accel_Scale accel_scale);


ICM20948_StatusTypeDef ICM20948_Read_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf);
ICM20948_StatusTypeDef ICM20948_Write_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf, uint8_t data);
ICM20948_StatusTypeDef ICM20948_Read_Accel(ICM20948 *imu);
ICM20948_StatusTypeDef ICM20948_Read_Gyro(ICM20948 *imu);
ICM20948_StatusTypeDef ICM20948_Read_Mag(ICM20948 *imu);
void ICM20948_SPI_Enable(ICM20948 *imu);
void ICM20948_SPI_Disable(ICM20948 *imu);
ICM20948_StatusTypeDef ICM20948_Select_Register_Bank(ICM20948 *imu, uint8_t bank_nr);
ICM20948_StatusTypeDef ICM20948_Read_Tempearature(ICM20948 *imu);

ICM20948_StatusTypeDef AK09916_Read_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf);
// ICM20948_StatusTypeDef AK09916_Write_Register(ICM20948 )
/* @} */

#endif /* SRC_ICM20948_H_ */ 

