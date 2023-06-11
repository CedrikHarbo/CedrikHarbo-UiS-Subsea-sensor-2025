/**
 * @file ICM20948.c
 *
 * @author Håvard Syslak
 * @date 07.01.2023
 *
 * @breif Low level SPI drivers for the ICM20948 IMU
 */

#include "ICM20948.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include "helpers.h"
#include <stdint.h>
#include <string.h>


/* 
 * note: SPI operation features
 * 1.   Data is delivered MSB first and LSB last
 * 2.   Data is latched on the rising edge of SCLK
 * 3.   Data should be transitioned on the falling edge of SCLK
 * 4.   The maximum frequency of SCLK is 7Mhz
 * 5.   SPI read and write operations are completed in 16 or more clock cycles (two or more bytes).
 *      The first byte contains the SPI adress. The second byte(s) contains the Data
 * 6.   MSB of adress = 1 ==> read operationicm
 *      MSB of adress = 0 ==> write operation
 */ 


/**
 * @brief Initialiser ICM20948 IMU-en ihht. ICM20948_InitTypeDef struktur.
 *
 * @param *hspi peker til SPI_HandleTypeDef struktur.
 * @param CS_pin pinne som "chip select" signalet går ut på.
 * @param *CS_GPIOx peiker til GPIO_TypeDef som "chip select" signalet går ut på
 * @param *config peiker til ICM20948_InitTypeDef struktor som inneheldt informasjon om korleis IMU-en skal initaiseres.
 *
 * @retval ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Init(ICM20948 *imu,
        SPI_HandleTypeDef *hspi, 
        uint16_t CS_Pin, 
        GPIO_TypeDef *CS_GPIOx,
        ICM20948_InitTypeDef *config)
{
    ICM20948_StatusTypeDef retval = ICM20948_OK;
    imu->hspi = hspi;
    imu->CS_Pin = CS_Pin;
    imu->CS_GPIO = CS_GPIOx;

    memset(imu->accel_offs, 0, sizeof(imu->accel_offs));
    memset(imu->gyro_offs, 0, sizeof(imu->gyro_offs));

    ICM20948_Set_Scale_Factor(imu, config->gyro_scale, config->accel_scale);

    // Set CS pin low to enable spi communication

    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    retval = ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    if (retval != ICM20948_OK) return retval;
    delay_ms(200);

    // Sowftware reset ref p.19 in datasheet
    ICM20948_Write_Register(imu, ICM20948_PWR_MGMT_1, tx_buf, rx_buf, ICM20948_RESET);
    delay_ms(100);

    // Starts in slepmode. Wakeup and select clk src.
    // Wakeup and select clock.
    retval = ICM20948_Write_Register(imu, ICM20948_PWR_MGMT_1, tx_buf, rx_buf, ICM20948_WAKEUP | config->clk_src);
    delay_ms(80);
    //while (HAL_GetTick() - tickstart <= 80) { }

    // Check who am I?
    retval = ICM20948_Read_Register(imu, ICM20948_WHO_AM_I, tx_buf, rx_buf);
    if (retval != HAL_OK) return retval;
    if (rx_buf[1] != 0xEAU) return ICM20948_INIT_ERROR; // TODO

    uint8_t data = 0x10; // Disable i2c (SPI mode only)
    retval = ICM20948_Write_Register(imu, ICM20948_USER_CTRL, tx_buf, rx_buf, data);
    if (retval != HAL_OK) return retval;

    data = 0x00; // Accel and gyro enabled
    if (!config->accel_enable) 
    {
        data = 0x07 << 3;
    }
    if (!config->gyro_enable)
    {
        data |= 0x07;
    }

    retval = ICM20948_Write_Register(imu, ICM20948_PWR_MGMT_2, tx_buf, rx_buf, data);
    if (retval != ICM20948_OK) return retval;

    retval = ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_2);
    if (retval != ICM20948_OK) return retval;

    // enable ord align
    data = 0x01;
    // retval = ICM20948_Write_Register(imu, ICM20948_ODR_ALIGN_ENABLE, tx_buf, rx_buf, data);
    // if (retval != HAL_OK) return retval;

    if (config->accel_enable) 
    {
        retval = ICM20948_Init_Accel(imu, config);
        if (retval != ICM20948_OK) return retval;
    }
    
    if (config->gyro_enable)
    {
        retval = ICM20948_Init_Gyro(imu, config);
        if (retval != ICM20948_OK) return retval;
    }

    if (config->mag_enable)
    {
        retval = ICM20948_Init_Mag(imu);
        if (retval != ICM20948_OK) return retval;
        imu->mag_scale_factor = 0.15f;
    }

    retval = ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    if (retval != HAL_OK) return retval;

    return retval;
}

/**
 * @brief Set skaleringsfaktor bassert på valgt måleområde
 *
 * @param *imu peiker til en ICM20948 struktur som inneholder IMU data
 * @param gryo_scale skaleringsfaktoren som skal settes for gyroskopet
 * @param accel_scale skaleringsfaktoren som skal settes for akselerometeret
 *
 * @retval None
 */
void ICM20948_Set_Scale_Factor(ICM20948 *imu, ICM20948_Gyro_Scale gyro_scale, ICM20948_Accel_Scale accel_scale)
{
    switch (gyro_scale) {
        case ICM20948_250dps: 
            imu->gyro_scale_factor = 131;
            break;

        case ICM20948_500dps:
            imu->gyro_scale_factor = 65.5;
            break;

        case ICM20948_1000dps:
            imu->gyro_scale_factor = 32.8;
            break;

        case ICM20948_2000dps:
            imu->gyro_scale_factor = 16.4;
            break;
    } 

    switch (accel_scale) {
        case ICM20948_2g: 
            imu->accel_scale_factor = 16384;
            break;

        case ICM20948_4g:
            imu->accel_scale_factor = 8192;
            break;

        case ICM20948_8g:
            imu->accel_scale_factor = 4096;
            break;

        case ICM20948_16g:
            imu->accel_scale_factor = 2048;
    } 
}

/**
 * @brief   Initialiser akselerometeret ihht. konfigurasjonsstrukturen ICM20948_InitTypeDef.
 * @param   *imu peker til ICM20948-struktur som inneholder IMU data.
 * @param   *config peker til ICM20948_InitTypeDef-strukturen som inneholder informasjon om hvordan akselerometeret skal
 *          initialiseres
 * @retval  ICM20948_StatusTypeDef
 *
 * @note Akselsrometerets samplingsrate register er urørt. Får då samplingsrate på 1125Hz.
 */
ICM20948_StatusTypeDef ICM20948_Init_Accel(ICM20948 *imu, ICM20948_InitTypeDef *config)
{
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};

    if (imu->selected_user_bank != ICM20948_SEL_BANK_2)
    {
        return ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_2);
    }

    uint8_t dplf_cfg = 0;
    if (config->accel_lpf_enable) {
        if (config->accel_break_freq <= 5.7) dplf_cfg = 0x06U;
        else if (config->accel_break_freq <= 11.5) dplf_cfg = 0x05U;
        else if (config->accel_break_freq <= 23.9) dplf_cfg = 0x04U;
        else if (config->accel_break_freq <= 50.4) dplf_cfg = 0x03U;
        else if (config->accel_break_freq <= 111.4) dplf_cfg = 0x02U;
        else if (config->accel_break_freq <= 196.6) dplf_cfg = 0x00U;
        else if (config->accel_break_freq <= 246.0) dplf_cfg = 0x01U;
        else dplf_cfg = 0x07U; // => break freq = 473Hz 
                               // Which is max when dplf is enabled
    }

    uint8_t data = (dplf_cfg << 3) | (config->accel_scale << 1) | config->accel_lpf_enable;
    return ICM20948_Write_Register(imu, ICM20948_ACCEL_CONFIG, rx_buf, tx_buf, data);
}

/**
 * @brief   Initialiser gyroskopet ihht. konfigurasjonsstrukturen ICM20948_InitTypeDef.
 * @param   *imu peker til ICM20948-struktur som inneholder IMU data.
 * @param   *config peker til ICM20948_InitTypeDef-strukturen som inneholder informasjon om hvordan akselerometeret skal
 *          initialiseres
 *
 * @note    Gyroskopets samplingsrate register er urørt. Får då samplingsrate på 1125Hz.
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Init_Gyro(ICM20948 *imu, ICM20948_InitTypeDef *config)
{
    if (imu->selected_user_bank != ICM20948_SEL_BANK_2)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_2);
    }

    uint8_t dplf_cfg = 0;
    if (config->gyro_lpf_enable) {
        if (config->gyro_break_freq <= 5.7) dplf_cfg = 0x06U;
        else if (config->gyro_break_freq <= 11.6) dplf_cfg = 0x05U;
        else if (config->gyro_break_freq <= 23.8) dplf_cfg = 0x04U;
        else if (config->gyro_break_freq <= 51.2) dplf_cfg = 0x03U;
        else if (config->gyro_break_freq <= 119.5) dplf_cfg = 0x02U;
        else if (config->gyro_break_freq <= 151.8) dplf_cfg = 0x01U;
        else if (config->gyro_break_freq <= 196.6) dplf_cfg = 0x00U;
        else dplf_cfg = 0x07U; // => break freq = 351.4Hz
                               // Which is max when dplf is enabled
    }

    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    uint8_t data = (dplf_cfg << 3) | (config->accel_scale << 1) | config->accel_lpf_enable;
    ICM20948_StatusTypeDef retval = ICM20948_Write_Register(imu, ICM20948_ACCEL_CONFIG, rx_buf, tx_buf, data);

    return retval;
}


/**
 * @brief   Initialiserer magnetometeret AK09916
 * @param   *imu Peker ti ICM20948-struktur
 *
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Init_Mag(ICM20948 *imu)
{
    ICM20948_StatusTypeDef retval;
    if (imu->selected_user_bank != ICM20948_SEL_BANK_0)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    }

    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    
    retval = ICM20948_Read_Register(imu, ICM20948_USER_CTRL, tx_buf, rx_buf);
    uint8_t user_ctrl_cfg = rx_buf[2];
    uint8_t data = user_ctrl_cfg | ICM20948_I2C_EN_SVL0;
    /* Enable the I2C master */ 
    retval = ICM20948_Write_Register(imu, ICM20948_USER_CTRL, tx_buf, rx_buf, data);
    if (retval != ICM20948_OK) return retval;

    /* Reset the I2C master */
    data = user_ctrl_cfg | 0x02;
    retval = ICM20948_Write_Register(imu, ICM20948_USER_CTRL, tx_buf, rx_buf, data);

    ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_3);
    data = 0x07 << 1; // Set I2C cloc as recomended in the datasheet ref. page 81
    retval = ICM20948_Write_Register(imu, ICM20948_I2C_MST_CTRL, tx_buf, rx_buf, data);
    
    retval = AK09916_Read_Register(imu, AK09916_DEVICE_ID, tx_buf, rx_buf);

    // Who am i erro
    if (rx_buf[1] != 0x09U)
    {
        return ICM20948_MAG_WHO_AM_I_ERROR;
    }

    ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    return retval;
}

/**
 * @breif   Velg IMU-en som som aktiv slave ved å sette CS-pinnen lav
 * @param   *imu Peiker på ICM20948-struktur
 *
 * @note    Kaller må påse at oppsettningstida er overholdt 
 * 
 * @retval None
 */
void ICM20948_SPI_Enable(ICM20948 *imu)
{
    //HAL_GPIO_WritePin(imu->CS_GPIO, imu->CS_Pin, GPIO_PIN_RESET);

    imu->CS_GPIO->BRR = (uint32_t)imu->CS_Pin;
    //imu->CS_Pin_Status = ENABLED;
    //delay_ns(ICM20948_CS_SETUP_TIME_NS);
}

/**
 * @breif   Set CS-pinnen lav for å slå av SPI-kommunikasjonen
 * @param   *imu Peiker på ICM20948-struktur
 *
 * @note    Kaller må påse at oppsettningstida er overholdt 
 * 
 * @retval None
 */
void ICM20948_SPI_Disable(ICM20948 *imu)
{
    //delay_ns(ICM20948_CS_HOLD_TIME_NS);
    HAL_GPIO_WritePin(imu->CS_GPIO, imu->CS_Pin, GPIO_PIN_SET);
    //imu->CS_Pin_Status = DISABLED;
}

/**
 * @brief   Leser verdien av et bestemt register i ICM20948 IMU-en
 * @param   *imu peker til ICM20948-struktur
 * @param   reg registeret som skal leses fra
 * @param   *pTx_buf en peker til bufferet som inneholder dataen som skal sendes over SPI
 * @param   *pRx_buf en peker til bufferet som inneholder dataen som skal mottas over SPI
 *
 * @retval  ICM20948_StatusTypeDef Statusen til ICM20948
 */
ICM20948_StatusTypeDef ICM20948_Read_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf)
{
    ICM20948_StatusTypeDef retval = ICM20948_OK;
    reg |= ICM20948_SPI_READ_MASK;
    pTx_buf[0] = reg;
    ICM20948_SPI_Enable(imu);
    retval = (ICM20948_StatusTypeDef) HAL_SPI_TransmitReceive(imu->hspi, pTx_buf, pRx_buf, 2, ICM20948_SPI_TIMEOUT);
    ICM20948_SPI_Disable(imu);
    return retval;
}

/**
 * @brief   Skriver til et bestemt register i ICM20948 IMU-en
 * @param   *imu peker til ICM20948-struktur
 * @param   reg registeret som skal skrives til
 * @param   *pTx_buf en peker til bufferet som inneholder dataen som skal sendes over SPI
 * @param   *pRx_buf en peker til bufferet som inneholder dataen som skal mottas over SPI
 *
 * @retval  ICM20948_StatusTypeDef Statusen til ICM20948
 */
ICM20948_StatusTypeDef ICM20948_Write_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf, uint8_t data)
{
    // TODO check if chip is ENABLED
    // If not enable and wait
    ICM20948_StatusTypeDef retval = ICM20948_OK;
    reg &= ICM20948_SPI_WRITE_MASK;
    pTx_buf[0] = reg;
    pTx_buf[1] = data;

    ICM20948_SPI_Enable(imu);
    //imu->CS_GPIO->ODR &= ~(0x01U << imu->CS_Pin);
    retval = (ICM20948_StatusTypeDef) HAL_SPI_TransmitReceive(imu->hspi, pTx_buf, pRx_buf, 2, ICM20948_SPI_TIMEOUT);
    ICM20948_SPI_Disable(imu);
    return retval;
}

/**
 * @breif Velg register bank
 *
 * @param   *imu peker på en ICM20948-struktur
 * @param   bank_nr register bank som skal byttes til
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Select_Register_Bank(ICM20948 *imu, uint8_t bank_nr)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    imu->selected_user_bank = bank_nr;
    bank_nr = bank_nr << 4; // bit [3:0]: reserved, [5:4]: Selected bank
    return ICM20948_Write_Register(imu, ICM20948_REG_BANK_SEL, tx_buf, rx_buf, bank_nr);
}

/**
 * @brief   Les akselerometer data fra ICM20948 IMU-en
 * @param   *imu peker til en ICM20948-struktur
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Read_Accel(ICM20948 *imu)
{
    if (imu->selected_user_bank != ICM20948_SEL_BANK_0)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
        imu->selected_user_bank = ICM20948_SEL_BANK_0;
    }

    ICM20948_StatusTypeDef retval = ICM20948_OK;
    uint8_t rx_buf[6];
    uint8_t reg = ICM20948_ACCEL_XOUT_H | ICM20948_SPI_READ_MASK;

    ICM20948_SPI_Enable(imu);
    retval = (ICM20948_StatusTypeDef) HAL_SPI_Transmit(imu->hspi, &reg, 1, ICM20948_SPI_TIMEOUT);
    retval = (ICM20948_StatusTypeDef) HAL_SPI_Receive(imu->hspi, rx_buf, 6, ICM20948_SPI_TIMEOUT);
    ICM20948_SPI_Disable(imu);

    imu->accel_data_raw[0] = (rx_buf[0] << 8 | rx_buf[1]) - imu->accel_offs[0];
    imu->accel_data_raw[1] = (rx_buf[2] << 8 | rx_buf[3]) - imu->accel_offs[1];
    imu->accel_data_raw[2] = (rx_buf[4] << 8 | rx_buf[5]) - imu->accel_offs[2];

    imu->accel_data[0] = (float) imu->accel_data_raw[0] / imu->accel_scale_factor;
    imu->accel_data[1] = (float) imu->accel_data_raw[1] / imu->accel_scale_factor;
    imu->accel_data[2] = (float) imu->accel_data_raw[2] / imu->accel_scale_factor;
    return retval;
}

/**
 * @brief   Les akselerometer data fra ICM20948 IMU-en
 * @param   *imu peker til en ICM20948-struktur
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef ICM20948_Read_Gyro(ICM20948 *imu)
{
    if (imu->selected_user_bank != ICM20948_SEL_BANK_0)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
        imu->selected_user_bank = ICM20948_SEL_BANK_0;
    }
    uint8_t tx_buf[2] = {ICM20948_DATA_RDY_STATUS | ICM20948_SPI_READ_MASK, 0};
    uint8_t rx_buf[6] = {0};

    ICM20948_StatusTypeDef retval = ICM20948_OK;

    uint8_t reg = ICM20948_GYRO_XOUT_H | ICM20948_SPI_READ_MASK;

    ICM20948_SPI_Enable(imu);
    HAL_SPI_Transmit(imu->hspi, &reg, 1, ICM20948_SPI_TIMEOUT);
    retval = (ICM20948_StatusTypeDef) HAL_SPI_Receive(imu->hspi, rx_buf, 6, ICM20948_SPI_TIMEOUT);
    ICM20948_SPI_Disable(imu);

    imu->gyro_data_raw[0] = (rx_buf[0] << 8 | rx_buf[1]) - imu->gyro_offs[0];
    imu->gyro_data_raw[1] = (rx_buf[2] << 8 | rx_buf[3]) - imu->gyro_offs[1];
    imu->gyro_data_raw[2] = (rx_buf[4] << 8 | rx_buf[5]) - imu->gyro_offs[2];

    imu->gyro_data[0] = (float) imu->gyro_data_raw[0] / imu->gyro_scale_factor;
    imu->gyro_data[1] = (float) imu->gyro_data_raw[1] / imu->gyro_scale_factor;
    imu->gyro_data[2] = (float) imu->gyro_data_raw[2] / imu->gyro_scale_factor;

    return retval;
}

/**
 * @brief   Les mangetometerdata fra AK09916 via ICM20948 IMU-en
 * @param   *imu peker til en ICM20948-struktur
 * @retval  ICM20948_StatusTypeDef
 */
ICM20948_StatusTypeDef AK09916_Read_Register(ICM20948 *imu, uint8_t reg, uint8_t *pTx_buf, uint8_t *pRx_buf)
{
    ICM20948_StatusTypeDef retval;
    if (imu->selected_user_bank != ICM20948_SEL_BANK_3)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_3);
    }

    /* Set I2C Slave in read mode */
    uint8_t data = (AK09916_I2C_ADDR) | 0x01U;
    retval = ICM20948_Write_Register(imu, ICM20948_I2C_SLV0_ADDR, pTx_buf, pRx_buf, data);
    if (retval != ICM20948_OK) return retval;
    /* Set the I2C register to read from */
    data = reg;
    retval = ICM20948_Write_Register(imu, ICM20948_I2C_SLV0_REG, pRx_buf, pRx_buf, data);
    if (retval != ICM20948_OK) return retval;

    /* Enable reading data from slave, and set number of bytes to read to 1 */
    data = 0x81;
    retval = ICM20948_Write_Register(imu, ICM20948_I2C_SLV0_CTRL, pTx_buf, pRx_buf, data);
    if (retval != ICM20948_OK) return retval;


    ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    //delay_ms(1);
    retval = ICM20948_Read_Register(imu, ICM20948_EXT_SLV_SENS_DATA_00, pTx_buf, pRx_buf);

    return retval;
}

/**
 * @breif   sjekk om magnetometer data fra AK09916 er klar.
 *          Les data.
 * @param   *imu peker til en ICM20948-struktur
 * @retval  ICM20948_StatusTypeDef 
*/
ICM20948_StatusTypeDef ICM20948_Read_Mag(ICM20948 *imu)
{
    ICM20948_StatusTypeDef retval;
    if (imu->selected_user_bank != ICM20948_SEL_BANK_3)
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_3);
    }
    
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[6] = {0};

    retval = AK09916_Read_Register(imu, AK09916_STATUS_1, tx_buf, rx_buf);
    
    if (rx_buf[1] & 0x01U) // Data ready
    {

        /* Set reg adress to read from slave 0 */
        uint8_t data = AK09916_MAG_XOUT_L;
        retval = ICM20948_Write_Register(imu, ICM20948_I2C_SLV0_REG, tx_buf, rx_buf, data);

        /* Enable data read from slave 0, and set number of bytes to read to 6. */
        data = 0x80 | 0x06;
        retval = ICM20948_Write_Register(imu, ICM20948_I2C_SLV0_CTRL, tx_buf, rx_buf, data);
        uint32_t tickstart = HAL_GetTick(); 
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);

        delay_ns(100000);
        
        uint8_t reg = ICM20948_EXT_SLV_SENS_DATA_00 | ICM20948_SPI_READ_MASK;

        ICM20948_SPI_Enable(imu);
        HAL_SPI_Transmit(imu->hspi, &reg, 1, ICM20948_SPI_TIMEOUT);
        retval = (ICM20948_StatusTypeDef) HAL_SPI_Receive(imu->hspi, rx_buf, 6, ICM20948_SPI_TIMEOUT);
        ICM20948_SPI_Disable(imu);

        imu->mag_data_raw[0] = (rx_buf[0] << 8 | rx_buf[1]);
        imu->mag_data_raw[1] = (rx_buf[2] << 8 | rx_buf[3]);
        imu->mag_data_raw[2] = (rx_buf[4] << 8 | rx_buf[5]);

        imu->mag_data[0] = (float) imu->mag_data_raw[0] / imu->mag_scale_factor;
        imu->mag_data[1] = (float) imu->mag_data_raw[1] / imu->mag_scale_factor;
        imu->mag_data[2] = (float) imu->mag_data_raw[2] / imu->mag_scale_factor;

    }
    else
    {
        ICM20948_Select_Register_Bank(imu, ICM20948_SEL_BANK_0);
    }
    return retval;
}

/**
 * @breif   Les data fra akselerometeret og gyroskpet dersom denne er klar.
 * @param   *imu peker til en ICM20948-struktur
 * @retval  ICM20948_StatusTypeDef IMU status
 */
ICM20948_StatusTypeDef ICM20948_Read_Data_Poll(ICM20948 *imu)
{
    ICM20948_StatusTypeDef retval = ICM20948_OK;
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};
    retval = ICM20948_Read_Register(imu, ICM20948_DATA_RDY_STATUS, tx_buf, rx_buf);
    if (retval != ICM20948_OK) return retval;

    if (rx_buf[1] & 0x01U)
    {
        retval = ICM20948_Read_Gyro(imu);
        if (retval != ICM20948_OK) return retval;
    }

    if (rx_buf[1] & 0x03U)
    {
        retval = ICM20948_Read_Accel(imu);
        if (retval != ICM20948_OK) return retval;
    }

    return retval;
}



