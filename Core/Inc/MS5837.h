#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"


#ifndef INC_MS5837_H_
#define INC_MS5837_H_


#define MS5837_EMPTY (0x76U << 1)
//#define MS5803_empty (0x77 << 1 | 0x01)

#define MS5837_Write (0xECU << 1)//1110 1100
#define MS5837_Read (0xEDU << 1) //1110 1101
// ox76 = 0111 0110

#define MS5837_ADC_READ 0x00U
#define MS5837_RESET 0x1EU
#define D1_256_PRESSURE 0x40U
//#define D2_256_TEMP 0x50U

typedef enum
{
    PROM_CRC = 0xA0,
    PROM_C1_PRESS_SENS =0xA2, // 1010 0010
    PROM_C2_PRESS_OFFSET =0xA4,
    PROM_C3_TEMP_COFF_FOR_PRESS_SENS = 0xA6,
    PROM_C4_TEMP_COFF_FOR_PRESS_OFF = 0xA8,
    PROM_C5_TEMP_REF = 0xAA,
    PROM_C6_TEMP_SENS = 0xAC,

} MS5837_PROM_REG;


typedef enum
{
//0 = ferskvann
//1 = saltvann
    FERSKVANN = 0x00,
    SALTVANN = 0x01,

} vanntype;

typedef struct
{
     I2C_HandleTypeDef *hi2c;
    //int16_t temp_raw;
    //float temp_degc;
	//uint16_t C1_SENS=16595;
	uint16_t C1_SENS;
	uint16_t C2_OFF;
	uint16_t C3_TCS;
	uint16_t C4_TCO;
	uint16_t C5_TREF;
	uint16_t C6_TEMPSENS;
	uint32_t D1_pressure;

	uint32_t D2_temp;

	int32_t dT;
	int32_t TEMP;

	int64_t OFF;
	int64_t SENS;
	int32_t P_mbar;

	int32_t P_mbar_iir;

	int32_t P_mbar_0;
	int32_t P_mbar_0_max;

	float bar;
	float mbar;
	int16_t cm_dybde;
	int16_t cm_dybde_iir;

	int16_t vanntemp;
	float rho;
	uint8_t error;

} MS5837;


typedef enum
{
	MS5837_OK             = 0x00U,
	MS5837_HAL_ERROR      = 0x01U,
	MS5837_HAL_BUSY       = 0x02U,
	MS5837_HAL_TIMEOUT    = 0x03U,
} MS5837_StatusTypeDef;


MS5837_StatusTypeDef MS5837_Read_Water_Temperature(MS5837 *sensor);
MS5837_StatusTypeDef MS5837_Read_Pressure(MS5837 *sensor);
MS5837_StatusTypeDef MS5837_Read_PROMS(MS5837*sensor);
MS5837_StatusTypeDef MS5837_check_connection(MS5837 *sensor);
//void MS5837_Init(MS5837 *sensor, I2C_HandleTypeDef *hi2c);
float MS5837_beregne_trykk_og_temp(MS5837 *sensor);
MS5837_StatusTypeDef MS5837_Read_Pressure2(MS5837 *sensor, uint8_t *data);
float MS5837_MS5837_mbar_til_cm(MS5837 *sensor);
float MS5837_nullstill_trykk(MS5837 *sensor);
MS5837_StatusTypeDef MS5837_Init(MS5837 *sensor, I2C_HandleTypeDef *hi2c,vanntype *vanntyp);
float MS5837_IIR_filter(MS5837 *sensor);
MS5837_StatusTypeDef MS5837_Dybde(MS5837 *sensor);

#endif /* INC_MS5837_H_ */
