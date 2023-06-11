
#include "MS5837.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include <stdint.h>


MS5837_StatusTypeDef MS5837_Init(MS5837 *sensor, I2C_HandleTypeDef *hi2c, vanntype *vanntyp)
{
	MS5837_StatusTypeDef retval = MS5837_OK;
	sensor->hi2c = hi2c;
	uint8_t var = 0x1EU;
	sensor->error = 0;

	//SET PROM
	retval =(MS5837_StatusTypeDef) HAL_I2C_Master_Transmit(sensor->hi2c,  (0x76<<1), &var, 1, 1000);


	//SETT VANNTYPE
	if(vanntyp = FERSKVANN) 
    {
		(sensor->rho) = 997.0474;
	}
	else if (vanntyp = SALTVANN)
    { //SALTVANN
		(sensor->rho) = 1023.6;
	}
	if (retval != MS5837_OK) {sensor->error = 1; }

    return retval;
}

MS5837_StatusTypeDef MS5837_Read_PROMS(MS5837*sensor)
{
	MS5837_StatusTypeDef retval = MS5837_OK;
	uint8_t recive_buf[2];

	//retval = MS5837_Read_Regsiter(sensor,PROM_C1_PRESS_SENS, &recive_buf_C1[2]);
	//retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5803_empty, PROM_C1_PRESS_SENS ,I2C_MEMADD_SIZE_8BIT, recive_buf, 2, 5000);
	//HAL_I2C_Master_Transmit(sensor->hi2c, MS5803_empty, PROM_C1_PRESS_SENS, 2, 1000);
	//HAL_I2C_Master_Receive(sensor->hi2c, MS5803_empty, recive_buf, 2, 1000);
	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C1_PRESS_SENS ,1, recive_buf, 2, 1000);
	//if (retval != MS5837_OK) return retval;
	sensor->C1_SENS = recive_buf[0] << 8 | recive_buf[1];


	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C2_PRESS_OFFSET ,1, recive_buf, 2, 1000);
	//if (retval != MS5837_OK) return retval;
	sensor->C2_OFF = recive_buf[0] << 8 | recive_buf[1];

	delay_ms(20);

	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C3_TEMP_COFF_FOR_PRESS_SENS ,1, recive_buf, 2, 1000);
	//if (retval != MS5837_OK) return retval;
	sensor->C3_TCS = recive_buf[0] << 8 | recive_buf[1];
	delay_ms(20);

	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C4_TEMP_COFF_FOR_PRESS_OFF ,1, recive_buf, 2, 1000);
	//if (retval != MS5837_OK) return retval;
	sensor->C4_TCO = recive_buf[0] << 8 | recive_buf[1];
	delay_ms(20);

	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C5_TEMP_REF ,1, recive_buf, 2, 1000);
	//if (retval != MS5837_OK) return retval;
	sensor->C5_TREF = recive_buf[0] << 8 | recive_buf[1];
	delay_ms(20);

	retval =(MS5837_StatusTypeDef)  HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, PROM_C6_TEMP_SENS ,1, recive_buf, 2, 1000);

	sensor->C6_TEMPSENS = recive_buf[0] << 8 | recive_buf[1];
	if (retval != MS5837_OK) {sensor->error = 1; }return retval;

}


MS5837_StatusTypeDef MS5837_Read_Pressure(MS5837 *sensor)
{
	MS5837_StatusTypeDef retval = MS5837_OK;
	uint8_t recive_buf[3];

	 uint8_t  D1_pressure_256 = 0X40u;   /* actual variable declaration */
	 //uint8_t  * D1_pressure_256;        /* pointer variable declaration */
	 //D1_pressure_256 = &var;  /* store address of var in pointer variable*/


	retval =(MS5837_StatusTypeDef) HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_EMPTY, &D1_pressure_256, 1, 1000);
	delay_ms(3);
	//delay_ns(5000);
	retval = (MS5837_StatusTypeDef) HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY,MS5837_ADC_READ ,1, recive_buf, 3, 2000);

	sensor->D1_pressure = recive_buf[0] << 16 | recive_buf[1] << 8 | recive_buf[2];
	if (retval != MS5837_OK) {sensor->error = 1; }return retval;
}

MS5837_StatusTypeDef MS5837_Read_Water_Temperature(MS5837 *sensor)
{
	MS5837_StatusTypeDef retval = MS5837_OK;
	uint8_t recive_buf[3];

	 uint8_t D2_256_TEMP = 0x50U;   /* actual variable declaration */
	 //uint8_t  * D1_pressure_256;        /* pointer variable declaration */
	 //D1_pressure_256 = &var;  /* store address of var in pointer variable*/


	retval =(MS5837_StatusTypeDef) HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_EMPTY, &D2_256_TEMP, 1, 1000);
	//delay_ms(400);
	//conversion ved OSR=256  , 9 bit = 0.6ms
	delay_ms(3);
	//delay_ns(5000);
	//delay_ns(10000);
	retval = (MS5837_StatusTypeDef) HAL_I2C_Mem_Read(sensor->hi2c, MS5837_EMPTY, MS5837_ADC_READ, 1, recive_buf, 3, 1000);

	sensor->D2_temp = recive_buf[0] << 16 | recive_buf[1] << 8 | recive_buf[2];
	if (retval != MS5837_OK) {sensor->error = 1; }return retval;
}

float MS5837_beregne_trykk_og_temp(MS5837 *sensor){

	float T2;
	float OFF2;
	float SENS2;
	float P_bar;

	//EXEMPEL for kontroll av matte, fra datablad
	//sensor->C1_SENS =0x4979;
	//sensor->C2_OFF =0x5c57;
	//sensor ->C3_TCS=0x76d1;
	//sensor ->C4_TCO=0x3C63;
	//sensor ->C5_TREF=0x75D3;
	//sensor->C6_TEMPSENS=0x67E7;

	//sensor->D1_pressure= 0X6EAC27;
	//sensor->D2_temp=0x75975E;

//dT - difference between actual and ref temp
	(sensor-> dT)	=	(sensor-> D2_temp)-(sensor->C5_TREF)*(pow(2,8));

	//TEMP - tempratur celcius
	(sensor ->TEMP)	=(2000+(sensor->dT)*(sensor->C6_TEMPSENS)/(pow(2,23)));

	//OFFSETT pressure offsett at certain temperatur
	(sensor->OFF) = ((sensor->C2_OFF)*(pow(2,19))+((sensor->dT)*(sensor->C4_TCO))/(pow(2,4)));

	//SENSitivty at actual temperature

	(sensor -> SENS)	= (((sensor->C1_SENS)+30000)*(pow(2,17))+(((sensor->dT)*(sensor->C3_TCS))/(pow(2,6))));

	//

	// temp below 20 c and over -15 ? ,

	T2 = 0;
	OFF2 =0;
	SENS2 =0;


	if (sensor->TEMP < 2000){
		T2 =(float) 9*pow(sensor->dT,2)/(pow(2,34));
		OFF2 = (float)4*pow((sensor->TEMP)-2000,2)/pow(2,18);
		SENS2 = (float) 2*pow((sensor->TEMP)-2000,2)/pow(2,38);
	}


	sensor->TEMP =sensor->TEMP -T2;
	sensor->OFF =sensor->OFF -OFF2;
	sensor->SENS =sensor->SENS-SENS2;

	//P ,temperature compensated pressure (var 24 bit , blir nå 32 bit )
	sensor->P_mbar = ((((sensor ->D1_pressure)*((sensor ->SENS)/(pow(2,21)))-(sensor ->OFF))/(pow(2,15))));
    sensor->vanntemp = (int16_t) (sensor->TEMP);

}

float MS5837_nullstill_trykk(MS5837 *sensor){

	int values = 5;
	int my_array[values];
	int sum = 0; // create an array of size 5

	for (int i = 0; i < values; i++) {  // loop through the indices of the array
	    MS5837_Read_Pressure(sensor);
	    my_array[i] =(sensor->D1_pressure);

	    //henter ut max verdien fra initaliseringen
	    if (my_array[i-1] < my_array[i]) {
	    	MS5837_beregne_trykk_og_temp(sensor);
	    	(sensor ->P_mbar_0_max) = (sensor->P_mbar);
	    }

	    sum += my_array[i]; // add the current number to the array
	}
	float average = (float)sum / values; // calculate the average
	(sensor->D1_pressure) = (uint32_t) round(average);  // transform to uint32_t



	MS5837_Read_Water_Temperature(sensor);
	MS5837_beregne_trykk_og_temp(sensor);



	(sensor ->P_mbar_0) = (sensor->P_mbar);

}

float MS5837_mbar_til_cm(MS5837 *sensor){
	MS5837_StatusTypeDef retval ;



	//0 = ferskvann
	//1 = saltvann
	//ved havoverflaten er det 1.013 bar 111 metres at 15 °C
	float mbar_delta;
	float cm_delta;
	float mbar_delta_iir;
	float cm_delta_iir;

	float tyngde_akks=9.806;
	//float bar_offset = 1.013 ; // for 115 meter over havet


	//ekempel måling
	//0x16b92 = 930 74 mbar
	//0xD6D8=  550 00 mbar
	//DETTE burde gi svare t , 5480 cm
	//(sensor->P_mbar) = 0x16b92 + 0xD6D8;


	//mbar til cm uten iir
	mbar_delta = (sensor->P_mbar)- (sensor->P_mbar_0);
	if (mbar_delta < 0) {
			mbar_delta = 0;
		    }
	//nullstiller basert på gjennomsnittet av målinger
	//mbar_delta = (sensor->P_mbar)- (sensor->P_mbar_0_max); // nullstiller basert på max verdien av de 20 målingene
	cm_delta = (mbar_delta/((sensor->rho)*tyngde_akks))*1000;
	(sensor -> cm_dybde) = (int16_t) round(cm_delta);



	//IIR filter mbar til cm
	/*
	mbar_delta_iir = (sensor->P_mbar_iir)- (sensor->P_mbar_0);
	if (mbar_delta_iir < 0) {
			mbar_delta_iir = 0;
		    }
	cm_delta_iir = (mbar_delta_iir/((sensor->rho)*tyngde_akks))*1000;
	*/
	// (sensor -> cm_dybde_iir) = (int16_t) round(cm_delta_iir);

}

float 	MS5837_IIR_filter(MS5837 *sensor){

	float siste_P_mbar =sensor->P_mbar;
	float siste_temp =sensor->TEMP;


	static float forrige_P_mbar_iir = 0;
	static float forrige_temp = 0;

	//sett parametere til iir filter
	double a =0.5, b=0.5;//vekting av første og forrige for trykk og temp



	if (forrige_P_mbar_iir > 0){ //start iir filter når vi har 2 trykk målinger

		//TRYKK IIR FILTER
		(sensor->P_mbar_iir)= (int32_t) round(a*siste_P_mbar +b*forrige_P_mbar_iir);

		//y_mbar[n] = b * x[n] + a * y[n-1]

		//TEMP IIR FILTER
		(sensor->TEMP)= (int32_t) round(a*siste_temp +b*forrige_temp);

		}else { //ved første måling settes iir utgangsverdien til siste mbar avlesning
			(sensor->P_mbar_iir) = sensor->P_mbar;
		}


	//etter iir filter er gjort , settes siste til forrige
	forrige_P_mbar_iir =sensor->P_mbar_iir;
	forrige_temp =sensor->TEMP;

}

MS5837_StatusTypeDef MS5837_Dybde(MS5837 *sensor)
{

	// TRYKKSENSOR //
	volatile uint32_t tick = HAL_GetTick();
	volatile MS5837_StatusTypeDef retval;
	retval = MS5837_Read_Water_Temperature(sensor);
	if (retval != MS5837_OK) return retval;
	//delay_ms(1);
	retval = MS5837_Read_Pressure(sensor);
	if (retval != MS5837_OK) return retval;
	//delay_ms(1);
	MS5837_beregne_trykk_og_temp(sensor);

	//IR filter on
	//MS5837_IIR_filter(sensor);

	MS5837_mbar_til_cm(sensor);
	volatile tickval = HAL_GetTick() - tick;
	if (tick)
	{
		volatile uint8_t mau = 0;
	}
	//delay_ms(1000);
	return retval;
};


