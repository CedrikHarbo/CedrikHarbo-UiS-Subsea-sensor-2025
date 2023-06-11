

#include <math.h>
#include "cordic.h"
#include "stm32g431xx.h"

/**
 * @brief   Initialiser CORDIC moudulen
 *
 * @note    Presisjonen blir satt til 8 iterasjoner. Størrelsen på argument, og resultat
 *          blir satt til 16-bit. 
 *
 * @retval  Ingen
 */
void CORDIC_Init()
{
    RCC->AHB1ENR |= CORDIC_CLK_EN;
    // Set presition to 8, ARGSIZE and RESIZE to 16bit
    CORDIC->CSR |= (CORDIC_CSR_PRECISION_1 | CORDIC_CSR_RESSIZE | CORDIC_CSR_ARGSIZE);
}

/**
 * @brief   Regn ut kvadratrot vhja. CORDIC
 *
 * @param   float x
 * @retval  float sqrt(x)
 *
 * @note    Cordic støtter kun x <= 2.341. Dersom argumentet er større enn dette,
 *          vil sqrt funksjonen fra <math.h> benyttes.
 */
float CORDIC_Sqrt(float x)
{
    uint32_t scale;
    if (x <= 0.75) scale = 0;
    else if (x <= 1.75) scale = 1;
    else if (x <= 2.341) scale = 2;
    else return sqrt(x); // Numbers bigger than 2.341 are not supported
    
    //x_fp = (int16_t) (x * (1<< 15))
    int16_t x_q15 = (int16_t)((x / (1 << scale)) * (1 << 15));
    CORDIC->CSR = (CORDIC->CSR & CORDIC_CLEAR_FN_MSK & CORDIC_CLEAR_SCALE_MSK) | CORDIC_SQRT | (scale << 8);
    

    CORDIC->WDATA = x_q15;
    while (!(CORDIC->CSR & CORDIC_RDATA_RES))
    {
    }
    int16_t res_q15 = (int16_t)(CORDIC->RDATA);
    float res = ((float)res_q15 / (1 << 15)) * (1 << scale);

    return res;
}


float CORDIC_Atan2f(float x, float y)
{
	if (x >= 1 || x < -1 || y >= 1 || y <= -1)
	{
		return (atan2f(y, x) / M_PI) * 180;
	}
    
    //int32_t arg_q15 = ((int32_t)(x * (1 << 15))) | (int32_t)(y * (1 << 15)) << 15 ;
    int32_t arg_q15 = ((int32_t)(x * (1 << 15))) | ((int32_t)(y * (1 << 15)) << 16);

    // int32_t y_q15 = (int32_t)(y * (1 << 15)) << 15;
    CORDIC->CSR = (CORDIC->CSR & CORDIC_CLEAR_FN_MSK & CORDIC_CLEAR_SCALE_MSK) | CORDIC_PHASE;

    CORDIC->WDATA = arg_q15;

    while (!(CORDIC->CSR & CORDIC_RDATA_RES))
    {
    }
    int16_t res_q15 = (int16_t)(CORDIC->RDATA & 0xFFFF);

    float res = ((float)res_q15 / (1 << 15)) * 180;
    
    return res;
}


