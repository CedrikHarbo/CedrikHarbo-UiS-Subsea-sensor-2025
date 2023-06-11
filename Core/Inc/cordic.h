/** 
 * @file    cordic.h
 * @brief   Deklerasjoner for cordic driveren
 * @author  Håvard Syslak
 * @date    25.03.23
 */


#ifndef CORDIC_H_
#define CORDIC_H_


#include <stdint.h>

#define CORDIC_CLK_EN (1 << 3)
#define Q_15 32768.0f

/** 
 * @defgroup CORDIC_functions
 * @{
 */
#define CORDIC_CLEAR_FN_MSK 0xFFFFFFF0
#define CORDIC_CLEAR_SCALE_MSK 0xFFFFF8FF
#define CORDIC_COS 0x00
#define CORDIC_SIN 0x01
#define CORDIC_PHASE 0x02
#define CORDIC_MODULUS 0x03
#define CORDIC_ATAN 0x04
#define CORDIC_COSH 0x05
#define CORDIC_SINH 0x06
#define CORIDC_ATANH 0x07
#define CORIDC_LN 0x08
#define CORDIC_SQRT 0x09

/* @} */

void CORDIC_Init();
float CORDIC_Sqrt(float x);
float CORDIC_Atan2f(float x, float y);

#endif /* ifndef CORDIC_H_ */
