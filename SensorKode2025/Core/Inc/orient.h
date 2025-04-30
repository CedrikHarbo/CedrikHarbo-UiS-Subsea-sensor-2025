/*
 * orient.h
 *
 *  Created on: Apr 2, 2025
 *      Author: Martin
 */

#ifndef INC_ORIENT_H_
#define INC_ORIENT_H_

#include "ICM20948.h"


struct orientation
{
    float roll_deg;
    float pitch_deg;
    float accel_roll;
    float gyro_roll;
    float accel_pitch;
    float gyro_pitch;
};


void compute_orientation(ICM20948 *imu, struct orientation *orient);
void comp_filter(struct orientation *orient, float alpha);
void IMU_cal(ICM20948 *imu, uint32_t n_samples);

#endif /* INC_ORIENT_H_ */
