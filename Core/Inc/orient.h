/**
 *
 * @file: orient.h
 *
 * @date: 28.02.23
 *
 * @author: Håvard Syslak
 *
 */

#ifndef __ORIENT_H
#define __ORIENT_H

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

#endif /* __ORIENT_H */
