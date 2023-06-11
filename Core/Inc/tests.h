
#ifndef TESTS_H_
#define TESTS_H_

#include <stdbool.h>
#include <stdint.h>
void IMU_com_test();
void cordic_test_sqrt(uint32_t *cordic_cycles, uint32_t *cycles_m, float num, float *cordic_res, float *res);
void cordic_test_atan2f(uint32_t *cordic_cycles, uint32_t *cycles_m, float x, float y, float *cordic_res, float *res);
extern bool TB_status;

#endif /* TESTS_H_ */
