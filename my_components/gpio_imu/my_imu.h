// my_imu.h
#ifndef MY_IMU_H
#define MY_IMU_H

#include "driver/i2c.h"

void my_imu_init(i2c_port_t i2c_num);
void my_imu_update(void);
void my_imu_get_angles(float *roll, float *pitch);

#endif
