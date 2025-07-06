// my_imu.h
#ifndef MY_IMU_H
#define MY_IMU_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

esp_err_t my_imu_init(i2c_port_t i2c_num);
esp_err_t my_imu_update(float update_rate);
float my_imu_get_pitch(int index); 
float my_imu_get_yaw(int index); 
float my_imu_get_roll(int index); 
#endif
