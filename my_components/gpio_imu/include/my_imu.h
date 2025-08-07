#ifndef MY_IMU_H
#define MY_IMU_H

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

// Function declarations
esp_err_t my_imu_add(i2c_port_t port, uint8_t address);
esp_err_t my_imu_update(float dt);
float my_imu_get_pitch(int index); 
float my_imu_get_yaw(int index); 
float my_imu_get_roll(int index); 

// IMU angle mapping functions
float my_imu_map_roll_to_servo(float roll, float center_roll, float servo_min, float servo_max);
float my_imu_map_pitch_to_servo(float pitch, float center_pitch, float servo_min, float servo_max);
float my_imu_map_roll_wrapped_to_servo(float roll, float center_roll, float servo_min, float servo_max);
#endif // MY_IMU_H