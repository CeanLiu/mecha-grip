// =================== mpu6050.h ===================
#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU6050_ADDR_LOW  0x68  // AD0 low
#define MPU6050_ADDR_HIGH 0x69  // AD0 high

esp_err_t mpu6050_init(i2c_port_t i2c_num, uint8_t addr);
esp_err_t mpu6050_read_accel(i2c_port_t i2c_num, uint8_t addr, int16_t *ax, int16_t *ay, int16_t *az);
esp_err_t mpu6050_read_gyro(i2c_port_t i2c_num, uint8_t addr, int16_t *gx, int16_t *gy, int16_t *gz);

#endif

