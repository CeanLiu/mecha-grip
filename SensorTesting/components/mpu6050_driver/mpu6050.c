
// =================== mpu6050.c ===================
#include "mpu6050.h"
#include <string.h>

#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

#define MPU6050_REG_GYRO_XOUT_H 0x43

esp_err_t mpu6050_read_gyro(i2c_port_t i2c_num, uint8_t addr, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = MPU6050_REG_GYRO_XOUT_H;
    uint8_t data[6];

    esp_err_t err = i2c_master_write_read_device(i2c_num, addr, &reg, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
        return err;

    *gx = (int16_t)(data[0] << 8 | data[1]);
    *gy = (int16_t)(data[2] << 8 | data[3]);
    *gz = (int16_t)(data[4] << 8 | data[5]);
    return ESP_OK;
}

esp_err_t mpu6050_init(i2c_port_t i2c_num, uint8_t addr)
{
    uint8_t buf[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    return i2c_master_write_to_device(i2c_num, addr, buf, 2, 1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_accel(i2c_port_t i2c_num, uint8_t addr, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[6];

    esp_err_t err = i2c_master_write_read_device(i2c_num, addr, &reg, 1, data, 6, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
        return err;

    *ax = (int16_t)(data[0] << 8 | data[1]);
    *ay = (int16_t)(data[2] << 8 | data[3]);
    *az = (int16_t)(data[4] << 8 | data[5]);
    return ESP_OK;
}
