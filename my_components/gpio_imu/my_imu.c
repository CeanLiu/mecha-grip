// my_imu.c
#include "my_imu.h"
#include "mpu6050.h"
#include <stdio.h>

static mpu6050_handle_t sensor = NULL;
static complimentary_angle_t angle = {0};

void my_imu_init(i2c_port_t i2c_num) {
    sensor = mpu6050_create(i2c_num, MPU6050_I2C_ADDRESS);
    if (!sensor) {
        printf("Failed to create MPU6050 handle\n");
        return;
    }
    mpu6050_wake_up(sensor);
    mpu6050_config(sensor, ACCE_FS_2G, GYRO_FS_250DPS);
}

void my_imu_update(void) {
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    if (mpu6050_get_acce(sensor, &acce) == ESP_OK &&
        mpu6050_get_gyro(sensor, &gyro) == ESP_OK) {
        mpu6050_complimentory_filter(sensor, &acce, &gyro, &angle);
    }
}

void my_imu_get_angles(float *roll, float *pitch) {
    if (roll) *roll = angle.roll;
    if (pitch) *pitch = angle.pitch;
}
