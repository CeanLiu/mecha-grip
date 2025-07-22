// my_imu.c - Dual MPU6050 IMU support
#include "my_imu.h"
#include "complementary_filter.h"
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDR0 0x68
#define MPU6050_ADDR1 0x69

static i2c_port_t s_i2c_num;
static i2c_master_dev_handle_t s_dev_handle_0;
static i2c_master_dev_handle_t s_dev_handle_1;

static QuaternionIMU imu_filters[2]; // One for each IMU

static float gyro_scale = 131.0f;
static float accel_scale = 16384.0f;

static esp_err_t init_single_mpu(i2c_master_bus_handle_t handle, uint8_t address, i2c_master_dev_handle_t* dev_handle, QuaternionIMU* filter) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) return ret;

    // Wake up device
    uint8_t wake_data[2] = {0x6B, 0x00};
    ret = i2c_master_transmit(*dev_handle, wake_data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    // Read gyro/accel config
    uint8_t gyro_reg = 0x1B, accel_reg = 0x1C;
    uint8_t gyro_config, accel_config;

    ret = i2c_master_transmit_receive(*dev_handle, &gyro_reg, 1, &gyro_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    ret = i2c_master_transmit_receive(*dev_handle, &accel_reg, 1, &accel_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    // Decode and set scaling
    uint8_t gyro_fs_sel = (gyro_config >> 3) & 0x03;
    uint8_t accel_fs_sel = (accel_config >> 3) & 0x03;
    gyro_scale = (float[]){131.0f, 65.5f, 32.8f, 16.4f}[gyro_fs_sel];
    accel_scale = (float[]){16384.0f, 8192.0f, 4096.0f, 2048.0f}[accel_fs_sel];

    init_quaternion_imu(filter, 0.98f);

    printf("MPU6050 (0x%02X): GYRO_CONFIG=0x%02X ACCEL_CONFIG=0x%02X\n", address, gyro_config, accel_config);
    return ESP_OK;
}

esp_err_t my_imu_init(i2c_port_t i2c_num) {
    s_i2c_num = i2c_num;
    i2c_master_bus_handle_t handle;
    esp_err_t ret = i2c_master_get_bus_handle(i2c_num, &handle);
    if (ret != ESP_OK) return ret;

    // Init both IMUs
    ret = init_single_mpu(handle, MPU6050_ADDR0, &s_dev_handle_0, &imu_filters[0]);
    if (ret != ESP_OK) return ret;

    ret = init_single_mpu(handle, MPU6050_ADDR1, &s_dev_handle_1, &imu_filters[1]);
    return ret;
}

static void extract_accel_data(const uint8_t* buf, float accel[3]) {
    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    accel[0] = (ax / accel_scale) * 9.81f;
    accel[1] = (ay / accel_scale) * 9.81f;
    accel[2] = (az / accel_scale) * 9.81f;
}

static void extract_gyro_data(const uint8_t* buf, float gyro[3]) {
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];
    gyro[0] = (gx / gyro_scale) * M_PI / 180.0f;
    gyro[1] = (gy / gyro_scale) * M_PI / 180.0f;
    gyro[2] = (gz / gyro_scale) * M_PI / 180.0f;
}

esp_err_t my_imu_update(float dt) {
    i2c_master_dev_handle_t handles[] = {s_dev_handle_0, s_dev_handle_1};
    for (int i = 0; i < 2; ++i) {
        uint8_t reg = 0x3B;
        uint8_t buf[14];
        esp_err_t ret = i2c_master_transmit_receive(handles[i], &reg, 1, buf, 14, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            printf("MPU6050[%d] update failed: %s\n", i, esp_err_to_name(ret));
            return ret;
        }

        float accel[3], gyro[3];
        extract_accel_data(buf, accel);
        extract_gyro_data(buf, gyro);
        update_quaternion_imu(&imu_filters[i], accel, gyro, dt);
    }
    return ESP_OK;
}

// Accessors for each IMU
float my_imu_get_roll(int index) { return quat_to_roll(imu_filters[index].orientation); }
float my_imu_get_pitch(int index){ return quat_to_pitch(imu_filters[index].orientation); }
float my_imu_get_yaw(int index)  { return quat_to_yaw(imu_filters[index].orientation); }
