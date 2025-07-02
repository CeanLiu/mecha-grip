// my_imu.c - Improved version with better responsiveness
#include "my_imu.h"
#include "complementary_filter.h"
#include <stdio.h>
#include <math.h>

// Create one global IMU filter instance
static QuaternionIMU imu_filter;

#define MPU6050_ADDR 0x68 

static i2c_port_t s_i2c_num;
static i2c_master_dev_handle_t s_dev_handle;

// Scaling factors based on configured ranges
static float gyro_scale = 131.0f;    // For ±250°/s range
static float accel_scale = 16384.0f; // For ±2g range

esp_err_t my_imu_init(i2c_port_t i2c_num) {
    s_i2c_num = i2c_num;  // Save the port number
    
    // This means 90% gyro, 10% accel - more responsive to accel changes
    init_quaternion_imu(&imu_filter, 0.98f);  

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_bus_handle_t handle;
    esp_err_t ret = i2c_master_get_bus_handle(i2c_num, &handle);
    if (ret != ESP_OK) {
        printf("Failed to get I2C bus handle: %s\n", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_bus_add_device(handle, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        printf("Failed to add I2C device: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Wake up MPU6050
    uint8_t wake_data[2] = {0x6B, 0x00};
    ret = i2c_master_transmit(s_dev_handle, wake_data, sizeof(wake_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to initialize MPU6050: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // IMPROVED: Configure DLPF (Digital Low Pass Filter) for better noise reduction
    uint8_t dlpf_config_data[2] = {0x1A, 0x03}; // DLPF_CFG = 3 (44Hz bandwidth)
    ret = i2c_master_transmit(s_dev_handle, dlpf_config_data, sizeof(dlpf_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to configure DLPF: %s\n", esp_err_to_name(ret));
    }

    // Configure gyroscope range to ±250°/s (default, but let's be explicit)
    uint8_t gyro_config_data[2] = {0x1B, 0x00}; // FS_SEL = 0 (±250°/s)
    ret = i2c_master_transmit(s_dev_handle, gyro_config_data, sizeof(gyro_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to configure gyroscope: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Configure accelerometer range to ±2g (default, but let's be explicit)
    uint8_t accel_config_data[2] = {0x1C, 0x00}; // AFS_SEL = 0 (±2g)
    ret = i2c_master_transmit(s_dev_handle, accel_config_data, sizeof(accel_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to configure accelerometer: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Verify configuration
    uint8_t gyro_config, accel_config;
    uint8_t gyro_reg = 0x1B;
    uint8_t accel_reg = 0x1C;

    // Read GYRO_CONFIG
    ret = i2c_master_transmit_receive(s_dev_handle, &gyro_reg, 1, &gyro_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to read gyro config: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Read ACCEL_CONFIG
    ret = i2c_master_transmit_receive(s_dev_handle, &accel_reg, 1, &accel_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to read accel config: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Decode and set scaling factors
    uint8_t gyro_fs_sel = (gyro_config >> 3) & 0x03;
    uint8_t accel_fs_sel = (accel_config >> 3) & 0x03;

    // Set gyro scaling factor based on configured range
    const float gyro_scales[] = {131.0f, 65.5f, 32.8f, 16.4f}; // For ±250, ±500, ±1000, ±2000 °/s
    const char *gyro_ranges[] = {"±250°/s", "±500°/s", "±1000°/s", "±2000°/s"};
    gyro_scale = gyro_scales[gyro_fs_sel];

    // Set accel scaling factor based on configured range
    const float accel_scales[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f}; // For ±2, ±4, ±8, ±16 g
    const char *accel_ranges[] = {"±2g", "±4g", "±8g", "±16g"};
    accel_scale = accel_scales[accel_fs_sel];

    printf("GYRO_CONFIG: 0x%02X → Range: %s (Scale: %.1f)\n", gyro_config, gyro_ranges[gyro_fs_sel], gyro_scale);
    printf("ACCEL_CONFIG: 0x%02X → Range: %s (Scale: %.0f)\n", accel_config, accel_ranges[accel_fs_sel], accel_scale);
    
    return ESP_OK;
}

static void extract_accel_data(const uint8_t* read_buffer, float accel[3]) {
    // Extract raw 16-bit accelerometer values (bytes 0-5)
    int16_t raw_accel_x = (read_buffer[0] << 8) | read_buffer[1];
    int16_t raw_accel_y = (read_buffer[2] << 8) | read_buffer[3];
    int16_t raw_accel_z = (read_buffer[4] << 8) | read_buffer[5];
    
    // Convert to m/s² (proper SI units)
    accel[0] = (raw_accel_x / accel_scale) * 9.81f;  // Convert g to m/s²
    accel[1] = (raw_accel_y / accel_scale) * 9.81f;
    accel[2] = (raw_accel_z / accel_scale) * 9.81f;
}

static void extract_gyro_data(const uint8_t* read_buffer, float gyro[3]) {
    // Extract raw 16-bit gyroscope values (bytes 8-13)
    int16_t raw_gyro_x = (read_buffer[8] << 8) | read_buffer[9];
    int16_t raw_gyro_y = (read_buffer[10] << 8) | read_buffer[11];
    int16_t raw_gyro_z = (read_buffer[12] << 8) | read_buffer[13];
    
    // Convert to rad/s (proper SI units for quaternion math)
    gyro[0] = (raw_gyro_x / gyro_scale) * M_PI / 180.0f;  // Convert °/s to rad/s
    gyro[1] = (raw_gyro_y / gyro_scale) * M_PI / 180.0f;
    gyro[2] = (raw_gyro_z / gyro_scale) * M_PI / 180.0f;
}

esp_err_t my_imu_update(float update_rate){
    // Read raw data from MPU6050
    uint8_t write_buffer = 0x3B;
    uint8_t read_buffer[14];
    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &write_buffer, 1, 
                                               read_buffer, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("Failed to update MPU6050: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Extract sensor data using helper functions
    float accel[3], gyro[3];
    extract_accel_data(read_buffer, accel);
    extract_gyro_data(read_buffer, gyro);

    // Update complementary filter
    update_quaternion_imu(&imu_filter, accel, gyro, update_rate);
    // Initialization logic
    return ESP_OK;
}

float my_imu_get_filtered_roll(void) {
    return quat_to_roll(imu_filter.orientation);
}

float my_imu_get_filtered_pitch(void) {
    return quat_to_pitch(imu_filter.orientation);
}

float my_imu_get_filtered_yaw(void) {
    return quat_to_yaw(imu_filter.orientation);
}