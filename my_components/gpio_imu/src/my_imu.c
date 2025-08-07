
#include "my_imu.h"
#include "complementary_filter.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_IMUS 3
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_GYRO_CONFIG 0x1B



#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_DATA_START 0x3B

static float gyro_scale = 131.0f;
static float accel_scale = 16384.0f;

typedef struct {
    i2c_port_t port;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    QuaternionIMU filter;
} IMUDevice;

static IMUDevice imu_devices[MAX_IMUS];
static int imu_device_count = 0;

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

esp_err_t my_imu_add(i2c_port_t port, uint8_t address) {
    if (imu_device_count >= MAX_IMUS) return ESP_ERR_NO_MEM;

    i2c_master_bus_handle_t bus;
    esp_err_t ret = i2c_master_get_bus_handle(port, &bus);
    if (ret != ESP_OK) return ret;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev;
    ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) return ret;

    uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(dev, wake_cmd, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    uint8_t gyro_reg = MPU6050_REG_GYRO_CONFIG;
    uint8_t accel_reg = MPU6050_REG_ACCEL_CONFIG;
    uint8_t gyro_config, accel_config;

    ret = i2c_master_transmit_receive(dev, &gyro_reg, 1, &gyro_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    ret = i2c_master_transmit_receive(dev, &accel_reg, 1, &accel_config, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    uint8_t gyro_fs_sel = (gyro_config >> 3) & 0x03;
    uint8_t accel_fs_sel = (accel_config >> 3) & 0x03;
    gyro_scale = (float[]){131.0f, 65.5f, 32.8f, 16.4f}[gyro_fs_sel];
    accel_scale = (float[]){16384.0f, 8192.0f, 4096.0f, 2048.0f}[accel_fs_sel];

    imu_devices[imu_device_count].port = port;
    imu_devices[imu_device_count].bus = bus;
    imu_devices[imu_device_count].dev = dev;
    init_quaternion_imu(&imu_devices[imu_device_count].filter, 0.98f);

    printf("MPU6050 #%d (0x%02X on I2C_NUM_%d): GYRO=0x%02X ACCEL=0x%02X\n",
           imu_device_count, address, port, gyro_config, accel_config);

    imu_device_count++;
    return ESP_OK;
}

esp_err_t my_imu_update(float dt) {
    for (int i = 0; i < imu_device_count; ++i) {
        uint8_t reg = MPU6050_REG_DATA_START;
        uint8_t buf[14];
        esp_err_t ret = i2c_master_transmit_receive(
            imu_devices[i].dev, &reg, 1, buf, 14, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            printf("MPU6050[%d] update failed: %s\n", i, esp_err_to_name(ret));
            return ret;
        }

        float accel[3], gyro[3];
        extract_accel_data(buf, accel);
        extract_gyro_data(buf, gyro);
        update_quaternion_imu(&imu_devices[i].filter, accel, gyro, dt);
    }
    return ESP_OK;
}

// Replace these functions in your my_imu.c file
float my_imu_get_roll(int index) {
    if (index >= imu_device_count) return 0.0f;
    // Use independent roll calculation
    return quat_to_roll_independent(imu_devices[index].filter.orientation);
}

float my_imu_get_pitch(int index) {
    if (index >= imu_device_count) return 0.0f;
    // Use independent pitch calculation
    return quat_to_pitch_independent(imu_devices[index].filter.orientation);
}

float my_imu_get_yaw(int index) {
    if (index >= imu_device_count) return 0.0f;
    // Yaw calculation remains the same
    return quat_to_yaw(imu_devices[index].filter.orientation);
}

// ===== IMU ANGLE MAPPING FUNCTIONS =====

// Generic mapping function for any IMU angle to servo angle
static float map_imu_to_servo(float imu_angle, float center_angle, float servo_min, float servo_max, float imu_min, float imu_max) {
    // Calculate offset from center
    float offset = imu_angle - center_angle;
    
    // Normalize offset to servo range
    float servo_range = servo_max - servo_min;
    float imu_range = imu_max - imu_min;
    float normalized_offset = (offset / imu_range) * servo_range;
    
    // Calculate servo angle
    float servo_angle = (servo_min + servo_max) / 2.0f + normalized_offset;
    
    // Clamp to servo limits
    if (servo_angle < servo_min) servo_angle = servo_min;
    if (servo_angle > servo_max) servo_angle = servo_max;
    
    return servo_angle;
}

// Pitch mapping (typically -90 to +90 degrees)
float my_imu_map_pitch_to_servo(float pitch, float center_pitch, float servo_min, float servo_max) {
    return map_imu_to_servo(pitch, center_pitch, servo_min, servo_max, -90.0f, 90.0f);
}

// Roll mapping (can be -180 to +180 degrees)
float my_imu_map_roll_to_servo(float roll, float center_roll, float servo_min, float servo_max) {
    return map_imu_to_servo(roll, center_roll, servo_min, servo_max, -180.0f, 180.0f);
}

// Special function for roll that handles wrapping around ±180°
float my_imu_map_roll_wrapped_to_servo(float roll, float center_roll, float servo_min, float servo_max) {
    // Normalize roll to -180 to +180 range
    while (roll > 180.0f) roll -= 360.0f;
    while (roll < -180.0f) roll += 360.0f;
    
    // Calculate shortest angular distance from center
    float diff = roll - center_roll;
    
    // Handle wrap-around (shortest path)
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    
    // Map to servo range
    float servo_center = (servo_min + servo_max) / 2.0f;
    float servo_range = servo_max - servo_min;
    float angle_range = 360.0f; // Full rotation
    
    float servo_angle = servo_center + (diff / angle_range) * servo_range;
    
    // Clamp to servo limits
    if (servo_angle < servo_min) servo_angle = servo_min;
    if (servo_angle > servo_max) servo_angle = servo_max;
    
    return servo_angle;
}