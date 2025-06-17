#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_FREQ_HZ 100000

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define SERVO_PIN 18  // Use appropriate GPIO

// I2C Setup
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// MPU6050 Init
void mpu6050_init() {
    uint8_t wake_data = 0;
    i2c_master_write_to_device(I2C_MASTER_PORT, MPU6050_ADDR, &MPU6050_PWR_MGMT_1, 1, 1000 / portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_MASTER_PORT, MPU6050_ADDR, &wake_data, 1, 1000 / portTICK_PERIOD_MS);
}

// Read raw accelerometer data
void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    i2c_master_write_read_device(I2C_MASTER_PORT, MPU6050_ADDR, &reg, 1, data, 6, 1000 / portTICK_PERIOD_MS);

    *ax = (data[0] << 8) | data[1];
    *ay = (data[2] << 8) | data[3];
    *az = (data[4] << 8) | data[5];
}

// LEDC Servo PWM Setup
void servo_pwm_init() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

// Convert angle (0–180) to LEDC duty
uint32_t angle_to_duty(float angle_deg) {
    angle_deg = fmax(0, fmin(180, angle_deg));
    float pulse_ms = 0.5 + (angle_deg / 180.0) * 2.0;  // 0.5ms–2.5ms
    return (uint32_t)((pulse_ms / 20.0) * 65535);      // For 50Hz = 20ms period
}

void app_main() {
    i2c_master_init();
    mpu6050_init();
    servo_pwm_init();

    int16_t ax, ay, az;

    while (1) {
        mpu6050_read_accel(&ax, &ay, &az);

        float ax_g = ax / 16384.0;
        float ay_g = ay / 16384.0;
        float az_g = az / 16384.0;

        float pitch = atan2(ay_g, az_g) * 180.0 / M_PI;
        pitch = fmax(0, fmin(180, pitch));  // Clamp to servo range

        uint32_t duty = angle_to_duty(pitch);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        printf("Pitch: %.2f°, Duty: %d\n", pitch, duty);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
