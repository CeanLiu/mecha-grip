#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "mpu6050.h"

#define TAG             "MPU_SERVO"
#define SERVO_GPIO      GPIO_NUM_33
#define I2C_SCL         GPIO_NUM_13
#define I2C_SDA         GPIO_NUM_32
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     100000

mpu6050_handle_t mpu;

uint32_t angle_to_duty_cycle(uint8_t angle) {
    if (angle > 180) angle = 180;
    float pulse_width_ms = 0.5 + (angle / 180.0f) * 2.0f; // 0.5ms to 2.5ms
    return (uint32_t)((pulse_width_ms / 20.0f) * 4096.0f); // out of 20ms
}

// I2C Scanner function to check if device is detected
void i2c_scanner() {
    ESP_LOGI(TAG, "Starting I2C scanner...");
    bool found_any = false;
    
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", i);
            found_any = true;
        } else if (i == 0x68 || i == 0x69) {
            ESP_LOGW(TAG, "MPU6050 address 0x%02X not responding: %s", i, esp_err_to_name(ret));
        }
    }
    
    if (!found_any) {
        ESP_LOGE(TAG, "No I2C devices found! Check wiring:");
        ESP_LOGE(TAG, "  VCC -> 3.3V (NOT 5V!)");
        ESP_LOGE(TAG, "  GND -> GND");
        ESP_LOGE(TAG, "  SDA -> GPIO32");
        ESP_LOGE(TAG, "  SCL -> GPIO13");
        ESP_LOGE(TAG, "  Try swapping SDA/SCL wires");
        ESP_LOGE(TAG, "  Check for loose connections");
    }
    
    ESP_LOGI(TAG, "I2C scanner finished");
}

void i2c_master_init() {
    ESP_LOGI(TAG, "Initializing I2C...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Add delay and scan for devices
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_scanner();
}

void servo_init() {
    ESP_LOGI(TAG, "Initializing servo...");
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 50, // 50Hz for servo
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    ESP_LOGI(TAG, "Servo initialized successfully");
}

void Gesture_Task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting Gesture Task...");
    
    // Give I2C some time to settle
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test basic I2C communication first
    ESP_LOGI(TAG, "Testing I2C communication...");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t i2c_test = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_test != ESP_OK) {
        ESP_LOGE(TAG, "Direct I2C test failed: %s", esp_err_to_name(i2c_test));
        ESP_LOGE(TAG, "MPU6050 not responding at 0x%02X", MPU6050_I2C_ADDRESS);
        
        // Try alternative address
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x69 << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        i2c_test = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (i2c_test != ESP_OK) {
            ESP_LOGE(TAG, "Alternative address 0x69 also failed: %s", esp_err_to_name(i2c_test));
            ESP_LOGE(TAG, "HARDWARE PROBLEM - Check your wiring!");
            vTaskDelete(NULL);
            return;
        } else {
            ESP_LOGI(TAG, "Found MPU6050 at alternative address 0x69");
        }
    } else {
        ESP_LOGI(TAG, "MPU6050 responding at address 0x68");
    }
    
    ESP_LOGI(TAG, "Creating MPU6050 handle...");
    mpu = mpu6050_create(I2C_PORT, MPU6050_I2C_ADDRESS);
    if (mpu == NULL) {
        ESP_LOGE(TAG, "MPU6050 create failed! Trying alternative address...");
        
        mpu = mpu6050_create(I2C_PORT, 0x69);
        if (mpu == NULL) {
            ESP_LOGE(TAG, "MPU6050 failed with both addresses!");
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGI(TAG, "MPU6050 created with address 0x69");
    } else {
        ESP_LOGI(TAG, "MPU6050 created with address 0x68");
    }
    
    ESP_LOGI(TAG, "MPU6050 handle created successfully");
    
    esp_err_t ret = mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_250DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 config failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "This usually means the sensor is not properly connected or powered");
        mpu6050_delete(mpu);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "MPU6050 configured successfully");

    vTaskDelay(pdMS_TO_TICKS(1000)); // Let sensor stabilize
    ESP_LOGI(TAG, "MPU6050 stabilized and ready");

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t angle;

    int previous_angle = 90;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty_cycle(previous_angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "Servo initialized to 90 degrees");

    int loop_count = 0;
    while (1) {
        loop_count++;
        
        esp_err_t acce_ret = mpu6050_get_acce(mpu, &acce);
        esp_err_t gyro_ret = mpu6050_get_gyro(mpu, &gyro);
        
        if (acce_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get accelerometer data: %s", esp_err_to_name(acce_ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        if (gyro_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get gyroscope data: %s", esp_err_to_name(gyro_ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        mpu6050_complimentory_filter(mpu, &acce, &gyro, &angle);
        
        // More detailed logging every 10 loops to avoid spam
        if (loop_count % 10 == 0) {
            ESP_LOGI(TAG, "Raw Acce: [%.2f, %.2f, %.2f] | Raw Gyro: [%.2f, %.2f, %.2f]",
                     acce.acce_x, acce.acce_y, acce.acce_z,
                     gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            ESP_LOGI(TAG, "Filtered Angles - Roll: %.2f°, Pitch: %.2f°",
                     angle.roll, angle.pitch);
        }
        
        float pitch = angle.pitch;
        int current_angle = (int)(pitch + 90.0f);
        if (current_angle < 0) current_angle = 0;
        if (current_angle > 180) current_angle = 180;

        ESP_LOGI(TAG, "Pitch: %.2f° -> Servo: %d° → %d°", pitch, previous_angle, current_angle);

        if (abs(current_angle - previous_angle) > 2) { // Only move if significant change
            int step = (current_angle > previous_angle) ? 1 : -1;
            for (int i = previous_angle; i != current_angle; i += step) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle_to_duty_cycle(i));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            previous_angle = current_angle;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Faster updates for better debugging
    }

    mpu6050_delete(mpu);
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Starting MPU6050 + Servo Debug Version ===");
    ESP_LOGI(TAG, "Pin Configuration:");
    ESP_LOGI(TAG, "  I2C SDA: GPIO%d", I2C_SDA);
    ESP_LOGI(TAG, "  I2C SCL: GPIO%d", I2C_SCL);
    ESP_LOGI(TAG, "  Servo:   GPIO%d", SERVO_GPIO);

    i2c_master_init();
    servo_init();

    xTaskCreate(Gesture_Task, "Gesture_Task", 8192, NULL, 5, NULL); // Increased stack size
}