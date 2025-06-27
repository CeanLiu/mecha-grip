#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "math.h"
#include "mpu6050.h"

#define SENSOR_COUNT 2

const int sda_pins[SENSOR_COUNT] = {21, 32};
const int scl_pins[SENSOR_COUNT] = {22, 33};
const uint8_t sensor_addresses[SENSOR_COUNT] = {0x68, 0x69};
i2c_port_t i2c_ports[SENSOR_COUNT] = {I2C_NUM_0, I2C_NUM_1};

int16_t ax_offset[SENSOR_COUNT], ay_offset[SENSOR_COUNT], az_offset[SENSOR_COUNT];
float pitch[SENSOR_COUNT] = {0}, roll[SENSOR_COUNT] = {0}; // store previous values

void i2c_master_init(i2c_port_t port, int sda, int scl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(port, &conf);
    i2c_driver_install(port, conf.mode, 0, 0, 0);
}

void calibrate_sensor(int index) {
    int16_t ax, ay, az;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    printf("Calibrating Sensor %d...\n", index + 1);
    for (int i = 0; i < 100; i++) {
        if (mpu6050_read_accel(i2c_ports[index], sensor_addresses[index], &ax, &ay, &az) == ESP_OK) {
            sum_x += ax;
            sum_y += ay;
            sum_z += az;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ax_offset[index] = sum_x / 100;
    ay_offset[index] = sum_y / 100;
    az_offset[index] = sum_z / 100;
}

void app_main(void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        i2c_master_init(i2c_ports[i], sda_pins[i], scl_pins[i]);
        mpu6050_init(i2c_ports[i], sensor_addresses[i]);
    }

    printf("Press Enter to calibrate sensors...\n");
    getchar();
    for (int i = 0; i < SENSOR_COUNT; i++) {
        calibrate_sensor(i);
    }

    const float alpha = 0.98;
    const float dt = 0.05;  // 50 ms

    while (1) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;

            if (mpu6050_read_accel(i2c_ports[i], sensor_addresses[i], &ax, &ay, &az) == ESP_OK &&
                mpu6050_read_gyro(i2c_ports[i], sensor_addresses[i], &gx, &gy, &gz) == ESP_OK) {

                ax -= ax_offset[i];
                ay -= ay_offset[i];
                az -= az_offset[i];

                float accel_pitch = atan2(ay, az) * 180.0 / M_PI;
                float accel_roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

                // Gyro sensitivity is 131 LSB/(°/s) for default config
                float gyro_pitch_rate = gx / 131.0;
                float gyro_roll_rate = gy / 131.0;

                pitch[i] = alpha * (pitch[i] + gyro_pitch_rate * dt) + (1 - alpha) * accel_pitch;
                roll[i]  = alpha * (roll[i]  + gyro_roll_rate  * dt) + (1 - alpha) * accel_roll;

                printf("Sensor %d => Pitch: %.2f, Roll: %.2f\n", i + 1, pitch[i], roll[i]);
            } else {
                printf("Sensor %d => Failed to read data\n", i + 1);
            }
        }

        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(dt * 1000));  // 50 ms
    }
}
