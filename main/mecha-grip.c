#include "my_servo.h"
#include "my_imu.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 33
#define SCL_GPIO 32

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void app_main(void) {
    i2c_master_init();

    my_imu_init(I2C_PORT);
    my_servo_init();  // your servo init function

    while (1) {
        my_imu_update();

        float roll, pitch;
        my_imu_get_angles(&roll, &pitch);

        printf("Roll: %.2f, Pitch: %.2f\n", roll, pitch);

        // Map IMU angles to servo angles and set servo
        // Example (simple clamp + map):
        int servo0_angle = (int)(pitch + 90);  // pitch (-90 to +90) mapped to (0 to 180)
        int servo1_angle = (int)(roll + 90);   // roll mapped similarly

        if (servo0_angle < 0) servo0_angle = 0;
        if (servo0_angle > 180) servo0_angle = 180;

        if (servo1_angle < 0) servo1_angle = 0;
        if (servo1_angle > 180) servo1_angle = 180;

        set_servo_angle(SERVO_0, servo0_angle);
        set_servo_angle(SERVO_1, servo1_angle);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
