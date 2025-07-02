#include "my_servo.h"
#include "my_imu.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 32
#define SCL_GPIO 33


#define IMU_TASK_STACK_SIZE 4096  // 4KB stack
static StackType_t imu_task_stack[IMU_TASK_STACK_SIZE];
static StaticTask_t imu_task_buffer;

void i2c_master_init() {
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,  // Use default clock source
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .glitch_ignore_cnt = 7,  // Filter out glitches shorter than 7 clock cycles
        .flags.enable_internal_pullup = true,  // Enable internal pull-ups
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
}

void imu_task(void *pvParameters) {
    printf("IMU Task started\n");
    ESP_ERROR_CHECK(my_imu_init(I2C_PORT));
    
    const float UPDATE_RATE_HZ = 50.0f;  // 50Hz update rate
    const float UPDATE_PERIOD_S = 1.0f / UPDATE_RATE_HZ;
    
    while (1) {
        ESP_ERROR_CHECK(my_imu_update(UPDATE_PERIOD_S));
        float pitch_angle = my_imu_get_filtered_pitch();
        float yaw_angle = my_imu_get_filtered_yaw();
        float roll_angle = my_imu_get_filtered_roll();
        printf("pitch angle: %.2f, yaw angle: %.2f, roll_angle: %.2f\n", pitch_angle, yaw_angle, roll_angle);

        int servo_angle = clampAngle(pitch_angle);
 
        vTaskDelay(pdMS_TO_TICKS(1000 / UPDATE_RATE_HZ));  // 20ms for 50Hz
    }
}

void wifi_client_task(void *pvParameters){
    printf("wifi client task started");
    
}

void app_main(void) {
    i2c_master_init();
    ESP_ERROR_CHECK(my_servo_init());
    xTaskCreateStatic( 
        imu_task,                    // Task function
        "IMU_Task",                  // Task name
        IMU_TASK_STACK_SIZE,         // Stack size in words
        NULL,                        // Task parameters
        5,                           // Priority (adjust as needed)
        imu_task_stack,              // Stack buffer
        &imu_task_buffer             // Task control block);
    );
}