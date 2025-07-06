#include "my_imu.h"
#include "client.h"
#include "nvs_flash.h"
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

#define CLIENT_TASK_STACK_SIZE 4096
static StackType_t client_task_stack[CLIENT_TASK_STACK_SIZE];
static StaticTask_t client_task_buffer;


static float latest_pitch = 0.0f;
static float latest_yaw = 0.0f;
static SemaphoreHandle_t imu_mutex;

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
    const float UPDATE_RATE_HZ = 50.0f;
    const float UPDATE_PERIOD_S = 1.0f / UPDATE_RATE_HZ;

    while (1) {
        ESP_ERROR_CHECK(my_imu_update(UPDATE_PERIOD_S));
        float pitch_angle = my_imu_get_pitch(0);
        float yaw_angle   = my_imu_get_yaw(1);

        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(10))) {
            latest_pitch = pitch_angle;
            latest_yaw = yaw_angle;
            xSemaphoreGive(imu_mutex);
        }

        //int p_angle = clampAngle(pitch_angle);
        vTaskDelay(pdMS_TO_TICKS(1000 / UPDATE_RATE_HZ));
    }
}


void client_task(void *pvParameters) {
    printf("Wi-Fi client task started\n");
    ESP_ERROR_CHECK(init_wifi_sta());

    const TickType_t delay = pdMS_TO_TICKS(1000 / 5);  // Send at 5Hz
    while (1) {
        float pitch = 0.0f, yaw = 0.0f;

        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(10))) {
            pitch = latest_pitch;
            yaw   = latest_yaw;
            xSemaphoreGive(imu_mutex);
        }

        post_angles(pitch, yaw, 0.0f, 0.0f);
        vTaskDelay(delay);
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    i2c_master_init();
    imu_mutex = xSemaphoreCreateMutex();
    // xTaskCreateStatic( 
    //     imu_task,                    // Task function
    //     "IMU_Task",                  // Task name
    //     IMU_TASK_STACK_SIZE,         // Stack size in words
    //     NULL,                        // Task parameters
    //     5,                           // Priority (adjust as needed)
    //     imu_task_stack,              // Stack buffer
    //     &imu_task_buffer             // Task control block);
    // );
    // xTaskCreateStatic(
    //     client_task,
    //     "Client_Task",
    //     CLIENT_TASK_STACK_SIZE,
    //     NULL,
    //     5,
    //     client_task_stack,
    //     &client_task_buffer
    // );
}