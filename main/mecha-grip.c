#include "my_imu.h"
#include "my_servo.h"
#include "client.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// === I2C Configuration ===
#define I2C_PORT_0 I2C_NUM_0
#define SDA_GPIO_0 32
#define SCL_GPIO_0 33

#define I2C_PORT_1 I2C_NUM_1
#define SDA_GPIO_1 25
#define SCL_GPIO_1 26

// MPU6050 I2C addresses
#define MPU6050_ADDR_0 0x68
#define MPU6050_ADDR_1 0x69

i2c_master_bus_handle_t bus_handle_0;
i2c_master_bus_handle_t bus_handle_1;

// === Task Stack Configurations ===
#define IMU_TASK_STACK_SIZE 4096
static StackType_t imu_task_stack[IMU_TASK_STACK_SIZE];
static StaticTask_t imu_task_buffer;

#define CLIENT_TASK_STACK_SIZE 4096
static StackType_t client_task_stack[CLIENT_TASK_STACK_SIZE];
static StaticTask_t client_task_buffer;

// === Servo Angle Buffer and Mutex ===
static float servo_angles[5]; // S1-S5 servo angles
static SemaphoreHandle_t imu_mutex;

// === IMU CENTER POSITIONS (calibrate these for your arm) ===
// These are the angles when your arm is in "rest" position
#define WRIST_PITCH_CENTER    0.0f    // Adjust this to your wrist rest position
#define WRIST_ROLL_CENTER     0.0f    // Adjust this to your wrist rest position
#define ELBOW_PITCH_CENTER    45.0f   // Adjust this to your elbow rest position
#define SHOULDER_PITCH_CENTER 0.0f    // Adjust this to your shoulder rest position
#define SHOULDER_ROLL_CENTER  0.0f    // Adjust this to your shoulder rest position

// === I2C Master Initialization ===
void i2c_master_init() {
    // I2C_NUM_0 init
    i2c_master_bus_config_t conf0 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_0,
        .sda_io_num = SDA_GPIO_0,
        .scl_io_num = SCL_GPIO_0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf0, &bus_handle_0));

    // I2C_NUM_1 init
    i2c_master_bus_config_t conf1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_1,
        .sda_io_num = SDA_GPIO_1,
        .scl_io_num = SCL_GPIO_1,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf1, &bus_handle_1));
}

// === IMU Initialization ===
void init_imus() {
    // Add IMU #0: I2C_PORT_0, address 0x69 (wrist)
    ESP_ERROR_CHECK(my_imu_add(I2C_PORT_0, MPU6050_ADDR_1));
    
    // Add IMU #1: I2C_PORT_1, address 0x68 (elbow)
    ESP_ERROR_CHECK(my_imu_add(I2C_PORT_1, MPU6050_ADDR_0));
    
    // Add IMU #2: I2C_PORT_0, address 0x68 (shoulder)
    ESP_ERROR_CHECK(my_imu_add(I2C_PORT_0, MPU6050_ADDR_0));
    
    printf("Initialized 3 IMUs successfully\n");
}

// === IMU Task ===
void imu_task(void *pvParameters) {
    printf("IMU Task started\n");
    
    init_imus();

    const float UPDATE_RATE_HZ = 50.0f;
    const float UPDATE_PERIOD_S = 1.0f / UPDATE_RATE_HZ;

    while (1) {
        ESP_ERROR_CHECK(my_imu_update(UPDATE_PERIOD_S));

        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(10))) {

            float shoulder_pitch = my_imu_get_pitch(2);   // IMU #2: shoulder
            float shoulder_roll = my_imu_get_roll(2);

            shoulder_pitch = shoulder_pitch + 90;


            float elbow_pitch_1 = my_imu_get_pitch(1);     
            float elbow_pitch = -elbow_pitch_1 + 90-shoulder_pitch;  // IMU #1: elbow
            
            // Get independent pitch and roll from each IMU
            float wrist_pitch = my_imu_get_pitch(0);      // IMU #0: wrist
            wrist_pitch = wrist_pitch-elbow_pitch_1+90; // Adjust based on elbow pitch
            float wrist_roll = my_imu_get_roll(0);
            float delta = wrist_roll;
            wrist_roll = 90.0f + (delta > 0 ? (180.0f - delta) : (180.0f + delta));

            
            

            // Map to servo angles directly
            servo_angles[0] = shoulder_roll;  // S1 -> SHOULDER ROLL
            servo_angles[1] = shoulder_pitch; // S2 -> SHOULDER PITCH
            servo_angles[2] = elbow_pitch;    // S3 -> ELBOW PITCH
            servo_angles[3] = wrist_pitch;    // S4 -> WRIST PITCH
            servo_angles[4] = wrist_roll;     // S5 -> WRIST ROLL

            for (int i = 0; i < 5; i++) {
                // Clamp angles to servo limits if needed
                servo_angles[i] = clampAngle(servo_angles[i]);
            }
            
            printf("S1(SH_Roll): %.2f | S2(SH_Pitch): %.2f | S3(EL_Pitch): %.2f | S4(WR_Pitch): %.2f | S5(WR_Roll): %.2f\n",
                servo_angles[0], servo_angles[1], servo_angles[2], servo_angles[3], servo_angles[4]);

            xSemaphoreGive(imu_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / UPDATE_RATE_HZ));
    }
}

// === Wi-Fi Client Task ===
void client_task(void *pvParameters) {
    printf("Wi-Fi client task started\n");
    ESP_ERROR_CHECK(init_wifi_sta());

    const TickType_t delay = pdMS_TO_TICKS(1000 / 5);  // 5 Hz
    float angles_copy[5]; // Only need 5 servo angles

    while (1) {
        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(10))) {
            memcpy(angles_copy, servo_angles, sizeof(servo_angles));
            xSemaphoreGive(imu_mutex);
        }
        post_angles(angles_copy, 5); // Send 5 angles instead of 9
        vTaskDelay(delay);
    }
}

// === Main Application Entry Point ===
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    i2c_master_init();

    imu_mutex = xSemaphoreCreateMutex();

    xTaskCreateStatic(
        imu_task,
        "IMU_Task",
        IMU_TASK_STACK_SIZE,
        NULL,
        5,
        imu_task_stack,
        &imu_task_buffer
    );

    xTaskCreateStatic(
        client_task,
        "Client_Task",
        CLIENT_TASK_STACK_SIZE,
        NULL,
        5,
        client_task_stack,
        &client_task_buffer
    );
}