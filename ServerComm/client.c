#include "my_servo.h"
#include "my_imu.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define TAG "IMU_CLIENT"
#define WIFI_SSID "ServoESP"
#define WIFI_PASS "12345678"
#define SERVER_URL "http://192.168.4.1/update"

#define UPDATE_RATE_HZ 50.0f
#define UPDATE_PERIOD_MS (1000 / UPDATE_RATE_HZ)
#define IMU_TASK_STACK_SIZE 4096
#define LED_PIN 2

static StackType_t imu_task_stack[IMU_TASK_STACK_SIZE];
static StaticTask_t imu_task_buffer;
static EventGroupHandle_t wifi_evt;
#define WIFI_OK BIT0

static void wifi_event_cb(void* a, esp_event_base_t b, int32_t id, void* d) {
    if (b == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (b == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (b == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(wifi_evt, WIFI_OK);
}

static void wifi_sta_init(void) {
    wifi_evt = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_cb, NULL, NULL);
    wifi_config_t sta = { 0 };
    strcpy((char*)sta.sta.ssid, WIFI_SSID);
    strcpy((char*)sta.sta.password, WIFI_PASS);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta);
    esp_wifi_start();
    ESP_LOGI(TAG, "Connecting to %s …", WIFI_SSID);
    xEventGroupWaitBits(wifi_evt, WIFI_OK, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connected ✔");
}

static esp_err_t post_angles(float p1, float p2, float p3, float p4) {
    char body[64];
    snprintf(body, sizeof body, "p1:%.1f,p2:%.1f,p3:%.1f,p4:%.1f", p1, p2, p3, p4);
    esp_http_client_config_t c = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000
    };
    esp_http_client_handle_t h = esp_http_client_init(&c);
    esp_http_client_set_header(h, "Content-Type", "text/plain");
    esp_http_client_set_post_field(h, body, strlen(body));
    esp_err_t err = esp_http_client_perform(h);
    esp_http_client_cleanup(h);
    return err;
}

void i2c_master_init() {
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 32,
        .scl_io_num = 33,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &bus_handle));
}

void imu_task(void *pvParameters) {
    ESP_LOGI(TAG, "IMU Task started");
    ESP_ERROR_CHECK(my_imu_init(I2C_NUM_0));
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        ESP_ERROR_CHECK(my_imu_update(1.0f / UPDATE_RATE_HZ));
        float p1 = my_imu_get_filtered_pitch();
        float p2 = my_imu_get_filtered_roll();
        float p3 = 0;
        float p4 = 0;

        ESP_LOGI(TAG, "p1: %.2f, p2: %.2f", p1, p2);
        if (post_angles(p1, p2, p3, p4) == ESP_OK) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(LED_PIN, 0);
        } else {
            ESP_LOGW(TAG, "POST err");
        }
        vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));
    }
}

void app_main(void) {
    nvs_flash_init();
    wifi_sta_init();
    i2c_master_init();
    ESP_ERROR_CHECK(my_servo_init());

    xTaskCreateStatic(
        imu_task,
        "IMU_Task",
        IMU_TASK_STACK_SIZE,
        NULL,
        5,
        imu_task_stack,
        &imu_task_buffer
    );
}
