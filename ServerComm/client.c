/*  =========  main.c  –  “MPU client”  =========
    - Reads two MPU-6050 IMUs (I²C0 @21/22, I²C1 @32/33)
    - Runs complementary filter at 20 Hz
    - Sends pitch values to the servo-ESP32 at 192.168.4.1 /update
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mpu6050.h"           // <-- your lightweight driver from earlier

/* ============================================================
 * Compile-time parameters
 * ========================================================== */
#define TAG                 "MPU_CLIENT"

#define WIFI_SSID           "ServoESP"
#define WIFI_PASS           "12345678"
#define SERVER_URL          "http://192.168.4.1/update"

#define SAMPLE_PERIOD_MS    50          // 20 Hz
#define ALPHA               0.98f       // complementary filter parameter

// I²C wiring for the two sensors
#define I2C0_SDA            21
#define I2C0_SCL            22
#define I2C1_SDA            32
#define I2C1_SCL            33

// LED for traffic feedback (GPIO 2 on most DevKit boards)
#define LED_PIN             2
#define LED_ON()            gpio_set_level(LED_PIN, 1)
#define LED_OFF()           gpio_set_level(LED_PIN, 0)

/* ============================================================
 * Globals
 * ========================================================== */
#define SENSOR_CNT          2
static const i2c_port_t bus [SENSOR_CNT] = { I2C_NUM_0,       I2C_NUM_1 };
static const uint8_t    adr [SENSOR_CNT] = { MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH };

static int16_t ax_off[SENSOR_CNT], ay_off[SENSOR_CNT], az_off[SENSOR_CNT];
static float   pitch  [SENSOR_CNT];

static EventGroupHandle_t wifi_evt_grp;
#define WIFI_CONNECTED_BIT  BIT0

/* ============================================================
 * Wi-Fi STA helper
 * ========================================================== */
static void wifi_event_cb(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();

    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();                    // reconnect

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(wifi_evt_grp, WIFI_CONNECTED_BIT);
}

static void wifi_init_sta(void)
{
    wifi_evt_grp = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_cb, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_cb, NULL, NULL);

    wifi_config_t sta = { 0 };
    strcpy((char *)sta.sta.ssid, WIFI_SSID);
    strcpy((char *)sta.sta.password, WIFI_PASS);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to %s …", WIFI_SSID);
    xEventGroupWaitBits(wifi_evt_grp, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected ✔");
}

/* ============================================================
 * I²C helper
 * ========================================================== */
static void i2c_bus_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    const i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000 };
    ESP_ERROR_CHECK(i2c_param_config(port, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(port, cfg.mode, 0, 0, 0));
}

/* ============================================================
 * Simple calibration (average of 200 samples)
 * ========================================================== */
static void calibrate(int idx)
{
    int32_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < 200; ++i) {
        int16_t ax, ay, az;
        mpu6050_read_accel(bus[idx], adr[idx], &ax, &ay, &az);
        sx += ax; sy += ay; sz += az;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ax_off[idx] = sx / 200;
    ay_off[idx] = sy / 200;
    az_off[idx] = sz / 200;
}

/* ============================================================
 * HTTP POST helper (blocking, tiny payload)
 * ========================================================== */
static esp_err_t post_pitch(float p1, float p2)
{
    char body[64];
    snprintf(body, sizeof(body), "pitch1:%.2f,pitch2:%.2f", p1, p2);

    esp_http_client_config_t cfg = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    esp_http_client_set_header(cli, "Content-Type", "text/plain");
    esp_http_client_set_post_field(cli, body, strlen(body));

    esp_err_t err = esp_http_client_perform(cli);
    esp_http_client_cleanup(cli);
    return err;
}

/* ============================================================
 * Main IMU / networking task
 * ========================================================== */
static void imu_task(void *arg)
{
    /* 1. Init I²C buses and sensors */
    i2c_bus_init(I2C_NUM_0, I2C0_SDA, I2C0_SCL);
    i2c_bus_init(I2C_NUM_1, I2C1_SDA, I2C1_SCL);

    for (int i = 0; i < SENSOR_CNT; ++i)
        mpu6050_init(bus[i], adr[i]);

    /* 2. Calibration (board still) */
    ESP_LOGI(TAG, "Keep sensors still…");
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i < SENSOR_CNT; ++i) calibrate(i);
    ESP_LOGI(TAG, "Calibration done");

    /* 3. LED GPIO */
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    const TickType_t period = pdMS_TO_TICKS(SAMPLE_PERIOD_MS);

    while (true) {
        /* 3-axis accel + gyro → complementary-filtered pitch */
        for (int i = 0; i < SENSOR_CNT; ++i) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu6050_read_accel(bus[i], adr[i], &ax, &ay, &az);
            mpu6050_read_gyro (bus[i], adr[i], &gx, &gy, &gz);

            ax -= ax_off[i]; ay -= ay_off[i]; az -= az_off[i];

            float acc_pitch  = atan2f((float)ay, (float)az) * 180.0f / M_PI;
            float gyro_rate  = gx / 131.0f;                 // °/s  (±250 dps range)
            pitch[i] = ALPHA * (pitch[i] + gyro_rate * (SAMPLE_PERIOD_MS/1000.0f))
                     + (1.0f - ALPHA) * acc_pitch;
        }

        /* 4. Send to server */
        if (post_pitch(pitch[0], pitch[1]) == ESP_OK) {
            LED_ON(); vTaskDelay(pdMS_TO_TICKS(25)); LED_OFF();
        } else {
            ESP_LOGW(TAG, "POST failed");
        }

        vTaskDelay(period);
    }
}

/* ============================================================
 * app_main
 * ========================================================== */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096 * 2,
                            NULL, 5, NULL, tskNO_AFFINITY);
}
