/*  =========  main.c  –  4-DOF IMU client  =========
    - Two MPU-6050                   (I²C0 21/22,  I²C1 32/33)
    - Complementary filter @ 20 Hz
    - Sends   p1…p4   to   http://192.168.4.1/update               */

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
#include "mpu6050.h"                     /* your minimal driver     */

/* ─── Build-time settings ──────────────────────────────────────── */
#define TAG           "IMU_CLIENT"
#define WIFI_SSID     "ServoESP"
#define WIFI_PASS     "12345678"
#define SERVER_URL    "http://192.168.4.1/update"

#define DT_MS         50                 /* 20 Hz loop              */
#define ALPHA         0.98f              /* Complementary filter    */

#define I2C0_SDA      21
#define I2C0_SCL      22
#define I2C1_SDA      32
#define I2C1_SCL      33

#define LED_PIN       2                  /* blink on successful TX */

/* ─── MPU wiring table ─────────────────────────────────────────── */
#define IMU_COUNT     2
static const i2c_port_t BUS[IMU_COUNT] = { I2C_NUM_0 , I2C_NUM_1 };
static const uint8_t    ADR[IMU_COUNT] = { MPU6050_ADDR_LOW , MPU6050_ADDR_HIGH };

/* ─── Calibration offsets & filtered states ───────────────────── */
static int16_t ax_off[IMU_COUNT], ay_off[IMU_COUNT], az_off[IMU_COUNT];
static float   pitch  [IMU_COUNT], roll  [IMU_COUNT];

/* ─── Wi-Fi bookkeeping ───────────────────────────────────────── */
static EventGroupHandle_t wifi_evt;
#define WIFI_OK BIT0
static void wifi_evt_cb(void*,esp_event_base_t,int32_t,void*);
static void wifi_sta_init(void);

/* ─── Helper: tiny blocking POST (payload < 64 B) ─────────────── */
static esp_err_t post_angles(float p1,float p2,float p3,float p4)
{
    char body[64];
    snprintf(body,sizeof body,
             "p1:%.1f,p2:%.1f,p3:%.1f,p4:%.1f",
             p1,p2,p3,p4);

    esp_http_client_config_t c = {
        .url = SERVER_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000 };
    esp_http_client_handle_t h = esp_http_client_init(&c);
    esp_http_client_set_header(h,"Content-Type","text/plain");
    esp_http_client_set_post_field(h,body,strlen(body));

    esp_err_t err = esp_http_client_perform(h);
    esp_http_client_cleanup(h);
    return err;
}

/* ─── I²C & calibration ───────────────────────────────────────── */
static void i2c_bus_init(i2c_port_t bus,int sda,int scl)
{
    const i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000 };
    i2c_param_config(bus,&cfg);
    i2c_driver_install(bus,cfg.mode,0,0,0);
}
static void calibrate(int idx)
{
    int32_t sx=0,sy=0,sz=0;
    for(int i=0;i<200;++i){
        int16_t ax,ay,az;
        mpu6050_read_accel(BUS[idx],ADR[idx],&ax,&ay,&az);
        sx+=ax; sy+=ay; sz+=az;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ax_off[idx]=sx/200; ay_off[idx]=sy/200; az_off[idx]=sz/200;
}

/* ─── IMU filter / TX task ────────────────────────────────────── */
static void imu_task(void* arg)
{
    /* 1. HW init ------------------------------------------------- */
    i2c_bus_init(I2C_NUM_0,I2C0_SDA,I2C0_SCL);
    i2c_bus_init(I2C_NUM_1,I2C1_SDA,I2C1_SCL);

    for(int i=0;i<IMU_COUNT;++i) mpu6050_init(BUS[i],ADR[i]);

    ESP_LOGI(TAG,"Keep sensors still for 1 s…"); vTaskDelay(pdMS_TO_TICKS(1000));
    for(int i=0;i<IMU_COUNT;++i) calibrate(i);
    ESP_LOGI(TAG,"Calibration OK");

    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);
    const TickType_t period = pdMS_TO_TICKS(DT_MS);

    /* 2. Main loop ---------------------------------------------- */
    while(1){
        for(int i=0;i<IMU_COUNT;++i){
            int16_t ax,ay,az,gx,gy,gz;
            mpu6050_read_accel(BUS[i],ADR[i],&ax,&ay,&az);
            mpu6050_read_gyro (BUS[i],ADR[i],&gx,&gy,&gz);

            ax-=ax_off[i]; ay-=ay_off[i]; az-=az_off[i];

            /* axis mapping:  pitch from Y-Z , roll from X-Z */
            float acc_pitch = atan2f((float)ay,(float)az)*180.0f/M_PI;
            float acc_roll  = atan2f(-(float)ax,
                                      sqrtf((float)ay*ay+(float)az*az))*180.0f/M_PI;

            float g_pitch   = gx/131.0f;   /* °/s   (±250 dps) */
            float g_roll    = gy/131.0f;

            pitch[i] = ALPHA*(pitch[i]+g_pitch*(DT_MS/1000.0f))
                     + (1-ALPHA)*acc_pitch;
            roll [i] = ALPHA*(roll [i]+g_roll *(DT_MS/1000.0f))
                     + (1-ALPHA)*acc_roll;
        }

        /* 3. Map to p1…p4  (edit here if you want a different scheme) */
        float p1 = pitch[0];          // waist
        float p2 = roll [0];          // shoulder
        float p3 = pitch[1];          // elbow
        float p4 = roll [1];          // wrist  (mirrored by server)

        /* 4. POST ------------------------------------------------- */
        if(post_angles(p1,p2,p3,p4)==ESP_OK){
            gpio_set_level(LED_PIN,1); vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(LED_PIN,0);
        }else{
            ESP_LOGW(TAG,"POST err");
        }
        vTaskDelay(period);
    }
}

/* ─── Wi-Fi STA helpers ───────────────────────────────────────── */
static void wifi_event_cb(void* a,esp_event_base_t b,int32_t id,void* d)
{
    if(b==WIFI_EVENT && id==WIFI_EVENT_STA_START) esp_wifi_connect();
    else if(b==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if(b==IP_EVENT && id==IP_EVENT_STA_GOT_IP)
        xEventGroupSetBits(wifi_evt,WIFI_OK);
}
static void wifi_sta_init(void)
{
    wifi_evt=xEventGroupCreate();
    esp_netif_init(); esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_cb,NULL,NULL);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event_cb,NULL,NULL);

    wifi_config_t sta={0};
    strcpy((char*)sta.sta.ssid,WIFI_SSID);
    strcpy((char*)sta.sta.password,WIFI_PASS);
    sta.sta.threshold.authmode=WIFI_AUTH_WPA_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA,&sta);
    esp_wifi_start();

    ESP_LOGI(TAG,"Connecting to %s …",WIFI_SSID);
    xEventGroupWaitBits(wifi_evt,WIFI_OK,pdFALSE,pdTRUE,portMAX_DELAY);
    ESP_LOGI(TAG,"Wi-Fi connected ✔");
}

/* ─── app_main ─────────────────────────────────────────────────── */
void app_main(void)
{
    nvs_flash_init();
    wifi_sta_init();

    xTaskCreatePinnedToCore(imu_task,"imu",4096*2,NULL,5,NULL,tskNO_AFFINITY);
}
