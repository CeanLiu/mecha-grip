/*  ==========  main.c  —  IMU-to-servo Wi-Fi client  ==========
 *  • ESP-IDF v5 style (i2c_new_master_bus)
 *  • Two MPU-6050 on I²C0 (SDA 32, SCL 33)          @50 Hz
 *  • Publishes four angles  p1…p4  to  http://192.168.4.1/update
 *    p1 = pitch( IMU-0 ),  p2 = roll( IMU-0 )
 *    p3 = pitch( IMU-1 ),  p4 = roll( IMU-1 )
 *  • Blink on GPIO2 when POST succeeded
 * ------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "my_imu.h"          /* <-- your wrapper around MPU-6050 */
#include "my_servo.h"        /* only for clampAngle helper       */

/* ---------- build-time constants -------------------------------- */
#define TAG              "IMU_CLIENT"

#define WIFI_SSID        "ServoESP"
#define WIFI_PASS        "12345678"
#define POST_URL         "http://192.168.4.1/update"

#define I2C_PORT         I2C_NUM_0
#define SDA_GPIO         32
#define SCL_GPIO         33

#define IMU_NUM          2
static const uint8_t IMU_ADDR[IMU_NUM] = { 0x68 , 0x69 };

#define FAST_LOOP_HZ     50.0f
#define DT_S             (1.0f/FAST_LOOP_HZ)

#define LED_PIN          2          /* blink on TX success */

/* ---------- Wi-Fi bookkeeping ----------------------------------- */
static EventGroupHandle_t wifi_evt;
#define WIFI_OK BIT0
static void wifi_evt_cb(void*,esp_event_base_t,int32_t,void*);

/* ---------- I²C bus handle -------------------------------------- */
static i2c_master_bus_handle_t i2c_bus;

/* ---------- IMU data ------------------------------------------- */
static float pitch[IMU_NUM], roll_[IMU_NUM];

/* --------------------------------------------------------------- */
/*  Tiny blocking POST                                             */
/* --------------------------------------------------------------- */
static esp_err_t post_angles(float p1,float p2,float p3,float p4)
{
    char body[64];
    snprintf(body,sizeof body,"p1:%.1f,p2:%.1f,p3:%.1f,p4:%.1f",
                                p1     , p2     , p3     , p4);

    esp_http_client_config_t cfg = {
        .url = POST_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    esp_http_client_set_header(cli,"Content-Type","text/plain");
    esp_http_client_set_post_field(cli,body,strlen(body));

    esp_err_t err = esp_http_client_perform(cli);
    esp_http_client_cleanup(cli);
    return err;
}

/* --------------------------------------------------------------- */
/*  IMU task – runs at FAST_LOOP_HZ                                */
/* --------------------------------------------------------------- */
static void imu_task(void *arg)
{
    /* ---- initialise IMUs on the shared bus ---- */
    for(int i=0;i<IMU_NUM;++i){
        ESP_ERROR_CHECK(my_imu_init(i2c_bus, IMU_ADDR[i]));
    }
    ESP_LOGI(TAG,"IMUs ready – keep still 1 s for internal calib");
    vTaskDelay(pdMS_TO_TICKS(1000));

    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);

    const TickType_t period = pdMS_TO_TICKS(1000/FAST_LOOP_HZ);

    while(1){
        for(int i=0;i<IMU_NUM;++i){
            ESP_ERROR_CHECK(my_imu_update(i, DT_S));
            pitch[i] = my_imu_get_pitch(i);
            roll_[i] = my_imu_get_roll (i);
        }

        /* mapping to p1…p4 ; edit if you need another scheme */
        float p1 =  pitch[0];          /* waist  */
        float p2 =  roll_[0];          /* shoulder */
        float p3 =  pitch[1];          /* elbow */
        float p4 =  roll_[1];          /* wrist – server mirrors for the 5th servo */

        if(post_angles(p1,p2,p3,p4)==ESP_OK){
            gpio_set_level(LED_PIN,1); vTaskDelay(pdMS_TO_TICKS(25));
            gpio_set_level(LED_PIN,0);
        }else{
            ESP_LOGW(TAG,"POST failed");
        }

        vTaskDelay(period);
    }
}

/* --------------------------------------------------------------- */
/*  Wi-Fi STA helper                                               */
/* --------------------------------------------------------------- */
static void wifi_evt_cb(void* a,esp_event_base_t base,int32_t id,void* d)
{
    if(base==WIFI_EVENT && id==WIFI_EVENT_STA_START)         esp_wifi_connect();
    else if(base==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if(base==IP_EVENT   && id==IP_EVENT_STA_GOT_IP)     xEventGroupSetBits(wifi_evt,WIFI_OK);
}
static void wifi_sta_init(void)
{
    wifi_evt = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_evt_cb,NULL,NULL);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_evt_cb,NULL,NULL);

    wifi_config_t sta = { 0 };
    strcpy((char*)sta.sta.ssid, WIFI_SSID);
    strcpy((char*)sta.sta.password, WIFI_PASS);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG,"Connecting to %s …",WIFI_SSID);
    xEventGroupWaitBits(wifi_evt,WIFI_OK,pdFALSE,pdTRUE,portMAX_DELAY);
    ESP_LOGI(TAG,"Wi-Fi connected ✓");
}

/* --------------------------------------------------------------- */
/*  I²C bus init (IDF v5)                                          */
/* --------------------------------------------------------------- */
static void i2c_master_bus_init(void)
{
    i2c_master_bus_config_t conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &i2c_bus));
}

/* --------------------------------------------------------------- */
/*  app_main                                                       */
/* --------------------------------------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    i2c_master_bus_init();
    wifi_sta_init();

    /* optional – if you want to use my_servo helper anywhere: */
    // ESP_ERROR_CHECK(my_servo_init());

    static StackType_t  imu_stack[4096];
    static StaticTask_t imu_tcb;
    xTaskCreateStaticPinnedToCore(
        imu_task, "imu",                sizeof(imu_stack)/sizeof(StackType_t),
        NULL, 5, imu_stack, &imu_tcb,   tskNO_AFFINITY);
}
