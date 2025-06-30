/*********************************************************************
 *  5-DOF SERVO SERVER (ESP32 – Soft-AP).
 *  - S1  pin 25  LEDC_CHANNEL_0
 *  - S2  pin 26  LEDC_CHANNEL_1
 *  - S3  pin 27  LEDC_CHANNEL_2
 *  - S4  pin 14  LEDC_CHANNEL_3   (user angle)
 *  - S5  pin 33  LEDC_CHANNEL_4   (mirror of S4)
 *********************************************************************/

#include <stdio.h>
#define MIN(a,b) ((a)<(b)?(a):(b))

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* ─── Servo HW ───────────────────────────────────────────────────── */
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define SERVO_FREQ    50         // 50 Hz => 20 ms period

#define SERVO_CNT     5
static const int  SERVO_PIN [SERVO_CNT] =
                   { 25, 26, 27, 14, 33 };
static const ledc_channel_t SERVO_CH[SERVO_CNT] =
                   { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
                     LEDC_CHANNEL_3, LEDC_CHANNEL_4 };

#define LED_PIN       2          // onboard LED feedback
#define LED_PULSE_MS  100
static volatile bool led_pulse_request = false;

/* ─── Control state ──────────────────────────────────────────────── */
static volatile bool manual_mode   = false;   // false = external
static float ext_deg [4] = {90,90,90,90};     // p1…p4 from IMU-client
static float man_deg [4] = {90,90,90,90};     // sliders

static const char *TAG = "SERVO_SERVER";

/* ─── Servo helpers ─────────────────────────────────────────────── */
static void servo_timer_init(void)
{
    static bool done = false;
    if (done) return;
    const ledc_timer_config_t t = {
        .speed_mode      = LEDC_HIGH_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz         = SERVO_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    done = true;
}
static void servo_channel_init(void)
{
    servo_timer_init();
    for (int i = 0; i < SERVO_CNT; ++i) {
        const ledc_channel_config_t ch = {
            .gpio_num   = SERVO_PIN[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel    = SERVO_CH[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0 };
        ledc_channel_config(&ch);
    }
}
static inline void servo_write_deg(ledc_channel_t ch, float deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    uint32_t us = SERVO_MIN_US +
                  (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    uint32_t duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

/* ─── HTTP handlers ─────────────────────────────────────────────── */
static esp_err_t ext_post_handler(httpd_req_t *r)          /* /update */
{
    char buf[64]={0};
    int n=httpd_req_recv(r,buf,MIN(r->content_len,sizeof buf-1));
    if (n<=0) return ESP_FAIL;
    if (!manual_mode &&
        sscanf(buf,"p1:%f,p2:%f,p3:%f,p4:%f",
               &ext_deg[0],&ext_deg[1],&ext_deg[2],&ext_deg[3])==4) {
        led_pulse_request=true;
        ESP_LOGI(TAG,"RX ext: %s",buf);
        httpd_resp_sendstr(r,"OK");
    } else httpd_resp_sendstr(r,"ERR");
    return ESP_OK;
}
static esp_err_t man_post_handler(httpd_req_t *r)          /* /manual */
{
    char buf[64]={0};
    int n=httpd_req_recv(r,buf,MIN(r->content_len,sizeof buf-1));
    if (n<=0) return ESP_FAIL;
    if (sscanf(buf,"p1:%f,p2:%f,p3:%f,p4:%f",
               &man_deg[0],&man_deg[1],&man_deg[2],&man_deg[3])==4) {
        ESP_LOGI(TAG,"RX manual: %s",buf);
        httpd_resp_sendstr(r,"OK");
    } else httpd_resp_sendstr(r,"ERR");
    return ESP_OK;
}
static esp_err_t mode_post_handler(httpd_req_t *r)         /* /mode */
{
    char ch='0'; httpd_req_recv(r,&ch,1);
    manual_mode = (ch=='1');
    ESP_LOGI(TAG,"Mode = %s",manual_mode?"MAN":"EXT");
    httpd_resp_sendstr(r,"OK");
    return ESP_OK;
}
static esp_err_t angles_get_handler(httpd_req_t *r)        /* /angles */
{
    float src[4]; for(int i=0;i<4;++i) src[i]=manual_mode?man_deg[i]:ext_deg[i];
    float a[SERVO_CNT] = {
        src[0],                       // S1
        src[1],                       // S2
        src[2],                       // S3
        src[3],                       // S4
        180.0f - src[3]               // S5 mirror
    };
    char json[96];
    snprintf(json,sizeof json,
        "{\"d\":[%.1f,%.1f,%.1f,%.1f,%.1f]}",
        a[0],a[1],a[2],a[3],a[4]);
    httpd_resp_set_type(r,"application/json");
    httpd_resp_sendstr(r,json);
    return ESP_OK;
}
/* index page (embedded HTML+JS) */
static esp_err_t index_handler(httpd_req_t *r)
{
    const char *pg =
"<!DOCTYPE html><html><head><meta charset='utf-8'><title>Servo Dash</title>"
"<style>body{font-family:Arial;background:#eef2f3;margin:0}"
"h1{margin:0;padding:14px;background:#3949ab;color:#fff;text-align:center}"
".box{max-width:480px;background:#fff;margin:18px auto;padding:18px;border-radius:8px;"
"box-shadow:0 2px 6px rgba(0,0,0,.15)}.row{display:flex;align-items:center;margin:8px 0}"
".row input[type=range]{flex:1;margin:0 8px}.val{width:42px;text-align:right}"
".toggle{position:relative;width:52px;height:26px} .toggle input{display:none}"
".slider{position:absolute;top:0;left:0;right:0;bottom:0;background:#ccc;border-radius:34px;cursor:pointer}"
".slider:before{content:'';position:absolute;height:20px;width:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.2s}"
"input:checked+.slider{background:#4caf50}input:checked+.slider:before{transform:translateX(26px)}"
"button{margin-top:12px;background:#3949ab;color:#fff;border:none;padding:8px 16px;border-radius:4px;cursor:pointer}"
"</style></head><body><h1>Servo Dashboard</h1><div class='box'>"
"<div class='row'><span>Manual Mode</span>"
"<label class='toggle'><input id='mode' type='checkbox'><span class='slider'></span></label></div>"
"<div id='ang' style='font-size:17px;margin-bottom:12px'></div>"
"<div id='ctrl' style='display:none'>"
"<div class='row'>S1 <input id='r0' type='range' min='0' max='180'><span class='val' id='v0'>90</span>°</div>"
"<div class='row'>S2 <input id='r1' type='range' min='0' max='180'><span class='val' id='v1'>90</span>°</div>"
"<div class='row'>S3 <input id='r2' type='range' min='0' max='180'><span class='val' id='v2'>90</span>°</div>"
"<div class='row'>S4 <input id='r3' type='range' min='0' max='180'><span class='val' id='v3'>90</span>°"
"<small style='margin-left:6px'>(S5 mirrors)</small></div>"
"<div style='text-align:center'><button onclick='send()'>Send</button></div></div></div>"
"<script>"
"const m=document.getElementById('mode'),box=document.getElementById('ctrl');"
"const ang=document.getElementById('ang');"
"const r=[...Array(4)].map((_,i)=>document.getElementById('r'+i));"
"const v=[...Array(4)].map((_,i)=>document.getElementById('v'+i));"
"r.forEach((s,i)=>s.oninput=_=>v[i].textContent=s.value);"
"m.onchange=_=>{box.style.display=m.checked?'block':'none';fetch('/mode',{method:'POST',body:m.checked?'1':'0'});} ;"
"function send(){const p=`p1:${r[0].value},p2:${r[1].value},p3:${r[2].value},p4:${r[3].value}`;"
"fetch('/manual',{method:'POST',body:p});}"
"function loop(){fetch('/angles').then(x=>x.json()).then(j=>{"
"const d=j.d;ang.innerHTML=`S1 ${d[0]}°  S2 ${d[1]}°<br>S3 ${d[2]}°  S4 ${d[3]}°  S5 ${d[4]}°`;"
"if(!m.checked) r.forEach((s,i)=>{s.value=d[i];v[i].textContent=d[i];});});}"
"setInterval(loop,700);loop();"
"</script></body></html>";
    httpd_resp_send(r,pg,HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ─── Web server & Soft-AP init ─────────────────────────────────── */
static void webserver_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t h=NULL; httpd_start(&h,&cfg);
    httpd_register_uri_handler(h,&(httpd_uri_t){"/",HTTP_GET,index_handler,NULL});
    httpd_register_uri_handler(h,&(httpd_uri_t){"/update",HTTP_POST,ext_post_handler,NULL});
    httpd_register_uri_handler(h,&(httpd_uri_t){"/manual",HTTP_POST,man_post_handler,NULL});
    httpd_register_uri_handler(h,&(httpd_uri_t){"/mode",HTTP_POST,mode_post_handler,NULL});
    httpd_register_uri_handler(h,&(httpd_uri_t){"/angles",HTTP_GET,angles_get_handler,NULL});
}
static void wifi_softap_start(void)
{
    esp_netif_init();  esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    wifi_config_t ap={.ap={.ssid="ServoESP",.password="12345678",
                           .authmode=WIFI_AUTH_WPA_WPA2_PSK,.max_connection=4}};
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP,&ap); esp_wifi_start();
}

/* ─── LED feedback task ─────────────────────────────────────────── */
static void led_task(void *a)
{
    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);
    while(1){
        if(led_pulse_request){led_pulse_request=false;
            gpio_set_level(LED_PIN,1);
            vTaskDelay(pdMS_TO_TICKS(LED_PULSE_MS));
            gpio_set_level(LED_PIN,0);}
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ─── MAIN ──────────────────────────────────────────────────────── */
void app_main(void)
{
    nvs_flash_init();
    wifi_softap_start();
    webserver_start();
    servo_channel_init();
    xTaskCreatePinnedToCore(led_task,"led",2048,NULL,5,NULL,tskNO_AFFINITY);

    while(1){
        float src[4]; for(int i=0;i<4;++i) src[i]=manual_mode?man_deg[i]:ext_deg[i];

        /* S1…S3 */
        for(int i=0;i<3;++i) servo_write_deg(SERVO_CH[i], src[i]);
        /* S4 & mirrored S5 */
        float a4 = src[3];
        servo_write_deg(SERVO_CH[3], a4);
        servo_write_deg(SERVO_CH[4], 180.0f - a4);

        vTaskDelay(pdMS_TO_TICKS(40));   // ≈25 Hz
    }
}
