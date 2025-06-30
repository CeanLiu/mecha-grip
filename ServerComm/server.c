#include <stdio.h>
#define MIN(a, b) ((a) < (b) ? (a) : (b))

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

#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define SERVO_FREQ    50
#define SERVO1_PIN    25
#define SERVO2_PIN    26

#define LED_PIN       2
#define LED_PULSE_MS  100

static const char *TAG = "SERVO_SERVER";

// Global control state
volatile bool led_pulse_request = false;
volatile bool is_manual_mode = false;

float received_pitch[2] = {90.0, 90.0};
float manual_pitch[2]   = {90.0, 90.0};

// ---------- PWM Setup ----------
static void init_servo_pwm(int gpio_num, ledc_channel_t channel) {
    static bool timer_init = false;
    if (!timer_init) {
        const ledc_timer_config_t t = {
            .speed_mode       = LEDC_HIGH_SPEED_MODE,
            .timer_num        = LEDC_TIMER_0,
            .duty_resolution  = LEDC_TIMER_16_BIT,
            .freq_hz          = SERVO_FREQ,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&t);
        timer_init = true;
    }

    const ledc_channel_config_t ch = {
        .gpio_num   = gpio_num,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch);
}

static inline void set_servo_angle(ledc_channel_t channel, float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    const uint32_t us = SERVO_MIN_US + (angle_deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    const uint32_t duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

// ---------- HTTP Handlers ----------
static esp_err_t pitch_post_handler(httpd_req_t *req) {
    char buf[64] = {0};
    int len = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf) - 1));
    if (len <= 0) return ESP_FAIL;

    if (!is_manual_mode &&
        sscanf(buf, "pitch1:%f,pitch2:%f", &received_pitch[0], &received_pitch[1]) == 2) {
        ESP_LOGI(TAG, "External pkt => %s", buf);
        led_pulse_request = true;
        httpd_resp_sendstr(req, "OK");
    } else {
        httpd_resp_sendstr(req, "ERR");
    }
    return ESP_OK;
}

static esp_err_t manual_post_handler(httpd_req_t *req) {
    char buf[64] = {0};
    int len = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf) - 1));
    if (len <= 0) return ESP_FAIL;

    if (sscanf(buf, "pitch1:%f,pitch2:%f", &manual_pitch[0], &manual_pitch[1]) == 2) {
        ESP_LOGI(TAG, "Manual input => %s", buf);
        httpd_resp_sendstr(req, "OK");
    } else {
        httpd_resp_sendstr(req, "ERR");
    }
    return ESP_OK;
}

static esp_err_t mode_post_handler(httpd_req_t *req) {
    char buf[8] = {0};
    httpd_req_recv(req, buf, sizeof(buf));
    is_manual_mode = (buf[0] == '1');
    ESP_LOGI(TAG, "Mode set to: %s", is_manual_mode ? "Manual" : "External");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t angles_get_handler(httpd_req_t *req) {
    char json[64];
    float s1 = is_manual_mode ? manual_pitch[0] : received_pitch[0] + 90.0f;
    float s2 = is_manual_mode ? manual_pitch[1] : received_pitch[1] + 90.0f;
    snprintf(json, sizeof(json), "{\"s1\":%.1f,\"s2\":%.1f}", s1, s2);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    const char *html =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<title>ESP32 Servo Control</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;background:#eef2f3;margin:0;padding:0}"
    "h1{margin:0;padding:16px;background:#3949ab;color:white;text-align:center}"
    ".container{max-width:480px;margin:20px auto;padding:20px;background:#fff;border-radius:8px;box-shadow:0 2px 6px rgba(0,0,0,0.1)}"
    ".angles{font-size:18px;margin-bottom:16px}"
    ".row{display:flex;align-items:center;justify-content:space-between;margin:10px 0}"
    "input[type=range]{flex:1;margin:0 10px}"
    ".val{width:40px;text-align:right}"
    ".toggle{position:relative;display:inline-block;width:50px;height:26px}"
    ".toggle input{display:none}"
    ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;border-radius:34px;transition:.2s}"
    ".slider:before{position:absolute;content:'';height:20px;width:20px;left:3px;bottom:3px;background:white;border-radius:50%;transition:.2s}"
    "input:checked+.slider{background:#4caf50}"
    "input:checked+.slider:before{transform:translateX(24px)}"
    "button{background:#3949ab;color:#fff;border:none;border-radius:4px;padding:8px 16px;cursor:pointer;margin-top:12px}"
    "</style></head><body>"
    "<h1>Servo Dashboard</h1>"
    "<div class='container'>"
        "<div class='row'><span>Manual Mode</span>"
        "<label class='toggle'><input type='checkbox' id='mode'><span class='slider'></span></label></div>"

        "<div class='angles'>"
            "Servo 1: <span id='a1'>...</span>°<br>"
            "Servo 2: <span id='a2'>...</span>°"
        "</div>"

        "<div id='controls' style='display:none'>"
            "<div class='row'>S1 <input type='range' id='s1' min='0' max='180'><span class='val' id='s1val'>90</span>°</div>"
            "<div class='row'>S2 <input type='range' id='s2' min='0' max='180'><span class='val' id='s2val'>90</span>°</div>"
            "<div style='text-align:center'><button onclick='sendManual()'>Send</button></div>"
        "</div>"
    "</div>"

    "<script>"
    "const mode=document.getElementById('mode'),"
    "controls=document.getElementById('controls'),"
    "s1=document.getElementById('s1'),s2=document.getElementById('s2'),"
    "v1=document.getElementById('s1val'),v2=document.getElementById('s2val'),"
    "a1=document.getElementById('a1'),a2=document.getElementById('a2');"

    "mode.onchange=()=>{"
        "controls.style.display=mode.checked?'block':'none';"
        "fetch('/mode',{method:'POST',body:mode.checked?'1':'0'});"
    "};"

    "s1.oninput=()=>{v1.textContent=s1.value};"
    "s2.oninput=()=>{v2.textContent=s2.value};"

    "function sendManual(){"
        "fetch('/manual',{method:'POST',body:`pitch1:${s1.value},pitch2:${s2.value}`})"
    "}"

    "function refresh(){"
        "fetch('/angles').then(r=>r.json()).then(d=>{"
            "a1.textContent=d.s1;"
            "a2.textContent=d.s2;"
            "if(!mode.checked){"
                "s1.value=d.s1;v1.textContent=d.s1;"
                "s2.value=d.s2;v2.textContent=d.s2;"
            "}"
        "});"
    "}"
    "setInterval(refresh,800);"
    "refresh();"
    "</script></body></html>";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// ---------- Webserver Setup ----------
static void start_webserver(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t srv = NULL;
    httpd_start(&srv, &cfg);

    httpd_register_uri_handler(srv, &(httpd_uri_t){"/",      HTTP_GET,  index_get_handler,   NULL});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/update",HTTP_POST, pitch_post_handler,  NULL});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/manual",HTTP_POST, manual_post_handler, NULL});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/mode",  HTTP_POST, mode_post_handler,   NULL});
    httpd_register_uri_handler(srv, &(httpd_uri_t){"/angles",HTTP_GET,  angles_get_handler,  NULL});
}

// ---------- Wi-Fi SoftAP ----------
static void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap = {
        .ap = {
            .ssid           = "ServoESP",
            .ssid_len       = 0,
            .password       = "12345678",
            .max_connection = 4,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();
}

// ---------- LED Feedback ----------
static void led_task(void *arg) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        if (led_pulse_request) {
            led_pulse_request = false;
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_PULSE_MS));
            gpio_set_level(LED_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------- Main ----------
void app_main(void) {
    nvs_flash_init();
    wifi_init_softap();
    start_webserver();

    init_servo_pwm(SERVO1_PIN, LEDC_CHANNEL_0);
    init_servo_pwm(SERVO2_PIN, LEDC_CHANNEL_1);

    xTaskCreatePinnedToCore(led_task, "ledTask", 2048, NULL, 5, NULL, tskNO_AFFINITY);

    while (1) {
        float s1 = is_manual_mode ? manual_pitch[0] : received_pitch[0] + 90.0f;
        float s2 = is_manual_mode ? manual_pitch[1] : received_pitch[1] + 90.0f;
        set_servo_angle(LEDC_CHANNEL_0, s1);
        set_servo_angle(LEDC_CHANNEL_1, s2);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
