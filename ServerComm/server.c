/**
 *  7-DOF SERVO SERVER (ESP32 – Soft-AP) + Three.js Dashboard
 *
 *  - S1  pin 25 LEDC_CHANNEL_0   (base angle p1)
 *  - S2  pin 26 LEDC_CHANNEL_1   (shoulder angle p2)
 *  - S3  pin 27 LEDC_CHANNEL_2   (mirror of S2)
 *  - S4  pin 14 LEDC_CHANNEL_3   (elbow angle p3)
 *  - S5  pin 33 LEDC_CHANNEL_4   (waist pitch p4)
 *  - S6  pin 32 LEDC_CHANNEL_5   (waist yaw p5)
 *  - S7  pin 13 LEDC_CHANNEL_6   (gripper)
 */

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
#include "three_js.h" // your embedded three.js blob

static const char *TAG = "SERVO_SERVER";

/* ─── Servo HW ───────────────────────────────────────────────────── */
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400
#define SERVO_FREQ 50

// Now 7 channels:
#define SERVO_CNT 7
static const int SERVO_PIN[SERVO_CNT] = {25, 26, 27, 14, 33, 32, 13};
static const ledc_channel_t SERVO_CH[SERVO_CNT] = {
    LEDC_CHANNEL_0, // S1 = p1
    LEDC_CHANNEL_1, // S2 = p2
    LEDC_CHANNEL_2, // S3 = mirror of p2
    LEDC_CHANNEL_3, // S4 = p3
    LEDC_CHANNEL_4, // S5 = p4
    LEDC_CHANNEL_5, // S6 = p5
    LEDC_CHANNEL_6  // S7 = gripper
};

#define LED_PIN 2
#define LED_PULSE_MS 100
static volatile bool led_pulse_request = false;

/* ─── Control state ──────────────────────────────────────────────── */
// manual_mode=false → external (IMU) angles, true → sliders
static volatile bool manual_mode = true;

// we now accept 5 independent inputs: p1…p5
// S1..S4 ← p1..p4, S5 mirrors p4, S6 ← p5
static float ext_deg[5] = {90, 90, 90, 90, 90};
static float man_deg[5] = {90, 90, 90, 90, 90};

// S7 is controlled by Python via HTTP POST
static float s7_angle = 90;

static const float SERVO_OFFSET[5] = {
    0.0f, // Offset for S1: center at 108°
    0.0f, // Offset for S2: center at 133°
    0.0f, // Offset for S4: center at 90° (no offset)
    0.0f, // Offset for S5: center at 90°
    0.0f  // Offset for S6: center at 46°
};

/* ─── Servo helpers ─────────────────────────────────────────────── */
static void servo_timer_init(void)
{
    static bool done = false;
    if (done)
        return;
    ledc_timer_config_t t = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    done = true;
}

static void servo_channel_init(void)
{
    servo_timer_init();
    for (int i = 0; i < SERVO_CNT; i++)
    {
        ledc_channel_config_t ch = {
            .gpio_num = SERVO_PIN[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = SERVO_CH[i],
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0};
        ledc_channel_config(&ch);
    }
}

static inline void servo_write_deg(ledc_channel_t ch, float deg)
{
    if (deg < 0)
        deg = 0;
    if (deg > 180)
        deg = 180;
    uint32_t us = SERVO_MIN_US + (deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    uint32_t duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

/* ─── HTTP handlers ─────────────────────────────────────────────── */
// POST /update  body = "p1:..,p2:..,p3:..,p4:..,p5:.."
static esp_err_t ext_post_handler(httpd_req_t *r)
{
    char buf[64] = {0};
    int n = httpd_req_recv(r, buf, MIN(r->content_len, sizeof(buf) - 1));
    if (n <= 0)
        return ESP_FAIL;
    if (!manual_mode &&
        sscanf(buf, "p1:%f,p2:%f,p3:%f,p4:%f,p5:%f",
               &ext_deg[0], &ext_deg[1], &ext_deg[2], &ext_deg[3], &ext_deg[4]) == 5)
    {
        led_pulse_request = true;
        ESP_LOGI(TAG, "RX ext: %s", buf);
        httpd_resp_sendstr(r, "OK");
    }
    else
    {
        httpd_resp_sendstr(r, "ERR");
    }
    return ESP_OK;
}

// POST /manual  same format → man_deg[0..4]
static esp_err_t man_post_handler(httpd_req_t *r)
{
    char buf[64] = {0};
    int n = httpd_req_recv(r, buf, MIN(r->content_len, sizeof(buf) - 1));
    if (n <= 0)
        return ESP_FAIL;
    if (sscanf(buf, "p1:%f,p2:%f,p3:%f,p4:%f,p5:%f",
               &man_deg[0], &man_deg[1], &man_deg[2], &man_deg[3], &man_deg[4]) == 5)
    {
        ESP_LOGI(TAG, "RX manual: %s", buf);
        httpd_resp_sendstr(r, "OK");
    }
    else
    {
        httpd_resp_sendstr(r, "ERR");
    }
    return ESP_OK;
}

// POST /mode   body='1' or '0'
static esp_err_t mode_post_handler(httpd_req_t *r)
{
    char ch = '0';
    httpd_req_recv(r, &ch, 1);
    manual_mode = (ch == '1');
    ESP_LOGI(TAG, "Mode = %s", manual_mode ? "MAN" : "EXT");
    httpd_resp_sendstr(r, "OK");
    return ESP_OK;
}

// GET /angles → {"d":[a0,a1,a2,a3,a4,a5]}
static esp_err_t angles_get_handler(httpd_req_t *r)
{
    float src[5];
    for (int i = 0; i < 5; i++)
    {
        src[i] = manual_mode ? man_deg[i] : ext_deg[i];
    }
    // reorder into the 6‐element array [S1,S2,S3,S4,S5,S6]:
    float a[SERVO_CNT] = {
        src[0],          // S1 = p1
        src[1],          // S2 = p2
        180.0f - src[1], // S3 = mirror of p2
        src[2],          // S4 = p3
        src[3],          // S5 = p4
        src[4],          // S6 = p5
        s7_angle         // S7 = gripper
    };
    char json[128];
    int len = snprintf(json, sizeof(json),
                       "{\"d\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}",
                       a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
    httpd_resp_set_type(r, "application/json");
    httpd_resp_send(r, json, len);
    return ESP_OK;
}

static esp_err_t py_post_handler(httpd_req_t *r)
{
    char buf[16] = {0};
    int n = httpd_req_recv(r, buf, MIN(r->content_len, sizeof(buf) - 1));
    if (n <= 0)
        return ESP_FAIL;
    float angle = 90;
    if (sscanf(buf, "%f", &angle) == 1)
    {
        s7_angle = angle;
        ESP_LOGI("PY_CTRL", "S7 angle set: %.1f", s7_angle);
        httpd_resp_sendstr(r, "OK");
    }
    else
    {
        httpd_resp_sendstr(r, "ERR");
    }
    return ESP_OK;
}

// GET /      → embeds HTML+JS+Three.js
static esp_err_t index_handler(httpd_req_t *r)
{
    const char *pg =
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>7-DOF Servo Dashboard</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "html,body{height:100%;margin:0;padding:0;}"
        "body{font-family:'Segoe UI',Arial,sans-serif;background:#f4f7fa;margin:0;padding:0;width:100vw;height:100vh;overflow:hidden;}"
        ".main-flex{display:flex;flex-direction:row;width:100vw;height:100vh;min-height:100vh;min-width:100vw;}"
        ".left-anim{flex:6;display:flex;align-items:center;justify-content:center;height:100vh;background:#e0e6ed;}"
        ".right-panel{flex:4;display:flex;flex-direction:column;justify-content:center;align-items:center;height:100vh;background:#fff;box-shadow:-2px 0 12px rgba(38,49,89,0.06);}"
        "#sim{display:block;width:96%;height:92vh;max-width:100vw;max-height:98vh;border-radius:14px;box-shadow:0 2px 8px rgba(38,49,89,0.08);}"
        ".panel-content{width:92%;max-width:420px;}"
        ".panel-title{font-size:2rem;color:#263159;font-weight:700;margin-bottom:18px;text-align:center;}"
        ".servo-angles{font-size:1.1em;color:#263159;text-align:center;margin-bottom:18px;line-height:1.3;}"
        ".row{display:flex;align-items:center;margin:14px 0;}"
        ".row label{width:170px;color:#3a3a3a;font-size:1rem;}"
        ".slider-wrap{flex:1;display:flex;align-items:center;}"
        ".slider{width:100%;margin:0 10px;appearance:none;height:6px;background:#e0e6ed;border-radius:4px;outline:none;transition:background 0.3s;}"
        ".slider::-webkit-slider-thumb{appearance:none;width:22px;height:22px;border-radius:50%;background:#4356e0;box-shadow:0 2px 6px rgba(67,86,224,0.15);border:2px solid #fff;cursor:pointer;transition:background 0.2s;}"
        ".slider:focus::-webkit-slider-thumb{background:#263159;}"
        ".slider::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:#4356e0;border:2px solid #fff;cursor:pointer;}"
        ".slider:focus::-moz-range-thumb{background:#263159;}"
        ".slider::-ms-thumb{width:22px;height:22px;border-radius:50%;background:#4356e0;border:2px solid #fff;cursor:pointer;}"
        ".val{width:44px;text-align:right;font-size:1.1em;color:#4356e0;font-weight:500;}"
        ".toggle{position:relative;width:48px;height:28px;margin-left:10px;}"
        ".toggle input{display:none;}"
        ".slider-toggle{position:absolute;top:0;left:0;width:48px;height:28px;background:#cfd8dc;border-radius:28px;cursor:pointer;transition:background 0.2s;}"
        ".slider-toggle:before{content:'';position:absolute;height:24px;width:24px;left:2px;top:2px;background:#fff;border-radius:50%;transition:.2s;box-shadow:0 2px 6px rgba(38,49,89,0.10);}"
        "input:checked+.slider-toggle{background:#4356e0;}"
        "input:checked+.slider-toggle:before{transform:translateX(20px);}"
        "button{margin-top:18px;background:#4356e0;color:#fff;border:none;padding:10px 28px;border-radius:6px;font-size:1.1em;font-weight:500;cursor:pointer;box-shadow:0 2px 8px rgba(67,86,224,0.10);transition:background 0.2s;}"
        "button:hover{background:#263159;}"
        "#ctrl{margin-top:10px;}"
        "@media(max-width:900px){.main-flex{flex-direction:column;}.left-anim,.right-panel{height:auto;min-height:0;}.left-anim{height:40vh;}.right-panel{height:60vh;}}"
        "</style></head><body>"
        "<div class='main-flex'>"
        "<div class='left-anim'><canvas id='sim'></canvas></div>"
        "<div class='right-panel'><div class='panel-content'>"
        "<div class='panel-title'>7-DOF Servo Dashboard</div>"
        "<div class='servo-angles' id='ang'></div>"
        "<div class='row' style='justify-content:flex-end;margin-bottom:10px;'><span style='color:#263159;font-weight:500;'>Manual Mode</span>"
        "<label class='toggle'>"
        "<input id='mode' type='checkbox'><span class='slider-toggle'></span>"
        "</label></div>"
        "<div id='ctrl' style='display:none'>"
        "<h3 style='margin:12px 0 6px 0;color:#3949ab'>Base & Joints</h3>"
        "<div class='row'><label>Base Yaw (S1)</label><div class='slider-wrap'><input id='r0' class='slider' type='range' min='0' max='180'><span class='val' id='v0'>90</span>°</div></div>"
        "<div class='row'><label>Shoulder Pitch (S2 / S3 mirrors)</label><div class='slider-wrap'><input id='r1' class='slider' type='range' min='0' max='180'><span class='val' id='v1'>90</span>°</div></div>"
        "<div class='row'><label>Elbow Pitch (S4)</label><div class='slider-wrap'><input id='r2' class='slider' type='range' min='0' max='180'><span class='val' id='v2'>90</span>°</div></div>"
        "<h3 style='margin:12px 0 6px 0;color:#3949ab'>Arm Linkage</h3>"
        "<div class='row'><label>Wrist Pitch (S5)</label><div class='slider-wrap'><input id='r3' class='slider' type='range' min='0' max='180'><span class='val' id='v3'>90</span>°</div></div>"
        "<div class='row'><label>Gripper Yaw (S6)</label><div class='slider-wrap'><input id='r4' class='slider' type='range' min='0' max='180'><span class='val' id='v4'>90</span>°</div></div>"
        "<h3 style='margin:12px 0 6px 0;color:#3949ab'>Gripper</h3>"
        "<div class='row'><label>Gripper Open/Close (S7)</label><div class='slider-wrap'><input id='r5' class='slider' type='range' min='0' max='180'><span class='val' id='v5'>90</span>°</div></div>"
        "<div style='text-align:center;'><button onclick='send()'>Send</button></div>"
        "</div>"
        "</div></div>"
        "</div>"
        "<script src='/three.js'></script><script>"
        "const m=document.getElementById('mode'), box=document.getElementById('ctrl');"
        "const ang=document.getElementById('ang');"
        "const r=[0,1,2,3,4,5].map(i=>document.getElementById('r'+i));"
        "const v=[0,1,2,3,4,5].map(i=>document.getElementById('v'+i));"
        "r.forEach((s,i)=>s.oninput=()=>v[i].textContent=s.value);"
        "window.addEventListener('DOMContentLoaded',()=>{"
        "  m.checked = true; box.style.display = 'block';"
        "});"
        "m.onchange=_=>{box.style.display=m.checked?'block':'none';fetch('/mode',{method:'POST',body:m.checked?'1':'0'});};"
        "function send(){const p=`p1:${r[0].value},p2:${r[1].value},p3:${r[2].value},p4:${r[3].value},p5:${r[4].value}`;fetch('/manual',{method:'POST',body:p});fetch('/gripper/angle',{method:'POST',body:r[5].value});}"
        "function loop(){fetch('/angles').then(res=>res.json()).then(j=>{const d=j.d;"
        "ang.innerHTML = `<div>S1 ${d[0]}° S2 ${d[1]}° S3 ${d[2]}°<br>S4 ${d[3]}° S5 ${d[4]}° S6 ${d[5]}° S7 ${d[6]}°</div>`;"
        "if(!m.checked) r.forEach((s,i)=>{s.value=d[i];v[i].textContent=d[i];});"
        "updateJointTransforms(d);});}"
        "setInterval(loop,700); loop();"
        "const canvas=document.getElementById('sim');"
        "const renderer=new THREE.WebGLRenderer({canvas,antialias:true});"
        "renderer.setPixelRatio(window.devicePixelRatio||1);"
        "const scene=new THREE.Scene(); scene.background=new THREE.Color(0xe0e6ed);"
        "const camera=new THREE.PerspectiveCamera(75,1.5,0.1,100);"
        "camera.position.set(1.1,2.5,3.2);"
        "scene.add(new THREE.AmbientLight(0x888888));"
        "const dl=new THREE.DirectionalLight(0xffffff,0.8);dl.position.set(5,10,5);scene.add(dl);"
        "scene.add(new THREE.GridHelper(10,10));scene.add(new THREE.AxesHelper(2));"
        "const mat=new THREE.MeshStandardMaterial({color:0x4356e0});"
        "const jointYaw=new THREE.Object3D();jointYaw.position.x=0.5;scene.add(jointYaw);"
        "const base=new THREE.Mesh(new THREE.BoxGeometry(0.6,0.2,0.6),mat.clone());"
        "base.geometry.translate(0,0.1,0);jointYaw.add(base);"
        "jointYaw.position.y = 0.7;"
        "const jointTilt=new THREE.Object3D();jointTilt.position.y=0.2;base.add(jointTilt);"
        "const link1=new THREE.Mesh(new THREE.BoxGeometry(0.2,1,0.2),mat.clone());"
        "link1.geometry.translate(0,0.5,0);jointTilt.add(link1);"
        "const jointB=new THREE.Object3D();jointB.position.y=1;link1.add(jointB);"
        "const link2=new THREE.Mesh(new THREE.BoxGeometry(0.2,1,0.2),mat.clone());"
        "link2.geometry.translate(0,0.5,0);jointB.add(link2);"
        "const jointC=new THREE.Object3D();jointC.position.y=1;link2.add(jointC);"
        "const link3=new THREE.Mesh(new THREE.BoxGeometry(0.2,0.7,0.2),mat.clone());"
        "link3.geometry.translate(0,0.35,0);jointC.add(link3);"
        "const jawGroup=new THREE.Object3D();jawGroup.position.y=0.7;link3.add(jawGroup);"
        "const jawL=new THREE.Mesh(new THREE.BoxGeometry(0.05,0.3,0.1),mat.clone());"
        "jawL.position.set(-0.06,0.15,0);jawGroup.add(jawL);"
        "const jawR=new THREE.Mesh(new THREE.BoxGeometry(0.05,0.3,0.1),mat.clone());"
        "jawR.position.set(0.06,0.15,0);jawGroup.add(jawR);"
        "function updateJointTransforms(a){"
        "jointYaw.rotation.y = THREE.MathUtils.degToRad(a[0]);"
        "jointTilt.rotation.x = THREE.MathUtils.degToRad(a[1]-90);"
        "jointB.rotation.x = THREE.MathUtils.degToRad(a[3]-45);"
        "jointC.rotation.x = THREE.MathUtils.degToRad(a[4]-45);"
        "const grip = (a[6]-90)/180 * 0.12;"
        "jawL.position.x = -0.06 - grip;"
        "jawR.position.x = 0.06 + grip;"
        "jawGroup.rotation.y = THREE.MathUtils.degToRad(a[5]);"
        "}"
        "function resize(){const w=canvas.clientWidth, h=canvas.clientHeight;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);}"
        "window.addEventListener('resize',resize);resize();"
        "function render(){renderer.render(scene,camera);requestAnimationFrame(render);}"
        "render();updateJointTransforms([108,133,47,80,90,46,0]);"
        "</script></body></html>";

    httpd_resp_send(r, pg, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// serve embedded three.js
static esp_err_t threejs_handler(httpd_req_t *r)
{
    httpd_resp_set_type(r, "application/javascript");
    httpd_resp_send(r, (const char *)three_js, three_js_len);
    return ESP_OK;
}

static void webserver_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t h = NULL;
    httpd_start(&h, &cfg);
    httpd_register_uri_handler(h, &(httpd_uri_t){"/", HTTP_GET, index_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/update", HTTP_POST, ext_post_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/manual", HTTP_POST, man_post_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/mode", HTTP_POST, mode_post_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/angles", HTTP_GET, angles_get_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/three.js", HTTP_GET, threejs_handler, NULL});
    httpd_register_uri_handler(h, &(httpd_uri_t){"/gripper/angle", HTTP_POST, py_post_handler, NULL});
}

static void wifi_softap_start(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t ap = {
        .ap = {
            .ssid = "ServoESP",
            .password = "12345678",
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = 4}};
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();
}

static void led_task(void *arg)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1)
    {
        if (led_pulse_request)
        {
            led_pulse_request = false;
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_PULSE_MS));
            gpio_set_level(LED_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    nvs_flash_init();
    wifi_softap_start();
    webserver_start();
    servo_channel_init();
    xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 5, NULL, tskNO_AFFINITY);

    while (1)
    {
        float raw[5];
        for (int i = 0; i < 5; i++)
        {
            raw[i] = manual_mode ? man_deg[i] : ext_deg[i];
        }

        // Apply center-offsets only in sensor (ext) mode
        float src[5];
        for (int i = 0; i < 5; i++)
        {
            if (!manual_mode)
                src[i] = raw[i] - SERVO_OFFSET[i];
            else
                src[i] = raw[i];
        }

        servo_write_deg(SERVO_CH[0], src[0]);          // S1 = p1
        servo_write_deg(SERVO_CH[1], src[1]);          // S2 = p2
        servo_write_deg(SERVO_CH[2], 180.0f - src[1]); // S3 = mirror of p2
        servo_write_deg(SERVO_CH[3], src[2]);          // S4 = p3
        servo_write_deg(SERVO_CH[4], src[3]);          // S5 = p4
        servo_write_deg(SERVO_CH[5], src[4]);          // S6 = p5
        servo_write_deg(SERVO_CH[6], s7_angle);        // S7 = gripper

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
