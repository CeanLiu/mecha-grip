#include "my_servo.h"

#define SERVO_TIMER              LEDC_TIMER_0
#define SERVO_MODE               LEDC_LOW_SPEED_MODE
#define SERVO_FREQ_HZ            50                  // 50Hz for servo
#define SERVO_DUTY_RES           LEDC_TIMER_16_BIT   // Resolution
#define GPIO_SHOULDER_RIGHT      13
#define GPIO_SHOULDER_LEFT       26

static const char *TAG = "my_servo";
// Internal struct to map servo pins and channels
typedef struct {
    int gpio_num;
    ledc_channel_t channel;
} servo_config_t;

// You can change these pins to your actual GPIOs
static const servo_config_t servo_configs[SERVO_COUNT] = {
    { .gpio_num = GPIO_SHOULDER_RIGHT, .channel = LEDC_CHANNEL_0 },
    { .gpio_num = GPIO_SHOULDER_LEFT, .channel = LEDC_CHANNEL_1 },
};

static uint32_t angle_to_duty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    const uint32_t min_duty = (1 << 16) * 2.0 / 100.0;   // 2.0%
    const uint32_t max_duty = (1 << 16) * 12.5 / 100.0;  // 12.5%

    return min_duty + ((max_duty - min_duty) * angle) / 180;
}

int clampAngle(float euler_angle){
    
    if (euler_angle < -90.0f) euler_angle = -90.0f;
    if (euler_angle > 90.0f) euler_angle = 90.0f;

    // Map [-90, 90] to [0, 180]
    int servo_angle = (int)(euler_angle + 90.0f);

    // Safety clamps
    if (servo_angle < 0) servo_angle = 0;
    if (servo_angle > 180) servo_angle = 180;
    return servo_angle;
        
}

esp_err_t my_servo_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode       = SERVO_MODE,
        .duty_resolution  = SERVO_DUTY_RES,
        .timer_num        = SERVO_TIMER,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel = {
            .gpio_num   = servo_configs[i].gpio_num,
            .speed_mode = SERVO_MODE,
            .channel    = servo_configs[i].channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = SERVO_TIMER,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    ESP_LOGI(TAG, "Servo initialization complete");
    return ESP_OK;
}

esp_err_t set_servo_angle(servo_id_t id, int angle) {
    if (id < 0 || id >= SERVO_COUNT) {
        ESP_LOGE(TAG, "Invalid servo ID: %d", id);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(SERVO_MODE, servo_configs[id].channel, duty);
    ledc_update_duty(SERVO_MODE, servo_configs[id].channel);

    return ESP_OK;
}
