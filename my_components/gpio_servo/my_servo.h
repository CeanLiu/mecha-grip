#ifndef MY_SERVO_H
#define MY_SERVO_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

typedef enum {
    SERVO_0 = 0,
    SERVO_1,
    SERVO_COUNT
} servo_id_t;

esp_err_t my_servo_init(void);
esp_err_t set_servo_angle(servo_id_t id, int angle);
int clampAngle(float euler_angle);
#endif // MY_SERVO_H
