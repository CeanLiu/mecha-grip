#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "quaternion.h"

typedef struct {
    Quaternion orientation;
    float alpha;
    float gyro_bias[3];
    float accel_threshold;
} QuaternionIMU;

void init_quaternion_imu(QuaternionIMU* imu, float alpha);
void update_quaternion_imu(QuaternionIMU* imu, float accel[3], float gyro[3], float dt);
void estimate_gyro_bias(QuaternionIMU* imu, float gyro[3], float learning_rate);

#endif // COMPLEMENTARY_FILTER_H

