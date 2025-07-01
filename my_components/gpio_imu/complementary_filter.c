#include "complementary_filter.h"

static int is_accel_valid(float accel[3], float threshold) {
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    return fabsf(norm - 9.81f) < threshold;
}

void init_quaternion_imu(QuaternionIMU* imu, float alpha) {
    imu->orientation = quat_identity();
    imu->alpha = alpha;
    imu->gyro_bias[0] = imu->gyro_bias[1] = imu->gyro_bias[2] = 0.0f;
    imu->accel_threshold = 3.0f;
}

void update_quaternion_imu(QuaternionIMU* imu, float accel[3], float gyro[3], float dt) {
    float gyro_corrected[3] = {
        gyro[0] - imu->gyro_bias[0],
        gyro[1] - imu->gyro_bias[1],
        gyro[2] - imu->gyro_bias[2]
    };

    Quaternion q_gyro_predicted = imu->orientation;
    float gyro_norm = sqrtf(
        gyro_corrected[0]*gyro_corrected[0] +
        gyro_corrected[1]*gyro_corrected[1] +
        gyro_corrected[2]*gyro_corrected[2]);

    if (gyro_norm > 1e-6f) {
        float half_angle = gyro_norm * dt * 0.5f;
        float sin_half = sinf(half_angle);
        float cos_half = cosf(half_angle);
        Quaternion q_gyro = {
            cos_half,
            (gyro_corrected[0]/gyro_norm) * sin_half,
            (gyro_corrected[1]/gyro_norm) * sin_half,
            (gyro_corrected[2]/gyro_norm) * sin_half
        };
        q_gyro_predicted = quat_multiply(imu->orientation, q_gyro);
        quat_normalize(&q_gyro_predicted);
    }

    float gyro_magnitude = gyro_norm * 180.0f / M_PI;
    float adaptive_alpha = imu->alpha;
    if (gyro_magnitude > 50.0f) adaptive_alpha = fminf(0.99f, imu->alpha + 0.05f);
    else if (gyro_magnitude < 5.0f) adaptive_alpha = fmaxf(0.85f, imu->alpha - 0.1f);

    if (is_accel_valid(accel, imu->accel_threshold)) {
        Quaternion q_accel = quat_from_accel(accel[0], accel[1], accel[2]);
        imu->orientation = quat_slerp(q_accel, q_gyro_predicted, adaptive_alpha);
    } else {
        imu->orientation = q_gyro_predicted;
    }

    quat_normalize(&imu->orientation);
}

void estimate_gyro_bias(QuaternionIMU* imu, float gyro[3], float learning_rate) {
    imu->gyro_bias[0] += learning_rate * gyro[0];
    imu->gyro_bias[1] += learning_rate * gyro[1];
    imu->gyro_bias[2] += learning_rate * gyro[2];
}
