#include "quaternion.h"

Quaternion quat_identity(void) {
    return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f};
}

void quat_normalize(Quaternion* q) {
    float norm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        q->w *= inv; q->x *= inv; q->y *= inv; q->z *= inv;
    }
}

Quaternion quat_multiply(Quaternion q1, Quaternion q2) {
    return (Quaternion){
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
}

Quaternion quat_conjugate(Quaternion q) {
    return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

float quat_dot(Quaternion q1, Quaternion q2) {
    return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

Quaternion quat_from_accel(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return quat_identity();
    ax /= norm; ay /= norm; az /= norm;
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    return (Quaternion){cr * cp, sr * cp, cr * sp, -sr * sp};
}

Quaternion quat_slerp(Quaternion q1, Quaternion q2, float t) {
    float dot = quat_dot(q1, q2);
    if (dot < 0.0f) { q2.w = -q2.w; q2.x = -q2.x; q2.y = -q2.y; q2.z = -q2.z; dot = -dot; }
    if (dot > 0.9995f) {
        Quaternion result = {
            q1.w + t * (q2.w - q1.w),
            q1.x + t * (q2.x - q1.x),
            q1.y + t * (q2.y - q1.y),
            q1.z + t * (q2.z - q1.z)
        };
        quat_normalize(&result);
        return result;
    }
    float theta_0 = acosf(dot);
    float sin_theta_0 = sinf(theta_0);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);
    float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;
    return (Quaternion){
        s0*q1.w + s1*q2.w,
        s0*q1.x + s1*q2.x,
        s0*q1.y + s1*q2.y,
        s0*q1.z + s1*q2.z
    };
}

float quat_to_yaw(Quaternion q) {
    float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    return atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

float quat_to_roll(Quaternion q) {
    float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    return atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
}

float quat_to_pitch(Quaternion q) {
    // Using atan2 with both sin and cos components
    float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    float cosp = 1.0f - 2.0f * (q.y*q.y + q.x*q.x);
    return atan2f(sinp, cosp) * 180.0f / M_PI;
}

// Add these functions to your quaternion.c file

// Extract independent pitch and roll from quaternion using gravity vector
// This gives you true tilt angles that don't couple with yaw rotation
float quat_to_roll_independent(Quaternion q) {
    // Extract gravity vector components in body frame
    float gy = 2.0f * (q.w*q.x + q.y*q.z);
    float gz = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    
    // Roll is rotation around X-axis (forward/back axis)
    return atan2f(gy, gz) * 180.0f / M_PI;
}



float quat_to_pitch_independent(Quaternion q) {
    // Extract gravity vector components in body frame
    float gx = 2.0f * (q.x*q.z - q.w*q.y);
    float gy = 2.0f * (q.w*q.x + q.y*q.z);
    float gz = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    
    // Pitch is rotation around Y-axis (left/right axis)
    return atan2f(-gx, sqrtf(gy*gy + gz*gz)) * 180.0f / M_PI;
}
