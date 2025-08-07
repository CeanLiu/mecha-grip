// ================= quaternion.h =================
#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float w, x, y, z;
} Quaternion;

Quaternion quat_identity(void);
void quat_normalize(Quaternion* q);
Quaternion quat_multiply(Quaternion q1, Quaternion q2);
Quaternion quat_conjugate(Quaternion q);
float quat_dot(Quaternion q1, Quaternion q2);
Quaternion quat_from_accel(float ax, float ay, float az);
Quaternion quat_slerp(Quaternion q1, Quaternion q2, float t);

float quat_to_yaw(Quaternion q);
float quat_to_pitch(Quaternion q);
float quat_to_roll(Quaternion q);

float quat_to_roll_independent(Quaternion q);
float quat_to_pitch_independent(Quaternion q);

#endif