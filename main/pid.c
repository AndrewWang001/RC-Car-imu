#include "pid.h"

#include <stdbool.h>

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integ = 0.0f;
}

float pid_update(pid_ctrl_t *pid, float kp, float ki, float kd,
                 float err, float meas_rate, float dt, float limit)
{
    float p = kp * err;
    float d = -kd * meas_rate;

    // anti-windup: don't integrate further into saturation
    float integ = clampf(pid->integ + ki * err * dt, -limit, limit);
    float out = p + integ + d;
    bool pushing_further = (out > limit && err > 0) || (out < -limit && err < 0);
    if (!pushing_further) {
        pid->integ = integ;
    }

    return clampf(p + pid->integ + d, -limit, limit);
}
