#pragma once

typedef struct {
    float integ;
} pid_ctrl_t;

void pid_reset(pid_ctrl_t *pid);

// meas_rate is the derivative of the measurement (e.g. gyro), used for D instead of
// differentiating the error
float pid_update(pid_ctrl_t *pid, float kp, float ki, float kd,
                 float err, float meas_rate, float dt, float limit);
