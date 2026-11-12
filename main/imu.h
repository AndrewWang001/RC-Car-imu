#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float roll, pitch, yaw;     // deg, yaw is gyro only and drifts
    float rate[3];              // deg/s
    float bias[3];              // deg/s
    bool still;
    bool valid;                 // false until calibrated
    int64_t updated_us;
    uint32_t read_errors;
} imu_state_t;

void imu_start(void);
void imu_get(imu_state_t *out);
