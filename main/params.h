#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    MODE_MANUAL = 0,
    MODE_ASSIST = 1,    // heading hold when steering is centered
    MODE_RATE   = 2,    // stick sets target yaw rate
    MODE_COUNT
} drive_mode_t;

// gains output normalized steering (-1..1)
typedef struct {
    uint32_t mode;

    float hold_kp;          // per deg
    float hold_ki;
    float hold_kd;          // per deg/s

    float rate_kp;          // per deg/s
    float rate_ki;
    float rate_ff;
    float max_rate_dps;

    float assist_limit;
    float throttle_limit;
    float throttle_slew;    // per second
    float tip_angle_deg;
    float servo_trim_us;
    float servo_travel_deg;
    float yaw_sign;
} params_t;

typedef struct {
    const char *name;
    float *value;
    float min, max;
    const char *help;
} param_def_t;

extern params_t g_params;

void params_init(void);
void params_reset(void);
int params_save(void);
const param_def_t *params_table(size_t *count);
const char *params_mode_name(uint32_t mode);
