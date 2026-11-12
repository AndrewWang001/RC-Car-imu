#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATE_DISARMED,
    STATE_DRIVING,
    STATE_FAILSAFE,
    STATE_TIPPED,
} drive_state_t;

typedef struct {
    uint32_t t_ms;
    drive_state_t state;
    uint32_t mode;
    bool imu_ok;
    bool closed_loop;
    uint32_t rx_steer_us, rx_throttle_us;
    float stick_steer, stick_throttle;
    float throttle;
    float steer;
    float correction;
    float target;       // heading error (assist) or target yaw rate (rate)
    float yaw_rate;     // + = right
    float heading;
    float roll, pitch;
    uint32_t servo_us, esc_us;
} control_snapshot_t;

void control_start(void);
void control_get_snapshot(control_snapshot_t *out);
const char *control_state_name(drive_state_t s);
