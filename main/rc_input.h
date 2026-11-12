#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float steer;        // -1..1, 0 in deadband
    float throttle;     // -1..1, 0 in deadband
    uint32_t steer_us;
    uint32_t throttle_us;
    bool ok;            // false if signal lost
} rc_cmd_t;

void rc_input_init(void);
void rc_input_read(rc_cmd_t *out);
