#pragma once

#include <stdint.h>

void actuators_init(void);
void actuators_set_esc_us(uint32_t us);     // sets both ESCs
void actuators_set_servo_us(uint32_t us);
