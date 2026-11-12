#pragma once

#include "driver/gpio.h"

// pins
#define PIN_ESC1            GPIO_NUM_25
#define PIN_ESC2            GPIO_NUM_26
#define PIN_SERVO           GPIO_NUM_27
#define PIN_LED             GPIO_NUM_2
#define PIN_RX_STEER        GPIO_NUM_34
#define PIN_RX_THROTTLE     GPIO_NUM_35
#define PIN_IMU_SDA         GPIO_NUM_21
#define PIN_IMU_SCL         GPIO_NUM_22

// receiver, measured from my transmitter
#define RX_STEER_MIN_US         1030
#define RX_STEER_CENTER_US      1598
#define RX_STEER_MAX_US         2140
#define RX_STEER_DEADBAND_US    10

#define RX_THR_NEUTRAL_US       1520
#define RX_THR_DEADBAND_US      20
#define RX_THR_SPAN_US          500

#define RX_VALID_MIN_US         800
#define RX_VALID_MAX_US         2200
#define RX_TIMEOUT_MS           100

// outputs
#define PWM_FREQ_HZ             50

#define ESC_NEUTRAL_US          1500
#define ESC_SPAN_US             500

#define SERVO_MIN_US            500
#define SERVO_NEUTRAL_US        1500
#define SERVO_MAX_US            2500
#define SERVO_SPEC_TRAVEL_DEG   180
#define SERVO_REVERSE           0

// timing
#define CONTROL_PERIOD_MS       20
#define IMU_PERIOD_MS           10
#define IMU_STALE_MS            100
#define TIP_DETECT_MS           300

#define TASK_PRIO_IMU           6
#define TASK_PRIO_CONTROL       5
#define TASK_PRIO_CONSOLE       3
#define TASK_PRIO_TELEMETRY     2
