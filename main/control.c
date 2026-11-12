#include "control.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "actuators.h"
#include "config.h"
#include "imu.h"
#include "params.h"
#include "pid.h"
#include "rc_input.h"

#define MIN_CLOSED_LOOP_THROTTLE  0.05f

static control_snapshot_t s_snap;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float signf(float x)
{
    return x >= 0.0f ? 1.0f : -1.0f;
}

// only speeding up is ramped, slowing down is instant. direction changes go through 0
static float ramp_throttle(float current, float target, float max_step)
{
    if (target * current < 0.0f) return 0.0f;
    if (fabsf(target) <= fabsf(current)) return target;
    return current + clampf(target - current, -max_step, max_step);
}

static uint32_t steer_to_servo_us(float steer)
{
    float half_range = (SERVO_MAX_US - SERVO_NEUTRAL_US) * g_params.servo_travel_deg / SERVO_SPEC_TRAVEL_DEG;
    if (SERVO_REVERSE) steer = -steer;
    float us = SERVO_NEUTRAL_US + g_params.servo_trim_us + steer * half_range;
    return (uint32_t)clampf(us, SERVO_MIN_US, SERVO_MAX_US);
}

// solid = driving, slow blink = disarmed / imu calibrating, fast blink = failsafe / tipped
static void update_led(drive_state_t state, bool imu_ok, uint32_t loop)
{
    int level;
    if (state == STATE_FAILSAFE || state == STATE_TIPPED) {
        level = (loop / 5) % 2;
    } else if (state == STATE_DISARMED || !imu_ok) {
        level = (loop / 25) % 2;
    } else {
        level = 1;
    }
    gpio_set_level(PIN_LED, level);
}

static void control_task(void *arg)
{
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    const int tip_loops = TIP_DETECT_MS / CONTROL_PERIOD_MS;

    drive_state_t state = STATE_DISARMED;
    pid_ctrl_t hold_pid = {0}, rate_pid = {0};
    bool holding = false;
    float heading_ref = 0.0f;
    float throttle = 0.0f;
    uint32_t last_mode = g_params.mode;
    int tip_count = 0;
    uint32_t loop = 0;

    TickType_t wake = xTaskGetTickCount();
    while (1) {
        const params_t *p = &g_params;

        rc_cmd_t rc;
        rc_input_read(&rc);
        imu_state_t imu;
        imu_get(&imu);

        int64_t now_us = esp_timer_get_time();
        bool imu_ok = imu.valid && (now_us - imu.updated_us) < IMU_STALE_MS * 1000LL;
        float yaw_sign = signf(p->yaw_sign);
        float yaw_rate = yaw_sign * imu.rate[2];
        float heading = yaw_sign * imu.yaw;

        bool tilted = imu_ok && (fabsf(imu.roll) > p->tip_angle_deg || fabsf(imu.pitch) > p->tip_angle_deg);
        tip_count = tilted ? tip_count + 1 : 0;

        if (!rc.ok) {
            state = STATE_FAILSAFE;
        } else if (tip_count >= tip_loops) {
            state = STATE_TIPPED;
        } else if (state == STATE_FAILSAFE || state == STATE_TIPPED) {
            state = STATE_DISARMED;
        }
        if (state == STATE_DISARMED && rc.ok && rc.throttle == 0.0f) {
            state = STATE_DRIVING;
        }

        float throttle_target = (state == STATE_DRIVING) ? rc.throttle * p->throttle_limit : 0.0f;
        throttle = ramp_throttle(throttle, throttle_target, p->throttle_slew * dt);

        if (p->mode != last_mode) {
            pid_reset(&hold_pid);
            pid_reset(&rate_pid);
            holding = false;
            last_mode = p->mode;
        }

        float stick = (state == STATE_FAILSAFE) ? 0.0f : rc.steer;
        float steer = stick;
        float correction = 0.0f;
        float target = 0.0f;
        // forward only: in reverse steering turns the car the other way
        bool can_close_loop = state == STATE_DRIVING && imu_ok && throttle > MIN_CLOSED_LOOP_THROTTLE;
        bool closed_loop = false;

        if (can_close_loop && p->mode == MODE_ASSIST && stick == 0.0f) {
            if (!holding) {
                heading_ref = heading;
                pid_reset(&hold_pid);
                holding = true;
            }
            target = heading_ref - heading;
            correction = pid_update(&hold_pid, p->hold_kp, p->hold_ki, p->hold_kd,
                                    target, yaw_rate, dt, p->assist_limit);
            steer = correction;
            closed_loop = true;
        } else if (can_close_loop && p->mode == MODE_RATE) {
            target = stick * p->max_rate_dps;
            correction = pid_update(&rate_pid, p->rate_kp, p->rate_ki, 0.0f,
                                    target - yaw_rate, 0.0f, dt, p->assist_limit);
            steer = p->rate_ff * stick + correction;
            closed_loop = true;
        } else {
            holding = false;
            pid_reset(&hold_pid);
            pid_reset(&rate_pid);
        }
        steer = clampf(steer, -1.0f, 1.0f);

        uint32_t servo_us = steer_to_servo_us(steer);
        uint32_t esc_us = (uint32_t)lroundf(ESC_NEUTRAL_US + throttle * ESC_SPAN_US);
        actuators_set_servo_us(servo_us);
        actuators_set_esc_us(esc_us);
        update_led(state, imu_ok, loop++);

        control_snapshot_t snap = {
            .t_ms = (uint32_t)(now_us / 1000),
            .state = state,
            .mode = p->mode,
            .imu_ok = imu_ok,
            .closed_loop = closed_loop,
            .rx_steer_us = rc.steer_us,
            .rx_throttle_us = rc.throttle_us,
            .stick_steer = rc.steer,
            .stick_throttle = rc.throttle,
            .throttle = throttle,
            .steer = steer,
            .correction = correction,
            .target = target,
            .yaw_rate = yaw_rate,
            .heading = heading,
            .roll = imu.roll,
            .pitch = imu.pitch,
            .servo_us = servo_us,
            .esc_us = esc_us,
        };
        taskENTER_CRITICAL(&s_lock);
        s_snap = snap;
        taskEXIT_CRITICAL(&s_lock);

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void control_start(void)
{
    gpio_config_t led = {.pin_bit_mask = 1ULL << PIN_LED, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&led));
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, TASK_PRIO_CONTROL, NULL, 1);
}

void control_get_snapshot(control_snapshot_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_snap;
    taskEXIT_CRITICAL(&s_lock);
}

const char *control_state_name(drive_state_t s)
{
    switch (s) {
    case STATE_DISARMED: return "DISARMED";
    case STATE_DRIVING:  return "DRIVING";
    case STATE_FAILSAFE: return "FAILSAFE";
    case STATE_TIPPED:   return "TIPPED";
    default:             return "?";
    }
}
