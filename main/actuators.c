#include "actuators.h"

#include "driver/ledc.h"

#include "config.h"

#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_RES_BITS    LEDC_TIMER_16_BIT
#define PWM_PERIOD_US   (1000000 / PWM_FREQ_HZ)

enum { CH_ESC1 = LEDC_CHANNEL_0, CH_ESC2 = LEDC_CHANNEL_1, CH_SERVO = LEDC_CHANNEL_2 };

static uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void write_us(ledc_channel_t ch, uint32_t us)
{
    uint32_t duty = (uint32_t)(((uint64_t)us * ((1U << 16) - 1U)) / PWM_PERIOD_US);
    ledc_set_duty(PWM_MODE, ch, duty);
    ledc_update_duty(PWM_MODE, ch);
}

static void add_channel(ledc_channel_t ch, gpio_num_t pin)
{
    ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = PWM_MODE,
        .channel = ch,
        .timer_sel = PWM_TIMER,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

void actuators_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES_BITS,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    add_channel(CH_ESC1, PIN_ESC1);
    add_channel(CH_ESC2, PIN_ESC2);
    add_channel(CH_SERVO, PIN_SERVO);

    actuators_set_esc_us(ESC_NEUTRAL_US);
    actuators_set_servo_us(SERVO_NEUTRAL_US);
}

void actuators_set_esc_us(uint32_t us)
{
    us = clamp_u32(us, ESC_NEUTRAL_US - ESC_SPAN_US, ESC_NEUTRAL_US + ESC_SPAN_US);
    write_us(CH_ESC1, us);
    write_us(CH_ESC2, us);
}

void actuators_set_servo_us(uint32_t us)
{
    write_us(CH_SERVO, clamp_u32(us, SERVO_MIN_US, SERVO_MAX_US));
}
