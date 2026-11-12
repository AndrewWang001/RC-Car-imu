#include "rc_input.h"

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"

#include "config.h"

#define MEDIAN_LEN 3

typedef struct {
    gpio_num_t pin;
    int64_t rise_us;
    int64_t last_pulse_us;
    uint32_t pulses[MEDIAN_LEN];
    uint8_t head;
    uint8_t count;
} rc_channel_t;

enum { CH_STEER, CH_THROTTLE, CH_COUNT };

static rc_channel_t s_ch[CH_COUNT] = {
    [CH_STEER]    = {.pin = PIN_RX_STEER},
    [CH_THROTTLE] = {.pin = PIN_RX_THROTTLE},
};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR rx_isr(void *arg)
{
    rc_channel_t *ch = (rc_channel_t *)arg;
    int64_t now = esp_timer_get_time();

    taskENTER_CRITICAL_ISR(&s_lock);
    if (gpio_get_level(ch->pin)) {
        ch->rise_us = now;
    } else if (ch->rise_us > 0) {
        uint32_t width = (uint32_t)(now - ch->rise_us);
        if (width >= RX_VALID_MIN_US && width <= RX_VALID_MAX_US) {
            ch->pulses[ch->head] = width;
            ch->head = (ch->head + 1) % MEDIAN_LEN;
            if (ch->count < MEDIAN_LEN) ch->count++;
            ch->last_pulse_us = now;
        }
    }
    taskEXIT_CRITICAL_ISR(&s_lock);
}

void rc_input_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_RX_STEER) | (1ULL << PIN_RX_THROTTLE),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (int i = 0; i < CH_COUNT; i++) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(s_ch[i].pin, rx_isr, &s_ch[i]));
    }
}

static uint32_t median3(const uint32_t *v, uint8_t n)
{
    if (n == 0) return 0;
    if (n < 3) return v[0];
    uint32_t a = v[0], b = v[1], c = v[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

// min..center..max -> -1..0..1, sides scaled separately since the stick isn't symmetric
static float normalize(uint32_t us, int min, int center, int max, int deadband)
{
    int d = (int)us - center;
    if (d >= -deadband && d <= deadband) return 0.0f;
    float out = (d < 0) ? (float)(d + deadband) / (float)(center - deadband - min)
                        : (float)(d - deadband) / (float)(max - center - deadband);
    return clampf(out, -1.0f, 1.0f);
}

void rc_input_read(rc_cmd_t *out)
{
    uint32_t widths[CH_COUNT];
    bool fresh = true;
    int64_t now = esp_timer_get_time();

    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < CH_COUNT; i++) {
        widths[i] = median3(s_ch[i].pulses, s_ch[i].count);
        if (s_ch[i].count == 0 || now - s_ch[i].last_pulse_us > RX_TIMEOUT_MS * 1000LL) {
            fresh = false;
        }
    }
    taskEXIT_CRITICAL(&s_lock);

    out->steer_us = widths[CH_STEER];
    out->throttle_us = widths[CH_THROTTLE];
    out->ok = fresh;
    if (!fresh) {
        out->steer = 0.0f;
        out->throttle = 0.0f;
        return;
    }
    out->steer = normalize(widths[CH_STEER], RX_STEER_MIN_US, RX_STEER_CENTER_US,
                           RX_STEER_MAX_US, RX_STEER_DEADBAND_US);
    out->throttle = normalize(widths[CH_THROTTLE], RX_THR_NEUTRAL_US - RX_THR_SPAN_US, RX_THR_NEUTRAL_US,
                              RX_THR_NEUTRAL_US + RX_THR_SPAN_US, RX_THR_DEADBAND_US);
}
