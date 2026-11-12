#include "params.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE   "rccar"
#define NVS_KEY         "params"
#define PARAMS_VERSION  1   // bump when params_t changes

static const char *TAG = "params";

params_t g_params;

static const params_t DEFAULTS = {
    .mode             = MODE_ASSIST,
    .hold_kp          = 0.020f,
    .hold_ki          = 0.005f,
    .hold_kd          = 0.004f,
    .rate_kp          = 0.004f,
    .rate_ki          = 0.005f,
    .rate_ff          = 1.0f,
    .max_rate_dps     = 180.0f,
    .assist_limit     = 0.4f,
    .throttle_limit   = 1.0f,
    .throttle_slew    = 2.0f,
    .tip_angle_deg    = 60.0f,
    .servo_trim_us    = 0.0f,
    .servo_travel_deg = 75.0f,
    .yaw_sign         = 1.0f,
};

static const param_def_t TABLE[] = {
    {"hold_kp",      &g_params.hold_kp,          0, 1,      "heading hold P"},
    {"hold_ki",      &g_params.hold_ki,          0, 1,      "heading hold I"},
    {"hold_kd",      &g_params.hold_kd,          0, 1,      "heading hold D"},
    {"rate_kp",      &g_params.rate_kp,          0, 1,      "rate mode P"},
    {"rate_ki",      &g_params.rate_ki,          0, 1,      "rate mode I"},
    {"rate_ff",      &g_params.rate_ff,          0, 2,      "rate mode feedforward"},
    {"max_rate",     &g_params.max_rate_dps,     10, 720,   "deg/s at full stick"},
    {"assist_limit", &g_params.assist_limit,     0, 1,      "max controller steering"},
    {"thr_limit",    &g_params.throttle_limit,   0, 1,      "max throttle"},
    {"thr_slew",     &g_params.throttle_slew,    0.1f, 50,  "throttle ramp per second"},
    {"tip_angle",    &g_params.tip_angle_deg,    10, 90,    "tilt that cuts throttle"},
    {"trim",         &g_params.servo_trim_us,    -300, 300, "steering trim (us)"},
    {"travel",       &g_params.servo_travel_deg, 10, 180,   "servo travel (deg)"},
    {"yaw_sign",     &g_params.yaw_sign,         -1, 1,     "+1 or -1"},
};

typedef struct {
    uint32_t version;
    params_t params;
} stored_params_t;

void params_reset(void)
{
    g_params = DEFAULTS;
}

void params_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    params_reset();

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "No saved parameters, using defaults");
        return;
    }
    stored_params_t stored;
    size_t len = sizeof(stored);
    err = nvs_get_blob(h, NVS_KEY, &stored, &len);
    nvs_close(h);

    if (err == ESP_OK && len == sizeof(stored) && stored.version == PARAMS_VERSION) {
        g_params = stored.params;
        ESP_LOGI(TAG, "Loaded saved parameters");
    } else {
        ESP_LOGW(TAG, "Saved parameters invalid, using defaults");
    }
}

int params_save(void)
{
    stored_params_t stored = {.version = PARAMS_VERSION, .params = g_params};
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, NVS_KEY, &stored, sizeof(stored));
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Save failed: %s", esp_err_to_name(err));
        return -1;
    }
    return 0;
}

const param_def_t *params_table(size_t *count)
{
    *count = sizeof(TABLE) / sizeof(TABLE[0]);
    return TABLE;
}

const char *params_mode_name(uint32_t mode)
{
    switch (mode) {
    case MODE_MANUAL: return "manual";
    case MODE_ASSIST: return "assist";
    case MODE_RATE:   return "rate";
    default:          return "?";
    }
}
