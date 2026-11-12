#include "imu.h"

#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"

#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

#define I2C_FREQ_HZ             400000
#define ACCEL_LSB_PER_G         16384.0f    // +/-2 g
#define GYRO_LSB_PER_DPS        131.0f      // +/-250 deg/s

#define COMP_ALPHA              0.98f

#define CALIB_SAMPLES           200
#define CALIB_MAX_STD_DPS       0.5f
#define CALIB_MAX_TRIES         5

#define KF_Q_RATE               0.02f
#define KF_Q_BIAS               1e-6f
#define KF_R_GYRO               0.03f
#define KF_R_ZUPT               0.001f

#define STILL_EMA_ALPHA         0.05f
#define STILL_MAX_VAR           0.05f
#define STILL_MAX_RATE          1.5f
#define STILL_ACCEL_TOL         0.05f
#define STILL_MIN_SAMPLES       50

#define RAD_TO_DEG              (180.0f / (float)M_PI)

static const char *TAG = "imu";

static imu_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    float a[3];     // g
    float g[3];     // deg/s
} mpu_sample_t;

typedef struct {
    float rate, bias;
    float P[2][2];
} gyro_kf_t;

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static i2c_master_dev_handle_t mpu_open(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_IMU_SDA,
        .scl_io_num = PIN_IMU_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    uint8_t addr = 0;
    while (addr == 0) {
        if (i2c_master_probe(bus, 0x68, 50) == ESP_OK) {
            addr = 0x68;
        } else if (i2c_master_probe(bus, 0x69, 50) == ESP_OK) {
            addr = 0x69;
        } else {
            ESP_LOGW(TAG, "MPU-6050 not found, check wiring (SDA %d, SCL %d)", PIN_IMU_SDA, PIN_IMU_SCL);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    uint8_t who = 0;
    ESP_ERROR_CHECK(reg_read(dev, MPU_REG_WHO_AM_I, &who, 1));
    ESP_LOGI(TAG, "Found 0x%02X, WHO_AM_I 0x%02X", addr, who);

    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_PWR_MGMT_1, 0x80));  // reset
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_PWR_MGMT_1, 0x01));  // wake, gyro PLL clock
    vTaskDelay(pdMS_TO_TICKS(10));

    // 20 Hz DLPF. 44 Hz let motor vibration alias into slow fake rotation at 100 Hz sampling
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_CONFIG, 0x04));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_SMPLRT_DIV, 0x00));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_GYRO_CONFIG, 0x00));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_ACCEL_CONFIG, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
    return dev;
}

static esp_err_t mpu_read(i2c_master_dev_handle_t dev, mpu_sample_t *s)
{
    uint8_t d[14];
    esp_err_t err = reg_read(dev, MPU_REG_ACCEL_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) return err;
    for (int i = 0; i < 3; i++) {
        s->a[i] = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]) / ACCEL_LSB_PER_G;
        s->g[i] = (int16_t)((d[8 + 2 * i] << 8) | d[9 + 2 * i]) / GYRO_LSB_PER_DPS;
    }
    return ESP_OK;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median(float *v, int n)
{
    qsort(v, n, sizeof(float), cmp_float);
    return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

// mean of samples within 3 sigma of the median (sigma from MAD), so a bump during
// calibration doesn't throw off the bias
static float robust_mean(const float *x, int n, float *std_out)
{
    static float tmp[CALIB_SAMPLES];
    for (int i = 0; i < n; i++) tmp[i] = x[i];
    float med = median(tmp, n);
    for (int i = 0; i < n; i++) tmp[i] = fabsf(x[i] - med);
    float sigma = fmaxf(1.4826f * median(tmp, n), 0.01f);

    float sum = 0, sum2 = 0;
    int k = 0;
    for (int i = 0; i < n; i++) {
        if (fabsf(x[i] - med) <= 3.0f * sigma) {
            sum += x[i];
            sum2 += x[i] * x[i];
            k++;
        }
    }
    float mean = sum / k;
    *std_out = sqrtf(fmaxf(sum2 / k - mean * mean, 0.0f));
    return mean;
}

static void calibrate_gyro(i2c_master_dev_handle_t dev, float bias[3])
{
    static float buf[3][CALIB_SAMPLES];
    mpu_sample_t s;

    for (int attempt = 1; attempt <= CALIB_MAX_TRIES; attempt++) {
        ESP_LOGI(TAG, "Calibrating gyro (try %d), keep still", attempt);
        int n = 0;
        while (n < CALIB_SAMPLES) {
            if (mpu_read(dev, &s) == ESP_OK) {
                for (int a = 0; a < 3; a++) buf[a][n] = s.g[a];
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
        }

        float std[3];
        bool ok = true;
        for (int a = 0; a < 3; a++) {
            bias[a] = robust_mean(buf[a], n, &std[a]);
            if (std[a] > CALIB_MAX_STD_DPS) ok = false;
        }
        ESP_LOGI(TAG, "Bias %.3f %.3f %.3f, noise %.3f %.3f %.3f",
                 bias[0], bias[1], bias[2], std[0], std[1], std[2]);
        if (ok) return;
        ESP_LOGW(TAG, "Moved during calibration, retrying");
    }
    ESP_LOGW(TAG, "Calibration didn't settle, using last estimate");
}

// 2-state Kalman per axis: [rate, bias]
static void kf_init(gyro_kf_t *kf, float bias0)
{
    *kf = (gyro_kf_t){.rate = 0, .bias = bias0, .P = {{1.0f, 0}, {0, 0.01f}}};
}

static void kf_update(gyro_kf_t *kf, float h0, float h1, float z, float R)
{
    float Ph0 = kf->P[0][0] * h0 + kf->P[0][1] * h1;
    float Ph1 = kf->P[1][0] * h0 + kf->P[1][1] * h1;
    float S = h0 * Ph0 + h1 * Ph1 + R;
    float K0 = Ph0 / S, K1 = Ph1 / S;
    float y = z - (h0 * kf->rate + h1 * kf->bias);

    kf->rate += K0 * y;
    kf->bias += K1 * y;
    kf->P[0][0] -= K0 * Ph0;  kf->P[0][1] -= K0 * Ph1;
    kf->P[1][0] -= K1 * Ph0;  kf->P[1][1] -= K1 * Ph1;
}

static void kf_step(gyro_kf_t *kf, float gyro_raw, bool still)
{
    kf->P[0][0] += KF_Q_RATE;
    kf->P[1][1] += KF_Q_BIAS;
    kf_update(kf, 1.0f, 1.0f, gyro_raw, KF_R_GYRO);
    if (still) {
        kf_update(kf, 1.0f, 0.0f, 0.0f, KF_R_ZUPT);     // rate is 0 when still, lets bias track drift
    }
}

static void imu_task(void *arg)
{
    i2c_master_dev_handle_t mpu = mpu_open();

    float bias0[3];
    calibrate_gyro(mpu, bias0);

    gyro_kf_t kf[3];
    float ema_mean[3], ema_var[3] = {0};
    for (int a = 0; a < 3; a++) {
        kf_init(&kf[a], bias0[a]);
        ema_mean[a] = bias0[a];
    }

    mpu_sample_t s;
    float roll = 0, pitch = 0, yaw = 0;
    if (mpu_read(mpu, &s) == ESP_OK) {
        roll = atan2f(s.a[1], s.a[2]) * RAD_TO_DEG;
        pitch = atan2f(-s.a[0], sqrtf(s.a[1] * s.a[1] + s.a[2] * s.a[2])) * RAD_TO_DEG;
    }

    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();
    int still_count = 0;
    uint32_t errors = 0;

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));

        if (mpu_read(mpu, &s) != ESP_OK) {
            errors++;
            taskENTER_CRITICAL(&s_lock);
            s_state.read_errors = errors;
            taskEXIT_CRITICAL(&s_lock);
            continue;
        }
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) / 1e6f;
        last_us = now_us;

        bool quiet = true;
        for (int a = 0; a < 3; a++) {
            float d = s.g[a] - ema_mean[a];
            ema_mean[a] += STILL_EMA_ALPHA * d;
            ema_var[a] = (1.0f - STILL_EMA_ALPHA) * (ema_var[a] + STILL_EMA_ALPHA * d * d);
            if (ema_var[a] > STILL_MAX_VAR || fabsf(s.g[a] - kf[a].bias) > STILL_MAX_RATE) quiet = false;
        }
        float acc_norm = sqrtf(s.a[0] * s.a[0] + s.a[1] * s.a[1] + s.a[2] * s.a[2]);
        if (fabsf(acc_norm - 1.0f) > STILL_ACCEL_TOL) quiet = false;
        still_count = quiet ? still_count + 1 : 0;
        bool still = still_count >= STILL_MIN_SAMPLES;

        for (int a = 0; a < 3; a++) kf_step(&kf[a], s.g[a], still);

        float gx = s.g[0] - kf[0].bias;
        float gy = s.g[1] - kf[1].bias;
        float gz = s.g[2] - kf[2].bias;

        float acc_roll = atan2f(s.a[1], s.a[2]) * RAD_TO_DEG;
        float acc_pitch = atan2f(-s.a[0], sqrtf(s.a[1] * s.a[1] + s.a[2] * s.a[2])) * RAD_TO_DEG;
        roll = COMP_ALPHA * (roll + gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
        pitch = COMP_ALPHA * (pitch + gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
        yaw += gz * dt;

        taskENTER_CRITICAL(&s_lock);
        s_state.roll = roll;
        s_state.pitch = pitch;
        s_state.yaw = yaw;
        for (int a = 0; a < 3; a++) {
            s_state.rate[a] = kf[a].rate;
            s_state.bias[a] = kf[a].bias;
        }
        s_state.still = still;
        s_state.valid = true;
        s_state.updated_us = now_us;
        s_state.read_errors = errors;
        taskEXIT_CRITICAL(&s_lock);
    }
}

void imu_start(void)
{
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, TASK_PRIO_IMU, NULL, 1);
}

void imu_get(imu_state_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}
