#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "config.h"
#include "control.h"
#include "imu.h"
#include "params.h"

#define CONSOLE_UART    UART_NUM_0
#define CMD_LINE_MAX    64
#define STATUS_EVERY    10      // 5 Hz

typedef enum { LOG_OFF, LOG_STATUS, LOG_CSV } log_mode_t;

static volatile log_mode_t s_log = LOG_STATUS;

static void print_csv_header(void)
{
    printf("t_ms,state,mode,closed_loop,rx_steer_us,rx_thr_us,stick_steer,stick_thr,throttle,"
           "steer,correction,target,yaw_rate,heading,roll,pitch,servo_us,esc_us\n");
}

static void telemetry_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t n = 0;
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        log_mode_t mode = s_log;
        if (mode == LOG_OFF) continue;

        control_snapshot_t s;
        control_get_snapshot(&s);

        if (mode == LOG_CSV) {
            printf("%lu,%d,%lu,%d,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%lu\n",
                   (unsigned long)s.t_ms, s.state, (unsigned long)s.mode, s.closed_loop,
                   (unsigned long)s.rx_steer_us, (unsigned long)s.rx_throttle_us,
                   s.stick_steer, s.stick_throttle, s.throttle, s.steer, s.correction, s.target,
                   s.yaw_rate, s.heading, s.roll, s.pitch,
                   (unsigned long)s.servo_us, (unsigned long)s.esc_us);
        } else if (++n % STATUS_EVERY == 0) {
            printf("%-8s %-6s | RX %4lu %4lu | thr %+.2f steer %+.2f (ctrl %+.2f%s) | "
                   "yaw %+6.1f dps hdg %+6.1f | roll %+5.1f pitch %+5.1f%s | servo %4lu esc %4lu\n",
                   control_state_name(s.state), params_mode_name(s.mode),
                   (unsigned long)s.rx_steer_us, (unsigned long)s.rx_throttle_us,
                   s.throttle, s.steer, s.correction, s.closed_loop ? " ON" : "",
                   s.yaw_rate, s.heading, s.roll, s.pitch, s.imu_ok ? "" : " IMU-WAIT",
                   (unsigned long)s.servo_us, (unsigned long)s.esc_us);
        }
    }
}

static void cmd_help(void)
{
    printf("\nCommands:\n"
           "  mode manual|assist|rate   drive mode\n"
           "  set <name> <value>        change a parameter (see 'get')\n"
           "  get                       show all parameters\n"
           "  save                      store parameters in flash\n"
           "  defaults                  restore default parameters (then 'save' to keep)\n"
           "  log off|status|csv        telemetry output\n"
           "  imu                       IMU details\n"
           "  help\n\n");
}

static void cmd_get(void)
{
    size_t n;
    const param_def_t *t = params_table(&n);
    printf("\nmode = %s\n", params_mode_name(g_params.mode));
    for (size_t i = 0; i < n; i++) {
        printf("%-13s = %8.4f   %s\n", t[i].name, *t[i].value, t[i].help);
    }
    printf("\n");
}

static void cmd_set(const char *name, const char *value)
{
    if (!name || !value) {
        printf("usage: set <name> <value>\n");
        return;
    }
    size_t n;
    const param_def_t *t = params_table(&n);
    for (size_t i = 0; i < n; i++) {
        if (strcasecmp(name, t[i].name) == 0) {
            char *end;
            float v = strtof(value, &end);
            if (end == value || v < t[i].min || v > t[i].max) {
                printf("%s must be a number in %.3f..%.3f\n", t[i].name, t[i].min, t[i].max);
                return;
            }
            *t[i].value = v;
            printf("%s = %.4f\n", t[i].name, v);
            return;
        }
    }
    printf("unknown parameter '%s' (see 'get')\n", name);
}

static void cmd_mode(const char *arg)
{
    for (uint32_t m = 0; m < MODE_COUNT; m++) {
        if (arg && strcasecmp(arg, params_mode_name(m)) == 0) {
            g_params.mode = m;
            printf("mode = %s\n", params_mode_name(m));
            return;
        }
    }
    printf("usage: mode manual|assist|rate\n");
}

static void cmd_log(const char *arg)
{
    if (arg && strcasecmp(arg, "off") == 0) {
        s_log = LOG_OFF;
    } else if (arg && strcasecmp(arg, "status") == 0) {
        s_log = LOG_STATUS;
    } else if (arg && strcasecmp(arg, "csv") == 0) {
        print_csv_header();
        s_log = LOG_CSV;
    } else {
        printf("usage: log off|status|csv\n");
    }
}

static void cmd_imu(void)
{
    imu_state_t s;
    imu_get(&s);
    printf("\nIMU %s, %s, read errors %lu\n", s.valid ? "valid" : "calibrating",
           s.still ? "still" : "moving", (unsigned long)s.read_errors);
    printf("roll %.2f pitch %.2f yaw %.2f deg\n", s.roll, s.pitch, s.yaw);
    printf("rate %.2f %.2f %.2f dps, bias %.3f %.3f %.3f dps\n\n",
           s.rate[0], s.rate[1], s.rate[2], s.bias[0], s.bias[1], s.bias[2]);
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *a1 = strtok(NULL, " \t");
    char *a2 = strtok(NULL, " \t");

    if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) cmd_help();
    else if (strcasecmp(cmd, "get") == 0) cmd_get();
    else if (strcasecmp(cmd, "set") == 0) cmd_set(a1, a2);
    else if (strcasecmp(cmd, "mode") == 0) cmd_mode(a1);
    else if (strcasecmp(cmd, "log") == 0) cmd_log(a1);
    else if (strcasecmp(cmd, "imu") == 0) cmd_imu();
    else if (strcasecmp(cmd, "save") == 0) printf(params_save() == 0 ? "saved\n" : "save failed\n");
    else if (strcasecmp(cmd, "defaults") == 0) { params_reset(); printf("defaults restored (not saved)\n"); }
    else printf("unknown command '%s', type 'help'\n", cmd);
}

static void console_task(void *arg)
{
    char line[CMD_LINE_MAX];
    size_t len = 0;
    while (1) {
        uint8_t c;
        if (uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY) != 1) continue;

        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            printf("\n");
            line[len] = '\0';
            len = 0;
            log_mode_t saved = s_log;
            if (saved == LOG_CSV) s_log = LOG_OFF;   // keep replies out of the csv
            handle_line(line);
            if (saved == LOG_CSV && s_log == LOG_OFF) s_log = LOG_CSV;
        } else if ((c == '\b' || c == 0x7F) && len > 0) {
            len--;
            printf("\b \b");
        } else if (c >= ' ' && c < 0x7F && len < CMD_LINE_MAX - 1) {
            line[len++] = (char)c;
            putchar(c);
        }
        fflush(stdout);
    }
}

void console_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART, &cfg));

    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, TASK_PRIO_CONSOLE, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, TASK_PRIO_TELEMETRY, NULL, 0);
}
