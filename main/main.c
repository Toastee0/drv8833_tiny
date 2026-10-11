// Tiny bipolar stepper driven by a DRV8833 on a Seeed XIAO ESP32-S3.
//   D0 (GPIO1) -> AIN1, D1 (GPIO2) -> AIN2  (coil A)
//   D2 (GPIO3) -> BIN1, D3 (GPIO4) -> BIN2  (coil B)
// DRV8833 nSLEEP must be pulled high.
//
// Controlled over the native USB port with a small G-code style protocol.
// Units are half-steps. Each line gets "ok" or "error: ...".
//   G90 / G91         absolute / relative moves
//   G0 X<n> [F<n>]    move (F = feed in steps/s), G1 is the same
//   G92 [X<n>]        set current position (default 0)
//   M17 / M18         hold coils energized when idle / release coils
//   $                 list commands
// Realtime characters, no newline needed and no "ok":
//   ?                 status: <Idle|MPos:123|F:200|Hold:0|Abs:1>
//   !                 stop immediately

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"

static const gpio_num_t PINS[4] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};

// Half-step sequence: {AIN1, AIN2, BIN1, BIN2}.
static const uint8_t SEQ[8][4] = {
    {1, 0, 0, 0},
    {1, 0, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

#define FEED_MIN 1
#define FEED_MAX 1000  // limited by the 1 ms tick

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t pos = 0;
static volatile int32_t target = 0;
static volatile int feed = 200;
static volatile bool hold = false;
static volatile bool running = false;
static bool absolute = true;
static TaskHandle_t motion_task_handle;
static int phase = 0;

static void apply_phase(int p)
{
    for (int i = 0; i < 4; i++) {
        gpio_set_level(PINS[i], SEQ[p][i]);
    }
}

// Coast with no holding current, so the motor doesn't heat up while idle.
static void release(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_set_level(PINS[i], 0);
    }
}

static void motion_task(void *arg)
{
    TickType_t last = 0;
    while (true) {
        taskENTER_CRITICAL(&mux);
        int32_t delta = target - pos;
        if (delta != 0) {
            pos += delta > 0 ? 1 : -1;
        }
        taskEXIT_CRITICAL(&mux);

        if (delta != 0) {
            if (!running) {
                running = true;
                last = xTaskGetTickCount();
            }
            phase = (phase + (delta > 0 ? 1 : 7)) % 8;
            apply_phase(phase);
            TickType_t period = pdMS_TO_TICKS((1000 + feed / 2) / feed);
            vTaskDelayUntil(&last, period ? period : 1);
        } else {
            if (running) {
                running = false;
                if (!hold) {
                    release();
                }
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
}

static void send(const char *s)
{
    usb_serial_jtag_write_bytes(s, strlen(s), pdMS_TO_TICKS(20));
}

static void send_status(void)
{
    char buf[96];
    snprintf(buf, sizeof(buf), "<%s|MPos:%ld|F:%d|Hold:%d|Abs:%d>\r\n",
             running ? "Run" : "Idle", (long)pos, feed, hold, absolute);
    send(buf);
}

static void stop(void)
{
    taskENTER_CRITICAL(&mux);
    target = pos;
    taskEXIT_CRITICAL(&mux);
}

// Parses [-+]digits[.digits]. Not strtod: with spaces stripped it would read
// "G0X10" as the hex number 0X10.
static double parse_number(const char *s, const char **end)
{
    double sign = 1, v = 0, scale = 1;
    const char *p = s;
    if (*p == '-' || *p == '+') {
        sign = *p++ == '-' ? -1 : 1;
    }
    const char *digits = p;
    for (; isdigit((unsigned char)*p); p++) {
        v = v * 10 + (*p - '0');
    }
    if (*p == '.') {
        for (p++; isdigit((unsigned char)*p); p++) {
            scale /= 10;
            v += (*p - '0') * scale;
        }
    }
    *end = p > digits ? p : s;
    return sign * v;
}

// Returns NULL on success, or an error message.
static const char *execute(char *line)
{
    // Strip comments and spaces, upper-case the rest.
    char *w = line;
    for (char *r = line; *r && *r != ';' && *r != '('; r++) {
        if (!isspace((unsigned char)*r)) {
            *w++ = toupper((unsigned char)*r);
        }
    }
    *w = '\0';
    if (line[0] == '\0') {
        return NULL;
    }
    if (strcmp(line, "$") == 0) {
        send("G90 G91 | G0/G1 X<n> F<n> | G92 X<n> | M17 M18 | ? ! | units: half-steps, F: steps/s\r\n");
        return NULL;
    }

    // Words are applied left to right, so "G91 G0 X10 F300" works on one line.
    bool has_x = false, set_pos = false;
    double x = 0;
    for (const char *p = line; *p;) {
        char c = *p;
        const char *end;
        double v = parse_number(p + 1, &end);
        if (end == p + 1) {
            return "error: expected number";
        }
        p = end;
        switch (c) {
        case 'G':
            switch ((int)v) {
            case 0:
            case 1:
                break;
            case 90:
                absolute = true;
                break;
            case 91:
                absolute = false;
                break;
            case 92:
                set_pos = true;
                break;
            default:
                return "error: unsupported G code";
            }
            break;
        case 'M':
            switch ((int)v) {
            case 17:
                hold = true;
                apply_phase(phase);
                break;
            case 18:
                hold = false;
                if (!running) {
                    release();
                }
                break;
            default:
                return "error: unsupported M code";
            }
            break;
        case 'X':
            has_x = true;
            x = v;
            break;
        case 'F':
            if (v < FEED_MIN || v > FEED_MAX) {
                return "error: F out of range (1-1000)";
            }
            feed = (int)v;
            break;
        default:
            return "error: unknown word";
        }
    }

    if (set_pos) {
        int32_t p = has_x ? (int32_t)x : 0;
        taskENTER_CRITICAL(&mux);
        int32_t remaining = target - pos;
        pos = p;
        target = p + remaining;
        taskEXIT_CRITICAL(&mux);
    } else if (has_x) {
        taskENTER_CRITICAL(&mux);
        target = absolute ? (int32_t)x : target + (int32_t)x;
        taskEXIT_CRITICAL(&mux);
        xTaskNotifyGive(motion_task_handle);
    }
    return NULL;
}

void app_main(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_OUTPUT,
    };
    for (int i = 0; i < 4; i++) {
        cfg.pin_bit_mask |= 1ULL << PINS[i];
    }
    ESP_ERROR_CHECK(gpio_config(&cfg));
    release();

    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    xTaskCreate(motion_task, "motion", 3072, NULL, 5, &motion_task_handle);

    char line[96];
    size_t len = 0;
    bool overflow = false;
    while (true) {
        uint8_t buf[64];
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), portMAX_DELAY);
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '?') {
                send_status();
            } else if (c == '!') {
                stop();
            } else if (c == '\n' || c == '\r') {
                if (overflow) {
                    send("error: line too long\r\n");
                } else if (len > 0) {
                    line[len] = '\0';
                    const char *err = execute(line);
                    send(err ? err : "ok");
                    send("\r\n");
                }
                len = 0;
                overflow = false;
            } else if (len < sizeof(line) - 1) {
                line[len++] = c;
            } else {
                overflow = true;
            }
        }
    }
}
