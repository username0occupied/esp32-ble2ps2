#include "input_router.h"

#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ps2emu_keyboard.h"
#include "ps2emu_mouse.h"

#define HID_USAGE_PAGEUP 0x4B
#define HID_USAGE_PAGEDOWN 0x4E
#define MOUSE_FLUSH_BURST_MAX 6
// PS/2 keyboard default typematic: delay 500 ms, rate 10.9 cps (~91.7 ms per repeat).
#define KBD_REPEAT_DELAY_MS 500
#define KBD_REPEAT_PERIOD_MS 92

#ifndef CONFIG_INPUT_ROUTER_MAGIC_TEXT
#define CONFIG_INPUT_ROUTER_MAGIC_TEXT ""
#endif

static const char *TAG = "INPUT_ROUTER";

typedef struct {
    uint8_t len;
    uint8_t bytes[8];
    bool breakable;
} scancode_map_t;

typedef struct {
    uint8_t left;
    uint8_t right;
    uint8_t middle;
    uint8_t b4;
    uint8_t b5;
} mouse_buttons_snapshot_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static input_router_status_t s_status;
static bool s_key_down[256];
static uint8_t s_mouse_left;
static uint8_t s_mouse_right;
static uint8_t s_mouse_middle;
static uint8_t s_mouse_b4;
static uint8_t s_mouse_b5;
static uint8_t s_mouse_raw_b4;
static uint8_t s_mouse_raw_b5;
static int16_t s_mouse_accum_dx;
static int16_t s_mouse_accum_dy;
static int16_t s_mouse_accum_wheel;
static bool s_mouse_btn_dirty;
static TaskHandle_t s_mouse_flush_task;
static TaskHandle_t s_kbd_repeat_task;
static bool s_kbd_repeat_active;
static uint8_t s_kbd_repeat_usage;
static uint8_t s_kbd_repeat_pc;
static TickType_t s_kbd_repeat_next_tick;
static input_router_status_cb_t s_status_cb;
static void *s_status_ctx;

static bool is_switch_usage(uint8_t usage)
{
#if CONFIG_INPUT_ROUTER_SWITCH_KEYBOARD_PAGE
    return usage == HID_USAGE_PAGEUP || usage == HID_USAGE_PAGEDOWN;
#else
    (void)usage;
    return false;
#endif
}

static scancode_map_t map_usage_to_set2(uint8_t usage)
{
    switch (usage) {
        case 0x04: return (scancode_map_t){1, {0x1C}, true}; // A
        case 0x05: return (scancode_map_t){1, {0x32}, true}; // B
        case 0x06: return (scancode_map_t){1, {0x21}, true}; // C
        case 0x07: return (scancode_map_t){1, {0x23}, true}; // D
        case 0x08: return (scancode_map_t){1, {0x24}, true}; // E
        case 0x09: return (scancode_map_t){1, {0x2B}, true}; // F
        case 0x0A: return (scancode_map_t){1, {0x34}, true}; // G
        case 0x0B: return (scancode_map_t){1, {0x33}, true}; // H
        case 0x0C: return (scancode_map_t){1, {0x43}, true}; // I
        case 0x0D: return (scancode_map_t){1, {0x3B}, true}; // J
        case 0x0E: return (scancode_map_t){1, {0x42}, true}; // K
        case 0x0F: return (scancode_map_t){1, {0x4B}, true}; // L
        case 0x10: return (scancode_map_t){1, {0x3A}, true}; // M
        case 0x11: return (scancode_map_t){1, {0x31}, true}; // N
        case 0x12: return (scancode_map_t){1, {0x44}, true}; // O
        case 0x13: return (scancode_map_t){1, {0x4D}, true}; // P
        case 0x14: return (scancode_map_t){1, {0x15}, true}; // Q
        case 0x15: return (scancode_map_t){1, {0x2D}, true}; // R
        case 0x16: return (scancode_map_t){1, {0x1B}, true}; // S
        case 0x17: return (scancode_map_t){1, {0x2C}, true}; // T
        case 0x18: return (scancode_map_t){1, {0x3C}, true}; // U
        case 0x19: return (scancode_map_t){1, {0x2A}, true}; // V
        case 0x1A: return (scancode_map_t){1, {0x1D}, true}; // W
        case 0x1B: return (scancode_map_t){1, {0x22}, true}; // X
        case 0x1C: return (scancode_map_t){1, {0x35}, true}; // Y
        case 0x1D: return (scancode_map_t){1, {0x1A}, true}; // Z
        case 0x1E: return (scancode_map_t){1, {0x16}, true}; // 1
        case 0x1F: return (scancode_map_t){1, {0x1E}, true}; // 2
        case 0x20: return (scancode_map_t){1, {0x26}, true}; // 3
        case 0x21: return (scancode_map_t){1, {0x25}, true}; // 4
        case 0x22: return (scancode_map_t){1, {0x2E}, true}; // 5
        case 0x23: return (scancode_map_t){1, {0x36}, true}; // 6
        case 0x24: return (scancode_map_t){1, {0x3D}, true}; // 7
        case 0x25: return (scancode_map_t){1, {0x3E}, true}; // 8
        case 0x26: return (scancode_map_t){1, {0x46}, true}; // 9
        case 0x27: return (scancode_map_t){1, {0x45}, true}; // 0
        case 0x28: return (scancode_map_t){1, {0x5A}, true}; // Enter
        case 0x29: return (scancode_map_t){1, {0x76}, true}; // Esc
        case 0x2A: return (scancode_map_t){1, {0x66}, true}; // Backspace
        case 0x2B: return (scancode_map_t){1, {0x0D}, true}; // Tab
        case 0x2C: return (scancode_map_t){1, {0x29}, true}; // Space
        case 0x2D: return (scancode_map_t){1, {0x4E}, true}; // -
        case 0x2E: return (scancode_map_t){1, {0x55}, true}; // =
        case 0x2F: return (scancode_map_t){1, {0x54}, true}; // [
        case 0x30: return (scancode_map_t){1, {0x5B}, true}; // ]
        case 0x31: return (scancode_map_t){1, {0x5D}, true}; // Backslash
        case 0x32: return (scancode_map_t){1, {0x61}, true}; // Non-US #
        case 0x33: return (scancode_map_t){1, {0x4C}, true}; // ;
        case 0x34: return (scancode_map_t){1, {0x52}, true}; // '
        case 0x35: return (scancode_map_t){1, {0x0E}, true}; // `
        case 0x36: return (scancode_map_t){1, {0x41}, true}; // ,
        case 0x37: return (scancode_map_t){1, {0x49}, true}; // .
        case 0x38: return (scancode_map_t){1, {0x4A}, true}; // /
        case 0x39: return (scancode_map_t){1, {0x58}, true}; // Caps Lock
        case 0x3A: return (scancode_map_t){1, {0x05}, true}; // F1
        case 0x3B: return (scancode_map_t){1, {0x06}, true}; // F2
        case 0x3C: return (scancode_map_t){1, {0x04}, true}; // F3
        case 0x3D: return (scancode_map_t){1, {0x0C}, true}; // F4
        case 0x3E: return (scancode_map_t){1, {0x03}, true}; // F5
        case 0x3F: return (scancode_map_t){1, {0x0B}, true}; // F6
        case 0x40: return (scancode_map_t){1, {0x83}, true}; // F7
        case 0x41: return (scancode_map_t){1, {0x0A}, true}; // F8
        case 0x42: return (scancode_map_t){1, {0x01}, true}; // F9
        case 0x43: return (scancode_map_t){1, {0x09}, true}; // F10
        case 0x44: return (scancode_map_t){1, {0x78}, true}; // F11
        case 0x45: return (scancode_map_t){1, {0x07}, true}; // F12
        case 0x46: return (scancode_map_t){4, {0xE0, 0x12, 0xE0, 0x7C}, true}; // PrintScreen
        case 0x47: return (scancode_map_t){1, {0x7E}, true}; // Scroll Lock
        case 0x48: return (scancode_map_t){8, {0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xF0, 0x77}, false}; // Pause
        case 0x49: return (scancode_map_t){2, {0xE0, 0x70}, true}; // Insert
        case 0x4A: return (scancode_map_t){2, {0xE0, 0x6C}, true}; // Home
        case 0x4B: return (scancode_map_t){2, {0xE0, 0x7D}, true}; // PageUp
        case 0x4C: return (scancode_map_t){2, {0xE0, 0x71}, true}; // Delete
        case 0x4D: return (scancode_map_t){2, {0xE0, 0x69}, true}; // End
        case 0x4E: return (scancode_map_t){2, {0xE0, 0x7A}, true}; // PageDown
        case 0x4F: return (scancode_map_t){2, {0xE0, 0x74}, true}; // Right
        case 0x50: return (scancode_map_t){2, {0xE0, 0x6B}, true}; // Left
        case 0x51: return (scancode_map_t){2, {0xE0, 0x72}, true}; // Down
        case 0x52: return (scancode_map_t){2, {0xE0, 0x75}, true}; // Up
        case 0x53: return (scancode_map_t){1, {0x77}, true}; // Num Lock
        case 0x54: return (scancode_map_t){2, {0xE0, 0x4A}, true}; // KP /
        case 0x55: return (scancode_map_t){1, {0x7C}, true}; // KP *
        case 0x56: return (scancode_map_t){1, {0x7B}, true}; // KP -
        case 0x57: return (scancode_map_t){1, {0x79}, true}; // KP +
        case 0x58: return (scancode_map_t){2, {0xE0, 0x5A}, true}; // KP Enter
        case 0x59: return (scancode_map_t){1, {0x69}, true}; // KP1
        case 0x5A: return (scancode_map_t){1, {0x72}, true}; // KP2
        case 0x5B: return (scancode_map_t){1, {0x7A}, true}; // KP3
        case 0x5C: return (scancode_map_t){1, {0x6B}, true}; // KP4
        case 0x5D: return (scancode_map_t){1, {0x73}, true}; // KP5
        case 0x5E: return (scancode_map_t){1, {0x74}, true}; // KP6
        case 0x5F: return (scancode_map_t){1, {0x6C}, true}; // KP7
        case 0x60: return (scancode_map_t){1, {0x75}, true}; // KP8
        case 0x61: return (scancode_map_t){1, {0x7D}, true}; // KP9
        case 0x62: return (scancode_map_t){1, {0x70}, true}; // KP0
        case 0x63: return (scancode_map_t){1, {0x71}, true}; // KP.
        case 0x64: return (scancode_map_t){1, {0x61}, true}; // Non-US backslash
        case 0x65: return (scancode_map_t){2, {0xE0, 0x2F}, true}; // Application
        case 0x66: return (scancode_map_t){2, {0xE0, 0x37}, true}; // Power
        case 0xE0: return (scancode_map_t){1, {0x14}, true}; // LCtrl
        case 0xE1: return (scancode_map_t){1, {0x12}, true}; // LShift
        case 0xE2: return (scancode_map_t){1, {0x11}, true}; // LAlt
        case 0xE3: return (scancode_map_t){2, {0xE0, 0x1F}, true}; // LGUI
        case 0xE4: return (scancode_map_t){2, {0xE0, 0x14}, true}; // RCtrl
        case 0xE5: return (scancode_map_t){1, {0x59}, true}; // RShift
        case 0xE6: return (scancode_map_t){2, {0xE0, 0x11}, true}; // RAlt
        case 0xE7: return (scancode_map_t){2, {0xE0, 0x27}, true}; // RGUI
        default: return (scancode_map_t){0, {0}, false};
    }
}

static esp_err_t send_make(uint8_t pc_idx, uint8_t usage)
{
    scancode_map_t map = map_usage_to_set2(usage);
    if (map.len == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    return ps2emu_keyboard_send_scancode_bytes(pc_idx, map.bytes, map.len);
}

static esp_err_t send_break(uint8_t pc_idx, uint8_t usage)
{
    scancode_map_t map = map_usage_to_set2(usage);
    uint8_t seq[16] = {0};

    if (map.len == 0 || !map.breakable) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (map.len == 1) {
        seq[0] = 0xF0;
        seq[1] = map.bytes[0];
        return ps2emu_keyboard_send_scancode_bytes(pc_idx, seq, 2);
    }

    if (map.len == 2 && map.bytes[0] == 0xE0) {
        seq[0] = 0xE0;
        seq[1] = 0xF0;
        seq[2] = map.bytes[1];
        return ps2emu_keyboard_send_scancode_bytes(pc_idx, seq, 3);
    }

    if (map.len == 4 && map.bytes[0] == 0xE0 && map.bytes[2] == 0xE0) {
        // Print Screen break sequence.
        uint8_t ps_break[] = {0xE0, 0xF0, 0x7C, 0xE0, 0xF0, 0x12};
        return ps2emu_keyboard_send_scancode_bytes(pc_idx, ps_break, sizeof(ps_break));
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static void log_kbd_send_error(const char *op, uint8_t pc_idx, uint8_t usage, esp_err_t err)
{
    if (err == ESP_OK || err == ESP_ERR_NOT_FOUND || err == ESP_ERR_NOT_SUPPORTED) {
        return;
    }
    ESP_LOGW(TAG, "PS/2 keyboard %s failed on PC%u usage=0x%02X: %s",
             op, (unsigned)pc_idx + 1, usage, esp_err_to_name(err));
}

static void log_mouse_send_error(uint8_t pc_idx, esp_err_t err)
{
    if (err == ESP_OK) {
        return;
    }
    ESP_LOGW(TAG, "PS/2 mouse send failed on PC%u: %s", (unsigned)pc_idx + 1, esp_err_to_name(err));
}

static bool ascii_to_hid_usage(char ch, uint8_t *usage, bool *need_shift)
{
    if (usage == NULL || need_shift == NULL) {
        return false;
    }

    if (ch >= 'a' && ch <= 'z') {
        *usage = (uint8_t)(0x04 + (ch - 'a'));
        *need_shift = false;
        return true;
    }
    if (ch >= 'A' && ch <= 'Z') {
        *usage = (uint8_t)(0x04 + (ch - 'A'));
        *need_shift = true;
        return true;
    }
    if (ch >= '1' && ch <= '9') {
        *usage = (uint8_t)(0x1E + (ch - '1'));
        *need_shift = false;
        return true;
    }
    if (ch == '0') {
        *usage = 0x27;
        *need_shift = false;
        return true;
    }
    return false;
}

static void type_magic_string(uint8_t pc_idx)
{
    static const char *k_magic = CONFIG_INPUT_ROUTER_MAGIC_TEXT;
    static const uint8_t k_lshift_usage = 0xE1; // Left Shift

    if (k_magic == NULL || k_magic[0] == '\0') {
        return;
    }

    for (size_t i = 0; k_magic[i] != '\0'; ++i) {
        uint8_t usage = 0;
        bool need_shift = false;
        if (!ascii_to_hid_usage(k_magic[i], &usage, &need_shift)) {
            ESP_LOGW(TAG, "unsupported magic char '%c' (0x%02X)", k_magic[i], (unsigned char)k_magic[i]);
            continue;
        }

        if (need_shift) {
            esp_err_t err = send_make(pc_idx, k_lshift_usage);
            log_kbd_send_error("make", pc_idx, k_lshift_usage, err);
        }

        esp_err_t err = send_make(pc_idx, usage);
        log_kbd_send_error("make", pc_idx, usage, err);
        err = send_break(pc_idx, usage);
        log_kbd_send_error("break", pc_idx, usage, err);

        if (need_shift) {
            err = send_break(pc_idx, k_lshift_usage);
            log_kbd_send_error("break", pc_idx, k_lshift_usage, err);
        }
    }
}

static bool is_typematic_eligible(uint8_t usage)
{
    if (is_switch_usage(usage)) {
        return false;
    }
    // Modifiers and lock keys are not typematic candidates.
    if ((usage >= 0xE0 && usage <= 0xE7) || usage == 0x39 || usage == 0x47 || usage == 0x53) {
        return false;
    }

    scancode_map_t map = map_usage_to_set2(usage);
    return map.len > 0 && map.breakable;
}

static inline void kbd_repeat_notify(void)
{
    if (s_kbd_repeat_task) {
        xTaskNotifyGive(s_kbd_repeat_task);
    }
}

static void kbd_repeat_stop_locked(void)
{
    s_kbd_repeat_active = false;
    s_kbd_repeat_usage = 0;
    s_kbd_repeat_next_tick = 0;
}

static void kbd_repeat_start_locked(uint8_t pc_idx, uint8_t usage)
{
    TickType_t delay = pdMS_TO_TICKS(KBD_REPEAT_DELAY_MS);
    if (delay == 0) {
        delay = 1;
    }
    s_kbd_repeat_active = true;
    s_kbd_repeat_pc = pc_idx;
    s_kbd_repeat_usage = usage;
    s_kbd_repeat_next_tick = xTaskGetTickCount() + delay;
}

static void kbd_repeat_update_from_report_locked(const bool prev[256], const bool next[256], uint8_t active_pc)
{
    if (s_kbd_repeat_active) {
        if (s_kbd_repeat_pc != active_pc || !next[s_kbd_repeat_usage]) {
            kbd_repeat_stop_locked();
        }
    }

    uint8_t candidate = 0;
    bool found = false;
    for (int u = 0; u < 256; ++u) {
        if (!prev[u] && next[u] && is_typematic_eligible((uint8_t)u)) {
            candidate = (uint8_t)u;
            found = true;
        }
    }

    if (found) {
        kbd_repeat_start_locked(active_pc, candidate);
    }
}

static void kbd_repeat_task(void *ctx)
{
    (void)ctx;
    while (true) {
        TickType_t wait_ticks = portMAX_DELAY;
        uint8_t usage = 0;
        uint8_t pc_idx = 0;
        bool send_repeat = false;

        portENTER_CRITICAL(&s_lock);
        if (s_kbd_repeat_active) {
            const bool still_down = s_key_down[s_kbd_repeat_usage];
            const bool pc_match = (s_status.active_pc == s_kbd_repeat_pc);
            if (!still_down || !pc_match) {
                kbd_repeat_stop_locked();
            } else {
                TickType_t now = xTaskGetTickCount();
                if ((int32_t)(now - s_kbd_repeat_next_tick) >= 0) {
                    send_repeat = true;
                    usage = s_kbd_repeat_usage;
                    pc_idx = s_kbd_repeat_pc;
                    TickType_t period = pdMS_TO_TICKS(KBD_REPEAT_PERIOD_MS);
                    if (period == 0) {
                        period = 1;
                    }
                    s_kbd_repeat_next_tick = now + period;
                } else {
                    wait_ticks = s_kbd_repeat_next_tick - now;
                }
            }
        }
        portEXIT_CRITICAL(&s_lock);

        if (send_repeat) {
            esp_err_t err = send_make(pc_idx, usage);
            log_kbd_send_error("make", pc_idx, usage, err);
            continue;
        }

        (void)ulTaskNotifyTake(pdTRUE, wait_ticks);
    }
}

static int8_t clamp_i16_to_i8_step(int16_t *accum, int16_t min_step, int16_t max_step)
{
    int16_t v = *accum;
    int16_t step = 0;
    if (v > max_step) {
        step = max_step;
    } else if (v < min_step) {
        step = min_step;
    } else {
        step = v;
    }
    *accum = (int16_t)(v - step);
    return (int8_t)step;
}

static bool mouse_flush_once(void)
{
    uint8_t active_pc;
    uint8_t left;
    uint8_t right;
    uint8_t middle;
    uint8_t b4;
    uint8_t b5;
    int8_t dx_step;
    int8_t dy_step;
    int8_t wheel_step;
    bool btn_dirty;

    portENTER_CRITICAL(&s_lock);
    if (s_mouse_accum_dx == 0 && s_mouse_accum_dy == 0 && s_mouse_accum_wheel == 0 && !s_mouse_btn_dirty) {
        portEXIT_CRITICAL(&s_lock);
        return false;
    }

    active_pc = s_status.active_pc;
    left = s_mouse_left;
    right = s_mouse_right;
    middle = s_mouse_middle;
    b4 = s_mouse_b4;
    b5 = s_mouse_b5;
    btn_dirty = s_mouse_btn_dirty;

    dx_step = clamp_i16_to_i8_step(&s_mouse_accum_dx, -127, 127);
    dy_step = clamp_i16_to_i8_step(&s_mouse_accum_dy, -127, 127);
    wheel_step = clamp_i16_to_i8_step(&s_mouse_accum_wheel, -7, 7);
    s_mouse_btn_dirty = false;
    portEXIT_CRITICAL(&s_lock);

    if (dx_step == 0 && dy_step == 0 && wheel_step == 0 && !btn_dirty) {
        return false;
    }

    esp_err_t err = ps2emu_mouse_send_report(active_pc, dx_step, dy_step, wheel_step, left, right, middle, b4, b5);
    log_mouse_send_error(active_pc, err);
    return true;
}

static bool mouse_has_pending(void)
{
    bool pending;
    portENTER_CRITICAL(&s_lock);
    pending = (s_mouse_accum_dx != 0) ||
              (s_mouse_accum_dy != 0) ||
              (s_mouse_accum_wheel != 0) ||
              s_mouse_btn_dirty;
    portEXIT_CRITICAL(&s_lock);
    return pending;
}

static inline void mouse_flush_notify(void)
{
    if (s_mouse_flush_task) {
        xTaskNotifyGive(s_mouse_flush_task);
    }
}

static void mouse_flush_task(void *ctx)
{
    (void)ctx;
    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (int i = 0; i < MOUSE_FLUSH_BURST_MAX; ++i) {
            if (!mouse_flush_once()) {
                break;
            }
        }
        if (mouse_has_pending()) {
            mouse_flush_notify();
        }
    }
}

static void collect_report_down(const bt_kbd_report_t *r, bool down[256])
{
    memset(down, 0, 256);

    for (int i = 0; i < 8; ++i) {
        if ((r->modifiers >> i) & 0x1) {
            down[0xE0 + i] = true;
        }
    }

    for (int i = 0; i < 6; ++i) {
        uint8_t usage = r->keys[i];
        if (usage <= 0x03) {
            continue;
        }
        down[usage] = true;
    }
}

static void status_notify(void)
{
    input_router_status_t snap;

    if (!s_status_cb) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    snap = s_status;
    portEXIT_CRITICAL(&s_lock);

    s_status_cb(&snap, s_status_ctx);
}

static void release_all_to_pc(uint8_t pc_idx, const bool down[256], const mouse_buttons_snapshot_t *btns)
{
    for (int u = 0; u < 256; ++u) {
        if (down[u] && !is_switch_usage((uint8_t)u)) {
            esp_err_t err = send_break(pc_idx, (uint8_t)u);
            log_kbd_send_error("break", pc_idx, (uint8_t)u, err);
        }
    }

    if (btns->left || btns->right || btns->middle || btns->b4 || btns->b5) {
        esp_err_t err = ps2emu_mouse_send_report(pc_idx, 0, 0, 0, 0, 0, 0, 0, 0);
        log_mouse_send_error(pc_idx, err);
    }
}

esp_err_t input_router_init(input_router_status_cb_t cb, void *ctx)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_status, 0, sizeof(s_status));
    memset(s_key_down, 0, sizeof(s_key_down));
    s_status.active_pc = 0;
    s_mouse_left = 0;
    s_mouse_right = 0;
    s_mouse_middle = 0;
    s_mouse_b4 = 0;
    s_mouse_b5 = 0;
    s_mouse_raw_b4 = 0;
    s_mouse_raw_b5 = 0;
    s_mouse_accum_dx = 0;
    s_mouse_accum_dy = 0;
    s_mouse_accum_wheel = 0;
    s_mouse_btn_dirty = false;
    s_kbd_repeat_active = false;
    s_kbd_repeat_usage = 0;
    s_kbd_repeat_pc = 0;
    s_kbd_repeat_next_tick = 0;
    s_status_cb = cb;
    s_status_ctx = ctx;
    portEXIT_CRITICAL(&s_lock);

    if (s_mouse_flush_task == NULL) {
        if (xTaskCreate(mouse_flush_task, "mouse_flush", 3072, NULL, 8, &s_mouse_flush_task) != pdPASS) {
            ESP_LOGE(TAG, "mouse flush task create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_kbd_repeat_task == NULL) {
        if (xTaskCreate(kbd_repeat_task, "kbd_repeat", 3072, NULL, 8, &s_kbd_repeat_task) != pdPASS) {
            ESP_LOGE(TAG, "keyboard repeat task create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    status_notify();
    ESP_LOGI(TAG, "Input router initialized, active PC=%u", (unsigned)(s_status.active_pc + 1));
    return ESP_OK;
}

esp_err_t input_router_get_status(input_router_status_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

void input_router_on_kbd_report(const bt_kbd_report_t *r, void *ctx)
{
    bool next_down[256];
    bool prev_down[256];
    mouse_buttons_snapshot_t prev_btns;
    uint8_t active_pc;
    bool switched = false;
    uint8_t switch_target = 0;
    bool target_changed = false;

    (void)ctx;
    if (!r) {
        return;
    }

    collect_report_down(r, next_down);

    portENTER_CRITICAL(&s_lock);
    memcpy(prev_down, s_key_down, sizeof(prev_down));
    prev_btns.left = s_mouse_left;
    prev_btns.right = s_mouse_right;
    prev_btns.middle = s_mouse_middle;
    prev_btns.b4 = s_mouse_b4;
    prev_btns.b5 = s_mouse_b5;
    active_pc = s_status.active_pc;

#if CONFIG_INPUT_ROUTER_SWITCH_KEYBOARD_PAGE
    if (!prev_down[HID_USAGE_PAGEUP] && next_down[HID_USAGE_PAGEUP]) {
        switched = true;
        switch_target = 0;
    } else if (!prev_down[HID_USAGE_PAGEDOWN] && next_down[HID_USAGE_PAGEDOWN]) {
        switched = true;
        switch_target = 1;
    }

    prev_down[HID_USAGE_PAGEUP] = false;
    prev_down[HID_USAGE_PAGEDOWN] = false;
    next_down[HID_USAGE_PAGEUP] = false;
    next_down[HID_USAGE_PAGEDOWN] = false;
#endif
    portEXIT_CRITICAL(&s_lock);

    if (switched && switch_target != active_pc) {
        target_changed = true;
        release_all_to_pc(active_pc, prev_down, &prev_btns);
        active_pc = switch_target;
        memset(next_down, 0, sizeof(next_down));
    } else {
        for (int u = 0; u < 256; ++u) {
            if (prev_down[u] && !next_down[u]) {
                esp_err_t err = send_break(active_pc, (uint8_t)u);
                log_kbd_send_error("break", active_pc, (uint8_t)u, err);
            }
        }
        for (int u = 0; u < 256; ++u) {
            if (!prev_down[u] && next_down[u]) {
                esp_err_t err = send_make(active_pc, (uint8_t)u);
                log_kbd_send_error("make", active_pc, (uint8_t)u, err);
            }
        }
    }

    portENTER_CRITICAL(&s_lock);
    if (target_changed) {
        s_status.active_pc = active_pc;
        s_mouse_left = 0;
        s_mouse_right = 0;
        s_mouse_middle = 0;
        s_mouse_b4 = 0;
        s_mouse_b5 = 0;
        s_mouse_accum_dx = 0;
        s_mouse_accum_dy = 0;
        s_mouse_accum_wheel = 0;
        s_mouse_btn_dirty = false;
    }
    kbd_repeat_update_from_report_locked(prev_down, next_down, active_pc);
    memcpy(s_key_down, next_down, sizeof(s_key_down));
    portEXIT_CRITICAL(&s_lock);
    kbd_repeat_notify();

    if (target_changed) {
        ESP_LOGI(TAG, "Switched active PC to %u", (unsigned)(active_pc + 1));
        status_notify();
    }
}

void input_router_on_mouse_report(const bt_mouse_report_t *r, void *ctx)
{
    uint8_t prev_left;
    uint8_t prev_right;
    uint8_t prev_middle;
    uint8_t prev_b4;
    uint8_t prev_b5;
    uint8_t new_left;
    uint8_t new_right;
    uint8_t new_middle;
    uint8_t new_b4;
    uint8_t new_b5;
#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    uint8_t prev_raw_b4;
    uint8_t prev_raw_b5;
    uint8_t new_raw_b4;
    uint8_t new_raw_b5;
#endif
    bool btn_changed;
#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    uint8_t active_pc;
    bool prev_down[256];
    bool switched = false;
    uint8_t switch_target = 0;
    bool repeated_hotkey = false;
#endif

    (void)ctx;
    if (!r) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    active_pc = s_status.active_pc;
#endif
    prev_left = s_mouse_left;
    prev_right = s_mouse_right;
    prev_middle = s_mouse_middle;
    prev_b4 = s_mouse_b4;
    prev_b5 = s_mouse_b5;
#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    prev_raw_b4 = s_mouse_raw_b4;
    prev_raw_b5 = s_mouse_raw_b5;
    memcpy(prev_down, s_key_down, sizeof(prev_down));
#endif
    portEXIT_CRITICAL(&s_lock);

    new_left = r->left ? 1 : 0;
    new_right = r->right ? 1 : 0;
    new_middle = r->middle ? 1 : 0;
    new_b4 = r->b4 ? 1 : 0;
    new_b5 = r->b5 ? 1 : 0;

#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    new_raw_b4 = new_b4;
    new_raw_b5 = new_b5;

    // Mouse side Forward button (b5) -> PC1, Back button (b4) -> PC2.
    if (!prev_raw_b5 && new_raw_b5) {
        switched = true;
        switch_target = 0;
        repeated_hotkey = (active_pc == 0);
    } else if (!prev_raw_b4 && new_raw_b4) {
        switched = true;
        switch_target = 1;
        repeated_hotkey = (active_pc == 1);
    }

    // Side buttons are hotkeys in this mode and must not be forwarded to PS/2.
    new_b4 = 0;
    new_b5 = 0;

    if (switched && switch_target != active_pc) {
        mouse_buttons_snapshot_t prev_btns = {
            .left = prev_left,
            .right = prev_right,
            .middle = prev_middle,
            .b4 = prev_b4,
            .b5 = prev_b5,
        };
        release_all_to_pc(active_pc, prev_down, &prev_btns);

        portENTER_CRITICAL(&s_lock);
        s_status.active_pc = switch_target;
        memset(s_key_down, 0, sizeof(s_key_down));
        kbd_repeat_stop_locked();
        s_mouse_left = 0;
        s_mouse_right = 0;
        s_mouse_middle = 0;
        s_mouse_b4 = 0;
        s_mouse_b5 = 0;
        s_mouse_raw_b4 = new_raw_b4;
        s_mouse_raw_b5 = new_raw_b5;
        s_mouse_accum_dx = 0;
        s_mouse_accum_dy = 0;
        s_mouse_accum_wheel = 0;
        s_mouse_btn_dirty = false;
        portEXIT_CRITICAL(&s_lock);
        kbd_repeat_notify();

        ESP_LOGI(TAG, "Switched active PC to %u", (unsigned)(switch_target + 1));
        status_notify();
        return;
    }

    if (repeated_hotkey) {
        type_magic_string(active_pc);
    }
#endif

    btn_changed = (prev_left != new_left) ||
                  (prev_right != new_right) ||
                  (prev_middle != new_middle) ||
                  (prev_b4 != new_b4) ||
                  (prev_b5 != new_b5);

    portENTER_CRITICAL(&s_lock);
    s_mouse_left = new_left;
    s_mouse_right = new_right;
    s_mouse_middle = new_middle;
    s_mouse_b4 = new_b4;
    s_mouse_b5 = new_b5;
#if CONFIG_INPUT_ROUTER_SWITCH_MOUSE_SIDE
    s_mouse_raw_b4 = new_raw_b4;
    s_mouse_raw_b5 = new_raw_b5;
#endif
    if (btn_changed) {
        s_mouse_btn_dirty = true;
    }
    s_mouse_accum_dx = (int16_t)(s_mouse_accum_dx + r->dx);
    s_mouse_accum_dy = (int16_t)(s_mouse_accum_dy + (int16_t)(-r->dy));
    s_mouse_accum_wheel = (int16_t)(s_mouse_accum_wheel + r->wheel);
    portEXIT_CRITICAL(&s_lock);

    mouse_flush_notify();
    if (btn_changed) {
        (void)mouse_flush_once();
    }
}

void input_router_on_kbd_conn(bool connected, void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_status.bt_kbd_connected = connected;
    s_status.bt_kbd_battery_valid = false;
    s_status.bt_kbd_battery = 0;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_mouse_conn(bool connected, void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_status.bt_mouse_connected = connected;
    s_status.bt_mouse_battery_valid = false;
    s_status.bt_mouse_battery = 0;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_kbd_battery(bool available, uint8_t level, void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_status.bt_kbd_battery_valid = available;
    s_status.bt_kbd_battery = available ? level : 0;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_mouse_battery(bool available, uint8_t level, void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_status.bt_mouse_battery_valid = available;
    s_status.bt_mouse_battery = available ? level : 0;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_passkey(bool show, uint32_t passkey, void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_status.passkey_visible = show;
    s_status.passkey = show ? passkey : 0;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_ps2_led(uint8_t pc_idx, const ps2_kbd_led_state_t *state, void *ctx)
{
    (void)ctx;
    if (pc_idx >= 2 || !state) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_status.pc_led[pc_idx] = *state;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "PC%u LED state updated: num=%u caps=%u scroll=%u",
             (unsigned)pc_idx + 1,
             state->num ? 1U : 0U,
             state->caps ? 1U : 0U,
             state->scroll ? 1U : 0U);
    status_notify();
}

void input_router_on_ps2_init(uint8_t pc_idx, bool inited, void *ctx)
{
    (void)ctx;
    if (pc_idx >= 2) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_status.pc_kbd_initialized[pc_idx] = inited;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_ps2_kbd_unhandled_cmd(uint8_t pc_idx, uint16_t last_cmd16, void *ctx)
{
    (void)ctx;
    if (pc_idx >= 2) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_status.pc_last_unhandled_valid[pc_idx] = true;
    s_status.pc_last_unhandled_mouse[pc_idx] = false;
    s_status.pc_last_unhandled_cmd16[pc_idx] = last_cmd16;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}

void input_router_on_ps2_mouse_unhandled_cmd(uint8_t pc_idx, uint16_t last_cmd16, void *ctx)
{
    (void)ctx;
    if (pc_idx >= 2) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_status.pc_last_unhandled_valid[pc_idx] = true;
    s_status.pc_last_unhandled_mouse[pc_idx] = true;
    s_status.pc_last_unhandled_cmd16[pc_idx] = last_cmd16;
    portEXIT_CRITICAL(&s_lock);
    status_notify();
}
