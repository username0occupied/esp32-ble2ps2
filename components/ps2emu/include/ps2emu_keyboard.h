#ifndef PS2EMU_KEYBOARD_H
#define PS2EMU_KEYBOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool scroll;
    bool num;
    bool caps;
} ps2_kbd_led_state_t;

typedef void (*ps2_kbd_led_cb_t)(uint8_t pc_idx, const ps2_kbd_led_state_t *state, void *ctx);
typedef void (*ps2_kbd_init_cb_t)(uint8_t pc_idx, bool inited, void *ctx);
typedef void (*ps2_kbd_unhandled_cmd_cb_t)(uint8_t pc_idx, uint16_t last_cmd16, void *ctx);

esp_err_t ps2emu_keyboard_set_led_callback(ps2_kbd_led_cb_t cb, void *ctx);
esp_err_t ps2emu_keyboard_set_init_callback(ps2_kbd_init_cb_t cb, void *ctx);
esp_err_t ps2emu_keyboard_set_unhandled_cmd_callback(ps2_kbd_unhandled_cmd_cb_t cb, void *ctx);
esp_err_t ps2emu_keyboard_send_scancode_bytes(uint8_t pc_idx, const uint8_t *bytes, size_t len);

#ifdef __cplusplus
}
#endif

#endif
