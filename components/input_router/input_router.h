#ifndef INPUT_ROUTER_H
#define INPUT_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "bt_hid_host.h"
#include "ps2emu_keyboard.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t active_pc;
    bool bt_kbd_connected;
    bool bt_mouse_connected;
    bool passkey_visible;
    uint32_t passkey;
    bool pc_kbd_initialized[2];
    ps2_kbd_led_state_t pc_led[2];
} input_router_status_t;

typedef void (*input_router_status_cb_t)(const input_router_status_t *status, void *ctx);

esp_err_t input_router_init(input_router_status_cb_t cb, void *ctx);
esp_err_t input_router_get_status(input_router_status_t *out);

void input_router_on_kbd_report(const bt_kbd_report_t *r, void *ctx);
void input_router_on_mouse_report(const bt_mouse_report_t *r, void *ctx);
void input_router_on_kbd_conn(bool connected, void *ctx);
void input_router_on_mouse_conn(bool connected, void *ctx);
void input_router_on_passkey(bool show, uint32_t passkey, void *ctx);

void input_router_on_ps2_led(uint8_t pc_idx, const ps2_kbd_led_state_t *state, void *ctx);
void input_router_on_ps2_init(uint8_t pc_idx, bool inited, void *ctx);

#ifdef __cplusplus
}
#endif

#endif
