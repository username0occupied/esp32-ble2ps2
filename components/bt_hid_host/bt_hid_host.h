#ifndef BT_HID_HOST_H
#define BT_HID_HOST_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t modifiers;
    uint8_t keys[6];
    bool has_report_id;
    uint8_t report_id;
} bt_kbd_report_t;

typedef struct {
    int8_t dx;
    int8_t dy;
    int8_t wheel;
    uint8_t left;
    uint8_t right;
    uint8_t middle;
    uint8_t b4;
    uint8_t b5;
} bt_mouse_report_t;

typedef struct {
    void (*on_kbd_report)(const bt_kbd_report_t *r, void *ctx);
    void (*on_mouse_report)(const bt_mouse_report_t *r, void *ctx);
    void (*on_kbd_conn)(bool connected, void *ctx);
    void (*on_mouse_conn)(bool connected, void *ctx);
    void (*on_passkey)(bool show, uint32_t passkey, void *ctx);
} bt_hid_host_callbacks_t;

esp_err_t bt_hid_host_init(const bt_hid_host_callbacks_t *cbs, void *ctx);
esp_err_t bt_hid_host_start(void);

#ifdef __cplusplus
}
#endif

#endif
