#ifndef PS2EMU_MOUSE_H
#define PS2EMU_MOUSE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ps2_mouse_unhandled_cmd_cb_t)(uint8_t pc_idx, uint16_t last_cmd16, void *ctx);

esp_err_t ps2emu_mouse_set_unhandled_cmd_callback(ps2_mouse_unhandled_cmd_cb_t cb, void *ctx);
esp_err_t ps2emu_mouse_send_report(uint8_t pc_idx, int8_t dx, int8_t dy, int8_t wheel,
                                   uint8_t left, uint8_t right, uint8_t middle,
                                   uint8_t b4, uint8_t b5);

#ifdef __cplusplus
}
#endif

#endif
