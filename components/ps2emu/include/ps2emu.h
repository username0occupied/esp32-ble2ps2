#ifndef PS2EMU_H
#define PS2EMU_H

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PS2_PORT_PC1_KBD = 0,
    PS2_PORT_PC1_MOUSE,
    PS2_PORT_PC2_KBD,
    PS2_PORT_PC2_MOUSE,
    PS2_PORT_MAX,
} ps2_port_id_t;

typedef struct {
    gpio_num_t pc1_kbd_clk;
    gpio_num_t pc1_kbd_dat;
    gpio_num_t pc1_mouse_clk;
    gpio_num_t pc1_mouse_dat;
    gpio_num_t pc2_kbd_clk;
    gpio_num_t pc2_kbd_dat;
    gpio_num_t pc2_mouse_clk;
    gpio_num_t pc2_mouse_dat;
    uint32_t half_clk_us;
    uint32_t inter_byte_us;
} ps2emu_cfg_t;

esp_err_t ps2emu_init(const ps2emu_cfg_t *cfg);
esp_err_t ps2emu_start(void);

#ifdef __cplusplus
}
#endif

#endif
