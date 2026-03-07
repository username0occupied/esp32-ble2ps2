#ifndef PS2EMU_INTERNAL_H
#define PS2EMU_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gptimer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ps2emu.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PS2EMU_RX_QUEUE_LEN 64
#define PS2EMU_TX_FIFO_LEN 128
#define PS2EMU_TIMER_TICK_US 10
#define PS2EMU_INTER_BYTE_US_DEFAULT 500

typedef enum {
    PS2_DEV_KBD = 0,
    PS2_DEV_MOUSE,
} ps2_dev_kind_t;

typedef struct {
    gpio_num_t clk;
    gpio_num_t dat;
} ps2_line_cfg_t;

typedef enum {
    PS2_LINK_IDLE = 0,
    PS2_LINK_HOST_REQ_DETECTED,
    PS2_LINK_RX_FROM_HOST,
    PS2_LINK_TX_TO_HOST,
    PS2_LINK_INHIBITED,
} ps2_link_state_t;

typedef struct {
    ps2_port_id_t port;
    uint8_t byte;
    bool parity_error;
    bool framing_error;
} ps2_rx_msg_t;

typedef struct {
    uint8_t buf[PS2EMU_TX_FIFO_LEN];
    uint16_t head;
    uint16_t tail;
} ps2_tx_fifo_t;

typedef enum {
    PS2_RX_STAGE_BITS = 0,
    PS2_RX_STAGE_WAIT_STOP,
    PS2_RX_STAGE_STOP_LOW,
    PS2_RX_STAGE_STOP_HIGH,
    PS2_RX_STAGE_ACK_SETUP,
    PS2_RX_STAGE_ACK_LOW,
} ps2_rx_stage_t;

typedef struct {
    ps2_dev_kind_t kind;
    uint8_t pc_idx;
    ps2_line_cfg_t line;
    ps2_link_state_t link_state;

    volatile bool host_req_pending;
    volatile bool tx_active;
    volatile bool rx_active;
    volatile uint16_t ticks_to_step;

    uint8_t tx_byte;
    uint8_t last_tx_byte;
    uint8_t tx_bit_index;
    bool tx_clk_low_phase;
    uint16_t tx_gap_ticks;

    uint8_t rx_byte;
    uint8_t rx_bit_index;
    uint8_t rx_start_bit;
    uint8_t rx_stop_bit;
    uint8_t rx_parity_bit;
    uint8_t rx_parity_acc;
    bool rx_clk_low_phase;
    ps2_rx_stage_t rx_stage;

    ps2_tx_fifo_t tx_fifo;
} ps2_port_runtime_t;

typedef struct {
    bool initialized;
    bool started;
    uint32_t half_clk_us;
    uint32_t inter_byte_us;
    uint16_t half_ticks;
    uint16_t inter_byte_ticks;

    ps2_port_runtime_t port[PS2_PORT_MAX];

    gptimer_handle_t timer;
    QueueHandle_t rx_queue;
    TaskHandle_t proto_task;

    portMUX_TYPE lock;
} ps2emu_ctx_t;

extern ps2emu_ctx_t g_ps2emu;

esp_err_t ps2emu_isr_init(const ps2emu_cfg_t *cfg);
esp_err_t ps2emu_isr_start(void);

esp_err_t ps2emu_core_send_byte(ps2_port_id_t port, uint8_t byte);
esp_err_t ps2emu_core_send_bytes(ps2_port_id_t port, const uint8_t *bytes, size_t len);
uint8_t ps2emu_core_get_last_tx(ps2_port_id_t port);
bool ps2emu_core_is_started(void);
void ps2emu_core_push_rx_from_isr(const ps2_rx_msg_t *msg, BaseType_t *woken);

void ps2emu_keyboard_init_state(void);
void ps2emu_keyboard_send_boot_all(void);
void ps2emu_keyboard_handle_host_byte(uint8_t pc_idx, uint8_t byte, bool parity_error, bool framing_error);

void ps2emu_mouse_init_state(void);
void ps2emu_mouse_send_boot_all(void);
void ps2emu_mouse_handle_host_byte(uint8_t pc_idx, uint8_t byte, bool parity_error, bool framing_error);

#ifdef __cplusplus
}
#endif

#endif
