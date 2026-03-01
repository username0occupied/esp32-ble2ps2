#include "ps2emu_internal.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "PS2EMU";

ps2emu_ctx_t g_ps2emu = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static void ps2emu_protocol_task(void *arg)
{
    (void)arg;
    ps2_rx_msg_t msg;

    while (true) {
        if (xQueueReceive(g_ps2emu.rx_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (msg.port >= PS2_PORT_MAX) {
            continue;
        }

        const ps2_port_runtime_t *p = &g_ps2emu.port[msg.port];
        if (p->kind == PS2_DEV_KBD) {
            ps2emu_keyboard_handle_host_byte(p->pc_idx, msg.byte, msg.parity_error, msg.framing_error);
        } else {
            ps2emu_mouse_handle_host_byte(p->pc_idx, msg.byte, msg.parity_error, msg.framing_error);
        }
    }
}

static bool tx_fifo_push(ps2_tx_fifo_t *fifo, uint8_t byte)
{
    uint16_t next = (uint16_t)((fifo->head + 1U) % PS2EMU_TX_FIFO_LEN);
    if (next == fifo->tail) {
        return false;
    }
    fifo->buf[fifo->head] = byte;
    fifo->head = next;
    return true;
}

esp_err_t ps2emu_core_send_byte(ps2_port_id_t port, uint8_t byte)
{
    if (!g_ps2emu.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (port >= PS2_PORT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    bool ok;
    taskENTER_CRITICAL(&g_ps2emu.lock);
    ok = tx_fifo_push(&g_ps2emu.port[port].tx_fifo, byte);
    taskEXIT_CRITICAL(&g_ps2emu.lock);

    if (!ok) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ps2emu_core_send_bytes(ps2_port_id_t port, const uint8_t *bytes, size_t len)
{
    ESP_RETURN_ON_FALSE(bytes != NULL || len == 0, ESP_ERR_INVALID_ARG, TAG, "bytes is NULL");

    for (size_t i = 0; i < len; ++i) {
        esp_err_t err = ps2emu_core_send_byte(port, bytes[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

uint8_t ps2emu_core_get_last_tx(ps2_port_id_t port)
{
    if (port >= PS2_PORT_MAX) {
        return 0;
    }
    return g_ps2emu.port[port].last_tx_byte;
}

bool ps2emu_core_is_started(void)
{
    return g_ps2emu.started;
}

void ps2emu_core_push_rx_from_isr(const ps2_rx_msg_t *msg, BaseType_t *woken)
{
    if (g_ps2emu.rx_queue == NULL) {
        return;
    }
    xQueueSendFromISR(g_ps2emu.rx_queue, msg, woken);
}

static bool valid_gpio(gpio_num_t pin)
{
    return GPIO_IS_VALID_GPIO(pin);
}

esp_err_t ps2emu_init(const ps2emu_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");

    ESP_RETURN_ON_FALSE(valid_gpio(cfg->pc1_kbd_clk) && valid_gpio(cfg->pc1_kbd_dat), ESP_ERR_INVALID_ARG, TAG,
                        "pc1 keyboard gpio invalid");
    ESP_RETURN_ON_FALSE(valid_gpio(cfg->pc1_mouse_clk) && valid_gpio(cfg->pc1_mouse_dat), ESP_ERR_INVALID_ARG, TAG,
                        "pc1 mouse gpio invalid");
    ESP_RETURN_ON_FALSE(valid_gpio(cfg->pc2_kbd_clk) && valid_gpio(cfg->pc2_kbd_dat), ESP_ERR_INVALID_ARG, TAG,
                        "pc2 keyboard gpio invalid");
    ESP_RETURN_ON_FALSE(valid_gpio(cfg->pc2_mouse_clk) && valid_gpio(cfg->pc2_mouse_dat), ESP_ERR_INVALID_ARG, TAG,
                        "pc2 mouse gpio invalid");

    if (g_ps2emu.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&g_ps2emu.port, 0, sizeof(g_ps2emu.port));
    g_ps2emu.half_clk_us = (cfg->half_clk_us == 0) ? 40 : cfg->half_clk_us;
    g_ps2emu.inter_byte_us = (cfg->inter_byte_us == 0) ? PS2EMU_INTER_BYTE_US_DEFAULT : cfg->inter_byte_us;
    g_ps2emu.half_ticks = (uint16_t)((g_ps2emu.half_clk_us + PS2EMU_TIMER_TICK_US - 1) / PS2EMU_TIMER_TICK_US);
    if (g_ps2emu.half_ticks == 0) {
        g_ps2emu.half_ticks = 1;
    }
    g_ps2emu.inter_byte_ticks = (uint16_t)((g_ps2emu.inter_byte_us + PS2EMU_TIMER_TICK_US - 1) / PS2EMU_TIMER_TICK_US);
    if (g_ps2emu.inter_byte_ticks == 0) {
        g_ps2emu.inter_byte_ticks = 1;
    }

    g_ps2emu.port[PS2_PORT_PC1_KBD].kind = PS2_DEV_KBD;
    g_ps2emu.port[PS2_PORT_PC1_KBD].pc_idx = 0;
    g_ps2emu.port[PS2_PORT_PC1_KBD].line = (ps2_line_cfg_t){cfg->pc1_kbd_clk, cfg->pc1_kbd_dat};

    g_ps2emu.port[PS2_PORT_PC1_MOUSE].kind = PS2_DEV_MOUSE;
    g_ps2emu.port[PS2_PORT_PC1_MOUSE].pc_idx = 0;
    g_ps2emu.port[PS2_PORT_PC1_MOUSE].line = (ps2_line_cfg_t){cfg->pc1_mouse_clk, cfg->pc1_mouse_dat};

    g_ps2emu.port[PS2_PORT_PC2_KBD].kind = PS2_DEV_KBD;
    g_ps2emu.port[PS2_PORT_PC2_KBD].pc_idx = 1;
    g_ps2emu.port[PS2_PORT_PC2_KBD].line = (ps2_line_cfg_t){cfg->pc2_kbd_clk, cfg->pc2_kbd_dat};

    g_ps2emu.port[PS2_PORT_PC2_MOUSE].kind = PS2_DEV_MOUSE;
    g_ps2emu.port[PS2_PORT_PC2_MOUSE].pc_idx = 1;
    g_ps2emu.port[PS2_PORT_PC2_MOUSE].line = (ps2_line_cfg_t){cfg->pc2_mouse_clk, cfg->pc2_mouse_dat};

    for (size_t i = 0; i < PS2_PORT_MAX; ++i) {
        g_ps2emu.port[i].link_state = PS2_LINK_IDLE;
        g_ps2emu.port[i].tx_gap_ticks = g_ps2emu.inter_byte_ticks;
    }

    g_ps2emu.rx_queue = xQueueCreate(PS2EMU_RX_QUEUE_LEN, sizeof(ps2_rx_msg_t));
    ESP_RETURN_ON_FALSE(g_ps2emu.rx_queue != NULL, ESP_ERR_NO_MEM, TAG, "rx queue alloc failed");

    ps2emu_keyboard_init_state();
    ps2emu_mouse_init_state();

    esp_err_t err = ps2emu_isr_init(cfg);
    if (err != ESP_OK) {
        vQueueDelete(g_ps2emu.rx_queue);
        g_ps2emu.rx_queue = NULL;
        return err;
    }

    g_ps2emu.initialized = true;
    ESP_LOGI(TAG, "Initialized 4 PS/2 ports on shared timer, half_clk=%lu us, inter_byte=%u us",
             (unsigned long)g_ps2emu.half_clk_us, (unsigned)g_ps2emu.inter_byte_us);

    return ESP_OK;
}

esp_err_t ps2emu_start(void)
{
    ESP_RETURN_ON_FALSE(g_ps2emu.initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    if (g_ps2emu.started) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreate(ps2emu_protocol_task, "ps2emu_proto", 4096, NULL, 12, &g_ps2emu.proto_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "proto task create failed");

    ESP_RETURN_ON_ERROR(ps2emu_isr_start(), TAG, "failed to start ISR backend");

    ps2emu_keyboard_send_boot_all();
    ps2emu_mouse_send_boot_all();

    g_ps2emu.started = true;
    ESP_LOGI(TAG, "Started");
    return ESP_OK;
}
