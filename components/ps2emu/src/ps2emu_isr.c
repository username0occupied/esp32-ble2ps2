#include "ps2emu_internal.h"

#include "esp_attr.h"
#include "esp_check.h"

typedef struct {
    ps2_port_id_t port;
    bool is_clk;
} ps2_gpio_isr_arg_t;

static ps2_gpio_isr_arg_t s_isr_args[PS2_PORT_MAX * 2];

static inline void IRAM_ATTR ps2_line_release(gpio_num_t pin) { gpio_set_level(pin, 1); }
static inline void IRAM_ATTR ps2_line_low(gpio_num_t pin) { gpio_set_level(pin, 0); }
static inline int IRAM_ATTR ps2_line_read(gpio_num_t pin) { return gpio_get_level(pin); }

static inline uint8_t IRAM_ATTR ps2_odd_parity(uint8_t byte)
{
    uint8_t parity = 1;
    for (int i = 0; i < 8; ++i) {
        parity ^= ((byte >> i) & 0x1);
    }
    return parity;
}

static inline uint8_t IRAM_ATTR ps2_tx_bit_value(const ps2_port_runtime_t *p)
{
    if (p->tx_bit_index == 0) {
        return 0;
    }
    if (p->tx_bit_index >= 1 && p->tx_bit_index <= 8) {
        return (uint8_t)((p->tx_byte >> (p->tx_bit_index - 1)) & 0x1);
    }
    if (p->tx_bit_index == 9) {
        return ps2_odd_parity(p->tx_byte);
    }
    return 1;
}

static inline bool IRAM_ATTR ps2_tx_fifo_pop_isr(ps2_tx_fifo_t *fifo, uint8_t *out)
{
    if (fifo->head == fifo->tail) {
        return false;
    }
    *out = fifo->buf[fifo->tail];
    fifo->tail = (uint16_t)((fifo->tail + 1U) % PS2EMU_TX_FIFO_LEN);
    return true;
}

static inline bool IRAM_ATTR ps2_bus_idle(ps2_port_runtime_t *p)
{
    return ps2_line_read(p->line.clk) == 1 && ps2_line_read(p->line.dat) == 1;
}

static void IRAM_ATTR ps2_start_tx_if_possible(ps2_port_runtime_t *p)
{
    if (p->tx_active || p->rx_active || p->host_req_pending) {
        return;
    }

    if (p->tx_gap_ticks > 0) {
        p->tx_gap_ticks--;
        return;
    }

    if (!ps2_bus_idle(p)) {
        p->link_state = (ps2_line_read(p->line.clk) == 0) ? PS2_LINK_INHIBITED : PS2_LINK_IDLE;
        return;
    }

    uint8_t next_byte;
    if (!ps2_tx_fifo_pop_isr(&p->tx_fifo, &next_byte)) {
        p->link_state = PS2_LINK_IDLE;
        return;
    }

    p->tx_active = true;
    p->link_state = PS2_LINK_TX_TO_HOST;
    p->tx_byte = next_byte;
    p->last_tx_byte = next_byte;
    p->tx_bit_index = 0;
    p->tx_clk_low_phase = false;
    p->ticks_to_step = g_ps2emu.half_ticks;

    ps2_line_low(p->line.dat);
    ps2_line_release(p->line.clk);
}

static void IRAM_ATTR ps2_step_tx(ps2_port_runtime_t *p)
{
    if (!p->tx_clk_low_phase) {
        ps2_line_low(p->line.clk);
        p->tx_clk_low_phase = true;
        return;
    }

    ps2_line_release(p->line.clk);
    p->tx_clk_low_phase = false;
    p->tx_bit_index++;

    if (p->tx_bit_index > 10) {
        p->tx_active = false;
        p->link_state = PS2_LINK_IDLE;
        p->ticks_to_step = 0;
        p->tx_gap_ticks = g_ps2emu.inter_byte_ticks;
        ps2_line_release(p->line.dat);
        return;
    }

    uint8_t bit = ps2_tx_bit_value(p);
    if (bit == 0) {
        ps2_line_low(p->line.dat);
    } else {
        ps2_line_release(p->line.dat);
    }
}

static void IRAM_ATTR ps2_start_rx(ps2_port_runtime_t *p)
{
    p->rx_active = true;
    p->link_state = PS2_LINK_RX_FROM_HOST;
    p->rx_stage = PS2_RX_STAGE_BITS;
    p->rx_bit_index = 0;
    p->rx_byte = 0;
    // Host request-to-send already guarantees start condition (DAT low while CLK high).
    p->rx_start_bit = 0;
    p->rx_stop_bit = 0;
    p->rx_parity_bit = 0;
    p->rx_parity_acc = 1;
    p->rx_clk_low_phase = false;
    p->ticks_to_step = g_ps2emu.half_ticks;

    ps2_line_release(p->line.clk);
    ps2_line_release(p->line.dat);
}

static void IRAM_ATTR ps2_finalize_rx(ps2_port_id_t port, ps2_port_runtime_t *p, BaseType_t *woken)
{
    ps2_rx_msg_t msg = {
        .port = port,
        .byte = p->rx_byte,
        .parity_error = (p->rx_parity_acc != 0),
        .framing_error = !(p->rx_start_bit == 0 && p->rx_stop_bit == 1),
    };

    ps2emu_core_push_rx_from_isr(&msg, woken);

    p->rx_active = false;
    p->host_req_pending = false;
    p->ticks_to_step = 0;
    p->link_state = (ps2_line_read(p->line.clk) == 0) ? PS2_LINK_INHIBITED : PS2_LINK_IDLE;

    ps2_line_release(p->line.clk);
    ps2_line_release(p->line.dat);
}

static void IRAM_ATTR ps2_step_rx(ps2_port_id_t port, ps2_port_runtime_t *p, BaseType_t *woken)
{
    if (p->rx_stage == PS2_RX_STAGE_BITS) {
        if (!p->rx_clk_low_phase) {
            ps2_line_low(p->line.clk);
            p->rx_clk_low_phase = true;
            return;
        }

        ps2_line_release(p->line.clk);
        p->rx_clk_low_phase = false;

        uint8_t sample = (uint8_t)(ps2_line_read(p->line.dat) & 0x1);
        if (p->rx_bit_index <= 7) {
            // In host-to-device transfer, first clocked bit is data0 (start is request phase).
            p->rx_byte |= (uint8_t)(sample << p->rx_bit_index);
            p->rx_parity_acc ^= sample;
        } else if (p->rx_bit_index == 8) {
            // Final sampled bit before stop is parity.
            p->rx_parity_bit = sample;
            p->rx_parity_acc ^= sample;
        }

        p->rx_bit_index++;
        if (p->rx_bit_index >= 9) {
            /*
             * After 8 data + parity, always generate one stop clock cycle first.
             * Some hosts release DATA only after seeing this clock transition.
             */
            p->rx_stage = PS2_RX_STAGE_STOP_LOW;
            p->ticks_to_step = g_ps2emu.half_ticks;
        }
        return;
    }

    if (p->rx_stage == PS2_RX_STAGE_WAIT_STOP) {
        if (ps2_line_read(p->line.clk) == 1 && ps2_line_read(p->line.dat) == 1) {
            p->rx_stop_bit = 1;
            ps2_line_low(p->line.dat);
            p->rx_stage = PS2_RX_STAGE_ACK_SETUP;
            p->ticks_to_step = g_ps2emu.half_ticks;
        }
        return;
    }

    if (p->rx_stage == PS2_RX_STAGE_STOP_LOW) {
        // Stop bit: clock low while DATA remains released (high).
        ps2_line_release(p->line.dat);
        ps2_line_low(p->line.clk);
        p->rx_stage = PS2_RX_STAGE_STOP_HIGH;
        return;
    }

    if (p->rx_stage == PS2_RX_STAGE_STOP_HIGH) {
        // End of stop bit clock, then wait host DATA release (stop=1) before ACK.
        ps2_line_release(p->line.clk);
        if (ps2_line_read(p->line.dat) == 1) {
            p->rx_stop_bit = 1;
            ps2_line_low(p->line.dat);
            p->rx_stage = PS2_RX_STAGE_ACK_SETUP;
        } else {
            p->rx_stage = PS2_RX_STAGE_WAIT_STOP;
        }
        p->ticks_to_step = g_ps2emu.half_ticks;
        return;
    }

    if (p->rx_stage == PS2_RX_STAGE_ACK_SETUP) {
        // ACK clock low phase.
        ps2_line_low(p->line.clk);
        p->rx_stage = PS2_RX_STAGE_ACK_LOW;
        return;
    }

    if (p->rx_stage == PS2_RX_STAGE_ACK_LOW) {
        // Release ACK by releasing both CLK and DAT in the same scheduler step.
        ps2_line_release(p->line.clk);
        ps2_line_release(p->line.dat);
        ps2_finalize_rx(port, p, woken);
        return;
    }

    ps2_finalize_rx(port, p, woken);
}

static bool IRAM_ATTR ps2_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    BaseType_t task_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&g_ps2emu.lock);
    for (int i = 0; i < PS2_PORT_MAX; ++i) {
        ps2_port_runtime_t *p = &g_ps2emu.port[i];

        if (p->host_req_pending && !p->tx_active && !p->rx_active &&
            ps2_line_read(p->line.clk) == 1 && ps2_line_read(p->line.dat) == 0) {
            ps2_start_rx(p);
        }

        if (p->tx_active || p->rx_active) {
            if (p->ticks_to_step > 0) {
                p->ticks_to_step--;
            }

            if (p->ticks_to_step == 0) {
                if (p->tx_active) {
                    ps2_step_tx(p);
                } else if (p->rx_active) {
                    ps2_step_rx((ps2_port_id_t)i, p, &task_woken);
                }

                if ((p->tx_active || p->rx_active) && p->ticks_to_step == 0) {
                    p->ticks_to_step = g_ps2emu.half_ticks;
                }
            }
        } else {
            ps2_start_tx_if_possible(p);
        }
    }
    portEXIT_CRITICAL_ISR(&g_ps2emu.lock);

    return (task_woken == pdTRUE);
}

static void IRAM_ATTR ps2_gpio_isr(void *arg)
{
    const ps2_gpio_isr_arg_t *isr_arg = (const ps2_gpio_isr_arg_t *)arg;
    if (isr_arg->port >= PS2_PORT_MAX) {
        return;
    }
    ps2_port_runtime_t *p = &g_ps2emu.port[isr_arg->port];

    int clk = ps2_line_read(p->line.clk);
    int dat = ps2_line_read(p->line.dat);

    portENTER_CRITICAL_ISR(&g_ps2emu.lock);

    if (isr_arg->is_clk) {
        if (clk == 0 && !p->tx_active && !p->rx_active) {
            p->link_state = PS2_LINK_INHIBITED;
        } else if (clk == 1 && !p->tx_active && !p->rx_active) {
            /*
             * Host request-to-send can be:
             * 1) pull CLK low, 2) pull DAT low, 3) release CLK high.
             * If DAT fell while CLK was low, detect it here on CLK rising edge.
             */
            if (dat == 0 && !p->host_req_pending) {
                p->host_req_pending = true;
                p->link_state = PS2_LINK_HOST_REQ_DETECTED;
                p->ticks_to_step = 1;
            } else if (p->link_state == PS2_LINK_INHIBITED) {
                p->link_state = PS2_LINK_IDLE;
            }
        }
    } else {
        // DAT falling while CLK is high can be a direct host request-to-send.
        if (dat == 0 && clk == 1 && !p->tx_active && !p->rx_active && !p->host_req_pending) {
            p->host_req_pending = true;
            p->link_state = PS2_LINK_HOST_REQ_DETECTED;
            p->ticks_to_step = 1;
        }
    }

    portEXIT_CRITICAL_ISR(&g_ps2emu.lock);
}

static esp_err_t ps2_gpio_init(const ps2emu_cfg_t *cfg)
{
    const gpio_num_t pins[] = {
        cfg->pc1_kbd_clk, cfg->pc1_kbd_dat, cfg->pc1_mouse_clk, cfg->pc1_mouse_dat,
        cfg->pc2_kbd_clk, cfg->pc2_kbd_dat, cfg->pc2_mouse_clk, cfg->pc2_mouse_dat,
    };

    uint64_t pin_mask = 0;
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        pin_mask |= (1ULL << pins[i]);
    }

    gpio_config_t io_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), "PS2EMU", "gpio config failed");

    for (size_t i = 0; i < PS2_PORT_MAX; ++i) {
        ps2_line_release(g_ps2emu.port[i].line.clk);
        ps2_line_release(g_ps2emu.port[i].line.dat);

        ESP_RETURN_ON_ERROR(gpio_set_intr_type(g_ps2emu.port[i].line.clk, GPIO_INTR_ANYEDGE), "PS2EMU",
                            "clk intr type failed");
        ESP_RETURN_ON_ERROR(gpio_set_intr_type(g_ps2emu.port[i].line.dat, GPIO_INTR_NEGEDGE), "PS2EMU",
                            "dat intr type failed");
    }

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    for (int i = 0; i < PS2_PORT_MAX; ++i) {
        s_isr_args[i * 2] = (ps2_gpio_isr_arg_t){.port = (ps2_port_id_t)i, .is_clk = true};
        s_isr_args[i * 2 + 1] = (ps2_gpio_isr_arg_t){.port = (ps2_port_id_t)i, .is_clk = false};

        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(g_ps2emu.port[i].line.clk, ps2_gpio_isr, &s_isr_args[i * 2]),
                            "PS2EMU", "clk isr add failed");
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(g_ps2emu.port[i].line.dat, ps2_gpio_isr, &s_isr_args[i * 2 + 1]),
                            "PS2EMU", "dat isr add failed");

        ESP_RETURN_ON_ERROR(gpio_intr_enable(g_ps2emu.port[i].line.clk), "PS2EMU", "clk intr enable failed");
        ESP_RETURN_ON_ERROR(gpio_intr_enable(g_ps2emu.port[i].line.dat), "PS2EMU", "dat intr enable failed");
    }

    return ESP_OK;
}

esp_err_t ps2emu_isr_init(const ps2emu_cfg_t *cfg)
{
    ESP_RETURN_ON_ERROR(ps2_gpio_init(cfg), "PS2EMU", "gpio init failed");

    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_cfg, &g_ps2emu.timer), "PS2EMU", "new gptimer failed");

    gptimer_event_callbacks_t cbs = {
        .on_alarm = ps2_timer_on_alarm,
    };
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(g_ps2emu.timer, &cbs, NULL), "PS2EMU", "register gptimer cb failed");
    ESP_RETURN_ON_ERROR(gptimer_enable(g_ps2emu.timer), "PS2EMU", "enable gptimer failed");

    return ESP_OK;
}

esp_err_t ps2emu_isr_start(void)
{
    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = PS2EMU_TIMER_TICK_US,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(g_ps2emu.timer, &alarm_cfg), "PS2EMU", "set alarm failed");
    ESP_RETURN_ON_ERROR(gptimer_start(g_ps2emu.timer), "PS2EMU", "start gptimer failed");
    return ESP_OK;
}
