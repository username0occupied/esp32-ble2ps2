#include "ps2emu_mouse.h"

#include "ps2emu_internal.h"

#include "esp_check.h"
#include "esp_log.h"

typedef enum {
    MOUSE_RX_IDLE = 0,
    MOUSE_RX_WAIT_SAMPLE_RATE,
    MOUSE_RX_WAIT_RESOLUTION,
} mouse_rx_state_t;

typedef struct {
    mouse_rx_state_t rx_state;

    bool wheel_compat_mode;
    bool host_seen_command;
    bool demo_force_output;
    bool data_reporting;
    bool remote_mode;
    bool wrap_mode;
    bool scaling_21;

    uint8_t resolution;
    uint8_t sample_rate;
    uint8_t last_sample_rate[3];

    bool has_wheel;
    bool has_5buttons;

    uint8_t btn_left;
    uint8_t btn_right;
    uint8_t btn_middle;
    uint8_t btn_4;
    uint8_t btn_5;

    int16_t accum_x;
    int16_t accum_y;
    int8_t accum_z;
} mouse_state_t;

static const char *TAG = "PS2MOUSE";
static mouse_state_t s_mouse[2];
static bool s_logged_wheel_not_enabled[2];

static ps2_port_id_t port_from_pc(uint8_t pc_idx)
{
    return (pc_idx == 0) ? PS2_PORT_PC1_MOUSE : PS2_PORT_PC2_MOUSE;
}

static inline void mouse_send(uint8_t pc_idx, uint8_t byte)
{
    (void)ps2emu_core_send_byte(port_from_pc(pc_idx), byte);
}

static void mouse_defaults(uint8_t pc_idx, bool demo_force_output)
{
    mouse_state_t *m = &s_mouse[pc_idx];
    bool keep_wheel_compat_mode = m->wheel_compat_mode;

    m->rx_state = MOUSE_RX_IDLE;
    m->wheel_compat_mode = keep_wheel_compat_mode;
    m->host_seen_command = false;
    m->demo_force_output = demo_force_output;
    m->data_reporting = false;
    m->remote_mode = false;
    m->wrap_mode = false;
    m->scaling_21 = false;
    m->resolution = 0x02;
    m->sample_rate = 100;
    m->last_sample_rate[0] = 0;
    m->last_sample_rate[1] = 0;
    m->last_sample_rate[2] = 0;
    m->has_wheel = false;
    m->has_5buttons = false;
    m->btn_left = 0;
    m->btn_right = 0;
    m->btn_middle = 0;
    m->btn_4 = 0;
    m->btn_5 = 0;
    m->accum_x = 0;
    m->accum_y = 0;
    m->accum_z = 0;

    if (m->wheel_compat_mode) {
        m->has_wheel = true;
        m->has_5buttons = false;
    }
    s_logged_wheel_not_enabled[pc_idx] = false;
}

static bool mouse_should_stream_now(const mouse_state_t *m)
{
    (void)m;
    // Per product requirement, forward BLE mouse reports even when host has not finished initialization.
    return true;
}

static void mouse_record_sample_rate(mouse_state_t *m, uint8_t rate)
{
    m->last_sample_rate[0] = m->last_sample_rate[1];
    m->last_sample_rate[1] = m->last_sample_rate[2];
    m->last_sample_rate[2] = rate;
}

static uint8_t mouse_get_device_id_and_update_mode(mouse_state_t *m)
{
    if (m->last_sample_rate[0] == 200 && m->last_sample_rate[1] == 100 && m->last_sample_rate[2] == 80) {
        m->has_wheel = true;
        m->has_5buttons = false;
        return 0x03;
    }

    if (m->has_wheel && m->last_sample_rate[0] == 200 && m->last_sample_rate[1] == 200 && m->last_sample_rate[2] == 80) {
        m->has_5buttons = true;
        return 0x04;
    }

    if (m->wheel_compat_mode) {
        m->has_wheel = true;
        m->has_5buttons = false;
        return 0x03;
    }

    m->has_wheel = false;
    m->has_5buttons = false;
    return 0x00;
}

static int16_t clamp_i16(int16_t v, int16_t min_v, int16_t max_v)
{
    if (v < min_v) {
        return min_v;
    }
    if (v > max_v) {
        return max_v;
    }
    return v;
}

static int8_t clamp_i8(int16_t v, int8_t min_v, int8_t max_v)
{
    if (v < min_v) {
        return min_v;
    }
    if (v > max_v) {
        return max_v;
    }
    return (int8_t)v;
}

static void mouse_build_packet(mouse_state_t *m, uint8_t *out, size_t *out_len)
{
    int16_t x = m->accum_x;
    int16_t y = m->accum_y;
    int8_t z = m->accum_z;

    bool x_overflow = (x > 255 || x < -255);
    bool y_overflow = (y > 255 || y < -255);
    x = clamp_i16(x, -255, 255);
    y = clamp_i16(y, -255, 255);
    z = clamp_i8(z, -8, 7);

    out[0] = (uint8_t)((m->btn_left & 0x1) | ((m->btn_right & 0x1) << 1) | ((m->btn_middle & 0x1) << 2) |
                     (1U << 3) | ((x < 0) ? (1U << 4) : 0) | ((y < 0) ? (1U << 5) : 0) |
                     (x_overflow ? (1U << 6) : 0) | (y_overflow ? (1U << 7) : 0));
    out[1] = (uint8_t)(x & 0xFF);
    out[2] = (uint8_t)(y & 0xFF);

    if (m->has_wheel && !m->has_5buttons) {
        out[3] = m->wheel_compat_mode ? (uint8_t)(z & 0x0F) : (uint8_t)z;
        *out_len = 4;
    } else if (m->has_wheel && m->has_5buttons) {
        out[3] = (uint8_t)((z & 0x0F) | ((m->btn_4 & 0x1) << 4) | ((m->btn_5 & 0x1) << 5));
        *out_len = 4;
    } else {
        *out_len = 3;
    }

    m->accum_x = 0;
    m->accum_y = 0;
    m->accum_z = 0;
}

static void mouse_send_report_packet(uint8_t pc_idx)
{
    mouse_state_t *m = &s_mouse[pc_idx];
    uint8_t packet[4] = {0};
    size_t packet_len = 0;
    mouse_build_packet(m, packet, &packet_len);
    (void)ps2emu_core_send_bytes(port_from_pc(pc_idx), packet, packet_len);
}

static void mouse_send_status(uint8_t pc_idx)
{
    mouse_state_t *m = &s_mouse[pc_idx];
    uint8_t status = 0;
    status |= (m->btn_right & 0x1);
    status |= (uint8_t)((m->btn_middle & 0x1) << 1);
    status |= (uint8_t)((m->btn_left & 0x1) << 2);
    status |= (uint8_t)((m->scaling_21 ? 1U : 0U) << 4);
    status |= (uint8_t)((m->data_reporting ? 1U : 0U) << 5);
    status |= (uint8_t)((m->remote_mode ? 1U : 0U) << 6);

    mouse_send(pc_idx, status);
    mouse_send(pc_idx, m->resolution & 0x03);
    mouse_send(pc_idx, m->sample_rate);
}

void ps2emu_mouse_init_state(void)
{
    for (uint8_t i = 0; i < 2; ++i) {
        memset(&s_mouse[i], 0, sizeof(s_mouse[i]));
        s_mouse[i].wheel_compat_mode = true;
        mouse_defaults(i, true);
    }
}

void ps2emu_mouse_send_boot_all(void)
{
    for (uint8_t i = 0; i < 2; ++i) {
        mouse_send(i, 0xAA);
        mouse_send(i, 0x00);
    }
}

void ps2emu_mouse_handle_host_byte(uint8_t pc_idx, uint8_t byte, bool parity_error, bool framing_error)
{
    if (pc_idx >= 2) {
        return;
    }

    mouse_state_t *m = &s_mouse[pc_idx];

    m->host_seen_command = true;
    m->demo_force_output = false;

    if (parity_error || framing_error) {
        mouse_send(pc_idx, 0xFE);
        return;
    }

    if (m->wrap_mode) {
        if (byte == 0xFF) {
            m->wrap_mode = false;
            mouse_send(pc_idx, 0xFA);
            mouse_defaults(pc_idx, false);
            mouse_send(pc_idx, 0xAA);
            mouse_send(pc_idx, 0x00);
            return;
        }
        if (byte == 0xEC) {
            mouse_send(pc_idx, 0xFA);
            m->wrap_mode = false;
            return;
        }
        if (byte == 0xEE) {
            mouse_send(pc_idx, 0xFA);
            return;
        }

        mouse_send(pc_idx, byte);
        return;
    }

    if (m->rx_state == MOUSE_RX_WAIT_SAMPLE_RATE) {
        m->sample_rate = byte;
        mouse_record_sample_rate(m, byte);
        mouse_send(pc_idx, 0xFA);
        m->rx_state = MOUSE_RX_IDLE;
        return;
    }

    if (m->rx_state == MOUSE_RX_WAIT_RESOLUTION) {
        if (byte <= 3) {
            m->resolution = byte;
        }
        mouse_send(pc_idx, 0xFA);
        m->rx_state = MOUSE_RX_IDLE;
        return;
    }

    switch (byte) {
        case 0xFF:  // Reset
            mouse_send(pc_idx, 0xFA);
            mouse_defaults(pc_idx, false);
            mouse_send(pc_idx, 0xAA);
            mouse_send(pc_idx, 0x00);
            break;

        case 0xFE:  // Resend
            mouse_send(pc_idx, ps2emu_core_get_last_tx(port_from_pc(pc_idx)));
            break;

        case 0xF6:  // Set defaults
            mouse_defaults(pc_idx, false);
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xF5:  // Disable data reporting
            m->data_reporting = false;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xF4:  // Enable data reporting
            m->data_reporting = true;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xF3:  // Set sample rate
            mouse_send(pc_idx, 0xFA);
            m->rx_state = MOUSE_RX_WAIT_SAMPLE_RATE;
            break;

        case 0xF2:  // Get device ID
            mouse_send(pc_idx, 0xFA);
            mouse_send(pc_idx, mouse_get_device_id_and_update_mode(m));
            break;

        case 0xF0:  // Set remote mode
            m->remote_mode = true;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xEA:  // Set stream mode
            m->remote_mode = false;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xEB:  // Read data
            mouse_send(pc_idx, 0xFA);
            mouse_send_report_packet(pc_idx);
            break;

        case 0xE8:  // Set resolution
            mouse_send(pc_idx, 0xFA);
            m->rx_state = MOUSE_RX_WAIT_RESOLUTION;
            break;

        case 0xE9:  // Status request
            mouse_send(pc_idx, 0xFA);
            mouse_send_status(pc_idx);
            break;

        case 0xE6:  // Set scaling 1:1
            m->scaling_21 = false;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xE7:  // Set scaling 2:1
            m->scaling_21 = true;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xEE:  // Set wrap mode
            m->wrap_mode = true;
            mouse_send(pc_idx, 0xFA);
            break;

        case 0xEC:  // Reset wrap mode
            m->wrap_mode = false;
            mouse_send(pc_idx, 0xFA);
            break;

        default:
            mouse_send(pc_idx, 0xFE);
            break;
    }
}

esp_err_t ps2emu_mouse_send_report(uint8_t pc_idx, int8_t dx, int8_t dy, int8_t wheel,
                                   uint8_t left, uint8_t right, uint8_t middle,
                                   uint8_t b4, uint8_t b5)
{
    ESP_RETURN_ON_FALSE(pc_idx < 2, ESP_ERR_INVALID_ARG, TAG, "pc index invalid");
    ESP_RETURN_ON_FALSE(ps2emu_core_is_started(), ESP_ERR_INVALID_STATE, TAG, "not started");

    mouse_state_t *m = &s_mouse[pc_idx];

    if (wheel != 0 && !m->has_wheel && !s_logged_wheel_not_enabled[pc_idx]) {
        ESP_LOGW(TAG, "PC%u wheel delta dropped because wheel mode is not enabled", (unsigned)pc_idx + 1);
        s_logged_wheel_not_enabled[pc_idx] = true;
    }

    m->btn_left = (left != 0) ? 1 : 0;
    m->btn_right = (right != 0) ? 1 : 0;
    m->btn_middle = (middle != 0) ? 1 : 0;
    m->btn_4 = (b4 != 0) ? 1 : 0;
    m->btn_5 = (b5 != 0) ? 1 : 0;

    m->accum_x += dx;
    m->accum_y += dy;
    m->accum_z = (int8_t)(m->accum_z + wheel);

    if (mouse_should_stream_now(m)) {
        mouse_send_report_packet(pc_idx);
    }

    return ESP_OK;
}
