#include "ps2emu_keyboard.h"

#include "ps2emu_internal.h"

#include "esp_check.h"
#include "esp_log.h"

typedef enum {
    KBD_RX_IDLE = 0,
    KBD_RX_WAIT_LED_MASK,
    KBD_RX_WAIT_TYPEMATIC,
    KBD_RX_WAIT_SCAN_SET,
} kbd_rx_state_t;

typedef struct {
    kbd_rx_state_t rx_state;
    bool scanning_enabled;
    bool initialized;
    uint8_t scan_set;
    ps2_kbd_led_state_t led;
} kbd_state_t;

static const char *TAG = "PS2KBD";
static kbd_state_t s_kbd[2];
static ps2_kbd_led_cb_t s_led_cb;
static void *s_led_cb_ctx;
static ps2_kbd_init_cb_t s_init_cb;
static void *s_init_cb_ctx;

static ps2_port_id_t port_from_pc(uint8_t pc_idx)
{
    return (pc_idx == 0) ? PS2_PORT_PC1_KBD : PS2_PORT_PC2_KBD;
}

static inline void kbd_send(uint8_t pc_idx, uint8_t byte)
{
    (void)ps2emu_core_send_byte(port_from_pc(pc_idx), byte);
}

static void emit_led_event_if_changed(uint8_t pc_idx, bool scroll, bool num, bool caps)
{
    bool changed = (s_kbd[pc_idx].led.scroll != scroll) ||
                   (s_kbd[pc_idx].led.num != num) ||
                   (s_kbd[pc_idx].led.caps != caps);

    s_kbd[pc_idx].led.scroll = scroll;
    s_kbd[pc_idx].led.num = num;
    s_kbd[pc_idx].led.caps = caps;

    if (!changed) {
        return;
    }

    if (s_led_cb != NULL) {
        s_led_cb(pc_idx, &s_kbd[pc_idx].led, s_led_cb_ctx);
    }
}

static void emit_init_event_if_changed(uint8_t pc_idx, bool initialized)
{
    if (s_kbd[pc_idx].initialized == initialized) {
        return;
    }
    s_kbd[pc_idx].initialized = initialized;
    if (s_init_cb) {
        s_init_cb(pc_idx, initialized, s_init_cb_ctx);
    }
}

void ps2emu_keyboard_init_state(void)
{
    for (uint8_t i = 0; i < 2; ++i) {
        s_kbd[i].rx_state = KBD_RX_IDLE;
        s_kbd[i].scanning_enabled = false;
        s_kbd[i].initialized = false;
        s_kbd[i].scan_set = 2;
        s_kbd[i].led.scroll = false;
        s_kbd[i].led.num = false;
        s_kbd[i].led.caps = false;
    }
}

void ps2emu_keyboard_send_boot_all(void)
{
    for (uint8_t i = 0; i < 2; ++i) {
        kbd_send(i, 0xAA);
    }
}

void ps2emu_keyboard_handle_host_byte(uint8_t pc_idx, uint8_t byte, bool parity_error, bool framing_error)
{
    if (pc_idx >= 2) {
        return;
    }

    kbd_state_t *kbd = &s_kbd[pc_idx];

    if (parity_error || framing_error) {
        kbd_send(pc_idx, 0xFE);
        return;
    }

    if (kbd->rx_state == KBD_RX_WAIT_LED_MASK) {
        kbd_send(pc_idx, 0xFA);
        emit_led_event_if_changed(pc_idx, (byte & 0x01) != 0, (byte & 0x02) != 0, (byte & 0x04) != 0);
        kbd->rx_state = KBD_RX_IDLE;
        return;
    }

    if (kbd->rx_state == KBD_RX_WAIT_TYPEMATIC) {
        (void)byte;
        kbd_send(pc_idx, 0xFA);
        kbd->rx_state = KBD_RX_IDLE;
        return;
    }

    if (kbd->rx_state == KBD_RX_WAIT_SCAN_SET) {
        if (byte == 0x00) {
            kbd_send(pc_idx, 0xFA);
            kbd_send(pc_idx, kbd->scan_set);
        } else {
            if (byte == 0x02) {
                kbd->scan_set = byte;
            }
            kbd_send(pc_idx, 0xFA);
        }
        kbd->rx_state = KBD_RX_IDLE;
        return;
    }

    switch (byte) {
        case 0xFF:  // Reset
            kbd_send(pc_idx, 0xFA);
            kbd_send(pc_idx, 0xAA);
            kbd->scanning_enabled = false;
            emit_init_event_if_changed(pc_idx, false);
            kbd->rx_state = KBD_RX_IDLE;
            break;

        case 0xFE:  // Resend
            kbd_send(pc_idx, ps2emu_core_get_last_tx(port_from_pc(pc_idx)));
            break;

        case 0xF6:  // Set defaults
            kbd->scanning_enabled = true;
            kbd->scan_set = 2;
            emit_led_event_if_changed(pc_idx, false, false, false);
            kbd_send(pc_idx, 0xFA);
            break;

        case 0xF5:  // Disable scanning
            kbd->scanning_enabled = false;
            kbd_send(pc_idx, 0xFA);
            break;

        case 0xF4:  // Enable scanning
            kbd->scanning_enabled = true;
            emit_init_event_if_changed(pc_idx, true);
            kbd_send(pc_idx, 0xFA);
            break;

        case 0xF3:  // Typematic
            kbd_send(pc_idx, 0xFA);
            kbd->rx_state = KBD_RX_WAIT_TYPEMATIC;
            break;

        case 0xF2:  // Get ID
            kbd_send(pc_idx, 0xFA);
            kbd_send(pc_idx, 0xAB);
            kbd_send(pc_idx, 0x83);
            break;

        case 0xF0:  // Set scan code set
            kbd_send(pc_idx, 0xFA);
            kbd->rx_state = KBD_RX_WAIT_SCAN_SET;
            break;

        case 0xEE:  // Echo
            kbd_send(pc_idx, 0xEE);
            break;

        case 0xED:  // Set/reset LEDs
            kbd_send(pc_idx, 0xFA);
            kbd->rx_state = KBD_RX_WAIT_LED_MASK;
            break;

        default:
            kbd_send(pc_idx, 0xFE);
            break;
    }
}

esp_err_t ps2emu_keyboard_set_led_callback(ps2_kbd_led_cb_t cb, void *ctx)
{
    s_led_cb = cb;
    s_led_cb_ctx = ctx;
    return ESP_OK;
}

esp_err_t ps2emu_keyboard_set_init_callback(ps2_kbd_init_cb_t cb, void *ctx)
{
    s_init_cb = cb;
    s_init_cb_ctx = ctx;
    return ESP_OK;
}

esp_err_t ps2emu_keyboard_send_scancode_bytes(uint8_t pc_idx, const uint8_t *bytes, size_t len)
{
    ESP_RETURN_ON_FALSE(pc_idx < 2, ESP_ERR_INVALID_ARG, TAG, "pc index invalid");
    ESP_RETURN_ON_FALSE(bytes != NULL || len == 0, ESP_ERR_INVALID_ARG, TAG, "bytes is NULL");
    ESP_RETURN_ON_FALSE(ps2emu_core_is_started(), ESP_ERR_INVALID_STATE, TAG, "not started");

    return ps2emu_core_send_bytes(port_from_pc(pc_idx), bytes, len);
}
