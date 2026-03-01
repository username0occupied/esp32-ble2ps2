/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "nvs.h"
#include "esp_bt.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#define ESP_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define ESP_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#else
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#endif

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "bt_hid_host.h"

typedef enum {
    HID_ROLE_UNKNOWN = 0,
    HID_ROLE_KEYBOARD,
    HID_ROLE_MOUSE,
} hid_role_t;

static const char *TAG = "ESP_HIDH_DEMO";

typedef struct {
    bool valid;
    uint8_t bda[6];
    esp_hid_transport_t transport;
    uint8_t addr_type;
    esp_hid_usage_t usage;
    hid_role_t role;
} hid_known_device_t;

#define HID_EVT_SCAN_REQUEST BIT0
#define HID_EVT_SCAN_IN_PROGRESS BIT1

#define HID_SCAN_RETRY_DELAY_MS 2000
#define HID_OPEN_TIMEOUT_MS 8000
#define HID_SCAN_IDLE_CHECK_MS 1000
#define SCAN_DURATION_SECONDS 5
#define HID_INPUT_VERBOSE_LOG 0
#define HID_LOW_LATENCY_CONN_MIN_INT 6   // 7.5 ms
#define HID_LOW_LATENCY_CONN_MAX_INT 9   // 11.25 ms
#define HID_LOW_LATENCY_CONN_LATENCY 0
#define HID_LOW_LATENCY_CONN_TIMEOUT 400 // 4 s

static EventGroupHandle_t s_hid_evt_group;
static hid_known_device_t s_known_kbd;
static hid_known_device_t s_known_mouse;
static hid_known_device_t s_pending_kbd;
static hid_known_device_t s_pending_mouse;
static bool s_kbd_connected;
static bool s_mouse_connected;
static int64_t s_kbd_open_ts;
static int64_t s_mouse_open_ts;
static bool s_mouse_min_interval_locked;
static uint16_t s_mouse_min_interval_units;
static uint8_t s_mouse_conn_probe_stage;
static bool s_mouse_conn_probe_active;
static bt_hid_host_callbacks_t s_callbacks;
static void *s_callbacks_ctx;
static bool s_started;

#if !CONFIG_BT_NIMBLE_ENABLED
static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}
#endif

static void hid_known_clear(hid_known_device_t *dev)
{
    memset(dev, 0, sizeof(*dev));
    dev->valid = false;
    dev->role = HID_ROLE_UNKNOWN;
}

static bool hid_bda_equal(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

static bool hid_usage_is_keyboard(esp_hid_usage_t usage)
{
    return (usage & ESP_HID_USAGE_KEYBOARD) != 0;
}

static bool hid_usage_is_mouse(esp_hid_usage_t usage)
{
    return (usage & ESP_HID_USAGE_MOUSE) != 0;
}

static hid_role_t hid_role_from_usage_appearance(esp_hid_usage_t usage, uint16_t appearance, esp_hid_transport_t transport)
{
    if (hid_usage_is_keyboard(usage)) {
        return HID_ROLE_KEYBOARD;
    }
    if (hid_usage_is_mouse(usage)) {
        return HID_ROLE_MOUSE;
    }
    if (transport == ESP_HID_TRANSPORT_BLE) {
        if (appearance == ESP_HID_APPEARANCE_KEYBOARD) {
            return HID_ROLE_KEYBOARD;
        }
        if (appearance == ESP_HID_APPEARANCE_MOUSE) {
            return HID_ROLE_MOUSE;
        }
    }
    return HID_ROLE_UNKNOWN;
}

static hid_role_t hid_role_from_bda(const uint8_t *bda, esp_hid_usage_t usage, esp_hid_transport_t transport, uint16_t appearance)
{
    if (s_pending_kbd.valid && hid_bda_equal(bda, s_pending_kbd.bda)) {
        return HID_ROLE_KEYBOARD;
    }
    if (s_pending_mouse.valid && hid_bda_equal(bda, s_pending_mouse.bda)) {
        return HID_ROLE_MOUSE;
    }
    if (s_known_kbd.valid && hid_bda_equal(bda, s_known_kbd.bda)) {
        return HID_ROLE_KEYBOARD;
    }
    if (s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda)) {
        return HID_ROLE_MOUSE;
    }
    return hid_role_from_usage_appearance(usage, appearance, transport);
}

static hid_role_t hid_role_from_reports(esp_hidh_dev_t *dev)
{
    size_t num_reports = 0;
    esp_hid_report_item_t *reports = NULL;

    if (esp_hidh_dev_reports_get(dev, &num_reports, &reports) != ESP_OK || reports == NULL) {
        return HID_ROLE_UNKNOWN;
    }
    for (size_t i = 0; i < num_reports; i++) {
        if (hid_usage_is_keyboard(reports[i].usage)) {
            return HID_ROLE_KEYBOARD;
        }
        if (hid_usage_is_mouse(reports[i].usage)) {
            return HID_ROLE_MOUSE;
        }
    }
    return HID_ROLE_UNKNOWN;
}

static bool hid_both_connected(void)
{
    return s_kbd_connected && s_mouse_connected;
}

static void emit_kbd_conn(bool connected)
{
    if (s_callbacks.on_kbd_conn) {
        s_callbacks.on_kbd_conn(connected, s_callbacks_ctx);
    }
}

static void emit_mouse_conn(bool connected)
{
    if (s_callbacks.on_mouse_conn) {
        s_callbacks.on_mouse_conn(connected, s_callbacks_ctx);
    }
}

static void emit_passkey(bool show, uint32_t passkey)
{
    if (s_callbacks.on_passkey) {
        s_callbacks.on_passkey(show, passkey, s_callbacks_ctx);
    }
}

static hid_role_t hid_role_from_scan(const esp_hid_scan_result_t *r)
{
    if (r->transport == ESP_HID_TRANSPORT_BLE) {
        if (r->ble.appearance == ESP_HID_APPEARANCE_KEYBOARD) {
            return HID_ROLE_KEYBOARD;
        }
        if (r->ble.appearance == ESP_HID_APPEARANCE_MOUSE) {
            return HID_ROLE_MOUSE;
        }
    }
    if (hid_usage_is_keyboard(r->usage)) {
        return HID_ROLE_KEYBOARD;
    }
    if (hid_usage_is_mouse(r->usage)) {
        return HID_ROLE_MOUSE;
    }
    return HID_ROLE_UNKNOWN;
}

typedef struct {
    uint8_t valid;
    uint8_t bda[6];
    uint8_t transport;
    uint8_t addr_type;
    uint8_t usage;
} hid_known_device_nvs_t;

#define HID_NVS_NAMESPACE "hid_host"
#define HID_NVS_KEY_KBD   "kbd"
#define HID_NVS_KEY_MOUSE "mouse"

static void hid_load_known_device(const char *key, hid_known_device_t *dev)
{
    nvs_handle_t handle;
    size_t size = sizeof(hid_known_device_nvs_t);
    hid_known_device_nvs_t stored = {0};

    hid_known_clear(dev);
    if (nvs_open(HID_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    if (nvs_get_blob(handle, key, &stored, &size) == ESP_OK && size == sizeof(stored) && stored.valid) {
        dev->valid = true;
        memcpy(dev->bda, stored.bda, sizeof(dev->bda));
        dev->transport = (esp_hid_transport_t)stored.transport;
        dev->addr_type = stored.addr_type;
        dev->usage = (esp_hid_usage_t)stored.usage;
    }
    nvs_close(handle);
}

static void hid_store_known_device(const char *key, const hid_known_device_t *dev)
{
    nvs_handle_t handle;
    hid_known_device_nvs_t stored = {0};

    if (nvs_open(HID_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed for key %s", key);
        return;
    }

    stored.valid = dev->valid ? 1 : 0;
    memcpy(stored.bda, dev->bda, sizeof(stored.bda));
    stored.transport = (uint8_t)dev->transport;
    stored.addr_type = dev->addr_type;
    stored.usage = (uint8_t)dev->usage;

    if (nvs_set_blob(handle, key, &stored, sizeof(stored)) != ESP_OK || nvs_commit(handle) != ESP_OK) {
        ESP_LOGW(TAG, "NVS save failed for key %s", key);
    }
    nvs_close(handle);
}

static void hid_load_known_devices(void)
{
    hid_load_known_device(HID_NVS_KEY_KBD, &s_known_kbd);
    hid_load_known_device(HID_NVS_KEY_MOUSE, &s_known_mouse);
}

static void hid_update_scan_targets(void)
{
    uint8_t bda_list[2][6];
    size_t num = 0;

    if (s_known_kbd.valid) {
        memcpy(bda_list[num], s_known_kbd.bda, sizeof(bda_list[num]));
        num++;
    }
    if (s_known_mouse.valid) {
        memcpy(bda_list[num], s_known_mouse.bda, sizeof(bda_list[num]));
        num++;
    }

    esp_hid_scan_set_known_bda(bda_list, num);
    ESP_LOGI(TAG, "Known BDA list: %u", (unsigned)num);
    if (s_known_kbd.valid) {
        ESP_LOGI(TAG, "Known KBD: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(s_known_kbd.bda));
    }
    if (s_known_mouse.valid) {
        ESP_LOGI(TAG, "Known MOUSE: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(s_known_mouse.bda));
    }
}

static uint8_t hid_addr_type_for_bda(const uint8_t *bda, esp_hid_transport_t transport)
{
    if (transport != ESP_HID_TRANSPORT_BLE) {
        return 0;
    }
    if (s_pending_kbd.valid && hid_bda_equal(bda, s_pending_kbd.bda)) {
        return s_pending_kbd.addr_type;
    }
    if (s_pending_mouse.valid && hid_bda_equal(bda, s_pending_mouse.bda)) {
        return s_pending_mouse.addr_type;
    }
    if (s_known_kbd.valid && hid_bda_equal(bda, s_known_kbd.bda)) {
        return s_known_kbd.addr_type;
    }
    if (s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda)) {
        return s_known_mouse.addr_type;
    }
    return 0;
}

static void hid_request_scan(void)
{
    if (s_hid_evt_group) {
        if (hid_both_connected()) {
            return;
        }
        xEventGroupSetBits(s_hid_evt_group, HID_EVT_SCAN_REQUEST);
    }
}

static void hid_clear_pending_by_bda(const uint8_t *bda)
{
    if (s_pending_kbd.valid && hid_bda_equal(bda, s_pending_kbd.bda)) {
        hid_known_clear(&s_pending_kbd);
        s_kbd_open_ts = 0;
    }
    if (s_pending_mouse.valid && hid_bda_equal(bda, s_pending_mouse.bda)) {
        hid_known_clear(&s_pending_mouse);
        s_mouse_open_ts = 0;
    }
}

#if !CONFIG_BT_NIMBLE_ENABLED
typedef struct {
    uint16_t min_int;
    uint16_t max_int;
} hid_conn_probe_profile_t;

static const hid_conn_probe_profile_t s_mouse_conn_probe_profiles[] = {
    {HID_LOW_LATENCY_CONN_MIN_INT, HID_LOW_LATENCY_CONN_MAX_INT}, // 7.5~11.25 ms
    {10, 12},                                                      // 12.5~15 ms
    {13, 16},                                                      // 16.25~20 ms
};

static esp_err_t hid_request_mouse_conn_interval_exact(const uint8_t *bda, uint16_t interval_units)
{
    if (bda == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (interval_units < 0x0006 || interval_units > 0x0C80) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, bda, sizeof(conn_params.bda));
    conn_params.min_int = interval_units;
    conn_params.max_int = interval_units;
    conn_params.latency = HID_LOW_LATENCY_CONN_LATENCY;
    conn_params.timeout = HID_LOW_LATENCY_CONN_TIMEOUT;
    return esp_ble_gap_update_conn_params(&conn_params);
}

static esp_err_t hid_request_mouse_conn_interval_window(const uint8_t *bda, uint16_t min_int, uint16_t max_int)
{
    if (bda == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (min_int < 0x0006 || max_int > 0x0C80 || min_int > max_int) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, bda, sizeof(conn_params.bda));
    conn_params.min_int = min_int;
    conn_params.max_int = max_int;
    conn_params.latency = HID_LOW_LATENCY_CONN_LATENCY;
    conn_params.timeout = HID_LOW_LATENCY_CONN_TIMEOUT;
    return esp_ble_gap_update_conn_params(&conn_params);
}

static void hid_request_low_latency_conn_params(esp_hidh_dev_t *dev, hid_role_t role)
{
    if (dev == NULL || role != HID_ROLE_MOUSE) {
        return;
    }
    if (s_mouse_min_interval_locked || s_mouse_conn_probe_active) {
        return;
    }

    if (esp_hidh_dev_transport_get(dev) != ESP_HID_TRANSPORT_BLE) {
        return;
    }

    const uint8_t *bda = esp_hidh_dev_bda_get(dev);
    if (bda == NULL) {
        return;
    }

    const uint8_t stage = 0;
    s_mouse_conn_probe_stage = stage;
    const hid_conn_probe_profile_t *p = &s_mouse_conn_probe_profiles[stage];

    esp_err_t err = hid_request_mouse_conn_interval_window(bda, p->min_int, p->max_int);
    if (err != ESP_OK) {
        s_mouse_conn_probe_active = false;
        ESP_LOGW(TAG,
                 "conn param probe request failed for mouse " ESP_BD_ADDR_STR
                 " stage=%u (min=%u max=%u): %s",
                 ESP_BD_ADDR_HEX(bda),
                 (unsigned)stage,
                 (unsigned)p->min_int,
                 (unsigned)p->max_int,
                 esp_err_to_name(err));
    } else {
        s_mouse_conn_probe_active = true;
        ESP_LOGI(TAG,
                 "requested mouse conn param probe " ESP_BD_ADDR_STR
                 " stage=%u (min=%u max=%u)",
                 ESP_BD_ADDR_HEX(bda),
                 (unsigned)stage,
                 (unsigned)p->min_int,
                 (unsigned)p->max_int);
    }
}

static void hid_gap_conn_params_cb(const esp_hid_gap_conn_params_evt_t *evt, void *ctx)
{
    (void)ctx;
    if (evt == NULL) {
        return;
    }
    if (!s_known_mouse.valid || !hid_bda_equal(evt->bda, s_known_mouse.bda)) {
        return;
    }

    if (evt->status != ESP_BT_STATUS_SUCCESS) {
        if (s_mouse_min_interval_locked) {
            return;
        }
        ESP_LOGI(TAG,
                 "mouse conn params observed " ESP_BD_ADDR_STR
                 " min=%u max=%u lat=%u conn=%ums to=%ums",
                 ESP_BD_ADDR_HEX(evt->bda),
                 (unsigned)evt->min_int,
                 (unsigned)evt->max_int,
                 (unsigned)evt->latency,
                 (unsigned)evt->conn_int_ms,
                 (unsigned)evt->timeout_ms);
        if (s_mouse_conn_probe_stage + 1 >= (uint8_t)(sizeof(s_mouse_conn_probe_profiles) / sizeof(s_mouse_conn_probe_profiles[0]))) {
            s_mouse_conn_probe_active = false;
            ESP_LOGW(TAG, "mouse conn probe exhausted, keep current interval (conn=%ums)",
                     (unsigned)evt->conn_int_ms);
            return;
        }

        const uint8_t next_stage = s_mouse_conn_probe_stage + 1;
        s_mouse_conn_probe_stage = next_stage;
        const hid_conn_probe_profile_t *p = &s_mouse_conn_probe_profiles[next_stage];
        esp_err_t probe_err = hid_request_mouse_conn_interval_window(evt->bda, p->min_int, p->max_int);
        if (probe_err != ESP_OK) {
            s_mouse_conn_probe_active = false;
            ESP_LOGW(TAG,
                     "mouse conn probe request failed " ESP_BD_ADDR_STR
                     " stage=%u (min=%u max=%u): %s",
                     ESP_BD_ADDR_HEX(evt->bda),
                     (unsigned)next_stage,
                     (unsigned)p->min_int,
                     (unsigned)p->max_int,
                     esp_err_to_name(probe_err));
        } else {
            s_mouse_conn_probe_active = true;
            ESP_LOGI(TAG,
                     "requested mouse conn probe " ESP_BD_ADDR_STR
                     " stage=%u (min=%u max=%u)",
                     ESP_BD_ADDR_HEX(evt->bda),
                     (unsigned)next_stage,
                     (unsigned)p->min_int,
                     (unsigned)p->max_int);
        }
        return;
    }

    ESP_LOGI(TAG,
             "mouse conn params observed " ESP_BD_ADDR_STR
             " min=%u max=%u lat=%u conn=%ums to=%ums",
             ESP_BD_ADDR_HEX(evt->bda),
             (unsigned)evt->min_int,
             (unsigned)evt->max_int,
             (unsigned)evt->latency,
             (unsigned)evt->conn_int_ms,
             (unsigned)evt->timeout_ms);

    // For accepted updates, min_int/max_int carry interval units (1.25ms steps) and
    // are more reliable than conn_int_ms (which may be rounded/truncated by stack logs).
    uint16_t exact_units = 0;
    if (evt->min_int >= 0x0006 && evt->min_int <= 0x0C80) {
        exact_units = evt->min_int;
    } else if (evt->conn_int_ms >= 0x0006 && evt->conn_int_ms <= 0x0C80) {
        exact_units = evt->conn_int_ms;
    }
    if (exact_units < 0x0006 || exact_units > 0x0C80) {
        s_mouse_conn_probe_active = false;
        return;
    }
    if (s_mouse_min_interval_locked && s_mouse_min_interval_units == exact_units) {
        s_mouse_conn_probe_active = false;
        return;
    }

    esp_err_t err = hid_request_mouse_conn_interval_exact(evt->bda, exact_units);
    if (err != ESP_OK) {
        s_mouse_conn_probe_active = false;
        ESP_LOGW(TAG, "request exact mouse interval failed " ESP_BD_ADDR_STR " units=%u: %s",
                 ESP_BD_ADDR_HEX(evt->bda), (unsigned)exact_units, esp_err_to_name(err));
        return;
    }

    s_mouse_min_interval_locked = true;
    s_mouse_min_interval_units = exact_units;
    s_mouse_conn_probe_active = false;
    ESP_LOGI(TAG, "requested exact mouse interval " ESP_BD_ADDR_STR " -> %u (%.2f ms)",
             ESP_BD_ADDR_HEX(evt->bda),
             (unsigned)exact_units,
             (double)exact_units * 1.25);
}
#endif

static void hid_on_open_success(esp_hidh_dev_t *dev)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(dev);
    esp_hid_transport_t transport = esp_hidh_dev_transport_get(dev);
    esp_hid_usage_t usage = esp_hidh_dev_usage_get(dev);
    uint8_t addr_type = hid_addr_type_for_bda(bda, transport);
    hid_role_t report_role = hid_role_from_reports(dev);
    hid_role_t actual_role = report_role;
    if (actual_role == HID_ROLE_UNKNOWN) {
        actual_role = hid_role_from_usage_appearance(usage, 0, transport);
    }
    bool pending_kbd = s_pending_kbd.valid;
    bool pending_mouse = s_pending_mouse.valid;
    bool matched_pending_kbd = pending_kbd && hid_bda_equal(bda, s_pending_kbd.bda);
    bool matched_pending_mouse = pending_mouse && hid_bda_equal(bda, s_pending_mouse.bda);
    if (!matched_pending_kbd && !matched_pending_mouse) {
        if (pending_kbd && !pending_mouse) {
            matched_pending_kbd = true;
        } else if (pending_mouse && !pending_kbd) {
            matched_pending_mouse = true;
        }
    }

    if (matched_pending_kbd) {
        hid_known_clear(&s_pending_kbd);
        s_kbd_open_ts = 0;
    }
    if (matched_pending_mouse) {
        hid_known_clear(&s_pending_mouse);
        s_mouse_open_ts = 0;
    }

    if (actual_role == HID_ROLE_UNKNOWN) {
        if (matched_pending_kbd && !matched_pending_mouse) {
            actual_role = HID_ROLE_KEYBOARD;
        } else if (matched_pending_mouse && !matched_pending_kbd) {
            actual_role = HID_ROLE_MOUSE;
        } else {
            actual_role = hid_role_from_bda(bda, usage, transport, 0);
        }
    }

    if (actual_role == HID_ROLE_KEYBOARD) {
        s_kbd_connected = true;
        emit_kbd_conn(true);
        s_known_kbd.valid = true;
        memcpy(s_known_kbd.bda, bda, sizeof(s_known_kbd.bda));
        s_known_kbd.transport = transport;
        s_known_kbd.addr_type = addr_type;
        s_known_kbd.usage = usage;
        s_known_kbd.role = HID_ROLE_KEYBOARD;
        hid_store_known_device(HID_NVS_KEY_KBD, &s_known_kbd);
        if (s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda)) {
            hid_known_clear(&s_known_mouse);
        }
    } else if (actual_role == HID_ROLE_MOUSE) {
        bool same_mouse_reopen = s_mouse_connected && s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda);
        s_mouse_connected = true;
        emit_mouse_conn(true);
        s_known_mouse.valid = true;
        memcpy(s_known_mouse.bda, bda, sizeof(s_known_mouse.bda));
        s_known_mouse.transport = transport;
        s_known_mouse.addr_type = addr_type;
        s_known_mouse.usage = usage;
        s_known_mouse.role = HID_ROLE_MOUSE;
        if (!same_mouse_reopen) {
            s_mouse_min_interval_locked = false;
            s_mouse_min_interval_units = 0;
            s_mouse_conn_probe_stage = 0;
            s_mouse_conn_probe_active = false;
        }
        hid_store_known_device(HID_NVS_KEY_MOUSE, &s_known_mouse);
        if (s_known_kbd.valid && hid_bda_equal(bda, s_known_kbd.bda)) {
            hid_known_clear(&s_known_kbd);
        }
    }

#if !CONFIG_BT_NIMBLE_ENABLED
    hid_request_low_latency_conn_params(dev, actual_role);
#endif

    hid_update_scan_targets();
}

typedef struct {
    bool valid;
    uint8_t bda[6];
    esp_hid_transport_t transport;
    uint8_t addr_type;
    esp_hid_usage_t usage;
} hid_open_candidate_t;

static void hid_set_candidate(hid_open_candidate_t *cand, const esp_hid_scan_result_t *r)
{
    if (cand->valid) {
        return;
    }
    cand->valid = true;
    memcpy(cand->bda, r->bda, sizeof(cand->bda));
    cand->transport = r->transport;
    cand->usage = r->usage;
    cand->addr_type = (r->transport == ESP_HID_TRANSPORT_BLE) ? r->ble.addr_type : 0;
}

static void hid_set_candidate_from_known(hid_open_candidate_t *cand, const hid_known_device_t *known)
{
    if (cand->valid || !known->valid) {
        return;
    }
    cand->valid = true;
    memcpy(cand->bda, known->bda, sizeof(cand->bda));
    cand->transport = known->transport;
    cand->usage = known->usage;
    cand->addr_type = known->addr_type;
}

static void hid_try_open_candidate(const hid_open_candidate_t *cand, hid_known_device_t *pending, hid_role_t role)
{
    if (!cand->valid || pending->valid) {
        return;
    }
    pending->valid = true;
    memcpy(pending->bda, cand->bda, sizeof(pending->bda));
    pending->transport = cand->transport;
    pending->usage = cand->usage;
    pending->addr_type = cand->addr_type;
    pending->role = role;
    if (role == HID_ROLE_KEYBOARD) {
        s_kbd_open_ts = esp_timer_get_time();
    } else if (role == HID_ROLE_MOUSE) {
        s_mouse_open_ts = esp_timer_get_time();
    }
    ESP_LOGI(TAG, "OPEN try %s " ESP_BD_ADDR_STR " (%s)",
             role == HID_ROLE_KEYBOARD ? "KEYBOARD" : "MOUSE",
             ESP_BD_ADDR_HEX(cand->bda),
             cand->transport == ESP_HID_TRANSPORT_BLE ? "BLE" : "BT");
    uint8_t bda[6];
    memcpy(bda, cand->bda, sizeof(bda));
    esp_hidh_dev_open(bda, cand->transport,
                      cand->transport == ESP_HID_TRANSPORT_BLE ? cand->addr_type : 0);
}

static bool hid_need_scan_retry(void)
{
    if (hid_both_connected()) {
        return false;
    }
    if (!s_kbd_connected && !s_pending_kbd.valid) {
        return true;
    }
    if (!s_mouse_connected && !s_pending_mouse.valid) {
        return true;
    }
    return false;
}

static bool hid_check_open_timeouts(void)
{
    if (HID_OPEN_TIMEOUT_MS <= 0) {
        return false;
    }
    int64_t now_us = esp_timer_get_time();
    bool cleared = false;
    if (s_pending_kbd.valid && s_kbd_open_ts > 0 && !s_kbd_connected &&
        (now_us - s_kbd_open_ts) > (int64_t)HID_OPEN_TIMEOUT_MS * 1000) {
        ESP_LOGW(TAG, "OPEN timeout for keyboard, retry scan");
        hid_known_clear(&s_pending_kbd);
        s_kbd_open_ts = 0;
        cleared = true;
    }
    if (s_pending_mouse.valid && s_mouse_open_ts > 0 && !s_mouse_connected &&
        (now_us - s_mouse_open_ts) > (int64_t)HID_OPEN_TIMEOUT_MS * 1000) {
        ESP_LOGW(TAG, "OPEN timeout for mouse, retry scan");
        hid_known_clear(&s_pending_mouse);
        s_mouse_open_ts = 0;
        cleared = true;
    }
    return cleared;
}

static void hid_scan_task(void *pvParameters)
{
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(s_hid_evt_group, HID_EVT_SCAN_REQUEST, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(HID_SCAN_IDLE_CHECK_MS));
        if (hid_both_connected()) {
            continue;
        }
        if (bits == 0) {
            if (!hid_check_open_timeouts()) {
                continue;
            }
        } else {
            hid_check_open_timeouts();
        }
        xEventGroupSetBits(s_hid_evt_group, HID_EVT_SCAN_IN_PROGRESS);

        size_t results_len = 0;
        esp_hid_scan_result_t *results = NULL;
        hid_open_candidate_t kbd_known = {0};
        hid_open_candidate_t kbd_any = {0};
        hid_open_candidate_t mouse_known = {0};
        hid_open_candidate_t mouse_any = {0};

        ESP_LOGI(TAG, "SCAN...");
        if (esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results) == ESP_OK) {
            ESP_LOGI(TAG, "SCAN: %u results", results_len);
            for (esp_hid_scan_result_t *r = results; r; r = r->next) {
                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    ESP_LOGI(TAG, "SCAN dev " ESP_BD_ADDR_STR " usage:%s appearance:0x%04x",
                             ESP_BD_ADDR_HEX(r->bda), esp_hid_usage_str(r->usage), r->ble.appearance);
                } else {
                    ESP_LOGI(TAG, "SCAN dev " ESP_BD_ADDR_STR " usage:%s",
                             ESP_BD_ADDR_HEX(r->bda), esp_hid_usage_str(r->usage));
                }
                hid_role_t scan_role = hid_role_from_scan(r);
                if (s_known_kbd.valid && hid_bda_equal(r->bda, s_known_kbd.bda)) {
                    hid_set_candidate(&kbd_known, r);
                    continue;
                }
                if (s_known_mouse.valid && hid_bda_equal(r->bda, s_known_mouse.bda)) {
                    if (scan_role == HID_ROLE_KEYBOARD) {
                        hid_set_candidate(&kbd_known, r);
                    } else {
                        hid_set_candidate(&mouse_known, r);
                    }
                    continue;
                }
                if (scan_role == HID_ROLE_KEYBOARD) {
                    hid_set_candidate(&kbd_any, r);
                } else if (scan_role == HID_ROLE_MOUSE) {
                    hid_set_candidate(&mouse_any, r);
                }
            }
            if (!s_kbd_connected) {
                if (kbd_known.valid) {
                    hid_try_open_candidate(&kbd_known, &s_pending_kbd, HID_ROLE_KEYBOARD);
                } else {
                    hid_try_open_candidate(&kbd_any, &s_pending_kbd, HID_ROLE_KEYBOARD);
                }
                if (!s_pending_kbd.valid) {
                    hid_set_candidate_from_known(&kbd_known, &s_known_kbd);
                    hid_try_open_candidate(&kbd_known, &s_pending_kbd, HID_ROLE_KEYBOARD);
                }
            }
            if (!s_mouse_connected) {
                if (mouse_known.valid) {
                    hid_try_open_candidate(&mouse_known, &s_pending_mouse, HID_ROLE_MOUSE);
                } else {
                    hid_try_open_candidate(&mouse_any, &s_pending_mouse, HID_ROLE_MOUSE);
                }
                if (!s_pending_mouse.valid) {
                    hid_set_candidate_from_known(&mouse_known, &s_known_mouse);
                    hid_try_open_candidate(&mouse_known, &s_pending_mouse, HID_ROLE_MOUSE);
                }
            }
            if (results) {
                esp_hid_scan_results_free(results);
            }
        } else {
            ESP_LOGW(TAG, "SCAN failed");
        }

        xEventGroupClearBits(s_hid_evt_group, HID_EVT_SCAN_IN_PROGRESS);
        if (hid_need_scan_retry()) {
            vTaskDelay(pdMS_TO_TICKS(HID_SCAN_RETRY_DELAY_MS));
            hid_request_scan();
        }
    }
}

static hid_role_t hid_role_from_dev(esp_hidh_dev_t *dev, esp_hid_usage_t usage_hint)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(dev);
    esp_hid_transport_t transport = esp_hidh_dev_transport_get(dev);
    esp_hid_usage_t usage = usage_hint;

    // Some stacks report generic usage in the input callback; fallback to device-level usage.
    if (usage == ESP_HID_USAGE_GENERIC) {
        usage = esp_hidh_dev_usage_get(dev);
    }

    hid_role_t role = hid_role_from_bda(bda, usage, transport, 0);
    if (role == HID_ROLE_UNKNOWN) {
        role = hid_role_from_reports(dev);
    }
    return role;
}

static hid_role_t hid_role_from_payload_shape(const esp_hidh_event_data_t *param)
{
    size_t len = param->input.length;
    uint8_t report_id = (uint8_t)(param->input.report_id & 0xFF);

    // Common split profile layout: keyboard on report-id 1, mouse on report-id 2.
    if (report_id == 1 && len >= 7) {
        return HID_ROLE_KEYBOARD;
    }
    if (report_id == 2 && len >= 3) {
        return HID_ROLE_MOUSE;
    }

    if (len >= 7) {
        return HID_ROLE_KEYBOARD;
    }
    if (len >= 3 && len <= 5) {
        return HID_ROLE_MOUSE;
    }
    return HID_ROLE_UNKNOWN;
}

static size_t hid_kbd_payload_offset(const uint8_t *payload, size_t len)
{
    if (len <= 7) {
        return 1;  // [mods][6 keys]
    }

    // Most boot reports use [mods][reserved][6 keys].
    if (payload[1] == 0x00) {
        return 2;
    }

    // Some devices use [mods][6 keys][pad] and keep the 2nd byte non-zero on key press.
    if (payload[2] == 0x00 && payload[1] > 0x03) {
        return 1;
    }

    return 2;
}

static void hid_copy_kbd_keys(uint8_t dst[6], const uint8_t *src, size_t src_len)
{
    size_t copy_len = (src_len > 6) ? 6 : src_len;
    memset(dst, 0, 6);
    if (copy_len > 0) {
        memcpy(dst, src, copy_len);
    }
}

static bool hid_parse_keyboard_report(const esp_hidh_event_data_t *param, bt_kbd_report_t *out)
{
    const uint8_t *d = param->input.data;
    size_t len = param->input.length;
    uint8_t report_id = (uint8_t)(param->input.report_id & 0xFF);
    const uint8_t *payload = d;
    size_t payload_len = len;
    size_t key_offset;

    memset(out, 0, sizeof(*out));
    if (!d || len < 7) {
        return false;
    }

    if (report_id != 0) {
        out->has_report_id = true;
        out->report_id = report_id;
    } else if (len >= 9) {
        // Fallback for stacks delivering report-id-prefixed payload in-band.
        out->has_report_id = true;
        out->report_id = d[0];
        payload = &d[1];
        payload_len = len - 1;
    }

    if (payload_len < 7) {
        return false;
    }

    out->modifiers = payload[0];
    key_offset = hid_kbd_payload_offset(payload, payload_len);
    if (payload_len <= key_offset) {
        return false;
    }
    hid_copy_kbd_keys(out->keys, &payload[key_offset], payload_len - key_offset);
    return true;
}

static bool hid_parse_mouse_report(const esp_hidh_event_data_t *param, bt_mouse_report_t *out)
{
    const uint8_t *d = param->input.data;
    size_t len = param->input.length;
    bool has_report_id = (param->input.report_id != 0);
    uint8_t report_id = (uint8_t)(param->input.report_id & 0xFF);
    uint8_t buttons;
    int8_t wheel = 0;

    memset(out, 0, sizeof(*out));
    if (!d || len < 3) {
        return false;
    }

    if (has_report_id) {
        // IDF HID host typically strips report-id from payload and provides it via input.report_id.
        buttons = d[0];
        if (report_id == 2 && len >= 7) {
            // Logitech M585/M590 BLE 7-byte mouse packet observed in field:
            // [btn][reserved][dx8][dy16_lo][dy16_hi][wheel][pan]
            // dy16 appears to be fixed-point with 4 fractional bits on this device.
            int16_t dy16 = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
            int16_t dy_step = (int16_t)(dy16 / 16);
            out->dx = (int8_t)d[2];
            if (dy_step > 127) {
                out->dy = 127;
            } else if (dy_step < -127) {
                out->dy = -127;
            } else {
                out->dy = (int8_t)dy_step;
            }
            wheel = (int8_t)(-((int8_t)d[5]));
            // Side buttons are usually encoded in the primary button bitmap.
            out->b4 = (buttons >> 3) & 0x1;
            out->b5 = (buttons >> 4) & 0x1;
        } else if (len >= 7) {
            // Some BLE mice use 7-byte report-id mouse payload:
            // [btn][...vendor...][dx][dy]. Keep boot layout as first choice, but
            // if boot dx/dy/wheel are all zero while tail bytes move, use tail pair.
            int8_t dx_boot = (int8_t)d[1];
            int8_t dy_boot = (int8_t)d[2];
            int8_t wheel_boot = (int8_t)d[3];
            int8_t dx_tail = (int8_t)d[5];
            int8_t dy_tail = (int8_t)d[6];
            int8_t wheel_tail = (int8_t)d[4];

            bool boot_active = (dx_boot != 0) || (dy_boot != 0) || (wheel_boot != 0);
            bool tail_active = (dx_tail != 0) || (dy_tail != 0) || (wheel_tail != 0);

            if (!boot_active && tail_active) {
                out->dx = dx_tail;
                out->dy = dy_tail;
                wheel = wheel_tail;
                out->b4 = (d[4] >> 4) & 0x1;
                out->b5 = (d[4] >> 5) & 0x1;
            } else {
                out->dx = dx_boot;
                out->dy = dy_boot;
                wheel = wheel_boot;
                out->b4 = (d[3] >> 4) & 0x1;
                out->b5 = (d[3] >> 5) & 0x1;
            }
        } else {
            out->dx = (int8_t)d[1];
            out->dy = (int8_t)d[2];
            if (len >= 4) {
                wheel = (int8_t)d[3];
                out->b4 = (d[3] >> 4) & 0x1;
                out->b5 = (d[3] >> 5) & 0x1;
            }
        }
        out->left = buttons & 0x1;
        out->right = (buttons >> 1) & 0x1;
        out->middle = (buttons >> 2) & 0x1;
        out->wheel = wheel;
        return true;
    }

    if (len == 3) {
        buttons = d[0];
        out->dx = (int8_t)d[1];
        out->dy = (int8_t)d[2];
    } else if (len == 4) {
        buttons = d[0];
        out->dx = (int8_t)d[1];
        out->dy = (int8_t)d[2];
        wheel = (int8_t)d[3];
    } else {
        // Fallback for stacks delivering in-band report-id-prefixed payloads.
        buttons = d[1];
        out->dx = (int8_t)d[2];
        out->dy = (int8_t)d[3];
        wheel = (int8_t)d[4];
        out->b4 = (d[4] >> 4) & 0x1;
        out->b5 = (d[4] >> 5) & 0x1;
    }

    out->left = buttons & 0x1;
    out->right = (buttons >> 1) & 0x1;
    out->middle = (buttons >> 2) & 0x1;
    out->wheel = wheel;
    return true;
}

static void hid_gap_passkey_cb(bool show, uint32_t passkey, void *ctx)
{
    (void)ctx;
    emit_passkey(show, passkey);
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            if (bda) {
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
                hid_on_open_success(param->open.dev);
            }
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
            if (param->open.dev) {
                const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
                if (bda) {
                    hid_clear_pending_by_bda(bda);
                }
            }
            hid_request_scan();
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        }
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        if (!bda) {
            ESP_LOGW(TAG, "INPUT event with null BDA, usage:%s MAP:%3u ID:%3u Len:%u",
                     esp_hid_usage_str(param->input.usage), (unsigned)param->input.map_index,
                     (unsigned)param->input.report_id, (unsigned)param->input.length);
#if HID_INPUT_VERBOSE_LOG
            if (param->input.data && param->input.length > 0) {
                ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
            }
#endif
            break;
        }

#if HID_INPUT_VERBOSE_LOG
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:",
                 ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage),
                 (unsigned)param->input.map_index, (unsigned)param->input.report_id,
                 (int)param->input.length);
        if (param->input.data && param->input.length > 0) {
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        }
#endif

        hid_role_t role = hid_role_from_dev(param->input.dev, param->input.usage);
        if (role == HID_ROLE_UNKNOWN) {
            role = hid_role_from_payload_shape(param);
        }

        if (role == HID_ROLE_KEYBOARD && s_callbacks.on_kbd_report) {
            bt_kbd_report_t report;
            if (hid_parse_keyboard_report(param, &report)) {
                s_callbacks.on_kbd_report(&report, s_callbacks_ctx);
            } else {
                ESP_LOGW(TAG, ESP_BD_ADDR_STR " INPUT keyboard parse failed, MAP:%3u, ID:%3u, Len:%u",
                         ESP_BD_ADDR_HEX(bda), (unsigned)param->input.map_index,
                         (unsigned)param->input.report_id, (unsigned)param->input.length);
            }
        } else if (role == HID_ROLE_MOUSE && s_callbacks.on_mouse_report) {
            bt_mouse_report_t report;
            if (hid_parse_mouse_report(param, &report)) {
                s_callbacks.on_mouse_report(&report, s_callbacks_ctx);
            } else {
                ESP_LOGW(TAG, ESP_BD_ADDR_STR " INPUT mouse parse failed, MAP:%3u, ID:%3u, Len:%u",
                         ESP_BD_ADDR_HEX(bda), (unsigned)param->input.map_index,
                         (unsigned)param->input.report_id, (unsigned)param->input.length);
            }
        } else {
            ESP_LOGW(TAG, ESP_BD_ADDR_STR " INPUT role unknown, usage:%s MAP:%3u ID:%3u Len:%u",
                     ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage),
                         (unsigned)param->input.map_index, (unsigned)param->input.report_id,
                         (unsigned)param->input.length);
        }
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        }
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
            if (s_known_kbd.valid && hid_bda_equal(bda, s_known_kbd.bda)) {
                s_kbd_connected = false;
                emit_kbd_conn(false);
            }
            if (s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda)) {
                s_mouse_connected = false;
                emit_mouse_conn(false);
                s_mouse_min_interval_locked = false;
                s_mouse_min_interval_units = 0;
                s_mouse_conn_probe_stage = 0;
                s_mouse_conn_probe_active = false;
            }
            emit_passkey(false, 0);
            hid_clear_pending_by_bda(bda);
            hid_request_scan();
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

esp_err_t bt_hid_host_init(const bt_hid_host_callbacks_t *cbs, void *ctx)
{
    if (cbs == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_callbacks, 0, sizeof(s_callbacks));
    s_callbacks = *cbs;
    s_callbacks_ctx = ctx;
    s_started = false;

    hid_known_clear(&s_known_kbd);
    hid_known_clear(&s_known_mouse);
    hid_known_clear(&s_pending_kbd);
    hid_known_clear(&s_pending_mouse);
    s_kbd_connected = false;
    s_mouse_connected = false;
    s_kbd_open_ts = 0;
    s_mouse_open_ts = 0;
    s_mouse_min_interval_locked = false;
    s_mouse_min_interval_units = 0;
    s_mouse_conn_probe_stage = 0;
    s_mouse_conn_probe_active = false;

    if (!s_hid_evt_group) {
        s_hid_evt_group = xEventGroupCreate();
        if (!s_hid_evt_group) {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

esp_err_t bt_hid_host_start(void)
{
    esp_err_t ret;

    if (!s_hid_evt_group) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_started) {
        return ESP_OK;
    }

#if CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGE(TAG, "NimBLE is enabled, but this project requires Bluedroid BLE host");
    return ESP_ERR_NOT_SUPPORTED;
#endif

#if !CONFIG_BT_BLUEDROID_ENABLED || !CONFIG_BT_BLE_ENABLED
    ESP_LOGE(TAG, "Invalid BT stack config, require BT_BLUEDROID_ENABLED=y and BT_BLE_ENABLED=y");
    return ESP_ERR_INVALID_STATE;
#endif

#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return ESP_ERR_INVALID_STATE;
#endif

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_RETURN_ON_ERROR(esp_hid_gap_init(HID_HOST_MODE), TAG, "esp_hid_gap_init failed");
    esp_hid_gap_set_passkey_callback(hid_gap_passkey_cb, NULL);
#if !CONFIG_BT_NIMBLE_ENABLED
    esp_hid_gap_set_conn_params_callback(hid_gap_conn_params_cb, NULL);
#endif
#if CONFIG_BT_BLE_ENABLED
    ESP_RETURN_ON_ERROR(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler), TAG,
                        "esp_ble_gattc_register_callback failed");
#endif /* CONFIG_BT_BLE_ENABLED */

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_RETURN_ON_ERROR(esp_hidh_init(&config), TAG, "esp_hidh_init failed");

#if !CONFIG_BT_NIMBLE_ENABLED
    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#endif

    hid_load_known_devices();
    hid_update_scan_targets();
    ESP_LOGI(TAG, "HIDH build: timeout=%d", HID_OPEN_TIMEOUT_MS);
    if (xTaskCreate(&hid_scan_task, "hid_scan_task", 6 * 1024, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "hid_scan_task create failed");
        return ESP_ERR_NO_MEM;
    }
    hid_request_scan();
    s_started = true;
    ret = ESP_OK;
    return ret;
}
