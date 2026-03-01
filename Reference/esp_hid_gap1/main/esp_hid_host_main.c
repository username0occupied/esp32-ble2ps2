/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
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

static EventGroupHandle_t s_hid_evt_group;
static hid_known_device_t s_known_kbd;
static hid_known_device_t s_known_mouse;
static hid_known_device_t s_pending_kbd;
static hid_known_device_t s_pending_mouse;
static bool s_kbd_connected;
static bool s_mouse_connected;
static int64_t s_kbd_open_ts;
static int64_t s_mouse_open_ts;

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
        s_mouse_connected = true;
        s_known_mouse.valid = true;
        memcpy(s_known_mouse.bda, bda, sizeof(s_known_mouse.bda));
        s_known_mouse.transport = transport;
        s_known_mouse.addr_type = addr_type;
        s_known_mouse.usage = usage;
        s_known_mouse.role = HID_ROLE_MOUSE;
        hid_store_known_device(HID_NVS_KEY_MOUSE, &s_known_mouse);
        if (s_known_kbd.valid && hid_bda_equal(bda, s_known_kbd.bda)) {
            hid_known_clear(&s_known_kbd);
        }
    }

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

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
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
        if (bda) {
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
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
            }
            if (s_known_mouse.valid && hid_bda_equal(bda, s_known_mouse.bda)) {
                s_mouse_connected = false;
            }
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

/* scan task implemented above */

#if CONFIG_BT_NIMBLE_ENABLED
void ble_hid_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_store_config_init(void);
#endif
void app_main(void)
{
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

#if !CONFIG_BT_NIMBLE_ENABLED
    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
#endif

    s_hid_evt_group = xEventGroupCreate();
    if (!s_hid_evt_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    hid_load_known_devices();
    hid_update_scan_targets();
    ESP_LOGI(TAG, "HIDH build: timeout=%d", HID_OPEN_TIMEOUT_MS);


#if CONFIG_BT_NIMBLE_ENABLED
    /* XXX Need to have template for store */
    ble_store_config_init();

    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	/* Starting nimble task after gatts is initialized*/
    ret = esp_nimble_enable(ble_hid_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

    vTaskDelay(200);

    uint8_t own_addr_type = 0;
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, NULL, NULL);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);

    if (rc != 0) {
        ESP_LOGI(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: ");
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x \n", addr_val[5], addr_val[4], addr_val[3],
		                                      addr_val[2], addr_val[1], addr_val[0]);

#endif
    xTaskCreate(&hid_scan_task, "hid_scan_task", 6 * 1024, NULL, 2, NULL);
    hid_request_scan();
}
