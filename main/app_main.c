#include <stdio.h>

#include "bt_hid_host.h"
#include "esp_check.h"
#include "esp_log.h"
#include "input_router.h"
#include "lcd1602_drv.h"
#include "nvs_flash.h"
#include "ps2emu.h"
#include "ps2emu_keyboard.h"

static const char *TAG = "APP";

static esp_err_t app_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    return err;
}

static void app_status_to_lcd(const input_router_status_t *status, void *ctx)
{
    (void)ctx;
    if (status == NULL) {
        return;
    }
    esp_err_t err = lcd1602_drv_update(status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "lcd update failed: %s", esp_err_to_name(err));
    }
}

void app_main(void)
{
    const ps2emu_cfg_t ps2_cfg = {
        .pc1_kbd_clk = GPIO_NUM_19,
        .pc1_kbd_dat = GPIO_NUM_13,
        .pc1_mouse_clk = GPIO_NUM_12,
        .pc1_mouse_dat = GPIO_NUM_18,
        .pc2_kbd_clk = GPIO_NUM_2,
        .pc2_kbd_dat = GPIO_NUM_3,
        .pc2_mouse_clk = GPIO_NUM_10,
        .pc2_mouse_dat = GPIO_NUM_6,
        .half_clk_us = 40,
        .inter_byte_us = 120,
    };
    const bt_hid_host_callbacks_t bt_cbs = {
        .on_kbd_report = input_router_on_kbd_report,
        .on_mouse_report = input_router_on_mouse_report,
        .on_kbd_conn = input_router_on_kbd_conn,
        .on_mouse_conn = input_router_on_mouse_conn,
        .on_passkey = input_router_on_passkey,
    };
    input_router_status_t status = {0};
    bool lcd_ready = false;

    ESP_LOGI(TAG, "Build date/time: %s %s", __DATE__, __TIME__);
    ESP_ERROR_CHECK(app_nvs_init());
    esp_err_t lcd_err = lcd1602_drv_init();
    if (lcd_err == ESP_OK) {
        lcd_ready = true;
    } else {
        ESP_LOGW(TAG, "LCD init failed, continue without LCD: %s", esp_err_to_name(lcd_err));
    }

    ESP_ERROR_CHECK(input_router_init(lcd_ready ? app_status_to_lcd : NULL, NULL));
    ESP_ERROR_CHECK(input_router_get_status(&status));
    if (lcd_ready) {
        lcd_err = lcd1602_drv_update(&status);
        if (lcd_err != ESP_OK) {
            ESP_LOGW(TAG, "lcd initial update failed: %s", esp_err_to_name(lcd_err));
        }
    }

    ESP_ERROR_CHECK(ps2emu_init(&ps2_cfg));
    ESP_ERROR_CHECK(ps2emu_keyboard_set_led_callback(input_router_on_ps2_led, NULL));
    ESP_ERROR_CHECK(ps2emu_keyboard_set_init_callback(input_router_on_ps2_init, NULL));
    ESP_ERROR_CHECK(ps2emu_start());

    ESP_ERROR_CHECK(bt_hid_host_init(&bt_cbs, NULL));
    ESP_ERROR_CHECK(bt_hid_host_start());

    ESP_LOGI(TAG, "BLE->Dual PS/2 router started, default active target: PC1");
}
