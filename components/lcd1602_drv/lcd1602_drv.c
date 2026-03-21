#include "lcd1602_drv.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lcd1602/lcd1602.h"

#define LCD_I2C_PORT I2C_NUM_0
#define LCD_I2C_SDA GPIO_NUM_4
#define LCD_I2C_SCL GPIO_NUM_5
#define LCD_I2C_ADDR_PRIMARY LCD1602_I2C_ADDRESS_DEFAULT
#define LCD_I2C_ADDR_FALLBACK LCD1602_I2C_ADDRESS_ALTERNATE
#define LCD_LEFT_ARROW_SLOT 1
#define LCD_CHAR_LEFT_ARROW ((char)LCD_LEFT_ARROW_SLOT)

static const char *TAG = "LCD1602_DRV";

static i2c_master_bus_handle_t s_bus;
static lcd1602_context s_ctx;
static input_router_status_t s_status;
static SemaphoreHandle_t s_lock;
static const uint8_t s_left_arrow_pattern[8] = {
    0x04,
    0x08,
    0x1F,
    0x08,
    0x04,
    0x00,
    0x00,
    0x00,
};

static void lcd_write_battery_field(char *dst, bool valid, uint8_t level)
{
    char text[4];

    if (!valid) {
        memcpy(dst, "   ", 3);
        return;
    }

    if (level > 100) {
        level = 100;
    }
    snprintf(text, sizeof(text), "%3u", (unsigned)level);
    memcpy(dst, text, 3);
}

static void lcd_write_unhandled_field(char *dst, bool is_mouse, uint16_t cmd16)
{
    char text[6];

    snprintf(text, sizeof(text), "%c%04X", is_mouse ? 'M' : 'K', (unsigned)cmd16);
    memcpy(dst, text, 5);
}

static void lcd_compose_line(char out[17], const input_router_status_t *st, uint8_t pc_idx, bool show_bt_cols)
{
    memset(out, ' ', 16);
    out[16] = 0;

    out[0] = (st->active_pc == pc_idx) ? '>' : ' ';
    out[1] = st->pc_kbd_initialized[pc_idx] ? 'S' : 's';
    out[2] = st->pc_led[pc_idx].num ? '1' : LCD_CHAR_LEFT_ARROW;
    out[3] = st->pc_led[pc_idx].caps ? 'A' : 'a';
    out[4] = st->pc_led[pc_idx].scroll ? '-' : '|';

    if (show_bt_cols) {
        out[5] = st->bt_kbd_connected ? 'K' : 'k';
        out[6] = st->bt_mouse_connected ? 'M' : 'm';

        if (st->passkey_visible) {
            char passkey[7];
            snprintf(passkey, sizeof(passkey), "%06" PRIu32, st->passkey % 1000000UL);
            memcpy(&out[7], passkey, 6);
        }
    } else {
        lcd_write_battery_field(&out[5], st->bt_kbd_battery_valid, st->bt_kbd_battery);
        lcd_write_battery_field(&out[8], st->bt_mouse_battery_valid, st->bt_mouse_battery);
    }

    // Column 12..16 (1-based): unsupported host command marker, higher priority than passkey.
    if (st->pc_last_unhandled_valid[pc_idx]) {
        lcd_write_unhandled_field(&out[11],
                                  st->pc_last_unhandled_mouse[pc_idx],
                                  st->pc_last_unhandled_cmd16[pc_idx]);
    }
}

static esp_err_t lcd_render_locked(void)
{
    char line1[17];
    char line2[17];
    input_router_status_t st;

    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    st = s_status;
    lcd_compose_line(line1, &st, 0, true);
    lcd_compose_line(line2, &st, 1, false);

    if (lcd1602_set_cursor(s_ctx, 0, 0) != 0) {
        return ESP_FAIL;
    }
    if (lcd1602_string(s_ctx, line1) != 0) {
        return ESP_FAIL;
    }
    if (lcd1602_set_cursor(s_ctx, 1, 0) != 0) {
        return ESP_FAIL;
    }
    if (lcd1602_string(s_ctx, line2) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void lcd_refresh_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_lock && xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            (void)lcd_render_locked();
            xSemaphoreGive(s_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t lcd1602_drv_init(void)
{
    i2c_lowlevel_config config = {0};
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = LCD_I2C_PORT,
        .sda_io_num = LCD_I2C_SDA,
        .scl_io_num = LCD_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (s_ctx != NULL) {
        return ESP_OK;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "lock alloc failed");
    }

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c init failed");
    config.bus = &s_bus;

    s_ctx = lcd1602_init(LCD_I2C_ADDR_PRIMARY, true, &config);
    if (s_ctx == NULL) {
        ESP_LOGW(TAG, "LCD @0x%02X init failed, trying 0x%02X", LCD_I2C_ADDR_PRIMARY, LCD_I2C_ADDR_FALLBACK);
        s_ctx = lcd1602_init(LCD_I2C_ADDR_FALLBACK, true, &config);
    }
    if (s_ctx == NULL) {
        return ESP_FAIL;
    }

    (void)lcd1602_clear(s_ctx);
    (void)lcd1602_set_display(s_ctx, true, false, false);
    if (lcd1602_set_custom_char(s_ctx, LCD_LEFT_ARROW_SLOT, s_left_arrow_pattern) != 0) {
        ESP_LOGW(TAG, "failed to set LCD custom left-arrow glyph");
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.active_pc = 0;

    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        (void)lcd_render_locked();
        xSemaphoreGive(s_lock);
    }

    if (xTaskCreate(lcd_refresh_task, "lcd_refresh", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "lcd refresh task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LCD initialized on SDA=%d SCL=%d", LCD_I2C_SDA, LCD_I2C_SCL);
    return ESP_OK;
}

esp_err_t lcd1602_drv_update(const input_router_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_lock || xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    s_status = *status;
    esp_err_t err = lcd_render_locked();
    xSemaphoreGive(s_lock);

    return err;
}
