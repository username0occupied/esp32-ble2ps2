#ifndef LCD1602_DRV_H
#define LCD1602_DRV_H

#include "esp_err.h"
#include "input_router.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lcd1602_drv_init(void);
esp_err_t lcd1602_drv_update(const input_router_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
