idf_component_register(
    SRCS "lcd1602_drv.c"
    INCLUDE_DIRS "include"
    REQUIRES lcd1602 input_router esp_driver_i2c
)
