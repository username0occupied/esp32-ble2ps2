idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bt_hid_host input_router lcd1602_drv ps2emu nvs_flash
)
