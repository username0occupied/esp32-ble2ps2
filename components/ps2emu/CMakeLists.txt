idf_component_register(
    SRCS
        "src/ps2emu_core.c"
        "src/ps2emu_isr.c"
        "src/ps2emu_keyboard.c"
        "src/ps2emu_mouse.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES driver esp_timer
)
