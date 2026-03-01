idf_component_register(
    SRCS "input_router.c"
    INCLUDE_DIRS "."
    REQUIRES ps2emu bt_hid_host
)
