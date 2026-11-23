idf_component_register(
    SRCS "out_bacnet.c" "bacnet_port.c"
    INCLUDE_DIRS "include"
    REQUIRES out_common board bacnet esp_timer esp_driver_uart
    PRIV_REQUIRES esp_hw_support
)
