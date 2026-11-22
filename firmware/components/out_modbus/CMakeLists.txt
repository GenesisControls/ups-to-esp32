idf_component_register(
    SRCS "out_modbus.c"
    INCLUDE_DIRS "include"
    REQUIRES out_common ups_hid board esp_timer esp_driver_uart
)
