idf_component_register(
    SRCS "out_common.c"
    INCLUDE_DIRS "include"
    REQUIRES ups_hid nvs_flash esp_timer esp_app_format esp_system
)
