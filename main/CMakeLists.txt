idf_component_register(SRCS "p2pro_uvc.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_ppa esp_timer esp_driver_i2c esp_partition esp_rom)
