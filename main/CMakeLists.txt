idf_component_register(SRCS "p2pro_uvc.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_ppa esp_driver_jpeg esp_timer esp_driver_i2c esp_partition esp_rom)
