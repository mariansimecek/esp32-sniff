idf_component_register(SRCS "sniff_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver mqtt esp_wifi nvs_flash bme280 esp_driver_i2c
                       )
