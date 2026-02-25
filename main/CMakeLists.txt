idf_component_register(
    SRCS "rmds_wifi.c" "main.c" "rmds_lora.c" "power.c"
    REQUIRES
        spi_flash
        esp_wifi
        esp_driver_i2c
        esp_http_client
        esp_lcd
        nvs_flash
        esp_driver_gpio
        esp_driver_uart
        lora
    INCLUDE_DIRS
        "."
)