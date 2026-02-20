idf_component_register(
    SRCS "rmds_wifi.c" "main.c" "rmds_lora.c" "power.c"
    PRIV_REQUIRES
        spi_flash
        esp_driver_i2c
        esp_lcd
        esp_driver_gpio
        esp_driver_uart
        lora
    INCLUDE_DIRS
        "."
)