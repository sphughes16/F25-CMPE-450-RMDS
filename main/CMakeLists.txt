idf_component_register(
    SRCS "main.c" "rmds_lora.c"
    PRIV_REQUIRES
        spi_flash
        esp_driver_i2c
        esp_lcd
        esp_driver_gpio
        lora
    INCLUDE_DIRS
        "."
)
