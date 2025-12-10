idf_component_register(
    SRCS
        "lora.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
)

# Shim old API away on newer ESP-IDF: make gpio_pad_select_gpio(...) a no-op
target_compile_options(${COMPONENT_LIB} PRIVATE "-Dgpio_pad_select_gpio(x)=")
