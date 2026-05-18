idf_component_register(
    SRCS
        "lora.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Dgpio_pad_select_gpio(x)=")
