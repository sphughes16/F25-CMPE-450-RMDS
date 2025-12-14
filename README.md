 # NOTE: Must have installed ESP-IDF VSCode extension, will need to expose container port to Windows.

## Target is ESP32 -> CUSTOM_BOARD, flash method is UART, always full clean before push and after pull. Always connect antenna before power.

# LoRa Configuration (idf.py menuconfig -> Component Config -> LoRa Config)
CONFIG_CS_GPIO=18
CONFIG_RST_GPIO=23
CONFIG_MISO_GPIO=19
CONFIG_MOSI_GPIO=27
CONFIG_SCK_GPIO=5

# FOR RX NODE: Change the line in lora.c: lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03); to lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0xC3);
