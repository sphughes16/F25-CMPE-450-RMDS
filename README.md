# These are important configuration and setup steps to replicate the RMDS project. Credit goes to the creators of the external esp32-lora-library: https://github.com/Inteform/esp32-lora-library. Board used was a LILYGO T-Beam Meshtastic LORA32 915MHz ESP32 TTGO Development Board WiFi BLE CH9102F Chip Soldered OLED Module: https://www.amazon.com/LILYGO-Meshtastic-Development-CH9102F-Soldered/dp/B0B63FV7FR?th=1

### Node role configuration is manual, check main.c for details.

### Must have installed ESP-IDF VSCode extension, will need to attach Windows port to container/WSL port (usbipd attach). Further documentation can be found at: https://github.com/espressif/vscode-esp-idf-extension/blob/master/README.md

### Target is ESP32 -> CUSTOM_BOARD, flash method is UART, always full clean before push and after pull. Always connect antenna before power.

#### LoRa Configuration (idf.py menuconfig -> Component Config -> LoRa Config)
CONFIG_CS_GPIO=18
CONFIG_RST_GPIO=23
CONFIG_MISO_GPIO=19
CONFIG_MOSI_GPIO=27
CONFIG_SCK_GPIO=5

### FOR RX NODE: Change the line in components/lora/lora.c: lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03); to lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0xC3);
