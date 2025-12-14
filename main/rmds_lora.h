#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"

// Max length of the text payload we send over LoRa
#define RMDS_LORA_PAYLOAD_MAX_LEN 96

// Start LoRa in TX-only mode (periodic packets every 400 ms).
// Now sends the latest methane sensor payload provided via rmds_lora_set_payload().
void rmds_lora_start_tx_only(void);

// Start LoRa in RX-only mode (continuous listen).
void rmds_lora_start_rx_only(void);

// Update the text payload that the TX task will send periodically.
// Safe to call from the UART RX task.
void rmds_lora_set_payload(const char *payload);

#ifdef __cplusplus
}
#endif
