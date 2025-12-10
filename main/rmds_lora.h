// rmds_lora.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Start LoRa in TX-only mode (periodic beacons every 400 ms).
void rmds_lora_start_tx_only(void);

// Start LoRa in RX-only mode (continuous listen).
void rmds_lora_start_rx_only(void);

#ifdef __cplusplus
}
#endif
