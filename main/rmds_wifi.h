// rmds_wifi.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize Wi-Fi in STA mode and connect to the configured AP.
 * This function blocks until connected or a failure occurs.
 */
void rmds_wifi_init(void);

/**
 * Send a single LoRa payload up to the cloud backend.
 * `payload` is a null-terminated string (what you received over LoRa).
 */
void send_frame_to_cloud(const char *payload);

#ifdef __cplusplus
}
#endif
