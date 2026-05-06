#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t node_id;
    uint32_t network_seq;
    uint32_t concentration_ppm;
    uint32_t faults;
    float temperature_k;
    uint32_t sensor_crc;
    uint32_t sensor_crc_inv;
    int rssi_dbm;
    float snr_db;
} rmds_wifi_cloud_frame_t;

/**
 * Initialize Wi-Fi in STA mode and connect to the configured AP.
 * This function blocks until connected or a failure occurs.
 */
void rmds_wifi_init(void);

/**
 * Queue a single received network frame for cloud upload.
 */
bool rmds_wifi_enqueue_frame(const rmds_wifi_cloud_frame_t *frame);
bool rmds_wifi_is_ready(void);

#ifdef __cplusplus
}
#endif
