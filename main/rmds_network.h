#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RMDS_NETWORK_ROLE_SENSOR = 0,
    RMDS_NETWORK_ROLE_MASTER = 1,
} rmds_network_role_t;

typedef struct {
    uint32_t concentration_ppm;
    uint32_t faults;
    uint32_t temp_deci_kelvin;
    uint32_t sensor_crc;
    uint32_t sensor_crc_inv;
    bool valid;
} rmds_network_sensor_sample_t;

typedef struct {
    rmds_network_role_t role;
    uint8_t node_id;
    uint8_t master_node_id;
    uint32_t wake_sync_ms;
    uint32_t primary_tx_window_ms;
    uint32_t ack_window_ms;
    uint32_t retry_window_ms;
    uint32_t sleep_prep_ms;
    uint32_t sleep_duration_s;
    uint32_t sync_broadcast_interval_ms;
} rmds_network_config_t;

bool rmds_network_start(const rmds_network_config_t *config);
bool rmds_network_update_local_sample(const rmds_network_sensor_sample_t *sample);
const char *rmds_network_role_to_string(rmds_network_role_t role);

#ifdef __cplusplus
}
#endif
