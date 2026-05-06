#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define RMDS_NETWORK_NO_PARENT_PREFERENCE 0xFFu

typedef enum {
    RMDS_NETWORK_ROLE_MESH_NODE = 0,
    RMDS_NETWORK_ROLE_GATEWAY   = 1,
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
    uint8_t gateway_node_id;

    /*
     * Optional next-hop restriction.
     *
     * Set this to a specific node ID to force this node to route through that
     * parent only. Set to RMDS_NETWORK_NO_PARENT_PREFERENCE to allow normal
     * best-route selection.
     */
    uint8_t preferred_parent_node_id;
} rmds_network_config_t;

bool rmds_network_start(const rmds_network_config_t *config);
bool rmds_network_update_local_sample(const rmds_network_sensor_sample_t *sample);
const char *rmds_network_role_to_string(rmds_network_role_t role);

#ifdef __cplusplus
}
#endif