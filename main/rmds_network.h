#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Special value indicating no preferred parent, routing selects best 
parent automatically when this value is used. */
#define RMDS_NETWORK_NO_PARENT_PREFERENCE 0xFFu

/* Node roles within the mesh: either a mesh node or a gateway */
typedef enum {
    RMDS_NETWORK_ROLE_MESH_NODE = 0,
    RMDS_NETWORK_ROLE_GATEWAY   = 1,
} rmds_network_role_t;

/* A single sensor measurement and its CRC/validity data used by the
 * network layer when broadcasting or forwarding readings. */
typedef struct {
    uint32_t concentration_ppm;
    uint32_t faults;
    uint32_t temp_deci_kelvin;
    uint32_t sensor_crc;
    uint32_t sensor_crc_inv;
    bool valid;
} rmds_network_sensor_sample_t;

/* Configuration used to start the network layer. Node role/IDs and an
 * optional preferred parent for routing. */
typedef struct {
    rmds_network_role_t role;
    uint8_t node_id;
    uint8_t gateway_node_id;
    uint8_t preferred_parent_node_id;
} rmds_network_config_t;

/* Starts the mesh network task with the provided configuration. */
bool rmds_network_start(const rmds_network_config_t *config);
/* Updates the locally-stored sensor sample used when broadcasting local data. */
bool rmds_network_update_local_sample(const rmds_network_sensor_sample_t *sample);
/* Converts a role enum value to a string. */
const char *rmds_network_role_to_string(rmds_network_role_t role);

#ifdef __cplusplus
}
#endif