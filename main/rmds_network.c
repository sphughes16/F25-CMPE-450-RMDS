#include "rmds_network.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "power.h"
#include "rmds_lora.h"
#include "rmds_wifi.h"

#define NETWORK_TAG "RMDS_NET"

#define RMDS_NETWORK_PROTOCOL_VERSION      2U
#define RMDS_NETWORK_BROADCAST_NODE        0xFFU
#define RMDS_NETWORK_TASK_STACK            8192
#define RMDS_NETWORK_POLL_MS               10

#define RMDS_NETWORK_MAX_SEEN_PACKETS      128
#define RMDS_NETWORK_MAX_HOPS              8
#define RMDS_NETWORK_BEACON_INTERVAL_MS    2000
#define RMDS_NETWORK_ROUTE_STALE_MS        60000
#define RMDS_NETWORK_FORWARD_DELAY_MS      50
#define RMDS_NETWORK_SEND_INTERVAL_MS      10000
#define RMDS_NETWORK_FORWARD_QUEUE_LEN     32

typedef enum {
    RMDS_NETWORK_PKT_BEACON = 1,
    RMDS_NETWORK_PKT_DATA   = 2,
    RMDS_NETWORK_PKT_ACK    = 3,
} rmds_network_packet_type_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t type;
    uint8_t origin_node_id;
    uint8_t sender_node_id;
    uint8_t dest_node_id;
    uint8_t gateway_node_id;
    uint8_t hop_count;
    uint8_t max_hops;
    uint32_t sequence;
} rmds_network_header_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
    uint8_t route_cost;
} rmds_network_beacon_packet_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
    uint32_t concentration_ppm;
    uint32_t faults;
    uint32_t temp_deci_kelvin;
    uint32_t sensor_crc;
    uint32_t sensor_crc_inv;
} rmds_network_data_packet_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
} rmds_network_ack_packet_t;

typedef struct {
    bool valid;
    uint8_t parent_node_id;
    uint8_t route_cost;
    int rssi_dbm;
    int64_t last_update_ms;
} rmds_network_route_t;

typedef struct {
    bool used;
    uint8_t origin_node_id;
    uint32_t sequence;
    int64_t seen_at_ms;
} rmds_network_seen_packet_t;

_Static_assert(sizeof(rmds_network_beacon_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "beacon packet too large");
_Static_assert(sizeof(rmds_network_data_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "data packet too large");
_Static_assert(sizeof(rmds_network_ack_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "ack packet too large");

static SemaphoreHandle_t s_sample_mutex = NULL;
static rmds_network_sensor_sample_t s_latest_sample;
static bool s_network_started = false;
static rmds_network_config_t s_network_config;

static rmds_network_route_t s_route;
static rmds_network_seen_packet_t s_seen[RMDS_NETWORK_MAX_SEEN_PACKETS];

static bool s_prefer_local_tx = true;
static bool s_local_send_due = false;

static rmds_network_data_packet_t s_forward_queue[RMDS_NETWORK_FORWARD_QUEUE_LEN];
static size_t s_forward_queue_head = 0;
static size_t s_forward_queue_tail = 0;
static size_t s_forward_queue_count = 0;

RTC_DATA_ATTR static uint32_t s_next_sequence = 1;

static bool rmds_network_ensure_sample_mutex(void)
{
    if (s_sample_mutex != NULL) {
        return true;
    }

    s_sample_mutex = xSemaphoreCreateMutex();
    if (s_sample_mutex == NULL) {
        ESP_LOGE(NETWORK_TAG, "Failed to create sample mutex");
        return false;
    }

    memset(&s_latest_sample, 0, sizeof(s_latest_sample));
    return true;
}

static int64_t rmds_network_now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool rmds_network_copy_latest_sample(rmds_network_sensor_sample_t *sample)
{
    if (sample == NULL || !rmds_network_ensure_sample_mutex()) {
        return false;
    }

    xSemaphoreTake(s_sample_mutex, portMAX_DELAY);
    *sample = s_latest_sample;
    xSemaphoreGive(s_sample_mutex);
    return sample->valid;
}

static void rmds_network_fill_header(rmds_network_header_t *header,
                                     rmds_network_packet_type_t type,
                                     uint8_t origin_node_id,
                                     uint8_t sender_node_id,
                                     uint8_t dest_node_id,
                                     uint8_t gateway_node_id,
                                     uint8_t hop_count,
                                     uint8_t max_hops,
                                     uint32_t sequence)
{
    header->version = RMDS_NETWORK_PROTOCOL_VERSION;
    header->type = (uint8_t)type;
    header->origin_node_id = origin_node_id;
    header->sender_node_id = sender_node_id;
    header->dest_node_id = dest_node_id;
    header->gateway_node_id = gateway_node_id;
    header->hop_count = hop_count;
    header->max_hops = max_hops;
    header->sequence = sequence;
}

static bool rmds_network_try_get_header(const uint8_t *buf,
                                        size_t len,
                                        rmds_network_header_t *header)
{
    if (buf == NULL || header == NULL || len < sizeof(rmds_network_header_t)) {
        return false;
    }

    memcpy(header, buf, sizeof(*header));
    if (header->version != RMDS_NETWORK_PROTOCOL_VERSION) {
        return false;
    }

    return true;
}

static bool rmds_network_wait_for_packet(uint8_t *buf,
                                         size_t buf_len,
                                         int64_t deadline_ms,
                                         int *packet_len)
{
    while (rmds_network_now_ms() < deadline_ms) {
        int len = rmds_lora_receive(buf, buf_len);
        if (len > 0) {
            if (packet_len != NULL) {
                *packet_len = len;
            }
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(RMDS_NETWORK_POLL_MS));
    }

    if (packet_len != NULL) {
        *packet_len = 0;
    }
    return false;
}

static void rmds_network_seen_expire_old(void)
{
    int64_t now_ms = rmds_network_now_ms();
    size_t i;

    for (i = 0; i < RMDS_NETWORK_MAX_SEEN_PACKETS; ++i) {
        if (s_seen[i].used && (now_ms - s_seen[i].seen_at_ms) > RMDS_NETWORK_ROUTE_STALE_MS) {
            s_seen[i].used = false;
        }
    }
}

static bool rmds_network_seen_contains(uint8_t origin_node_id, uint32_t sequence)
{
    size_t i;

    for (i = 0; i < RMDS_NETWORK_MAX_SEEN_PACKETS; ++i) {
        if (s_seen[i].used &&
            s_seen[i].origin_node_id == origin_node_id &&
            s_seen[i].sequence == sequence) {
            return true;
        }
    }

    return false;
}

static void rmds_network_seen_add(uint8_t origin_node_id, uint32_t sequence)
{
    size_t i;
    size_t replace_index = 0;
    int64_t oldest_ms = INT64_MAX;

    for (i = 0; i < RMDS_NETWORK_MAX_SEEN_PACKETS; ++i) {
        if (!s_seen[i].used) {
            s_seen[i].used = true;
            s_seen[i].origin_node_id = origin_node_id;
            s_seen[i].sequence = sequence;
            s_seen[i].seen_at_ms = rmds_network_now_ms();
            return;
        }

        if (s_seen[i].seen_at_ms < oldest_ms) {
            oldest_ms = s_seen[i].seen_at_ms;
            replace_index = i;
        }
    }

    s_seen[replace_index].used = true;
    s_seen[replace_index].origin_node_id = origin_node_id;
    s_seen[replace_index].sequence = sequence;
    s_seen[replace_index].seen_at_ms = rmds_network_now_ms();
}

static bool rmds_network_have_fresh_route(void)
{
    if (!s_route.valid) {
        return false;
    }

    return (rmds_network_now_ms() - s_route.last_update_ms) <= RMDS_NETWORK_ROUTE_STALE_MS;
}

static void rmds_network_maybe_update_route(uint8_t parent_node_id,
                                            uint8_t route_cost,
                                            int rssi_dbm)
{
    bool better = false;
    bool same_parent = false;

    if (parent_node_id == s_network_config.node_id) {
        return;
    }

    if (s_network_config.preferred_parent_node_id != RMDS_NETWORK_NO_PARENT_PREFERENCE &&
        parent_node_id != s_network_config.preferred_parent_node_id) {
        return;
    }

    if (s_route.valid && parent_node_id == s_route.parent_node_id) {
        same_parent = true;
    }

    if (!s_route.valid) {
        better = true;
    } else if (route_cost < s_route.route_cost) {
        better = true;
    } else if (route_cost == s_route.route_cost && rssi_dbm > s_route.rssi_dbm) {
        better = true;
    }

    if (better || same_parent) {
        bool changed = (!s_route.valid) ||
                       (s_route.parent_node_id != parent_node_id) ||
                       (s_route.route_cost != route_cost) ||
                       (s_route.rssi_dbm != rssi_dbm);

        s_route.valid = true;
        s_route.parent_node_id = parent_node_id;
        s_route.route_cost = route_cost;
        s_route.rssi_dbm = rssi_dbm;
        s_route.last_update_ms = rmds_network_now_ms();

        if (changed) {
            ESP_LOGI(NETWORK_TAG,
                     "Route updated: parent=%u cost=%u rssi=%d",
                     (unsigned int)s_route.parent_node_id,
                     (unsigned int)s_route.route_cost,
                     s_route.rssi_dbm);
        }
    }
}

static bool rmds_network_send_beacon(void)
{
    rmds_network_beacon_packet_t pkt;

    rmds_network_fill_header(&pkt.header,
                             RMDS_NETWORK_PKT_BEACON,
                             s_network_config.node_id,
                             s_network_config.node_id,
                             RMDS_NETWORK_BROADCAST_NODE,
                             s_network_config.gateway_node_id,
                             0,
                             RMDS_NETWORK_MAX_HOPS,
                             0);

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        pkt.route_cost = 0;
    } else if (rmds_network_have_fresh_route()) {
        pkt.route_cost = (uint8_t)(s_route.route_cost);
    } else {
        return false;
    }

    return rmds_lora_send(&pkt, sizeof(pkt));
}

static bool rmds_network_send_ack(uint8_t origin_node_id,
                                  uint8_t dest_node_id,
                                  uint32_t sequence)
{
    rmds_network_ack_packet_t pkt;

    rmds_network_fill_header(&pkt.header,
                             RMDS_NETWORK_PKT_ACK,
                             origin_node_id,
                             s_network_config.node_id,
                             dest_node_id,
                             s_network_config.gateway_node_id,
                             0,
                             RMDS_NETWORK_MAX_HOPS,
                             sequence);

    return rmds_lora_send(&pkt, sizeof(pkt));
}

static bool rmds_network_send_local_data(void)
{
    rmds_network_sensor_sample_t sample;
    rmds_network_data_packet_t pkt;

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        return false;
    }

    if (!rmds_network_copy_latest_sample(&sample)) {
        return false;
    }

    rmds_network_fill_header(&pkt.header,
                             RMDS_NETWORK_PKT_DATA,
                             s_network_config.node_id,
                             s_network_config.node_id,
                             RMDS_NETWORK_BROADCAST_NODE,
                             s_network_config.gateway_node_id,
                             0,
                             RMDS_NETWORK_MAX_HOPS,
                             s_next_sequence++);

    pkt.concentration_ppm = sample.concentration_ppm;
    pkt.faults = sample.faults;
    pkt.temp_deci_kelvin = sample.temp_deci_kelvin;
    pkt.sensor_crc = sample.sensor_crc;
    pkt.sensor_crc_inv = sample.sensor_crc_inv;

    /* Mark our own packet as seen so we do not forward it back if we hear it later. */
    rmds_network_seen_expire_old();
    rmds_network_seen_add(pkt.header.origin_node_id, pkt.header.sequence);

    ESP_LOGI(NETWORK_TAG,
             "LOCAL BROADCAST node=%u seq=%" PRIu32 " conc=%" PRIu32,
             (unsigned int)s_network_config.node_id,
             pkt.header.sequence,
             pkt.concentration_ppm);

    return rmds_lora_send(&pkt, sizeof(pkt));
}

static bool rmds_network_forward_data(const rmds_network_data_packet_t *incoming)
{
    rmds_network_data_packet_t pkt;

    if (incoming == NULL) {
        return false;
    }

    if (incoming->header.hop_count >= incoming->header.max_hops) {
        ESP_LOGW(NETWORK_TAG,
                 "Hop limit reached for origin=%u seq=%" PRIu32,
                 (unsigned int)incoming->header.origin_node_id,
                 incoming->header.sequence);
        return false;
    }

    pkt = *incoming;
    pkt.header.sender_node_id = s_network_config.node_id;
    pkt.header.dest_node_id = RMDS_NETWORK_BROADCAST_NODE;
    pkt.header.hop_count++;

    /* Random delay keeps all nodes from rebroadcasting at exactly the same time. */
    vTaskDelay(pdMS_TO_TICKS(RMDS_NETWORK_FORWARD_DELAY_MS + (esp_random() % 150U)));

    ESP_LOGI(NETWORK_TAG,
             "FLOOD FORWARD origin=%u seq=%" PRIu32 " sender=%u hop=%u",
             (unsigned int)pkt.header.origin_node_id,
             pkt.header.sequence,
             (unsigned int)pkt.header.sender_node_id,
             (unsigned int)pkt.header.hop_count);

    return rmds_lora_send(&pkt, sizeof(pkt));
}

static bool rmds_network_forward_queue_push(const rmds_network_data_packet_t *pkt)
{
    if (pkt == NULL) {
        return false;
    }

    if (s_forward_queue_count >= RMDS_NETWORK_FORWARD_QUEUE_LEN) {
        ESP_LOGW(NETWORK_TAG,
                 "Forward queue full; dropping origin=%u seq=%" PRIu32,
                 (unsigned int)pkt->header.origin_node_id,
                 pkt->header.sequence);
        return false;
    }

    s_forward_queue[s_forward_queue_tail] = *pkt;
    s_forward_queue_tail = (s_forward_queue_tail + 1U) % RMDS_NETWORK_FORWARD_QUEUE_LEN;
    s_forward_queue_count++;
    return true;
}

static bool rmds_network_forward_queue_peek(rmds_network_data_packet_t *pkt)
{
    if (pkt == NULL || s_forward_queue_count == 0) {
        return false;
    }

    *pkt = s_forward_queue[s_forward_queue_head];
    return true;
}

static void rmds_network_forward_queue_pop(void)
{
    if (s_forward_queue_count == 0) {
        return;
    }

    s_forward_queue_head = (s_forward_queue_head + 1U) % RMDS_NETWORK_FORWARD_QUEUE_LEN;
    s_forward_queue_count--;
}

static void rmds_network_gateway_consume_data(const rmds_network_data_packet_t *pkt,
                                              int rssi_dbm,
                                              float snr_db)
{
    rmds_wifi_cloud_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.node_id = pkt->header.origin_node_id;
    frame.network_seq = pkt->header.sequence;
    frame.concentration_ppm = pkt->concentration_ppm;
    frame.faults = pkt->faults;
    frame.temperature_k = pkt->temp_deci_kelvin / 10.0f;
    frame.sensor_crc = pkt->sensor_crc;
    frame.sensor_crc_inv = pkt->sensor_crc_inv;
    frame.rssi_dbm = rssi_dbm;
    frame.snr_db = snr_db;

    ESP_LOGI(NETWORK_TAG,
             "Gateway received routed data origin=%u seq=%" PRIu32 " sender=%u conc=%" PRIu32,
             (unsigned int)pkt->header.origin_node_id,
             pkt->header.sequence,
             (unsigned int)pkt->header.sender_node_id,
             pkt->concentration_ppm);

    if (!rmds_wifi_enqueue_frame(&frame)) {
        ESP_LOGW(NETWORK_TAG,
                 "Failed to enqueue cloud frame origin=%u seq=%" PRIu32,
                 (unsigned int)pkt->header.origin_node_id,
                 pkt->header.sequence);
    }
}

static void rmds_network_handle_beacon(const rmds_network_beacon_packet_t *pkt)
{
    uint8_t advertised_cost;

    if (pkt == NULL) {
        return;
    }

    if (pkt->header.origin_node_id == s_network_config.node_id) {
        return;
    }

    if (pkt->header.gateway_node_id != s_network_config.gateway_node_id) {
        return;
    }

    if (pkt->route_cost >= RMDS_NETWORK_MAX_HOPS) {
        return;
    }

    if (s_network_config.preferred_parent_node_id != RMDS_NETWORK_NO_PARENT_PREFERENCE &&
        pkt->header.sender_node_id != s_network_config.preferred_parent_node_id) {
        return;
    }

    ESP_LOGI(NETWORK_TAG,
             "Node %u got beacon from %u cost=%u",
             (unsigned int)s_network_config.node_id,
             (unsigned int)pkt->header.sender_node_id,
             (unsigned int)pkt->route_cost);

    advertised_cost = (uint8_t)(pkt->route_cost + 1U);
    rmds_network_maybe_update_route(pkt->header.sender_node_id,
                                    advertised_cost,
                                    rmds_lora_last_packet_rssi());
}

static void rmds_network_handle_data(const rmds_network_data_packet_t *pkt)
{
    bool duplicate;

    if (pkt == NULL) {
        return;
    }

    if (pkt->header.gateway_node_id != s_network_config.gateway_node_id) {
        return;
    }

    if (pkt->header.origin_node_id == s_network_config.node_id) {
        return;
    }

    rmds_network_seen_expire_old();
    duplicate = rmds_network_seen_contains(pkt->header.origin_node_id,
                                           pkt->header.sequence);
    if (duplicate) {
        ESP_LOGI(NETWORK_TAG,
                 "Duplicate dropped origin=%u seq=%" PRIu32,
                 (unsigned int)pkt->header.origin_node_id,
                 pkt->header.sequence);
        rmds_lora_start_listening();
        return;
    }

    /* First time this node has seen this origin+sequence. */
    rmds_network_seen_add(pkt->header.origin_node_id,
                          pkt->header.sequence);

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        rmds_network_gateway_consume_data(pkt,
                                          rmds_lora_last_packet_rssi(),
                                          rmds_lora_last_packet_snr());
        rmds_lora_start_listening();
        return;
    }

    if (pkt->header.hop_count >= pkt->header.max_hops) {
        ESP_LOGW(NETWORK_TAG,
                 "Not forwarding, max hops reached origin=%u seq=%" PRIu32,
                 (unsigned int)pkt->header.origin_node_id,
                 pkt->header.sequence);
        rmds_lora_start_listening();
        return;
    }

    if (!rmds_network_forward_queue_push(pkt)) {
        ESP_LOGW(NETWORK_TAG,
                 "Forward queue enqueue failed for origin=%u seq=%" PRIu32,
                 (unsigned int)pkt->header.origin_node_id,
                 pkt->header.sequence);
    }

    rmds_lora_start_listening();
}

static void rmds_network_handle_ack(const rmds_network_ack_packet_t *pkt)
{
    if (pkt == NULL) {
        return;
    }

    if (pkt->header.dest_node_id != s_network_config.node_id) {
        return;
    }

    ESP_LOGI(NETWORK_TAG,
             "ACK received for origin=%u seq=%" PRIu32,
             (unsigned int)pkt->header.origin_node_id,
             pkt->header.sequence);
}

static void rmds_network_process_incoming_once(int64_t deadline_ms)
{
    uint8_t rx_buf[RMDS_LORA_MAX_PACKET_LEN];
    int len = 0;
    rmds_network_header_t header;

    if (!rmds_network_wait_for_packet(rx_buf, sizeof(rx_buf), deadline_ms, &len)) {
        return;
    }

    if (!rmds_network_try_get_header(rx_buf, (size_t)len, &header)) {
        return;
    }

    switch ((rmds_network_packet_type_t)header.type) {
    case RMDS_NETWORK_PKT_BEACON:
        if (len == (int)sizeof(rmds_network_beacon_packet_t)) {
            rmds_network_beacon_packet_t pkt;
            memcpy(&pkt, rx_buf, sizeof(pkt));
            rmds_network_handle_beacon(&pkt);
        }
        break;

    case RMDS_NETWORK_PKT_DATA:
        if (len == (int)sizeof(rmds_network_data_packet_t)) {
            rmds_network_data_packet_t pkt;
            memcpy(&pkt, rx_buf, sizeof(pkt));
            rmds_network_handle_data(&pkt);
        }
        break;

    case RMDS_NETWORK_PKT_ACK:
        if (len == (int)sizeof(rmds_network_ack_packet_t)) {
            rmds_network_ack_packet_t pkt;
            memcpy(&pkt, rx_buf, sizeof(pkt));
            rmds_network_handle_ack(&pkt);
        }
        break;

    default:
        break;
    }
}

static void rmds_network_service_tx(void)
{
    bool local_available;
    bool forward_available;
    rmds_network_data_packet_t forward_pkt;

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        return;
    }

    local_available = s_local_send_due;
    forward_available = rmds_network_forward_queue_peek(&forward_pkt);

    if (!local_available && !forward_available) {
        return;
    }

    if (s_prefer_local_tx) {
        if (local_available && rmds_network_send_local_data()) {
            s_local_send_due = false;
            s_prefer_local_tx = false;
            rmds_lora_start_listening();
            return;
        }

        if (forward_available && rmds_network_forward_data(&forward_pkt)) {
            rmds_network_forward_queue_pop();
            s_prefer_local_tx = true;
            rmds_lora_start_listening();
            return;
        }
    } else {
        if (forward_available && rmds_network_forward_data(&forward_pkt)) {
            rmds_network_forward_queue_pop();
            s_prefer_local_tx = true;
            rmds_lora_start_listening();
            return;
        }

        if (local_available && rmds_network_send_local_data()) {
            s_local_send_due = false;
            s_prefer_local_tx = false;
            rmds_lora_start_listening();
            return;
        }
    }
}

static void rmds_network_mesh_task(void *pvParameters)
{
    int64_t next_beacon_ms;
    int64_t next_send_ms;

    (void)pvParameters;

    if (!rmds_lora_init(rmds_lora_default_config())) {
        ESP_LOGE(NETWORK_TAG, "Failed to initialize LoRa");
        vTaskDelete(NULL);
        return;
    }

    memset(&s_route, 0, sizeof(s_route));
    memset(s_seen, 0, sizeof(s_seen));
    memset(s_forward_queue, 0, sizeof(s_forward_queue));
    s_forward_queue_head = 0;
    s_forward_queue_tail = 0;
    s_forward_queue_count = 0;
    s_prefer_local_tx = true;
    s_local_send_due = false;

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        s_route.valid = true;
        s_route.parent_node_id = s_network_config.node_id;
        s_route.route_cost = 0;
        s_route.rssi_dbm = 0;
        s_route.last_update_ms = rmds_network_now_ms();
    }

    next_beacon_ms = rmds_network_now_ms() + 500;
    next_send_ms = rmds_network_now_ms() + RMDS_NETWORK_SEND_INTERVAL_MS + (esp_random() % 500U);

    rmds_lora_start_listening();

    while (1) {
        int64_t now_ms = rmds_network_now_ms();

        if (now_ms >= next_beacon_ms) {
            if (rmds_network_send_beacon()) {
                ESP_LOGI(NETWORK_TAG,
                         "Beacon sent role=%s node=%u",
                         rmds_network_role_to_string(s_network_config.role),
                         (unsigned int)s_network_config.node_id);
            }
            rmds_lora_start_listening();
            next_beacon_ms = now_ms + RMDS_NETWORK_BEACON_INTERVAL_MS;
        }

        if (s_network_config.role == RMDS_NETWORK_ROLE_MESH_NODE &&
            now_ms >= next_send_ms) {
            s_local_send_due = true;
            s_prefer_local_tx = true;
            next_send_ms = now_ms + RMDS_NETWORK_SEND_INTERVAL_MS + (esp_random() % 500U);
        }

        rmds_network_service_tx();
        rmds_network_process_incoming_once(now_ms + 100);
    }
}

bool rmds_network_start(const rmds_network_config_t *config)
{
    if (config == NULL || s_network_started) {
        return false;
    }

    if (config->role != RMDS_NETWORK_ROLE_GATEWAY &&
        config->role != RMDS_NETWORK_ROLE_MESH_NODE) {
        return false;
    }

    if (!rmds_network_ensure_sample_mutex()) {
        return false;
    }

    s_network_config = *config;

    if (xTaskCreate(rmds_network_mesh_task,
                    "rmds_network_task",
                    RMDS_NETWORK_TASK_STACK,
                    NULL,
                    5,
                    NULL) != pdPASS) {
        ESP_LOGE(NETWORK_TAG, "Failed to create network task");
        return false;
    }

    s_network_started = true;
    ESP_LOGI(NETWORK_TAG,
             "Started mesh role=%s node=%u gateway=%u preferred_parent=%u",
             rmds_network_role_to_string(config->role),
             (unsigned int)config->node_id,
             (unsigned int)config->gateway_node_id,
             (unsigned int)config->preferred_parent_node_id);
    return true;
}

bool rmds_network_update_local_sample(const rmds_network_sensor_sample_t *sample)
{
    if (sample == NULL || !rmds_network_ensure_sample_mutex()) {
        return false;
    }

    xSemaphoreTake(s_sample_mutex, portMAX_DELAY);
    s_latest_sample = *sample;
    xSemaphoreGive(s_sample_mutex);
    return true;
}

const char *rmds_network_role_to_string(rmds_network_role_t role)
{
    switch (role) {
    case RMDS_NETWORK_ROLE_GATEWAY:
        return "gateway";
    case RMDS_NETWORK_ROLE_MESH_NODE:
        return "mesh_node";
    default:
        return "unknown";
    }
}