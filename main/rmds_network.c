#include "rmds_network.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

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

#define RMDS_NETWORK_PROTOCOL_VERSION 1U
#define RMDS_NETWORK_BROADCAST_NODE   0xFFU
#define RMDS_NETWORK_TASK_STACK       8192
#define RMDS_NETWORK_MAX_TRACKED_RX   16
#define RMDS_NETWORK_POLL_MS          10
#define RMDS_NETWORK_GUARD_MS         50
#define RMDS_NETWORK_ACK_SPACING_MS   75
#define RMDS_NETWORK_ACK_TURN_MS      40

typedef enum {
    RMDS_NETWORK_PKT_SYNC = 1,
    RMDS_NETWORK_PKT_DATA = 2,
    RMDS_NETWORK_PKT_ACK = 3,
} rmds_network_packet_type_t;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t type;
    uint8_t src_node_id;
    uint8_t dest_node_id;
    uint32_t cycle_id;
} rmds_network_header_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
    uint16_t ms_until_primary_window;
    uint16_t ms_until_ack_window;
    uint16_t ms_until_retry_window;
    uint16_t ms_until_sleep_prep;
    uint16_t cycle_period_ms;
} rmds_network_sync_packet_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
    uint32_t sequence;
    uint32_t concentration_ppm;
    uint32_t faults;
    uint32_t temp_deci_kelvin;
    uint32_t sensor_crc;
    uint32_t sensor_crc_inv;
} rmds_network_data_packet_t;

typedef struct __attribute__((packed)) {
    rmds_network_header_t header;
    uint32_t sequence;
} rmds_network_ack_packet_t;

typedef struct {
    int64_t primary_open_ms;
    int64_t ack_open_ms;
    int64_t retry_open_ms;
    int64_t sleep_prep_ms;
    uint32_t cycle_id;
    bool synced;
} rmds_network_sensor_schedule_t;

typedef struct {
    bool used;
    bool ack_sent;
    uint8_t node_id;
    uint32_t sequence;
    rmds_wifi_cloud_frame_t cloud_frame;
} rmds_network_rx_record_t;

_Static_assert(sizeof(rmds_network_sync_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "sync packet too large");
_Static_assert(sizeof(rmds_network_data_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "data packet too large");
_Static_assert(sizeof(rmds_network_ack_packet_t) <= RMDS_LORA_MAX_PACKET_LEN, "ack packet too large");

static SemaphoreHandle_t s_sample_mutex = NULL;
static rmds_network_sensor_sample_t s_latest_sample;
static bool s_network_started = false;
static rmds_network_config_t s_network_config;

RTC_DATA_ATTR static uint32_t s_next_sequence = 1;
RTC_DATA_ATTR static uint32_t s_fallback_cycle_id = 1;

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

static void rmds_network_delay_until_ms(int64_t deadline_ms)
{
    int64_t now_ms = rmds_network_now_ms();

    if (deadline_ms <= now_ms) {
        return;
    }

    vTaskDelay(pdMS_TO_TICKS((uint32_t)(deadline_ms - now_ms)));
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
                                     uint8_t src_node_id,
                                     uint8_t dest_node_id,
                                     uint32_t cycle_id)
{
    header->version = RMDS_NETWORK_PROTOCOL_VERSION;
    header->type = (uint8_t)type;
    header->src_node_id = src_node_id;
    header->dest_node_id = dest_node_id;
    header->cycle_id = cycle_id;
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

static bool rmds_network_wait_for_ack(uint8_t node_id,
                                      uint32_t sequence,
                                      int64_t deadline_ms)
{
    uint8_t rx_buf[RMDS_LORA_MAX_PACKET_LEN];
    int len = 0;

    while (rmds_network_wait_for_packet(rx_buf, sizeof(rx_buf), deadline_ms, &len)) {
        rmds_network_header_t header;
        rmds_network_ack_packet_t ack_packet;

        if (!rmds_network_try_get_header(rx_buf, (size_t)len, &header)) {
            continue;
        }

        if (header.type != RMDS_NETWORK_PKT_ACK || len != (int)sizeof(ack_packet)) {
            continue;
        }

        memcpy(&ack_packet, rx_buf, sizeof(ack_packet));
        if (ack_packet.header.dest_node_id == node_id && ack_packet.sequence == sequence) {
            ESP_LOGI(NETWORK_TAG,
                     "Received ACK for node=%u seq=%" PRIu32 " cycle=%" PRIu32,
                     (unsigned int)node_id,
                     sequence,
                     ack_packet.header.cycle_id);
            return true;
        }
    }

    return false;
}

static bool rmds_network_send_sync_packet(int64_t cycle_start_ms)
{
    rmds_network_sync_packet_t sync_packet;
    uint32_t elapsed_ms;
    uint32_t active_total_ms;
    uint32_t cycle_period_ms;

    elapsed_ms = (uint32_t)(rmds_network_now_ms() - cycle_start_ms);
    active_total_ms = s_network_config.wake_sync_ms +
                      s_network_config.primary_tx_window_ms +
                      s_network_config.ack_window_ms +
                      s_network_config.retry_window_ms +
                      s_network_config.sleep_prep_ms;
    cycle_period_ms = active_total_ms + (s_network_config.sleep_duration_s * 1000U);

    rmds_network_fill_header(&sync_packet.header,
                             RMDS_NETWORK_PKT_SYNC,
                             s_network_config.node_id,
                             RMDS_NETWORK_BROADCAST_NODE,
                             s_fallback_cycle_id);
    sync_packet.ms_until_primary_window =
        (elapsed_ms >= s_network_config.wake_sync_ms)
            ? 0
            : (uint16_t)(s_network_config.wake_sync_ms - elapsed_ms);
    sync_packet.ms_until_ack_window =
        (elapsed_ms >= (s_network_config.wake_sync_ms + s_network_config.primary_tx_window_ms))
            ? 0
            : (uint16_t)((s_network_config.wake_sync_ms +
                          s_network_config.primary_tx_window_ms) - elapsed_ms);
    sync_packet.ms_until_retry_window =
        (elapsed_ms >= (s_network_config.wake_sync_ms +
                        s_network_config.primary_tx_window_ms +
                        s_network_config.ack_window_ms))
            ? 0
            : (uint16_t)((s_network_config.wake_sync_ms +
                          s_network_config.primary_tx_window_ms +
                          s_network_config.ack_window_ms) - elapsed_ms);
    sync_packet.ms_until_sleep_prep =
        (elapsed_ms >= active_total_ms)
            ? 0
            : (uint16_t)(active_total_ms - elapsed_ms);
    sync_packet.cycle_period_ms = (uint16_t)cycle_period_ms;

    return rmds_lora_send(&sync_packet, sizeof(sync_packet));
}

static bool rmds_network_send_data_packet(const rmds_network_sensor_sample_t *sample,
                                          uint32_t cycle_id,
                                          uint32_t sequence)
{
    rmds_network_data_packet_t data_packet;

    if (sample == NULL || !sample->valid) {
        return false;
    }

    rmds_network_fill_header(&data_packet.header,
                             RMDS_NETWORK_PKT_DATA,
                             s_network_config.node_id,
                             s_network_config.master_node_id,
                             cycle_id);
    data_packet.sequence = sequence;
    data_packet.concentration_ppm = sample->concentration_ppm;
    data_packet.faults = sample->faults;
    data_packet.temp_deci_kelvin = sample->temp_deci_kelvin;
    data_packet.sensor_crc = sample->sensor_crc;
    data_packet.sensor_crc_inv = sample->sensor_crc_inv;

    return rmds_lora_send(&data_packet, sizeof(data_packet));
}

static bool rmds_network_send_ack_packet(uint8_t dest_node_id, uint32_t cycle_id, uint32_t sequence)
{
    rmds_network_ack_packet_t ack_packet;

    rmds_network_fill_header(&ack_packet.header,
                             RMDS_NETWORK_PKT_ACK,
                             s_network_config.node_id,
                             dest_node_id,
                             cycle_id);
    ack_packet.sequence = sequence;

    return rmds_lora_send(&ack_packet, sizeof(ack_packet));
}

static bool rmds_network_receive_sync_schedule(rmds_network_sensor_schedule_t *schedule)
{
    uint8_t rx_buf[RMDS_LORA_MAX_PACKET_LEN];
    int len = 0;
    int64_t sync_deadline_ms;

    if (schedule == NULL) {
        return false;
    }

    memset(schedule, 0, sizeof(*schedule));
    sync_deadline_ms = rmds_network_now_ms() + s_network_config.wake_sync_ms;

    while (rmds_network_wait_for_packet(rx_buf, sizeof(rx_buf), sync_deadline_ms, &len)) {
        rmds_network_header_t header;
        rmds_network_sync_packet_t sync_packet;
        int64_t packet_rx_ms = rmds_network_now_ms();

        if (!rmds_network_try_get_header(rx_buf, (size_t)len, &header)) {
            continue;
        }

        if (header.type != RMDS_NETWORK_PKT_SYNC || len != (int)sizeof(sync_packet)) {
            continue;
        }

        memcpy(&sync_packet, rx_buf, sizeof(sync_packet));
        if (sync_packet.header.src_node_id != s_network_config.master_node_id) {
            continue;
        }

        schedule->primary_open_ms = packet_rx_ms + sync_packet.ms_until_primary_window;
        schedule->ack_open_ms = packet_rx_ms + sync_packet.ms_until_ack_window;
        schedule->retry_open_ms = packet_rx_ms + sync_packet.ms_until_retry_window;
        schedule->sleep_prep_ms = packet_rx_ms + sync_packet.ms_until_sleep_prep;
        schedule->cycle_id = sync_packet.header.cycle_id;
        schedule->synced = true;
        return true;
    }

    schedule->primary_open_ms = sync_deadline_ms;
    schedule->ack_open_ms = schedule->primary_open_ms + s_network_config.primary_tx_window_ms;
    schedule->retry_open_ms = schedule->ack_open_ms + s_network_config.ack_window_ms;
    schedule->sleep_prep_ms = schedule->retry_open_ms + s_network_config.retry_window_ms;
    schedule->cycle_id = s_fallback_cycle_id++;
    schedule->synced = false;
    return false;
}

static rmds_network_rx_record_t *rmds_network_find_record(rmds_network_rx_record_t *records,
                                                          size_t count,
                                                          uint8_t node_id,
                                                          uint32_t sequence)
{
    size_t i;

    for (i = 0; i < count; ++i) {
        if (records[i].used && records[i].node_id == node_id && records[i].sequence == sequence) {
            return &records[i];
        }
    }

    return NULL;
}

static rmds_network_rx_record_t *rmds_network_alloc_record(rmds_network_rx_record_t *records,
                                                           size_t count)
{
    size_t i;

    for (i = 0; i < count; ++i) {
        if (!records[i].used) {
            memset(&records[i], 0, sizeof(records[i]));
            records[i].used = true;
            return &records[i];
        }
    }

    return NULL;
}

static void rmds_network_record_rx_packet(rmds_network_rx_record_t *records,
                                          const rmds_network_data_packet_t *packet,
                                          int rssi_dbm,
                                          float snr_db)
{
    rmds_network_rx_record_t *record;

    record = rmds_network_find_record(records,
                                      RMDS_NETWORK_MAX_TRACKED_RX,
                                      packet->header.src_node_id,
                                      packet->sequence);
    if (record != NULL) {
        return;
    }

    record = rmds_network_alloc_record(records, RMDS_NETWORK_MAX_TRACKED_RX);
    if (record == NULL) {
        ESP_LOGW(NETWORK_TAG,
                 "Tracking table full, dropping node=%u seq=%" PRIu32,
                 (unsigned int)packet->header.src_node_id,
                 packet->sequence);
        return;
    }

    record->node_id = packet->header.src_node_id;
    record->sequence = packet->sequence;
    record->cloud_frame.node_id = packet->header.src_node_id;
    record->cloud_frame.network_seq = packet->sequence;
    record->cloud_frame.concentration_ppm = packet->concentration_ppm;
    record->cloud_frame.faults = packet->faults;
    record->cloud_frame.temperature_k = packet->temp_deci_kelvin / 10.0f;
    record->cloud_frame.sensor_crc = packet->sensor_crc;
    record->cloud_frame.sensor_crc_inv = packet->sensor_crc_inv;
    record->cloud_frame.rssi_dbm = rssi_dbm;
    record->cloud_frame.snr_db = snr_db;

    if (!rmds_wifi_enqueue_frame(&record->cloud_frame)) {
        ESP_LOGW(NETWORK_TAG,
                 "Failed to queue cloud frame for node=%u seq=%" PRIu32,
                 (unsigned int)record->node_id,
                 record->sequence);
    }
}

static void rmds_network_process_master_rx_window(rmds_network_rx_record_t *records, int64_t deadline_ms)
{
    uint8_t rx_buf[RMDS_LORA_MAX_PACKET_LEN];
    int len = 0;

    while (rmds_network_wait_for_packet(rx_buf, sizeof(rx_buf), deadline_ms, &len)) {
        rmds_network_header_t header;
        rmds_network_data_packet_t data_packet;

        if (!rmds_network_try_get_header(rx_buf, (size_t)len, &header)) {
            continue;
        }

        if (header.type != RMDS_NETWORK_PKT_DATA || len != (int)sizeof(data_packet)) {
            continue;
        }

        memcpy(&data_packet, rx_buf, sizeof(data_packet));
        if (data_packet.header.dest_node_id != s_network_config.node_id) {
            continue;
        }

        ESP_LOGI(NETWORK_TAG,
                 "RX data from node=%u seq=%" PRIu32 " conc=%" PRIu32 " faults=%" PRIu32,
                 (unsigned int)data_packet.header.src_node_id,
                 data_packet.sequence,
                 data_packet.concentration_ppm,
                 data_packet.faults);

        rmds_network_record_rx_packet(records,
                                      &data_packet,
                                      rmds_lora_last_packet_rssi(),
                                      rmds_lora_last_packet_snr());
    }
}

static void rmds_network_sensor_task(void *pvParameters)
{
    rmds_network_sensor_schedule_t schedule;

    (void)pvParameters;

    if (!rmds_lora_init(rmds_lora_default_config())) {
        ESP_LOGE(NETWORK_TAG, "Failed to initialize LoRa for sensor node");
        vTaskDelete(NULL);
        return;
    }

    rmds_lora_start_listening();
    rmds_network_receive_sync_schedule(&schedule);

    if (schedule.synced) {
        ESP_LOGI(NETWORK_TAG,
                 "Sensor node synced to cycle=%" PRIu32,
                 schedule.cycle_id);
    } else {
        ESP_LOGW(NETWORK_TAG, "No sync received, falling back to local schedule");
    }

    {
        rmds_network_sensor_sample_t sample;
        uint32_t sequence;
        int64_t primary_send_deadline_ms;
        int64_t tx_time_ms;
        uint32_t jitter_window_ms;
        bool acked = false;
        bool sample_ready = false;

        primary_send_deadline_ms = schedule.ack_open_ms - RMDS_NETWORK_GUARD_MS;
        if (primary_send_deadline_ms <= schedule.primary_open_ms) {
            primary_send_deadline_ms = schedule.primary_open_ms;
        }

        jitter_window_ms = (uint32_t)(primary_send_deadline_ms - schedule.primary_open_ms);
        tx_time_ms = schedule.primary_open_ms +
                     ((jitter_window_ms == 0U) ? 0 : (esp_random() % (jitter_window_ms + 1U)));
        rmds_network_delay_until_ms(tx_time_ms);

        sample_ready = rmds_network_copy_latest_sample(&sample);
        sequence = s_next_sequence++;

        if (!sample_ready) {
            ESP_LOGW(NETWORK_TAG, "No local sensor sample available for this cycle");
        } else if (!rmds_network_send_data_packet(&sample, schedule.cycle_id, sequence)) {
            ESP_LOGE(NETWORK_TAG, "Failed to send primary uplink seq=%" PRIu32, sequence);
        } else {
            ESP_LOGI(NETWORK_TAG,
                     "Primary TX sent seq=%" PRIu32 " conc=%" PRIu32,
                     sequence,
                     sample.concentration_ppm);
        }

        if (sample_ready) {
            acked = rmds_network_wait_for_ack(s_network_config.node_id,
                                              sequence,
                                              schedule.retry_open_ms);
        }

        if (sample_ready && !acked) {
            int64_t retry_send_deadline_ms = schedule.sleep_prep_ms - RMDS_NETWORK_GUARD_MS;
            uint32_t retry_jitter_ms;

            if (retry_send_deadline_ms <= schedule.retry_open_ms) {
                retry_send_deadline_ms = schedule.retry_open_ms;
            }

            retry_jitter_ms = (uint32_t)(retry_send_deadline_ms - schedule.retry_open_ms);
            tx_time_ms = schedule.retry_open_ms +
                         ((retry_jitter_ms == 0U) ? 0 : (esp_random() % (retry_jitter_ms + 1U)));
            rmds_network_delay_until_ms(tx_time_ms);

            if (rmds_network_send_data_packet(&sample, schedule.cycle_id, sequence)) {
                ESP_LOGW(NETWORK_TAG, "Retransmitted seq=%" PRIu32, sequence);
                acked = rmds_network_wait_for_ack(s_network_config.node_id,
                                                  sequence,
                                                  schedule.sleep_prep_ms);
            }
        }

        if (!acked) {
            ESP_LOGW(NETWORK_TAG, "No ACK received for seq=%" PRIu32, sequence);
        }
    }

    rmds_network_delay_until_ms(schedule.sleep_prep_ms);
    rmds_lora_sleep_radio();
    ESP_LOGI(NETWORK_TAG,
             "Entering deep sleep for %u seconds",
             (unsigned int)s_network_config.sleep_duration_s);
    enter_deep_sleep(s_network_config.sleep_duration_s);
}

static void rmds_network_master_task(void *pvParameters)
{
    (void)pvParameters;

    if (!rmds_lora_init(rmds_lora_default_config())) {
        ESP_LOGE(NETWORK_TAG, "Failed to initialize LoRa for master node");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int64_t cycle_start_ms = rmds_network_now_ms();
        int64_t sync_end_ms = cycle_start_ms + s_network_config.wake_sync_ms;
        int64_t primary_end_ms = sync_end_ms + s_network_config.primary_tx_window_ms;
        int64_t ack_end_ms = primary_end_ms + s_network_config.ack_window_ms;
        int64_t retry_end_ms = ack_end_ms + s_network_config.retry_window_ms;
        int64_t sleep_prep_end_ms = retry_end_ms + s_network_config.sleep_prep_ms;
        rmds_network_rx_record_t records[RMDS_NETWORK_MAX_TRACKED_RX];

        memset(records, 0, sizeof(records));
        ESP_LOGI(NETWORK_TAG,
                 "Master cycle=%" PRIu32 " started",
                 s_fallback_cycle_id);

        while (rmds_network_now_ms() < sync_end_ms) {
            if (!rmds_network_send_sync_packet(cycle_start_ms)) {
                ESP_LOGW(NETWORK_TAG, "Failed to send sync packet");
            }
            vTaskDelay(pdMS_TO_TICKS(s_network_config.sync_broadcast_interval_ms));
        }

        rmds_network_process_master_rx_window(records, primary_end_ms);

        {
            size_t i;

            for (i = 0; i < RMDS_NETWORK_MAX_TRACKED_RX; ++i) {
                if (!records[i].used || records[i].ack_sent) {
                    continue;
                }

                if (rmds_network_send_ack_packet(records[i].node_id,
                                                 s_fallback_cycle_id,
                                                 records[i].sequence)) {
                    records[i].ack_sent = true;
                    ESP_LOGI(NETWORK_TAG,
                             "ACK sent to node=%u seq=%" PRIu32,
                             (unsigned int)records[i].node_id,
                             records[i].sequence);
                }
                vTaskDelay(pdMS_TO_TICKS(RMDS_NETWORK_ACK_SPACING_MS));
            }
        }

        while (rmds_network_now_ms() < retry_end_ms) {
            uint8_t rx_buf[RMDS_LORA_MAX_PACKET_LEN];
            int len = 0;

            if (!rmds_network_wait_for_packet(rx_buf, sizeof(rx_buf), retry_end_ms, &len)) {
                break;
            }

            {
                rmds_network_header_t header;
                rmds_network_data_packet_t data_packet;

                if (!rmds_network_try_get_header(rx_buf, (size_t)len, &header)) {
                    continue;
                }

                if (header.type != RMDS_NETWORK_PKT_DATA || len != (int)sizeof(data_packet)) {
                    continue;
                }

                memcpy(&data_packet, rx_buf, sizeof(data_packet));
                if (data_packet.header.dest_node_id != s_network_config.node_id) {
                    continue;
                }

                rmds_network_record_rx_packet(records,
                                              &data_packet,
                                              rmds_lora_last_packet_rssi(),
                                              rmds_lora_last_packet_snr());
                vTaskDelay(pdMS_TO_TICKS(RMDS_NETWORK_ACK_TURN_MS));
                rmds_network_send_ack_packet(data_packet.header.src_node_id,
                                             s_fallback_cycle_id,
                                             data_packet.sequence);
            }
        }

        rmds_network_delay_until_ms(sleep_prep_end_ms);
        rmds_lora_start_listening();
        ESP_LOGI(NETWORK_TAG,
                 "Master waiting %u seconds before next sync phase",
                 (unsigned int)s_network_config.sleep_duration_s);
        vTaskDelay(pdMS_TO_TICKS(s_network_config.sleep_duration_s * 1000U));
        s_fallback_cycle_id++;
    }
}

bool rmds_network_start(const rmds_network_config_t *config)
{
    TaskFunction_t task_entry = NULL;

    if (config == NULL || s_network_started) {
        return false;
    }

    if (config->role != RMDS_NETWORK_ROLE_SENSOR &&
        config->role != RMDS_NETWORK_ROLE_MASTER) {
        return false;
    }

    if (!rmds_network_ensure_sample_mutex()) {
        return false;
    }

    s_network_config = *config;
    task_entry = (config->role == RMDS_NETWORK_ROLE_MASTER)
                     ? rmds_network_master_task
                     : rmds_network_sensor_task;

    if (xTaskCreate(task_entry,
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
             "Started network role=%s node=%u master=%u",
             rmds_network_role_to_string(config->role),
             (unsigned int)config->node_id,
             (unsigned int)config->master_node_id);
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
    case RMDS_NETWORK_ROLE_SENSOR:
        return "sensor";
    case RMDS_NETWORK_ROLE_MASTER:
        return "master";
    default:
        return "unknown";
    }
}
