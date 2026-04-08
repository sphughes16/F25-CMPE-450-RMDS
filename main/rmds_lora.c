#include "rmds_lora.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "lora.h"

#define LORA_TAG "RMDS_LORA"

static const rmds_lora_config_t s_default_config = {
    .frequency_hz = 915000000L,
    .bandwidth_hz = 125000L,
    .spreading_factor = 7,
    .coding_rate = 5,
    .preamble_len = 8,
    .sync_word = 0x34,
    .enable_crc = true,
};

static SemaphoreHandle_t s_lora_mutex = NULL;
static bool s_lora_initialized = false;

static bool rmds_lora_ensure_mutex(void)
{
    if (s_lora_mutex != NULL) {
        return true;
    }

    s_lora_mutex = xSemaphoreCreateMutex();
    if (s_lora_mutex == NULL) {
        ESP_LOGE(LORA_TAG, "Failed to create LoRa mutex");
        return false;
    }

    return true;
}

const rmds_lora_config_t *rmds_lora_default_config(void)
{
    return &s_default_config;
}

bool rmds_lora_init(const rmds_lora_config_t *config)
{
    const rmds_lora_config_t *effective_config =
        (config != NULL) ? config : &s_default_config;

    if (!rmds_lora_ensure_mutex()) {
        return false;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);

    if (s_lora_initialized) {
        xSemaphoreGive(s_lora_mutex);
        return true;
    }

    ESP_LOGI(LORA_TAG, "Initializing LoRa radio");
    if (!lora_init()) {
        xSemaphoreGive(s_lora_mutex);
        ESP_LOGE(LORA_TAG, "lora_init() failed");
        return false;
    }

    lora_set_frequency(effective_config->frequency_hz);
    lora_set_bandwidth(effective_config->bandwidth_hz);
    lora_set_spreading_factor(effective_config->spreading_factor);
    lora_set_coding_rate(effective_config->coding_rate);
    lora_set_preamble_length(effective_config->preamble_len);
    lora_set_sync_word(effective_config->sync_word);

    if (effective_config->enable_crc) {
        lora_enable_crc();
    } else {
        lora_disable_crc();
    }

    lora_receive();
    s_lora_initialized = true;
    xSemaphoreGive(s_lora_mutex);

    ESP_LOGI(LORA_TAG,
             "LoRa configured: freq=%ld Hz BW=%ld Hz SF=%d CR=4/%d sync=0x%02x",
             effective_config->frequency_hz,
             effective_config->bandwidth_hz,
             effective_config->spreading_factor,
             effective_config->coding_rate,
             effective_config->sync_word);
    return true;
}

bool rmds_lora_send(const void *data, size_t len)
{
    if (!s_lora_initialized || data == NULL || len == 0 || len > RMDS_LORA_MAX_PACKET_LEN) {
        return false;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    lora_send_packet((uint8_t *)data, (int)len);
    lora_receive();
    xSemaphoreGive(s_lora_mutex);
    return true;
}

int rmds_lora_receive(uint8_t *buf, size_t buf_len)
{
    int len = 0;

    if (!s_lora_initialized || buf == NULL || buf_len == 0) {
        return 0;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    len = lora_receive_packet(buf, (int)buf_len);
    if (len > 0) {
        lora_receive();
    }
    xSemaphoreGive(s_lora_mutex);

    return len;
}

void rmds_lora_start_listening(void)
{
    if (!s_lora_initialized) {
        return;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    lora_receive();
    xSemaphoreGive(s_lora_mutex);
}

void rmds_lora_sleep_radio(void)
{
    if (!s_lora_initialized) {
        return;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    lora_sleep();
    xSemaphoreGive(s_lora_mutex);
}

bool rmds_lora_is_initialized(void)
{
    return s_lora_initialized;
}

int rmds_lora_last_packet_rssi(void)
{
    int rssi = 0;

    if (!s_lora_initialized) {
        return 0;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    rssi = lora_packet_rssi();
    xSemaphoreGive(s_lora_mutex);
    return rssi;
}

float rmds_lora_last_packet_snr(void)
{
    float snr = 0.0f;

    if (!s_lora_initialized) {
        return 0.0f;
    }

    xSemaphoreTake(s_lora_mutex, portMAX_DELAY);
    snr = lora_packet_snr();
    xSemaphoreGive(s_lora_mutex);
    return snr;
}
