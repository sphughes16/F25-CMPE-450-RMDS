#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "lora.h"
#include "rmds_lora.h"

// ==========================
//  LoRa configuration
// ==========================
#define LORA_TAG               "RMDS_LORA"

#define RMDS_LORA_FREQ_HZ      915000000L   // 915 MHz (US ISM band)
#define RMDS_LORA_BW_HZ        125000L      // 125 kHz bandwidth
#define RMDS_LORA_SF           7            // spreading factor
#define RMDS_LORA_CR           5            // coding rate 4/5
#define RMDS_LORA_PREAMBLE_LEN 8
#define RMDS_LORA_SYNC_WORD    0x34

// Transmission interval: 400 ms
#define RMDS_LORA_TX_PERIOD_MS 400

// Shared payload buffer (set by UART RX task, used by TX task)
static char g_lora_payload[RMDS_LORA_PAYLOAD_MAX_LEN];
static SemaphoreHandle_t g_lora_payload_mutex = NULL;

// ==========================
//  Common init helper
// ==========================
static bool rmds_lora_common_init(const char *tag)
{
    ESP_LOGI(tag, "LoRa init: calling lora_init()");
    if (!lora_init()) {
        ESP_LOGE(tag, "lora_init() failed");
        return false;
    }

    lora_set_frequency(RMDS_LORA_FREQ_HZ);
    lora_set_bandwidth(RMDS_LORA_BW_HZ);
    lora_set_spreading_factor(RMDS_LORA_SF);
    lora_set_coding_rate(RMDS_LORA_CR);
    lora_set_preamble_length(RMDS_LORA_PREAMBLE_LEN);
    lora_set_sync_word(RMDS_LORA_SYNC_WORD);
    lora_enable_crc();

    ESP_LOGI(tag,
             "LoRa configured: freq=%ld Hz, BW=%ld Hz, SF=%d, CR=4/%d",
             (long)RMDS_LORA_FREQ_HZ,
             (long)RMDS_LORA_BW_HZ,
             RMDS_LORA_SF,
             RMDS_LORA_CR);
    return true;
}

// ==========================
//  TX-only task
// ==========================
static void rmds_lora_tx_task(void *pvParameters)
{
    (void)pvParameters;
    const char *TAG = LORA_TAG;

    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "TX task starting");

    if (!rmds_lora_common_init(TAG)) {
        ESP_LOGE(TAG, "TX task: init failed, deleting task");
        vTaskDelete(NULL);
        return;
    }

    // Create mutex for the shared payload buffer
    g_lora_payload_mutex = xSemaphoreCreateMutex();
    if (g_lora_payload_mutex == NULL) {
        ESP_LOGE(TAG, "TX task: failed to create payload mutex");
        vTaskDelete(NULL);
        return;
    }
    g_lora_payload[0] = '\0';  // no payload yet

    while (1) {
        // Copy the latest payload under mutex
        char payload[RMDS_LORA_PAYLOAD_MAX_LEN];

        if (g_lora_payload_mutex) {
            xSemaphoreTake(g_lora_payload_mutex, portMAX_DELAY);
            strncpy(payload, g_lora_payload, sizeof(payload) - 1);
            payload[sizeof(payload) - 1] = '\0';
            xSemaphoreGive(g_lora_payload_mutex);
        } else {
            payload[0] = '\0';
        }

        int len = (int)strlen(payload);
        if (len == 0) {
            ESP_LOGI(TAG, "TX: no sensor payload yet, skipping this period");
            vTaskDelay(pdMS_TO_TICKS(RMDS_LORA_TX_PERIOD_MS));
            continue;
        }

        ESP_LOGI(TAG,
                 "TX: sending sensor payload len=%d: \"%.*s\"",
                 len, len, payload);

        // This call blocks until the packet is transmitted
        lora_send_packet((uint8_t *)payload, len);
        ESP_LOGI(TAG, "TX: packet sent");

        vTaskDelay(pdMS_TO_TICKS(RMDS_LORA_TX_PERIOD_MS));
    }
}

// Public API to start TX-only behavior
void rmds_lora_start_tx_only(void)
{
    BaseType_t ok = xTaskCreate(
        rmds_lora_tx_task,
        "rmds_lora_tx_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (ok != pdPASS) {
        ESP_LOGE(LORA_TAG, "Failed to create rmds_lora_tx_task");
    }
}

// Public API called from UART RX task to update the payload
void rmds_lora_set_payload(const char *payload)
{
    if (!payload || !g_lora_payload_mutex) {
        return;
    }

    xSemaphoreTake(g_lora_payload_mutex, portMAX_DELAY);
    strncpy(g_lora_payload, payload, RMDS_LORA_PAYLOAD_MAX_LEN - 1);
    g_lora_payload[RMDS_LORA_PAYLOAD_MAX_LEN - 1] = '\0';
    xSemaphoreGive(g_lora_payload_mutex);
}

// ==========================
//  RX-only task
// ==========================
static void rmds_lora_rx_task(void *pvParameters)
{
    (void)pvParameters;
    const char *TAG = LORA_TAG;

    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "RX task starting");

    if (!rmds_lora_common_init(TAG)) {
        ESP_LOGE(TAG, "RX task: init failed, deleting task");
        vTaskDelete(NULL);
        return;
    }

    uint8_t buf[256];

    // Put radio into continuous receive mode
    ESP_LOGI(TAG, "RX: entering continuous receive mode");
    lora_receive();

    while (1) {
        int len = lora_receive_packet(buf, sizeof(buf) - 1);
        if (len > 0) {
            buf[len] = '\0';
            printf("[LoRa RX] %s\n", (char *)buf);
            ESP_LOGI(TAG, "RX: got packet len=%d payload=\"%s\"", len, buf);

            // Resume continuous receive
            lora_receive();
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // small delay to avoid hogging CPU
    }
}

// Public API to start RX-only behavior
void rmds_lora_start_rx_only(void)
{
    BaseType_t ok = xTaskCreate(
        rmds_lora_rx_task,
        "rmds_lora_rx_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (ok != pdPASS) {
        ESP_LOGE(LORA_TAG, "Failed to create rmds_lora_rx_task");
    }
}
