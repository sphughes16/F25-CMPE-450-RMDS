#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
// (Actual on-air time of a short LoRa packet is << 400 ms; this keeps us
// under FCC dwell-time limits per channel.)
#define RMDS_LORA_TX_PERIOD_MS 400

// Simple fixed node ID for now (you can later take from Kconfig)
#define RMDS_NODE_ID           1

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

    int seq = 0;

    while (1) {
        char payload[64];
        int methane_fake_ppm = 123;  // placeholder for real sensor reading

        int len = snprintf(payload, sizeof(payload),
                           "NODE=%d,SEQ=%d,METHANE=%d",
                           RMDS_NODE_ID, seq++, methane_fake_ppm);
        if (len < 0) len = 0;
        if (len > (int)sizeof(payload)) len = sizeof(payload);

        ESP_LOGI(TAG, "TX: sending packet len=%d", len);
        ESP_LOGI(TAG, "TX: calling lora_send_packet");

        // NOTE: If this call never returns, the problem is inside the LoRa
        // library (likely DIO0 / pin config / wiring).
        lora_send_packet((uint8_t *)payload, len);

        // These prints will only happen if lora_send_packet() returns:
        printf("[LoRa TX] %.*s\n", len, payload);
        ESP_LOGI(TAG, "TX: packet sent: %.*s", len, payload);

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
        // Most esp32-lora-library examples use lora_received() or directly
        // lora_receive_packet(). Here we poll in a non-blocking loop.
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
