#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lora.h"
#include "rmds_lora.h"

#define LORA_TAG  "RMDS_LORA"

// LoRa radio configuration (adjust as needed)
#define RMDS_LORA_FREQ_HZ       915000000L   // 915 MHz (US)
#define RMDS_LORA_BW_HZ         125000L      // 125 kHz
#define RMDS_LORA_SF            7            // spreading factor 7
#define RMDS_LORA_CR            5            // coding rate 4/5
#define RMDS_LORA_PREAMBLE_LEN  8
#define RMDS_LORA_SYNC_WORD     0x34

// Period between transmissions
#define RMDS_LORA_TX_PERIOD_MS  5000

// For now you can leave this the same on both nodes.
// Later, make it configurable to distinguish nodes.
#define RMDS_NODE_ID            1

typedef enum {
    RMDS_LORA_STATE_INIT = 0,
    RMDS_LORA_STATE_RX,
    RMDS_LORA_STATE_TX,
    RMDS_LORA_STATE_ERROR,
} rmds_lora_state_t;


static void rmds_lora_task(void *pvParameters)
{
    (void)pvParameters;

    rmds_lora_state_t state = RMDS_LORA_STATE_INIT;
    TickType_t last_tx_tick = 0;
    uint8_t rx_buf[256];

    ESP_LOGI(LORA_TAG, "LoRa node task starting");

    for (;;) {
        switch (state) {
        case RMDS_LORA_STATE_INIT:
            ESP_LOGI(LORA_TAG, "Initializing LoRa radio");

            if (!lora_init()) {
                ESP_LOGE(LORA_TAG, "lora_init() failed");
                state = RMDS_LORA_STATE_ERROR;
                break;
            }

            lora_set_frequency(RMDS_LORA_FREQ_HZ);
            lora_set_bandwidth(RMDS_LORA_BW_HZ);
            lora_set_spreading_factor(RMDS_LORA_SF);
            lora_set_coding_rate(RMDS_LORA_CR);
            lora_set_preamble_length(RMDS_LORA_PREAMBLE_LEN);
            lora_set_sync_word(RMDS_LORA_SYNC_WORD);
            lora_enable_crc();

            ESP_LOGI(LORA_TAG,
                     "LoRa configured: freq=%ld Hz, BW=%ld Hz, SF=%d, CR=4/%d",
                     (long)RMDS_LORA_FREQ_HZ,
                     (long)RMDS_LORA_BW_HZ,
                     RMDS_LORA_SF,
                     RMDS_LORA_CR);

            last_tx_tick = xTaskGetTickCount();
            state = RMDS_LORA_STATE_RX;
            break;

        case RMDS_LORA_STATE_RX: {
            // Enter continuous RX mode
            lora_receive();

            // Check for received packets
            while (lora_received()) {
                int len = lora_receive_packet(rx_buf, sizeof(rx_buf) - 1);

                if (len > 0) {
                    rx_buf[len] = '\0'; // null-terminate for printing
                    printf("[LoRa RX] %s\n", (char *)rx_buf);
                    ESP_LOGI(LORA_TAG, "RX (%d bytes): %s", len, rx_buf);
                }

                // Back to RX for the next packet
                lora_receive();
            }

            // Time to send our own packet?
            TickType_t now = xTaskGetTickCount();
            if ((now - last_tx_tick) >= pdMS_TO_TICKS(RMDS_LORA_TX_PERIOD_MS)) {
                state = RMDS_LORA_STATE_TX;
            }

            vTaskDelay(pdMS_TO_TICKS(10));  // yield briefly
            break;
        }

        case RMDS_LORA_STATE_TX: {
            // For now: fake methane reading. Later: real sensor.
            int fake_methane_ppm = 123;

            char tx_buf[64];
            int len = snprintf(tx_buf, sizeof(tx_buf),
                               "NODE=%d,METHANE=%d",
                               RMDS_NODE_ID, fake_methane_ppm);
            if (len < 0) len = 0;
            if (len > (int)sizeof(tx_buf)) len = sizeof(tx_buf);

            if (len > 0) {
                lora_send_packet((uint8_t *)tx_buf, len);

                // Print the contents of the sent packet
                printf("[LoRa TX] %.*s\n", len, tx_buf);
                ESP_LOGI(LORA_TAG, "TX (%d bytes): %.*s", len, len, tx_buf);
            } else {
                ESP_LOGW(LORA_TAG, "TX: nothing to send (len = %d)", len);
            }

            last_tx_tick = xTaskGetTickCount();
            state = RMDS_LORA_STATE_RX;
            break;
        }

        case RMDS_LORA_STATE_ERROR:
        default:
            ESP_LOGE(LORA_TAG, "LoRa in ERROR state, retrying init in 1s");
            vTaskDelay(pdMS_TO_TICKS(1000));
            state = RMDS_LORA_STATE_INIT;
            break;
        }
    }
}

void rmds_lora_start(void)
{
    BaseType_t ok = xTaskCreate(
        rmds_lora_task,
        "rmds_lora_task",
        4096,       // stack size
        NULL,
        5,          // priority
        NULL
    );

    if (ok != pdPASS) {
        ESP_LOGE(LORA_TAG, "Failed to create rmds_lora_task");
    }
}
