#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lora.h"
#include "rmds_lora.h"

#define LORA_TAG  "RMDS_LORA"

// LoRa radio configuration
#define RMDS_LORA_FREQ_HZ       915000000L   // 915 MHz (US)
#define RMDS_LORA_BW_HZ         125000L      // 125 kHz
#define RMDS_LORA_SF            7            // spreading factor 7
#define RMDS_LORA_CR            5            // coding rate 4/5
#define RMDS_LORA_PREAMBLE_LEN  8
#define RMDS_LORA_SYNC_WORD     0x34

// Make TX faster for debugging (1 second)
#define RMDS_LORA_TX_PERIOD_MS  1000

// Node ID (constant for now)
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

    // For debug: count RX loop iterations
    uint32_t rx_loop_counter = 0;

    ESP_LOGI(LORA_TAG, "LoRa node task starting");

    for (;;) {
        switch (state) {
        case RMDS_LORA_STATE_INIT:
            ESP_LOGI(LORA_TAG, "State=INIT: Initializing LoRa radio");

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
                     "LoRa configured: freq=%ld Hz, BW=%ld Hz, SF=%d, CR=4/%d, TX_PERIOD_MS=%d",
                     (long)RMDS_LORA_FREQ_HZ,
                     (long)RMDS_LORA_BW_HZ,
                     RMDS_LORA_SF,
                     RMDS_LORA_CR,
                     RMDS_LORA_TX_PERIOD_MS);

            last_tx_tick = xTaskGetTickCount();
            ESP_LOGI(LORA_TAG, "INIT complete, last_tx_tick=%u", (unsigned)last_tx_tick);

            state = RMDS_LORA_STATE_RX;
            break;

        case RMDS_LORA_STATE_RX: {
            rx_loop_counter++;
            if ((rx_loop_counter % 100) == 0) {
                // Log occasionally so we know RX loop is alive
                TickType_t now_dbg = xTaskGetTickCount();
                ESP_LOGI(LORA_TAG,
                         "State=RX: loop=%u, now=%u, last_tx=%u, diff_ticks=%u",
                         (unsigned)rx_loop_counter,
                         (unsigned)now_dbg,
                         (unsigned)last_tx_tick,
                         (unsigned)(now_dbg - last_tx_tick));
            }

            // Put radio in receive mode
            lora_receive();

            // Check for received packets
            while (lora_received()) {
                int len = lora_receive_packet(rx_buf, sizeof(rx_buf) - 1);

                if (len > 0) {
                    rx_buf[len] = '\0'; // null-terminate
                    printf("[LoRa RX] %s\n", (char *)rx_buf);
                    ESP_LOGI(LORA_TAG, "State=RX: got packet (%d bytes): %s", len, rx_buf);
                } else {
                    ESP_LOGW(LORA_TAG, "State=RX: lora_received() but len=%d", len);
                }

                // Go back to RX listening mode for more packets
                lora_receive();
            }

            // Time to transmit?
            TickType_t now = xTaskGetTickCount();
            TickType_t diff = now - last_tx_tick;
            TickType_t period_ticks = pdMS_TO_TICKS(RMDS_LORA_TX_PERIOD_MS);

            if (diff >= period_ticks) {
                ESP_LOGI(LORA_TAG,
                         "State=RX: switching to TX (diff_ticks=%u, period_ticks=%u)",
                         (unsigned)diff,
                         (unsigned)period_ticks);
                state = RMDS_LORA_STATE_TX;
            }

            vTaskDelay(pdMS_TO_TICKS(10));  // small delay
            break;
        }

        case RMDS_LORA_STATE_TX: {
            ESP_LOGI(LORA_TAG, "State=TX: preparing packet");

            int fake_methane_ppm = 123;

            char tx_buf[64];
            int len = snprintf(tx_buf, sizeof(tx_buf),
                               "NODE=%d,METHANE=%d",
                               RMDS_NODE_ID, fake_methane_ppm);
            if (len < 0) len = 0;
            if (len > (int)sizeof(tx_buf)) len = sizeof(tx_buf);

            ESP_LOGI(LORA_TAG, "State=TX: payload len=%d", len);

            if (len > 0) {
                ESP_LOGI(LORA_TAG, "State=TX: calling lora_send_packet");
                lora_send_packet((uint8_t *)tx_buf, len);

                // These prints should appear even if radio is dead
                printf("[LoRa TX] %.*s\n", len, tx_buf);
                ESP_LOGI(LORA_TAG, "State=TX: sent (%d bytes): %.*s", len, len, tx_buf);
            } else {
                ESP_LOGW(LORA_TAG, "State=TX: nothing to send (len=%d)", len);
            }

            last_tx_tick = xTaskGetTickCount();
            ESP_LOGI(LORA_TAG, "State=TX: done, updating last_tx_tick=%u",
                     (unsigned)last_tx_tick);

            state = RMDS_LORA_STATE_RX;
            break;
        }

        case RMDS_LORA_STATE_ERROR:
        default:
            ESP_LOGE(LORA_TAG, "State=ERROR: re-entering INIT in 1s");
            vTaskDelay(pdMS_TO_TICKS(1000));
            state = RMDS_LORA_STATE_INIT;
            break;
        }
    }
}

void rmds_lora_start(void)
{
    // Raise log level for this tag if needed
    esp_log_level_set(LORA_TAG, ESP_LOG_INFO);

    BaseType_t ok = xTaskCreate(
        rmds_lora_task,
        "rmds_lora_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (ok != pdPASS) {
        ESP_LOGE(LORA_TAG, "Failed to create rmds_lora_task");
    }
}
