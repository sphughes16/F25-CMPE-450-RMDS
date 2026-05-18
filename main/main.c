#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "power.h"
#include "rmds_network.h"
#include "rmds_wifi.h"
#include "rmds_lora.h"
#include "esp_random.h"
#include <string.h>

#define APP_TAG  "RMDS_APP"
#define TAG_UART "UART_RX"

/*
 * MANUAL NODE ROLE CONFIGURATION
 *
 * Gateway:
 *   #define RMDS_APP_NODE_ROLE    RMDS_NETWORK_ROLE_GATEWAY
 *   #define RMDS_APP_NODE_ID      0
 *   #define RMDS_APP_GATEWAY_ID   0
 *
 *  Sensor node:
 *   #define RMDS_APP_NODE_ROLE    RMDS_NETWORK_ROLE_MESH_NODE
 *   #define RMDS_APP_NODE_ID      <nonzero node id>
 *   #define RMDS_APP_GATEWAY_ID   0
 *
 * Dummy data node:
 *   Leave the node as RMDS_NETWORK_ROLE_MESH_NODE, set the desired node ID,
 *   and enable RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE below.
 *
 * To force a node to route through a specific parent, set
 * RMDS_APP_PREFERRED_PARENT_NODE_ID to that node ID.
 *
 */
#define RMDS_APP_NODE_ROLE    RMDS_NETWORK_ROLE_MESH_NODE
#define RMDS_APP_NODE_ID      1
#define RMDS_APP_GATEWAY_ID   0

/*
 * Currently, the local samples are set to dummy values if the node ID is not 1 or 2 for demoing purposes. 
 * In a real deployment, either comment this section out or change both branches 
 * to define RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE to 0
 */
#if RMDS_APP_NODE_ROLE == RMDS_NETWORK_ROLE_MESH_NODE && (RMDS_APP_NODE_ID != 1 || RMDS_APP_NODE_ID != 2)
#define RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE 1
#else
#define RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE 0
#endif

#define RMDS_APP_PREFERRED_PARENT_NODE_ID  RMDS_NETWORK_NO_PARENT_PREFERENCE //Leave this as RMDS_NETWORK_NO_PARENT_PREFERENCE unless you want to force a specific parent node ID for routing

/*
 * Dummy sample update rate.
 * This only updates the local sample in the network layer.
 * Actual LoRa send timing is still controlled in rmds_network.c.
 */
#define RMDS_APP_DUMMY_SAMPLE_INTERVAL_MS 10000

// UART configuration (UART1 on GPIO 14/25) for methane sensor nodes.
#define SENSOR_UART_NUM   UART_NUM_1
#define SENSOR_TX_PIN     GPIO_NUM_14 // Methane sensor TX pin goes to MCU pin 25
#define SENSOR_RX_PIN     GPIO_NUM_25 // Methane sensor RX pin goes to MCU pin 14
#define SENSOR_BAUD_RATE  38400
#define SENSOR_RX_BUF_SZ  2048

/* Parsed sensor frame layout produced by the methane sensor over UART.
 * Fields are parsed from hex strings into these integer slots. */
typedef struct {
    uint32_t start;
    uint32_t conc_ppm;
    uint32_t faults;
    uint32_t temp_raw;
    uint32_t crc;
    uint32_t crc_inv;
    uint32_t end;
} sensor_frame_t;

/* Static configuration used to start the network layer for this application. */
static const rmds_network_config_t s_network_config = {
    .role = RMDS_APP_NODE_ROLE,
    .node_id = RMDS_APP_NODE_ID,
    .gateway_node_id = RMDS_APP_GATEWAY_ID,
    .preferred_parent_node_id = RMDS_APP_PREFERRED_PARENT_NODE_ID,
};

/* Parses a 32-bit value from a hexadecimal NULL-terminated string. */
static uint32_t parse_hex32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 16);
}

/* Checks whether the sensor frame CRC and its inverted complement match. */
static bool frame_crc_ok(const sensor_frame_t *frame)
{
    return (frame->crc ^ frame->crc_inv) == 0xFFFFFFFFu;
}

/* Validates a parsed sensor frame (currently validates only CRC). */
static bool frame_is_valid(const sensor_frame_t *frame)
{
    if (!frame_crc_ok(frame)) {
        ESP_LOGW(TAG_UART,
                 "CRC mismatch: crc=0x%08" PRIx32 " inv=0x%08" PRIx32,
                 frame->crc,
                 frame->crc_inv);
        return false;
    }

    return true;
}

/* Logs a representation of the parsed sensor frame. */
static void dump_frame(const sensor_frame_t *frame)
{
    ESP_LOGI(TAG_UART,
             "Frame: Conc=%" PRIu32 " ppm, Faults=%" PRIu32
             ", Temp=%.1f K, CRC=0x%08" PRIx32 ", CRC_1C=0x%08" PRIx32,
             frame->conc_ppm,
             frame->faults,
             frame->temp_raw / 10.0f,
             frame->crc,
             frame->crc_inv);
}

/* Converts a parsed sensor frame into the network sample format and updates
 * the network layer's local sample store. */
static void publish_sensor_sample(const sensor_frame_t *frame)
{
    rmds_network_sensor_sample_t sample = {
        .concentration_ppm = frame->conc_ppm,
        .faults = frame->faults,
        .temp_deci_kelvin = frame->temp_raw,
        .sensor_crc = frame->crc,
        .sensor_crc_inv = frame->crc_inv,
        .valid = true,
    };

    if (!rmds_network_update_local_sample(&sample)) {
        ESP_LOGW(TAG_UART, "Failed to publish sensor sample to network layer");
        return;
    }

    ESP_LOGI(TAG_UART,
             "Updated network sample: conc=%" PRIu32 " faults=%" PRIu32 " temp=%.1f K",
             sample.concentration_ppm,
             sample.faults,
             sample.temp_deci_kelvin / 10.0f);
}

/* FreeRTOS task that reads lines from the UART, parses hex fields into a
 * sensor_frame_t, validates them, and publishes valid samples. */
static void uart_rx_task(void *pvParameters)
{
    uint8_t rx_buf[128];
    char line_buf[16];
    uint32_t fields[7];
    int line_len = 0;
    int field_count = 0;

    (void)pvParameters;

    ESP_LOGI(TAG_UART, "UART RX task started");

    while (1) {
        int len = uart_read_bytes(SENSOR_UART_NUM,
                                  rx_buf,
                                  sizeof(rx_buf),
                                  pdMS_TO_TICKS(1000));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            char c = (char)rx_buf[i];

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';

                    if (field_count < 7) {
                        fields[field_count++] = parse_hex32(line_buf);
                    }

                    if (field_count == 7) {
                        sensor_frame_t frame = {
                            .start = fields[0],
                            .conc_ppm = fields[1],
                            .faults = fields[2],
                            .temp_raw = fields[3],
                            .crc = fields[4],
                            .crc_inv = fields[5],
                            .end = fields[6],
                        };

                        if (frame_is_valid(&frame)) {
                            dump_frame(&frame);
                            publish_sensor_sample(&frame);
                        } else {
                            ESP_LOGW(TAG_UART,
                                     "Invalid frame: start=0x%08" PRIx32 " end=0x%08" PRIx32,
                                     frame.start,
                                     frame.end);
                        }

                        field_count = 0;
                    }
                }

                line_len = 0;
            } else if (line_len < (int)sizeof(line_buf) - 1) {
                line_buf[line_len++] = c;
            } else {
                line_len = 0;
            }
        }
    }
}

// Configures UART parameters and installs the UART driver used for the methane sensor. 
static void init_uart_sensor(void)
{
    uart_config_t uart_config = {
        .baud_rate = SENSOR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(SENSOR_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SENSOR_UART_NUM,
                                 SENSOR_TX_PIN,
                                 SENSOR_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(SENSOR_UART_NUM,
                                        SENSOR_RX_BUF_SZ,
                                        0,
                                        0,
                                        NULL,
                                        0));

    ESP_LOGI(TAG_UART,
             "UART%d configured: baud=%d, 8N2, TX=%d, RX=%d",
             SENSOR_UART_NUM,
             SENSOR_BAUD_RATE,
             SENSOR_TX_PIN,
             SENSOR_RX_PIN);
}

#if RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE


/* Periodically generates synthetic sensor samples for demo
 * purposes and publishes them into the network layer. */
static void dummy_sample_task(void *pvParameters)
{
    uint32_t seq = 1;

    (void)pvParameters;
    ESP_LOGI(APP_TAG, "Dummy sample task started (%d ms updates)",
             RMDS_APP_DUMMY_SAMPLE_INTERVAL_MS);

    for (;;) {
        rmds_network_sensor_sample_t sample = {
            .concentration_ppm = 400 + (esp_random() % 100),
            .faults = 0,
            .temp_deci_kelvin = 2930 + (esp_random() % 50),
            .sensor_crc = esp_random(),
            .valid = true,
        };

        sample.sensor_crc_inv = ~sample.sensor_crc;

        if (rmds_network_update_local_sample(&sample)) {
            ESP_LOGI(APP_TAG,
                     "Dummy sample updated: node=%u sample_seq=%" PRIu32 " conc=%" PRIu32,
                     (unsigned int)RMDS_APP_NODE_ID,
                     seq++,
                     sample.concentration_ppm);
        } else {
            ESP_LOGW(APP_TAG, "Failed to update dummy sample");
        }

        vTaskDelay(pdMS_TO_TICKS(RMDS_APP_DUMMY_SAMPLE_INTERVAL_MS));
    }

    vTaskDelete(NULL);
}

#endif

void app_main(void)
{
    check_wake_reason(); //Leftover from previous implementation, does nothing

    ESP_LOGI(APP_TAG,
             "Booting role=%s node=%u gateway=%u preferred_parent=%u",
             rmds_network_role_to_string(s_network_config.role),
             (unsigned int)s_network_config.node_id,
             (unsigned int)s_network_config.gateway_node_id,
             (unsigned int)s_network_config.preferred_parent_node_id);

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        ESP_LOGI(APP_TAG, "Starting always-on mesh gateway");
        rmds_wifi_init();
    } else {
#if RMDS_APP_ENABLE_DUMMY_LOCAL_SAMPLE
        ESP_LOGI(APP_TAG, "Starting dummy mesh source node");
        if (xTaskCreate(dummy_sample_task,
                        "dummy_sample_task",
                        4096,
                        NULL,
                        5,
                        NULL) != pdPASS) {
            ESP_LOGE(APP_TAG, "Failed to create dummy sample task");
            return;
        }
#else
        ESP_LOGI(APP_TAG, "Starting mesh sensor node");
        init_uart_sensor();

        if (xTaskCreate(uart_rx_task,
                        "uart_rx_task",
                        4096,
                        NULL,
                        5,
                        NULL) != pdPASS) {
            ESP_LOGE(APP_TAG, "Failed to create UART RX task");
            return;
        }
#endif
    }

    if (!rmds_network_start(&s_network_config)) {
        ESP_LOGE(APP_TAG, "Failed to start network layer");
    }
}