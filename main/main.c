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

#define APP_TAG  "RMDS_APP"
#define TAG_UART "UART_RX"

/*
 * Flash gateway with:
 *   RMDS_APP_NODE_ROLE = RMDS_NETWORK_ROLE_GATEWAY
 *   RMDS_APP_NODE_ID   = 0
 *
 * Flash mesh nodes with:
 *   RMDS_APP_NODE_ROLE = RMDS_NETWORK_ROLE_MESH_NODE
 *   RMDS_APP_NODE_ID   = unique nonzero ID
 *
 * All nodes should use the same gateway ID.
 */
#define RMDS_APP_NODE_ROLE    RMDS_NETWORK_ROLE_GATEWAY
#define RMDS_APP_NODE_ID      0
#define RMDS_APP_GATEWAY_ID   0

// UART configuration (UART1 on GPIO 14/25) for methane sensor nodes.
#define SENSOR_UART_NUM   UART_NUM_1
#define SENSOR_TX_PIN     GPIO_NUM_14
#define SENSOR_RX_PIN     GPIO_NUM_25
#define SENSOR_BAUD_RATE  38400
#define SENSOR_RX_BUF_SZ  2048

typedef struct {
    uint32_t start;
    uint32_t conc_ppm;
    uint32_t faults;
    uint32_t temp_raw;
    uint32_t crc;
    uint32_t crc_inv;
    uint32_t end;
} sensor_frame_t;

static const rmds_network_config_t s_network_config = {
    .role = RMDS_APP_NODE_ROLE,
    .node_id = RMDS_APP_NODE_ID,
    .gateway_node_id = RMDS_APP_GATEWAY_ID,
};

static uint32_t parse_hex32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 16);
}

static bool frame_crc_ok(const sensor_frame_t *frame)
{
    return (frame->crc ^ frame->crc_inv) == 0xFFFFFFFFu;
}

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

void app_main(void)
{
    check_wake_reason();

    ESP_LOGI(APP_TAG,
             "Booting role=%s node=%u gateway=%u",
             rmds_network_role_to_string(s_network_config.role),
             (unsigned int)s_network_config.node_id,
             (unsigned int)s_network_config.gateway_node_id);

    if (s_network_config.role == RMDS_NETWORK_ROLE_GATEWAY) {
        ESP_LOGI(APP_TAG, "Starting always-on mesh gateway");
        rmds_wifi_init();
    } else {
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
    }

    if (!rmds_network_start(&s_network_config)) {
        ESP_LOGE(APP_TAG, "Failed to start network layer");
    }
}
