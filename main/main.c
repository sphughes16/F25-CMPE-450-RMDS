#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "power.h"
#include "esp_sleep.h"

#include "rmds_lora.h"   // LoRa task interface
#include "rmds_wifi.h"   // WiFi/cloud interface (used on RX node)

#define TAG        "RMDS_OLED"
#define TAG_UART   "UART_RX"

//  UART configuration (UART1 on GPIO 14/25) TX node
#define SENSOR_UART_NUM   UART_NUM_1
#define SENSOR_TX_PIN     GPIO_NUM_14
#define SENSOR_RX_PIN     GPIO_NUM_25
#define SENSOR_BAUD_RATE  38400
#define SENSOR_RX_BUF_SZ  2048

//  typedef struct to hold UART frame
//
// Frame layout (NORMAL mode):
//   1) 0x0000005B   (start, '[')
//   2) concentration (PPM)        - HEX on wire, we show DECIMAL
//   3) faults                     - HEX on wire, we show DECIMAL
//   4) sensor temperature (K*10)  - HEX on wire, we show Kelvin = value/10
//   5) CRC                        - HEX
//   6) CRC 1's complement         - HEX, crc ^ crc_1c == 0xFFFFFFFF
//   7) 0x0000005D   (end, ']')

typedef struct {
    uint32_t start;
    uint32_t conc_ppm;
    uint32_t faults;
    uint32_t temp_raw;   // Kelvin * 10
    uint32_t crc;
    uint32_t crc_inv;
    uint32_t end;
} sensor_frame_t;

static uint32_t parse_hex32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 16);
}

//Validates the CRC Check
static bool frame_crc_ok(const sensor_frame_t *f)
{
    return ((f->crc ^ f->crc_inv) == 0xFFFFFFFFu);
}

static bool frame_is_valid(const sensor_frame_t *f)
{
    /*if (f->start != 0x0000005B || f->end != 0x0000005D) {
        return false;
    }*/
    if (!frame_crc_ok(f)) {
        ESP_LOGW(TAG_UART,
                 "CRC mismatch: crc=0x%08" PRIx32 " inv=0x%08" PRIx32,
                 f->crc, f->crc_inv);
        return false;
    }
    return true;
}

static void dump_frame(const sensor_frame_t *f)
{
    float temp_K = f->temp_raw / 10.0f;

    ESP_LOGI(TAG_UART,
             "Frame: Conc=%" PRIu32 " ppm, Faults=%" PRIu32
             ", Temp=%.1f K, CRC=0x%08" PRIx32 ", CRC_1C=0x%08" PRIx32,
             f->conc_ppm,
             f->faults,
             temp_K,
             f->crc,
             f->crc_inv);
}

static void build_lora_payload_from_frame(const sensor_frame_t *f, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return;

    float temp_K = f->temp_raw / 10.0f;

    int n = snprintf(out, out_sz,
                     "Concentration=%" PRIu32 "ppm, "
                     "Faults=%" PRIu32 ", "
                     "Sensor Temp=%.1fK, "
                     "CRC=%08" PRIx32 ", "
                     "CRC_1C=%08" PRIx32,
                     f->conc_ppm,
                     f->faults,
                     temp_K,
                     f->crc,
                     f->crc_inv);

    if (n < 0 || (size_t)n >= out_sz) {
        ESP_LOGW(TAG_UART, "LoRa payload truncated (size=%zu)", out_sz);
    }
}
//  UART RX FreeRTOS task (TX node)
static void uart_rx_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG_UART, "UART RX task started");

    uint8_t rx_buf[128];
    char line_buf[16];          // 8 hex chars + LF + NUL
    int line_len = 0;

    uint32_t fields[7];
    int field_count = 0;

    while (1) {
        int len = uart_read_bytes(SENSOR_UART_NUM, rx_buf,
                                  sizeof(rx_buf),
                                  pdMS_TO_TICKS(1000));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            char c = (char)rx_buf[i];

            if (c == '\r') {
                continue;  // ignore CR, handle LF only
            }

            if (c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';

                    uint32_t value = parse_hex32(line_buf);

                    if (field_count < 7) {
                        fields[field_count++] = value;
                    }

                    if (field_count == 7) {
                        sensor_frame_t f = {
                            .start    = fields[0],
                            .conc_ppm = fields[1],
                            .faults   = fields[2],
                            .temp_raw = fields[3],
                            .crc      = fields[4],
                            .crc_inv  = fields[5],
                            .end      = fields[6],
                        };

                        if (frame_is_valid(&f)) {
                            dump_frame(&f);

                            char payload[RMDS_LORA_PAYLOAD_MAX_LEN];
                            build_lora_payload_from_frame(&f, payload, sizeof(payload));
                            rmds_lora_set_payload(payload);
                            ESP_LOGI(TAG_UART, "Updated LoRa payload: %s", payload);
                        } else {
                            ESP_LOGW(TAG_UART,
                                     "Invalid frame: start=0x%08" PRIx32
                                     " end=0x%08" PRIx32,
                                     f.start, f.end);
                        }

                        field_count = 0;
                    }
                }

                // reset line buffer after newline
                line_len = 0;
            } else {
                // build current line
                if (line_len < (int)sizeof(line_buf) - 1) {
                    line_buf[line_len++] = c;
                } else {
                    // overlong line, discard and resync
                    line_len = 0;
                }
            }
        }
    }
}

//  UART initialization helper (TX node)
static void init_uart_sensor(void)
{
    uart_config_t uart_config = {
        .baud_rate = SENSOR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,     // 2 stop bits
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
    
    //USE FOR TX NODE
    init_uart_sensor();
    xTaskCreate(uart_rx_task,
                "uart_rx_task",
                4096,
                NULL,
                5,
                NULL);

    //rmds_lora_start_tx_only();

    //enter_modem_sleep();
    //enter_deep_sleep(10); 

    // USE FOR RX NODE
    //
    rmds_wifi_init();          // connect to Wi-Fi, only uncomment this line if master node
    // ESP_LOGI("APP", "Starting RX-only node firmware");
    rmds_lora_start_rx_only(); // LoRa RX + cloud forwarding is in rmds_lora.c
}
