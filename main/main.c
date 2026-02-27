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

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_io_i2c.h"

#include "power.h"
#include "esp_sleep.h"

#include "rmds_lora.h"   // LoRa task interface
#include "rmds_wifi.h"   // WiFi/cloud interface (used on RX node)

#define TAG        "RMDS_OLED"
#define TAG_UART   "UART_RX"

//  OLED + I2C configuration
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_PORT        I2C_NUM_0

#define I2C_MASTER_CLK_SOURCE  I2C_CLK_SRC_DEFAULT
#define I2C_MASTER_FREQ_HZ     400000

// SSD1306 I2C address
#define OLED_I2C_ADDR          0x3C
#define OLED_WIDTH             128
#define OLED_HEIGHT            64

// Animation timing (ms)
#define STEP_DELAY_MS          300
#define HOLD_FULL_COUNT        4
#define HOLD_FULL_DELAY_MS     400

//  UART configuration (UART1 on GPIO 14/25) TX node
#define SENSOR_UART_NUM   UART_NUM_1
#define SENSOR_TX_PIN     GPIO_NUM_14
#define SENSOR_RX_PIN     GPIO_NUM_25
#define SENSOR_BAUD_RATE  38400
#define SENSOR_RX_BUF_SZ  2048

//  Global handles / framebuffer
static i2c_master_bus_handle_t  i2c_bus_handle = NULL;
static i2c_master_dev_handle_t  i2c_dev_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle    = NULL;
static esp_lcd_panel_handle_t    panel_handle = NULL;

// 1-bpp framebuffer: 8 vertical pixels per byte
static uint8_t frame_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

//  Framebuffer helper implementations
static inline void fb_clear(void)
{
    memset(frame_buffer, 0x00, sizeof(frame_buffer));
}

static inline void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }

    // Rotate 180: physical (WIDTH-1, HEIGHT-1)
    int hw_x = (OLED_WIDTH  - 1) - x;
    int hw_y = (OLED_HEIGHT - 1) - y;

    int byte_index = (hw_y / 8) * OLED_WIDTH + hw_x;
    uint8_t bit_mask = 1 << (hw_y & 7);

    if (on) {
        frame_buffer[byte_index] |= bit_mask;
    } else {
        frame_buffer[byte_index] &= ~bit_mask;
    }
}

static void fb_fill_rect(int x0, int y0, int w, int h, bool on)
{
    for (int y = y0; y < y0 + h; y++) {
        for (int x = x0; x < x0 + w; x++) {
            fb_set_pixel(x, y, on);
        }
    }
}

// 1-pixel border around the whole screen
static void fb_draw_border(void)
{
    for (int x = 0; x < OLED_WIDTH; x++) {
        fb_set_pixel(x, 0, true);
        fb_set_pixel(x, OLED_HEIGHT - 1, true);
    }
    for (int y = 0; y < OLED_HEIGHT; y++) {
        fb_set_pixel(0, y, true);
        fb_set_pixel(OLED_WIDTH - 1, y, true);
    }
}

// Push framebuffer to the panel
static void fb_flush_to_panel(void)
{
    esp_lcd_panel_draw_bitmap(panel_handle,
                              0, 0,
                              OLED_WIDTH, OLED_HEIGHT,
                              frame_buffer);
}

//  I2C & OLED initialization
static void init_i2c_and_oled(void)
{
    // --- I2C bus config ---
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .clk_source = I2C_MASTER_CLK_SOURCE,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_handle));

    // --- Attach SSD1306 as a device ---
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_I2C_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle));

    // --- Create panel I/O over I2C ---
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr            = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
        .dc_bit_offset       = 6,
        .scl_speed_hz        = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &io_cfg, &io_handle));

    // --- Create SSD1306 panel (no color_space in v6.1) ---
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // No mirroring: draw exactly as we put it in the framebuffer
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

// Block-style R
static void draw_letter_R(int x0, int y0, int w, int h)
{
    int stroke = w / 4;
    if (stroke < 2) stroke = 2;

    int right = x0 + w - 1;
    int mid_y = y0 + h / 2;

    // Left vertical bar
    fb_fill_rect(x0, y0, stroke, h, true);

    // Top horizontal bar
    fb_fill_rect(x0, y0, w - stroke, stroke, true);

    // Middle bar
    fb_fill_rect(x0, mid_y - stroke / 2, w - stroke, stroke, true);

    // Right vertical of the upper loop
    fb_fill_rect(right - stroke + 1, y0 + stroke,
                 stroke, mid_y - y0 - stroke, true);

    // Diagonal leg
    for (int i = 0; i < h / 2; i++) {
        int y = mid_y + i;
        int x = x0 + stroke + (w - 2 * stroke) * i / (h / 2);
        fb_fill_rect(x, y, stroke, 2, true);
    }
}

// Block-style M
static void draw_letter_M(int x0, int y0, int w, int h)
{
    int stroke = w / 5;
    if (stroke < 2) stroke = 2;

    int right = x0 + w - 1;
    int mid_x = x0 + w / 2;

    // Two vertical bars
    fb_fill_rect(x0, y0, stroke, h, true);
    fb_fill_rect(right - stroke + 1, y0, stroke, h, true);

    // Diagonals to the center
    for (int i = 0; i < h / 2; i++) {
        int y = y0 + i;
        int x_left  = x0 + stroke + (mid_x - x0 - stroke) * i / (h / 2);
        int x_right = right - stroke - (right - stroke - mid_x) * i / (h / 2);

        fb_fill_rect(x_left,  y, stroke, 1, true);
        fb_fill_rect(x_right, y, stroke, 1, true);
    }
}

// Block-style D
static void draw_letter_D(int x0, int y0, int w, int h)
{
    int stroke = w / 4;
    if (stroke < 2) stroke = 2;

    int right  = x0 + w - 1;
    int bottom = y0 + h - 1;

    // Left vertical bar
    fb_fill_rect(x0, y0, stroke, h, true);
    // Top bar
    fb_fill_rect(x0, y0, w - stroke, stroke, true);
    // Bottom bar
    fb_fill_rect(x0, bottom - stroke + 1, w - stroke, stroke, true);
    // Right vertical bar
    fb_fill_rect(right - stroke + 1, y0 + stroke,
                 stroke, h - 2 * stroke, true);
}

// Block-style S
static void draw_letter_S(int x0, int y0, int w, int h)
{
    int stroke = w / 4;
    if (stroke < 2) stroke = 2;

    int right  = x0 + w - 1;
    int bottom = y0 + h - 1;
    int mid_y  = y0 + h / 2;

    // Top bar
    fb_fill_rect(x0 + stroke / 2, y0, w - stroke, stroke, true);
    // Upper left vertical
    fb_fill_rect(x0, y0, stroke, mid_y - y0, true);

    // Middle bar
    fb_fill_rect(x0 + stroke / 2, mid_y - stroke / 2,
                 w - stroke, stroke, true);

    // Lower right vertical
    fb_fill_rect(right - stroke + 1, mid_y,
                 stroke, bottom - mid_y + 1, true);
    // Bottom bar
    fb_fill_rect(x0 + stroke / 2, bottom - stroke + 1,
                 w - stroke, stroke, true);
}

static void draw_rmds_partial(int letters_to_show)
{
    fb_clear();
    fb_draw_border();

    int total_w  = 100;
    int base_x   = (OLED_WIDTH - total_w) / 2;
    int base_y   = 10;
    int letter_w = 22;
    int letter_h = 40;
    int gap      = 3;

    int x = base_x;

    if (letters_to_show >= 1) {
        draw_letter_R(x, base_y, letter_w, letter_h);
    }
    x += letter_w + gap;

    if (letters_to_show >= 2) {
        draw_letter_M(x, base_y, letter_w, letter_h);
    }
    x += letter_w + gap;

    if (letters_to_show >= 3) {
        draw_letter_D(x, base_y, letter_w, letter_h);
    }
    x += letter_w + gap;

    if (letters_to_show >= 4) {
        draw_letter_S(x, base_y, letter_w, letter_h);
    }
}

static void rmds_oled_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "RMDS OLED task started");

    while (1) {
        // Step through R -> RM -> RMD -> RMDS
        for (int letters = 1; letters <= 4; ++letters) {
            draw_rmds_partial(letters);
            fb_flush_to_panel();
            vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
        }

        // Hold / flash "RMDS" a few times
        for (int i = 0; i < HOLD_FULL_COUNT; ++i) {
            // Full RMDS ON
            draw_rmds_partial(4);
            fb_flush_to_panel();
            vTaskDelay(pdMS_TO_TICKS(HOLD_FULL_DELAY_MS));

            // Blink-off effect
            fb_clear();
            fb_draw_border();
            fb_flush_to_panel();
            vTaskDelay(pdMS_TO_TICKS(HOLD_FULL_DELAY_MS));
        }
    }
}

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

    ESP_LOGI("APP", "Starting TX-only node firmware");
    rmds_lora_start_tx_only();

    enter_modem_sleep();
    enter_deep_sleep(10); 

    // USE FOR RX NODE
    //
    // rmds_wifi_init();          // connect to Wi-Fi, only uncomment this line if master node
    // ESP_LOGI("APP", "Starting RX-only node firmware");
    // rmds_lora_start_rx_only(); // LoRa RX + cloud forwarding is in rmds_lora.c
}
