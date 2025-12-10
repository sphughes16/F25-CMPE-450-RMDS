#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_io_i2c.h"

#include "rmds_lora.h"   // <---- NEW: LoRa task interface

#define TAG "RMDS_OLED"

// ================================
//  OLED + I2C configuration
// ================================
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_PORT        I2C_NUM_0

#define I2C_MASTER_CLK_SOURCE  I2C_CLK_SRC_DEFAULT
#define I2C_MASTER_FREQ_HZ     400000

// SSD1306 I2C address (commonly 0x3C)
#define OLED_I2C_ADDR          0x3C
#define OLED_WIDTH             128
#define OLED_HEIGHT            64

// Animation timing (ms)
#define STEP_DELAY_MS          300   // R -> RM -> RMD -> RMDS step delay
#define HOLD_FULL_COUNT        4     // how many times to flash RMDS
#define HOLD_FULL_DELAY_MS     400   // delay while holding full RMDS

// ===============================
//  Global handles / framebuffer
// ===============================
static i2c_master_bus_handle_t  i2c_bus_handle = NULL;
static i2c_master_dev_handle_t  i2c_dev_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle    = NULL;
static esp_lcd_panel_handle_t    panel_handle = NULL;

// 1-bpp framebuffer: 8 vertical pixels per byte
static uint8_t frame_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// ==================================
//  Framebuffer helper implementations
// ==================================
static inline void fb_clear(void)
{
    memset(frame_buffer, 0x00, sizeof(frame_buffer));
}

static inline void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }

    // Rotate 180°: logical (0,0) -> physical (WIDTH-1, HEIGHT-1)
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

// ===========================
//  I2C & OLED initialization
// ===========================
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
        .dev_addr           = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 8,
        .dc_bit_offset      = 6,
        .scl_speed_hz       = I2C_MASTER_FREQ_HZ,
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

// =======================
//  Simple block letters
// =======================

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

// ===============================
//  RMDS logo drawing / animation
// ===============================

// Draw 1–4 letters of "RMDS"
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

// ===========================
//  OLED animation FreeRTOS task
// ===========================
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

            // Optional blink-off effect
            fb_clear();
            fb_draw_border();
            fb_flush_to_panel();
            vTaskDelay(pdMS_TO_TICKS(HOLD_FULL_DELAY_MS));
        }
    }
}

void app_main(void)
{
    init_i2c_and_oled();      // your existing OLED init
    xTaskCreate(rmds_oled_task,    // whatever you named it
                "oled_task",
                4096,
                NULL,
                4,
                NULL);

    ESP_LOGI("APP", "Starting TX-only node firmware");
    rmds_lora_start_tx_only();

    /* UNCOMMENT FOR RX MODE
    ESP_LOGI("APP", "Starting RX-only node firmware");
    rmds_lora_start_rx_only();*/
}

