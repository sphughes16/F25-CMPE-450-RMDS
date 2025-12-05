/*
 * Rocket animation on SSD1306 OLED (LILYGO T-Beam, ESP-IDF v6, new I2C API)
 *
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

#define TAG "OLED_ROCKET"

/* ---- I2C config ---- */
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TIMEOUT_MS  1000

/* ---- SSD1306 definitions ---- */
#define SSD1306_I2C_ADDR  0x3C
#define SSD1306_CMD       0x00
#define SSD1306_DATA      0x40

#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_PAGES        (OLED_HEIGHT / 8)

/* Framebuffer: 1bpp, 8 rows per page */
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

/* New I2C-bus handles */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t oled_dev = NULL;

/* -------- I2C helpers (new bus API) -------- */

static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t dev_cfg = {
        .device_address = SSD1306_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &oled_dev));

    return ESP_OK;
}

static esp_err_t ssd1306_write_command(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CMD, cmd };
    return i2c_master_transmit(
        oled_dev,
        buf, sizeof(buf),
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
}

static esp_err_t ssd1306_write_data(const uint8_t *data, size_t len)
{
    if (len > 128) len = 128;   // safety

    uint8_t buf[1 + 128];
    buf[0] = SSD1306_DATA;
    memcpy(&buf[1], data, len);

    return i2c_master_transmit(
        oled_dev,
        buf, len + 1,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
}

/* -------- SSD1306 low-level -------- */

static void ssd1306_init(void)
{
    const uint8_t init_cmds[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock divide / oscillator freq
        0xA8, 0x3F, // Multiplex ratio (1/64)
        0xD3, 0x00, // Display offset = 0
        0x40,       // Start line = 0
        0x8D, 0x14, // Charge pump ON
        0x20, 0x00, // Memory mode = horizontal
        0xA1,       // Segment remap
        0xC8,       // COM scan direction remapped
        0xDA, 0x12, // COM pins config
        0x81, 0x7F, // Contrast
        0xD9, 0xF1, // Pre-charge
        0xDB, 0x40, // VCOMH deselect level
        0xA4,       // Display from RAM
        0xA6,       // Normal (not inverted)
        0x2E,       // Deactivate scroll
        0xAF        // Display ON
    };

    for (size_t i = 0; i < sizeof(init_cmds); ++i) {
        ESP_ERROR_CHECK(ssd1306_write_command(init_cmds[i]));
    }
}

/* page = 0..7, column = 0..127 */
static void ssd1306_set_page_column(uint8_t page, uint8_t column)
{
    page &= 0x07;
    column &= 0x7F;

    ESP_ERROR_CHECK(ssd1306_write_command(0xB0 | page));            // page
    ESP_ERROR_CHECK(ssd1306_write_command(0x00 | (column & 0x0F))); // low column
    ESP_ERROR_CHECK(ssd1306_write_command(0x10 | (column >> 4)));   // high column
}

/* -------- Framebuffer helpers -------- */

static void oled_clear_buffer(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_draw_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    int page = y / 8;
    int index = page * OLED_WIDTH + x;
    uint8_t bit = 1 << (y & 7);

    if (on) {
        oled_buffer[index] |= bit;
    } else {
        oled_buffer[index] &= ~bit;
    }
}

static void oled_draw_hline(int x0, int x1, int y, bool on)
{
    if (y < 0 || y >= OLED_HEIGHT) return;
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (x0 < 0) x0 = 0;
    if (x1 >= OLED_WIDTH) x1 = OLED_WIDTH - 1;

    for (int x = x0; x <= x1; ++x) {
        oled_draw_pixel(x, y, on);
    }
}

static void oled_draw_vline(int x, int y0, int y1, bool on)
{
    if (x < 0 || x >= OLED_WIDTH) return;
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (y0 < 0) y0 = 0;
    if (y1 >= OLED_HEIGHT) y1 = OLED_HEIGHT - 1;

    for (int y = y0; y <= y1; ++y) {
        oled_draw_pixel(x, y, on);
    }
}

static void oled_flush(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; ++page) {
        ssd1306_set_page_column(page, 0);
        const uint8_t *line = &oled_buffer[page * OLED_WIDTH];
        ESP_ERROR_CHECK(ssd1306_write_data(line, OLED_WIDTH));
    }
}

/* -------- Simple drawing primitives -------- */

static void oled_draw_filled_rect(int x0, int y0, int w, int h, bool on)
{
    for (int y = y0; y < y0 + h; ++y) {
        for (int x = x0; x < x0 + w; ++x) {
            oled_draw_pixel(x, y, on);
        }
    }
}

/* Tiny 5x7 font for digits and a few letters (for HUD text) */

static const uint8_t font5x7[][5] = {
    // '0'..'9'
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
};

static void oled_draw_char5x7(int x, int y, char c, bool on)
{
    if (c < '0' || c > '9') return;
    const uint8_t *glyph = font5x7[c - '0'];
    for (int col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            if (bits & (1 << row)) {
                oled_draw_pixel(x + col, y + row, on);
            }
        }
    }
}

/* -------- Rocket drawing -------- */

/*
 * Rocket is built from simple shapes:
 *  - body: vertical rectangle
 *  - nose: small triangle-ish top
 *  - fins: small side triangles
 *  - window: small circle
 *  - flame: small flickering "tail"
 */

static void draw_rocket(int cx, int cy, bool big_flame)
{
    int body_w = 14;
    int body_h = 26;

    int x0 = cx - body_w / 2;
    int y0 = cy - body_h;

    // Body
    oled_draw_filled_rect(x0, y0, body_w, body_h, true);

    // Nose (simple tapered top)
    for (int i = 0; i < 5; ++i) {
        int line_w = body_w - 2 * i;
        if (line_w <= 0) break;
        int lx0 = cx - line_w / 2;
        int ly  = y0 - i - 1;
        oled_draw_hline(lx0, lx0 + line_w - 1, ly, true);
    }

    // Fins
    for (int i = 0; i < 5; ++i) {
        int ly = cy - 5 + i;
        oled_draw_hline(x0 - i, x0 - 1, ly, true);                 // left fin
        oled_draw_hline(x0 + body_w, x0 + body_w + i, ly, true);   // right fin
    }

    // Window (tiny circle / ring)
    int wx = cx;
    int wy = cy - body_h + 6;
    oled_draw_pixel(wx, wy, false);
    oled_draw_pixel(wx - 1, wy, false);
    oled_draw_pixel(wx + 1, wy, false);
    oled_draw_pixel(wx, wy - 1, false);
    oled_draw_pixel(wx, wy + 1, false);

    // Flame (flickering length)
    int flame_h = big_flame ? 9 : 5;
    int flame_w = 8;
    int fx0 = cx - flame_w / 2;
    int fy0 = cy + 1;

    for (int y = 0; y < flame_h; ++y) {
        int row_w = flame_w - (y / 2); // taper a bit
        int flx0 = cx - row_w / 2;
        for (int x = 0; x < row_w; ++x) {
            // Dither pattern to suggest "color"/glow
            bool on = ((x + y) & 1) == (big_flame ? 0 : 1);
            oled_draw_pixel(flx0 + x, fy0 + y, on);
        }
    }
}

/* Tiny HUD text showing a countdown till loop restart */
static void draw_hud_counter(int value_0_to_9)
{
    oled_draw_char5x7(4, 4, '0' + value_0_to_9, true);
}

/* Some static background stars */
static const struct {
    uint8_t x, y;
} stars[] = {
    {10, 10}, {25, 5}, {40, 15}, {60, 8}, {90, 12},
    {110, 4}, {15, 30}, {50, 24}, {80, 20}, {120, 28},
    {5, 50}, {35, 40}, {70, 45}, {100, 38}, {115, 52}
};

static void draw_starfield(int twinkle_phase)
{
    for (size_t i = 0; i < sizeof(stars)/sizeof(stars[0]); ++i) {
        bool on = ((i + twinkle_phase) & 1) == 0;
        oled_draw_pixel(stars[i].x, stars[i].y, on);
    }
}

/* -------- app_main: rocket animation loop -------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C bus and OLED");
    ESP_ERROR_CHECK(i2c_master_init());
    ssd1306_init();

    printf("Rocket animation starting on OLED...\n");

    while (true) {
        /*
         * One full pass of the rocket from left to right.
         * We start a bit off-screen (-20) and end a bit off-screen (WIDTH+20)
         * so the movement looks smooth. Step of 2 pixels per frame.
         * Each frame ~80 ms -> total time ~6–7 seconds.
         */
        int frame = 0;
        for (int x = -20; x < OLED_WIDTH + 20; x += 2, ++frame) {
            oled_clear_buffer();

            // Starfield with slight twinkle
            draw_starfield(frame & 0x03);

            // Ground / horizon line
            int ground_y = OLED_HEIGHT - 6;
            oled_draw_hline(0, OLED_WIDTH - 1, ground_y, true);

            // Rocket path (slight arc: small vertical sine-ish wobble)
            int center_y = ground_y - 8;
            int wobble = (frame % 16 < 8) ? (frame % 8) : (15 - (frame % 16));
            int rocket_y = center_y - wobble;

            bool big_flame = (frame & 1) == 0;
            draw_rocket(x, rocket_y, big_flame);

            // HUD countdown in top-left (0..9)
            int t = (frame / 8) % 10;
            draw_hud_counter(t);

            // Push frame to OLED
            oled_flush();

            vTaskDelay(pdMS_TO_TICKS(80)); // ~12.5 fps
        }
    }
}
