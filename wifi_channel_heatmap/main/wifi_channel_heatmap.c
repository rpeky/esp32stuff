#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ssd1306.h"

#define TAG             "HEATMAP"

/* ===== OLED wiring ===== */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDR       0x3C
#define OLED_SDA_PIN    21
#define OLED_SCL_PIN    22

/* ===== Timing ===== */
#define RESCAN_MS       8000   // rescan interval
#define FRAME_MS        200    // refresh rate (draw)

/* ===== Layout (5x7 font -> 6x8 cell) ===== */
#define CELL_W          6
#define CELL_H          8
#define MAX_COLS        (OLED_WIDTH / CELL_W)   // 21
#define MAX_ROWS        (OLED_HEIGHT / CELL_H)  // 8

#define HEADER_ROWS     2                        // 16 px header
#define LABEL_ROWS      1                        // 8 px labels at bottom
#define BAR_AREA_Y0     (HEADER_ROWS * CELL_H)   // 16
#define BAR_AREA_Y1     (OLED_HEIGHT - LABEL_ROWS * CELL_H) // 56
#define BAR_H_MAX       (BAR_AREA_Y1 - BAR_AREA_Y0)         // 40

/* Bars & X spacing */
#define CH_FIRST        1
#define CH_LAST         13
#define NCH             (CH_LAST - CH_FIRST + 1)
#define BAR_W           5     // thinner bars
#define BAR_G           2     // gaps between bars

/* Y-axis margin on the left (line + tick labels space) */
#define Y_AXIS_W        10

/* ===== 1bpp framebuffer ===== */
static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8];

/* ===== 5x7 font (ASCII 32..126), columns w/ LSB=top ===== */
static const uint8_t font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x62,0x51,0x49,0x49,0x46},{0x22,0x49,0x49,0x49,0x36},{0x18,0x14,0x12,0x7F,0x10},
    {0x2F,0x49,0x49,0x49,0x31},{0x3E,0x49,0x49,0x49,0x36},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x26,0x49,0x49,0x49,0x3E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x59,0x09,0x06},{0x3E,0x41,0x5D,0x55,0x1E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x10,0x28,0x44,0x00},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x03,0x05,0x00,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x14,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08},
};

/* ===== fb helpers ===== */
static inline void fb_clear(void) { memset(fb, 0, sizeof(fb)); }
static inline void fb_set_px(int x, int y, int on) {
    if ((unsigned)x >= OLED_WIDTH || (unsigned)y >= OLED_HEIGHT) return;
    size_t idx = (y >> 3) * OLED_WIDTH + x;
    uint8_t bit = 1u << (y & 7);
    if (on) fb[idx] |= bit; else fb[idx] &= (uint8_t)~bit;
}
static void fb_fill_rect(int x, int y, int w, int h, int on) {
    if (w <= 0 || h <= 0) return;
    for (int yy = y; yy < y + h; ++yy)
        for (int xx = x; xx < x + w; ++xx)
            fb_set_px(xx, yy, on);
}
static void draw_y_axis(void) {
    /* vertical axis line */
    int x = Y_AXIS_W - 1;
    if (x < 0) x = 0;
    fb_fill_rect(x, BAR_AREA_Y0, 1, BAR_H_MAX, 1);

    /* ticks at 0%,25%,50%,75%,100% */
    const int tick_len = 3;        // tick length to the right
    const int percents[5] = {0,25,50,75,100};
    for (int i = 0; i < 5; ++i) {
        int y = BAR_AREA_Y1 - (BAR_H_MAX * percents[i]) / 100;
        if (y < BAR_AREA_Y0) y = BAR_AREA_Y0;
        if (y > BAR_AREA_Y1) y = BAR_AREA_Y1;
        fb_fill_rect(x+1, y, tick_len, 1, 1);
    }
}
/* compute left X for bars so the group is centered in the area right of Y axis */
static int bars_x0(void) {
    int avail = OLED_WIDTH - Y_AXIS_W;
    int total = NCH*BAR_W + (NCH-1)*BAR_G;
    int x0 = Y_AXIS_W + (avail - total)/2;
    if (x0 < Y_AXIS_W) x0 = Y_AXIS_W;
    return x0;
}
static void fb_draw_char(int x, int y, char c) {
    if ((unsigned char)c < 32 || (unsigned char)c > 126) c = '?';
    const uint8_t *col = font5x7[(uint8_t)c - 32];
    for (int dx = 0; dx < 5; ++dx) {
        uint8_t bits = col[dx];
        for (int dy = 0; dy < 7; ++dy) {
            fb_set_px(x + dx, y + dy, (bits >> dy) & 1);
        }
    }
}
static void fb_draw_text_fit(int col, int row, const char *s, int max_cols) {
    if (row < 0 || row >= MAX_ROWS) return;
    int x = col * CELL_W, y = row * CELL_H;
    for (int i = 0; i < max_cols && s[i]; ++i) {
        fb_draw_char(x, y, s[i]); x += CELL_W;
        if (x + 5 > OLED_WIDTH) break;
    }
}
static void oled_flush_full(esp_lcd_panel_handle_t panel) {
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
}

/* ===== OLED bring-up ===== */
static i2c_master_bus_handle_t i2c_bus;
static esp_lcd_panel_handle_t panel;
static void oled_init(void) {
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    const esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_ADDR,
        .scl_speed_hz = 100000,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &io));

    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = { .height = OLED_HEIGHT };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1,
        .vendor_config = &ssd1306_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    /* 180° rotate */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, true));
}

/* ===== Wi-Fi scan -> histogram ===== */
static uint16_t hist[NCH];  // counts per channel 1..13
static uint16_t ap_count = 0;

static void do_scan(void) {
    memset(hist, 0, sizeof(hist));
    ap_count = 0;

    wifi_scan_config_t cfg = {
        .ssid = 0, .bssid = 0, .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 120, .max = 300 }, // ms/channel
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, true));

    uint16_t n = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&n));
    ap_count = n;

    wifi_ap_record_t *recs = (wifi_ap_record_t *)calloc(n ? n : 1, sizeof(*recs));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&n, recs));

    for (uint16_t i = 0; i < n; i++) {
        int ch = recs[i].primary;
        if (ch >= CH_FIRST && ch <= CH_LAST) {
            hist[ch - CH_FIRST]++;
        }
    }
    free(recs);

    ESP_LOGI(TAG, "APs: %u", (unsigned)n);
    for (int ch = CH_FIRST; ch <= CH_LAST; ++ch) {
        ESP_LOGI(TAG, "ch%-2d: %u", ch, (unsigned)hist[ch - CH_FIRST]);
    }
}

/* ===== drawing ===== */
static void draw_header(int ms_to_next) {
    char line1[32];
    char line2[40];                 // larger buffer => no trunc warn
    int secs = ms_to_next / 1000;   // compute once
    if (secs < 0) secs = 0;         // just in case

    snprintf(line1, sizeof(line1), "WiFi Channel Heatmap");
    snprintf(line2, sizeof(line2), "APs:%u next:%ds",
             (unsigned)ap_count, secs);

    fb_draw_text_fit(0, 0, line1, MAX_COLS);  // still clamps to 21 chars on OLED
    fb_draw_text_fit(0, 1, line2, MAX_COLS);
}

static void draw_labels(void) {
    int x0 = bars_x0();

    /* baseline under bars */
    fb_fill_rect(Y_AXIS_W - 1, BAR_AREA_Y1, OLED_WIDTH - (Y_AXIS_W - 1), 1, 1);

    /* draw a few channel numbers roughly under their centers */
    const int label_row = MAX_ROWS - 1; // last 8 px row
    int indices[] = {0, 3, 6, 9, 12};   // show 1,4,7,10,13
    for (int k = 0; k < 5; ++k) {
        int i = indices[k];
        if (i < 0 || i >= NCH) continue;
        int cx = x0 + i*(BAR_W + BAR_G) + BAR_W/2;  // bar center
        int col = (cx - 3) / CELL_W;                // approx center - 1 char
        if (col < 0) col = 0;

        char buf[4];
        snprintf(buf, sizeof(buf), "%d", CH_FIRST + i);
        fb_draw_text_fit(col, label_row, buf, MAX_COLS - col);
    }
}

static void draw_bars(void) {
    uint16_t maxv = 1;
    for (int i = 0; i < NCH; ++i) if (hist[i] > maxv) maxv = hist[i];

    int x0 = bars_x0();
    for (int i = 0; i < NCH; ++i) {
        int x = x0 + i * (BAR_W + BAR_G);
        if (x >= OLED_WIDTH) break;

        int h = (int)((uint32_t)hist[i] * BAR_H_MAX / maxv);
        if (h < 1 && hist[i] > 0) h = 1;

        int y = BAR_AREA_Y1 - h;
        fb_fill_rect(x, y, BAR_W, h, 1);
    }
}

void app_main(void) {
    /* Wi-Fi bring-up (station mode, not connecting) */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* OLED */
    oled_init();

    /* initial scan */
    do_scan();
    int64_t last_scan_us = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();
        int elapsed_ms = (int)((now - last_scan_us) / 1000);
        if (elapsed_ms >= RESCAN_MS) {
            do_scan();
            last_scan_us = now;
            elapsed_ms = 0;
        }
        int ms_to_next = RESCAN_MS - elapsed_ms;

        fb_clear();
        draw_header(ms_to_next);
        draw_y_axis();      
        draw_bars();
        draw_labels();
        oled_flush_full(panel);

        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

