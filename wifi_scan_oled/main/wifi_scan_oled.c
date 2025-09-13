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

#define TAG             "WIFI+OLED"

/* ===== OLED wiring ===== */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDR       0x3C
#define OLED_SDA_PIN    21
#define OLED_SCL_PIN    22

/* ===== UI timing ===== */
#define SCROLL_MS       900     // advance one AP every step
#define RESCAN_MS       8000    // Wi-Fi rescan interval

/* ===== Text grid (5x7 font -> 6x8 cell) ===== */
#define CELL_W          6
#define CELL_H          8
#define MAX_COLS        (OLED_WIDTH / CELL_W)   // 21 chars/line
#define MAX_ROWS        (OLED_HEIGHT / CELL_H)  // 8 lines total
#define LINES_PER_AP    2                       // we use two lines per AP
#define APS_PER_SCREEN  (MAX_ROWS / LINES_PER_AP) // 4 APs visible

/* ===== 1-bpp framebuffer ===== */
static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8];

/* ===== 5x7 ASCII font (32..126), 5 columns/char, LSB=top ===== */
static const uint8_t font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x62,0x51,0x49,0x49,0x46},{0x22,0x49,0x49,0x49,0x36},{0x18,0x14,0x12,0x7F,0x10},
    {0x2F,0x49,0x49,0x49,0x31},{0x3E,0x49,0x49,0x49,0x32},{0x01,0x71,0x09,0x05,0x03},
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

typedef struct {
    char ssid[33];
    int  rssi;
    int  ch;
} ap_row_t;

/* ===== framebuffer helpers ===== */
static inline void fb_clear(void) { memset(fb, 0x00, sizeof(fb)); }

static inline void fb_set_px(int x, int y, int on) {
    if ((unsigned)x >= OLED_WIDTH || (unsigned)y >= OLED_HEIGHT) return;
    size_t idx = (y >> 3) * OLED_WIDTH + x;  // page-major
    uint8_t bit = 1u << (y & 7);
    if (on) fb[idx] |= bit; else fb[idx] &= (uint8_t)~bit;
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
    int x = col * CELL_W;
    int y = row * CELL_H;
    for (int i = 0; i < max_cols && s[i]; ++i) {
        fb_draw_char(x, y, s[i]);
        x += CELL_W;
        if (x + 5 > OLED_WIDTH) break;
    }
}

static void oled_flush_full(esp_lcd_panel_handle_t panel) {
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
}

/* ===== band helper ===== */
static const char *band_from_channel(int ch) {
    if (ch >= 1 && ch <= 14) return "2G";
    if ((ch >= 36 && ch <= 64) || (ch >= 100 && ch <= 165)) return "5G";
    if (ch >= 1 && ch <= 233) return "6G";
    return "?";
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

    /* === 180° rotate: flip both axes === */
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, true));
}

/* ===== Wi-Fi scan ===== */
static int cmp_rssi(const void *a, const void *b) {
    const ap_row_t *A = (const ap_row_t *)a, *B = (const ap_row_t *)b;
    return B->rssi - A->rssi; // strongest first
}

static ap_row_t *scan_wifi(size_t *out_n) {
    wifi_scan_config_t cfg = {
        .ssid = 0, .bssid = 0, .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 120, .max = 300 }, // ms per channel
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, true));

    uint16_t n = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&n));
    wifi_ap_record_t *recs = (wifi_ap_record_t *)calloc(n ? n : 1, sizeof(*recs));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&n, recs));

    ap_row_t *rows = (ap_row_t *)calloc(n ? n : 1, sizeof(*rows));
    for (uint16_t i = 0; i < n; i++) {
        snprintf(rows[i].ssid, sizeof(rows[i].ssid), "%s",
                 recs[i].ssid[0] ? (char *)recs[i].ssid : "");
        rows[i].rssi = recs[i].rssi;
        rows[i].ch   = recs[i].primary;
    }
    free(recs);

    qsort(rows, n, sizeof(ap_row_t), cmp_rssi);
    *out_n = n;

    /* Serial dump once per scan */
    ESP_LOGI(TAG, "Scan results (%u):", (unsigned)n);
    for (uint16_t i = 0; i < n; i++) {
        const char *band = band_from_channel(rows[i].ch);
        ESP_LOGI(TAG, "%4d dBm ch%-3d (%s)  %s",
                 rows[i].rssi, rows[i].ch, band,
                 rows[i].ssid[0] ? rows[i].ssid : "<hidden>");
    }
    return rows;
}

/* ===== line builders ===== */

/* Map RSSI -90..-30 dBm to a nearest-integer fraction N/5 (0..5) */
static int rssi_to_num_over_5(int rssi) {
    int x = rssi;
    if (x < -90) x = -90;
    if (x > -30) x = -30;
    /* nearest: +30 is half of 60 */
    return ((x + 90) * 5 + 30) / 60;  // 0..5
}


/* Compact metrics for line 2. Always <= 15 chars (+NUL). Example: "-45  3/5 2Gc06 " */
static int build_metrics_prefix(char dst[18], int rssi, int ch) {
    int n_over_5 = rssi_to_num_over_5(rssi);
    /* width calc: 4 (rssi) +1 (space) +3 ("N/5") +1 (space) +2 (band)
       +1 ('c') +2 (ch) +1 (space) = 15, plus NUL => needs 16 bytes */
    int n = snprintf(dst, 16, "%4d %1d/5 %2sc%02d ",
                     rssi, n_over_5, band_from_channel(ch), ch);
    if (n < 0) n = 0;
    if (n > 15) n = 15;
    dst[n] = '\0';
    return n;
}


/* Copy at most 'n' chars from src to dst, ensure NUL */
static void copy_n(char *dst, size_t dst_sz, const char *src, size_t n) {
    if (dst_sz == 0) return;
    size_t i = 0;
    for (; i < n && src[i] && i + 1 < dst_sz; ++i) dst[i] = src[i];
    dst[i] = '\0';
}

/* Split SSID into lineA (<=21 chars) and remainder pointer/len */
static int split_ssid(const char *ssid, char lineA[22], const char **rem_ptr) {
    size_t len = strlen(ssid);
    size_t take = (len > MAX_COLS) ? MAX_COLS : len; // up to 21 chars
    copy_n(lineA, 22, ssid, take);
    *rem_ptr = ssid + take;
    return (int)take;
}

void app_main(void) {
    /* Wi-Fi bring-up */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* OLED bring-up */
    oled_init();

    /* Initial scan */
    size_t n = 0;
    ap_row_t *rows = scan_wifi(&n);
    int start_idx = 0;
    int64_t last_scan_us = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();
        if ((now - last_scan_us) / 1000 >= RESCAN_MS) {
            free(rows);
            rows = scan_wifi(&n);
            last_scan_us = now;
            start_idx = 0;
        }

        fb_clear();

        if (n == 0) {
            fb_draw_text_fit(0, 0, "No APs found", MAX_COLS);
        } else {
            for (int slot = 0; slot < APS_PER_SCREEN; ++slot) {
                size_t i = (start_idx + slot) % n;
                const ap_row_t *ap = &rows[i];

                /* Prepare SSID pieces */
                const char *name = ap->ssid[0] ? ap->ssid : "<hidden>";
                char lineA[22];
                const char *rem;
                split_ssid(name, lineA, &rem);

                /* Build metrics prefix and then pack remainder into lineB */
                char pref[18];
                int pref_len = build_metrics_prefix(pref, ap->rssi, ap->ch);

                char lineB[22];
                // Copy prefix
                int k = 0;
                for (; k < pref_len && k < MAX_COLS; ++k) lineB[k] = pref[k];
                // Fill with remainder of SSID
                for (int j = 0; rem[j] && k < MAX_COLS; ++j, ++k) lineB[k] = rem[j];
                lineB[k] = '\0';

                /* Draw two lines (slot * 2 rows) */
                int rowA = slot * LINES_PER_AP;
                int rowB = rowA + 1;
                fb_draw_text_fit(0, rowA, lineA, MAX_COLS);
                fb_draw_text_fit(0, rowB, lineB, MAX_COLS);
            }

            /* advance by one AP per step */
            start_idx = (start_idx + 1) % n;
        }

        oled_flush_full(panel);
        vTaskDelay(pdMS_TO_TICKS(SCROLL_MS));
    }
}

