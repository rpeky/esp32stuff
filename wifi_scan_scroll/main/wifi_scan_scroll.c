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

#define TAG                 "WIFI_LIST"

/* ===== OLED wiring & size ===== */
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_ADDR           0x3C
#define OLED_SDA_PIN        21
#define OLED_SCL_PIN        22

/* ===== Font & layout (5x7 into 6x8 cells) ===== */
#define CELL_W              6
#define CELL_H              8
#define MAX_COLS            (OLED_WIDTH / CELL_W)   // 21
#define MAX_ROWS            (OLED_HEIGHT / CELL_H)  // 8
#define HEADER_ROWS         1                       // 1 text row header (8 px)
#define LIST_Y0             (HEADER_ROWS * CELL_H)  // = 8
#define VISIBLE_ROWS        (MAX_ROWS - HEADER_ROWS) // 7 rows visible

/* ===== Timing ===== */
#define RESCAN_MS           8000    // rescan Wi-Fi every 8s
#define FRAME_MS            150     // redraw rate
#define SCROLL_MS           800     // scroll step time when overflowing

/* ===== 1bpp framebuffer ===== */
static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8];

/* ===== 5x7 ASCII font (32..126), LSB=top ===== */
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

/* ===== Wi-Fi scan state ===== */
static wifi_ap_record_t *ap_list = NULL;
static uint16_t ap_count = 0;

/* ===== utils ===== */
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* Map RSSI -90..-30 dBm -> N/5 (0..5) */
static int rssi_to_num_over_5(int rssi) {
    int x = rssi; if (x < -90) x = -90; if (x > -30) x = -30;
    return ((x + 90) * 5 + 30) / 60; // nearest
}

/* band letter from channel (coarse) */
static char band_letter(int ch) {
    if (ch >= 1 && ch <= 14) return '2';         // 2.4 GHz
    if (ch >= 32 && ch < 200) return '5';        // 5 GHz
    return '6';                                  // 6 GHz-ish or unknown
}
/* qsort comparator: RSSI desc, then channel asc, then SSID */
static int cmp_ap_records(const void *a, const void *b) {
    const wifi_ap_record_t *x = (const wifi_ap_record_t *)a;
    const wifi_ap_record_t *y = (const wifi_ap_record_t *)b;

    if (x->rssi != y->rssi)       return (y->rssi - x->rssi);       // stronger first
    if (x->primary != y->primary) return (x->primary - y->primary); // lower channel first
    return strncmp((const char *)x->ssid, (const char *)y->ssid, 32);
}
/* ===== scanning ===== */
static void do_scan(void) {
    wifi_scan_config_t cfg = {
        .ssid = 0, .bssid = 0, .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 120, .max = 300 },
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&cfg, true));

    uint16_t n = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&n));

    wifi_ap_record_t *tmp = calloc(n ? n : 1, sizeof(*tmp));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&n, tmp));
    ap_count = n;

    /* sort by RSSI descending, then by channel, then SSID */
    qsort(tmp, n, sizeof(*tmp), cmp_ap_records);

    free(ap_list);
    ap_list = tmp;

    /* log summary to serial */
    ESP_LOGI(TAG, "APs: %u", (unsigned)ap_count);
    for (uint16_t i = 0; i < ap_count && i < 40; ++i) {
        ESP_LOGI(TAG, "#%02u ch%02d %4ddBm %1d/5  %s",
                 i, ap_list[i].primary, ap_list[i].rssi,
                 rssi_to_num_over_5(ap_list[i].rssi),
                 (const char*)ap_list[i].ssid);
    }
}

/* ===== header & list render ===== */
static void draw_header(int ms_to_next) {
    char line[40];
    int secs = ms_to_next / 1000; if (secs < 0) secs = 0;
    snprintf(line, sizeof(line), "APs:%u  next:%ds", (unsigned)ap_count, secs);
    fb_draw_text_fit(0, 0, line, MAX_COLS);
}

/* Build one 21-char max line: "N/5 Bcc rssi SSID..." (fixed-width prefix + SSID) */
static void build_line(char out[32], const wifi_ap_record_t *ap) {
    char pref[16];  // 12 visible + NUL
    int rssi = ap->rssi; if (rssi < -99) rssi = -99; // keep width ≤3
    int n5 = rssi_to_num_over_5(rssi);
    char b = band_letter(ap->primary);
    int pn = snprintf(pref, sizeof(pref), "%1d/5 %c%02d %3d", n5, b, ap->primary, rssi);
    pn = clampi(pn, 0, 12);

    /* compose: prefix + space + SSID truncated to fill MAX_COLS */
    int avail_ssid = MAX_COLS - (pn + 1);
    if (avail_ssid < 0) avail_ssid = 0;

    int ssid_len = strnlen((const char*)ap->ssid, 32);
    int use = ssid_len > avail_ssid ? avail_ssid : ssid_len;

    int o = 0;
    memcpy(out + o, pref, pn); o += pn;
    if (o < 31) out[o++] = ' ';
    if (use > 0) memcpy(out + o, ap->ssid, use), o += use;
    out[o] = '\0';
}

static void draw_list(int start_idx) {
    for (int row = 0; row < VISIBLE_ROWS; ++row) {
        int idx = start_idx + row;
        if (ap_count == 0) break;
        if (idx >= ap_count) break;  // static (no wrap) when not scrolling
        char line[32];
        build_line(line, &ap_list[idx]);
        fb_draw_text_fit(0, (LIST_Y0 / CELL_H) + row, line, MAX_COLS);
    }
}

static void draw_list_scrolling(int start_idx) {
    for (int row = 0; row < VISIBLE_ROWS; ++row) {
        if (ap_count == 0) break;
        int idx = (start_idx + row) % ap_count;  // wrap
        char line[32];
        build_line(line, &ap_list[idx]);
        fb_draw_text_fit(0, (LIST_Y0 / CELL_H) + row, line, MAX_COLS);
    }
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

    /* OLED */
    oled_init();

    /* initial scan */
    do_scan();
    int64_t last_scan_us = esp_timer_get_time();
    int64_t last_scroll_us = last_scan_us;
    int scroll_idx = 0;

    while (1) {
        int64_t now = esp_timer_get_time();
        int elapsed_ms = (int)((now - last_scan_us) / 1000);

        /* rescan */
        if (elapsed_ms >= RESCAN_MS) {
            do_scan();
            last_scan_us = now;
            /* reset scroll position so user sees strongest first after each scan */
            scroll_idx = 0;
            last_scroll_us = now;
            elapsed_ms = 0;
        }

        /* decide whether we need scrolling */
        bool need_scroll = (ap_count > VISIBLE_ROWS);

        if (need_scroll) {
            int since_scroll = (int)((now - last_scroll_us) / 1000);
            if (since_scroll >= SCROLL_MS) {
                scroll_idx = (scroll_idx + 1) % ap_count;
                last_scroll_us = now;
            }
        }

        /* draw */
        fb_clear();
        draw_header(RESCAN_MS - elapsed_ms);
        if (!need_scroll) {
            draw_list(0);                 // fits: keep current static format
        } else {
            draw_list_scrolling(scroll_idx); // overflow: scroll through
        }
        oled_flush_full(panel);

        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

