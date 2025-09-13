#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

/* I2C + panel */
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"      // esp_lcd_new_panel_ssd1306()
#include "esp_lcd_panel_ssd1306.h"     // esp_lcd_panel_ssd1306_config_t

#define TAG           "i2c_oled"
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C
#define OLED_SDA_PIN  21
#define OLED_SCL_PIN  22

static void fill_hstripe(esp_lcd_panel_handle_t panel,
                         int x0, int y0, int x1, int y1, bool on)
{
    const int w = x1 - x0, h = y1 - y0;         // h must be multiple of 8 for 1bpp
    const size_t bytes = (w * h) / 8;
    uint8_t *buf = (uint8_t *)malloc(bytes);
    if (!buf) return;
    memset(buf, on ? 0xFF : 0x00, bytes);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x0, y0, x1, y1, buf));
    free(buf);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "Init I2C SDA=%d SCL=%d @100kHz", OLED_SDA_PIN, OLED_SCL_PIN);

    /* ----- I2C master bus (new driver API) ----- */
    i2c_master_bus_handle_t i2c_bus = NULL;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* ----- Panel IO over I2C ----- */
    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = 0x3C,          // or 0x3D for some modules
        .scl_speed_hz = 100000,    // 100kHz is safe; bump later if stable
        .control_phase_bytes = 1,  // SSD1306 uses 1 control byte
        .dc_bit_offset = 6,        // D/C# is bit 6 of that control byte
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        // .flags = { .dc_low_on_data = 0 }, // default is fine (0=cmd, 1=data)
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &io));

    /* ----- SSD1306 panel (monochrome 1bpp) ----- */
    esp_lcd_panel_handle_t panel = NULL;

    // Only 'height' lives in vendor config; width is implicit 128.
    const esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {
        .height = OLED_HEIGHT,      // 64 (default) or 32
    };

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,       // set GPIO if your module exposes RST
        .bits_per_pixel = 1,        // SSD1306 is 1bpp
        .vendor_config = &ssd1306_cfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    /* ----- Clear screen ----- */
    size_t full_bytes = (OLED_WIDTH * OLED_HEIGHT) / 8;
    uint8_t *clear = (uint8_t *)calloc(1, full_bytes);
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, clear));
    free(clear);

    /* ----- Simple splash: alternating 8px bands ----- */
    for (int y = 0; y < OLED_HEIGHT; y += 16) {
        fill_hstripe(panel, 0, y, OLED_WIDTH, y + 8, true);
        fill_hstripe(panel, 0, y + 8, OLED_WIDTH, y + 16, false);
    }

    ESP_LOGI(TAG, "SSD1306 init OK. Showing bands.");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

