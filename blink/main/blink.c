#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "led_strip.h"

#define LED_GPIO    16                // your WS2812B data pin
#define LED_NUM     1                 // number of pixels
#define RMT_RES_HZ  (10 * 1000 * 1000) // 10 MHz

static inline void color_wheel(uint8_t x, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (x < 85)      { *r = 3*x;      *g = 255 - 3*x; *b = 0; }
    else if (x < 170){ x -= 85; *r = 255 - 3*x; *g = 0; *b = 3*x; }
    else             { x -= 170; *r = 0;         *g = 3*x; *b = 255 - 3*x; }
}

void app_main(void) {
    led_strip_handle_t strip;

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // WS2812B is GRB
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,      // optional; buffer size “symbols”
        .flags = {
            .with_dma = false,        // set true only if you really need DMA (not on all chips)
        },
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));

    uint8_t hue = 0;
    while (1) {
        uint8_t r,g,b;
        color_wheel(hue, &r, &g, &b);
        ESP_ERROR_CHECK(led_strip_set_pixel(strip, 0, r, g, b));
        ESP_ERROR_CHECK(led_strip_refresh(strip));
        vTaskDelay(pdMS_TO_TICKS(30));
        hue = (hue + 2) % 255;
    }
}

