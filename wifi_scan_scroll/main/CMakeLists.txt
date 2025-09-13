idf_component_register(
  SRCS "wifi_scan_scroll.c"
  PRIV_REQUIRES
    esp_driver_i2c
    esp_lcd
    esp_event
    esp_netif
    esp_wifi
    nvs_flash
    esp_timer
)

