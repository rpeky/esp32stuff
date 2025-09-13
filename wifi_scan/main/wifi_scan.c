#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "scan";

static const char *authmode_to_str(wifi_auth_mode_t m) {
    switch (m) {
        case WIFI_AUTH_OPEN:           return "OPEN";
        case WIFI_AUTH_WEP:            return "WEP";
        case WIFI_AUTH_WPA_PSK:        return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK:       return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:   return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE:return "WPA2-ENT";
        case WIFI_AUTH_WPA3_PSK:       return "WPA3-PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK:  return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK:       return "WAPI-PSK";
        default:                       return "UNKNOWN";
    }
}

void app_main(void) {
    // NVS is required by Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void) esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    while (1) {
        ESP_LOGI(TAG, "Starting scan...");
        wifi_scan_config_t sc = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,                // scan all channels
            .show_hidden = false,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time.active = { .min = 100, .max = 300 },
        };
        ESP_ERROR_CHECK(esp_wifi_scan_start(&sc, true)); // block until done

        uint16_t ap_num = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
        wifi_ap_record_t *aps = calloc(ap_num, sizeof(*aps));
        if (!aps) {
            ESP_LOGE(TAG, "calloc failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, aps));

        ESP_LOGI(TAG, "Found %u APs:", ap_num);
        for (int i = 0; i < ap_num; i++) {
            char ssid[33];
            memcpy(ssid, aps[i].ssid, sizeof(aps[i].ssid));
            ssid[32] = '\0';
            ESP_LOGI(TAG, "[%2d] %-32s | ch %2u | RSSI %3d | %s",
                     i, ssid, aps[i].primary, aps[i].rssi, authmode_to_str(aps[i].authmode));
        }
        free(aps);

        vTaskDelay(pdMS_TO_TICKS(5000)); // scan every ~5s
    }
}
