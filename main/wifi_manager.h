#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <esp_log.h>

#define TAG "WiFiManager"

class WiFiManager {
    public:
        WiFiManager(const char* ssid, const char* password);
        void init();

    private:
        static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
        wifi_config_t wifi_config_ = {}; // Wi-Fi 配置
};

#endif // WIFIMANAGER_H