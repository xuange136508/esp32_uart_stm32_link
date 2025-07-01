#include "wifi_manager.h"

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "protocol.h"
#include "websocket_protocol.h"

#include <list>
#include "background_task.h"
#include "MyApp.h"
#include "http_server.h"

WiFiManager::WiFiManager(const char* ssid, const char* password) {
    strcpy((char*)wifi_config_.sta.ssid, ssid);
    strcpy((char*)wifi_config_.sta.password, password);
}

void WiFiManager::init() {
    /* 初始化NVS，已在main.cc初始化了 */
    // esp_err_t ret;
    // ret = nvs_flash_init(); 
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }

    /* 网卡初始化 */
    ESP_ERROR_CHECK(esp_netif_init());
    /* 创建新的事件循环, 已在main.cc初始化了 */
    // ESP_ERROR_CHECK(esp_event_loop_create_default()); 
    /* 用户初始化STA模式 */
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    // 初始化 Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

   
    // 注册事件处理函数
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

     // 设置 Wi-Fi 为 STA 模式并启动
     esp_wifi_set_mode(WIFI_MODE_STA);
     esp_wifi_set_config(WIFI_IF_STA, &wifi_config_);
    // 启动WIFI 
    esp_wifi_start();

    /* 开始连接 */
    esp_wifi_connect();
}


void WiFiManager::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGI(TAG, "Wi-Fi 连接失败，原因: %d", disconnected->reason);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect(); // 延迟 1 秒后尝试重连

        } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI(TAG, "Wi-Fi 已连接");
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {    
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG, "Wi-Fi 连接成功，IP 地址: " IPSTR, IP2STR(&event->ip_info.ip));
    
            /* 联网后，测试 WebSocket 代码 */
            // std::unique_ptr<Protocol> protocol_;
            // protocol_ = std::make_unique<WebsocketProtocol>();
            // protocol_->OpenAudioChannel();
           

            // 开启主循环任务
            MyApp::GetInstance().startLoop();
        }
    } 
}
