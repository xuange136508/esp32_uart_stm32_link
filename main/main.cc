#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "protocol.h"
#include "websocket_protocol.h"
#include "wifi_manager.h"

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <driver/i2s_std.h>

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#include <string>
#include <mutex>
#include <list>
#include "MyApp.h"
#include "esp_psram.h"



extern "C" void app_main(void)
{
    // 初始化事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 初始化 NVS flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    printf("开始配置网络！\n");
    const char* ssid = "mama"; 
    const char* password = "Gzscmama.cn";
    WiFiManager wifiManager(ssid, password); 
    wifiManager.init();  // 连接wifi

}



