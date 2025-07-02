#include "stm32_comm.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "STM32_COMM";

STM32Comm::STM32Comm() 
    : uart_queue_(nullptr)
    , uart_receive_task_handle_(nullptr)
    , is_initialized_(false)
    , data_callback_(nullptr)
    , json_callback_(nullptr) {
}

STM32Comm::~STM32Comm() {
    Deinit();
}

bool STM32Comm::Init() {
    if (is_initialized_) {
        ESP_LOGW(TAG, "STM32通信已经初始化");
        return true;
    }

    ESP_LOGI(TAG, "初始化STM32 UART通信...");
    
    // UART配置
    uart_config_t uart_config = {
        .baud_rate = STM32_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 创建UART队列
    uart_queue_ = xQueueCreate(10, sizeof(UartMessage));
    if (uart_queue_ == nullptr) {
        ESP_LOGE(TAG, "创建UART队列失败");
        return false;
    }

    // 安装UART驱动
    esp_err_t ret = uart_driver_install(STM32_UART_NUM, STM32_UART_BUF_SIZE * 2, STM32_UART_BUF_SIZE * 2, 20, &uart_queue_, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "安装UART驱动失败: %s", esp_err_to_name(ret));
        vQueueDelete(uart_queue_);
        uart_queue_ = nullptr;
        return false;
    }

    ret = uart_param_config(STM32_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "配置UART参数失败: %s", esp_err_to_name(ret));
        uart_driver_delete(STM32_UART_NUM);
        vQueueDelete(uart_queue_);
        uart_queue_ = nullptr;
        return false;
    }

    ret = uart_set_pin(STM32_UART_NUM, STM32_UART_TX_PIN, STM32_UART_RX_PIN, STM32_UART_RTS_PIN, STM32_UART_CTS_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置UART引脚失败: %s", esp_err_to_name(ret));
        uart_driver_delete(STM32_UART_NUM);
        vQueueDelete(uart_queue_);
        uart_queue_ = nullptr;
        return false;
    }
    
    is_initialized_ = true;
    ESP_LOGI(TAG, "STM32 UART初始化完成 - TX: GPIO%d, RX: GPIO%d, 波特率: %d", 
             STM32_UART_TX_PIN, STM32_UART_RX_PIN, STM32_UART_BAUD_RATE);
    
    return true;
}

void STM32Comm::Deinit() {
    if (!is_initialized_) {
        return;
    }

    StopReceiveTask();

    if (uart_queue_) {
        uart_driver_delete(STM32_UART_NUM);
        vQueueDelete(uart_queue_);
        uart_queue_ = nullptr;
    }

    is_initialized_ = false;
    ESP_LOGI(TAG, "STM32通信已关闭");
}

bool STM32Comm::SendJsonToSTM32(const std::string& json_data) {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "STM32通信未初始化");
        return false;
    }

    std::lock_guard<std::mutex> lock(uart_mutex_);
    
    std::string data_with_ending = json_data + "\r\n";
    int len = uart_write_bytes(STM32_UART_NUM, data_with_ending.c_str(), data_with_ending.length());
    if (len > 0) {
        ESP_LOGI(TAG, "发送JSON到STM32成功: %s", json_data.c_str());
        return true;
    } else {
        ESP_LOGE(TAG, "发送JSON到STM32失败");
        return false;
    }
}

void STM32Comm::SendControlCommand(bool breathing_led, bool rgb_led) {
    cJSON *json = cJSON_CreateObject();
    cJSON *device = cJSON_CreateString("ESP32");
    cJSON *command = cJSON_CreateString("control");
    cJSON *breathing = cJSON_CreateString(breathing_led ? "on" : "off");
    cJSON *rgb = cJSON_CreateString(rgb_led ? "on" : "off");
    
    cJSON_AddItemToObject(json, "device", device);
    cJSON_AddItemToObject(json, "command", command);
    cJSON_AddItemToObject(json, "breathing_led", breathing);
    cJSON_AddItemToObject(json, "rgb_led", rgb);
    
    char *json_string = cJSON_Print(json);
    if (json_string) {
        SendJsonToSTM32(std::string(json_string));
        free(json_string);
    }
    
    cJSON_Delete(json);
}

void STM32Comm::SetDataReceivedCallback(DataReceivedCallback callback) {
    data_callback_ = callback;
}

void STM32Comm::SetJsonReceivedCallback(JsonReceivedCallback callback) {
    json_callback_ = callback;
}

void STM32Comm::StartReceiveTask() {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "STM32通信未初始化，无法启动接收任务");
        return;
    }

    if (uart_receive_task_handle_ != nullptr) {
        ESP_LOGW(TAG, "UART接收任务已经在运行");
        return;
    }

    xTaskCreatePinnedToCore([](void* arg) {
        static_cast<STM32Comm*>(arg)->UartReceiveTask();
        vTaskDelete(NULL);
    }, "stm32_uart_rx", 4096 * 2, this, 5, &uart_receive_task_handle_, 0);

    ESP_LOGI(TAG, "STM32 UART接收任务已启动");
}

void STM32Comm::StopReceiveTask() {
    if (uart_receive_task_handle_) {
        vTaskDelete(uart_receive_task_handle_);
        uart_receive_task_handle_ = nullptr;
        ESP_LOGI(TAG, "STM32 UART接收任务已停止");
    }
}

void STM32Comm::UartReceiveTask() {
    char data[STM32_UART_BUF_SIZE];
    uart_event_t event;
    ESP_LOGI(TAG, "UART接收任务启动 - 开始监听STM32数据...");
    
    while (true) {
        // 等待UART事件
        if (xQueueReceive(uart_queue_, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                {
                    // 读取数据
                    int len = uart_read_bytes(STM32_UART_NUM, data, event.size, portMAX_DELAY);
                    if (len > 0) {
                        // ESP_LOGI(TAG, "接收到%d字节数据", len);
                        
                        std::lock_guard<std::mutex> lock(uart_mutex_);
                        uart_rx_buffer_.append(data, len);
                        
                        // ESP_LOGI(TAG, "当前缓冲区内容: %s", uart_rx_buffer_.c_str());
                        
                        // 查找完整的JSON消息（以\r\n结尾）
                        size_t pos;
                        while ((pos = uart_rx_buffer_.find("\r\n")) != std::string::npos) {
                            std::string json_message = uart_rx_buffer_.substr(0, pos);
                            uart_rx_buffer_.erase(0, pos + 2);
                            
                            if (!json_message.empty()) {
                                ESP_LOGI(TAG, "解析到完整JSON: %s", json_message.c_str());
                                ProcessSTM32Data(json_message);
                            }
                        }
                        
                        // 防止缓冲区溢出
                        if (uart_rx_buffer_.length() > STM32_UART_BUF_SIZE) {
                            ESP_LOGW(TAG, "UART接收缓冲区溢出，清空缓冲区");
                            uart_rx_buffer_.clear();
                        }
                    }
                    break;
                }
                    
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO溢出");
                    uart_flush_input(STM32_UART_NUM);
                    xQueueReset(uart_queue_);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART环形缓冲区已满");
                    uart_flush_input(STM32_UART_NUM);
                    xQueueReset(uart_queue_);
                    break;
                    
                case UART_BREAK:
                case UART_DATA_BREAK:
                    ESP_LOGW(TAG, "UART接收到break信号 - 请检查硬件连接!");
                    break;
                    
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART校验错误 - 可能是波特率不匹配");
                    break;
                    
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART帧错误 - 可能是波特率不匹配");
                    break;
                    
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "UART模式检测");
                    break;
                    
                case UART_WAKEUP:
                    ESP_LOGI(TAG, "UART唤醒事件");
                    break;
                    
                case UART_EVENT_MAX:
                    ESP_LOGW(TAG, "UART事件最大值");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "UART未知事件类型: %d", event.type);
                    break;
            }
        }
    }
}

void STM32Comm::ProcessSTM32Data(const std::string& json_data) {
    cJSON *json = cJSON_Parse(json_data.c_str());
    if (json == NULL) {
        ESP_LOGE(TAG, "JSON解析失败");
        return;
    }
    
    // 调用JSON回调（如果设置了）
    if (json_callback_) {
        json_callback_(json_data);
    }
    
    cJSON *device = cJSON_GetObjectItem(json, "device");
    if (device == NULL || !cJSON_IsString(device)) {
        ESP_LOGE(TAG, "JSON格式错误：缺少device字段");
        cJSON_Delete(json);
        return;
    }
    
    std::string device_name = device->valuestring;
    
    if (device_name == "STM32_Sensor") {
        // 解析传感器数据
        STM32SensorData sensor_data = {0};
        if (ParseSensorData(json, sensor_data)) {
            ESP_LOGI(TAG, "传感器数据解析成功");
            // 调用数据回调（如果设置了）
            if (data_callback_) {
                data_callback_(sensor_data);
            }
        }
        
    } else if (device_name == "STM32_Control") {
        // 处理控制状态数据
        cJSON *controls = cJSON_GetObjectItem(json, "controls");
        if (controls) {
            cJSON *breathing_led = cJSON_GetObjectItem(controls, "breathing_led");
            cJSON *rgb_led = cJSON_GetObjectItem(controls, "rgb_led");
            
            if (breathing_led && cJSON_IsBool(breathing_led)) {
                ESP_LOGI(TAG, "呼吸灯当前状态: %s", cJSON_IsTrue(breathing_led) ? "开启" : "关闭");
            }
            if (rgb_led && cJSON_IsBool(rgb_led)) {
                ESP_LOGI(TAG, "RGB灯当前状态: %s", cJSON_IsTrue(rgb_led) ? "开启" : "关闭");
            }
        }
        
    } else if (device_name == "STM32_PlayControl") {
        // 处理播放控制命令
        ESP_LOGI(TAG, "接收到播放控制命令");
        cJSON *playback = cJSON_GetObjectItem(json, "playback");
        if (playback) {
            cJSON *status = cJSON_GetObjectItem(playback, "status");
            cJSON *content_type = cJSON_GetObjectItem(playback, "content_type");
            
            if (status && cJSON_IsString(status) && content_type && cJSON_IsString(content_type)) {
                ESP_LOGI(TAG, "播放状态: %s, 内容类型: %s", status->valuestring, content_type->valuestring);
            }
        }
        
        // 处理控制状态
        cJSON *controls = cJSON_GetObjectItem(json, "controls");
        if (controls) {
            cJSON *breathing_led = cJSON_GetObjectItem(controls, "breathing_led");
            cJSON *rgb_led = cJSON_GetObjectItem(controls, "rgb_led");
            
            if (breathing_led && cJSON_IsBool(breathing_led)) {
                ESP_LOGI(TAG, "播放时呼吸灯状态: %s", cJSON_IsTrue(breathing_led) ? "开启" : "关闭");
            }
            if (rgb_led && cJSON_IsBool(rgb_led)) {
                ESP_LOGI(TAG, "播放时RGB灯状态: %s", cJSON_IsTrue(rgb_led) ? "开启" : "关闭");
            }
        }
    }
    
    cJSON_Delete(json);
}

bool STM32Comm::ParseSensorData(const cJSON* json, STM32SensorData& sensor_data) {
    // 解析时间戳
    cJSON *timestamp = cJSON_GetObjectItem(json, "timestamp");
    if (timestamp && cJSON_IsNumber(timestamp)) {
        sensor_data.timestamp = timestamp->valueint;
    }
    
    // 解析传感器数据
    cJSON *sensors = cJSON_GetObjectItem(json, "sensors");
    if (sensors) {
        cJSON *light_voltage = cJSON_GetObjectItem(sensors, "light_voltage");
        cJSON *vibration = cJSON_GetObjectItem(sensors, "vibration");
        cJSON *touch = cJSON_GetObjectItem(sensors, "touch");
        cJSON *temperature = cJSON_GetObjectItem(sensors, "temperature");
        cJSON *humidity = cJSON_GetObjectItem(sensors, "humidity");
        cJSON *dht11_valid = cJSON_GetObjectItem(sensors, "dht11_valid");
        
        if (light_voltage && cJSON_IsNumber(light_voltage)) {
            sensor_data.light_voltage = light_voltage->valuedouble;
            ESP_LOGI(TAG, "光照电压: %.2f V", sensor_data.light_voltage);
        }
        if (vibration && cJSON_IsBool(vibration)) {
            sensor_data.vibration = cJSON_IsTrue(vibration);
            ESP_LOGI(TAG, "振动状态: %s", sensor_data.vibration ? "检测到" : "无");
        }
        if (touch && cJSON_IsBool(touch)) {
            sensor_data.touch = cJSON_IsTrue(touch);
            ESP_LOGI(TAG, "触摸状态: %s", sensor_data.touch ? "按下" : "释放");
        }
        if (temperature && cJSON_IsNumber(temperature)) {
            sensor_data.temperature = temperature->valuedouble;
            ESP_LOGI(TAG, "温度: %.1f °C", sensor_data.temperature);
        }
        if (humidity && cJSON_IsNumber(humidity)) {
            sensor_data.humidity = humidity->valuedouble;
            ESP_LOGI(TAG, "湿度: %.1f %%", sensor_data.humidity);
        }
        if (dht11_valid && cJSON_IsBool(dht11_valid)) {
            sensor_data.dht11_valid = cJSON_IsTrue(dht11_valid);
            ESP_LOGI(TAG, "DHT11状态: %s", sensor_data.dht11_valid ? "正常" : "异常");
        }
    }
    
    // 解析控制状态
    cJSON *controls = cJSON_GetObjectItem(json, "controls");
    if (controls) {
        cJSON *breathing_led = cJSON_GetObjectItem(controls, "breathing_led");
        cJSON *rgb_led = cJSON_GetObjectItem(controls, "rgb_led");
        
        if (breathing_led && cJSON_IsBool(breathing_led)) {
            sensor_data.breathing_led = cJSON_IsTrue(breathing_led);
            ESP_LOGI(TAG, "呼吸灯状态: %s", sensor_data.breathing_led ? "开启" : "关闭");
        }
        if (rgb_led && cJSON_IsBool(rgb_led)) {
            sensor_data.rgb_led = cJSON_IsTrue(rgb_led);
            ESP_LOGI(TAG, "RGB灯状态: %s", sensor_data.rgb_led ? "开启" : "关闭");
        }
    }
    
    return true;
} 