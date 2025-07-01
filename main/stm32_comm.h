#ifndef STM32_COMM_H
#define STM32_COMM_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include <string>
#include <mutex>
#include <functional>

// UART配置
#define STM32_UART_NUM UART_NUM_2
#define STM32_UART_TX_PIN GPIO_NUM_17
#define STM32_UART_RX_PIN GPIO_NUM_18
#define STM32_UART_RTS_PIN UART_PIN_NO_CHANGE
#define STM32_UART_CTS_PIN UART_PIN_NO_CHANGE
#define STM32_UART_BAUD_RATE 115200
#define STM32_UART_BUF_SIZE 1024

// 串口通信数据结构
struct UartMessage {
    std::string data;
    size_t length;
};

// STM32传感器数据结构
struct STM32SensorData {
    double light_voltage;
    bool vibration;
    bool touch;
    double temperature;
    double humidity;
    bool dht11_valid;
    bool breathing_led;
    bool rgb_led;
    uint32_t timestamp;
};

// STM32通信类
class STM32Comm {
public:
    // 数据接收回调函数类型
    using DataReceivedCallback = std::function<void(const STM32SensorData&)>;
    using JsonReceivedCallback = std::function<void(const std::string&)>;

    STM32Comm();
    ~STM32Comm();

    // 基本通信接口
    bool Init();
    void Deinit();
    bool SendJsonToSTM32(const std::string& json_data);
    void SendControlCommand(bool breathing_led, bool rgb_led);

    // 回调设置
    void SetDataReceivedCallback(DataReceivedCallback callback);
    void SetJsonReceivedCallback(JsonReceivedCallback callback);

    // 启动和停止通信任务
    void StartReceiveTask();
    void StopReceiveTask();

private:
    // 内部方法
    void UartReceiveTask();
    void ProcessSTM32Data(const std::string& json_data);
    bool ParseSensorData(const cJSON* json, STM32SensorData& sensor_data);

    // 成员变量
    QueueHandle_t uart_queue_;
    TaskHandle_t uart_receive_task_handle_;
    std::mutex uart_mutex_;
    std::string uart_rx_buffer_;
    bool is_initialized_;
    
    // 回调函数
    DataReceivedCallback data_callback_;
    JsonReceivedCallback json_callback_;
};

#endif // STM32_COMM_H 