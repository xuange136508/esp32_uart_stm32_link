#ifndef MYAPP_H
#define MYAPP_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <mutex>
#include <list>
#include <functional>
#include <memory>
#include "protocol.h"
#include "background_task.h"
#include "ai_role.h"
#include "stm32_comm.h"

#define APP_TAG "MyApplication"

enum DeviceState {
    kDeviceStateUnknown,
    kDeviceStateStarting,
    kDeviceStateWifiConfiguring,
    kDeviceStateIdle,
    kDeviceStateConnecting,
    kDeviceStateListening,
    kDeviceStateSpeaking,
    kDeviceStateUpgrading,
    kDeviceStateActivating,
    kDeviceStateFatalError
};

class MyApp {
public:
public:
    static MyApp& GetInstance() {
        static MyApp instance;
        return instance;
    }

    void startLoop();

    // 添加任务到任务列表
    void addTask(std::function<void()> task);
    // 设置设备状态
    void SetDeviceState(DeviceState state);
    // 开启设备麦克风监听
    void StartListening();
    void StopListening();
    std::vector<std::shared_ptr<AiRole>> getAgentList();

    // STM32通信接口
    STM32Comm& GetSTM32Comm() { return stm32_comm_; }

private:
    MyApp(); // 构造函数
    ~MyApp(); // 析构函数
    void MainLoop();

    void AudioLoop();
    void OnAudioInput();
    void OnAudioOutput();
    void ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples);
    void ResetDecoder();
    size_t GetFeedSize();

    // STM32数据处理回调
    void OnSTM32SensorData(const STM32SensorData& sensor_data);
    void OnSTM32JsonData(const std::string& json_data);
    
    // STM32播放控制处理
    void HandlePlayControlCommand(const cJSON* json);
    void PlayPrenatalEducation();
    void PlayWhiteNoise();
    void StopPlaying();
    void HandleVolumeControl(const std::string& status);

    EventGroupHandle_t event_group_ = nullptr;

    TaskHandle_t main_loop_task_handle_ = nullptr;
    // Audio encode / decode
    TaskHandle_t audio_loop_task_handle_ = nullptr;
    // STM32通信测试任务
    TaskHandle_t stm32_test_task_handle_ = nullptr;
    
    BackgroundTask* background_task_ = nullptr;
    // 当前状态
    volatile DeviceState device_state_ = kDeviceStateUnknown;

    std::list<std::function<void()>> main_tasks_;
    std::list<std::vector<int16_t>> audio_decode_queue_;
    std::mutex mutex_;
    std::unique_ptr<Protocol> protocol_;
    const EventBits_t SCHEDULE_EVENT = 0x01; // 示例事件位
    // 用于模拟分包
    int pkg_type = 1;
    int pkg_order = 0;
    // 用于存储json解析信息（内置智能体）
    std::vector<std::shared_ptr<AiRole>> agentRoles;

    // STM32通信对象
    STM32Comm stm32_comm_;
};

#endif // MYAPP_H
