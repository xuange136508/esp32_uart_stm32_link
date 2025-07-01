#include "MyApp.h"
#include "wifi_manager.h"
#include "board.h"
#include "audio_codecs/audio_codec.h"
#include "websocket_protocol.h"
#include <esp_log.h>
#include "utils/base64_encoder.h"
#include <cstring>
#include "http_server.h"
#include "settings.h"
#include "ai_role.h"
#include "esp_spiffs.h"
#include "http_client.h"

static const char* const STATE_STRINGS[] = {
    "unknown",
    "starting",
    "configuring",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "activating",
    "fatal_error",
    "invalid_state"
};

MyApp::MyApp() {
    // 初始化 event_group_
    event_group_ = xEventGroupCreate();
    background_task_ = new BackgroundTask(4096 * 8);
}

MyApp::~MyApp() {
    if (main_loop_task_handle_) {
        vTaskDelete(main_loop_task_handle_);
    }
    if (audio_loop_task_handle_) {
        vTaskDelete(audio_loop_task_handle_);
    }
    if (stm32_test_task_handle_) {
        vTaskDelete(stm32_test_task_handle_);
    }
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
    if (background_task_ != nullptr) {
        delete background_task_;
    }
}

// STM32传感器数据处理回调
void MyApp::OnSTM32SensorData(const STM32SensorData& sensor_data) {
    ESP_LOGI(APP_TAG, "接收到STM32传感器数据:");
    ESP_LOGI(APP_TAG, "  光照电压: %.2f V", sensor_data.light_voltage);
    ESP_LOGI(APP_TAG, "  振动状态: %s", sensor_data.vibration ? "检测到" : "无");
    ESP_LOGI(APP_TAG, "  触摸状态: %s", sensor_data.touch ? "按下" : "释放");
    ESP_LOGI(APP_TAG, "  温度: %.1f °C", sensor_data.temperature);
    ESP_LOGI(APP_TAG, "  湿度: %.1f %%", sensor_data.humidity);
    ESP_LOGI(APP_TAG, "  DHT11状态: %s", sensor_data.dht11_valid ? "正常" : "异常");
    ESP_LOGI(APP_TAG, "  呼吸灯: %s", sensor_data.breathing_led ? "开启" : "关闭");
    ESP_LOGI(APP_TAG, "  RGB灯: %s", sensor_data.rgb_led ? "开启" : "关闭");
    
    // 在这里可以根据传感器数据进行相应的处理
    // 例如：根据触摸状态控制语音识别，根据环境温湿度调整系统参数等
}

// STM32 JSON数据处理回调
void MyApp::OnSTM32JsonData(const std::string& json_data) {
    ESP_LOGI(APP_TAG, "接收到STM32 JSON数据: %s", json_data.c_str());
}

void MyApp::startLoop() {
    SetDeviceState(kDeviceStateIdle);

    // 初始化STM32通信
    if (!stm32_comm_.Init()) {
        ESP_LOGE(APP_TAG, "STM32通信初始化失败");
        return;
    }

    // 设置STM32数据接收回调
    stm32_comm_.SetDataReceivedCallback([this](const STM32SensorData& data) {
        addTask([this, data]() {
            OnSTM32SensorData(data);
        });
    });

    stm32_comm_.SetJsonReceivedCallback([this](const std::string& json_data) {
        addTask([this, json_data]() {
            OnSTM32JsonData(json_data);
        });
    });

    // 启动STM32接收任务
    stm32_comm_.StartReceiveTask();

    // 开启http服务器
    auto& httpServer = HttpServer::GetInstance();
    httpServer.Start();

    /* 初始化板子：SPI、LCD屏幕、Audio音频编码器 */
    auto& board = Board::GetInstance();
    /* 获取 Audio 音频编码器 */
    auto codec = board.GetAudioCodec();

    // 创建websocket对象
    protocol_ = std::make_unique<WebsocketProtocol>();
    // 初始化参数
    protocol_->initParams();
    // 设置音频数据回调
    protocol_->OnIncomingAudio([this](std::vector<uint8_t>&& data) {
        // 接受到如果是音频数据，放到解码队列
        // std::lock_guard<std::mutex> lock(mutex_);
        // audio_decode_queue_.emplace_back(std::move(data));
    });
    protocol_->OnIncomingJson([this](const cJSON* root) {
        // 接受到如果是json数据，进行解析
        auto type = cJSON_GetObjectItem(root, "type");
        auto device = cJSON_GetObjectItem(root, "device"); //"google_baba"
        
        if (type->valueint == 3) { // 比较字符串函数
            auto sub_type = cJSON_GetObjectItem(root, "sub_type");// 1、2、3
            auto data = cJSON_GetObjectItem(root, "data"); 

            if (sub_type->valueint == 1 || sub_type->valueint == 2 || sub_type->valueint == 3) {
                // auto file_name = cJSON_GetObjectItem(data, "file_name"); //"0.pcm"
                // auto file_len = cJSON_GetObjectItem(data, "file_len"); //38962
                // auto type = cJSON_GetObjectItem(data, "stream");//"voice"
                // auto index = cJSON_GetObjectItem(data, "index");//"0"
                // auto bin_data_len = cJSON_GetObjectItem(data, "bin_data_len"); //2048
                auto bin_data = cJSON_GetObjectItem(data, "bin_data"); //二进制数据
                // ESP_LOGI(APP_TAG, "文件名：%s",file_name->valuestring);
                // ESP_LOGI(APP_TAG, "文件长度：%d",file_len->valueint);

                if(sub_type->valueint == 3){
                    SetDeviceState(kDeviceStateIdle);
                }else {
                    SetDeviceState(kDeviceStateSpeaking); // 语音回复中
                }
                
                // 获取音频数据
                // ESP_LOGI(TAG, "base64解码前的数据 ==> %s", bin_data->valuestring);
                std::string base64_audio = Base64Encoder::decode(bin_data->valuestring);
                if (base64_audio.empty()) {
                    // ESP_LOGI(TAG, "解码结束");
                } else {
                    // ESP_LOGI(TAG, "base64解码后数据 ==> %s", base64_audio.c_str());
                    if (base64_audio.size() % 2 != 0) {
                        throw std::invalid_argument("音频数据长度必须是偶数");
                    }
                    std::vector<int16_t> transDatas(base64_audio.size() / 2);
                    memcpy(transDatas.data(), base64_audio.data(), base64_audio.size());
                
                    // 直接输出喇叭
                    auto codec = Board::GetInstance().GetAudioCodec();
                    codec->OutputData(transDatas);
                    // background_task_->Schedule([this, codec, opus = std::move(transDatas)]() mutable {
                    //     codec->OutputData(opus);
                    // });

                    // 直接队列输出
                    // if (base64_audio.size() >= sizeof(int16_t)) {
                    //     audio_decode_queue_.push_back({transDatas});//入栈 audio_decode_queue_.emplace_back(std::move(value));
                    // }else{
                    //     ESP_LOGI(TAG, "memcpy 读取越界");
                    // }
                }

            }
        }
    });

    // 智能体ID
    // Settings settings("agentId", true);
    // std::string agentId = settings.GetString("agentId", "sr_7");
    // ESP_LOGI(APP_TAG, "当前智能体ID为：%s",agentId.c_str());

    // 开启主循环 
    xTaskCreatePinnedToCore([](void* arg) {
        static_cast<MyApp*>(arg)->MainLoop();
        vTaskDelete(NULL);
    }, "main_loop", 4096 * 2, this, 4, &main_loop_task_handle_, 0);


    // 专门处理音频 输入输出 任务
    xTaskCreatePinnedToCore([](void* arg) {
        static_cast<MyApp*>(arg)->AudioLoop();
        vTaskDelete(NULL);
    }, "audio_loop", 4096 * 2, this, 8, &audio_loop_task_handle_, 1);

    // 创建STM32通信测试任务（定时发送控制命令）
    xTaskCreatePinnedToCore([](void* arg) {
        MyApp* app = static_cast<MyApp*>(arg);
        vTaskDelay(pdMS_TO_TICKS(3000)); // 延迟3秒后开始
        
        while (true) {
            // 每10秒发送一次控制命令示例
            app->GetSTM32Comm().SendControlCommand(true, false);  // 开启呼吸灯，关闭RGB灯
            vTaskDelay(pdMS_TO_TICKS(10000));
            
            app->GetSTM32Comm().SendControlCommand(false, true);  // 关闭呼吸灯，开启RGB灯
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        vTaskDelete(NULL);
    }, "stm32_test", 4096, this, 3, &stm32_test_task_handle_, 0);

    // json解析任务
    addTask([this]() {
        try {
            // 挂载Flash的文件系统
            esp_vfs_spiffs_conf_t conf = {
                .base_path = "/spiffs",
                .partition_label = "model",
                .max_files = 5,
                .format_if_mount_failed = true
            };
            esp_vfs_spiffs_register(&conf);

            // 自动加载编译时嵌入的JSON
            AiRoleManager roleManager;
            if (!roleManager.loadFromEmbeddedData()) {
                ESP_LOGE(APP_TAG, "AiRoleManager exception");
                return;
            }
            agentRoles = roleManager.getRoles();

            Settings settings("agentId", true);
            int agentIndex = settings.GetInt("agentIndex", 0);  // 智能体游标

            // 获取第1个元素（会进行边界检查）
            std::shared_ptr<AiRole> role = agentRoles.at(agentIndex);
            settings.SetString("agentId", role.get()->getId());
            ESP_LOGI(APP_TAG, "解析智能体ID：%s",role.get()->getId().c_str());

            // 渲染智能体对应的动画
            auto& board = Board::GetInstance();
            //board.drawAgentImage(role.get()->getImageUrl());

        } catch (const std::exception& e) {
            ESP_LOGE(APP_TAG, "myApp Exception: %s", e.what());
        } catch (...) {
            ESP_LOGE(APP_TAG, "Unknown exception");
        }

        // 获取所有角色
        // const auto& roles = roleManager.getRoles();
        // for (const auto& role : roles) {
        //     std::string roleName = role.get()->getName();
        //     ESP_LOGI(APP_TAG, "测试json解析：%s", roleName.c_str()); 
        // }
        // 按ID查找特定角色
        // auto role = roleManager.findRoleById("sr_7");
        // 从SPIFFS文件系统加载
        // roleManager.loadFromFile("/spiffs/roles.json");
        // 从网络或其他来源加载
        // roleManager.loadFromJsonString(jsonString);
    });

    // 开启音频编码器(目前直接i2c读取麦克风数据)
    codec->Start();
    codec->EnableInput(false);
    // codec->EnableOutput(false);

}


// 主任务
void MyApp::MainLoop() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, SCHEDULE_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & SCHEDULE_EVENT) {
            std::unique_lock<std::mutex> lock(mutex_);
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void MyApp::addTask(std::function<void()> callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// 音视频输入输出任务
void MyApp::AudioLoop() {
    // 初始化包类型
    // pkg_type = 1;
    auto codec = Board::GetInstance().GetAudioCodec();
    while (true) {
        if (codec->input_enabled()) {
            OnAudioInput();
        }
        // if (codec->output_enabled()) {
            // OnAudioOutput();
        // }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}



void MyApp::OnAudioInput() {
    std::vector<int16_t> data;
    ReadAudio(data, 16000, GetFeedSize());

    // 直接处理pcm数据
    addTask([this, data = std::move(data)]() {
        // PCM数据转Base64
        std::string base64_audio = Base64Encoder::encode(
            reinterpret_cast<const unsigned char*>(data.data()),
            data.size() * sizeof(int16_t) // Total bytes = samples * 2 (since int16_t is 2 bytes)
        );

        Settings settings("agentId", true);
        std::string agentId = settings.GetString("agentId", "sr_7");
        //ESP_LOGI(APP_TAG, "当前agentId：%s",agentId.c_str());

        // 构造JSON消息
        std::string message = R"({
            "type": 2,
            "sub_type": )" + std::to_string(pkg_type) + R"(,
            "device": "google_baba",
            "data": {
                "ai_id": ")" + agentId + R"(",
                "bin_len": )" + std::to_string(data.size() * sizeof(int16_t)) + R"(,
                "bin_data": ")" + base64_audio + R"("
            }
        })";
        // ESP_LOGI(APP_TAG, "pcm数据json打印：%s",message.c_str());
        // std::string log = std::to_string(pkg_type);
        // ESP_LOGI(APP_TAG, "打印类型：%s",log.c_str());

        // 【测试用】
        // pkg_order++;
        // if(pkg_order > 30){
        //     pkg_type = 3;
        // }else{
        //     pkg_type = 2;
        // }
        // if(pkg_order < (30 + 3)){
        //     protocol_->SendPcmAudio(message); // 发送当前包
        // } 
        // if(pkg_type == 33){
        //     auto codec = Board::GetInstance().GetAudioCodec();
        //     codec->EnableInput(false);
        // }

        // 发送当前包
        if(pkg_order == 0){
            // std::string log = std::to_string(pkg_type);
            // ESP_LOGI(APP_TAG, "打印类型：%s",log.c_str());
            protocol_->SendPcmAudio(message); 
        }

        if(pkg_type == 1){
            pkg_type = 2;

        }else if(pkg_type == 3){
            pkg_order = 1;
            // auto codec = Board::GetInstance().GetAudioCodec();
            // codec->EnableOutput(true);
            auto codec = Board::GetInstance().GetAudioCodec();
            codec->EnableInput(false);
        }
        

    });

    // vTaskDelay(pdMS_TO_TICKS(30));
}

size_t MyApp::GetFeedSize() {
    auto codec = Board::GetInstance().GetAudioCodec();
    //常见的chunksize值可能是：16kHz采样率、16位单声道：通常为320字节（对应10ms音频数据：16000 samples/s * 0.01s * 2 bytes/sample = 320 bytes）
    //return afe_iface_->get_feed_chunksize(afe_data_) * codec_->input_channels();
    return 512; // 暂时
}


// 读取音频输入数据(参数：数据、采样率、数据块的大小)
void MyApp::ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples) {
    auto codec = Board::GetInstance().GetAudioCodec();
    // 检查当前编码器的输入采样率是否与所需的采样率不同
    // todo 先注释掉重采样代码
    if (codec->input_sample_rate() != sample_rate) {
        // // 根据目标采样率调整数据大小
        // data.resize(samples * codec->input_sample_rate() / sample_rate);
        // // 【从编码器读取数据】 如果失败则返回 **
        // if (!codec->InputData(data)) { 
        //     return;
        // }
        // if (codec->input_channels() == 2) { // 如果编码器的输入通道数为2（立体声）
        //     // 创建两个通道的向量，分别存储麦克风和参考通道的数据
        //     auto mic_channel = std::vector<int16_t>(data.size() / 2);
        //     auto reference_channel = std::vector<int16_t>(data.size() / 2);
        //     for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
        //         mic_channel[i] = data[j];  // 麦克风通道数据
        //         reference_channel[i] = data[j + 1]; // 参考通道数据
        //     }
        //      // 获取重采样后的输出样本数量
        //     auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
        //     auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
        //     // 处理重采样
        //     input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
        //     reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());

        //     // 调整最终数据大小以容纳重采样后的数据
        //     data.resize(resampled_mic.size() + resampled_reference.size());
        //     for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
        //         data[j] = resampled_mic[i];  // 将重采样后的数据合并回原始数据向量
        //         data[j + 1] = resampled_reference[i];  
        //     }
        // } else {
        //     // 处理单通道音频数据
        //     auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
        //     input_resampler_.Process(data.data(), data.size(), resampled.data());
        //     data = std::move(resampled); // 将重采样后的数据移动到原数据向量
        // }
    } else {
        // 如果采样率相同，调整数据大小并读取数据
        //ESP_LOGI(TAG, "Websocket 数据发送");
        data.resize(samples);
        if (!codec->InputData(data)) {
            return;
        }
    }
}

void MyApp::OnAudioOutput() {
    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    // 加锁互斥量
    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
    //     // Disable the output if there is no audio data for a long time
    //     // 长时间无音频数据关掉输出
    //     if (device_state_ == kDeviceStateIdle) {
    //         auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
    //         if (duration > max_silence_seconds) {
    //             codec->EnableOutput(false);
    //         }
    //     }
        return;
    }

    // if (device_state_ == kDeviceStateListening) {
    //     audio_decode_queue_.clear();
    //     return;
    // }
    // 从音频解码队列中获取前一个音频数据并移动到 opus 变量中
    auto opus = std::move(audio_decode_queue_.front());
    // 从队列中移除该音频数据
    audio_decode_queue_.pop_front();
    // 解锁互斥量，允许其他线程访问共享资源
    lock.unlock();

    background_task_->Schedule([this, codec, opus = std::move(opus)]() mutable {
        // if (aborted_) {
        //     return;
        // }

        std::vector<int16_t> pcm;
        // if (!opus_decoder_->Decode(std::move(opus), pcm)) {
        //     return;
        // }
        // Resample if the sample rate is different
        // 如果解码后的采样率与目标采样率不同，则进行重采样
        // if (opus_decoder_->sample_rate() != codec->output_sample_rate()) {
        //     int target_size = output_resampler_.GetOutputSamples(pcm.size()); // 计算重采样后的目标大小
        //     std::vector<int16_t> resampled(target_size);
        //     output_resampler_.Process(pcm.data(), pcm.size(), resampled.data()); // 进行重采样处理
        //     pcm = std::move(resampled); // 移动重采样后的数据到 pcm 向量中
        // }
        // 将解码后的 PCM 数据输出到 codec
        codec->OutputData(opus);

        // 更新最后输出时间
        // last_output_time_ = std::chrono::steady_clock::now();
    });
}


void MyApp::ResetDecoder() {
    // std::lock_guard<std::mutex> lock(mutex_);
    // opus_decoder_->ResetState();
    // audio_decode_queue_.clear();
    // last_output_time_ = std::chrono::steady_clock::now();
    
    // auto codec = Board::GetInstance().GetAudioCodec();
    // codec->EnableOutput(true);
}


void MyApp::SetDeviceState(DeviceState state) {
    if (device_state_ == state) {
        return;
    }
    auto previous_state = device_state_;
    device_state_ = state;
    // ESP_LOGI(APP_TAG, "STATE: %s", STATE_STRINGS[device_state_]);

    auto& board = Board::GetInstance();
    switch (state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
            break;
        case kDeviceStateConnecting:
            break;
        case kDeviceStateListening:
            break;
        case kDeviceStateSpeaking:
            ResetDecoder();
            break;
        default:
            // Do nothing
            break;
    }
}


void MyApp::StartListening() {
    // 初始音频包类型
    pkg_type = 1;
    pkg_order = 0;
    if (device_state_ == kDeviceStateIdle) {
        SetDeviceState(kDeviceStateListening);
        auto codec = Board::GetInstance().GetAudioCodec();
        codec->EnableInput(true);
    } else if (device_state_ == kDeviceStateSpeaking) {
        // 可以写中止打断会话
    }
}

void MyApp::StopListening() {
    // 结束音频包类型
    pkg_type = 3;
    if (device_state_ == kDeviceStateListening) {
        SetDeviceState(kDeviceStateIdle);
    }
}


std::vector<std::shared_ptr<AiRole>> MyApp::getAgentList(){
    return agentRoles;
}