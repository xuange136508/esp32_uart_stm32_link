#include "websocket_protocol.h"
#include "system_info.h"
#include <cstring>
#include <cJSON.h>
#include <esp_log.h>
#include <arpa/inet.h>
#include <tcp_transport.h>
#include <tls_transport.h>
#include "board.h"

#define TAG "WS"

WebsocketProtocol::WebsocketProtocol() {
    event_group_handle_ = xEventGroupCreate();
}

WebsocketProtocol::~WebsocketProtocol() {
    if (websocket_ != nullptr) {
        delete websocket_;
    }
    vEventGroupDelete(event_group_handle_);
}

void WebsocketProtocol::Start() {
}

void WebsocketProtocol::SendAudio(const std::vector<uint8_t>& data) {
    if (websocket_ == nullptr) {
        return;
    }

    websocket_->Send(data.data(), data.size(), true);
}

void WebsocketProtocol::SendPcmAudio(const std::string& text) {
    if (websocket_ == nullptr) {
        return;
    }
    if (!websocket_->Send(text)) {
        ESP_LOGE(TAG, "Failed to send text: %s", text.c_str());
        //SetError(Lang::Strings::SERVER_ERROR);
    }
}


void WebsocketProtocol::SendText(const std::string& text) {
    if (websocket_ == nullptr) {
        return;
    }

    if (!websocket_->Send(text)) {
        ESP_LOGE(TAG, "Failed to send text: %s", text.c_str());
        //SetError(Lang::Strings::SERVER_ERROR);
    }
}

bool WebsocketProtocol::IsAudioChannelOpened() const {
    return websocket_ != nullptr && websocket_->IsConnected() && !error_occurred_ && !IsTimeout();
}

void WebsocketProtocol::CloseAudioChannel() {
    if (websocket_ != nullptr) {
        delete websocket_;
        websocket_ = nullptr;
    }
}

bool WebsocketProtocol::OpenAudioChannel() {
    if (websocket_ != nullptr) {
        delete websocket_;
    }

    error_occurred_ = false;
    std::string url = CONFIG_WEBSOCKET_URL;
    std::string token = std::string(CONFIG_WEBSOCKET_ACCESS_TOKEN); 
    std::string device = std::string(CONFIG_WEBSOCKET_DEVICE); 
    
    //websocket_ = Board::GetInstance().CreateWebSocket();
    if (url.find("wss://") == 0) {
        websocket_ =  new WebSocket(new TlsTransport());
    } else {
        websocket_ =  new WebSocket(new TcpTransport());
    }
    // 小智默认
    // websocket_->SetHeader("Protocol-Version", "1");
    // websocket_->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    //websocket_->SetHeader("Authorization", token.c_str());
    //websocket_->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    // 妈妈网
    websocket_->SetHeader("Device", device.c_str());
    websocket_->SetHeader("Authorization", token.c_str());

    ESP_LOGE(TAG,"ws地址==>: %s", url.c_str());
    ESP_LOGE(TAG,"token==>: %s", token.c_str());
    ESP_LOGE(TAG,"Device==>: %s", device.c_str());

    websocket_->OnData([this](const char* data, size_t len, bool binary) {
        if (binary) {
            // 如果是二进制数据
            if (on_incoming_audio_ != nullptr) {
                on_incoming_audio_(std::vector<uint8_t>((uint8_t*)data, (uint8_t*)data + len));
            }
        } else {
            // 直接打印返回
            ESP_LOGE(TAG, "websocket json 接收 ==>: %s", data);
        }
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    websocket_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "Websocket disconnected");
        // if (on_audio_channel_closed_ != nullptr) {
        //     on_audio_channel_closed_();
        // }
    });

    if (!websocket_->Connect(url.c_str())) {
        ESP_LOGE(TAG, "Failed to connect to websocket server");
        //SetError(Lang::Strings::SERVER_NOT_FOUND);
        return false;
    }

    // 发送妈妈网 【硬件登录上报】 指令
    std::string message = "{";
    message += "\"type\":1,";
    message += "\"sub_type\":1,";
    message += "\"device\":\"google_baba\",";
    message += "\"data\":{";
    message += "\"mac\":\"google_baba\",";
    message += "\"register\":true,";
    message += "\"ai_id\":\"ur_109842\",";
    message += "\"softvertion\":\"0.1\",";
    message += "\"pcbvertion\":\"0.1\"";
    message += "}}";
    websocket_->Send(message);

    return true;
}


void WebsocketProtocol::initParams() {
    if (websocket_ != nullptr) {
        delete websocket_;
    }

    error_occurred_ = false;
    std::string url = CONFIG_WEBSOCKET_URL;
    std::string token = std::string(CONFIG_WEBSOCKET_ACCESS_TOKEN); 
    std::string device = std::string(CONFIG_WEBSOCKET_DEVICE); 
    
    if (url.find("wss://") == 0) {
        websocket_ =  new WebSocket(new TlsTransport());
    } else {
        websocket_ =  new WebSocket(new TcpTransport());
    }
    websocket_->SetHeader("Device", device.c_str());
    websocket_->SetHeader("Authorization", token.c_str());

    ESP_LOGI(TAG,"ws地址==>: %s", url.c_str());
    // ESP_LOGE(TAG,"token==>: %s", token.c_str());
    // ESP_LOGE(TAG,"Device==>: %s", device.c_str());

    //设置buffer大小
    websocket_->SetReceiveBufferSize(4096);

    websocket_->OnData([this](const char* data, size_t len, bool binary) {
        if (binary) {
            // 如果是二进制数据
            if (on_incoming_audio_ != nullptr) {
                on_incoming_audio_(std::vector<uint8_t>((uint8_t*)data, (uint8_t*)data + len));
            }
        } else {
            // 直接打印返回
            //ESP_LOGE(TAG, "websocket json 接收 ==>: %s", data);

            auto root = cJSON_Parse(data);
            auto type = cJSON_GetObjectItem(root, "type");
            if (type != NULL) {
                if (on_incoming_json_ != nullptr) {
                    on_incoming_json_(root);
                }
            } else {
                ESP_LOGE(TAG, "websocket 解析报错");
            }
            cJSON_Delete(root);
        }
        last_incoming_time_ = std::chrono::steady_clock::now();
    });

    websocket_->OnDisconnected([this]() {
        ESP_LOGI(TAG, "Websocket disconnected");
    });

    if (!websocket_->Connect(url.c_str())) {
        ESP_LOGE(TAG, "Failed to connect to websocket server");
    }

    // 发送测试数据
    // std::string message = "{";
    // message += "\"type\":1,";
    // message += "\"sub_type\":1,";
    // message += "\"device\":\"google_baba\",";
    // message += "\"data\":{";
    // message += "\"mac\":\"google_baba\",";
    // message += "\"register\":true,";
    // message += "\"ai_id\":\"ur_109842\",";
    // message += "\"softvertion\":\"0.1\",";
    // message += "\"pcbvertion\":\"0.1\"";
    // message += "}}";
    // websocket_->Send(message);
}

// 测试妈网音频包发送
// json example：
// 上报音频开始 {\"type\": 2, \"sub_type\": 1, \"device\": \"fc012cca4dc4\", \"data\": {\"ai_id\": \"skr_5\", \"bin_num\": 1, \"bin_len\": 1368, \"bin_data\": \"\"}}"
// 上报音频分包 {\"type\": 2, \"sub_type\": 2, \"device\": \"fc012cca4dc4\", \"data\": {\"ai_id\": \"skr_5\", \"bin_num\": 2, \"bin_len\": 1368, \"bin_data\": \"\"}}"
// 上报音频结束 {\"type\": 2, \"sub_type\": 3, \"device\": \"fc012cca4dc4\", \"data\": {\"ai_id\": \"skr_5\", \"bin_num\": 124, \"bin_len\": 1368, \"bin_data\": \"\"}}"
void WebsocketProtocol::testSendAudioPack(){



}


