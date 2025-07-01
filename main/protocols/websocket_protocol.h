#ifndef _WEBSOCKET_PROTOCOL_H_
#define _WEBSOCKET_PROTOCOL_H_


#include "protocol.h"
#include <web_socket.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define WEBSOCKET_PROTOCOL_SERVER_HELLO_EVENT (1 << 0)

#define OPUS_FRAME_DURATION_MS 60
#define CONFIG_WEBSOCKET_URL "ws://toy-socket.mama.cn:80"
#define CONFIG_WEBSOCKET_ACCESS_TOKEN "txkOlEHu2br4jM3neH7kLdFoEuurg6zR"
#define CONFIG_WEBSOCKET_DEVICE "google_baba"


class WebsocketProtocol : public Protocol {
public:
    WebsocketProtocol();
    ~WebsocketProtocol();

    void Start() override;
    void SendAudio(const std::vector<uint8_t>& data) override;
    void SendPcmAudio(const std::string& text) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel() override;
    bool IsAudioChannelOpened() const override;
    // 测试妈网音频包发送
    void testSendAudioPack() override;
    // 初始化参数
    void initParams() override;

private:
    EventGroupHandle_t event_group_handle_;
    WebSocket* websocket_ = nullptr;

    void ParseServerHello(const cJSON* root);
    void SendText(const std::string& text) override;
};

#endif
