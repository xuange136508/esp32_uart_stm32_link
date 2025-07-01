#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "button.h"
#include "myApp.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#include <driver/gpio.h>


#define TAG "CompactWifiBoardLCD"


#define AUDIO_INPUT_SAMPLE_RATE  16000
// #define AUDIO_OUTPUT_SAMPLE_RATE 16000 小智AI配置
#define AUDIO_OUTPUT_SAMPLE_RATE 8000  


// i2s 引脚（麦克风、喇叭外接）
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

// 按钮
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40 // key 1 音量+
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39 // key 2 音量-
#define AUDIO_BUTTON_GPIO       GPIO_NUM_1 // key 3 音频录入
#define AGENT_BUTTON_GPIO       GPIO_NUM_38 // key 4 切换智能体



class CompactWifiBoardLCD : public WifiBoard {
private:
    Button volume_up_button_;
    Button volume_down_button_;
    Button audio_button_;
    Button agent_button_;
 
    void InitializeButtons() {
        agent_button_.OnClick([this]() {
            try {
                // 找到匹配的智能体ID
                std::vector<std::shared_ptr<AiRole>> m_roles = MyApp::GetInstance().getAgentList();
                Settings agentSettings("agentId", true);
                int agentIndex = agentSettings.GetInt("agentIndex", 0);  
                if(agentIndex < 5){
                    agentIndex++;
                }else{
                    agentIndex = 0;
                }
                agentSettings.SetInt("agentIndex", agentIndex);  // 设置智能体游标
                std::shared_ptr<AiRole> role = m_roles.at(agentIndex);
                agentSettings.SetString("agentId", role.get()->getId());  // 设置智能体ID
                ESP_LOGI(APP_TAG, "按键切换智能体：%s 游标为: %s",role.get()->getId().c_str(), std::to_string(agentIndex).c_str());

                // 渲染智能体对应的动画
                // drawAgentImage(role.get()->getImageUrl());

            } catch (const std::exception& e) {
                ESP_LOGE(TAG, "board Exception: %s", e.what());
            } catch (...) {
                ESP_LOGE(TAG, "Unknown exception");
            }
        });
        audio_button_.OnPressDown([this]() {
            try {
                Settings agentSettings("agentId", true);
                int agentIndex = agentSettings.GetInt("agentIndex", 0);  
                std::vector<std::shared_ptr<AiRole>> m_roles = MyApp::GetInstance().getAgentList();
                std::shared_ptr<AiRole> role = m_roles.at(agentIndex);
                // drawAgentImage(role.get()->getHearImageUrl());

            } catch (...) {
                ESP_LOGE(TAG, "Unknown exception");
            }
            ESP_LOGI(TAG, "开始录音"); 
            MyApp::GetInstance().StartListening();
        });
        audio_button_.OnPressUp([this]() {
            ESP_LOGI(TAG, "结束录音"); 
            MyApp::GetInstance().StopListening();
        });
        volume_up_button_.OnClick([this]() {
            Settings boardSettings("board", true);
            int cur = boardSettings.GetInt("volume", 30); 
            cur += 10;
            if (cur >= 100) {
                cur = 100; 
            }
            boardSettings.SetInt("volume", cur);
            ESP_LOGI(TAG, "当前音量为：%d", cur); 
        });
        volume_down_button_.OnClick([this]() {
            Settings boardSettings("board", true);
            int cur = boardSettings.GetInt("volume", 30); 
            cur -= 10; 
            if (cur <= 10) {
                cur = 10; // 限制最小音量为10
            }
            boardSettings.SetInt("volume", cur); 
            ESP_LOGI(TAG, "当前音量为：%d", cur); 
        });
    }


public:
    CompactWifiBoardLCD():
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO),
        audio_button_(AUDIO_BUTTON_GPIO),
        agent_button_(AGENT_BUTTON_GPIO)  
        {
        // 初始化按钮
        InitializeButtons();
    }


    virtual AudioCodec* GetAudioCodec() override {

        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);

        return &audio_codec;
    }



};

DECLARE_BOARD(CompactWifiBoardLCD);
