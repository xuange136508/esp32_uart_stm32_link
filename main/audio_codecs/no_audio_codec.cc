#include "no_audio_codec.h"
#include "settings.h"

#include <esp_log.h>
#include <cmath>
#include <cstring>

#define TAG "NoAudioCodec"

/**
 * 实现了一个音频编解码器（NoAudioCodec），支持双工和单工模式，使用 I2S 接口与音频硬件进行通信。
 * (直接是PCM格式)
 */

NoAudioCodec::~NoAudioCodec() {
     // 禁用接收通道
    if (rx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
    // 禁用发送通道
    if (tx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
}


// 单工音频编解码器构造函数（单声道）
NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // 1. 首先配置I2S通道参数
    i2s_chan_config_t chan_cfg = {
        .id = (i2s_port_t)0,          // 使用I2S0
        .role = I2S_ROLE_MASTER,       // 主机模式
        .dma_desc_num = 6,             // DMA缓冲区描述符数量
        .dma_frame_num = 240,          // 每个描述符的帧数（建议240的整数倍）
        .auto_clear_after_cb = true,   // 自动清除DMA缓存
        .auto_clear_before_cb = false,
        .intr_priority = 0             // 中断优先级
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,    // 固定8kHz采样率（关键修改）
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // MCLK = 256*8kHz = 2.048MHz
            #ifdef I2S_HW_VERSION_2
                .ext_clk_freq_hz = 0,  // 不使用外部时钟
            #endif
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,  // 16位数据（关键修改）
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,  // 固定16位槽位（关键修改）
            .slot_mode = I2S_SLOT_MODE_MONO,             // 单声道
            .slot_mask = I2S_STD_SLOT_LEFT,              // 仅左声道有效
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,       // WS宽度=16bit（关键修改）
            .ws_pol = false,                            // WS低电平=左声道
            .bit_shift = true,                          // 数据右对齐（MSB优先）
            #ifdef I2S_HW_VERSION_2
                .left_align = false,    // 16bit数据通常右对齐
                .big_endian = false,    // 小端模式
                .bit_order_lsb = false  // MSB优先
            #endif
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   // MAX98357A不需要MCLK
            .bclk = spk_bclk,          // 位时钟引脚（如GPIO14）
            .ws = spk_ws,              // 声道选择引脚（如GPIO15）
            .dout = spk_dout,          // 数据输出引脚（如GPIO22）
            .din = I2S_GPIO_UNUSED,    // 输入未使用
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,     // MAX98357A通常不反转BCLK
                .ws_inv = false         // 确保WS极性正确
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));// 初始化扬声器通道

    // Create a new channel for MIC
    // 创建麦克风的 I2S 通道
    chan_cfg.id = (i2s_port_t)1; // 使用 I2S 端口 1
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));// 创建麦克风通道
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_; // 设置输入采样率
    std_cfg.gpio_cfg.bclk = mic_sck;// 设置麦克风的 BCLK 引脚
    std_cfg.gpio_cfg.ws = mic_ws;// 设置麦克风的 WS 引脚
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;// 麦克风数据输出引脚未使用
    std_cfg.gpio_cfg.din = mic_din; // 设置麦克风的数据输入引脚
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg)); // 初始化麦克风通道
    ESP_LOGI(TAG, "Simplex channels created");
}

// 另一个单工音频编解码器构造函数，支持插槽掩码
NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, i2s_std_slot_mask_t spk_slot_mask, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din, i2s_std_slot_mask_t mic_slot_mask){
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = (i2s_port_t)0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
			#ifdef   I2S_HW_VERSION_2
				.ext_clk_freq_hz = 0,
			#endif

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = spk_slot_mask,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #ifdef   I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

    // Create a new channel for MIC
    chan_cfg.id = (i2s_port_t)1;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_;
    std_cfg.slot_cfg.slot_mask = mic_slot_mask;
    std_cfg.gpio_cfg.bclk = mic_sck;
    std_cfg.gpio_cfg.ws = mic_ws;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = mic_din;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Simplex channels created");
}



// 写入音频数据到输出通道
int NoAudioCodec::Write(const int16_t* data, int samples) {
    Settings boardSettings("board", false);
    int volume_ = boardSettings.GetInt("volume", 30); 

    // 直接使用16位缓冲区（MAX98357A支持16位数据）
    std::vector<int16_t> buffer(samples);
    // 音量调节：output_volume_范围0-100 → 转换为0-32767的增益
    float volume_scale = powf(static_cast<float>(volume_) / 100.0f, 2.0f);
    for (int i = 0; i < samples; i++) {
        // 直接对16位数据进行音量调节（避免32位转换）
        int32_t temp = static_cast<int32_t>(data[i]) * volume_scale;
        
        // 限制在16位范围内
        if (temp > INT16_MAX) {
            buffer[i] = INT16_MAX;
        } else if (temp < INT16_MIN) {
            buffer[i] = INT16_MIN;
        } else {
            buffer[i] = static_cast<int16_t>(temp);
        }
    }

    size_t bytes_written;
    // 直接写入16位数据（无需转换）
    ESP_ERROR_CHECK(i2s_channel_write(
        tx_handle_, 
        buffer.data(), 
        samples * sizeof(int16_t),  // 数据大小 = 样本数 × 2字节
        &bytes_written, 
        portMAX_DELAY
    ));
    return bytes_written / sizeof(int16_t);  // 返回实际写入的样本数

}

// 从输入通道读取音频数据
int NoAudioCodec::Read(int16_t* dest, int samples) {
    size_t bytes_read;

    std::vector<int32_t> bit32_buffer(samples);
    // 从 I2S 通道读取数据
    if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed!");
        return 0;
    }

    samples = bytes_read / sizeof(int32_t);// 计算实际读取的样本数
    for (int i = 0; i < samples; i++) {
        // 将 32 位数据转换为 16 位并限制范围
        int32_t value = bit32_buffer[i] >> 12; // 右移 12 位
        dest[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value; // 限制范围
    }
    return samples;
}

