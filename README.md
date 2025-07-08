# ESP32-S3 智能语音交互系统

| 支持平台 | ESP32-S3 |
| -------- | -------- |

## 项目简介

这是一个基于ESP32-S3的智能语音交互系统，支持实时音频处理、STM32协处理器通信、WebSocket音频流传输以及智能设备控制。系统主要面向智能家居、胎教设备等应用场景。

## 主要功能特性

### 🎙️ 音频处理
- **实时音频采集**: 支持16kHz采样率的PCM音频输入
- **音频编码输出**: 支持多种音频格式的播放输出
- **Base64编码传输**: 音频数据经Base64编码通过WebSocket传输
- **音量控制**: 支持静音、调节音量、最大音量等操作

### 🔗 通信协议
- **WebSocket客户端**: 与服务器进行实时音频和JSON数据交互
- **STM32 UART通信**: 通过串口与STM32协处理器进行双向通信
- **HTTP服务器**: 提供Web管理界面和RESTful API
- **WiFi连接管理**: 支持WiFi配网和连接管理

### 🎵 播放控制
- **胎教音乐播放**: 支持远程胎教音乐播放控制
- **白噪音播放**: 提供白噪音播放功能
- **播放状态控制**: 支持播放、暂停、停止等操作
- **音频文件管理**: 支持多种音频文件格式

### 💡 设备控制
- **LED控制**: 支持呼吸灯和RGB灯的开关和亮度调节
- **传感器监控**: 监控光照、振动、触摸、温湿度等传感器数据
- **智能响应**: 根据传感器数据智能调整设备状态

### 🤖 AI智能体
- **多角色支持**: 支持多个AI智能体角色切换
- **JSON配置**: 通过JSON文件配置AI角色参数
- **动态加载**: 支持运行时动态加载和切换AI角色

### 📱 用户界面
- **LCD显示**: 支持LCD屏幕显示系统状态和信息
- **Web界面**: 提供Web管理界面进行远程控制
- **实时状态**: 实时显示设备运行状态和传感器数据

## 系统架构

```
┌─────────────────┐    UART    ┌─────────────────┐
│     STM32       │ ◄────────► │    ESP32-S3     │
│   协处理器      │            │     主控制器    │
│                 │            │                 │
│ • 传感器采集    │            │ • WiFi通信      │
│ • LED控制       │            │ • 音频处理      │
│ • 按键检测      │            │ • WebSocket     │
│ • 播放控制      │            │ • HTTP服务器    │
└─────────────────┘            └─────────────────┘
                                        │
                                        │ WebSocket/HTTP
                                        ▼
                               ┌─────────────────┐
                               │   云端服务器    │
                               │                 │
                               │ • AI语音处理    │
                               │ • 智能体服务    │
                               │ • 音频文件      │
                               └─────────────────┘
```

## 硬件要求

### ESP32-S3主板
- **芯片**: ESP32-S3 (双核Xtensa LX7)
- **内存**: 至少512KB SRAM
- **Flash**: 至少4MB
- **音频**: I2S音频编解码器
- **显示**: SPI LCD显示屏
- **网络**: WiFi 802.11 b/g/n

### STM32协处理器
- **通信**: UART2 (115200波特率)
- **引脚**: TX=GPIO17, RX=GPIO18
- **功能**: 传感器数据采集、LED控制、按键处理

### 外设设备
- **麦克风**: I2S数字麦克风
- **扬声器**: I2S音频输出
- **传感器**: 光照、振动、触摸、温湿度传感器
- **LED**: 呼吸灯、RGB灯

## 软件依赖

- **ESP-IDF**: v5.0+
- **FreeRTOS**: 实时操作系统
- **cJSON**: JSON解析库
- **WebSocket**: WebSocket客户端库
- **HTTP**: HTTP服务器组件

## 快速开始

### 1. 环境搭建
```bash
# 安装ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
source export.sh

# 克隆项目
git clone <project-repo>
cd uart_link
```

### 2. 配置项目
```bash
# 配置目标芯片
idf.py set-target esp32s3

# 配置项目参数
idf.py menuconfig
```

### 3. 编译和烧录
```bash
# 编译项目
idf.py build

# 烧录到设备
idf.py -p /dev/ttyUSB0 flash

# 监控日志输出
idf.py -p /dev/ttyUSB0 monitor
```

## 配置说明

### WiFi配置
在 `menuconfig` 中配置WiFi参数，或通过Web配网页面进行配置。

### 音频参数
- 采样率: 16kHz
- 位深度: 16bit
- 声道数: 单声道/立体声
- 编码格式: PCM

### STM32通信
- 波特率: 115200
- 数据位: 8
- 停止位: 1
- 校验位: 无

## API接口

### WebSocket消息格式

**音频数据上传**:
```json
{
    "type": 2,
    "sub_type": 1,
    "device": "google_baba",
    "data": {
        "ai_id": "sr_7",
        "bin_len": 1024,
        "bin_data": "base64_encoded_audio_data"
    }
}
```

**播放控制**:
```json
{
    "type": 8,
    "sub_type": 1,
    "device": "google_baba",
    "command_source": "stm32_command",
    "data": {
        "id": 1,
        "url": "/path/to/audio/file.pcm",
        "file_name": "胎教音乐"
    }
}
```

### STM32通信协议

**播放控制命令**:
```json
{
    "device": "STM32_PlayControl",
    "timestamp": 12345678,
    "playback": {
        "status": "play",
        "content_type": "prenatal_education",
        "command_source": "USART2"
    },
    "controls": {
        "breathing_led": true,
        "rgb_led": false
    }
}
```

**音量控制命令**:
```json
{
    "device": "STM32_PlayControl",
    "playback": {
        "status": "vol_up",
        "content_type": "volume_control"
    }
}
```

## 目录结构

```
uart_link/
├── main/                          # 主程序目录
│   ├── audio_codecs/              # 音频编解码器
│   ├── board/                     # 板级支持包
│   ├── http/                      # HTTP服务器和客户端
│   ├── protocols/                 # 通信协议实现
│   ├── utils/                     # 工具函数
│   ├── main.cc                    # 程序入口
│   ├── myApp.cc                   # 主应用逻辑
│   ├── stm32_comm.cc              # STM32通信模块
│   ├── wifi_manager.cc            # WiFi管理
│   └── settings.cc                # 配置管理
├── assets/                        # 资源文件
├── managed_components/            # 管理的组件
├── CMakeLists.txt                 # 构建配置
├── partitions.csv                 # 分区表
└── README.md                      # 项目说明
```

## 故障排除

### 常见问题

**1. 编译错误**
- 检查ESP-IDF版本是否为v5.0+
- 确认所有依赖组件已正确安装
- 清理构建缓存: `idf.py clean`

**2. 烧录失败**
- 检查串口连接和权限
- 尝试降低烧录波特率
- 确认开发板进入下载模式

**3. WiFi连接问题**
- 检查WiFi配置参数
- 确认路由器支持ESP32连接
- 查看WiFi信号强度

**4. STM32通信异常**
- 检查UART引脚连接
- 确认波特率设置正确
- 检查STM32端程序是否正常运行

**5. 音频问题**
- 检查I2S引脚连接
- 确认音频编解码器配置
- 调整音频采样率和缓冲区大小

## 技术支持

如有技术问题，请通过以下方式获取支持：

- 📧 邮件支持: [技术支持邮箱]
- 💬 在线讨论: [项目论坛/群组]
- 🐛 问题反馈: [GitHub Issues]
- 📖 技术文档: [在线文档链接]

## 开发计划

- [ ] 支持更多音频格式
- [ ] 添加语音识别本地处理
- [ ] 增强AI智能体功能
- [ ] 优化音频延迟和质量
- [ ] 支持OTA在线升级
- [ ] 添加更多传感器支持

## 许可证

本项目采用 [MIT License](LICENSE) 开源许可证。

---

**注意**: 本项目仅供学习和研究使用，商业使用请遵循相关许可证条款。
