// FunctionType.h
#ifndef FUNCTION_TYPE_H
#define FUNCTION_TYPE_H

enum FunctionType {
    CONNECT = 0,        // 测试连接
    SWITCH_AGENT = 1,   // 切换智能体
    AUDIO_PLAY = 2,     // 音频播放
    GET_DEVICE_INFO = 3,  // 获取设备信息
    VOLUME_SET = 4, // 音量设置
    SLEEP = 5, // 关机-深度睡眠
    PRESS_DOWN = 6, // 按下
    PRESS_UP = 7 //  弹起
};

#endif // FUNCTION_TYPE_H
