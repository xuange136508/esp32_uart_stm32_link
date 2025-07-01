#include "base64_encoder.h"

#include <vector>
#include <algorithm>

// 测试解码
// std::string encoded = "hello world";
// const unsigned char* data = reinterpret_cast<const unsigned char*>(encoded.c_str());
// size_t len = encoded.length();
// std::string base64Encoded = Base64Encoder::encode(data, len);
// std::string base64Test = Base64Encoder::decode(base64Encoded);
// if (base64Test.empty()) {
//     ESP_LOGE(TAG, "解码失败");
// } else {
//     ESP_LOGE(TAG, "解码数据 ==> %s", base64Test.c_str());
// }

static std::string base64_encode(const unsigned char *data, size_t len) {
    const char *base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    size_t i = 0;
    while (i < len) {
        size_t chunk_size = std::min((size_t)3, len - i);
        
        for (size_t j = 0; j < 3; j++) {
            char_array_3[j] = (j < chunk_size) ? data[i + j] : 0;
        }

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (size_t j = 0; j < 4; j++) {
            if (j <= chunk_size) {  // 修正为 <=
                encoded.push_back(base64_chars[char_array_4[j]]);
            } else {
                encoded.push_back('=');
            }
        }

        i += chunk_size;
    }
    return encoded;
}

std::string Base64Encoder::encode(const unsigned char *data, size_t len) {
    return base64_encode(data, len);
}


static std::string base64_decode(const std::string &encoded) {
    const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<unsigned char> decoded;
    unsigned char char_array_4[4], char_array_3[3];
    size_t i = 0, in_len = encoded.size();
    
    while (i < in_len) {
        // 跳过非 Base64 字符（如换行符）
        if (base64_chars.find(encoded[i]) == std::string::npos) {
            i++;
            continue;
        }

        // 收集 4 个有效 Base64 字符（或到字符串末尾）
        size_t j = 0;
        while (j < 4 && i < in_len) {
            if (encoded[i] != '=') {
                char_array_4[j] = static_cast<unsigned char>(base64_chars.find(encoded[i]));
                j++;
            } else {
                // 遇到 '=' 填充符，标记缺失的字符
                char_array_4[j] = 0;
                j++;
            }
            i++;
        }

        // 转换 4 Base64 字符 → 3 字节
        if (j >= 2) {  // 至少 2 个有效字符才能解码
            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0x0F) << 4) + ((char_array_4[2] & 0x3C) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x03) << 6) + char_array_4[3];

            // 根据填充符数量调整输出长度
            size_t decoded_len = 3;
            if (i > 0 && encoded[i-1] == '=') {
                if (i >= 2 && encoded[i-2] == '=') {
                    decoded_len = 1;  // 两个 '='，只解码 1 字节
                } else {
                    decoded_len = 2;  // 一个 '='，解码 2 字节
                }
            }
            decoded.insert(decoded.end(), char_array_3, char_array_3 + decoded_len);
        }
    }

    return std::string(decoded.begin(), decoded.end());
}

std::string Base64Encoder::decode(const std::string &data) {
    return base64_decode(data);
}