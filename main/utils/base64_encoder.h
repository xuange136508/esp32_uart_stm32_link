#ifndef BASE64ENCODER_H
#define BASE64ENCODER_H

#include <string>

class Base64Encoder {
public:
    static std::string encode(const unsigned char *data, size_t len);
    static std::string decode(const std::string &encoded);
};

#endif // BASE64ENCODER_H
