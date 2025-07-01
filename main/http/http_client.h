
#ifndef _HTTP_CLIENT_H
#define _HTTP_CLIENT_H

#include "esp_http_client.h"
#include "esp_log.h"
#include <string>

class HttpClient {
public:
    HttpClient();
    ~HttpClient();

    // GET 请求
    std::string get(const std::string& url);
    
    // POST 请求
    std::string post(const std::string& url, const std::string& post_data, 
                    const std::string& content_type = "application/json");

private:
    static esp_err_t event_handler(esp_http_client_event_t* evt);
    static std::string response_data;
};

#endif