#ifndef _HTTP_SERVER_H_
#define _HTTP_SERVER_H_

#include <string>
#include <esp_http_server.h>

class HttpServer {
public:
    static HttpServer& GetInstance();
    void Start();
    void Stop();
    std::string GetWebServerUrl();
    std::pair<std::string, std::string> extractParams(const std::string& url);
    std::string getAgentId();
    void setAgentId(std::string agentId);

    // Delete copy constructor and assignment operator
    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

private:
    // Private constructor
    HttpServer();
    ~HttpServer();

    httpd_handle_t server_ = NULL;
    std::string agentId = "sr_7";

    void StartWebServer();
};

#endif // _HTTP_SERVER_H_
