#include "function_type.h"
#include "http_server.h"
#include "settings.h"
#include "esp_sleep.h"
#include "board.h"
#include "MyApp.h"

#include <cstdio>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <esp_log.h>
#include <cJSON.h>

#define TAG "HttpServer"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

//嵌入式系统中使用编译时嵌入的HTML资源
// asm 表示使用GCC的汇编标签语法将这个C数组与链接器生成的符号关联
extern const char index_html_start[] asm("_binary_http_server_html_start");

HttpServer& HttpServer::GetInstance() {
    static HttpServer instance;
    return instance;
}

HttpServer::HttpServer()
{

}

HttpServer::~HttpServer()
{

}


void HttpServer::Start()
{
    StartWebServer();
}


std::string HttpServer::GetWebServerUrl()
{
    return "http://192.168.4.1";
}

// 开启web服务器
void HttpServer::StartWebServer()
{
    // 配置并启动HTTP服务器
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20; // 增加URI处理器的默认数量
    config.uri_match_fn = httpd_uri_match_wildcard; // 启用URI通配符匹配
    config.max_open_sockets = 3; // 设置了合适的最大连接数
    ESP_ERROR_CHECK(httpd_start(&server_, &config));

    // Register the index.html file
    // 注册index.html
    httpd_uri_t index_html = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = [](httpd_req_t *req) -> esp_err_t {
            // 发送预定义的HTML首页内容
            httpd_resp_send(req, index_html_start, strlen(index_html_start));
            return ESP_OK;
        },
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &index_html));


    // Register the reboot endpoint
    // 注册重启接口(POST /reboot)
    // postman测试post请求 （http://192.168.3.208/reboot）
    httpd_uri_t reboot = {
        .uri = "/reboot",
        .method = HTTP_POST,
        .handler = [](httpd_req_t *req) -> esp_err_t {
            auto* this_ = static_cast<HttpServer*>(req->user_ctx);
            
            // 设置响应头，防止浏览器缓存
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            // 发送响应
            std::string json_str = "{";
            json_str += "\"code\":" + std::to_string(200) + ",";
            json_str += "\"message\":\"success\"";
            json_str += "}";
            httpd_resp_send(req, json_str.c_str(), HTTPD_RESP_USE_STRLEN);
            
            // 创建一个延迟重启任务
            ESP_LOGI(TAG, "Rebooting...");
            xTaskCreate([](void *ctx) {
                // 等待200ms确保HTTP响应完全发送
                vTaskDelay(pdMS_TO_TICKS(200));
                // 停止Web服务器
                auto* self = static_cast<HttpServer*>(ctx);
                if (self->server_) {
                    httpd_stop(self->server_);
                }
                // 再等待100ms确保所有连接都已关闭
                vTaskDelay(pdMS_TO_TICKS(100));
                // 执行重启
                esp_restart();
            }, "reboot_task", 4096, this_, 5, NULL);
            
            return ESP_OK;
        },
        .user_ctx = this // 传递当前对象指针作为上下文
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &reboot));

    // 多功能接口，用于妈网硬件交互
    // (GET /multifunctionalIntl?funcIndex=X&&params=yy)
    // 实例： http://192.168.3.208/multifunctionalIntl?funcIndex=1&&params=skr_7
    httpd_uri_t multifunctionalIntl = {
        .uri = "/multifunctionalIntl",
        .method = HTTP_GET,
        .handler = [](httpd_req_t *req) -> esp_err_t {
            // 从查询字符串解析index参数
            std::string uri = req->uri;
            // auto pos = uri.find("?funcIndex=");
            // if (pos != std::string::npos) {
            //     int index = std::stoi(uri.substr(pos + 11));
            // }
            auto [funcIndex, params] = HttpServer::GetInstance().extractParams(req->uri);

            int code = 200;
            std::string message = "success";
            int id = 1;
            std::string json_str = "{";
            json_str += "\"code\":" + std::to_string(code) + ",";
            json_str += "\"message\":\"" + message + "\",";
            json_str += "\"data\":{";
            json_str += "\"id\":" + std::to_string(id) + ",";

            Settings agentSettings("agentId", true);
            Settings boardSettings("board", true);
            // int cur = boardSettings.GetInt("volume", 30); 

            // 调用功能
            FunctionType function = static_cast<FunctionType>(std::stoi(funcIndex)); 
            std::vector<std::shared_ptr<AiRole>> m_roles = MyApp::GetInstance().getAgentList();
            int cur = 10;
            switch (function) {
                case CONNECT:
                    break;
                case SWITCH_AGENT:
                    // agentSettings.SetString("agentId", params);
                    try {
                        // 找到匹配的智能体ID
                        for (size_t i = 0; i < m_roles.size(); ++i) {
                            if (m_roles[i]->getId() == params) {
                                int agentIndex = i;
                                agentSettings.SetInt("agentIndex", agentIndex);  // 设置智能体游标
                                agentSettings.SetString("agentId", params);  // 设置智能体ID
                                ESP_LOGI(TAG, "手机远程切换智能体：%s 游标为: %s", params.c_str(), std::to_string(agentIndex).c_str());

                                // 渲染智能体对应的动画
                                // Board::GetInstance().drawAgentImage(m_roles[i]->getImageUrl());
                                break;
                            }
                        }

                    } catch (const std::exception& e) {
                        ESP_LOGE(TAG, "http_server Exception: %s", e.what());
                    } catch (...) {
                        ESP_LOGE(TAG, "Unknown exception");
                    }    

                    break;
                case AUDIO_PLAY:
                    break;
                case GET_DEVICE_INFO:
                    break;
                case VOLUME_SET:
                    cur = std::stoi(params);
                    boardSettings.SetInt("volume", cur);
                    ESP_LOGI(TAG, "当前设置音量为：%d", cur); 
                    json_str += "\"volume\":\"" + std::to_string(cur) + "\",";
                    // cur += 10;
                    // if (cur >= 100) {
                    //     cur = 100; 
                    // }
                    // boardSettings.SetInt("volume", cur);
                    // ESP_LOGI(TAG, "当前音量为：%d", cur); 
                    // json_str += "\"volume\":\"" + std::to_string(cur) + "\",";
                    // cur -= 10; 
                    // if (cur <= 10) {
                    //     cur = 10; // 限制最小音量为10
                    // }
                    // boardSettings.SetInt("volume", cur); 
                    // ESP_LOGI(TAG, "当前音量为：%d", cur); 
                    // json_str += "\"volume\":\"" + std::to_string(cur) + "\",";
                    break;
                case SLEEP:
                    // 关闭 Wi-Fi
                    // esp_wifi_stop();
                    // esp_wifi_deinit();
                    // 进入深度睡眠 (没有直接 "关机"（完全断电）函数，因为 ESP32 的设计主要针对低功耗场景而非完全关机)
                    // 仅保留RTC内存标记为RTC_DATA_ATTR的数据, 能通过 定时器/外部信号/触摸等重新唤醒
                    //Board::GetInstance().TurnOffDisplay();
                    esp_deep_sleep_start();
                    return ESP_OK;
                    break;    
                case PRESS_DOWN:
                    // 开启音频输入
                    ESP_LOGI(TAG, "开始录音"); 
                    MyApp::GetInstance().StartListening();
                    return ESP_OK;
                    break;
                case PRESS_UP:
                    // 开启音频停止
                    ESP_LOGI(TAG, "结束录音"); 
                    MyApp::GetInstance().StopListening();
                    return ESP_OK;
                    break;
                default:
                    break;
            }
            json_str += "\"funcIndex\":\"" + funcIndex + "\"";
            json_str += "}}";

            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_str.c_str(), HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        },
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &multifunctionalIntl));

    //网页服务器已启动
    ESP_LOGI(TAG, "Web server started");
}

void  HttpServer::setAgentId(std::string setAgentId){
    agentId = setAgentId;
}


std::string HttpServer::getAgentId(){
    return agentId;
}

void HttpServer::Stop() {
    // 停止Web服务器
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
    }
}


std::pair<std::string, std::string> HttpServer::extractParams(const std::string& url){
    std::string funcIndex;
    std::string params;

    // 查找 funcIndex 的位置
    auto funcIndexPos = url.find("funcIndex=");
    if (funcIndexPos != std::string::npos) {
        // 找到 funcIndex 后提取值
        funcIndexPos += std::string("funcIndex=").length();
        auto endPos = url.find("&&", funcIndexPos);
        if (endPos == std::string::npos) {
            endPos = url.length(); // 到达字符串末尾
        }
        funcIndex = url.substr(funcIndexPos, endPos - funcIndexPos);
    }

    // 查找 params 的位置
    auto paramsPos = url.find("params=");
    if (paramsPos != std::string::npos) {
        // 找到 params 后提取值
        paramsPos += std::string("params=").length();
        auto endPos = url.length(); // 到达字符串末尾
        params = url.substr(paramsPos, endPos - paramsPos);
    }

    return {funcIndex, params};
}

