#include "http_client.h"

std::string HttpClient::response_data;

HttpClient::HttpClient() {
    response_data.clear();
}

HttpClient::~HttpClient() {
    // 清理资源
}

esp_err_t HttpClient::event_handler(esp_http_client_event_t* evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (!esp_http_client_is_chunked_response(evt->client)) {
                response_data.append((char*)evt->data, evt->data_len);
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

std::string HttpClient::get(const std::string& url) {
    response_data.clear();
    
    esp_http_client_config_t config = {
        .url = url.c_str(),
        .event_handler = event_handler
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE("HTTP_CLIENT", "GET request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return response_data;
}

std::string HttpClient::post(const std::string& url, 
                           const std::string& post_data,
                           const std::string& content_type) {
    response_data.clear();

    esp_http_client_config_t config = {
        .url = url.c_str(),
        .method = HTTP_METHOD_POST,
        .event_handler = event_handler
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_post_field(client, post_data.c_str(), post_data.length());
    esp_http_client_set_header(client, "Content-Type", content_type.c_str());

    esp_err_t err = esp_http_client_perform(client);

    if (err != ESP_OK) {
        ESP_LOGE("HTTP_CLIENT", "POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return response_data;
}