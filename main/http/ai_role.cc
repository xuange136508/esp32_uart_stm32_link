#include "ai_role.h"
#include <esp_log.h>
#include <cJSON.h>
#include <fstream>
#include <sstream>

// 嵌入式数据声明
extern "C" {
    extern const uint8_t role_list_json_start[] asm("_binary_role_list_json_start");
    extern const uint8_t role_list_json_end[] asm("_binary_role_list_json_end");
}

static const char* TAG = "AiRoleManager";

// AiRole 实现
AiRole::AiRole(const std::string& id, 
               const std::string& name,
               const std::string& img,
               const std::string& sayImg,
               const std::string& hearImg) :
    m_id(id),
    m_name(name),
    m_imageUrl(img),
    m_sayImageUrl(sayImg),
    m_hearImageUrl(hearImg) {}

const std::string& AiRole::getId() const { return m_id; }
const std::string& AiRole::getName() const { return m_name; }
const std::string& AiRole::getImageUrl() const { return m_imageUrl; }
const std::string& AiRole::getSayImageUrl() const { return m_sayImageUrl; }
const std::string& AiRole::getHearImageUrl() const { return m_hearImageUrl; }

// AiRoleManager 实现
AiRoleManager::AiRoleManager() {}

bool AiRoleManager::loadFromEmbeddedData() {
    std::string jsonStr(
        reinterpret_cast<const char*>(role_list_json_start),
        reinterpret_cast<const char*>(role_list_json_end)
    );
    return parseJson(jsonStr);
}

bool AiRoleManager::loadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        ESP_LOGE(TAG, "Failed to open file: %s", path.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return parseJson(buffer.str());
}

bool AiRoleManager::loadFromJsonString(const std::string& jsonStr) {
    return parseJson(jsonStr);
}

const std::vector<std::shared_ptr<AiRole>>& AiRoleManager::getRoles() const {
    return m_roles;
}

std::shared_ptr<AiRole> AiRoleManager::findRoleById(const std::string& id) const {
    for (const auto& role : m_roles) {
        if (role->getId() == id) {
            return role;
        }
    }
    return nullptr;
}

bool AiRoleManager::parseJson(const std::string& jsonStr) {
    m_roles.clear();
    
    cJSON* root = cJSON_Parse(jsonStr.c_str());
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error: %s", cJSON_GetErrorPtr());
        return false;
    }

    cJSON* aiList = cJSON_GetObjectItem(root, "ai_list");
    if (!aiList) {
        ESP_LOGE(TAG, "Missing 'ai_list' in JSON");
        cJSON_Delete(root);
        return false;
    }

    int arraySize = cJSON_GetArraySize(aiList);
    for (int i = 0; i < arraySize; i++) {
        cJSON* item = cJSON_GetArrayItem(aiList, i);
        
        auto role = std::make_shared<AiRole>(
            cJSON_GetObjectItem(item, "ai_id")->valuestring,
            cJSON_GetObjectItem(item, "name")->valuestring,
            cJSON_GetObjectItem(item, "ai_img")->valuestring,
            cJSON_GetObjectItem(item, "ai_say_img")->valuestring,
            cJSON_GetObjectItem(item, "ai_hear_img")->valuestring
        );
        
        m_roles.push_back(role);
        // ESP_LOGI(TAG, "Loaded role: %s", role->getName().c_str());
    }

    cJSON_Delete(root);
    return true;
}