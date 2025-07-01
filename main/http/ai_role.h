#ifndef _AI_ROLE_H
#define _AI_ROLE_H

#include <string>
#include <vector>
#include <memory>

class AiRole {
public:
    AiRole(const std::string& id, 
           const std::string& name,
           const std::string& img,
           const std::string& sayImg,
           const std::string& hearImg);
    
    const std::string& getId() const;
    const std::string& getName() const;
    const std::string& getImageUrl() const;
    const std::string& getSayImageUrl() const;
    const std::string& getHearImageUrl() const;

private:
    std::string m_id;
    std::string m_name;
    std::string m_imageUrl;
    std::string m_sayImageUrl;
    std::string m_hearImageUrl;
};

class AiRoleManager {
public:
    AiRoleManager();
    
    bool loadFromEmbeddedData();
    bool loadFromFile(const std::string& path);
    bool loadFromJsonString(const std::string& jsonStr);
    
    const std::vector<std::shared_ptr<AiRole>>& getRoles() const;
    std::shared_ptr<AiRole> findRoleById(const std::string& id) const;

private:
    bool parseJson(const std::string& jsonStr);
    
    std::vector<std::shared_ptr<AiRole>> m_roles;
};

#endif // _AI_ROLE_H