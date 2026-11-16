#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>

class Shader {
public:
    Shader() : m_programID(0) {}
    ~Shader();

    bool loadFromFiles(const std::string &vertexPath, const std::string &fragmentPath);
    void use() const;

    GLuint getID() const { return m_programID; }

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec2(const std::string &name, const glm::vec2 &value) const;
    void setVec2(const std::string &name, float x, float y) const;
    void setVec3(const std::string &name, const glm::vec3 &value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    void setMat3(const std::string &name, const glm::mat3 &mat) const;
    void setMat4(const std::string &name, const glm::mat4 &mat) const;

private:
    GLuint m_programID;

    static std::string readFile(const std::string &path);
    static GLuint compileShader(GLenum type, const std::string &source, const std::string &debugPath);
    static bool checkCompileErrors(GLuint shader, const std::string &type, const std::string &path);
    static std::string resolveFilePath(const std::string &path);
};
