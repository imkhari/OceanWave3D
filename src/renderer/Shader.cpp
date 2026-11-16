#include "Shader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

std::string Shader::resolveFilePath(const std::string &path) {
    std::ifstream f(path);
    if (f.good()) return path;

    std::string fallback1 = "../" + path;
    std::ifstream f1(fallback1);
    if (f1.good()) return fallback1;

    std::string fallback2 = "../../" + path;
    std::ifstream f2(fallback2);
    if (f2.good()) return fallback2;

    return path;
}

std::string Shader::readFile(const std::string &path) {
    std::string resolved = resolveFilePath(path);
    std::ifstream file(resolved);
    if (!file.is_open()) {
        std::cerr << "[Shader ERROR] Failed to open file: " << path 
                  << " (Resolved: " << resolved << ")\n";
        return "";
    }
    std::stringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

GLuint Shader::compileShader(GLenum type, const std::string &source, const std::string &debugPath) {
    GLuint shader = glCreateShader(type);
    const char *src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    std::string typeStr = (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";
    if (!checkCompileErrors(shader, typeStr, debugPath)) {
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool Shader::checkCompileErrors(GLuint shader, const std::string &type, const std::string &path) {
    GLint success = 0;
    GLchar infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
            std::cerr << "[Shader Compile ERROR] in " << type << " shader (" << path << "):\n"
                      << infoLog << "\n";
            return false;
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
            std::cerr << "[Shader Link ERROR] for program:\n" << infoLog << "\n";
            return false;
        }
    }
    return true;
}

Shader::~Shader() {
    if (m_programID != 0) {
        glDeleteProgram(m_programID);
        m_programID = 0;
    }
}

bool Shader::loadFromFiles(const std::string &vertexPath, const std::string &fragmentPath) {
    if (m_programID != 0) {
        glDeleteProgram(m_programID);
        m_programID = 0;
    }

    std::string vSource = readFile(vertexPath);
    std::string fSource = readFile(fragmentPath);

    if (vSource.empty() || fSource.empty()) {
        return false;
    }

    GLuint vShader = compileShader(GL_VERTEX_SHADER, vSource, vertexPath);
    if (vShader == 0) return false;

    GLuint fShader = compileShader(GL_FRAGMENT_SHADER, fSource, fragmentPath);
    if (fShader == 0) {
        glDeleteShader(vShader);
        return false;
    }

    m_programID = glCreateProgram();
    glAttachShader(m_programID, vShader);
    glAttachShader(m_programID, fShader);
    glLinkProgram(m_programID);

    bool linkOk = checkCompileErrors(m_programID, "PROGRAM", vertexPath + " & " + fragmentPath);

    glDeleteShader(vShader);
    glDeleteShader(fShader);

    if (!linkOk) {
        glDeleteProgram(m_programID);
        m_programID = 0;
        return false;
    }

    return true;
}

void Shader::use() const {
    if (m_programID != 0) {
        glUseProgram(m_programID);
    }
}

void Shader::setBool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(m_programID, name.c_str()), (int)value);
}

void Shader::setInt(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(m_programID, name.c_str()), value);
}

void Shader::setFloat(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(m_programID, name.c_str()), value);
}

void Shader::setVec2(const std::string &name, const glm::vec2 &value) const {
    glUniform2fv(glGetUniformLocation(m_programID, name.c_str()), 1, &value[0]);
}

void Shader::setVec2(const std::string &name, float x, float y) const {
    glUniform2f(glGetUniformLocation(m_programID, name.c_str()), x, y);
}

void Shader::setVec3(const std::string &name, const glm::vec3 &value) const {
    glUniform3fv(glGetUniformLocation(m_programID, name.c_str()), 1, &value[0]);
}

void Shader::setVec3(const std::string &name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(m_programID, name.c_str()), x, y, z);
}

void Shader::setMat3(const std::string &name, const glm::mat3 &mat) const {
    glUniformMatrix3fv(glGetUniformLocation(m_programID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Shader::setMat4(const std::string &name, const glm::mat4 &mat) const {
    glUniformMatrix4fv(glGetUniformLocation(m_programID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}
