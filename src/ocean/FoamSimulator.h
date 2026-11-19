#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "../renderer/Shader.h"

class FoamSimulator {
public:
    FoamSimulator();
    ~FoamSimulator();

    bool init(int resolution = 512, float domainSize = 120.0f);
    void update(float dt, const glm::vec3 &boatPos, float boatSpeed, float boatHeading, float hullContact = 1.0f);

    int getResolution() const { return m_resolution; }
    GLuint getFoamTexture() const { return m_colorTextures[m_currentRead]; }
    glm::vec2 getDomainCenter() const { return m_domainCenter; }
    float getDomainSize() const { return m_domainSize; }

private:
    int m_resolution;
    float m_domainSize;
    glm::vec2 m_domainCenter;
    glm::vec2 m_prevCenter;

    GLuint m_fbo[2];
    GLuint m_colorTextures[2];
    int m_currentRead;
    int m_currentWrite;

    GLuint m_quadVAO;
    GLuint m_quadVBO;
    Shader m_simShader;

    void setupBuffers();
    void cleanup();
};
