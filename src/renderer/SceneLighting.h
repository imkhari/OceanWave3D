#pragma once
#include "Shader.h"
#include <glm/gtc/matrix_transform.hpp>

// Depth-only directional shadow map. All resources die before the window/context.
class SunShadow {
    GLuint m_fbo = 0, m_depth = 0;
    int m_size = 0;
public:
    glm::mat4 matrix{1.0f};
    ~SunShadow() { if (m_depth) glDeleteTextures(1, &m_depth); if (m_fbo) glDeleteFramebuffers(1, &m_fbo); }
    bool resize(int size) {
        if (size == m_size) return true;
        if (!m_fbo) glGenFramebuffers(1, &m_fbo);
        if (!m_depth) glGenTextures(1, &m_depth);
        glBindTexture(GL_TEXTURE_2D, m_depth);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, size, size, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        const float border[] = {1,1,1,1};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depth, 0);
        glDrawBuffer(GL_NONE); glReadBuffer(GL_NONE);
        bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (ok) m_size = size;
        return ok;
    }
    void begin(const glm::vec3 &center, const glm::vec3 &light) {
        // Snap light-space XY to texels, keeping shadows stable as the boat moves.
        const float extent = 105.0f;
        glm::vec3 up = std::abs(light.y) > 0.97f ? glm::vec3(0,0,1) : glm::vec3(0,1,0);
        glm::mat4 lightView = glm::lookAt(center + light * 220.0f, center, up);
        glm::mat4 projection = glm::ortho(-extent, extent, -extent, extent, 1.0f, 440.0f);
        matrix = projection * lightView;
        glm::vec4 origin = matrix * glm::vec4(0,0,0,1);
        glm::vec2 snap = glm::round(glm::vec2(origin) * (m_size * 0.5f)) - glm::vec2(origin) * (m_size * 0.5f);
        projection[3][0] += snap.x * 2.0f / m_size;
        projection[3][1] += snap.y * 2.0f / m_size;
        matrix = projection * lightView;
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0,0,m_size,m_size);
        glEnable(GL_DEPTH_TEST); glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        glEnable(GL_POLYGON_OFFSET_FILL); glPolygonOffset(1.5f, 2.0f);
        glClear(GL_DEPTH_BUFFER_BIT);
    }
    void end() { glDisable(GL_POLYGON_OFFSET_FILL); glBindFramebuffer(GL_FRAMEBUFFER, 0); }
    void bind(const Shader &shader, bool enabled) const {
        shader.setMat4("uLightMatrix", matrix);
        shader.setInt("uShadows", enabled ? 1 : 0);
        glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, m_depth);
        shader.setInt("uShadowMap", 3);
        glActiveTexture(GL_TEXTURE0);
    }
};

inline void setSceneLighting(const Shader &shader, const glm::mat4 &view, const glm::mat4 &projection,
                            const glm::vec3 &eye, const glm::vec3 &sunDirection, const glm::vec3 &sunColor,
                            const glm::vec3 &moonDirection, const glm::vec3 &moonColor,
                            const glm::vec3 &horizon, float rain, const SunShadow &shadow, bool shadows) {
    shader.use();
    shader.setMat4("view", view); shader.setMat4("projection", projection);
    shader.setVec3("viewPos", eye); shader.setVec3("fogColor", horizon);
    shader.setVec3("uSunDir", sunDirection); shader.setVec3("uSunColor", sunColor);
    shader.setVec3("uMoonDir", moonDirection); shader.setVec3("uMoonColor", moonColor);
    shader.setVec3("uHorizonColor", horizon); shader.setFloat("uRainIntensity", rain);
    shadow.bind(shader, shadows);
}
