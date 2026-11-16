#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "Shader.h"
#include "../core/Camera.h"

class Sky {
public:
    Sky();
    ~Sky();

    void setTimeOfDay(float timeOfDay) { m_timeOfDay = timeOfDay; }
    float getTimeOfDay() const { return m_timeOfDay; }

    glm::vec3 getSunDirection() const;
    glm::vec3 getSunColor() const;
    glm::vec3 getMoonDirection() const;
    glm::vec3 getMoonColor() const;
    glm::vec3 getFogColor() const;
    glm::vec3 getSkyHorizonColor() const;
    glm::vec3 getSkyZenithColor() const;

    void render(const Shader &shader, const Camera &camera, float gameTime, int screenWidth, int screenHeight,
                float rainIntensity = 0.0f, const glm::vec3 &lightningColor = glm::vec3(0.0f));

private:
    GLuint m_vao;
    float m_timeOfDay; // 0.0 - 24.0 hours (default: 13.5h - bright sunny ocean)
};
