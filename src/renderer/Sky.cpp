#include "Sky.h"
#include <cmath>
#include <algorithm>

Sky::Sky() : m_vao(0), m_timeOfDay(13.5f) {
    glGenVertexArrays(1, &m_vao);
}

Sky::~Sky() {
    if (m_vao != 0) {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }
}

glm::vec3 Sky::getSunDirection() const {
    // 06:00 = Horizon dawn, 12:00 = Overhead noon, 18:00 = Horizon sunset, 00:00 = Midnight
    float angle = ((m_timeOfDay - 6.0f) / 12.0f) * 3.14159265f;
    float y = std::sin(angle);
    float x = -std::cos(angle) * 0.95f;
    float z = 0.25f;
    return glm::normalize(glm::vec3(x, y, z));
}

glm::vec3 Sky::getMoonDirection() const {
    return -getSunDirection();
}

glm::vec3 Sky::getSunColor() const {
    glm::vec3 dir = getSunDirection();
    float altitude = dir.y;

    if (altitude < -0.05f) {
        return glm::vec3(0.0f);
    }

    // Sunset/sunrise transition
    if (altitude < 0.25f) {
        float t = glm::clamp(altitude / 0.25f, 0.0f, 1.0f);
        glm::vec3 sunsetColor(1.0f, 0.45f, 0.12f);
        glm::vec3 noonColor(1.0f, 0.98f, 0.92f);
        return glm::mix(sunsetColor, noonColor, t) * (1.8f * glm::smoothstep(-0.05f, 0.12f, altitude));
    }

    return glm::vec3(1.0f, 0.98f, 0.92f) * 1.8f;
}

glm::vec3 Sky::getMoonColor() const {
    glm::vec3 dir = getMoonDirection();
    float altitude = dir.y;
    if (altitude <= 0.0f) return glm::vec3(0.0f);

    float t = glm::clamp(altitude / 0.3f, 0.0f, 1.0f);
    return glm::vec3(0.55f, 0.70f, 0.95f) * (0.85f * t);
}

glm::vec3 Sky::getFogColor() const {
    return getSkyHorizonColor();
}

glm::vec3 Sky::getSkyHorizonColor() const {
    float sunY = getSunDirection().y;

    if (sunY > 0.25f) {
        // Daytime: Bright azure blue ocean horizon
        return glm::vec3(0.62f, 0.78f, 0.94f);
    } else if (sunY > -0.05f) {
        // Sunset / Sunrise: Rich warm golden orange
        float t = glm::clamp((sunY + 0.05f) / 0.30f, 0.0f, 1.0f);
        glm::vec3 sunsetHorizon(0.96f, 0.48f, 0.22f);
        glm::vec3 dayHorizon(0.62f, 0.78f, 0.94f);
        return glm::mix(sunsetHorizon, dayHorizon, t);
    } else {
        // Night: Deep mystical indigo navy
        return glm::vec3(0.015f, 0.022f, 0.055f);
    }
}

glm::vec3 Sky::getSkyZenithColor() const {
    float sunY = getSunDirection().y;

    if (sunY > 0.25f) {
        // Daytime: Deep celestial blue
        return glm::vec3(0.12f, 0.35f, 0.78f);
    } else if (sunY > -0.05f) {
        // Sunset: Twilight purple
        float t = glm::clamp((sunY + 0.05f) / 0.30f, 0.0f, 1.0f);
        glm::vec3 sunsetZenith(0.06f, 0.08f, 0.24f);
        glm::vec3 dayZenith(0.12f, 0.35f, 0.78f);
        return glm::mix(sunsetZenith, dayZenith, t);
    } else {
        // Night: Starry black-blue
        return glm::vec3(0.003f, 0.005f, 0.015f);
    }
}

void Sky::render(const Shader &shader, const Camera &camera, float gameTime, int screenWidth, int screenHeight,
                 float rainIntensity, const glm::vec3 &lightningColor) {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    shader.use();

    float aspect = static_cast<float>(screenWidth) / static_cast<float>(screenHeight ? screenHeight : 1);
    glm::mat4 proj = camera.getProjectionMatrix(aspect);
    glm::mat4 view = camera.getViewMatrix();

    glm::mat4 invProj = glm::inverse(proj);
    glm::mat3 invViewR = glm::mat3(glm::inverse(view));

    glm::vec3 sunDir = getSunDirection();
    glm::vec3 sunCol = getSunColor();
    glm::vec3 moonDir = getMoonDirection();
    glm::vec3 moonCol = getMoonColor();

    shader.setMat4("uInvProj", invProj);
    shader.setMat3("uInvViewRot", invViewR);
    shader.setVec2("uResolution", static_cast<float>(screenWidth), static_cast<float>(screenHeight));
    shader.setVec3("uSunDir", sunDir);
    shader.setVec3("uSunColor", sunCol);
    shader.setVec3("uMoonDir", moonDir);
    shader.setVec3("uMoonColor", moonCol);
    shader.setVec3("uHorizonColor", getSkyHorizonColor());
    shader.setVec3("uZenithColor", getSkyZenithColor());
    shader.setFloat("uTimeOfDay", m_timeOfDay);
    shader.setFloat("time", gameTime);
    shader.setFloat("uRainIntensity", rainIntensity);
    shader.setVec3("uLightningColor", lightningColor);

    glBindVertexArray(m_vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}
