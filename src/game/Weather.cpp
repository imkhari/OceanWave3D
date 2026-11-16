#include "Weather.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdlib>
#include <algorithm>

static constexpr float PI = 3.14159265358979323846f;

WeatherSystem::WeatherSystem()
    : m_type(WeatherType::CLEAR),
      m_targetRainIntensity(0.0f),
      m_rainIntensity(0.0f),
      m_wetness(0.0f),
      m_wiperActive(false),
      m_wiperAngle(-1.05f),
      m_wiperPhase(0.0f),
      m_lightningFlash(0.0f),
      m_lightningTimer(5.0f),
      m_thunderDelayTimer(0.0f),
      m_thunderPending(false) {}

void WeatherSystem::init() {
    buildRainMesh();
}

void WeatherSystem::setWeatherType(WeatherType type) {
    m_type = type;
    if (type == WeatherType::CLEAR) {
        m_targetRainIntensity = 0.0f;
    } else if (type == WeatherType::OVERCAST) {
        m_targetRainIntensity = 0.0f;
    } else if (type == WeatherType::RAIN) {
        m_targetRainIntensity = 0.65f;
    } else if (type == WeatherType::STORM) {
        m_targetRainIntensity = 1.0f;
    }
}

void WeatherSystem::cycleWeather() {
    int next = (static_cast<int>(m_type) + 1) % 4;
    setWeatherType(static_cast<WeatherType>(next));
}

const char *WeatherSystem::getWeatherName() const {
    switch (m_type) {
        case WeatherType::CLEAR:    return "Nang dep / Quang dang (Clear)";
        case WeatherType::OVERCAST: return "Nhieu may / Am u (Overcast)";
        case WeatherType::RAIN:     return "Mua rao / Song gon (Rain)";
        case WeatherType::STORM:    return "Bao giong / Sam chop (Storm)";
    }
    return "Clear";
}

glm::vec3 WeatherSystem::getLightningColor() const {
    if (m_lightningFlash <= 0.02f) return glm::vec3(0.0f);
    return glm::vec3(2.4f, 2.6f, 3.2f) * m_lightningFlash;
}

void WeatherSystem::addWetness(float amount) {
    m_wetness = std::clamp(m_wetness + amount, 0.0f, 1.0f);
}

void WeatherSystem::setWetness(float w) {
    m_wetness = std::clamp(w, 0.0f, 1.0f);
}

void WeatherSystem::update(float dt, float time, const glm::vec3 &camPos, const glm::vec3 &camFront,
                           AudioEngine &audio, float boatSpeed) {
    // Smooth transition of rain intensity
    m_rainIntensity += (m_targetRainIntensity - m_rainIntensity) * std::min(dt * 2.0f, 1.0f);
    audio.setRainIntensity(m_rainIntensity);

    // Windshield droplet accumulation (Feature 1)
    if (m_rainIntensity > 0.05f) {
        m_wetness += m_rainIntensity * dt * 0.22f;
    }
    // High-speed bow wave spray onto camera/windshield
    if (boatSpeed > 4.5f) {
        m_wetness += (boatSpeed - 4.5f) * 0.012f * dt;
    }
    // Natural evaporation
    m_wetness = std::max(0.0f, m_wetness - dt * 0.022f);

    // Wiper blade simulation
    if (m_wiperActive) {
        m_wiperPhase += dt * 3.8f;
        m_wiperAngle = std::sin(m_wiperPhase) * 1.08f;
        // Wiper clears droplets from glass
        m_wetness = std::max(0.0f, m_wetness - dt * 0.22f);
    } else {
        // Return smoothly to parked resting angle (-1.05 rad)
        m_wiperAngle += (-1.05f - m_wiperAngle) * std::min(dt * 4.0f, 1.0f);
    }
    m_wetness = std::clamp(m_wetness, 0.0f, 1.0f);

    // Lightning simulation in STORM mode
    if (m_type == WeatherType::STORM) {
        m_lightningTimer -= dt;
        if (m_lightningTimer <= 0.0f) {
            m_lightningFlash = 1.0f; // Peak flash
            m_thunderDelayTimer = 0.35f + static_cast<float>(rand() % 10) / 10.0f; // 0.35s to 1.35s delay
            m_thunderPending = true;
            m_lightningTimer = 4.5f + static_cast<float>(rand() % 60) / 10.0f; // Next strike in 4.5s to 10.5s
        }

        if (m_thunderPending) {
            m_thunderDelayTimer -= dt;
            if (m_thunderDelayTimer <= 0.0f) {
                audio.playThunder(0.95f);
                m_thunderPending = false;
            }
        }
    }

    // Exponential decay of lightning flash
    if (m_lightningFlash > 0.01f) {
        m_lightningFlash *= std::exp(-dt * 9.0f);
    } else {
        m_lightningFlash = 0.0f;
    }

    // Dynamic physical rainfall animation
    if (m_rainIntensity > 0.02f) {
        float boxHeight = 24.0f;
        float boxRadius = 28.0f;
        glm::vec3 tilt(0.12f, -1.0f, 0.08f);

        for (int i = 0; i < RAIN_STREAK_COUNT; ++i) {
            m_streaks[i].pos.y -= dt * m_streaks[i].speed;
            if (m_streaks[i].pos.y < -boxHeight * 0.5f) {
                m_streaks[i].pos.y += boxHeight;
                m_streaks[i].pos.x = ((rand() % 2000) / 1000.0f - 1.0f) * boxRadius;
                m_streaks[i].pos.z = ((rand() % 2000) / 1000.0f - 1.0f) * boxRadius;
            }
            glm::vec3 top = m_streaks[i].pos;
            glm::vec3 bot = top + tilt * m_streaks[i].length;
            m_rainVertices[i * 2] = top;
            m_rainVertices[i * 2 + 1] = bot;
        }
        m_rainMesh.upload(m_rainVertices, m_rainIndices);
    }
}

void WeatherSystem::renderRain(const Shader &rainShader, const glm::mat4 &view, const glm::mat4 &proj,
                              const glm::vec3 &camPos, float dt) const {
    if (m_rainIntensity <= 0.02f) return;

    rainShader.use();
    rainShader.setMat4("view", view);
    rainShader.setMat4("projection", proj);

    // Follow camera with vertical wrapping
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(camPos.x, camPos.y - 1.0f, camPos.z));
    rainShader.setMat4("model", model);
    rainShader.setFloat("uIntensity", m_rainIntensity);
    rainShader.setVec3("uRainColor", glm::vec3(0.85f, 0.90f, 0.98f));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glLineWidth(1.0f);

    m_rainMesh.drawLines();

    glDepthMask(GL_TRUE);
}

// -----------------------------------------------------------------------------
// Rain Mesh (Dynamic falling particles)
// -----------------------------------------------------------------------------

void WeatherSystem::buildRainMesh() {
    m_streaks.resize(RAIN_STREAK_COUNT);
    m_rainVertices.resize(RAIN_STREAK_COUNT * 2);
    m_rainIndices.resize(RAIN_STREAK_COUNT * 2);

    float boxRadius = 28.0f;
    float boxHeight = 24.0f;

    for (int i = 0; i < RAIN_STREAK_COUNT; ++i) {
        float rx = ((rand() % 2000) / 1000.0f - 1.0f) * boxRadius;
        float rz = ((rand() % 2000) / 1000.0f - 1.0f) * boxRadius;
        float ry = ((rand() % 2000) / 1000.0f - 1.0f) * (boxHeight * 0.5f);

        m_streaks[i].pos = glm::vec3(rx, ry, rz);
        m_streaks[i].speed = 22.0f + (rand() % 100) / 100.0f * 12.0f; // 22 - 34 m/s fast natural rainfall
        m_streaks[i].length = 0.8f + (rand() % 100) / 100.0f * 0.6f;

        m_rainIndices[i * 2] = static_cast<unsigned int>(i * 2);
        m_rainIndices[i * 2 + 1] = static_cast<unsigned int>(i * 2 + 1);
    }
}
