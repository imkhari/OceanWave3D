#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "../audio/AudioEngine.h"

enum class WeatherType {
    CLEAR,      // Troi quang dang, nang dep
    OVERCAST,   // Am u, nhieu may
    RAIN,       // Mua rao, gop song mua
    STORM       // Giong bao, cuong phong, sam chop
};

class WeatherSystem {
public:
    WeatherSystem();

    void init();
    void update(float dt, float time, const glm::vec3 &camPos, const glm::vec3 &camFront,
                AudioEngine &audio, float boatSpeed = 0.0f);
    void renderRain(const Shader &rainShader, const glm::mat4 &view, const glm::mat4 &proj,
                    const glm::vec3 &camPos, float dt) const;

    WeatherType getWeatherType() const { return m_type; }
    void setWeatherType(WeatherType type);

    float getRainIntensity() const { return m_rainIntensity; }
    float getLightningFlash() const { return m_lightningFlash; }
    bool isLightningActive() const { return m_lightningFlash > 0.05f; }
    glm::vec3 getLightningColor() const;

    // Windshield Droplets & Wiper (Feature 1)
    float getWetness() const { return m_wetness; }
    void addWetness(float amount);
    void setWetness(float w);
    bool isWiperActive() const { return m_wiperActive; }
    void setWiperActive(bool active) { m_wiperActive = active; }
    void toggleWiper() { m_wiperActive = !m_wiperActive; }
    float getWiperAngle() const { return m_wiperAngle; }

    // UI helpers
    void cycleWeather();
    void setQuality(int quality) { m_quality = glm::clamp(quality,0,2); }
    const char *getWeatherName() const;

private:
    int m_quality = 1;
    WeatherType m_type;
    float m_targetRainIntensity;
    float m_rainIntensity;

    // Windshield Droplets & Wiper
    float m_wetness;
    bool  m_wiperActive;
    float m_wiperAngle;
    float m_wiperPhase;

    // Lightning simulation
    float m_lightningFlash;
    float m_lightningTimer;
    float m_thunderDelayTimer;
    bool m_thunderPending;

    // Rain Particle Streaks (Dynamic physics rainfall)
    struct RainStreak {
        glm::vec3 pos;
        float speed;
        float length;
    };
    Mesh m_rainMesh;
    static constexpr int RAIN_STREAK_COUNT = 1500;
    std::vector<RainStreak> m_streaks;
    std::vector<glm::vec3> m_rainVertices;
    std::vector<unsigned int> m_rainIndices;

    void buildRainMesh();
};
