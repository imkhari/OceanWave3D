#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "../ocean/GerstnerWave.h"
#include "../audio/AudioEngine.h"

enum class BuoyType {
    PORT,       // Red buoy - Port side of channel (Red flashing light)
    STARBOARD,  // Green buoy - Starboard side of channel (Green flashing light)
    DANGER      // Yellow & Black cardinal buoy - Warning near rocks (Amber quick strobe)
};

class Buoy {
public:
    Buoy(BuoyType type, const glm::vec2 &position);

    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight);
    void render(const Shader &shader, const Mesh &mesh) const;

    const glm::vec3 &getPosition() const { return m_currentPos; }
    BuoyType getType() const { return m_type; }
    bool isLightOn() const { return m_isLightOn; }
    glm::vec3 getLightColor() const;

private:
    BuoyType m_type;
    glm::vec2 m_anchorXZ;
    glm::vec3 m_currentPos;
    float m_pitch;
    float m_roll;
    float m_blinkTimer;
    bool m_isLightOn;
};

class BuoyManager {
public:
    BuoyManager();

    void init();
    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight,
                const glm::vec3 &boatPos, AudioEngine &audio);
    void render(const Shader &shader) const;

    const std::vector<Buoy> &getBuoys() const { return m_buoys; }
    float getDistanceToNearestBuoy(const glm::vec3 &boatPos) const;

private:
    std::vector<Buoy> m_buoys;
    Mesh m_portBuoyMesh;
    Mesh m_starboardBuoyMesh;
    Mesh m_dangerBuoyMesh;
    float m_bellCooldown;

    void buildPortBuoyMesh();
    void buildStarboardBuoyMesh();
    void buildDangerBuoyMesh();
};
