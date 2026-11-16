#include "Buoy.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <limits>
#include <algorithm>

static constexpr float PI = 3.14159265358979323846f;

Buoy::Buoy(BuoyType type, const glm::vec2 &position)
    : m_type(type),
      m_anchorXZ(position),
      m_currentPos(position.x, 0.0f, position.y),
      m_pitch(0.0f),
      m_roll(0.0f),
      m_blinkTimer(static_cast<float>(rand() % 100) / 50.0f), // Random offset so buoys don't blink in unison
      m_isLightOn(true) {}

void Buoy::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight) {
    // 1. Hydrodynamic Bobbing & Trochoidal Drift: Follow ocean wave height & horizontal displacement
    glm::vec3 waveDisp = waves.evaluate(m_anchorXZ, time, waveHeight);
    m_currentPos.x = m_anchorXZ.x + waveDisp.x * 0.4f;
    m_currentPos.z = m_anchorXZ.y + waveDisp.z * 0.4f;
    m_currentPos.y += (waveDisp.y - m_currentPos.y) * std::min(dt * 8.0f, 1.0f);

    // 2. Wave Surface Tilt: Pitch and roll with ocean wave normal
    glm::vec3 normal = waves.calculateNormal(m_anchorXZ, time, waveHeight);
    float targetPitch = -normal.z * 0.55f;
    float targetRoll  =  normal.x * 0.55f;
    m_pitch += (targetPitch - m_pitch) * std::min(dt * 6.0f, 1.0f);
    m_roll  += (targetRoll - m_roll) * std::min(dt * 6.0f, 1.0f);

    // 3. Strobe Blinking Rhythm
    m_blinkTimer += dt;
    if (m_type == BuoyType::DANGER) {
        // Quick amber warning flash: 0.25s ON, 0.25s OFF
        m_isLightOn = (std::fmod(m_blinkTimer, 0.5f) < 0.25f);
    } else {
        // Port & Starboard flash: 0.75s ON, 0.75s OFF
        m_isLightOn = (std::fmod(m_blinkTimer, 1.5f) < 0.75f);
    }
}

glm::vec3 Buoy::getLightColor() const {
    if (m_type == BuoyType::PORT) {
        return glm::vec3(1.0f, 0.08f, 0.05f); // Red
    } else if (m_type == BuoyType::STARBOARD) {
        return glm::vec3(0.05f, 1.0f, 0.15f); // Green
    } else {
        return glm::vec3(1.0f, 0.85f, 0.05f); // Amber / Yellow
    }
}

void Buoy::render(const Shader &shader, const Mesh &mesh) const {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_currentPos);
    model = glm::rotate(model, m_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    model = glm::rotate(model, m_roll, glm::vec3(0.0f, 0.0f, 1.0f));
    shader.setMat4("model", model);

    int matID = (m_type == BuoyType::PORT) ? 7 : ((m_type == BuoyType::STARBOARD) ? 8 : 9);
    shader.setInt("uMaterialID", matID);
    shader.setInt("uBuoyLightOn", m_isLightOn ? 1 : 0);
    shader.setVec3("uBuoyLightColor", getLightColor());

    mesh.draw();
}

// -----------------------------------------------------------------------------
// BuoyManager Implementation
// -----------------------------------------------------------------------------

BuoyManager::BuoyManager() : m_bellCooldown(0.0f) {}

void BuoyManager::init() {
    buildPortBuoyMesh();
    buildStarboardBuoyMesh();
    buildDangerBuoyMesh();

    m_buoys.clear();
    // Layout a marked shipping channel from boat spawn towards the lighthouse
    m_buoys.emplace_back(BuoyType::STARBOARD, glm::vec2( 22.0f, -20.0f));
    m_buoys.emplace_back(BuoyType::PORT,      glm::vec2(-22.0f, -20.0f));
    m_buoys.emplace_back(BuoyType::STARBOARD, glm::vec2( 48.0f, -55.0f));
    m_buoys.emplace_back(BuoyType::PORT,      glm::vec2(  6.0f, -55.0f));
    m_buoys.emplace_back(BuoyType::DANGER,     glm::vec2( 72.0f, -88.0f)); // Near rocky islet
}

void BuoyManager::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight,
                         const glm::vec3 &boatPos, AudioEngine &audio) {
    m_bellCooldown -= dt;

    float nearestDist = std::numeric_limits<float>::max();
    glm::vec3 nearestBuoyPos(0.0f);

    for (auto &buoy : m_buoys) {
        buoy.update(dt, waves, time, waveHeight);

        float dist = glm::length(glm::vec2(boatPos.x - buoy.getPosition().x, boatPos.z - buoy.getPosition().z));
        if (dist < nearestDist) {
            nearestDist = dist;
            nearestBuoyPos = buoy.getPosition();
        }
    }

    // Play buoy bell chime when sailing close to a buoy (< 28m)
    if (nearestDist < 28.0f && m_bellCooldown <= 0.0f) {
        audio.playBuoyBell(nearestBuoyPos);
        m_bellCooldown = 3.8f; // Bell rings every ~3.8 seconds when in vicinity
    }
}

void BuoyManager::render(const Shader &shader) const {
    for (const auto &buoy : m_buoys) {
        if (buoy.getType() == BuoyType::PORT) {
            buoy.render(shader, m_portBuoyMesh);
        } else if (buoy.getType() == BuoyType::STARBOARD) {
            buoy.render(shader, m_starboardBuoyMesh);
        } else {
            buoy.render(shader, m_dangerBuoyMesh);
        }
    }
}

float BuoyManager::getDistanceToNearestBuoy(const glm::vec3 &boatPos) const {
    float nearestDist = std::numeric_limits<float>::max();
    for (const auto &buoy : m_buoys) {
        float dist = glm::length(glm::vec2(boatPos.x - buoy.getPosition().x, boatPos.z - buoy.getPosition().z));
        if (dist < nearestDist) {
            nearestDist = dist;
        }
    }
    return nearestDist;
}

// -----------------------------------------------------------------------------
// Buoy 3D Meshes
// -----------------------------------------------------------------------------

void BuoyManager::buildPortBuoyMesh() {
    // Red cylindrical can buoy
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 14;
    // Lower float collar cylinder: Y: -0.8m to +0.8m, radius 0.9m
    int baseIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.9f, -0.8f, std::sin(angle) * 0.9f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.9f,  0.8f, std::sin(angle) * 0.9f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = baseIdx + s * 2;
        int s1 = baseIdx + s * 2 + 1;
        int n0 = baseIdx + nextS * 2;
        int n1 = baseIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Top superstructure tower: Y: 0.8m to 2.2m, radius tapers 0.5m to 0.35m
    int towerIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.5f,  0.8f, std::sin(angle) * 0.5f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.35f, 2.2f, std::sin(angle) * 0.35f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = towerIdx + s * 2;
        int s1 = towerIdx + s * 2 + 1;
        int n0 = towerIdx + nextS * 2;
        int n1 = towerIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Strobe Lantern on peak (Y: 2.2m to 2.5m)
    int lanternIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.18f, 2.2f, std::sin(angle) * 0.18f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.18f, 2.5f, std::sin(angle) * 0.18f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = lanternIdx + s * 2;
        int s1 = lanternIdx + s * 2 + 1;
        int n0 = lanternIdx + nextS * 2;
        int n1 = lanternIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    m_portBuoyMesh.upload(vertices, indices);
}

void BuoyManager::buildStarboardBuoyMesh() {
    // Green conical buoy
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 14;
    // Lower cylinder: Y: -0.8m to +0.6m, radius 0.9m
    int baseIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.9f, -0.8f, std::sin(angle) * 0.9f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.9f,  0.6f, std::sin(angle) * 0.9f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = baseIdx + s * 2;
        int s1 = baseIdx + s * 2 + 1;
        int n0 = baseIdx + nextS * 2;
        int n1 = baseIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Conical top: Y: 0.6m to 2.3m, radius 0.85m tapers to 0.12m
    int coneIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.85f, 0.6f, std::sin(angle) * 0.85f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.12f, 2.3f, std::sin(angle) * 0.12f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = coneIdx + s * 2;
        int s1 = coneIdx + s * 2 + 1;
        int n0 = coneIdx + nextS * 2;
        int n1 = coneIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Lantern peak (Y: 2.3m to 2.55m)
    int lanternIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.16f, 2.3f, std::sin(angle) * 0.16f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.16f, 2.55f, std::sin(angle) * 0.16f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = lanternIdx + s * 2;
        int s1 = lanternIdx + s * 2 + 1;
        int n0 = lanternIdx + nextS * 2;
        int n1 = lanternIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    m_starboardBuoyMesh.upload(vertices, indices);
}

void BuoyManager::buildDangerBuoyMesh() {
    // Yellow & Black striped cardinal hazard buoy
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 14;
    // Lower cylinder: Y: -0.9m to +0.8m, radius 1.05m
    int baseIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 1.05f, -0.9f, std::sin(angle) * 1.05f));
        vertices.push_back(glm::vec3(std::cos(angle) * 1.05f,  0.8f, std::sin(angle) * 1.05f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = baseIdx + s * 2;
        int s1 = baseIdx + s * 2 + 1;
        int n0 = baseIdx + nextS * 2;
        int n1 = baseIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Double-cone top mark cage: Y: 0.8m to 2.4m
    int cageIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.65f, 0.8f, std::sin(angle) * 0.65f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.40f, 2.4f, std::sin(angle) * 0.40f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = cageIdx + s * 2;
        int s1 = cageIdx + s * 2 + 1;
        int n0 = cageIdx + nextS * 2;
        int n1 = cageIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // Top strobe lantern (Y: 2.4m to 2.7m)
    int lanternIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.20f, 2.4f, std::sin(angle) * 0.20f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.20f, 2.7f, std::sin(angle) * 0.20f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = lanternIdx + s * 2;
        int s1 = lanternIdx + s * 2 + 1;
        int n0 = lanternIdx + nextS * 2;
        int n1 = lanternIdx + nextS * 2 + 1;
        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    m_dangerBuoyMesh.upload(vertices, indices);
}
