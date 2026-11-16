#include "Lighthouse.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdlib>

static constexpr float PI = 3.14159265358979323846f;

Lighthouse::Lighthouse()
    : m_position(90.0f, 0.0f, -110.0f),
      m_rotationAngle(0.0f),
      m_beamIntensity(1.0f),
      m_topBeaconBlink(false) {}

void Lighthouse::init(const glm::vec3 &position) {
    m_position = position;
    buildIsletGeometry();
    buildTowerGeometry();
    buildLanternGeometry();
    buildBeamConeGeometry();
}

void Lighthouse::update(float dt, float time, float sunDirY) {
    // Rotate lighthouse beam: 0.65 rad/s (~9.6s per rotation)
    m_rotationAngle += dt * 0.65f;
    if (m_rotationAngle > 2.0f * PI) {
        m_rotationAngle -= 2.0f * PI;
    }

    // Lighthouse beam is fully bright at night & dusk (sunDirY < 0.1), fades during bright midday
    float nightness = glm::clamp(1.0f - (sunDirY - 0.05f) * 4.0f, 0.05f, 1.0f);
    m_beamIntensity = nightness;

    // Top red aviation beacon blinks at 1 Hz
    m_topBeaconBlink = (std::fmod(time, 1.0f) < 0.5f);
}

glm::vec3 Lighthouse::getLanternPosition() const {
    return m_position + glm::vec3(0.0f, 23.8f, 0.0f);
}

glm::vec3 Lighthouse::getBeamDirection() const {
    // Slightly pitched down towards the sea horizon
    return glm::normalize(glm::vec3(std::cos(m_rotationAngle), -0.065f, std::sin(m_rotationAngle)));
}

void Lighthouse::render(const Shader &shader) const {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
    shader.setMat4("model", model);

    // 1. Rocky Islet Base
    shader.setInt("uMaterialID", 0); // Rock
    m_isletMesh.draw();

    // 2. Lighthouse Striped Tower
    shader.setInt("uMaterialID", 1); // Red/White stripes
    m_towerMesh.draw();

    // 3. Lantern Room, Dome, and Glowing Core
    shader.setInt("uMaterialID", 2); // Lantern & Dome
    shader.setInt("uTopBeaconOn", m_topBeaconBlink ? 1 : 0);
    shader.setFloat("uBeamIntensity", m_beamIntensity);
    m_lanternMesh.draw();
}

void Lighthouse::renderBeam(const Shader &beamShader, const glm::mat4 &view, const glm::mat4 &proj) const {
    if (m_beamIntensity <= 0.05f) return;

    beamShader.use();
    beamShader.setMat4("view", view);
    beamShader.setMat4("projection", proj);

    glm::vec3 lanternPos = getLanternPosition();
    glm::vec3 beamDir = getBeamDirection();

    // Orient cone along beamDir
    glm::mat4 model = glm::translate(glm::mat4(1.0f), lanternPos);
    float yaw = std::atan2(beamDir.x, beamDir.z);
    float pitch = -std::asin(beamDir.y);
    model = glm::rotate(model, yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::rotate(model, pitch, glm::vec3(1.0f, 0.0f, 0.0f));

    beamShader.setMat4("model", model);
    beamShader.setFloat("uIntensity", m_beamIntensity);
    beamShader.setVec3("uBeamColor", glm::vec3(1.0f, 0.96f, 0.82f));

    // Additive blending for volumetric atmospheric light shaft
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);

    m_beamConeMesh.draw();

    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

// -----------------------------------------------------------------------------
// Procedural Geometry Generation
// -----------------------------------------------------------------------------

void Lighthouse::buildIsletGeometry() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 18;
    const int RINGS = 4;
    float ringRadii[RINGS] = { 16.0f, 13.5f, 9.5f, 6.0f };
    float ringHeights[RINGS] = { -5.0f, -0.5f, 2.2f, 3.8f };

    // Procedural jagged rock profile
    for (int r = 0; r < RINGS; ++r) {
        float baseR = ringRadii[r];
        float baseH = ringHeights[r];

        for (int s = 0; s < SEGS; ++s) {
            float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
            // Perturbation for natural craggy look
            float perturbR = 1.0f + 0.18f * std::sin(angle * 3.0f + r * 1.5f) + 0.12f * std::cos(angle * 5.0f);
            float perturbH = 0.35f * std::sin(angle * 4.0f + r * 0.8f);

            float x = std::cos(angle) * (baseR * perturbR);
            float z = std::sin(angle) * (baseR * perturbR);
            float y = baseH + perturbH;

            vertices.push_back(glm::vec3(x, y, z));
        }
    }

    // Connect rings with quads
    for (int r = 0; r < RINGS - 1; ++r) {
        int r0 = r * SEGS;
        int r1 = (r + 1) * SEGS;
        for (int s = 0; s < SEGS; ++s) {
            int nextS = (s + 1) % SEGS;
            indices.push_back(r0 + s);
            indices.push_back(r1 + s);
            indices.push_back(r0 + nextS);

            indices.push_back(r0 + nextS);
            indices.push_back(r1 + s);
            indices.push_back(r1 + nextS);
        }
    }

    // Top cap center vertex
    int centerIdx = static_cast<int>(vertices.size());
    vertices.push_back(glm::vec3(0.0f, ringHeights[RINGS - 1] + 0.2f, 0.0f));

    int topRing = (RINGS - 1) * SEGS;
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        indices.push_back(topRing + s);
        indices.push_back(centerIdx);
        indices.push_back(topRing + nextS);
    }

    m_isletMesh.upload(vertices, indices);
}

void Lighthouse::buildTowerGeometry() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 16;
    const int TIERS = 9; // Multiple height slices for striped texturing

    float tierHeights[TIERS] = { 3.8f, 5.5f, 8.5f, 11.5f, 14.5f, 17.5f, 20.0f, 21.8f, 22.2f };
    float tierRadii[TIERS]   = { 3.4f, 3.2f, 2.95f, 2.70f, 2.45f, 2.25f, 2.10f, 2.05f, 3.0f }; // Top flare for gallery deck

    for (int t = 0; t < TIERS; ++t) {
        float r = tierRadii[t];
        float y = tierHeights[t];

        for (int s = 0; s < SEGS; ++s) {
            float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
            vertices.push_back(glm::vec3(std::cos(angle) * r, y, std::sin(angle) * r));
        }
    }

    for (int t = 0; t < TIERS - 1; ++t) {
        int t0 = t * SEGS;
        int t1 = (t + 1) * SEGS;
        for (int s = 0; s < SEGS; ++s) {
            int nextS = (s + 1) % SEGS;
            indices.push_back(t0 + s);
            indices.push_back(t1 + s);
            indices.push_back(t0 + nextS);

            indices.push_back(t0 + nextS);
            indices.push_back(t1 + s);
            indices.push_back(t1 + nextS);
        }
    }

    // Gallery Safety Railing (simple circular ring mesh)
    int railBaseIdx = static_cast<int>(vertices.size());
    float railR = 2.95f;
    float railH = 23.1f;
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * railR, 22.2f, std::sin(angle) * railR));
        vertices.push_back(glm::vec3(std::cos(angle) * railR, railH, std::sin(angle) * railR));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = railBaseIdx + s * 2;
        int s1 = railBaseIdx + s * 2 + 1;
        int n0 = railBaseIdx + nextS * 2;
        int n1 = railBaseIdx + nextS * 2 + 1;

        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    m_towerMesh.upload(vertices, indices);
}

void Lighthouse::buildLanternGeometry() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 16;

    // 1. Glass lantern cylindrical cage (Y: 22.2 to 25.4, radius 1.95m)
    int cageBaseIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 1.95f, 22.2f, std::sin(angle) * 1.95f));
        vertices.push_back(glm::vec3(std::cos(angle) * 1.95f, 25.4f, std::sin(angle) * 1.95f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = cageBaseIdx + s * 2;
        int s1 = cageBaseIdx + s * 2 + 1;
        int n0 = cageBaseIdx + nextS * 2;
        int n1 = cageBaseIdx + nextS * 2 + 1;

        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    // 2. Copper Dome Roof (Y: 25.4 to 27.2)
    int domeBaseIdx = static_cast<int>(vertices.size());
    const int DOME_RINGS = 4;
    for (int r = 0; r <= DOME_RINGS; ++r) {
        float phi = (static_cast<float>(r) / DOME_RINGS) * (PI * 0.5f);
        float ringR = std::cos(phi) * 2.1f;
        float ringY = 25.4f + std::sin(phi) * 1.8f;

        for (int s = 0; s < SEGS; ++s) {
            float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
            vertices.push_back(glm::vec3(std::cos(angle) * ringR, ringY, std::sin(angle) * ringR));
        }
    }

    for (int r = 0; r < DOME_RINGS; ++r) {
        int r0 = domeBaseIdx + r * SEGS;
        int r1 = domeBaseIdx + (r + 1) * SEGS;
        for (int s = 0; s < SEGS; ++s) {
            int nextS = (s + 1) % SEGS;
            indices.push_back(r0 + s); indices.push_back(r1 + s); indices.push_back(r0 + nextS);
            indices.push_back(r0 + nextS); indices.push_back(r1 + s); indices.push_back(r1 + nextS);
        }
    }

    // 3. Lightning Spire (Y: 27.2 to 29.5)
    int spireIdx = static_cast<int>(vertices.size());
    vertices.push_back(glm::vec3(-0.08f, 27.2f, 0.0f));
    vertices.push_back(glm::vec3( 0.08f, 27.2f, 0.0f));
    vertices.push_back(glm::vec3( 0.0f,  29.5f, 0.0f));
    indices.push_back(spireIdx);
    indices.push_back(spireIdx + 1);
    indices.push_back(spireIdx + 2);

    // 4. Central Glowing Beacon Lens Core (inside lantern room)
    int coreIdx = static_cast<int>(vertices.size());
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * 0.75f, 23.2f, std::sin(angle) * 0.75f));
        vertices.push_back(glm::vec3(std::cos(angle) * 0.75f, 24.5f, std::sin(angle) * 0.75f));
    }
    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        int s0 = coreIdx + s * 2;
        int s1 = coreIdx + s * 2 + 1;
        int n0 = coreIdx + nextS * 2;
        int n1 = coreIdx + nextS * 2 + 1;

        indices.push_back(s0); indices.push_back(s1); indices.push_back(n0);
        indices.push_back(n0); indices.push_back(s1); indices.push_back(n1);
    }

    m_lanternMesh.upload(vertices, indices);
}

void Lighthouse::buildBeamConeGeometry() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    const int SEGS = 24;
    float len = 140.0f;
    float startR = 0.5f;
    float endR = 24.0f;

    // Origin apex ring (Z = 0)
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * startR, std::sin(angle) * startR, 0.0f));
    }

    // Outer far ring (Z = len)
    for (int s = 0; s < SEGS; ++s) {
        float angle = (static_cast<float>(s) / SEGS) * 2.0f * PI;
        vertices.push_back(glm::vec3(std::cos(angle) * endR, std::sin(angle) * endR, len));
    }

    for (int s = 0; s < SEGS; ++s) {
        int nextS = (s + 1) % SEGS;
        indices.push_back(s);
        indices.push_back(SEGS + s);
        indices.push_back(nextS);

        indices.push_back(nextS);
        indices.push_back(SEGS + s);
        indices.push_back(SEGS + nextS);
    }

    m_beamConeMesh.upload(vertices, indices);
}
