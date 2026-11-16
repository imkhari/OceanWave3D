#include "Wildlife.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdlib>
#include <algorithm>

static constexpr float PI = 3.14159265358979323846f;

WildlifeManager::WildlifeManager()
    : m_seagullCallTimer(4.0f) {}

void WildlifeManager::init() {
    buildSeagullMesh();

    // Initialize Seagulls (Two Flocks)
    m_seagulls.clear();
    // Flock 0: 8 seagulls circling the Lighthouse Islet
    for (int i = 0; i < 8; ++i) {
        Seagull gull;
        float angle = (static_cast<float>(i) / 8.0f) * 2.0f * PI;
        float radius = 28.0f + static_cast<float>(i % 3) * 6.0f;
        gull.position = glm::vec3(85.0f + std::cos(angle) * radius,
                                  20.0f + static_cast<float>(i % 4) * 3.0f,
                                  -105.0f + std::sin(angle) * radius);
        gull.velocity = glm::vec3(-std::sin(angle) * 8.0f, 0.0f, std::cos(angle) * 8.0f);
        gull.heading = std::atan2(gull.velocity.x, gull.velocity.z);
        gull.pitch = 0.0f;
        gull.roll = 0.25f;
        gull.flapPhase = static_cast<float>(rand() % 100) / 10.0f;
        gull.flockId = 0;
        m_seagulls.push_back(gull);
    }

    // Flock 1: 5 seagulls roaming around the open sea
    for (int i = 0; i < 5; ++i) {
        Seagull gull;
        float angle = (static_cast<float>(i) / 5.0f) * 2.0f * PI;
        gull.position = glm::vec3(std::cos(angle) * 45.0f,
                                  15.0f + static_cast<float>(i) * 2.5f,
                                  std::sin(angle) * 45.0f - 30.0f);
        gull.velocity = glm::vec3(-std::sin(angle) * 7.5f, 0.0f, std::cos(angle) * 7.5f);
        gull.heading = std::atan2(gull.velocity.x, gull.velocity.z);
        gull.pitch = 0.0f;
        gull.roll = 0.15f;
        gull.flapPhase = static_cast<float>(rand() % 100) / 10.0f;
        gull.flockId = 1;
        m_seagulls.push_back(gull);
    }
}

void WildlifeManager::update(float dt, const glm::vec3 &boatPos, AudioEngine &audio) {
    updateSeagulls(dt, boatPos, audio);
}

void WildlifeManager::updateSeagulls(float dt, const glm::vec3 &boatPos, AudioEngine &audio) {
    m_seagullCallTimer -= dt;

    float nearestDistToBoat = 9999.0f;
    glm::vec3 nearestGullPos(0.0f);

    for (size_t i = 0; i < m_seagulls.size(); ++i) {
        auto &gull = m_seagulls[i];

        // 1. Determine Target Center
        glm::vec3 flockTarget(0.0f);
        if (gull.flockId == 0) {
            // Circling the lighthouse island
            flockTarget = glm::vec3(85.0f, 22.0f, -105.0f);
        } else {
            // Circling above boat or roaming
            flockTarget = boatPos + glm::vec3(0.0f, 16.0f, 0.0f);
        }

        // 2. Craig Reynolds Boids Steering Rules
        glm::vec3 separation(0.0f);
        glm::vec3 alignment(0.0f);
        glm::vec3 cohesion(0.0f);
        int neighborCount = 0;

        for (size_t j = 0; j < m_seagulls.size(); ++j) {
            if (i == j || m_seagulls[j].flockId != gull.flockId) continue;
            glm::vec3 diff = gull.position - m_seagulls[j].position;
            float dist = glm::length(diff);
            if (dist < 4.0f && dist > 0.001f) {
                separation += (diff / (dist * dist)) * 5.0f;
            }
            if (dist < 22.0f) {
                alignment += m_seagulls[j].velocity;
                cohesion += m_seagulls[j].position;
                neighborCount++;
            }
        }

        glm::vec3 steerForce(0.0f);
        // Attraction to target center
        glm::vec3 toTarget = flockTarget - gull.position;
        float distTarget = glm::length(toTarget);
        if (distTarget > 0.1f) {
            // Tangential orbit velocity around target
            glm::vec3 radial = toTarget / distTarget;
            glm::vec3 tangent(-radial.z, 0.0f, radial.x); // Counter-clockwise circle
            steerForce += tangent * 7.0f + (radial * (distTarget - 30.0f)) * 0.35f;
            // Height holding
            steerForce.y += (flockTarget.y - gull.position.y) * 0.8f;
        }

        if (neighborCount > 0) {
            alignment = (alignment / static_cast<float>(neighborCount) - gull.velocity) * 0.6f;
            cohesion = (cohesion / static_cast<float>(neighborCount) - gull.position) * 0.25f;
            steerForce += alignment + cohesion;
        }
        steerForce += separation;

        // Apply steering
        gull.velocity += steerForce * dt;
        float speed = glm::length(gull.velocity);
        if (speed > 13.0f) gull.velocity = (gull.velocity / speed) * 13.0f;
        if (speed < 5.0f)  gull.velocity = (gull.velocity / speed) * 5.0f;

        gull.position += gull.velocity * dt;

        // Smooth flight orientation
        gull.heading = std::atan2(gull.velocity.x, gull.velocity.z);
        float forwardSpeed = std::max(0.1f, std::sqrt(gull.velocity.x * gull.velocity.x + gull.velocity.z * gull.velocity.z));
        gull.pitch = -std::atan2(gull.velocity.y, forwardSpeed);

        // Banking into turns
        float yawDelta = steerForce.x * gull.velocity.z - steerForce.z * gull.velocity.x;
        gull.roll = glm::clamp(-yawDelta * 0.08f, -0.65f, 0.65f);

        // Flapping animation: faster when climbing, glides when soaring
        float flapSpeed = (gull.velocity.y > 0.5f) ? 12.0f : ((gull.velocity.y < -0.5f) ? 3.0f : 7.0f);
        gull.flapPhase += dt * flapSpeed;

        float distBoat = glm::length(gull.position - boatPos);
        if (distBoat < nearestDistToBoat) {
            nearestDistToBoat = distBoat;
            nearestGullPos = gull.position;
        }
    }

    // Play periodic seagull call sound when close to seagulls
    if (nearestDistToBoat < 60.0f && m_seagullCallTimer <= 0.0f) {
        audio.playSeagullCall(nearestGullPos);
        m_seagullCallTimer = 5.5f + static_cast<float>(rand() % 40) / 10.0f; // 5.5s to 9.5s
    }
}

void WildlifeManager::render(const Shader &shader) const {
    shader.setInt("uMaterialID", 11);
    for (const auto &gull : m_seagulls) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), gull.position);
        model = glm::rotate(model, gull.heading, glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, gull.pitch, glm::vec3(1.0f, 0.0f, 0.0f));
        model = glm::rotate(model, gull.roll, glm::vec3(0.0f, 0.0f, 1.0f));
        shader.setMat4("model", model);
        m_seagullMesh.draw();
    }
}

// -----------------------------------------------------------------------------
// Procedural Seagull 3D Mesh
// -----------------------------------------------------------------------------

void WildlifeManager::buildSeagullMesh() {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;

    // Torso / Body (Streamlined fuselage: length 0.85m, width 0.22m, height 0.18m)
    Mesh::addCube(vertices, indices, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.20f, 0.16f, 0.65f));

    // Head (Y: +0.06m, Z: +0.38m)
    Mesh::addCube(vertices, indices, glm::vec3(0.0f, 0.06f, 0.38f), glm::vec3(0.14f, 0.14f, 0.18f));

    // Beak (Yellow tip: Z: +0.48m to +0.58m)
    Mesh::addCube(vertices, indices, glm::vec3(0.0f, 0.04f, 0.50f), glm::vec3(0.06f, 0.05f, 0.14f));

    // Tail Fan (Z: -0.35m to -0.55m)
    Mesh::addCube(vertices, indices, glm::vec3(0.0f, 0.02f, -0.42f), glm::vec3(0.18f, 0.03f, 0.22f));

    // Left Wing (X: -0.10m to -0.95m, span ~1.9m)
    Mesh::addCube(vertices, indices, glm::vec3(-0.52f, 0.03f, 0.05f), glm::vec3(0.85f, 0.03f, 0.28f));

    // Right Wing (X: +0.10m to +0.95m)
    Mesh::addCube(vertices, indices, glm::vec3(0.52f, 0.03f, 0.05f), glm::vec3(0.85f, 0.03f, 0.28f));

    m_seagullMesh.upload(vertices, indices);
}
