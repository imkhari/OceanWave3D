#include "ParticleSystem.h"
#include <cstdlib>
#include <cmath>
#include <algorithm>

ParticleSystem::ParticleSystem()
    : m_activeCount(0), m_quadVAO(0), m_quadVBO(0), m_instanceVBO(0) {
    m_particles.resize(MAX_PARTICLES);
    m_instanceData.reserve(MAX_PARTICLES);
}

ParticleSystem::~ParticleSystem() {
    if (m_instanceVBO != 0) glDeleteBuffers(1, &m_instanceVBO);
    if (m_quadVBO != 0) glDeleteBuffers(1, &m_quadVBO);
    if (m_quadVAO != 0) glDeleteVertexArrays(1, &m_quadVAO);
}

void ParticleSystem::init() {
    setupBuffers();
    clear();
}

void ParticleSystem::clear() {
    m_activeCount = 0;
    for (auto &p : m_particles) {
        p.life = 0.0f;
    }
}

void ParticleSystem::setupBuffers() {
    float quadVertices[] = {
        -0.5f, -0.5f, 0.0f,
         0.5f, -0.5f, 0.0f,
         0.5f,  0.5f, 0.0f,
        -0.5f, -0.5f, 0.0f,
         0.5f,  0.5f, 0.0f,
        -0.5f,  0.5f, 0.0f
    };

    glGenVertexArrays(1, &m_quadVAO);
    glGenBuffers(1, &m_quadVBO);
    glGenBuffers(1, &m_instanceVBO);

    glBindVertexArray(m_quadVAO);

    // Quad geometry buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Dynamic instance buffer
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, MAX_PARTICLES * sizeof(InstanceData), nullptr, GL_DYNAMIC_DRAW);

    // Location 1: Position (vec3)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void *)offsetof(InstanceData, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    // Location 2: Size (float)
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void *)offsetof(InstanceData, size));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    // Location 3: Alpha (float)
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void *)offsetof(InstanceData, alpha));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glBindVertexArray(0);
}

void ParticleSystem::emitSplash(const glm::vec3 &origin, const glm::vec3 &baseVelocity, int count, float intensity) {
    int emitted = 0;
    for (auto &p : m_particles) {
        if (emitted >= count) break;
        if (p.life > 0.0f) continue;

        p.position = origin;

        // Random fan spray laterally and upward
        float randX = ((std::rand() / static_cast<float>(RAND_MAX)) - 0.5f) * 2.0f;
        float randY = (std::rand() / static_cast<float>(RAND_MAX)) * 1.5f + 0.8f;
        float randZ = ((std::rand() / static_cast<float>(RAND_MAX)) - 0.5f) * 1.2f;

        glm::vec3 sprayDir = glm::normalize(glm::vec3(randX, randY, randZ));
        float spraySpeed = (3.0f + (std::rand() / static_cast<float>(RAND_MAX)) * 6.0f) * intensity;

        p.velocity = baseVelocity * 0.4f + sprayDir * spraySpeed;
        p.maxLife = 0.6f + (std::rand() / static_cast<float>(RAND_MAX)) * 0.6f;
        p.life = p.maxLife;
        p.size = 0.25f + (std::rand() / static_cast<float>(RAND_MAX)) * 0.35f;
        p.alpha = 1.0f;

        emitted++;
    }
}

void ParticleSystem::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight) {
    m_activeCount = 0;
    constexpr float GRAVITY = 9.8f;

    for (auto &p : m_particles) {
        if (p.life <= 0.0f) continue;

        p.life -= dt;
        if (p.life <= 0.0f) {
            p.life = 0.0f;
            continue;
        }

        // Apply physics
        p.velocity.y -= GRAVITY * dt;
        p.velocity *= (1.0f - 0.4f * dt); // Air drag
        p.position += p.velocity * dt;

        // Kill particle if submerged under ocean surface
        glm::vec2 pos2D(p.position.x, p.position.z);
        float surfaceY = waves.evaluate(pos2D, time, waveHeight).y;
        if (p.position.y < surfaceY - 0.1f) {
            p.life = 0.0f;
            continue;
        }

        p.alpha = p.life / p.maxLife;
        p.size += 0.3f * dt; // Droplet expands as it disperses

        m_activeCount++;
    }
}

void ParticleSystem::render(const Shader &shader, const Camera &camera, const glm::vec3 &sunColor) const {
    if (m_activeCount == 0) return;

    m_instanceData.clear();
    for (const auto &p : m_particles) {
        if (p.life > 0.0f) {
            m_instanceData.push_back({ p.position, p.size, p.alpha });
        }
    }

    if (m_instanceData.empty()) return;

    // Update instance VBO with active particle data
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, m_instanceData.size() * sizeof(InstanceData), m_instanceData.data());

    // Camera basis vectors for billboarding
    glm::mat4 view = camera.getViewMatrix();
    glm::vec3 camRight(view[0][0], view[1][0], view[2][0]);
    glm::vec3 camUp(view[0][1], view[1][1], view[2][1]);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE); // Particles don't write to depth buffer

    shader.use();
    shader.setMat4("view", view);
    shader.setMat4("projection", camera.getProjectionMatrix(16.0f / 9.0f));
    shader.setVec3("camRight", camRight);
    shader.setVec3("camUp", camUp);
    shader.setVec3("uSunColor", sunColor);

    glBindVertexArray(m_quadVAO);
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(m_instanceData.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}
