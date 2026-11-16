#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include "Shader.h"
#include "../core/Camera.h"
#include "../ocean/GerstnerWave.h"

struct Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    float life;
    float maxLife;
    float size;
    float alpha;
};

class ParticleSystem {
public:
    static constexpr int MAX_PARTICLES = 1500;

    ParticleSystem();
    ~ParticleSystem();

    void init();
    void emitSplash(const glm::vec3 &origin, const glm::vec3 &baseVelocity, int count, float intensity = 1.0f);
    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight);
    void render(const Shader &shader, const Camera &camera, const glm::vec3 &sunColor) const;
    void clear();

    int getActiveCount() const { return m_activeCount; }

private:
    std::vector<Particle> m_particles;
    int m_activeCount;

    GLuint m_quadVAO;
    GLuint m_quadVBO;
    GLuint m_instanceVBO;

    // Instance buffer data format
    struct InstanceData {
        glm::vec3 pos;
        float size;
        float alpha;
    };
    mutable std::vector<InstanceData> m_instanceData;

    void setupBuffers();
};
