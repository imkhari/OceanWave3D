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
    void setBudget(int budget) { m_budget = glm::clamp(budget,1,MAX_PARTICLES); }
    void emitSplash(const glm::vec3 &origin, const glm::vec3 &baseVelocity, int count, float intensity = 1.0f, const glm::vec3 &outward = glm::vec3(0.0f));
    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight);
    void render(const Shader &shader, const Camera &camera, const glm::vec3 &sunColor, float aspect = 16.0f/9.0f) const;
    void clear();

    int getActiveCount() const { return m_activeCount; }

private:
    std::vector<Particle> m_particles;
    int m_activeCount;
    int m_budget = MAX_PARTICLES;

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
