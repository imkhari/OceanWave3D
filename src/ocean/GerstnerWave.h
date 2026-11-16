#pragma once

#include <glm/glm.hpp>
#include "../renderer/Shader.h"

struct WaveComponent {
    glm::vec2 direction;
    float amplitude;
    float wavelength;
    float omega;
    float steepness;
};

class GerstnerWaveModel {
public:
    static constexpr int N_WAVES = 16;
    static constexpr float GRAVITY = 9.8f;
    static constexpr float EPSILON = 0.02f;

    GerstnerWaveModel();

    void init(unsigned int seed = 1337);
    void uploadUniforms(const Shader &shader, float waveHeightScale) const;

    glm::vec3 evaluate(const glm::vec2 &pos, float time, float waveHeightScale = 1.0f) const;
    glm::vec3 calculateNormal(const glm::vec2 &pos, float time, float waveHeightScale = 1.0f) const;

    const WaveComponent &getComponent(int index) const { return m_waves[index]; }

private:
    WaveComponent m_waves[N_WAVES];
};
