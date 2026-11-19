#include "GerstnerWave.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <algorithm>

GerstnerWaveModel::GerstnerWaveModel() {
    init(1337);
}

void GerstnerWaveModel::init(unsigned int seed) {
    constexpr float PI2 = 6.28318530718f;

    // Prevailing wind direction ~ 25 degrees
    float windAngle = 0.45f;

    // Fixed realistic ocean wave spectrum (Phillips & JONSWAP empirical model)
    // 16 waves: rolling primary ocean swells + wind sea + subtle surface chop
    struct WavePreset {
        float wavelength;
        float amplitude;
        float angleOffsetDeg;
        float steepness;
    };

    const WavePreset presets[N_WAVES] = {
        // 1. Primary Swells (Majestic rolling ocean swells, aligned with wind)
        { 120.0f, 0.65f,   0.0f, 0.45f },
        {  92.0f, 0.48f,   7.0f, 0.48f },
        {  76.0f, 0.38f,  -8.0f, 0.50f },
        { 145.0f, 0.32f,  14.0f, 0.42f },

        // 2. Secondary Wind Sea (Harmonics with gentle angular dispersion)
        {  46.0f, 0.24f,  -6.0f, 0.55f },
        {  35.0f, 0.18f,  16.0f, 0.58f },
        {  27.0f, 0.14f, -20.0f, 0.55f },
        {  21.0f, 0.10f,  26.0f, 0.60f },
        {  17.0f, 0.08f, -14.0f, 0.60f },
        {  13.5f, 0.06f,   6.0f, 0.62f },

        // 3. Surface Detail & Micro-swells (Fine texture, low amplitude)
        {  10.2f, 0.045f, -32.0f, 0.65f },
        {   7.8f, 0.035f,  38.0f, 0.65f },
        {   6.0f, 0.025f, -44.0f, 0.70f },
        {   4.6f, 0.018f,  50.0f, 0.70f },
        {   3.4f, 0.012f, -56.0f, 0.70f },
        {   2.6f, 0.008f,  62.0f, 0.70f }
    };

    for (int i = 0; i < N_WAVES; ++i) {
        float rad = windAngle + presets[i].angleOffsetDeg * (3.14159265f / 180.0f);
        m_waves[i].direction = glm::normalize(glm::vec2(std::cos(rad), std::sin(rad)));
        m_waves[i].wavelength = presets[i].wavelength;
        m_waves[i].amplitude = presets[i].amplitude;
        m_waves[i].steepness = presets[i].steepness;

        float k = PI2 / presets[i].wavelength;
        m_waves[i].omega = std::sqrt(GRAVITY * k);
    }
}

void GerstnerWaveModel::uploadUniforms(const Shader &shader, float waveHeightScale) const {
    shader.setFloat("waveHeight", waveHeightScale);
    float slopeBound=0;
    for (const auto &w:m_waves) slopeBound += w.steepness*w.amplitude*waveHeightScale*6.2831853f/w.wavelength;
    float horizontalLimit=std::min(1.0f,0.85f/std::max(slopeBound,0.0001f));
    for (int i = 0; i < N_WAVES; ++i) {
        std::string prefix = "waveDir[" + std::to_string(i) + "]";
        shader.setVec2(prefix, m_waves[i].direction);

        prefix = "waveAmp[" + std::to_string(i) + "]";
        shader.setFloat(prefix, m_waves[i].amplitude);

        prefix = "waveWLen[" + std::to_string(i) + "]";
        shader.setFloat(prefix, m_waves[i].wavelength);

        prefix = "waveOmega[" + std::to_string(i) + "]";
        shader.setFloat(prefix, m_waves[i].omega);

        prefix = "steepness[" + std::to_string(i) + "]";
        shader.setFloat(prefix, m_waves[i].steepness * horizontalLimit);
    }
}

glm::vec3 GerstnerWaveModel::evaluate(const glm::vec2 &pos, float time, float waveHeightScale) const {
    constexpr float PI2 = 6.28318530718f;
    glm::vec3 p(pos.x, 0.0f, pos.y);
    float slopeBound=0;
    for(const auto &w:m_waves) slopeBound += w.steepness*w.amplitude*waveHeightScale*PI2/w.wavelength;
    float horizontalLimit=std::min(1.0f,0.85f/std::max(slopeBound,0.0001f));

    for (int i = 0; i < N_WAVES; ++i) {
        glm::vec2 dir = m_waves[i].direction;
        float k = PI2 / m_waves[i].wavelength;
        float f = k * glm::dot(dir, pos) - m_waves[i].omega * time;
        float A = m_waves[i].amplitude * waveHeightScale;
        float S = m_waves[i].steepness * horizontalLimit;

        p.x += (S * A * dir.x) * std::sin(f);
        p.z += (S * A * dir.y) * std::sin(f);
        p.y += A * std::cos(f);
    }
    return p;
}

glm::vec3 GerstnerWaveModel::calculateNormal(const glm::vec2 &pos, float time, float waveHeightScale) const {
    glm::vec3 p = evaluate(pos, time, waveHeightScale);
    glm::vec3 pDx = evaluate(pos + glm::vec2(EPSILON, 0.0f), time, waveHeightScale);
    glm::vec3 pDz = evaluate(pos + glm::vec2(0.0f, EPSILON), time, waveHeightScale);
    return glm::normalize(glm::cross(pDz - p, pDx - p));
}

glm::vec3 GerstnerWaveModel::surfaceAt(const glm::vec2 &worldXZ, float time, float scale) const {
    glm::vec2 parameter=worldXZ;
    float slopeBound=0;
    for(const auto &w:m_waves) slopeBound += w.steepness*w.amplitude*scale*6.2831853f/w.wavelength;
    float limit=std::min(1.0f,0.85f/std::max(slopeBound,0.0001f));
    for(int i=0;i<8;++i) {
        glm::vec3 point=evaluate(parameter,time,scale);
        glm::vec2 error=glm::vec2(point.x,point.z)-worldXZ;
        if(glm::dot(error,error)<1.0e-10f) break;
        float xx=1,zz=1,xz=0;
        for(const auto &w:m_waves) {
            float k=6.2831853f/w.wavelength;
            float d=w.steepness*limit*w.amplitude*scale*k*std::cos(k*glm::dot(w.direction,parameter)-w.omega*time);
            xx+=d*w.direction.x*w.direction.x;
            zz+=d*w.direction.y*w.direction.y;
            xz+=d*w.direction.x*w.direction.y;
        }
        float determinant=xx*zz-xz*xz;
        parameter -= glm::vec2(zz*error.x-xz*error.y,xx*error.y-xz*error.x)/determinant;
    }
    return glm::vec3(worldXZ.x,evaluate(parameter,time,scale).y,worldXZ.y);
}
