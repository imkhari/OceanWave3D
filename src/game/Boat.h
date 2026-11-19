#pragma once

#include <glm/glm.hpp>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "../renderer/ParticleSystem.h"
#include "../ocean/GerstnerWave.h"
#include "../core/Input.h"
#include "../core/Camera.h"
#include "BoatPhysics.h"

class AudioEngine;

class Boat {
public:
    Boat();

    void init();
    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight,
                bool canControl, const Input &input, AudioEngine *audio = nullptr);
    void render(const Shader &shader) const;
    void renderGlass(const Shader &shader) const;
    void renderParticles(const Shader &particleShader, const Camera &camera, const glm::vec3 &sunColor, float aspect = 16.0f/9.0f) const;
    void reset();

    const glm::vec3 &getPosition() const { return m_physics.getPosition(); }
    float getHeading() const { return m_physics.getHeading(); }
    float getPitch() const { return m_physics.getPitch(); }
    float getRoll() const { return m_physics.getRoll(); }
    float getSpeed() const { return m_physics.getSpeed(); }
    glm::mat4 getTransform() const;

    // Cockpit eye level position in world space
    glm::vec3 getCockpitEyePosition() const;

    // Spotlight controls
    bool isSpotlightOn() const { return m_spotlightOn; }
    void toggleSpotlight() { m_spotlightOn = !m_spotlightOn; }
    void setSpotlight(bool on) { m_spotlightOn = on; }
    glm::vec3 getSpotlightPosition() const;
    glm::vec3 getSpotlightDirection() const;

    // Docking & Mooring
    bool isDocked() const { return m_physics.isDocked(); }
    void setDocked(bool docked, const glm::vec3 &dockPos = glm::vec3(0.0f), float dockHeading = 0.0f) {
        m_physics.setDocked(docked, dockPos, dockHeading);
    }

    BoatPhysics &getPhysics() { return m_physics; }
    const BoatPhysics &getPhysics() const { return m_physics; }
    ParticleSystem &getParticleSystem() { return m_particles; }
    const ParticleSystem &getParticleSystem() const { return m_particles; }

private:
    Mesh m_mesh, m_wheel, m_glass, m_dials, m_throttle;
    float m_steering = 0.0f, m_throttleInput = 0.0f;
    BoatPhysics m_physics;
    ParticleSystem m_particles;
    glm::vec3 m_scale;
    bool m_spotlightOn;
    float m_splashSoundTimer;
    float m_sprayTimer = 0.0f;

    static void buildMesh(int seg, float L, float halfW0, float H,
                          std::vector<glm::vec3> &V, std::vector<unsigned int> &I);
};
