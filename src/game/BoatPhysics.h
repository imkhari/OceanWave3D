#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "../ocean/GerstnerWave.h"
#include "../core/Input.h"

struct BuoyancyProbe {
    glm::vec3 localPos;
    glm::vec3 worldPos;
    float submersion;
    float force;
    bool isSubmerged;
};

struct BoatPhysicsParams {
    float mass = 1200.0f;          // Boat mass in kg
    float springK = 7500.0f;       // Buoyancy spring stiffness (N/m per probe)
    float damperC = 1800.0f;       // Buoyancy damper (Ns/m per probe)
    float linearDrag = 0.55f;      // Water drag forward/reverse
    float angularDrag = 3.2f;      // Angular damping for pitch and roll
    float engineThrust = 45.0f;    // Engine power
    float turnSpeed = 1.6f;        // Turning rate
    float bowRiseFactor = 1.4f;    // Bow lift during acceleration
    float turnBankFactor = 1.8f;   // Roll inward when banking in turns
};

class BoatPhysics {
public:
    static constexpr int PROBE_COUNT = 6;

    BoatPhysics();

    void reset();
    void setReviewPose(const glm::vec3 &position, float heading) { reset(); m_position = position; m_heading = heading; }
    void update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight, bool canControl, const Input &input);

    const glm::vec3 &getPosition() const { return m_position; }
    const glm::vec3 &getVelocity() const { return m_linearVelocity; }
    float getSpeed() const { return m_forwardSpeed; }
    float getHeading() const { return m_heading; }
    float getPitch() const { return m_pitch; }
    float getRoll() const { return m_roll; }
    glm::mat4 getTransformMatrix(const glm::vec3 &scale) const;

    // Docking & Mooring
    bool isDocked() const { return m_isDocked; }
    void setDocked(bool docked, const glm::vec3 &dockPos = glm::vec3(0.0f), float dockHeading = 0.0f);

    // Splash triggering info
    bool shouldEmitBowSplash(glm::vec3 &outBowPos, glm::vec3 &outBowVel) const;

    // Access to parameters for ImGui tweaking
    BoatPhysicsParams &getParams() { return m_params; }
    const BoatPhysicsParams &getParams() const { return m_params; }
    const std::vector<BuoyancyProbe> &getProbes() const { return m_probes; }

private:
    BoatPhysicsParams m_params;

    glm::vec3 m_position;
    glm::vec3 m_linearVelocity;
    float m_forwardSpeed;

    float m_heading;   // Yaw in radians
    float m_yawRate;
    float m_pitch;     // Nose up/down in radians
    float m_pitchRate;
    float m_roll;      // Lean left/right in radians
    float m_rollRate;

    bool m_isDocked;
    glm::vec3 m_dockTargetPos;
    float m_dockTargetHeading;

    std::vector<BuoyancyProbe> m_probes;
    BuoyancyProbe m_bowTipProbe;

    void initProbes();
};
