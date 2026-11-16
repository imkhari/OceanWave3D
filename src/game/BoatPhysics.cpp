#include "BoatPhysics.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <algorithm>

BoatPhysics::BoatPhysics() {
    initProbes();
    reset();
}

void BoatPhysics::initProbes() {
    m_probes.clear();
    // 6 Submerged Probes arranged around the hull bottom
    m_probes.push_back({ glm::vec3(-1.0f, -0.4f,  4.0f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Bow Left
    m_probes.push_back({ glm::vec3( 1.0f, -0.4f,  4.0f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Bow Right
    m_probes.push_back({ glm::vec3(-1.4f, -0.5f,  0.0f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Mid Left
    m_probes.push_back({ glm::vec3( 1.4f, -0.5f,  0.0f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Mid Right
    m_probes.push_back({ glm::vec3(-1.1f, -0.4f, -4.2f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Stern Left
    m_probes.push_back({ glm::vec3( 1.1f, -0.4f, -4.2f), glm::vec3(0.0f), 0.0f, 0.0f, false }); // Stern Right

    // Splash probe at the sharp front tip of the bow
    m_bowTipProbe = { glm::vec3(0.0f, -0.2f, 5.8f), glm::vec3(0.0f), 0.0f, 0.0f, false };
}

void BoatPhysics::reset() {
    m_position = glm::vec3(0.0f, 0.0f, 0.0f);
    m_linearVelocity = glm::vec3(0.0f);
    m_forwardSpeed = 0.0f;

    m_heading = 0.0f;
    m_yawRate = 0.0f;
    m_pitch = 0.0f;
    m_pitchRate = 0.0f;
    m_roll = 0.0f;
    m_rollRate = 0.0f;

    m_isDocked = false;
    m_dockTargetPos = glm::vec3(0.0f);
    m_dockTargetHeading = 0.0f;

    for (auto &probe : m_probes) {
        probe.submersion = 0.0f;
        probe.force = 0.0f;
        probe.isSubmerged = false;
    }
}

void BoatPhysics::setDocked(bool docked, const glm::vec3 &dockPos, float dockHeading) {
    m_isDocked = docked;
    if (docked) {
        m_dockTargetPos = dockPos;
        m_dockTargetHeading = dockHeading;
        m_forwardSpeed = 0.0f;
        m_linearVelocity = glm::vec3(0.0f);
    }
}

void BoatPhysics::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight, bool canControl, const Input &input) {
    if (dt <= 0.0f) return;
    dt = glm::min(dt, 0.05f); // Prevent physics explosion on lag spikes

    // 1. Process steering & thrust input
    float thrustInput = 0.0f;
    float steerInput = 0.0f;

    if (canControl && !m_isDocked) {
        if (input.isKeyDown(GLFW_KEY_W)) thrustInput += 1.0f;
        if (input.isKeyDown(GLFW_KEY_S)) thrustInput -= 0.6f;
        if (input.isKeyDown(GLFW_KEY_A)) steerInput += 1.0f;
        if (input.isKeyDown(GLFW_KEY_D)) steerInput -= 1.0f;
    }

    if (m_isDocked) {
        // Moored to dock: gently settle position and heading
        m_position.x += (m_dockTargetPos.x - m_position.x) * std::min(dt * 3.5f, 1.0f);
        m_position.z += (m_dockTargetPos.z - m_position.z) * std::min(dt * 3.5f, 1.0f);
        m_heading += (m_dockTargetHeading - m_heading) * std::min(dt * 3.5f, 1.0f);
        m_forwardSpeed = 0.0f;
        m_linearVelocity.x = 0.0f;
        m_linearVelocity.z = 0.0f;
        thrustInput = 0.0f;
        steerInput = 0.0f;
    }

    // Forward drive force & water drag
    float targetThrust = thrustInput * m_params.engineThrust;
    m_forwardSpeed += (targetThrust - m_forwardSpeed * m_params.linearDrag) * dt;
    m_forwardSpeed = glm::clamp(m_forwardSpeed, -6.0f, 18.0f);

    // Yaw rotation (steers proportionally to forward speed)
    float speedRatio = glm::clamp(std::abs(m_forwardSpeed) / 4.0f, 0.15f, 1.0f);
    m_yawRate = steerInput * m_params.turnSpeed * speedRatio;
    m_heading += m_yawRate * dt;

    // Direction vector
    glm::vec3 forwardDir(std::sin(m_heading), 0.0f, std::cos(m_heading));
    m_position.x += forwardDir.x * m_forwardSpeed * dt;
    m_position.z += forwardDir.z * m_forwardSpeed * dt;

    // 2. Compute Orientation Matrix
    glm::mat4 rotMatrix = glm::rotate(glm::mat4(1.0f), m_heading, glm::vec3(0.0f, 1.0f, 0.0f));
    rotMatrix = glm::rotate(rotMatrix, m_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    rotMatrix = glm::rotate(rotMatrix, m_roll, glm::vec3(0.0f, 0.0f, 1.0f));

    // 3. Multi-point Buoyancy & Torques
    float totalBuoyancyForce = 0.0f;
    float pitchTorque = 0.0f;
    float rollTorque = 0.0f;

    for (auto &probe : m_probes) {
        // Transform probe to world space
        glm::vec3 worldOffset = glm::vec3(rotMatrix * glm::vec4(probe.localPos, 1.0f));
        probe.worldPos = m_position + worldOffset;

        // Vertical velocity of probe due to linear + angular motion
        float probeVelY = m_linearVelocity.y - probe.localPos.z * m_pitchRate + probe.localPos.x * m_rollRate;

        // Sample water wave height at probe's world (x, z)
        glm::vec2 pos2D(probe.worldPos.x, probe.worldPos.z);
        float waveSurfaceY = waves.evaluate(pos2D, time, waveHeight).y;

        probe.submersion = waveSurfaceY - probe.worldPos.y;
        probe.isSubmerged = (probe.submersion > 0.0f);

        if (probe.isSubmerged) {
            float springF = m_params.springK * probe.submersion;
            float damperF = -m_params.damperC * probeVelY;
            probe.force = glm::max(0.0f, springF + damperF);

            totalBuoyancyForce += probe.force;

            // Torque contributions (around boat center of mass)
            // Bow (+Z) upward force pitches nose down (-pitch)
            pitchTorque -= probe.localPos.z * probe.force;
            // Starboard (+X) upward force rolls boat left (+roll)
            rollTorque += probe.localPos.x * probe.force;
        } else {
            probe.force = 0.0f;
        }
    }

    // 4. Update Bow Tip Probe for Splash Detection
    glm::vec3 bowWorldOffset = glm::vec3(rotMatrix * glm::vec4(m_bowTipProbe.localPos, 1.0f));
    m_bowTipProbe.worldPos = m_position + bowWorldOffset;
    glm::vec2 bowPos2D(m_bowTipProbe.worldPos.x, m_bowTipProbe.worldPos.z);
    float bowWaveY = waves.evaluate(bowPos2D, time, waveHeight).y;
    m_bowTipProbe.submersion = bowWaveY - m_bowTipProbe.worldPos.y;
    m_bowTipProbe.isSubmerged = (m_bowTipProbe.submersion > 0.0f);

    // 5. Coupled Hydrodynamic Torques
    // A. Bow Rise on acceleration: Thrust creates positive pitch torque (nose rises)
    if (thrustInput > 0.1f) {
        pitchTorque += thrustInput * m_params.bowRiseFactor * 1800.0f;
    }

    // B. Centripetal Roll: When turning at speed, boat banks inward into the turn
    float centripetalAccel = m_forwardSpeed * m_yawRate;
    rollTorque -= centripetalAccel * m_params.turnBankFactor * 800.0f;

    // 6. Integrate Linear Vertical Dynamics (F = m * a)
    constexpr float GRAVITY = 9.8f;
    float gravityForce = m_params.mass * GRAVITY;
    float netVerticalForce = totalBuoyancyForce - gravityForce;
    float verticalAccel = netVerticalForce / m_params.mass;

    m_linearVelocity.y += verticalAccel * dt;
    m_linearVelocity.y *= (1.0f - 0.2f * dt); // Subtle vertical fluid damping
    m_position.y += m_linearVelocity.y * dt;

    // 7. Integrate Rotational Dynamics (Torque = I * alpha)
    float momentPitch = m_params.mass * 8.5f;
    float momentRoll = m_params.mass * 2.2f;

    // Pitch integration
    float pitchAccel = (pitchTorque - m_pitchRate * m_params.angularDrag * momentPitch * 0.4f) / momentPitch;
    m_pitchRate += pitchAccel * dt;
    m_pitch += m_pitchRate * dt;
    m_pitch = glm::clamp(m_pitch, -0.45f, 0.45f); // Limit max pitch angle (~25 deg)

    // Roll integration
    float rollAccel = (rollTorque - m_rollRate * m_params.angularDrag * momentRoll * 0.4f) / momentRoll;
    m_rollRate += rollAccel * dt;
    m_roll += m_rollRate * dt;
    m_roll = glm::clamp(m_roll, -0.45f, 0.45f);   // Limit max roll angle (~25 deg)
}

bool BoatPhysics::shouldEmitBowSplash(glm::vec3 &outBowPos, glm::vec3 &outBowVel) const {
    outBowPos = m_bowTipProbe.worldPos;
    outBowVel = glm::vec3(std::sin(m_heading), 0.0f, std::cos(m_heading)) * m_forwardSpeed;

    // Emit splash when boat has forward momentum and bow penetrates wave
    bool highSpeed = (m_forwardSpeed > 4.5f);
    bool waveImpact = (m_bowTipProbe.submersion > 0.05f && m_forwardSpeed > 2.0f);
    return highSpeed || waveImpact;
}

glm::mat4 BoatPhysics::getTransformMatrix(const glm::vec3 &scale) const {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position + glm::vec3(0.0f, 0.35f, 0.0f));
    model = glm::rotate(model, m_heading, glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::rotate(model, m_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    model = glm::rotate(model, m_roll, glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::scale(model, scale);
    return model;
}
