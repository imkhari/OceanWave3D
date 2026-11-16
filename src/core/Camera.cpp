#include "Camera.h"
#include <cmath>
#include <algorithm>

Camera::Camera()
    : m_position(0.0f, 10.0f, 35.0f),
      m_target(0.0f, 2.0f, 0.0f),
      m_up(0.0f, 1.0f, 0.0f),
      m_yaw(180.0f),
      m_pitch(20.0f),
      m_radius(35.0f),
      m_fov(50.0f),
      m_nearPlane(0.1f),
      m_farPlane(800.0f) {}

void Camera::update(float dt, CameraMode mode, const glm::vec3 &boatPos, float boatHeading,
                    float boatPitch, float boatRoll, const glm::vec3 &cockpitEyePos, const Input &input) {
    if (mode == CameraMode::COCKPIT) {
        // First-Person Cockpit View (Eye level seated behind steering wheel & dashboard)
        m_position = cockpitEyePos;

        float eyePitch = boatPitch - 0.08f; // Natural slight downward glance to see instruments & ocean
        glm::vec3 forwardDir(
            std::sin(boatHeading) * std::cos(eyePitch),
            std::sin(eyePitch),
            std::cos(boatHeading) * std::cos(eyePitch)
        );
        m_target = m_position + forwardDir * 20.0f;
        m_up = glm::vec3(0.0f, 1.0f, 0.0f);
        m_fov = 72.0f; // Wider FOV for full cockpit instruments & horizon
    }
    else if (mode == CameraMode::THIRD_PERSON) {
        // Third Person chase camera behind the boat
        glm::vec3 boatForward(std::sin(boatHeading), 0.0f, std::cos(boatHeading));
        float distBehind = 14.0f;
        float height = 5.5f;

        m_position = boatPos - (boatForward * distBehind) + glm::vec3(0.0f, height, 0.0f);
        m_target = boatPos + (boatForward * 6.0f) + glm::vec3(0.0f, 1.8f, 0.0f);
        m_up = glm::vec3(0.0f, 1.0f, 0.0f);
        m_fov = 50.0f;
    }
    else {
        // Orbit camera around boat
        if (input.isKeyDown(GLFW_KEY_LEFT))  m_yaw += 60.0f * dt;
        if (input.isKeyDown(GLFW_KEY_RIGHT)) m_yaw -= 60.0f * dt;
        if (input.isKeyDown(GLFW_KEY_UP))    m_pitch = glm::clamp(m_pitch + 60.0f * dt, -10.0f, 80.0f);
        if (input.isKeyDown(GLFW_KEY_DOWN))  m_pitch = glm::clamp(m_pitch - 60.0f * dt, -10.0f, 80.0f);
        if (input.isKeyDown(GLFW_KEY_Q))     m_radius = glm::max(5.0f, m_radius - 30.0f * dt);
        if (input.isKeyDown(GLFW_KEY_E))     m_radius += 30.0f * dt;

        m_target = boatPos + glm::vec3(0.0f, 2.0f, 0.0f);
        float radYaw = glm::radians(m_yaw);
        float radPitch = glm::radians(m_pitch);

        m_position = m_target + glm::vec3(
            m_radius * std::cos(radYaw) * std::cos(radPitch),
            m_radius * std::sin(radPitch),
            m_radius * std::sin(radYaw) * std::cos(radPitch)
        );
        m_up = glm::vec3(0.0f, 1.0f, 0.0f);
        m_fov = 50.0f;
    }
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(m_position, m_target, m_up);
}

glm::mat4 Camera::getProjectionMatrix(float aspectRatio) const {
    return glm::perspective(glm::radians(m_fov), aspectRatio, m_nearPlane, m_farPlane);
}
