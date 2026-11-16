#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Input.h"

enum class CameraMode {
    THIRD_PERSON,
    COCKPIT,
    ORBIT
};

class Camera {
public:
    Camera();

    void update(float dt, CameraMode mode, const glm::vec3 &boatPos, float boatHeading,
                float boatPitch, float boatRoll, const glm::vec3 &cockpitEyePos, const Input &input);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix(float aspectRatio) const;
    const glm::vec3 &getPosition() const { return m_position; }
    const glm::vec3 &getTarget() const { return m_target; }

    void setOrbitRadius(float radius) { m_radius = radius; }
    void setFov(float fov) { m_fov = fov; }

private:
    glm::vec3 m_position;
    glm::vec3 m_target;
    glm::vec3 m_up;

    float m_yaw;
    float m_pitch;
    float m_radius;
    float m_fov;
    float m_nearPlane;
    float m_farPlane;
};
