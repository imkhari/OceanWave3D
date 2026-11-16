#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"

class Lighthouse {
public:
    Lighthouse();

    void init(const glm::vec3 &position = glm::vec3(90.0f, 0.0f, -110.0f));
    void update(float dt, float time, float sunDirY);
    void render(const Shader &shader) const;
    void renderBeam(const Shader &beamShader, const glm::mat4 &view, const glm::mat4 &proj) const;

    const glm::vec3 &getPosition() const { return m_position; }
    glm::vec3 getLanternPosition() const;
    glm::vec3 getBeamDirection() const;
    float getBeamIntensity() const { return m_beamIntensity; }
    bool isBeamActive() const { return m_beamIntensity > 0.05f; }

private:
    glm::vec3 m_position;
    float m_rotationAngle;
    float m_beamIntensity;
    bool m_topBeaconBlink;

    Mesh m_isletMesh;
    Mesh m_towerMesh;
    Mesh m_lanternMesh;
    Mesh m_beamConeMesh;

    void buildIsletGeometry();
    void buildTowerGeometry();
    void buildLanternGeometry();
    void buildBeamConeGeometry();
};
