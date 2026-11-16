#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "Boat.h"
#include "../core/Input.h"
#include "../audio/AudioEngine.h"

class Harbor {
public:
    Harbor();

    void init(const glm::vec3 &lighthousePos = glm::vec3(85.0f, 0.0f, -105.0f));
    void update(float dt, Boat &boat, const Input &input, AudioEngine &audio);
    void render(const Shader &shader) const;

    bool canDock() const { return m_canDock; }
    bool isDocked() const { return m_isDocked; }
    const glm::vec3 &getBerthPosition() const { return m_dockBerthPos; }
    float getDistanceToBerth(const glm::vec3 &boatPos) const;
    float getDistanceToPier(const glm::vec3 &boatPos) const;

    void toggleDock(Boat &boat, AudioEngine &audio);
    void renderHUD(int screenWidth, int screenHeight, const Boat &boat) const;

private:
    glm::vec3 m_centerPos;
    glm::vec3 m_dockBerthPos;
    float m_dockHeading;

    bool m_canDock;
    bool m_isDocked;
    bool m_ePressedLast;
    float m_distToPier;
    float m_hudTime;

    Mesh m_pierDeckMesh;
    Mesh m_pilingsMesh;
    Mesh m_fittingsMesh;
    Mesh m_lanternsMesh;
    Mesh m_mooringRopesMesh;

    void buildGeometry();
    void buildMooringRopes(const glm::vec3 &boatPos, float boatHeading);
};
