#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"

class Mainland {
public:
    Mainland();

    void init(const glm::vec3 &origin = glm::vec3(95.0f, 0.0f, -125.0f));
    void render(const Shader &shader) const;

    const glm::vec3 &getOrigin() const { return m_origin; }

private:
    glm::vec3 m_origin;

    Mesh m_beachMesh;    // Material ID 16: Golden Sand & Shoreline
    Mesh m_hillsMesh;    // Material ID 17: Rolling Emerald Grassy Slopes
    Mesh m_cliffsMesh;   // Material ID 18: Granite Sea Cliffs & Distant Mountains
    Mesh m_villageMesh;  // Material ID 19: Coastal Cottages, Watchtower, Boathouse & Windows
    Mesh m_treesMesh;    // Material ID 20: Coastal Conifer / Pine Trees

    void buildTerrainGeometry();
    void buildVillageGeometry();
    void buildVegetationGeometry();
};
