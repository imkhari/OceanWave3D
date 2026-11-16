#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "../ocean/GerstnerWave.h"

struct CargoItem {
    glm::vec3 pos;
    bool active;
    float angle;
};

class CargoManager {
public:
    CargoManager();

    void init(int count = 10, float spawnRange = 100.0f);
    void reset();

    int checkCollisions(const glm::vec3 &boatPos, float pickupRadius = 3.0f);
    void render(const Shader &shader, const GerstnerWaveModel &waves, float time, float waveHeight) const;

    int getActiveCount() const;
    int getTotalCount() const { return static_cast<int>(m_items.size()); }
    const std::vector<CargoItem> &getItems() const { return m_items; }

private:
    Mesh m_cubeMesh;
    std::vector<CargoItem> m_items;
};
