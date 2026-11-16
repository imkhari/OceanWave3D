#include "Cargo.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cstdlib>

CargoManager::CargoManager() {
    std::vector<glm::vec3> verts;
    std::vector<unsigned int> indices;
    Mesh::addCube(verts, indices, glm::vec3(0.0f), glm::vec3(1.0f));
    m_cubeMesh.upload(verts, indices);

    init(10, 100.0f);
}

void CargoManager::init(int count, float spawnRange) {
    m_items.clear();
    for (int i = 0; i < count; ++i) {
        float rX = ((std::rand() / static_cast<float>(RAND_MAX)) - 0.5f) * spawnRange;
        float rZ = ((std::rand() / static_cast<float>(RAND_MAX)) - 0.5f) * spawnRange;
        m_items.push_back({ glm::vec3(rX, 0.0f, rZ), true, static_cast<float>(i) });
    }
}

void CargoManager::reset() {
    for (auto &item : m_items) {
        item.active = true;
    }
}

int CargoManager::checkCollisions(const glm::vec3 &boatPos, float pickupRadius) {
    int collected = 0;
    for (auto &item : m_items) {
        if (!item.active) continue;

        float dist = glm::distance(glm::vec2(boatPos.x, boatPos.z), glm::vec2(item.pos.x, item.pos.z));
        if (dist < pickupRadius) {
            item.active = false;
            collected++;
        }
    }
    return collected;
}

void CargoManager::render(const Shader &shader, const GerstnerWaveModel &waves, float time, float waveHeight) const {
    shader.setVec3("uColorOverride", glm::vec3(1.0f, 0.8f, 0.0f)); // Golden color

    for (const auto &item : m_items) {
        if (!item.active) continue;

        glm::vec2 pos2D(item.pos.x, item.pos.z);
        glm::vec3 waveP = waves.evaluate(pos2D, time, waveHeight);
        glm::vec3 nW = waves.calculateNormal(pos2D, time, waveHeight);

        glm::vec3 stableN = glm::normalize(glm::mix(glm::vec3(0.0f, 1.0f, 0.0f), nW, 0.5f));

        glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(item.pos.x, waveP.y - 0.2f, item.pos.z));
        model = model * glm::mat4_cast(glm::rotation(glm::vec3(0.0f, 1.0f, 0.0f), stableN));
        model = glm::rotate(model, item.angle + time * 0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(1.5f));

        shader.setMat4("model", model);
        m_cubeMesh.draw();
    }

    shader.setVec3("uColorOverride", glm::vec3(0.0f));
}

int CargoManager::getActiveCount() const {
    int count = 0;
    for (const auto &item : m_items) {
        if (item.active) count++;
    }
    return count;
}
