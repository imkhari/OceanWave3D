#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"
#include "../ocean/GerstnerWave.h"
#include "../audio/AudioEngine.h"

struct Seagull {
    glm::vec3 position;
    glm::vec3 velocity;
    float heading;
    float pitch;
    float roll;
    float flapPhase;
    int flockId;
};

class WildlifeManager {
public:
    WildlifeManager();

    void init();
    void update(float dt, const glm::vec3 &boatPos, AudioEngine &audio);
    void render(const Shader &shader) const;

private:
    std::vector<Seagull> m_seagulls;
    Mesh m_seagullMesh;
    float m_seagullCallTimer;

    void updateSeagulls(float dt, const glm::vec3 &boatPos, AudioEngine &audio);
    void buildSeagullMesh();
};
