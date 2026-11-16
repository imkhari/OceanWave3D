#include "Boat.h"
#include "../audio/AudioEngine.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

Boat::Boat() : m_scale(3.0f, 2.1f, 5.0f), m_spotlightOn(true), m_splashSoundTimer(0.0f) {
    init();
}

void Boat::init() {
    std::vector<glm::vec3> verts;
    std::vector<unsigned int> indices;
    buildMesh(24, 2.8f, 0.70f, 0.34f, verts, indices);
    m_mesh.upload(verts, indices);
    m_particles.init();
    reset();
}

void Boat::reset() {
    m_physics.reset();
    m_particles.clear();
}

void Boat::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight,
                  bool canControl, const Input &input, AudioEngine *audio) {
    // 1. Advance multi-point hydrodynamics simulation
    m_physics.update(dt, waves, time, waveHeight, canControl, input);

    m_splashSoundTimer -= dt;

    // 2. Emit bow spray particles on wave impact or high speed
    glm::vec3 bowPos, bowVel;
    if (m_physics.shouldEmitBowSplash(bowPos, bowVel)) {
        float speed = m_physics.getSpeed();
        int particleCount = (speed > 9.0f) ? 6 : (speed > 5.0f ? 4 : 2);
        float intensity = glm::clamp(speed / 8.0f, 0.4f, 1.8f);
        m_particles.emitSplash(bowPos, bowVel, particleCount, intensity);

        if (audio && m_splashSoundTimer <= 0.0f && speed > 2.8f) {
            audio->playBowSplash(intensity, bowPos);
            m_splashSoundTimer = 0.32f;
        }
    }

    // 3. Update active water particles
    m_particles.update(dt, waves, time, waveHeight);
}

glm::mat4 Boat::getTransform() const {
    return m_physics.getTransformMatrix(m_scale);
}

glm::vec3 Boat::getCockpitEyePosition() const {
    glm::mat4 transform = getTransform();
    // Driver eye level position seated at the helm behind steering wheel and instruments
    glm::vec4 localEye(0.0f, 0.58f, -0.56f, 1.0f);
    return glm::vec3(transform * localEye);
}

glm::vec3 Boat::getSpotlightPosition() const {
    glm::mat4 transform = getTransform();
    // Bow spotlight emitter housing location
    glm::vec4 localSpot(0.0f, 0.28f, 1.35f, 1.0f);
    return glm::vec3(transform * localSpot);
}

glm::vec3 Boat::getSpotlightDirection() const {
    float heading = m_physics.getHeading();
    float pitch = m_physics.getPitch();

    // Beam pointed forward and slightly downward onto the water
    glm::vec3 dir(
        std::sin(heading) * std::cos(pitch - 0.12f),
        std::sin(pitch - 0.12f),
        std::cos(heading) * std::cos(pitch - 0.12f)
    );
    return glm::normalize(dir);
}

void Boat::render(const Shader &shader) const {
    glDisable(GL_CULL_FACE); // Render interior and exterior
    shader.setMat4("model", getTransform());
    shader.setVec3("uColorOverride", glm::vec3(0.0f));
    shader.setInt("uSpotlightOn", m_spotlightOn ? 1 : 0);
    m_mesh.draw();
    glEnable(GL_CULL_FACE);
}

void Boat::renderParticles(const Shader &particleShader, const Camera &camera, const glm::vec3 &sunColor) const {
    m_particles.render(particleShader, camera, sunColor);
}

void Boat::buildMesh(int seg, float L, float halfW0, float H,
                     std::vector<glm::vec3> &V, std::vector<unsigned int> &I) {
    V.clear();
    I.clear();

    // --- 1. MAIN HULL HULL SHELL ---
    auto halfWidth = [&](float t) {
        float w = halfW0 * std::pow(1.0f - t, 0.25f) + 0.02f * (1.0f - t);
        return std::max(0.02f, w);
    };
    auto yDeck = [](float) { return 0.22f; };
    auto yBottom = [&](float t) { return -H * (0.8f - 0.4f * t); };
    const float rimInset = 0.15f;
    const float rimDrop = 0.05f;
    float z0 = -L * 0.5f, z1 = L * 0.5f;

    for (int i = 0; i <= seg; i++) {
        float t = i / static_cast<float>(seg);
        float z = z0 + (z1 - z0) * t;
        float hw = halfWidth(t);
        float yT = yDeck(t);
        float yB = yBottom(t);
        float hwInner = std::max(0.01f, hw * (1.0f - rimInset));
        float yR = yT - rimDrop;

        V.push_back({-hw, yT, z});         // 0 LT_o
        V.push_back({hw, yT, z});          // 1 RT_o
        V.push_back({-hw * 0.72f, yB, z}); // 2 LB_o
        V.push_back({hw * 0.72f, yB, z});  // 3 RB_o
        V.push_back({-hwInner, yR, z});    // 4 LT_i
        V.push_back({hwInner, yR, z});     // 5 RT_i
    }

    auto vid = [](int i, int k) { return i * 6 + k; };
    for (int i = 0; i < seg; i++) {
        int LT0 = vid(i, 0), RT0 = vid(i, 1), LB0 = vid(i, 2), RB0 = vid(i, 3), LTi0 = vid(i, 4), RTi0 = vid(i, 5);
        int LT1 = vid(i + 1, 0), RT1 = vid(i + 1, 1), LB1 = vid(i + 1, 2), RB1 = vid(i + 1, 3), LTi1 = vid(i + 1, 4), RTi1 = vid(i + 1, 5);
        I.insert(I.end(), {
            (unsigned)LB0, (unsigned)LT0, (unsigned)LB1, (unsigned)LT0, (unsigned)LT1, (unsigned)LB1,
            (unsigned)RT0, (unsigned)RB0, (unsigned)RT1, (unsigned)RB0, (unsigned)RB1, (unsigned)RT1,
            (unsigned)LB0, (unsigned)RB1, (unsigned)RB0, (unsigned)LB0, (unsigned)LB1, (unsigned)RB1,
            (unsigned)LT0, (unsigned)LT1, (unsigned)LTi1, (unsigned)LT0, (unsigned)LTi1, (unsigned)LTi0,
            (unsigned)RT0, (unsigned)RTi1, (unsigned)RT1, (unsigned)RT0, (unsigned)RTi0, (unsigned)RTi1
        });
    }

    // Transom (stern closing)
    {
        unsigned LT = vid(0, 0), RT = vid(0, 1), LB = vid(0, 2), RB = vid(0, 3), LTi = vid(0, 4), RTi = vid(0, 5);
        I.insert(I.end(), {LT, RT, RB, LT, RB, LB});
        I.insert(I.end(), {LT, LTi, RTi, LT, RTi, RT});
    }

    // Bow (front tip)
    {
        unsigned LT = vid(seg, 0), RT = vid(seg, 1), LB = vid(seg, 2), RB = vid(seg, 3);
        unsigned LTi = vid(seg, 4), RTi = vid(seg, 5);
        float zTip = (-L * 0.5f + L * 0.5f) + 0.05f;
        float yB = yBottom(1.0f) - 0.02f;
        float yR = yDeck(1.0f) - 0.05f;

        unsigned botTip = static_cast<unsigned>(V.size());
        V.push_back({0.0f, yB, zTip});
        unsigned rimTip = static_cast<unsigned>(V.size());
        V.push_back({0.0f, yR, zTip - 0.01f});

        I.insert(I.end(), {
            LB, botTip, RB,
            LT, LB, botTip,
            RB, RT, botTip,
            LT, botTip, RT,
            LTi, rimTip, RTi,
            LT, LTi, rimTip,
            rimTip, RTi, RT
        });
    }

    // --- 2. CABIN, SEATS & COCKPIT HELM INTERIOR ---
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.45f, -0.92f), glm::vec3(1.0f, 0.45f, 0.65f));      // Rear Cabin Roof
    Mesh::addCube(V, I, glm::vec3(0.0f, 1.0f, -0.92f), glm::vec3(0.05f, 1.5f, 0.05f));       // Mast

    // Captain's Bucket Seat
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.28f, -0.58f), glm::vec3(0.42f, 0.14f, 0.38f));     // Seat Base Cushion
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.50f, -0.74f), glm::vec3(0.40f, 0.36f, 0.10f));     // Seat Backrest

    // Ergonomic Helm Dashboard Console
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.34f, -0.28f), glm::vec3(0.76f, 0.26f, 0.24f));     // Console Main Body
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.46f, -0.24f), glm::vec3(0.70f, 0.12f, 0.18f));     // Slanted Instrument Panel

    // Instrument Cluster: Analog Speedometer, Marine Compass Dome, Engine Tachometer
    Mesh::addCube(V, I, glm::vec3(-0.16f, 0.46f, -0.21f), glm::vec3(0.09f, 0.09f, 0.03f));   // Speedometer Dial
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.49f, -0.20f), glm::vec3(0.10f, 0.08f, 0.10f));     // Spherical Compass
    Mesh::addCube(V, I, glm::vec3(0.16f, 0.46f, -0.21f), glm::vec3(0.09f, 0.09f, 0.03f));    // Tachometer Dial
    Mesh::addCube(V, I, glm::vec3(-0.02f, 0.43f, -0.21f), glm::vec3(0.12f, 0.06f, 0.02f));   // GPS / Marine Radio Display

    // Marine Steering Wheel (Stainless Steel Spokes & Mahogany/Leather Rim)
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.44f, -0.32f), glm::vec3(0.04f, 0.04f, 0.16f));     // Steering Column Stalk
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.45f, -0.38f), glm::vec3(0.07f, 0.07f, 0.04f));     // Wheel Center Hub
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.45f, -0.38f), glm::vec3(0.24f, 0.02f, 0.02f));     // Horizontal Spoke
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.42f, -0.38f), glm::vec3(0.02f, 0.12f, 0.02f));     // Vertical Spoke
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.54f, -0.38f), glm::vec3(0.26f, 0.03f, 0.03f));     // Wheel Rim Top
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.36f, -0.38f), glm::vec3(0.26f, 0.03f, 0.03f));     // Wheel Rim Bottom
    Mesh::addCube(V, I, glm::vec3(-0.13f, 0.45f, -0.38f), glm::vec3(0.03f, 0.20f, 0.03f));   // Wheel Rim Left
    Mesh::addCube(V, I, glm::vec3(0.13f, 0.45f, -0.38f), glm::vec3(0.03f, 0.20f, 0.03f));    // Wheel Rim Right

    // Throttle Control Quadrant (Starboard side)
    Mesh::addCube(V, I, glm::vec3(0.28f, 0.40f, -0.34f), glm::vec3(0.08f, 0.10f, 0.14f));    // Throttle Box Base
    Mesh::addCube(V, I, glm::vec3(0.28f, 0.46f, -0.32f), glm::vec3(0.025f, 0.14f, 0.025f));  // Lever Stalk
    Mesh::addCube(V, I, glm::vec3(0.28f, 0.52f, -0.30f), glm::vec3(0.05f, 0.05f, 0.05f));    // Red Throttle Knob

    // Windshield Frame & Tinted Glass (Wraparound)
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.49f, -0.14f), glm::vec3(0.92f, 0.04f, 0.06f));     // Base Coaming
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.69f, -0.06f), glm::vec3(0.88f, 0.03f, 0.04f));     // Top Header Rail
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.59f, -0.10f), glm::vec3(0.03f, 0.18f, 0.03f));     // Center Post
    Mesh::addCube(V, I, glm::vec3(-0.44f, 0.59f, -0.10f), glm::vec3(0.03f, 0.18f, 0.03f));   // Port Post
    Mesh::addCube(V, I, glm::vec3(0.44f, 0.59f, -0.10f), glm::vec3(0.03f, 0.18f, 0.03f));    // Starboard Post
    Mesh::addCube(V, I, glm::vec3(-0.46f, 0.58f, -0.22f), glm::vec3(0.02f, 0.16f, 0.22f));   // Port Wing Glass
    Mesh::addCube(V, I, glm::vec3(0.46f, 0.58f, -0.22f), glm::vec3(0.02f, 0.16f, 0.22f));    // Starboard Wing Glass
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.59f, -0.10f), glm::vec3(0.86f, 0.18f, 0.02f));     // Front Glass Panel

    // --- 3. OUTBOARD MOTOR & PROPELLER (STERN) ---
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.08f, -1.60f), glm::vec3(0.35f, 0.55f, 0.38f));  // Engine Cowling
    Mesh::addCube(V, I, glm::vec3(0.0f, -0.30f, -1.60f), glm::vec3(0.12f, 0.40f, 0.16f)); // Shaft Column
    Mesh::addCube(V, I, glm::vec3(0.0f, -0.46f, -1.68f), glm::vec3(0.16f, 0.16f, 0.26f)); // Propeller Hub
    Mesh::addCube(V, I, glm::vec3(0.0f, -0.46f, -1.72f), glm::vec3(0.38f, 0.04f, 0.04f)); // Prop Blade H
    Mesh::addCube(V, I, glm::vec3(0.0f, -0.46f, -1.72f), glm::vec3(0.04f, 0.38f, 0.04f)); // Prop Blade V

    // --- 4. CHROME RAILINGS (BOW & GUNWALE) ---
    // Port Railing & Stanchions
    Mesh::addCube(V, I, glm::vec3(-0.62f, 0.34f, 0.40f), glm::vec3(0.025f, 0.025f, 1.40f));
    Mesh::addCube(V, I, glm::vec3(-0.62f, 0.26f, -0.25f), glm::vec3(0.025f, 0.16f, 0.025f));
    Mesh::addCube(V, I, glm::vec3(-0.62f, 0.26f, 0.40f), glm::vec3(0.025f, 0.16f, 0.025f));
    Mesh::addCube(V, I, glm::vec3(-0.62f, 0.26f, 1.05f), glm::vec3(0.025f, 0.16f, 0.025f));

    // Starboard Railing & Stanchions
    Mesh::addCube(V, I, glm::vec3(0.62f, 0.34f, 0.40f), glm::vec3(0.025f, 0.025f, 1.40f));
    Mesh::addCube(V, I, glm::vec3(0.62f, 0.26f, -0.25f), glm::vec3(0.025f, 0.16f, 0.025f));
    Mesh::addCube(V, I, glm::vec3(0.62f, 0.26f, 0.40f), glm::vec3(0.025f, 0.16f, 0.025f));
    Mesh::addCube(V, I, glm::vec3(0.62f, 0.26f, 1.05f), glm::vec3(0.025f, 0.16f, 0.025f));

    // Bow Pulpit railing connecting front
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.34f, 1.10f), glm::vec3(1.24f, 0.025f, 0.025f));

    // --- 5. MARITIME NAVIGATION LIGHTS & SPOTLIGHT ---
    Mesh::addCube(V, I, glm::vec3(-0.68f, 0.26f, 0.80f), glm::vec3(0.08f, 0.08f, 0.12f));  // Port (Red)
    Mesh::addCube(V, I, glm::vec3(0.68f, 0.26f, 0.80f), glm::vec3(0.08f, 0.08f, 0.12f));   // Starboard (Green)
    Mesh::addCube(V, I, glm::vec3(0.0f, 1.75f, -0.80f), glm::vec3(0.08f, 0.10f, 0.08f));   // Masthead (White)
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.28f, 1.35f), glm::vec3(0.14f, 0.14f, 0.14f));    // Bow Spotlight

    // --- 6. CABIN LIFEBUOY (ORANGE SAFETY RING) ---
    Mesh::addCube(V, I, glm::vec3(-0.52f, 0.45f, -0.80f), glm::vec3(0.04f, 0.26f, 0.26f));
}
