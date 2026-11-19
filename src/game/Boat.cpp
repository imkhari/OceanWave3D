#include "Boat.h"
#include "../audio/AudioEngine.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

Boat::Boat() : m_scale(3.0f, 2.1f, 5.0f), m_spotlightOn(false), m_splashSoundTimer(0.0f) {
    init();
}

void Boat::init() {
    std::vector<glm::vec3> verts;
    std::vector<unsigned int> indices;
    buildMesh(24, 2.8f, 0.70f, 0.34f, verts, indices);
    m_mesh.upload(verts, indices);
    // Wheel geometry in local space, with its physical radius compensated for hull scale.
    verts.clear(); indices.clear();
    const int rings=48, sides=8;
    for(int i=0;i<rings;++i) for(int j=0;j<sides;++j) {
        float a=i*6.2831853f/rings, b=j*6.2831853f/sides;
        float r=0.38f+0.028f*std::cos(b);
        verts.push_back(glm::vec3(r*std::cos(a)/3.0f,r*std::sin(a)/2.1f,0.028f*std::sin(b)/5.0f));
    }
    for(int i=0;i<rings;++i) for(int j=0;j<sides;++j) {
        unsigned a=i*sides+j,b=((i+1)%rings)*sides+j,c=i*sides+(j+1)%sides,d=((i+1)%rings)*sides+(j+1)%sides;
        indices.insert(indices.end(),{a,b,c,b,d,c});
    }
    Mesh::addRoundedBox(verts,indices,{0,0,0},{0.045f,0.064f,0.018f},0.006f);
    for(int i=0;i<3;++i) {
        float a=i*6.2831853f/3+1.5708f;
        unsigned start=static_cast<unsigned>(verts.size());
        Mesh::addCube(verts,indices,{0,0.085f,0},{0.012f,0.17f,0.009f});
        for(unsigned k=start;k<verts.size();++k) {
            glm::vec2 xy(verts[k].x*3,verts[k].y*2.1f);
            verts[k].x=(xy.x*std::cos(a)-xy.y*std::sin(a))/3;
            verts[k].y=(xy.x*std::sin(a)+xy.y*std::cos(a))/2.1f;
        }
    }
    m_wheel.upload(verts,indices);
    verts.clear(); indices.clear();
    // Three dial disks facing the seated captain (-Z).
    for(float x : {-0.17f,0.0f,0.17f}) {
        unsigned start=static_cast<unsigned>(verts.size());
        verts.push_back({x,0.54f,-0.342f});
        for(int i=0;i<48;++i) {
            float a=i*6.2831853f/48;
            verts.push_back({x+std::cos(a)*0.065f,0.54f+std::sin(a)*0.093f,-0.342f});
        }
        for(int i=0;i<48;++i) indices.insert(indices.end(),{start,start+1+(unsigned)((i+1)%48),start+1+(unsigned)i});
    }
    m_dials.upload(verts,indices);
    verts.clear(); indices.clear();
    // Thin glass surfaces; frames remain in the opaque hull mesh.
    verts={{-0.425f,0.515f,-0.125f},{0.425f,0.515f,-0.125f},{0.425f,0.735f,-0.035f},{-0.425f,0.735f,-0.035f},
           {-0.44f,0.515f,-0.31f},{-0.425f,0.515f,-0.125f},{-0.425f,0.735f,-0.035f},{-0.44f,0.735f,-0.26f},
           {0.425f,0.515f,-0.125f},{0.44f,0.515f,-0.31f},{0.44f,0.735f,-0.26f},{0.425f,0.735f,-0.035f}};
    indices={0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11};
    m_glass.upload(verts,indices);
    verts.clear(); indices.clear();
    Mesh::addRoundedBox(verts,indices,{0,0.055f,0},{0.018f,0.11f,0.018f},0.004f);
    Mesh::addRoundedBox(verts,indices,{0,0.12f,0},{0.05f,0.045f,0.035f},0.012f);
    m_throttle.upload(verts,indices);
    m_particles.init();
    reset();
}

void Boat::reset() {
    m_physics.reset();
    m_particles.clear();
    m_steering = m_throttleInput = 0.0f;
    m_sprayTimer = m_splashSoundTimer = 0.0f;
}

void Boat::update(float dt, const GerstnerWaveModel &waves, float time, float waveHeight,
                  bool canControl, const Input &input, AudioEngine *audio) {
    float targetSteering = canControl ? float(input.isKeyDown(GLFW_KEY_A))-float(input.isKeyDown(GLFW_KEY_D)) : 0.0f;
    float targetThrottle = canControl ? float(input.isKeyDown(GLFW_KEY_W))-float(input.isKeyDown(GLFW_KEY_S))*0.6f : 0.0f;
    float response = 1.0f-std::exp(-dt*7.0f);
    m_steering += (targetSteering-m_steering)*response;
    m_throttleInput += (targetThrottle-m_throttleInput)*response;
    // 1. Advance multi-point hydrodynamics simulation
    m_physics.update(dt, waves, time, waveHeight, canControl, input);

    m_splashSoundTimer -= dt;

    // Fine paired spray from the shoulders of the bow, aligned with the hull.
    // Sample actual surface contact; a fast airborne bow must not spray continuously.
    const float speed=m_physics.getSpeed();
    m_sprayTimer-=dt;
    if (speed>2.0f && !m_physics.isDocked() && m_sprayTimer<=0.0f) {
        m_sprayTimer=1.0f/30.0f;
        glm::mat4 transform=getTransform();
        glm::vec3 forward(std::sin(getHeading()),0,std::cos(getHeading()));
        glm::vec3 right(std::cos(getHeading()),0,-std::sin(getHeading()));
        bool contacted=false;
        for (float side : {-1.0f,1.0f}) {
            glm::vec3 source(0);
            float bestDistance=0.60f;
            bool touchesWater=false;
            // Spray from a wet shoulder, not from an airborne bow tip.
            for (float localZ : {1.0f,0.55f,0.0f,-0.55f}) {
                float t=(localZ+1.4f)/2.8f;
                float halfWidth=0.70f*std::pow(1.0f-t,0.25f)+0.02f*(1.0f-t);
                glm::vec3 candidate=glm::vec3(transform*glm::vec4(side*halfWidth,-0.16f,localZ,1));
                float waterY=waves.surfaceAt({candidate.x,candidate.z},time,waveHeight).y;
                float distance=std::abs(candidate.y-waterY);
                if (distance<bestDistance) {
                    bestDistance=distance;
                    source=candidate+right*side*0.08f;
                    source.y=waterY+0.04f;
                    touchesWater=true;
                }
            }
            if (!touchesWater) continue;
            float intensity=glm::clamp(speed/12.0f,0.35f,1.0f);
            m_particles.emitSplash(source,forward*speed,speed>9.0f ? 2 : 1,intensity,right*side);
            contacted=true;
        }
        if (contacted && audio && m_splashSoundTimer<=0.0f && speed>3.0f) {
            audio->playBowSplash(glm::clamp(speed/14.0f,0.25f,0.9f),getPosition()+forward*5.0f);
            m_splashSoundTimer=0.45f;
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
    glm::vec4 localEye(0.0f, 0.86f, -0.68f, 1.0f);
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
    shader.setInt("uPart",0);
    m_mesh.draw();
    shader.setInt("uPart",1);
    glm::mat4 wheel = glm::translate(getTransform(),glm::vec3(0,0.47f,-0.49f));
    wheel = glm::scale(wheel,1.0f/m_scale);
    wheel = glm::rotate(wheel,-m_steering*1.1f,glm::vec3(0,0,1));
    wheel = glm::scale(wheel,m_scale);
    shader.setMat4("model",wheel); m_wheel.draw();
    shader.setInt("uPart",3); shader.setMat4("model",getTransform());
    shader.setFloat("uSpeedKnots",getSpeed()*1.94384f); shader.setFloat("uHeading",getHeading());
    m_dials.draw();
    shader.setInt("uPart",4);
    glm::mat4 throttle=glm::translate(getTransform(),glm::vec3(0.29f,0.44f,-0.35f));
    throttle=glm::rotate(throttle,m_throttleInput*0.5f,glm::vec3(1,0,0));
    shader.setMat4("model",throttle); m_throttle.draw();
    shader.setInt("uPart",0);
    glEnable(GL_CULL_FACE);
}


void Boat::renderGlass(const Shader &shader) const {
    glDisable(GL_CULL_FACE); glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE);
    shader.use(); shader.setInt("uPart",2); shader.setMat4("model",getTransform());
    m_glass.draw(); shader.setInt("uPart",0);
    glDepthMask(GL_TRUE); glDisable(GL_BLEND); glEnable(GL_CULL_FACE);
}

void Boat::renderParticles(const Shader &particleShader, const Camera &camera, const glm::vec3 &sunColor, float aspect) const {
    m_particles.render(particleShader, camera, sunColor, aspect);
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
        float zTip = L * 0.5f + 0.05f;
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

    // Teak floor follows the narrowing hull rather than protruding through the bow.
    unsigned floorStart=static_cast<unsigned>(V.size());
    for (int j=0;j<=seg;++j) {
        float z=-1.30f+2.68f*j/seg;
        float hw=halfWidth((z-z0)/L)*0.80f;
        V.push_back({-hw,0.145f,z}); V.push_back({hw,0.145f,z});
    }
    for (int j=0;j<seg;++j) {
        unsigned a=floorStart+j*2;
        I.insert(I.end(),{a,a+2,a+1,a+1,a+2,a+3});
    }

    // --- 2. CABIN, SEATS & COCKPIT HELM INTERIOR ---
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.45f, -0.92f), glm::vec3(1.0f, 0.45f, 0.65f), 0.035f);      // Rear Cabin Roof
    Mesh::addCube(V, I, glm::vec3(0.0f, 1.0f, -0.92f), glm::vec3(0.05f, 1.5f, 0.05f));       // Mast

    // Captain's Bucket Seat
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.28f, -0.58f), glm::vec3(0.42f, 0.14f, 0.38f), 0.035f);     // Seat Base Cushion
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.50f, -0.74f), glm::vec3(0.40f, 0.36f, 0.10f), 0.035f);     // Seat Backrest

    // Ergonomic Helm Dashboard Console
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.34f, -0.28f), glm::vec3(0.76f, 0.26f, 0.24f), 0.035f);     // Console Main Body
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.46f, -0.24f), glm::vec3(0.70f, 0.12f, 0.18f), 0.035f);     // Slanted Instrument Panel

    Mesh::addRoundedBox(V,I,{0.0f,0.49f,-0.25f},{0.73f,0.29f,0.18f},0.025f);
    Mesh::addRoundedBox(V,I,{0.29f,0.40f,-0.34f},{0.085f,0.10f,0.14f},0.014f);
    Mesh::addCube(V,I,{0,0.47f,-0.40f},{0.035f,0.035f,0.14f});

    // Windshield Frame & Tinted Glass (Wraparound)
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.49f, -0.14f), glm::vec3(0.92f, 0.04f, 0.06f));     // Base Coaming
    Mesh::addCube(V, I, glm::vec3(0.0f, 0.75f, -0.035f), glm::vec3(0.88f, 0.03f, 0.04f));     // Top Header Rail
    Mesh::addCube(V, I, glm::vec3(-0.44f, 0.625f, -0.08f), glm::vec3(0.02f, 0.24f, 0.03f));   // Port Post
    Mesh::addCube(V, I, glm::vec3(0.44f, 0.625f, -0.08f), glm::vec3(0.02f, 0.24f, 0.03f));    // Starboard Post

    // --- 3. OUTBOARD MOTOR & PROPELLER (STERN) ---
    Mesh::addRoundedBox(V, I, glm::vec3(0.0f, 0.08f, -1.60f), glm::vec3(0.35f, 0.55f, 0.38f), 0.035f);  // Engine Cowling
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
