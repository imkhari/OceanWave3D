#include "Harbor.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <imgui.h>

Harbor::Harbor()
    : m_centerPos(85.0f, 0.0f, -105.0f),
      m_dockBerthPos(58.0f, 0.0f, -100.5f),
      m_dockHeading(1.5707963f), // Pi/2 (parallel to pier facing +X)
      m_canDock(false),
      m_isDocked(false),
      m_ePressedLast(false),
      m_distToPier(999.0f),
      m_hudTime(0.0f) {}

void Harbor::init(const glm::vec3 &lighthousePos) {
    m_centerPos = lighthousePos;
    // Berth slip located on calm southern face of the pier
    m_dockBerthPos = m_centerPos + glm::vec3(-27.0f, 0.0f, 4.5f);
    m_dockHeading = 1.5707963f;

    buildGeometry();
}

float Harbor::getDistanceToBerth(const glm::vec3 &boatPos) const {
    return glm::length(glm::vec2(boatPos.x - m_dockBerthPos.x, boatPos.z - m_dockBerthPos.z));
}

float Harbor::getDistanceToPier(const glm::vec3 &boatPos) const {
    // Distance to the 34m pier walkway centerline between (44.0, -105.0) and (78.0, -105.0)
    float t = glm::clamp((boatPos.x - 44.0f) / (78.0f - 44.0f), 0.0f, 1.0f);
    glm::vec2 closestPierPt(44.0f + t * 34.0f, m_centerPos.z);
    return glm::length(glm::vec2(boatPos.x, boatPos.z) - closestPierPt);
}

void Harbor::toggleDock(Boat &boat, AudioEngine &audio) {
    m_isDocked = !m_isDocked;

    if (m_isDocked) {
        // Calculate smart berth position and heading alongside the pier
        glm::vec3 boatPos = boat.getPosition();
        bool southSide = (boatPos.z >= m_centerPos.z);
        float berthZ = southSide ? (m_centerPos.z + 4.5f) : (m_centerPos.z - 4.5f);
        float berthX = glm::clamp(boatPos.x, 46.0f, 74.0f);

        float boatHeading = boat.getHeading();
        float normHeading = std::atan2(std::sin(boatHeading), std::cos(boatHeading));
        float berthHeading = (normHeading >= 0.0f) ? 1.5707963f : -1.5707963f; // East (+X) or West (-X)

        m_dockBerthPos = glm::vec3(berthX, 0.0f, berthZ);
        m_dockHeading = berthHeading;

        boat.setDocked(true, m_dockBerthPos, m_dockHeading);
        buildMooringRopes(m_dockBerthPos, m_dockHeading);
        audio.playBuoyBell(m_dockBerthPos); // Resonant nautical bell chime on dock
    } else {
        boat.setDocked(false);
        audio.playBuoyBell(boat.getPosition());
    }
}

void Harbor::update(float dt, Boat &boat, const Input &input, AudioEngine &audio) {
    m_hudTime += dt;
    glm::vec3 boatPos = boat.getPosition();
    m_distToPier = getDistanceToPier(boatPos);
    float speed = boat.getSpeed();

    // Forgiving docking zone: anywhere along pier within 16 meters at speed < 2.8 m/s (~5.5 knots)
    m_canDock = (m_distToPier < 16.0f && std::abs(speed) < 2.8f);

    // Toggle Docking via key 'E'
    bool eDown = input.isKeyDown(GLFW_KEY_E);
    if (eDown && !m_ePressedLast && (m_canDock || m_isDocked)) {
        toggleDock(boat, audio);
    }
    m_ePressedLast = eDown;

    // Disengage if player accelerates hard with W key
    if (m_isDocked && input.isKeyDown(GLFW_KEY_W)) {
        toggleDock(boat, audio);
    }
}

void Harbor::render(const Shader &shader) const {
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);

    // 1. Weathered Nautical Pier Boardwalk & Approach Ramp
    shader.setInt("uMaterialID", 12);
    m_pierDeckMesh.draw();

    // 2. Timber Pilings Rooted in Seabed
    shader.setInt("uMaterialID", 12);
    m_pilingsMesh.draw();

    // 3. Cast Iron Mooring Bollards & Black Rubber Bumpers
    shader.setInt("uMaterialID", 13);
    m_fittingsMesh.draw();

    // 4. Harbor Pierhead Lanterns & Boardwalk Lamps
    shader.setInt("uMaterialID", 14);
    m_lanternsMesh.draw();

    // 5. Braided Nautical Hemp Mooring Ropes (When docked)
    if (m_isDocked) {
        shader.setInt("uMaterialID", 15);
        m_mooringRopesMesh.draw();
    }
}

void Harbor::renderHUD(int screenWidth, int screenHeight, const Boat &boat) const {
    if (m_distToPier > 48.0f && !m_isDocked) return;

    ImDrawList *drawList = ImGui::GetForegroundDrawList();
    if (!drawList) return;

    float speedKnots = boat.getSpeed() * 1.94384f;
    float bannerWidth = 540.0f;
    float bannerHeight = 68.0f;
    float bannerX = (static_cast<float>(screenWidth) - bannerWidth) * 0.5f;
    float bannerY = static_cast<float>(screenHeight) - bannerHeight - 24.0f;

    ImVec2 pMin(bannerX, bannerY);
    ImVec2 pMax(bannerX + bannerWidth, bannerY + bannerHeight);

    if (m_isDocked) {
        // Docked Mode (Cyan/Green Glow)
        drawList->AddRectFilled(pMin, pMax, IM_COL32(8, 32, 24, 235), 10.0f);
        drawList->AddRect(pMin, pMax, IM_COL32(40, 240, 140, 255), 10.0f, 0, 2.5f);

        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 12.0f),
                          IM_COL32(60, 255, 160, 255),
                          "[✓] DA NEO TAU AN TOAN TAI CAU CANG (MOORED)");
        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 38.0f),
                          IM_COL32(230, 245, 230, 240),
                          ">> Nhan [E] hoac vao ga [W] de thao day thung khoi hanh <<");
    }
    else if (m_canDock) {
        // Ready to Dock (Pulsing Gold / Emerald Alert)
        float pulse = 0.7f + 0.3f * std::sin(m_hudTime * 6.0f);
        ImU32 borderCol = IM_COL32(static_cast<int>(255 * pulse), static_cast<int>(215 * pulse), 40, 255);

        drawList->AddRectFilled(pMin, pMax, IM_COL32(32, 28, 6, 235), 10.0f);
        drawList->AddRect(pMin, pMax, borderCol, 10.0f, 0, 3.0f);

        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 12.0f),
                          IM_COL32(255, 230, 60, 255),
                          "[!] DA VAO VI TRI CAP BEN -- BAM [E] DE BUOC DAY NEO TAU!");
        char subText[128];
        std::snprintf(subText, sizeof(subText), "Cach cau cang: %.1fm | Toc do: %.1f kts (Dat chuan cap ben an toan)", m_distToPier, speedKnots);
        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 38.0f),
                          IM_COL32(220, 225, 210, 240), subText);
    }
    else {
        // Approaching Pier Guide (< 48m)
        drawList->AddRectFilled(pMin, pMax, IM_COL32(12, 18, 26, 210), 10.0f);
        drawList->AddRect(pMin, pMax, IM_COL32(60, 140, 200, 180), 10.0f, 0, 1.5f);

        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 12.0f),
                          IM_COL32(100, 200, 255, 255),
                          "⚓ DANG TIEN VAO KHU VUC CAU CANG (HARBOR APPROACH)");
        char subText[128];
        std::snprintf(subText, sizeof(subText), "Khoang cach: %.1fm | Toc do hien tai: %.1f kts (Giam toc < 5.5 kts de neo)", m_distToPier, speedKnots);
        drawList->AddText(ImVec2(bannerX + 16.0f, bannerY + 38.0f),
                          IM_COL32(180, 200, 220, 220), subText);
    }
}

// -----------------------------------------------------------------------------
// Procedural Geometry Construction
// -----------------------------------------------------------------------------

void Harbor::buildGeometry() {
    std::vector<glm::vec3> deckV, pilingsV, fittingsV, lanternsV;
    std::vector<unsigned int> deckI, pilingsI, fittingsI, lanternsI;

    float pierY = 1.15f;
    float pierWidth = 4.6f;
    float pierZ = m_centerPos.z;

    // --- 1. PIER DECK BOARDWALK & ISLAND RAMP ---
    // Main Pier Walkway (X from 78m down to 44m, length 34m)
    Mesh::addCube(deckV, deckI, glm::vec3(m_centerPos.x - 24.0f, pierY, pierZ),
                  glm::vec3(34.0f, 0.24f, pierWidth));

    // Pierhead L-Shaped Breakwater Extension (Protective arm at west end)
    Mesh::addCube(deckV, deckI, glm::vec3(m_centerPos.x - 41.0f, pierY, pierZ - 3.8f),
                  glm::vec3(pierWidth, 0.24f, 8.0f));

    // Wooden Boardwalk Ramp connecting island shore to pier
    Mesh::addCube(deckV, deckI, glm::vec3(m_centerPos.x - 4.0f, pierY + 0.35f, pierZ),
                  glm::vec3(6.5f, 0.24f, 3.8f));

    // --- 2. TIMBER PILINGS (Sturdy tree-trunk stilts into seabed) ---
    for (float x = m_centerPos.x - 8.0f; x >= m_centerPos.x - 40.0f; x -= 5.5f) {
        // South row (docking side)
        Mesh::addCube(pilingsV, pilingsI, glm::vec3(x, -2.5f, pierZ + 2.05f),
                      glm::vec3(0.36f, 7.5f, 0.36f));
        // North row
        Mesh::addCube(pilingsV, pilingsI, glm::vec3(x, -2.5f, pierZ - 2.05f),
                      glm::vec3(0.36f, 7.5f, 0.36f));
        // Underdeck support crossbeam
        Mesh::addCube(pilingsV, pilingsI, glm::vec3(x, pierY - 0.25f, pierZ),
                      glm::vec3(0.28f, 0.28f, 4.4f));
    }
    // L-extension pilings
    for (float z = pierZ - 2.0f; z >= pierZ - 7.5f; z -= 3.0f) {
        Mesh::addCube(pilingsV, pilingsI, glm::vec3(m_centerPos.x - 43.0f, -2.5f, z),
                      glm::vec3(0.36f, 7.5f, 0.36f));
        Mesh::addCube(pilingsV, pilingsI, glm::vec3(m_centerPos.x - 39.0f, -2.5f, z),
                      glm::vec3(0.36f, 7.5f, 0.36f));
    }

    // --- 3. MOORING BOLLARDS & RUBBER BUMPER TIRES ---
    for (float x = m_centerPos.x - 12.0f; x >= m_centerPos.x - 36.0f; x -= 7.0f) {
        // South Cast Iron Mooring Bollard (Base + Post + Cross-pin)
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY + 0.30f, pierZ + 2.10f),
                      glm::vec3(0.24f, 0.36f, 0.24f));
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY + 0.42f, pierZ + 2.10f),
                      glm::vec3(0.38f, 0.10f, 0.14f));

        // North Cast Iron Mooring Bollard
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY + 0.30f, pierZ - 2.10f),
                      glm::vec3(0.24f, 0.36f, 0.24f));
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY + 0.42f, pierZ - 2.10f),
                      glm::vec3(0.38f, 0.10f, 0.14f));

        // Black Rubber Dock Fender Bumpers (Absorbs boat contact)
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY - 0.45f, pierZ + 2.38f),
                      glm::vec3(0.48f, 0.65f, 0.22f));
        Mesh::addCube(fittingsV, fittingsI, glm::vec3(x, pierY - 0.45f, pierZ - 2.38f),
                      glm::vec3(0.48f, 0.65f, 0.22f));
    }

    // Safety Railings along the island approach ramp
    Mesh::addCube(fittingsV, fittingsI, glm::vec3(m_centerPos.x - 4.0f, pierY + 1.15f, pierZ + 1.9f),
                  glm::vec3(6.5f, 0.04f, 0.04f));
    Mesh::addCube(fittingsV, fittingsI, glm::vec3(m_centerPos.x - 4.0f, pierY + 1.15f, pierZ - 1.9f),
                  glm::vec3(6.5f, 0.04f, 0.04f));

    // --- 4. PIERHEAD LANTERNS & WARM BOARDWALK POST LAMPS ---
    // Pierhead Green Starboard Navigation Beacon
    Mesh::addCube(lanternsV, lanternsI, glm::vec3(m_centerPos.x - 41.0f, pierY + 1.1f, pierZ - 7.5f),
                  glm::vec3(0.18f, 1.8f, 0.18f)); // Post
    Mesh::addCube(lanternsV, lanternsI, glm::vec3(m_centerPos.x - 41.0f, pierY + 2.1f, pierZ - 7.5f),
                  glm::vec3(0.32f, 0.45f, 0.32f)); // Beacon Housing

    // Warm Amber Boardwalk Post Lamps
    float lampX[] = { m_centerPos.x - 14.0f, m_centerPos.x - 26.0f, m_centerPos.x - 38.0f };
    for (float lx : lampX) {
        Mesh::addCube(lanternsV, lanternsI, glm::vec3(lx, pierY + 0.9f, pierZ - 2.1f),
                      glm::vec3(0.12f, 1.5f, 0.12f)); // Post
        Mesh::addCube(lanternsV, lanternsI, glm::vec3(lx, pierY + 1.7f, pierZ - 2.1f),
                      glm::vec3(0.24f, 0.30f, 0.24f)); // Lamp
    }

    // 3 Mooring Slip Marker Buoys (Emissive Amber/Green channel guides along calm south berth)
    float guideX[] = { m_centerPos.x - 14.0f, m_centerPos.x - 26.0f, m_centerPos.x - 36.0f };
    for (float gx : guideX) {
        Mesh::addCube(lanternsV, lanternsI, glm::vec3(gx, 0.15f, pierZ + 6.8f),
                      glm::vec3(0.40f, 0.35f, 0.40f)); // Floating buoy
        Mesh::addCube(lanternsV, lanternsI, glm::vec3(gx, 0.45f, pierZ + 6.8f),
                      glm::vec3(0.18f, 0.25f, 0.18f)); // Glowing top light
    }

    m_pierDeckMesh.upload(deckV, deckI);
    m_pilingsMesh.upload(pilingsV, pilingsI);
    m_fittingsMesh.upload(fittingsV, fittingsI);
    m_lanternsMesh.upload(lanternsV, lanternsI);
}

void Harbor::buildMooringRopes(const glm::vec3 &boatPos, float boatHeading) {
    std::vector<glm::vec3> ropeV;
    std::vector<unsigned int> ropeI;

    bool southSide = (boatPos.z >= m_centerPos.z);
    float bollardZ = southSide ? (m_centerPos.z + 2.10f) : (m_centerPos.z - 2.10f);

    // Bow mooring line: connects pier bollard near boat bow
    glm::vec3 pierBollardBow(boatPos.x + 4.2f, 1.48f, bollardZ);
    glm::vec3 boatBowCleat = boatPos + glm::vec3(std::sin(boatHeading) * 3.8f, 0.85f, std::cos(boatHeading) * 3.8f);

    // Stern mooring line: connects pier bollard near boat stern
    glm::vec3 pierBollardStern(boatPos.x - 4.2f, 1.48f, bollardZ);
    glm::vec3 boatSternCleat = boatPos + glm::vec3(-std::sin(boatHeading) * 3.8f, 0.75f, -std::cos(boatHeading) * 3.8f);

    // Add thick braided rope segments
    Mesh::addCube(ropeV, ropeI, (pierBollardBow + boatBowCleat) * 0.5f,
                  glm::vec3(glm::length(pierBollardBow - boatBowCleat), 0.05f, 0.05f));
    Mesh::addCube(ropeV, ropeI, (pierBollardStern + boatSternCleat) * 0.5f,
                  glm::vec3(glm::length(pierBollardStern - boatSternCleat), 0.05f, 0.05f));

    m_mooringRopesMesh.upload(ropeV, ropeI);
}
