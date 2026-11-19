#include "Radar.h"
#include "Boat.h"
#include "Lighthouse.h"
#include "Buoy.h"
#include "Cargo.h"
#include "Harbor.h"
#include "../audio/AudioEngine.h"

#include <cmath>
#include <cstdio>
#include <algorithm>

static constexpr float PI = 3.14159265358979323846f;

MarineRadar::MarineRadar()
    : m_isOpen(false),
      m_headingUp(true),
      m_sweepAngle(0.0f),
      m_sweepSpeed(2.4f), // ~23 RPM
      m_rangeIdx(1),      // Default 120m
      m_pingCooldown(0.0f) {}

void MarineRadar::cycleRange() {
    m_rangeIdx = (m_rangeIdx + 1) % 4;
}

void MarineRadar::update(float dt, AudioEngine &audio, const Boat &boat, const Lighthouse &lighthouse,
                         const BuoyManager &buoyManager, const CargoManager &cargo) {
    if (!m_isOpen) return;

    m_sweepAngle += dt * m_sweepSpeed;
    if (m_sweepAngle > 2.0f * PI) {
        m_sweepAngle -= 2.0f * PI;
    }

    m_pingCooldown -= dt;

    // Check if sweep beam passes near any target, play sonar ping
    float currentRange = m_ranges[m_rangeIdx];
    glm::vec3 boatPos = boat.getPosition();
    float boatHeading = boat.getHeading();

    auto checkTargetPing = [&](const glm::vec3 &targetPos) {
        glm::vec2 delta(targetPos.x - boatPos.x, targetPos.z - boatPos.z);
        float dist = glm::length(delta);
        if (dist > currentRange || dist < 2.0f) return;

        float targetAngle = std::atan2(delta.x, -delta.y); // Angle from forward
        if (m_headingUp) {
            targetAngle -= boatHeading;
        }
        while (targetAngle < 0.0f) targetAngle += 2.0f * PI;
        while (targetAngle >= 2.0f * PI) targetAngle -= 2.0f * PI;

        float angleDiff = std::abs(targetAngle - m_sweepAngle);
        if (angleDiff > PI) angleDiff = 2.0f * PI - angleDiff;

        if (angleDiff < 0.08f && m_pingCooldown <= 0.0f) {
            audio.playRadarPing();
            m_pingCooldown = 0.45f;
        }
    };

    checkTargetPing(lighthouse.getPosition());
    for (const auto &buoy : buoyManager.getBuoys()) {
        checkTargetPing(buoy.getPosition());
    }
}

void MarineRadar::renderUI(const Boat &boat, const Lighthouse &lighthouse,
                           const BuoyManager &buoyManager, const CargoManager &cargo,
                           const Harbor *harbor) {
    if (!m_isOpen) return;

    ImGui::SetNextWindowPos(ImVec2(std::max(10.0f, ImGui::GetIO().DisplaySize.x - 325), 14), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(std::min(315.0f, ImGui::GetIO().DisplaySize.x - 20), std::min(385.0f, ImGui::GetIO().DisplaySize.y - 28)), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse;
    if (!ImGui::Begin("MARINE RADAR (ARPA / ECDIS)", &m_isOpen, flags)) {
        ImGui::End();
        return;
    }

    float currentRange = m_ranges[m_rangeIdx];

    // Controls line
    ImGui::Text("RANGE:");
    ImGui::SameLine();
    for (int i = 0; i < 4; ++i) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.0fm", m_ranges[i]);
        bool selected = i == m_rangeIdx;
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.6f, 0.3f, 1.0f));
        }
        if (ImGui::Button(buf)) {
            m_rangeIdx = i;
        }
        if (selected) {
            ImGui::PopStyleColor();
        }
        ImGui::SameLine();
    }

    ImGui::NewLine();
    if (ImGui::Button(m_headingUp ? "HEAD-UP" : "NORTH-UP")) {
        m_headingUp = !m_headingUp;
    }

    ImGui::Separator();

    // Canvas drawing
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    float canvasSize = std::max(120.0f, std::min(250.0f, ImGui::GetContentRegionAvail().x));
    ImVec2 center(canvasPos.x + canvasSize * 0.5f, canvasPos.y + canvasSize * 0.5f);
    float radarRadius = canvasSize * 0.46f;

    // 1. Radar Screen Dark Phosphor Background
    drawList->AddCircleFilled(center, radarRadius, IM_COL32(4, 18, 14, 250), 48);
    drawList->AddCircle(center, radarRadius, IM_COL32(30, 95, 65, 255), 48, 2.5f);

    // 2. Concentric Range Rings & Labels
    const int NUM_RINGS = 4;
    for (int r = 1; r <= NUM_RINGS; ++r) {
        float ringFraction = static_cast<float>(r) / NUM_RINGS;
        float rRadius = radarRadius * ringFraction;
        drawList->AddCircle(center, rRadius, IM_COL32(20, 80, 55, 160), 36, 1.0f);

        char distLabel[16];
        std::snprintf(distLabel, sizeof(distLabel), "%.0fm", currentRange * ringFraction);
        drawList->AddText(ImVec2(center.x + 3.0f, center.y - rRadius + 1.0f), IM_COL32(35, 140, 90, 180), distLabel);
    }

    // 3. Compass Crosshairs & Degree Markings
    drawList->AddLine(ImVec2(center.x - radarRadius, center.y), ImVec2(center.x + radarRadius, center.y),
                      IM_COL32(20, 80, 55, 120), 1.0f);
    drawList->AddLine(ImVec2(center.x, center.y - radarRadius), ImVec2(center.x, center.y + radarRadius),
                      IM_COL32(20, 80, 55, 120), 1.0f);

    // Bearing labels around the rim
    const char *bearings[] = { "N", "045", "E", "135", "S", "225", "W", "315" };
    float boatHeading = boat.getHeading();
    for (int i = 0; i < 8; ++i) {
        float angle = i * (PI * 0.25f);
        if (m_headingUp) {
            angle -= boatHeading;
        }
        float lx = center.x + std::sin(angle) * (radarRadius - 10.0f);
        float ly = center.y - std::cos(angle) * (radarRadius - 10.0f);
        drawList->AddText(ImVec2(lx - 6.0f, ly - 6.0f), IM_COL32(50, 180, 120, 220), bearings[i]);
    }

    // 4. Phosphor Sweep Beam & Sector Afterglow
    const int AFTERGLOW_STEPS = 14;
    for (int step = 0; step < AFTERGLOW_STEPS; ++step) {
        float lagAngle = m_sweepAngle - (static_cast<float>(step) / AFTERGLOW_STEPS) * (PI * 0.25f);
        float alpha = (1.0f - static_cast<float>(step) / AFTERGLOW_STEPS) * 120.0f;
        ImVec2 pEdge(center.x + std::sin(lagAngle) * radarRadius, center.y - std::cos(lagAngle) * radarRadius);
        drawList->AddLine(center, pEdge, IM_COL32(40, 240, 120, static_cast<int>(alpha)), 1.5f);
    }
    // Main leading sweep line
    ImVec2 sweepTip(center.x + std::sin(m_sweepAngle) * radarRadius, center.y - std::cos(m_sweepAngle) * radarRadius);
    drawList->AddLine(center, sweepTip, IM_COL32(180, 255, 200, 255), 2.0f);

    // 5. Target Transform Helper
    glm::vec3 boatPos = boat.getPosition();
    int visibleTargets = 0;

    auto worldToRadar = [&](const glm::vec3 &targetPos, ImVec2 &outScreen) -> bool {
        glm::vec2 delta(targetPos.x - boatPos.x, targetPos.z - boatPos.z);
        float dist = glm::length(delta);
        if (dist > currentRange) return false;

        float angle = std::atan2(delta.x, -delta.y); // Relative to world North
        if (m_headingUp) {
            angle -= boatHeading; // Relative to boat bow
        }

        float rPix = (dist / currentRange) * radarRadius;
        outScreen.x = center.x + std::sin(angle) * rPix;
        outScreen.y = center.y - std::cos(angle) * rPix;
        return true;
    };

    // 6. Draw Targets
    // 6a. Rocky Islet & Lighthouse Echo
    ImVec2 lhScreen;
    if (worldToRadar(lighthouse.getPosition(), lhScreen)) {
        // Draw island echo cluster
        drawList->AddCircleFilled(lhScreen, 9.0f, IM_COL32(230, 140, 30, 160));
        drawList->AddCircle(lhScreen, 12.0f, IM_COL32(255, 180, 50, 220), 16, 1.5f);
        drawList->AddCircleFilled(lhScreen, 3.5f, IM_COL32(255, 255, 255, 255));
        drawList->AddText(ImVec2(lhScreen.x + 10.0f, lhScreen.y - 6.0f), IM_COL32(255, 200, 80, 240), "HAI DANG");
        visibleTargets++;
    }

    // 6a2. Harbor Pier & Mooring Berth Echo
    if (harbor) {
        ImVec2 berthScreen;
        if (worldToRadar(harbor->getBerthPosition(), berthScreen)) {
            // Draw pier structure line toward island
            if (worldToRadar(lighthouse.getPosition() + glm::vec3(-12.0f, 0.0f, 0.0f), lhScreen)) {
                drawList->AddLine(lhScreen, berthScreen, IM_COL32(80, 220, 180, 230), 2.5f);
            }
            ImU32 dockCol = harbor->isDocked() ? IM_COL32(50, 255, 120, 255) : IM_COL32(80, 220, 255, 220);
            drawList->AddCircleFilled(berthScreen, 4.5f, dockCol);
            drawList->AddCircle(berthScreen, 7.5f, dockCol, 12, 1.2f);
            drawList->AddText(ImVec2(berthScreen.x + 8.0f, berthScreen.y - 6.0f), dockCol, harbor->isDocked() ? "BEN TAU [MOORED]" : "BEN TAU");
            visibleTargets++;
        }
    }

    // 6a3. Mainland Coastline Contour (Green radar land echo)
    static const glm::vec3 coastPts[] = {
        { 74.0f, 0.0f, -60.0f }, { 78.0f, 0.0f, -85.0f }, { 84.0f, 0.0f, -105.0f },
        { 78.0f, 0.0f, -135.0f }, { 70.0f, 0.0f, -170.0f }, { 74.0f, 0.0f, -210.0f },
        { 82.0f, 0.0f, -260.0f }
    };
    ImVec2 prevScreen;
    bool hasPrev = false;
    for (int p = 0; p < 7; ++p) {
        ImVec2 curScreen;
        if (worldToRadar(coastPts[p], curScreen)) {
            if (hasPrev) {
                drawList->AddLine(prevScreen, curScreen, IM_COL32(40, 180, 80, 180), 2.0f);
            }
            prevScreen = curScreen;
            hasPrev = true;
        } else {
            hasPrev = false;
        }
    }
    ImVec2 coastLabelPos;
    if (worldToRadar(glm::vec3(120.0f, 0.0f, -125.0f), coastLabelPos)) {
        drawList->AddText(ImVec2(coastLabelPos.x - 20.0f, coastLabelPos.y - 6.0f),
                          IM_COL32(50, 200, 90, 220), "[DAT LIEN]");
    }

    // 6b. Navigational Buoys
    for (const auto &buoy : buoyManager.getBuoys()) {
        ImVec2 bScreen;
        if (worldToRadar(buoy.getPosition(), bScreen)) {
            ImU32 bCol = IM_COL32(240, 50, 40, 255); // Red Port
            const char *bTag = "PORT";
            if (buoy.getType() == BuoyType::STARBOARD) {
                bCol = IM_COL32(40, 240, 80, 255); // Green Starboard
                bTag = "STBD";
            } else if (buoy.getType() == BuoyType::DANGER) {
                bCol = IM_COL32(255, 220, 30, 255); // Amber Danger
                bTag = "WARN";
            }

            drawList->AddCircleFilled(bScreen, 4.0f, bCol);
            if (buoy.isLightOn()) {
                drawList->AddCircle(bScreen, 7.5f, bCol, 12, 1.2f);
            }
            drawList->AddText(ImVec2(bScreen.x + 6.0f, bScreen.y - 6.0f), bCol, bTag);
            visibleTargets++;
        }
    }

    // 6c. Cargo Crates
    int cIdx = 0;
    for (const auto &crate : cargo.getItems()) {
        if (!crate.active) continue;
        ImVec2 cScreen;
        if (worldToRadar(crate.pos, cScreen)) {
            drawList->AddRectFilled(ImVec2(cScreen.x - 3.5f, cScreen.y - 3.5f),
                                    ImVec2(cScreen.x + 3.5f, cScreen.y + 3.5f),
                                    IM_COL32(255, 220, 60, 240));
            drawList->AddRect(ImVec2(cScreen.x - 5.5f, cScreen.y - 5.5f),
                              ImVec2(cScreen.x + 5.5f, cScreen.y + 5.5f),
                              IM_COL32(255, 240, 100, 180));
            visibleTargets++;
        }
        cIdx++;
    }

    // 7. Player Boat Icon at Center
    float boatIconAngle = m_headingUp ? 0.0f : boatHeading;
    float cosB = std::cos(boatIconAngle);
    float sinB = std::sin(boatIconAngle);

    ImVec2 bowP(center.x + sinB * 9.0f, center.y - cosB * 9.0f);
    ImVec2 portP(center.x - cosB * 5.0f - sinB * 6.0f, center.y - sinB * 5.0f + cosB * 6.0f);
    ImVec2 stbdP(center.x + cosB * 5.0f - sinB * 6.0f, center.y + sinB * 5.0f + cosB * 6.0f);

    drawList->AddTriangleFilled(bowP, portP, stbdP, IM_COL32(60, 220, 255, 255));
    drawList->AddTriangle(bowP, portP, stbdP, IM_COL32(255, 255, 255, 255), 1.2f);

    // Speed vector line projecting ahead
    float speedVecLen = glm::clamp(boat.getSpeed() * 2.0f, 4.0f, 26.0f);
    ImVec2 speedVecTip(center.x + sinB * (9.0f + speedVecLen), center.y - cosB * (9.0f + speedVecLen));
    drawList->AddLine(bowP, speedVecTip, IM_COL32(60, 220, 255, 200), 1.5f);

    ImGui::Dummy(ImVec2(canvasSize, canvasSize));

    // Telemetry Footer
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.6f, 1.0f), "MUC TIEU TRONG VUNG: %d", visibleTargets);
    ImGui::Text("TOC DO: %.1f kts | HUONG: %.0f deg", boat.getSpeed() * 1.94384f, glm::degrees(boatHeading));

    ImGui::End();
}
