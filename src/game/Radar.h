#pragma once

#include <imgui.h>
#include <glm/glm.hpp>
#include <vector>

class Boat;
class Lighthouse;
class BuoyManager;
class CargoManager;
class AudioEngine;
class Harbor;

class MarineRadar {
public:
    MarineRadar();

    void update(float dt, AudioEngine &audio, const Boat &boat, const Lighthouse &lighthouse,
                const BuoyManager &buoyManager, const CargoManager &cargo);
    void renderUI(const Boat &boat, const Lighthouse &lighthouse,
                  const BuoyManager &buoyManager, const CargoManager &cargo,
                  const Harbor *harbor = nullptr);

    void toggle() { m_isOpen = !m_isOpen; }
    bool isOpen() const { return m_isOpen; }
    void setOpen(bool open) { m_isOpen = open; }

    float getRange() const { return m_ranges[m_rangeIdx]; }
    void cycleRange();

private:
    bool m_isOpen;
    bool m_headingUp; // true = Heading-Up (Boat points UP), false = North-Up
    float m_sweepAngle; // Current radial sweep angle in radians
    float m_sweepSpeed;
    int m_rangeIdx;
    static constexpr float m_ranges[4] = { 60.0f, 120.0f, 200.0f, 350.0f };
    float m_pingCooldown;
};
