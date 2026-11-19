#pragma once

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "Boat.h"
#include "Cargo.h"
#include "Lighthouse.h"
#include "Harbor.h"
#include "Buoy.h"
#include "Weather.h"
#include "../ocean/OceanMesh.h"
#include "../renderer/Sky.h"
#include "../core/Camera.h"
#include "../audio/AudioEngine.h"

enum class GamePlayMode {
    FREE_ROAM,  // Lái tự do khám phá (Mặc định khi vào game)
    MISSION     // Chế độ thử thách thu thập thùng hàng có đếm ngược
};

enum class GameState {
    MENU,
    PLAYING,
    WON,
    LOST
};

class GameManager {
public:
    GameManager();
    ~GameManager();

    void init(GLFWwindow *window);
    void update(float dt, Boat &boat, CargoManager &cargo);
    void renderUI(Boat &boat, CargoManager &cargo, OceanMesh &ocean, Sky &sky,
                  AudioEngine &audio, const Lighthouse &lighthouse, const BuoyManager &buoyManager,
                  WeatherSystem &weather, const Harbor &harbor, bool isUnderwater = false, float underwaterDepth = 0.0f);

    GamePlayMode getPlayMode() const { return m_playMode; }
    void setPlayMode(GamePlayMode mode) { m_playMode = mode; }

    GameState getState() const { return m_state; }
    void setState(GameState state) { m_state = state; }

    bool isBoatControllable() const {
        return (m_playMode == GamePlayMode::FREE_ROAM) || (m_state == GameState::PLAYING);
    }

    CameraMode getCameraMode() const { return m_cameraMode; }
    void cycleCameraMode();
    void toggleSettings() { m_settingsOpen = !m_settingsOpen; }
    void toggleMissions() { m_missionsOpen = !m_missionsOpen; }
    void setQuality(int quality) { m_quality = glm::clamp(quality,0,2); }
    int getQuality() const { return m_quality; }
    float getExposure() const { return m_exposure; }
    void setCameraMode(CameraMode mode) { m_cameraMode = mode; }

    float getWaveHeight() const { return m_waveHeight; }
    float getTimeSpeed() const { return m_timeSpeed; }
    int getOceanResolution() const { return m_oceanResolution; }
    float getGameTime(float totalTime) const { return totalTime * m_timeSpeed; }

    void startMission(Boat &boat, CargoManager &cargo);
    void switchToFreeRoam();

private:
    bool m_settingsOpen = false;
    bool m_missionsOpen = false;
    int m_quality = 1;
    float m_exposure = 0.95f;
    GamePlayMode m_playMode;
    GameState m_state;
    CameraMode m_cameraMode;
    float m_gameTimer;
    int m_score;
    const int m_targetScore;

    // Simulation GUI parameters
    float m_waveHeight;
    float m_timeSpeed;
    int m_oceanResolution;
    int m_beaufortScale; // 0 (Calm) to 10 (Storm)
};
