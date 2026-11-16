#include "GameManager.h"
#include <iostream>
#include <cmath>

GameManager::GameManager()
    : m_playMode(GamePlayMode::FREE_ROAM),
      m_state(GameState::PLAYING),
      m_cameraMode(CameraMode::THIRD_PERSON),
      m_gameTimer(60.0f),
      m_score(0),
      m_targetScore(10),
      m_waveHeight(1.2f),
      m_timeSpeed(1.0f),
      m_oceanResolution(240),
      m_beaufortScale(4) {}

GameManager::~GameManager() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void GameManager::init(GLFWwindow *window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void GameManager::cycleCameraMode() {
    if (m_cameraMode == CameraMode::THIRD_PERSON) {
        m_cameraMode = CameraMode::COCKPIT;
    } else if (m_cameraMode == CameraMode::COCKPIT) {
        m_cameraMode = CameraMode::ORBIT;
    } else {
        m_cameraMode = CameraMode::THIRD_PERSON;
    }
}

void GameManager::startMission(Boat &boat, CargoManager &cargo) {
    m_playMode = GamePlayMode::MISSION;
    m_state = GameState::PLAYING;
    m_gameTimer = 60.0f;
    m_score = 0;
    cargo.reset();
}

void GameManager::switchToFreeRoam() {
    m_playMode = GamePlayMode::FREE_ROAM;
    m_state = GameState::PLAYING;
}

void GameManager::update(float dt, Boat &boat, CargoManager &cargo) {
    if (m_playMode == GamePlayMode::MISSION && m_state == GameState::PLAYING) {
        m_gameTimer -= dt;

        int pickedUp = cargo.checkCollisions(boat.getPosition(), 3.8f);
        m_score += pickedUp;

        if (m_score >= m_targetScore) {
            m_state = GameState::WON;
        } else if (m_gameTimer <= 0.0f) {
            m_state = GameState::LOST;
        }
    }
}

void GameManager::renderUI(Boat &boat, CargoManager &cargo, OceanMesh &ocean, Sky &sky,
                           AudioEngine &audio, const Lighthouse &lighthouse, const BuoyManager &buoyManager,
                           WeatherSystem &weather, const Harbor &harbor, bool isUnderwater, float underwaterDepth) {
    // -------------------------------------------------------------------------
    // WINDOW 1: Simulation & Environment Control Panel
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 620), ImGuiCond_FirstUseEver);
    ImGui::Begin("Ocean Simulation & Ship Control");

    // 1. Day-Night Cycle
    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "1. Chu ky Ngay - Dem (Day-Night Cycle):");
    float tod = sky.getTimeOfDay();
    if (ImGui::SliderFloat("Gio trong ngay", &tod, 0.0f, 24.0f, "%.1f h")) {
        sky.setTimeOfDay(tod);
    }

    if (ImGui::Button("Trua (13h)")) sky.setTimeOfDay(13.5f);
    ImGui::SameLine();
    if (ImGui::Button("Hoang hon (18h)")) sky.setTimeOfDay(18.0f);
    ImGui::SameLine();
    if (ImGui::Button("Dem trang (23h)")) sky.setTimeOfDay(23.0f);
    ImGui::SameLine();
    if (ImGui::Button("Binh minh (6h)")) sky.setTimeOfDay(6.2f);

    ImGui::Separator();

    // 2. Weather & Windshield Droplets / Wiper (Features 1 & 2)
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "2. Thoi tiet & Gat nuoc (Weather & Wiper - Phim M / K):");
    int weatherIdx = static_cast<int>(weather.getWeatherType());
    const char *weatherOptions[] = {
        "Nang dep / Quang dang (Clear)",
        "Am u / Nhieu may (Overcast)",
        "Mua rao & Gon song mua (Rain)",
        "Bao to & Sam chop (Storm)"
    };
    if (ImGui::Combo("Thoi tiet (M)", &weatherIdx, weatherOptions, 4)) {
        weather.setWeatherType(static_cast<WeatherType>(weatherIdx));
    }
    if (weather.isLightningActive()) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "[!] SAM CHOP DANG DANH!");
    }

    bool wiper = weather.isWiperActive();
    if (ImGui::Checkbox("Can gat nuoc kinh (Wiper - Phim K)", &wiper)) {
        weather.setWiperActive(wiper);
    }
    float wetness = weather.getWetness();
    if (ImGui::SliderFloat("Do uot kinh (Wetness)", &wetness, 0.0f, 1.0f, "%.2f")) {
        weather.setWetness(wetness);
    }
    if (ImGui::Button("Tat nuoc len kinh (Spray)", ImVec2(165, 24))) {
        weather.addWetness(0.55f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Lau sach (Dry)", ImVec2(165, 24))) {
        weather.setWetness(0.0f);
    }

    if (isUnderwater) {
        ImGui::TextColored(ImVec4(0.1f, 0.95f, 0.85f, 1.0f), "[~] DANG LAN DUOI BIEN: -%.2f m", underwaterDepth);
    }

    ImGui::Separator();

    // 3. Beaufort Sea State
    ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.9f, 1.0f), "3. Cap do bien (Beaufort Scale):");
    const char *beaufortNames[] = {
        "Cap 0: Bien lang nhu guong (0.15m)",
        "Cap 2: Gio thoang song gon (0.55m)",
        "Cap 4: Gio vua song luot cano (1.25m)",
        "Cap 6: Gio manh bot trang (2.40m)",
        "Cap 8: Bao to song cuon (4.0m)",
        "Cap 10: Cuong phong song than (6.0m)"
    };
    int beaufortIdx = m_beaufortScale / 2;
    if (ImGui::Combo("Trang thai bien", &beaufortIdx, beaufortNames, 6)) {
        m_beaufortScale = beaufortIdx * 2;
        float heights[] = { 0.15f, 0.55f, 1.25f, 2.40f, 4.0f, 6.0f };
        m_waveHeight = heights[beaufortIdx];
    }
    ImGui::SliderFloat("Chieu cao song thu cong", &m_waveHeight, 0.05f, 8.0f, "%.2f m");

    ImGui::Separator();

    // 4. Audio System Controls
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "4. Am thanh Song dong (Audio System):");
    bool muted = audio.isMuted();
    if (ImGui::Checkbox("Tat am thanh (Mute)", &muted)) {
        audio.toggleMute();
    }
    float masterVol = audio.getMasterVolume();
    if (ImGui::SliderFloat("Am luong tong (Master)", &masterVol, 0.0f, 1.0f, "%.2f")) {
        audio.setMasterVolume(masterVol);
    }
    float engineVol = audio.getEngineVolume();
    if (ImGui::SliderFloat("Tieng dong co (Engine)", &engineVol, 0.0f, 1.0f, "%.2f")) {
        audio.setEngineVolume(engineVol);
    }
    float oceanVol = audio.getAmbientVolume();
    if (ImGui::SliderFloat("Tieng song & gio (Ocean/Wind)", &oceanVol, 0.0f, 1.0f, "%.2f")) {
        audio.setAmbientVolume(oceanVol);
    }
    float sfxVol = audio.getSfxVolume();
    if (ImGui::SliderFloat("Tieng vo song/chuong (SFX)", &sfxVol, 0.0f, 1.0f, "%.2f")) {
        audio.setSfxVolume(sfxVol);
    }

    ImGui::Separator();

    // 5. Camera View & Boat Features
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "5. Goc nhin & Thiet bi (Hotkeys):");
    ImGui::Text("Phim C: Goc nhin | Phim E: Neo tau | Phim R: Radar | Phim F: Den | Phim M: Thoi tiet | Phim K: Gat nuoc");
    int camIdx = static_cast<int>(m_cameraMode);
    const char *camModes[] = { "Nguoi thu ba (Third-Person)", "Buong lai (Cockpit FPS)", "Xoay tu do (Orbit)" };
    if (ImGui::Combo("Goc nhin", &camIdx, camModes, 3)) {
        m_cameraMode = static_cast<CameraMode>(camIdx);
    }

    bool spot = boat.isSpotlightOn();
    if (ImGui::Checkbox("Bat den pha chieu bien (Spotlight - Phim F)", &spot)) {
        boat.setSpotlight(spot);
    }

    ImGui::Separator();

    // 6. Hydrodynamics Telemetry
    ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "6. Vat ly noi (Hydrodynamics):");
    auto &params = boat.getPhysics().getParams();
    ImGui::SliderFloat("Khoi luong (Mass)", &params.mass, 500.0f, 3000.0f, "%.0f kg");
    ImGui::SliderFloat("Nang mui khi ga (Bow Rise)", &params.bowRiseFactor, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Nghieng man khi re (Turn Bank)", &params.turnBankFactor, 0.0f, 3.0f, "%.2f");

    if (ImGui::Button("Reset Thuyen ve toa do 0", ImVec2(240, 26))) {
        boat.reset();
    }
    ImGui::End();

    // -------------------------------------------------------------------------
    // WINDOW 2: Navigation Telemetry & Mission Control HUD
    // -------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 185), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Once);
    ImGui::Begin("NAVIGATION & MISSION CONTROL");

    // Compass & Speed Calculations
    float speedKnots = boat.getSpeed() * 1.94384f;
    float speedKmh   = boat.getSpeed() * 3.6f;

    float headingDeg = glm::degrees(boat.getHeading());
    while (headingDeg < 0.0f)   headingDeg += 360.0f;
    while (headingDeg >= 360.0f) headingDeg -= 360.0f;

    const char *compassPoints[] = { "Bac (N)", "Dong Bac (NE)", "Dong (E)", "Dong Nam (SE)",
                                    "Nam (S)", "Tay Nam (SW)", "Tay (W)", "Tay Bac (NW)" };
    int compassIdx = static_cast<int>((headingDeg + 22.5f) / 45.0f) % 8;

    glm::vec3 boatPos = boat.getPosition();
    float distToLighthouse = glm::length(glm::vec2(boatPos.x - lighthouse.getPosition().x, boatPos.z - lighthouse.getPosition().z));
    float distToBuoy = buoyManager.getDistanceToNearestBuoy(boatPos);

    if (m_playMode == GamePlayMode::FREE_ROAM) {
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "[ CHE DO: LAI TU DO KHAM PHA ]");
        ImGui::Text("Toc do: %.1f knots (%.1f km/h)", speedKnots, speedKmh);
        ImGui::Text("Huong la ban: %.0f deg - %s", headingDeg, compassPoints[compassIdx]);
        ImGui::Text("Toa do: X: %+.1f m | Z: %+.1f m", boatPos.x, boatPos.z);
        float distToHarbor = harbor.getDistanceToPier(boatPos);
        ImGui::Text("Hai Dang: %.1fm | Cau Cang: %.1fm | Dat Lien: %.1fm", distToLighthouse, distToHarbor, std::max(0.0f, 85.0f - boatPos.x));

        if (harbor.isDocked()) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "[✓] DA NEO TAU TAI CAU CANG (MOORED)");
            ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.3f, 1.0f), ">> Nhan [E] hoac [W] de thao day tiep tuc hanh trinh <<");
        } else if (harbor.canDock()) {
            ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.2f, 1.0f), "[!] DA VAO VI TRI CAP BEN - BAM [E] DE NEO TAU!");
        } else if (distToHarbor < 45.0f) {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), ">> Dang tien vao cau cang (Giam toc < 5.5 kts)");
        }

        ImGui::Separator();
        if (ImGui::Button("THU THACH NHIEM VU (MISSION CHALLENGE)", ImVec2(330, 32))) {
            startMission(boat, cargo);
        }
    } else {
        // MISSION MODE
        ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.2f, 1.0f), "[ CHE DO: THU THAP 10 THUNG HANG ]");

        if (m_state == GameState::PLAYING) {
            ImGui::Text("THOI GIAN CON LAI: %.1f s", std::max(0.0f, m_gameTimer));
            float timeFraction = glm::clamp(m_gameTimer / 60.0f, 0.0f, 1.0f);
            ImGui::ProgressBar(timeFraction, ImVec2(330, 16));

            ImGui::Text("TIEN DO: %d / %d THUNG HANG", m_score, m_targetScore);
            ImGui::ProgressBar(static_cast<float>(m_score) / static_cast<float>(m_targetScore), ImVec2(330, 16));

            if (ImGui::Button("HUY / QUAY LAI LAI TU DO", ImVec2(330, 26))) {
                switchToFreeRoam();
            }
        } else if (m_state == GameState::WON) {
            ImGui::TextColored(ImVec4(0.1f, 1.0f, 0.2f, 1.0f), "CHUC MUNG! BAN DA THANG!");
            ImGui::Text("Thoi gian con du: %.1f s", m_gameTimer);
            if (ImGui::Button("CHOI LAI (REPLAY)", ImVec2(160, 30))) {
                startMission(boat, cargo);
            }
            ImGui::SameLine();
            if (ImGui::Button("LAI TU DO", ImVec2(160, 30))) {
                switchToFreeRoam();
            }
        } else if (m_state == GameState::LOST) {
            ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "HET GIO! NHIEM VU THAT BAI.");
            if (ImGui::Button("THU LAI (RETRY)", ImVec2(160, 30))) {
                startMission(boat, cargo);
            }
            ImGui::SameLine();
            if (ImGui::Button("LAI TU DO", ImVec2(160, 30))) {
                switchToFreeRoam();
            }
        }
    }

    ImGui::End();
}
