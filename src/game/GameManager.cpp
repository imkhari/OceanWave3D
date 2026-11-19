#include "GameManager.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>

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
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr; // Layout follows current logical window size, never stale desktop coordinates.

    float sx = 1.0f, sy = 1.0f;
    glfwGetWindowContentScale(window, &sx, &sy);
    float dpi = std::max(1.0f, std::max(sx, sy));
    const char *fonts[] = {
        "assets/fonts/DejaVuSans.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "C:/Windows/Fonts/segoeui.ttf"
    };
    ImFontGlyphRangesBuilder ranges;
    ranges.AddRanges(io.Fonts->GetGlyphRangesDefault());
    ranges.AddRanges(io.Fonts->GetGlyphRangesVietnamese());
    static ImVector<ImWchar> glyphs;
    ranges.BuildRanges(&glyphs);
    bool loaded = false;
    for (auto path : fonts) {
        if (!std::ifstream(path).good()) continue;
        ImFontConfig config; config.OversampleH = 2; config.OversampleV = 2;
        loaded = io.Fonts->AddFontFromFileTTF(path, 16.0f * dpi, &config, glyphs.Data) != nullptr;
        if (loaded) break;
    }
    if (!loaded) std::cerr << "[WARN] Install a Vietnamese font or place DejaVuSans.ttf in assets/fonts\n";
    io.FontGlobalScale = loaded ? 1.0f / dpi : 1.0f;
    auto &style = ImGui::GetStyle();
    style.WindowRounding = 9.0f; style.FrameRounding = 5.0f;
    style.WindowPadding = ImVec2(14,12); style.ItemSpacing = ImVec2(9,8);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.025f,0.055f,0.08f,0.90f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.08f,0.28f,0.36f,1.0f);
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
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    if (m_settingsOpen) {
    ImGui::SetNextWindowPos(ImVec2(14, 150), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(460.0f, display.x - 28), std::max(180.0f, display.y - 180)), ImGuiCond_Always);
    ImGui::Begin("Thiết lập mô phỏng · F1", &m_settingsOpen);
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
    ImGui::Combo("Chất lượng", &m_quality, "Low\0Medium\0High\0");
    ImGui::SliderFloat("Phơi sáng", &m_exposure, 0.45f, 1.5f, "%.2f");
    ImGui::Text("%.1f FPS · %.2f ms", ImGui::GetIO().Framerate, 1000.0f / std::max(1.0f, ImGui::GetIO().Framerate));
    ImGui::Text("X %.1f m · Z %.1f m", boat.getPosition().x, boat.getPosition().z);

    // 1. Day-Night Cycle
    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "Ánh sáng & thời gian");
    float tod = sky.getTimeOfDay();
    if (ImGui::SliderFloat("Giờ trong ngày", &tod, 0.0f, 24.0f, "%.1f h")) {
        sky.setTimeOfDay(tod);
    }

    if (ImGui::Button("Trưa")) sky.setTimeOfDay(13.5f);
    ImGui::SameLine();
    if (ImGui::Button("Hoàng hôn")) sky.setTimeOfDay(18.0f);
    if (ImGui::Button("Đêm")) sky.setTimeOfDay(23.0f);
    ImGui::SameLine();
    if (ImGui::Button("Bình minh")) sky.setTimeOfDay(6.2f);

    ImGui::Separator();

    // 2. Weather & Windshield Droplets / Wiper (Features 1 & 2)
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Thời tiết & kính chắn gió");
    int weatherIdx = static_cast<int>(weather.getWeatherType());
    const char *weatherOptions[] = {
        "Quang đãng",
        "Nhiều mây",
        "Mưa rào",
        "Giông bão"
    };
    if (ImGui::Combo("Thời tiết · M", &weatherIdx, weatherOptions, 4)) {
        weather.setWeatherType(static_cast<WeatherType>(weatherIdx));
    }
    if (weather.isLightningActive()) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.3f, 1.0f), "Đang có sấm chớp");
    }

    bool wiper = weather.isWiperActive();
    if (ImGui::Checkbox("Cần gạt nước · K", &wiper)) {
        weather.setWiperActive(wiper);
    }
    float wetness = weather.getWetness();
    if (ImGui::SliderFloat("Độ ướt kính", &wetness, 0.0f, 1.0f, "%.2f")) {
        weather.setWetness(wetness);
    }
    if (ImGui::Button("Làm ướt kính", ImVec2(165, 24))) {
        weather.addWetness(0.55f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Lau khô", ImVec2(165, 24))) {
        weather.setWetness(0.0f);
    }

    if (isUnderwater) {
        ImGui::TextColored(ImVec4(0.1f, 0.95f, 0.85f, 1.0f), "Độ sâu camera: %.2f m", underwaterDepth);
    }

    ImGui::Separator();

    // 3. Beaufort Sea State
    ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.9f, 1.0f), "Trạng thái biển");
    const char *beaufortNames[] = {
        "Cấp 0 · Lặng",
        "Cấp 2 · Sóng gợn",
        "Cấp 4 · Sóng vừa",
        "Cấp 6 · Gió mạnh",
        "Cấp 8 · Bão",
        "Cấp 10 · Cuồng phong"
    };
    int beaufortIdx = m_beaufortScale / 2;
    if (ImGui::Combo("Trạng thái biển", &beaufortIdx, beaufortNames, 6)) {
        m_beaufortScale = beaufortIdx * 2;
        float heights[] = { 0.15f, 0.55f, 1.25f, 2.40f, 4.0f, 6.0f };
        m_waveHeight = heights[beaufortIdx];
    }
    ImGui::SliderFloat("Biên độ sóng", &m_waveHeight, 0.05f, 8.0f, "%.2f m");

    ImGui::Separator();

    // 4. Audio System Controls
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Âm thanh");
    bool muted = audio.isMuted();
    if (ImGui::Checkbox("Tắt âm thanh", &muted)) {
        audio.toggleMute();
    }
    float masterVol = audio.getMasterVolume();
    if (ImGui::SliderFloat("Âm lượng tổng", &masterVol, 0.0f, 1.0f, "%.2f")) {
        audio.setMasterVolume(masterVol);
    }
    float engineVol = audio.getEngineVolume();
    if (ImGui::SliderFloat("Động cơ", &engineVol, 0.0f, 1.0f, "%.2f")) {
        audio.setEngineVolume(engineVol);
    }
    float oceanVol = audio.getAmbientVolume();
    if (ImGui::SliderFloat("Sóng & gió", &oceanVol, 0.0f, 1.0f, "%.2f")) {
        audio.setAmbientVolume(oceanVol);
    }
    float sfxVol = audio.getSfxVolume();
    if (ImGui::SliderFloat("Hiệu ứng", &sfxVol, 0.0f, 1.0f, "%.2f")) {
        audio.setSfxVolume(sfxVol);
    }

    ImGui::Separator();

    // 5. Camera View & Boat Features
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "Camera & điều khiển");
    ImGui::TextWrapped("C: Camera · E: Neo tàu · R: Radar · F: Đèn · M: Thời tiết · K: Gạt nước · N: Nhiệm vụ · F1: Thiết lập · Q/Z: Zoom orbit");
    int camIdx = static_cast<int>(m_cameraMode);
    const char *camModes[] = { "Theo tàu", "Buồng lái", "Xoay tự do" };
    if (ImGui::Combo("Góc nhìn", &camIdx, camModes, 3)) {
        m_cameraMode = static_cast<CameraMode>(camIdx);
    }

    bool spot = boat.isSpotlightOn();
    if (ImGui::Checkbox("Đèn pha · F", &spot)) {
        boat.setSpotlight(spot);
    }

    ImGui::Separator();

    // 6. Hydrodynamics Telemetry
    ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Vật lý & công cụ");
    auto &params = boat.getPhysics().getParams();
    ImGui::SliderFloat("Khối lượng", &params.mass, 500.0f, 3000.0f, "%.0f kg");
    ImGui::SliderFloat("Nâng mũi khi tăng ga", &params.bowRiseFactor, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("Nghiêng khi rẽ", &params.turnBankFactor, 0.0f, 3.0f, "%.2f");

    if (ImGui::Button("Đặt lại tàu", ImVec2(240, 26))) {
        boat.reset();
    }
    ImGui::PopItemWidth();
    ImGui::End();

    }
    const float heading = std::fmod(glm::degrees(boat.getHeading()) + 36000.0f, 360.0f);
    ImGui::SetNextWindowPos(ImVec2(14,14), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(std::min(310.0f, display.x - 28),0), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.84f);
    ImGui::Begin("Hải trình", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoNav);
    ImGui::TextColored(ImVec4(0.35f,0.85f,0.9f,1), "OCEANWAVE / HẢI TRÌNH");
    ImGui::Text("%.1f kn    |    %03.0f°", boat.getSpeed()*1.94384f, heading);
    if (m_playMode == GamePlayMode::MISSION) {
        ImGui::TextWrapped("Thu hồi hàng: %d/%d · Còn %.0f giây", m_score, m_targetScore, std::max(0.0f,m_gameTimer));
        if (m_state != GameState::PLAYING) ImGui::TextWrapped(m_state == GameState::WON ? "Hoàn thành! Nhấn N để tiếp tục." : "Hết giờ. Nhấn N để thử lại.");
    } else {
        ImGui::Text("Cảng hải đăng · %.0f m", harbor.getDistanceToPier(boat.getPosition()));
    }
    if (harbor.isDocked()) ImGui::TextWrapped("Đã neo an toàn · E/W để khởi hành");
    else if (harbor.canDock()) ImGui::TextWrapped("Nhấn E để neo tàu");
    ImGui::TextDisabled("F1 Thiết lập   N Nhiệm vụ   R Radar");
    ImGui::End();

    if (m_missionsOpen) {
        ImGui::SetNextWindowPos(ImVec2(display.x*0.5f,display.y*0.5f),ImGuiCond_Appearing,ImVec2(0.5f,0.5f));
        ImGui::SetNextWindowSize(ImVec2(std::min(420.0f,display.x-28),0),ImGuiCond_Always);
        ImGui::Begin("Nhiệm vụ · N", &m_missionsOpen, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::TextWrapped("Thu hồi 10 thùng hàng trôi dạt trong 60 giây. Dùng radar để tìm hàng, điều khiển tàu đến gần để thu hồi.");
        if (ImGui::Button("Bắt đầu / Thử lại", ImVec2(-1,32))) { startMission(boat,cargo); m_missionsOpen=false; }
        if (ImGui::Button("Lái tự do",ImVec2(-1,32))) { switchToFreeRoam(); m_missionsOpen=false; }
        ImGui::End();
    }
}
