#pragma once
#include "Window.h"
#include "../game/GameManager.h"
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include "../ocean/FoamSimulator.h"
#include <cmath>

// Optional deterministic capture/smoke harness; normal gameplay never glFinish()es.
struct RenderReview {
    std::string capturePath;
    int frameLimit = 0, reviewView = 0;
    float reviewHour = 13.5f;
    int reviewQuality = 1, reviewWeather = 0;
    bool smoke = false, settings = false, spotlight = false, reviewFailure = false;
    int reviewFrame = 0;
    double reviewMilliseconds = 0.0;
    bool selfTest = false;
    double peakWakeArea = 0.0;
    std::string wakeScenario;
    RenderReview(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--self-test") selfTest = true;
        if (arg == "--smoke") { smoke = true; frameLimit = 360; }
        else if (arg == "--wake-review" && i+1<argc) { wakeScenario=argv[++i]; if (!frameLimit) frameLimit=360; }
        else if (arg == "--settings") settings = true;
        else if (arg == "--spotlight") spotlight = true;
        else if (arg == "--quality" && i+1<argc) reviewQuality = std::atoi(argv[++i]);
        else if (arg == "--weather" && i+1<argc) reviewWeather = std::atoi(argv[++i]);
        else if (arg == "--capture" && i + 1 < argc) capturePath = argv[++i];
        else if (arg == "--frames" && i + 1 < argc) frameLimit = std::max(1, std::atoi(argv[++i]));
        else if (arg == "--view" && i + 1 < argc) reviewView = std::atoi(argv[++i]);
        else if (arg == "--hour" && i + 1 < argc) reviewHour = std::atof(argv[++i]);
    }
    if (!capturePath.empty() && !frameLimit) frameLimit = 120;
    }
    void drive(Input &input, Boat &boat, Harbor &harbor, GameManager &gameManager,
               WeatherSystem &weather, CargoManager &cargo, GLFWwindow *window) {
        auto check = [&](bool passed, const char *label) {
            std::cout << (passed ? "[SMOKE PASS] " : "[SMOKE FAIL] ") << label << "\n";
            reviewFailure |= !passed;
        };
        if (!wakeScenario.empty()) {
            input.replay(true);
            input.setReplayKey(GLFW_KEY_W,wakeScenario != "idle" && wakeScenario != "reverse" && (wakeScenario != "stop" || reviewFrame < 180));
            input.setReplayKey(GLFW_KEY_S,wakeScenario == "reverse");
            input.setReplayKey(GLFW_KEY_A,wakeScenario == "turn" && reviewFrame >= 180 && reviewFrame < 240);
            if (wakeScenario == "stop" && reviewFrame == 180) boat.setDocked(true,boat.getPosition(),boat.getHeading());
        }
        if (smoke) {
            input.replay(true);
            input.setReplayKey(GLFW_KEY_W, (reviewFrame >= 5 && reviewFrame < 70) || reviewFrame == 210);
            input.setReplayKey(GLFW_KEY_A, reviewFrame >= 5 && reviewFrame < 40);
            input.setReplayKey(GLFW_KEY_C, reviewFrame == 75);
            input.setReplayKey(GLFW_KEY_R, reviewFrame == 90);
            input.setReplayKey(GLFW_KEY_M, reviewFrame == 100 || reviewFrame == 110);
            input.setReplayKey(GLFW_KEY_K, reviewFrame == 120);
            input.setReplayKey(GLFW_KEY_F, reviewFrame == 120);
            input.setReplayKey(GLFW_KEY_E, reviewFrame == 191);
            if (reviewFrame == 70) check(boat.getSpeed()>1.0f && std::abs(boat.getHeading()-glm::radians(156.0f))>0.1f,"throttle and steering");
            if (reviewFrame == 76) check(gameManager.getCameraMode()==CameraMode::COCKPIT,"camera shortcut");
            if (reviewFrame == 121) check(boat.isSpotlightOn() && weather.isWiperActive() && weather.getWeatherType()==WeatherType::RAIN,"weather, wiper and spotlight shortcuts");
            if (reviewFrame == 130) glfwSetWindowSize(window,900,600);
            if (reviewFrame == 145) gameManager.toggleSettings();
            if (reviewFrame == 160) gameManager.toggleSettings();
            if (reviewFrame == 170 || reviewFrame == 180) gameManager.toggleMissions();
            if (reviewFrame == 190) boat.getPhysics().setReviewPose(harbor.getBerthPosition(),1.5707963f);
            if (reviewFrame == 192) check(harbor.isDocked() && boat.isDocked(),"dock with E");
            if (reviewFrame == 211) check(!harbor.isDocked() && !boat.isDocked(),"undock with W");
            if (reviewFrame == 230) weather.setWeatherType(WeatherType::STORM);
            if (reviewFrame == 260) { gameManager.startMission(boat,cargo); check(gameManager.getState()==GameState::PLAYING,"start mission"); }
            if (reviewFrame == 280) gameManager.setQuality(0);
            if (reviewFrame == 300) gameManager.setQuality(2);
            if (reviewFrame == 320) glfwSetWindowSize(window,1280,720);
            if (reviewFrame == 340) gameManager.setCameraMode(CameraMode::ORBIT);
        }
    }
    void inspectWake(const FoamSimulator &foam, const Boat &boat) {
        if (wakeScenario.empty() || (reviewFrame!=frameLimit && !(wakeScenario=="stop" && reviewFrame==180))) return;
        GLint active=0,binding=0;
        glGetIntegerv(GL_ACTIVE_TEXTURE,&active); glActiveTexture(GL_TEXTURE0);
        glGetIntegerv(GL_TEXTURE_BINDING_2D,&binding);
        glBindTexture(GL_TEXTURE_2D,foam.getFoamTexture());
        int size=foam.getResolution();
        std::vector<float> pixels(size*size*4);
        glGetTexImage(GL_TEXTURE_2D,0,GL_RGBA,GL_FLOAT,pixels.data());
        glBindTexture(GL_TEXTURE_2D,binding); glActiveTexture(active);
        double left=0,right=0,total=0;
        bool finite=true;
        glm::vec2 lateral(std::cos(boat.getHeading()),-std::sin(boat.getHeading()));
        const double cell=foam.getDomainSize()/size;
        for (int y=0;y<size;++y) for(int x=0;x<size;++x) {
            const float *field=&pixels[(y*size+x)*4];
            for(int k=0;k<4;++k) finite &= std::isfinite(field[k]);
            glm::vec2 world=foam.getDomainCenter()+(glm::vec2(x+0.5f,y+0.5f)/float(size)-0.5f)*foam.getDomainSize();
            double coverage=field[0]*cell*cell;
            total+=coverage;
            if(glm::dot(world-glm::vec2(boat.getPosition().x,boat.getPosition().z),lateral)<0) left+=coverage;
            else right+=coverage;
        }
        if (reviewFrame==180 && wakeScenario=="stop") { peakWakeArea=total; return; }
        bool passed=finite && glGetError()==GL_NO_ERROR;
        if (wakeScenario=="idle") passed &= total<0.001;
        else if (wakeScenario=="stop" && frameLimit>=600) passed &= peakWakeArea>0.1 && total<peakWakeArea*0.25;
        else if (wakeScenario=="straight" || wakeScenario=="reverse") passed &= total>0.1 && std::min(left,right)>std::max(left,right)*0.25;
        else if(wakeScenario=="turn") passed &= total>0.1;
        std::cout << (passed ? "[WAKE PASS] " : "[WAKE FAIL] ") << wakeScenario << " coverage_m2=" << total
                  << " left=" << left << " right=" << right << " peak=" << peakWakeArea
                  << " spray_particles=" << boat.getParticleSystem().getActiveCount() << "\n";
        reviewFailure |= !passed;
    }
    void finish(int width, int height, const std::chrono::steady_clock::time_point &reviewStart, Window &window) {
        if (frameLimit) {
            glFinish(); // Diagnostic mode measures completed CPU + GPU frame work, excluding swap.
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - reviewStart).count();
            if (reviewFrame >= 30) reviewMilliseconds += ms;
            GLenum err;
            while ((err = glGetError()) != GL_NO_ERROR) { reviewFailure = true; std::cerr << "[GL ERROR] frame=" << reviewFrame << " code=" << err << "\n"; }
            ++reviewFrame;
            if (reviewFrame == frameLimit) {
                if (!capturePath.empty()) {
                    std::vector<unsigned char> pixels(width * height * 3);
                    glPixelStorei(GL_PACK_ALIGNMENT, 1);
                    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
                    std::ofstream file(capturePath, std::ios::binary);
                    file << "P6\n" << width << " " << height << "\n255\n";
                    for (int y = height - 1; y >= 0; --y) file.write(reinterpret_cast<const char *>(pixels.data() + y * width * 3), width * 3);
                    if (!file) { std::cerr << "[CAPTURE ERROR] " << capturePath << "\n"; reviewFailure = true; window.close(); return; }
                }
                std::cout << "[REVIEW] view=" << reviewView << " hour=" << reviewHour << " frames=" << reviewFrame
                          << " framebuffer=" << width << "x" << height << " mean_completed_frame_ms="
                          << reviewMilliseconds / std::max(1, reviewFrame - 30) << "\n";
                window.close();
            }
        }
    }
};
