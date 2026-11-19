#pragma once
#include "../game/BoatPhysics.h"
#include <iostream>
#include <cmath>

inline bool simulationChecks() {
    GerstnerWaveModel waves;
    float maxResidual = 0.0f;
    for (float scale : {0.0f,1.2f,6.0f,8.0f}) for (int i=0;i<100;++i) {
        glm::vec2 parameter(i*3.17f-150.0f,i*-1.71f+80.0f);
        glm::vec3 point=waves.evaluate(parameter,12.5f,scale);
        glm::vec3 sampled=waves.surfaceAt({point.x,point.z},12.5f,scale);
        maxResidual=std::max(maxResidual,std::abs(point.y-sampled.y));
    }
    if (maxResidual > 0.03f) { std::cerr << "[TEST FAIL] Gerstner inverse error " << maxResidual << " m\n"; return false; }
    Input input(nullptr); input.replay(true);
    BoatPhysics boat;
    // Calm water must reach a bounded equilibrium, then move under thrust and stop when moored.
    for(int i=0;i<7200;++i) boat.update(1.0f/120,waves,i/120.0f,0,false,input);
    if (!std::isfinite(boat.getPosition().y) || std::abs(boat.getPosition().y)>0.5f || std::abs(boat.getVelocity().y)>0.02f) {
        std::cerr << "[TEST FAIL] Calm-water equilibrium\n"; return false;
    }
    input.setReplayKey(GLFW_KEY_W,true);
    for(int i=0;i<240;++i) boat.update(1.0f/120,waves,60+i/120.0f,1.2f,true,input);
    if (boat.getSpeed()<1 || boat.getPosition().z<1) { std::cerr << "[TEST FAIL] Thrust input\n"; return false; }
    boat.setDocked(true,{0,0,0},0);
    for(int i=0;i<600;++i) boat.update(1.0f/120,waves,62+i/120.0f,1.2f,true,input);
    if (std::abs(boat.getSpeed())>0.01f || glm::length(glm::vec2(boat.getPosition().x,boat.getPosition().z))>0.01f) {
        std::cerr << "[TEST FAIL] Mooring constraint\n"; return false;
    }
    ImGui::CreateContext(); ImGui::GetIO().WantCaptureKeyboard=true;
    bool blocked=!input.isKeyDown(GLFW_KEY_W); ImGui::DestroyContext();
    if (!blocked) { std::cerr << "[TEST FAIL] UI keyboard capture\n"; return false; }
    std::cout << "[TEST PASS] Water inversion, buoyancy equilibrium, thrust, mooring, UI capture; max inverse error=" << maxResidual << " m\n";
    return true;
}
