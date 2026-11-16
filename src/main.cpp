#include "core/Window.h"
#include "core/Camera.h"
#include "core/Input.h"
#include "renderer/Shader.h"
#include "renderer/Sky.h"
#include "renderer/Framebuffer.h"
#include "ocean/GerstnerWave.h"
#include "ocean/OceanMesh.h"
#include "ocean/FoamSimulator.h"
#include "game/Boat.h"
#include "game/Cargo.h"
#include "game/Lighthouse.h"
#include "game/Buoy.h"
#include "game/Radar.h"
#include "game/Wildlife.h"
#include "game/Weather.h"
#include "game/Harbor.h"
#include "game/Mainland.h"
#include "game/GameManager.h"
#include "audio/AudioEngine.h"

#include <iostream>

int main() {
    Window window(1280, 720, "OceanWave3D - Photorealistic Ocean Simulation");
    if (!window.init()) {
        std::cerr << "[FATAL] Failed to initialize display window\n";
        return -1;
    }

    Input input(window.getGLFWWindow());
    Camera camera;
    Sky sky;
    GerstnerWaveModel waves;
    OceanMesh ocean(240, 650.0f); // 650m Infinite Ocean grid
    Boat boat;
    CargoManager cargo;
    Lighthouse lighthouse;
    lighthouse.init(glm::vec3(85.0f, 0.0f, -105.0f));

    Harbor harbor;
    harbor.init(lighthouse.getPosition());

    Mainland mainland;
    mainland.init();

    BuoyManager buoys;
    buoys.init();

    MarineRadar radar;
    WildlifeManager wildlife;
    wildlife.init();

    WeatherSystem weather;
    weather.init();

    GameManager gameManager;
    gameManager.init(window.getGLFWWindow());

    AudioEngine audio;
    if (!audio.init()) {
        std::cerr << "[WARN] Audio engine failed to initialize, running without sound\n";
    }

    // SSR G-Buffer Framebuffer
    Framebuffer sceneFBO;
    if (!sceneFBO.init(window.getWidth(), window.getHeight(), true)) {
        std::cerr << "[FATAL] Failed to initialize HDR Scene Framebuffer\n";
        return -1;
    }

    // Composite Scene Framebuffer for Post-Processing Resolve
    Framebuffer compositeFBO;
    if (!compositeFBO.init(window.getWidth(), window.getHeight(), true)) {
        std::cerr << "[FATAL] Failed to initialize HDR Composite Framebuffer\n";
        return -1;
    }

    // Fullscreen Post-Processing VAO
    GLuint postProcessVAO = 0;
    glGenVertexArrays(1, &postProcessVAO);

    FoamSimulator foamSim;
    if (!foamSim.init(512, 240.0f)) {
        std::cerr << "[FATAL] Failed to initialize Foam Simulator\n";
        return -1;
    }

    Shader waterShader;
    if (!waterShader.loadFromFiles("assets/shaders/water.vert", "assets/shaders/water.frag")) {
        std::cerr << "[FATAL] Failed to load water shaders\n";
        return -1;
    }

    Shader boatShader;
    if (!boatShader.loadFromFiles("assets/shaders/boat.vert", "assets/shaders/boat.frag")) {
        std::cerr << "[FATAL] Failed to load boat shaders\n";
        return -1;
    }

    Shader landmarkShader;
    if (!landmarkShader.loadFromFiles("assets/shaders/landmark.vert", "assets/shaders/landmark.frag")) {
        std::cerr << "[FATAL] Failed to load landmark shaders\n";
        return -1;
    }

    Shader beamShader;
    if (!beamShader.loadFromFiles("assets/shaders/light_beam.vert", "assets/shaders/light_beam.frag")) {
        std::cerr << "[FATAL] Failed to load light beam shaders\n";
        return -1;
    }

    Shader skyShader;
    if (!skyShader.loadFromFiles("assets/shaders/sky.vert", "assets/shaders/sky.frag")) {
        std::cerr << "[FATAL] Failed to load sky shaders\n";
        return -1;
    }

    Shader particleShader;
    if (!particleShader.loadFromFiles("assets/shaders/particle.vert", "assets/shaders/particle.frag")) {
        std::cerr << "[FATAL] Failed to load particle shaders\n";
        return -1;
    }

    Shader rainShader;
    if (!rainShader.loadFromFiles("assets/shaders/rain.vert", "assets/shaders/rain.frag")) {
        std::cerr << "[FATAL] Failed to load rain shaders\n";
        return -1;
    }

    Shader postProcessShader;
    if (!postProcessShader.loadFromFiles("assets/shaders/postprocess.vert", "assets/shaders/postprocess.frag")) {
        std::cerr << "[FATAL] Failed to load postprocess shaders\n";
        return -1;
    }

    float lastFrameTime = window.getTime();

    // Key toggle state flags
    bool cPressedLast = false;
    bool fPressedLast = false;
    bool rPressedLast = false;
    bool mPressedLast = false;
    bool kPressedLast = false;

    while (!window.shouldClose()) {
        float currentFrameTime = window.getTime();
        float dt = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        // Clamp delta time to avoid large physics steps when moving window
        if (dt > 0.1f) dt = 0.1f;

        if (input.isKeyDown(GLFW_KEY_ESCAPE)) {
            window.close();
            break;
        }

        // Toggle Camera Mode via 'C'
        bool cDown = input.isKeyDown(GLFW_KEY_C);
        if (cDown && !cPressedLast) {
            gameManager.cycleCameraMode();
        }
        cPressedLast = cDown;

        // Toggle Spotlight via 'F'
        bool fDown = input.isKeyDown(GLFW_KEY_F);
        if (fDown && !fPressedLast) {
            boat.toggleSpotlight();
        }
        fPressedLast = fDown;

        // Toggle Radar via 'R'
        bool rDown = input.isKeyDown(GLFW_KEY_R);
        if (rDown && !rPressedLast) {
            radar.toggle();
        }
        rPressedLast = rDown;

        // Cycle Weather Mode via 'M' (Feature 2)
        bool mDown = input.isKeyDown(GLFW_KEY_M);
        if (mDown && !mPressedLast) {
            weather.cycleWeather();
        }
        mPressedLast = mDown;

        // Toggle Windshield Wiper via 'K' (Feature 1)
        bool kDown = input.isKeyDown(GLFW_KEY_K);
        if (kDown && !kPressedLast) {
            weather.toggleWiper();
        }
        kPressedLast = kDown;

        float gameTime = gameManager.getGameTime(currentFrameTime);

        // --- UPDATE LOGIC ---
        float throttle = (input.isKeyDown(GLFW_KEY_W) ? 1.0f : 0.0f) - (input.isKeyDown(GLFW_KEY_S) ? 1.0f : 0.0f);
        bool isControllable = gameManager.isBoatControllable();

        boat.update(dt, waves, gameTime, gameManager.getWaveHeight(), isControllable, input, &audio);
        lighthouse.update(dt, gameTime, sky.getSunDirection().y);
        harbor.update(dt, boat, input, audio);
        buoys.update(dt, waves, gameTime, gameManager.getWaveHeight(), boat.getPosition(), audio);
        gameManager.update(dt, boat, cargo);

        // Update Marine Wildlife (Seagulls flocking around lighthouse)
        wildlife.update(dt, boat.getPosition(), audio);

        // Update Marine Radar Display
        radar.update(dt, audio, boat, lighthouse, buoys, cargo);

        // Update persistent foam simulation (lingering wake & spray)
        foamSim.update(dt, boat.getPosition(), boat.getSpeed(), boat.getHeading());

        // Camera update
        CameraMode currentCamMode = gameManager.getCameraMode();
        camera.update(dt, currentCamMode, boat.getPosition(), boat.getHeading(),
                      boat.getPitch(), boat.getRoll(), boat.getCockpitEyePosition(), input);

        int width = window.getWidth();
        int height = window.getHeight();
        sceneFBO.resize(width, height);
        compositeFBO.resize(width, height);

        float aspect = window.getAspectRatio();
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 proj = camera.getProjectionMatrix(aspect);
        glm::vec3 camPos = camera.getPosition();
        glm::vec3 camFront = glm::normalize(camera.getTarget() - camPos);

        // Feature 3: Underwater Submersion Detection
        glm::vec3 camWaterSample = waves.evaluate(glm::vec2(camPos.x, camPos.z), gameTime, gameManager.getWaveHeight());
        bool isUnderwater = (camPos.y < (camWaterSample.y - 0.05f));
        float submergedDepth = isUnderwater ? (camWaterSample.y - camPos.y) : 0.0f;

        // Dynamic Weather & Audio Update
        weather.update(dt, gameTime, camPos, camFront, audio, boat.getSpeed());
        audio.setUnderwater(isUnderwater);
        audio.update(dt, boat.getSpeed(), throttle, gameManager.getWaveHeight(),
                     camPos, camFront, boat.getPosition(), currentCamMode == CameraMode::COCKPIT);

        glm::vec3 sunDir = sky.getSunDirection();
        glm::vec3 sunCol = sky.getSunColor();
        glm::vec3 moonDir = sky.getMoonDirection();
        glm::vec3 moonCol = sky.getMoonColor();
        glm::vec3 horizonColor = sky.getSkyHorizonColor();
        glm::vec3 zenithColor = sky.getSkyZenithColor();
        glm::vec3 lightningCol = weather.getLightningColor();
        glm::vec3 mainLightDir = (sunDir.y > -0.10f) ? sunDir : moonDir;
        glm::vec3 lightPos = camPos + mainLightDir * 100.0f;

        // Light color enhanced by lightning flash
        glm::vec3 litSunCol = sunCol + lightningCol;

        // Defensive OpenGL state setup before 3D passes
        glDisable(GL_SCISSOR_TEST);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        // =========================================================================
        // PASS 1: Render Scene (Sky + Landmarks + Wildlife + Boat + Cargo) for SSR
        // =========================================================================
        sceneFBO.bind();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 1a. Sky into Scene FBO
        sky.render(skyShader, camera, gameTime, width, height, weather.getRainIntensity(), lightningCol);

        // 1b. Landmarks (Lighthouse & Buoys) and Wildlife into Scene FBO for SSR
        landmarkShader.use();
        landmarkShader.setMat4("view", view);
        landmarkShader.setMat4("projection", proj);
        landmarkShader.setVec3("viewPos", camPos);
        landmarkShader.setVec3("fogColor", horizonColor);
        landmarkShader.setVec3("uSunDir", sunDir);
        landmarkShader.setVec3("uSunColor", litSunCol);
        landmarkShader.setVec3("uMoonDir", moonDir);
        landmarkShader.setVec3("uMoonColor", moonCol);
        landmarkShader.setVec3("uHorizonColor", horizonColor);
        landmarkShader.setInt("uSpotlightOn", boat.isSpotlightOn() ? 1 : 0);
        landmarkShader.setVec3("uSpotlightPos", boat.getSpotlightPosition());
        landmarkShader.setVec3("uSpotlightDir", boat.getSpotlightDirection());

        lighthouse.render(landmarkShader);
        harbor.render(landmarkShader);
        mainland.render(landmarkShader);
        buoys.render(landmarkShader);
        wildlife.render(landmarkShader);

        // 1c. Boat & Cargo into Scene FBO
        boatShader.use();
        boatShader.setMat4("view", view);
        boatShader.setMat4("projection", proj);
        boatShader.setVec3("lightPos", lightPos);
        boatShader.setVec3("viewPos", camPos);
        boatShader.setVec3("fogColor", horizonColor);
        boatShader.setVec3("uSunDir", sunDir);
        boatShader.setVec3("uSunColor", litSunCol);
        boatShader.setVec3("uMoonDir", moonDir);
        boatShader.setVec3("uMoonColor", moonCol);
        boatShader.setVec3("uHorizonColor", horizonColor);
        boatShader.setInt("uSpotlightOn", boat.isSpotlightOn() ? 1 : 0);
        boatShader.setVec3("uSpotlightPos", boat.getSpotlightPosition());
        boatShader.setVec3("uSpotlightDir", boat.getSpotlightDirection());

        cargo.render(boatShader, waves, gameTime, gameManager.getWaveHeight());
        boat.render(boatShader);

        sceneFBO.unbind();

        // =========================================================================
        // PASS 2: Render Full Composition into HDR Composite FBO
        // =========================================================================
        compositeFBO.bind();
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 2a. Background Sky
        sky.render(skyShader, camera, gameTime, width, height, weather.getRainIntensity(), lightningCol);

        // 2b. Infinite Ocean with Analytical Normals + SSR + Sweeping Searchlight + Rain Rings + Lightning
        waterShader.use();
        waterShader.setMat4("view", view);
        waterShader.setMat4("projection", proj);
        waterShader.setMat4("model", glm::mat4(1.0f));
        waterShader.setFloat("time", gameTime);
        waterShader.setVec3("viewPos", camPos);
        waterShader.setVec2("uResolution", static_cast<float>(width), static_cast<float>(height));

        // Dynamic Weather & Environment Uniforms
        waterShader.setFloat("uRainIntensity", weather.getRainIntensity());
        waterShader.setVec3("uLightningColor", lightningCol);
        waterShader.setInt("uIsUnderwater", isUnderwater ? 1 : 0);

        // Bind Scene Color and Depth Textures for SSR & Refraction
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sceneFBO.getColorTexture());
        waterShader.setInt("uSceneColor", 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, sceneFBO.getDepthTexture());
        waterShader.setInt("uSceneDepth", 1);

        // Bind Persistent Foam Simulator Texture
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, foamSim.getFoamTexture());
        waterShader.setInt("uFoamTexture", 2);
        waterShader.setVec2("uFoamCenter", foamSim.getDomainCenter());
        waterShader.setFloat("uFoamSize", foamSim.getDomainSize());

        waterShader.setVec3("uCamPos", camPos);
        waterShader.setFloat("uOceanSize", ocean.getSize());
        waterShader.setInt("uResolution", ocean.getResolution());
        waterShader.setVec3("uSunDir", sunDir);
        waterShader.setVec3("uSunColor", litSunCol);
        waterShader.setVec3("uMoonDir", moonDir);
        waterShader.setVec3("uMoonColor", moonCol);
        waterShader.setVec3("uHorizonColor", horizonColor);
        waterShader.setVec3("uZenithColor", zenithColor);
        waterShader.setFloat("uTimeOfDay", sky.getTimeOfDay());
        waterShader.setVec3("uBoatPos", boat.getPosition());
        waterShader.setFloat("uBoatSpeed", boat.getSpeed());
        waterShader.setFloat("uBoatHeading", boat.getHeading());
        waterShader.setInt("uSpotlightOn", boat.isSpotlightOn() ? 1 : 0);
        waterShader.setVec3("uSpotlightPos", boat.getSpotlightPosition());
        waterShader.setVec3("uSpotlightDir", boat.getSpotlightDirection());

        // Lighthouse searchlight beam illumination on ocean waves
        waterShader.setInt("uLighthouseActive", lighthouse.isBeamActive() ? 1 : 0);
        waterShader.setVec3("uLighthousePos", lighthouse.getLanternPosition());
        waterShader.setVec3("uLighthouseBeamDir", lighthouse.getBeamDirection());
        waterShader.setFloat("uLighthouseIntensity", lighthouse.getBeamIntensity());

        waves.uploadUniforms(waterShader, gameManager.getWaveHeight());
        ocean.render();

        // 2c. Re-render Landmarks and Wildlife above the water surface
        landmarkShader.use();
        landmarkShader.setMat4("view", view);
        landmarkShader.setMat4("projection", proj);
        landmarkShader.setVec3("viewPos", camPos);
        landmarkShader.setVec3("fogColor", horizonColor);
        landmarkShader.setVec3("uSunDir", sunDir);
        landmarkShader.setVec3("uSunColor", litSunCol);
        landmarkShader.setVec3("uMoonDir", moonDir);
        landmarkShader.setVec3("uMoonColor", moonCol);
        landmarkShader.setVec3("uHorizonColor", horizonColor);
        landmarkShader.setInt("uSpotlightOn", boat.isSpotlightOn() ? 1 : 0);
        landmarkShader.setVec3("uSpotlightPos", boat.getSpotlightPosition());
        landmarkShader.setVec3("uSpotlightDir", boat.getSpotlightDirection());

        lighthouse.render(landmarkShader);
        harbor.render(landmarkShader);
        mainland.render(landmarkShader);
        buoys.render(landmarkShader);
        wildlife.render(landmarkShader);

        // 2d. Re-render Boat and Cargo above the water surface
        boatShader.use();
        boatShader.setMat4("view", view);
        boatShader.setMat4("projection", proj);
        boatShader.setVec3("lightPos", lightPos);
        boatShader.setVec3("viewPos", camPos);
        boatShader.setVec3("fogColor", horizonColor);
        boatShader.setVec3("uSunDir", sunDir);
        boatShader.setVec3("uSunColor", litSunCol);
        boatShader.setVec3("uMoonDir", moonDir);
        boatShader.setVec3("uMoonColor", moonCol);
        boatShader.setVec3("uHorizonColor", horizonColor);
        boatShader.setInt("uSpotlightOn", boat.isSpotlightOn() ? 1 : 0);
        boatShader.setVec3("uSpotlightPos", boat.getSpotlightPosition());
        boatShader.setVec3("uSpotlightDir", boat.getSpotlightDirection());

        cargo.render(boatShader, waves, gameTime, gameManager.getWaveHeight());
        boat.render(boatShader);

        // 2e. Volumetric sweeping lighthouse beam in the night sky
        lighthouse.renderBeam(beamShader, view, proj);

        // 2f. Splash Particles (Boat Bow & Wake)
        glm::vec3 particleLightTint = (sunCol.y > 0.05f) ? litSunCol : glm::vec3(0.6f, 0.75f, 0.95f);
        boat.renderParticles(particleShader, camera, particleLightTint);

        // 2g. Dynamic Rain Streaks Falling from Sky (Feature 2)
        weather.renderRain(rainShader, view, proj, camPos, dt);

        compositeFBO.unbind();

        // =========================================================================
        // PASS 3: Post-Processing Resolve to Default Screen Framebuffer (Features 1 & 3)
        // =========================================================================
        glViewport(0, 0, width, height);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);

        postProcessShader.use();
        postProcessShader.setVec2("uResolution", static_cast<float>(width), static_cast<float>(height));
        postProcessShader.setFloat("time", gameTime);

        // Feature 1: Droplets & Windshield Wiper
        bool isCockpit = (currentCamMode == CameraMode::COCKPIT);
        postProcessShader.setInt("uIsCockpit", isCockpit ? 1 : 0);
        postProcessShader.setFloat("uWetness", weather.getWetness());
        postProcessShader.setFloat("uWiperAngle", weather.getWiperAngle());
        postProcessShader.setInt("uWiperActive", weather.isWiperActive() ? 1 : 0);
        postProcessShader.setVec2("uWiperPivot", glm::vec2(0.5f, 0.05f));
        postProcessShader.setFloat("uWiperLength", 0.85f);

        // Feature 3: Underwater Diving View
        postProcessShader.setInt("uIsUnderwater", isUnderwater ? 1 : 0);
        postProcessShader.setFloat("uSubmergedDepth", submergedDepth);
        postProcessShader.setVec3("uSunDir", sunDir);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, compositeFBO.getColorTexture());
        postProcessShader.setInt("uSceneColor", 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, compositeFBO.getDepthTexture());
        postProcessShader.setInt("uSceneDepth", 1);

        glBindVertexArray(postProcessVAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);

        // =========================================================================
        // PASS 4: UI Pass (Dear ImGui)
        // =========================================================================
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        gameManager.renderUI(boat, cargo, ocean, sky, audio, lighthouse, buoys, weather, harbor, isUnderwater, submergedDepth);
        radar.renderUI(boat, lighthouse, buoys, cargo, &harbor);
        harbor.renderHUD(width, height, boat);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        window.swapBuffers();
        window.pollEvents();
    }

    if (postProcessVAO != 0) {
        glDeleteVertexArrays(1, &postProcessVAO);
    }

    return 0;
}
