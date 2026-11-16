#pragma once

#include <glm/glm.hpp>

class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();

    bool init();
    void update(float dt, float boatSpeed, float throttleInput, float waveHeight,
                const glm::vec3 &camPos, const glm::vec3 &camFront, const glm::vec3 &boatPos,
                bool inCockpit);
    void playBowSplash(float intensity, const glm::vec3 &position);
    void playBuoyBell(const glm::vec3 &position);
    void playSeagullCall(const glm::vec3 &position);
    void playRadarPing();
    void setRainIntensity(float intensity);
    void playThunder(float intensity);
    void setUnderwater(bool underwater);
    bool isUnderwater() const { return m_isUnderwater; }
    void shutdown();

    // Volume controls
    void setMasterVolume(float vol);
    float getMasterVolume() const { return m_masterVolume; }

    void setEngineVolume(float vol);
    float getEngineVolume() const { return m_engineVolume; }

    void setAmbientVolume(float vol);
    float getAmbientVolume() const { return m_ambientVolume; }

    void setSfxVolume(float vol);
    float getSfxVolume() const { return m_sfxVolume; }

    bool isMuted() const { return m_isMuted; }
    void toggleMute();

    bool isInitialized() const { return m_initialized; }

private:
    void *m_device;
    void *m_context;
    bool m_initialized;
    bool m_isMuted;

    float m_masterVolume;
    float m_engineVolume;
    float m_ambientVolume;
    float m_sfxVolume;

    // Smoothed state variables
    float m_currentEnginePitch;
    float m_currentEngineGain;
    float m_currentWindGain;
    float m_currentOceanGain;

    // OpenAL Sources & Buffers
    unsigned int m_engineBuffer;
    unsigned int m_engineSource;

    unsigned int m_oceanBuffer;
    unsigned int m_oceanSource;

    unsigned int m_windBuffer;
    unsigned int m_windSource;

    // Splash sound pool
    static constexpr int SPLASH_POOL_SIZE = 4;
    unsigned int m_splashBuffer;
    unsigned int m_splashSources[SPLASH_POOL_SIZE];
    int m_splashPoolIndex;

    // Buoy bell chime
    unsigned int m_bellBuffer;
    unsigned int m_bellSource;

    // Wildlife: Seagulls
    unsigned int m_seagullBuffer;
    unsigned int m_seagullSource;

    // Radar ping
    unsigned int m_radarBuffer;
    unsigned int m_radarSource;

    // Weather: Rain & Thunder
    unsigned int m_rainBuffer;
    unsigned int m_rainSource;

    unsigned int m_thunderBuffer;
    unsigned int m_thunderSource;

    // Underwater audio state
    bool m_isUnderwater;
    float m_currentRainGain;

    void generateEngineSound();
    void generateOceanSurfSound();
    void generateWindSound();
    void generateSplashSound();
    void generateBuoyBellSound();
    void generateSeagullSound();
    void generateRadarSound();
    void generateRainSound();
    void generateThunderSound();
};
