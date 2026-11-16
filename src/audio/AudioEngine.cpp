#include "AudioEngine.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib>

#if defined(__APPLE__)
#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#else
#include <AL/al.h>
#include <AL/alc.h>
#endif

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

static constexpr int SAMPLE_RATE = 44100;
static constexpr float PI = 3.14159265358979323846f;

AudioEngine::AudioEngine()
    : m_device(nullptr),
      m_context(nullptr),
      m_initialized(false),
      m_isMuted(false),
      m_masterVolume(0.80f),
      m_engineVolume(0.75f),
      m_ambientVolume(0.65f),
      m_sfxVolume(0.85f),
      m_currentEnginePitch(1.0f),
      m_currentEngineGain(0.25f),
      m_currentWindGain(0.0f),
      m_currentOceanGain(0.35f),
      m_engineBuffer(0),
      m_engineSource(0),
      m_oceanBuffer(0),
      m_oceanSource(0),
      m_windBuffer(0),
      m_windSource(0),
      m_splashBuffer(0),
      m_splashPoolIndex(0),
      m_bellBuffer(0),
      m_bellSource(0),
      m_seagullBuffer(0),
      m_seagullSource(0),
      m_radarBuffer(0),
      m_radarSource(0),
      m_rainBuffer(0),
      m_rainSource(0),
      m_thunderBuffer(0),
      m_thunderSource(0),
      m_isUnderwater(false),
      m_currentRainGain(0.0f) {
    for (int i = 0; i < SPLASH_POOL_SIZE; ++i) {
        m_splashSources[i] = 0;
    }
}

AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::init() {
    ALCdevice *dev = alcOpenDevice(nullptr);
    if (!dev) {
        std::cerr << "[AudioEngine] Warning: No audio output device available\n";
        return false;
    }
    m_device = dev;

    ALCcontext *ctx = alcCreateContext(dev, nullptr);
    if (!ctx) {
        std::cerr << "[AudioEngine] Warning: Failed to create OpenAL context\n";
        alcCloseDevice(dev);
        m_device = nullptr;
        return false;
    }
    m_context = ctx;
    alcMakeContextCurrent(ctx);

    // Set listener initial defaults
    alListener3f(AL_POSITION, 0.0f, 2.0f, 0.0f);
    alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f);
    float orientation[] = { 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f };
    alListenerfv(AL_ORIENTATION, orientation);

    // Generate waveforms in memory
    generateEngineSound();
    generateOceanSurfSound();
    generateWindSound();
    generateSplashSound();
    generateBuoyBellSound();
    generateSeagullSound();
    generateRadarSound();
    generateRainSound();
    generateThunderSound();

    // Start background loops
    if (m_engineSource != 0) {
        alSourcePlay(m_engineSource);
    }
    if (m_oceanSource != 0) {
        alSourcePlay(m_oceanSource);
    }
    if (m_windSource != 0) {
        alSourcePlay(m_windSource);
    }
    if (m_rainSource != 0) {
        alSourcePlay(m_rainSource);
    }

    m_initialized = true;
    std::cout << "[AudioEngine] Initialized OpenAL procedural audio successfully\n";
    return true;
}

void AudioEngine::shutdown() {
    if (!m_initialized) return;

    if (m_engineSource != 0) {
        alSourceStop(m_engineSource);
        alDeleteSources(1, &m_engineSource);
        m_engineSource = 0;
    }
    if (m_engineBuffer != 0) {
        alDeleteBuffers(1, &m_engineBuffer);
        m_engineBuffer = 0;
    }

    if (m_oceanSource != 0) {
        alSourceStop(m_oceanSource);
        alDeleteSources(1, &m_oceanSource);
        m_oceanSource = 0;
    }
    if (m_oceanBuffer != 0) {
        alDeleteBuffers(1, &m_oceanBuffer);
        m_oceanBuffer = 0;
    }

    if (m_windSource != 0) {
        alSourceStop(m_windSource);
        alDeleteSources(1, &m_windSource);
        m_windSource = 0;
    }
    if (m_windBuffer != 0) {
        alDeleteBuffers(1, &m_windBuffer);
        m_windBuffer = 0;
    }

    for (int i = 0; i < SPLASH_POOL_SIZE; ++i) {
        if (m_splashSources[i] != 0) {
            alSourceStop(m_splashSources[i]);
            alDeleteSources(1, &m_splashSources[i]);
            m_splashSources[i] = 0;
        }
    }
    if (m_splashBuffer != 0) {
        alDeleteBuffers(1, &m_splashBuffer);
        m_splashBuffer = 0;
    }

    if (m_bellSource != 0) {
        alSourceStop(m_bellSource);
        alDeleteSources(1, &m_bellSource);
        m_bellSource = 0;
    }
    if (m_bellBuffer != 0) {
        alDeleteBuffers(1, &m_bellBuffer);
        m_bellBuffer = 0;
    }

    if (m_seagullSource != 0) {
        alSourceStop(m_seagullSource);
        alDeleteSources(1, &m_seagullSource);
        m_seagullSource = 0;
    }
    if (m_seagullBuffer != 0) {
        alDeleteBuffers(1, &m_seagullBuffer);
        m_seagullBuffer = 0;
    }

    if (m_radarSource != 0) {
        alSourceStop(m_radarSource);
        alDeleteSources(1, &m_radarSource);
        m_radarSource = 0;
    }
    if (m_radarBuffer != 0) {
        alDeleteBuffers(1, &m_radarBuffer);
        m_radarBuffer = 0;
    }

    if (m_rainSource != 0) {
        alSourceStop(m_rainSource);
        alDeleteSources(1, &m_rainSource);
        m_rainSource = 0;
    }
    if (m_rainBuffer != 0) {
        alDeleteBuffers(1, &m_rainBuffer);
        m_rainBuffer = 0;
    }

    if (m_thunderSource != 0) {
        alSourceStop(m_thunderSource);
        alDeleteSources(1, &m_thunderSource);
        m_thunderSource = 0;
    }
    if (m_thunderBuffer != 0) {
        alDeleteBuffers(1, &m_thunderBuffer);
        m_thunderBuffer = 0;
    }

    if (m_context) {
        alcMakeContextCurrent(nullptr);
        alcDestroyContext(static_cast<ALCcontext*>(m_context));
        m_context = nullptr;
    }
    if (m_device) {
        alcCloseDevice(static_cast<ALCdevice*>(m_device));
        m_device = nullptr;
    }

    m_initialized = false;
}

void AudioEngine::setMasterVolume(float vol) {
    m_masterVolume = glm::clamp(vol, 0.0f, 1.0f);
}

void AudioEngine::setEngineVolume(float vol) {
    m_engineVolume = glm::clamp(vol, 0.0f, 1.0f);
}

void AudioEngine::setAmbientVolume(float vol) {
    m_ambientVolume = glm::clamp(vol, 0.0f, 1.0f);
}

void AudioEngine::setSfxVolume(float vol) {
    m_sfxVolume = glm::clamp(vol, 0.0f, 1.0f);
}

void AudioEngine::toggleMute() {
    m_isMuted = !m_isMuted;
}

void AudioEngine::update(float dt, float boatSpeed, float throttleInput, float waveHeight,
                        const glm::vec3 &camPos, const glm::vec3 &camFront, const glm::vec3 &boatPos,
                        bool inCockpit) {
    if (!m_initialized) return;

    float effectiveMaster = m_isMuted ? 0.0f : m_masterVolume;

    // 1. Update OpenAL 3D Audio Listener (Camera Head)
    alListener3f(AL_POSITION, camPos.x, camPos.y, camPos.z);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    float orientation[] = { camFront.x, camFront.y, camFront.z, up.x, up.y, up.z };
    alListenerfv(AL_ORIENTATION, orientation);

    // 2. Dynamic Marine Engine Tone & Pitch Shift
    // Throttle input is +1 for W (forward), -1 for S (reverse), 0 for neutral
    float normalizedSpeed = glm::clamp(boatSpeed / 12.0f, 0.0f, 1.0f);
    float targetPitch = 0.82f + normalizedSpeed * 1.05f + std::max(0.0f, throttleInput) * 0.35f;
    float targetGain = (0.28f + normalizedSpeed * 0.55f + std::abs(throttleInput) * 0.15f) * m_engineVolume * effectiveMaster;

    if (inCockpit) {
        // Cockpit interior: deep muffled engine rumble
        targetGain *= 0.85f;
    }

    if (m_isUnderwater) {
        // Underwater: heavy acoustic dampening, deep subterranean rumble
        targetPitch *= 0.52f;
        targetGain *= 0.40f;
    }

    // Smooth response curves
    m_currentEnginePitch += (targetPitch - m_currentEnginePitch) * std::min(dt * 5.0f, 1.0f);
    m_currentEngineGain += (targetGain - m_currentEngineGain) * std::min(dt * 6.0f, 1.0f);

    alSourcef(m_engineSource, AL_PITCH, m_currentEnginePitch);
    alSourcef(m_engineSource, AL_GAIN, m_currentEngineGain);
    // Position engine at the boat stern
    alSource3f(m_engineSource, AL_POSITION, boatPos.x, boatPos.y + 0.2f, boatPos.z);

    // 3. Ambient Ocean Waves (Rì rào biển)
    // Scale ocean ambient sound with sea state (Beaufort wave height)
    float waveFactor = glm::clamp(waveHeight / 3.0f, 0.3f, 1.8f);
    float targetOceanGain = (0.35f + waveFactor * 0.35f) * m_ambientVolume * effectiveMaster;
    if (m_isUnderwater) {
        targetOceanGain = 0.55f * m_ambientVolume * effectiveMaster; // Sub-bass water pressure
    }
    m_currentOceanGain += (targetOceanGain - m_currentOceanGain) * std::min(dt * 3.0f, 1.0f);
    alSourcef(m_oceanSource, AL_GAIN, m_currentOceanGain);
    alSource3f(m_oceanSource, AL_POSITION, camPos.x, camPos.y - 1.0f, camPos.z);

    // 4. Wind Roar at Speed & In Storm
    float speedWind = normalizedSpeed * 0.65f;
    float stormWind = glm::clamp((waveHeight - 1.5f) / 3.5f, 0.0f, 0.65f);
    float targetWindGain = (speedWind + stormWind) * m_ambientVolume * effectiveMaster;
    if (inCockpit) targetWindGain *= 0.5f; // Windshield blocks wind
    if (m_isUnderwater) targetWindGain = 0.0f; // Zero wind underwater

    m_currentWindGain += (targetWindGain - m_currentWindGain) * std::min(dt * 4.0f, 1.0f);
    alSourcef(m_windSource, AL_GAIN, m_currentWindGain);
    alSourcef(m_windSource, AL_PITCH, 0.9f + normalizedSpeed * 0.3f);
    alSource3f(m_windSource, AL_POSITION, camPos.x, camPos.y, camPos.z);

    // 5. Rain ambient loop
    if (m_rainSource != 0) {
        float effectiveRainGain = m_currentRainGain * m_ambientVolume * effectiveMaster;
        if (m_isUnderwater) effectiveRainGain *= 0.25f;
        alSourcef(m_rainSource, AL_GAIN, effectiveRainGain);
        alSource3f(m_rainSource, AL_POSITION, camPos.x, camPos.y, camPos.z);
    }
}

void AudioEngine::playBowSplash(float intensity, const glm::vec3 &position) {
    if (!m_initialized || m_isMuted) return;

    // Use round-robin splash pool
    unsigned int source = m_splashSources[m_splashPoolIndex];
    m_splashPoolIndex = (m_splashPoolIndex + 1) % SPLASH_POOL_SIZE;

    float gain = glm::clamp(intensity * 0.8f, 0.25f, 1.0f) * m_sfxVolume * m_masterVolume;
    float pitch = 0.92f + (static_cast<float>(rand() % 100) / 500.0f); // 0.92 to 1.12

    alSourceStop(source);
    alSourcef(source, AL_GAIN, gain);
    alSourcef(source, AL_PITCH, pitch);
    alSource3f(source, AL_POSITION, position.x, position.y, position.z);
    alSourcePlay(source);
}

void AudioEngine::playBuoyBell(const glm::vec3 &position) {
    if (!m_initialized || m_isMuted) return;

    alSourceStop(m_bellSource);
    alSourcef(m_bellSource, AL_GAIN, 0.85f * m_sfxVolume * m_masterVolume);
    alSourcef(m_bellSource, AL_PITCH, 0.98f + (static_cast<float>(rand() % 40) / 1000.0f));
    alSource3f(m_bellSource, AL_POSITION, position.x, position.y + 1.5f, position.z);
    alSourcePlay(m_bellSource);
}

void AudioEngine::playSeagullCall(const glm::vec3 &position) {
    if (!m_initialized || m_isMuted) return;

    alSourceStop(m_seagullSource);
    alSourcef(m_seagullSource, AL_GAIN, 0.75f * m_sfxVolume * m_masterVolume);
    alSourcef(m_seagullSource, AL_PITCH, 0.95f + (static_cast<float>(rand() % 60) / 500.0f)); // 0.95 to 1.07
    alSource3f(m_seagullSource, AL_POSITION, position.x, position.y, position.z);
    alSourcePlay(m_seagullSource);
}

void AudioEngine::playRadarPing() {
    if (!m_initialized || m_isMuted) return;

    alSourceStop(m_radarSource);
    alSourcef(m_radarSource, AL_GAIN, 0.45f * m_sfxVolume * m_masterVolume);
    alSourcePlay(m_radarSource);
}

void AudioEngine::setRainIntensity(float intensity) {
    m_currentRainGain = glm::clamp(intensity, 0.0f, 1.0f) * 0.75f;
}

void AudioEngine::playThunder(float intensity) {
    if (!m_initialized || m_isMuted) return;

    alSourceStop(m_thunderSource);
    alSourcef(m_thunderSource, AL_GAIN, glm::clamp(intensity, 0.3f, 1.0f) * m_sfxVolume * m_masterVolume);
    alSourcef(m_thunderSource, AL_PITCH, 0.95f + (static_cast<float>(rand() % 40) / 400.0f));
    alSourcePlay(m_thunderSource);
}

void AudioEngine::setUnderwater(bool underwater) {
    m_isUnderwater = underwater;
}

// -----------------------------------------------------------------------------
// Procedural Waveform Synthesis
// -----------------------------------------------------------------------------

void AudioEngine::generateEngineSound() {
    // 2.0 seconds seamless loop (110 periods at 55.0 Hz)
    float duration = 2.0f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    float f0 = 55.0f; // Idle fundamental combustion frequency

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float phase = std::fmod(t * f0, 1.0f);

        // Multi-cylinder marine engine synthesis:
        // Asymmetric pulse / sawtooth wave
        float pulse = (phase < 0.45f) ? (phase * 4.44f - 1.0f) : (1.0f - (phase - 0.45f) * 3.63f);
        // Deep sub-octave rumble
        float sub = std::sin(2.0f * PI * (f0 * 0.5f) * t);
        // Combustion harmonics
        float h2 = std::sin(2.0f * PI * (f0 * 2.0f) * t);
        float h3 = std::sin(2.0f * PI * (f0 * 3.0f) * t);
        float h4 = std::sin(2.0f * PI * (f0 * 4.0f) * t);

        // Exhaust bubble turbulence
        float bubbling = ((rand() % 1000) / 500.0f - 1.0f) * 0.08f;

        float sample = pulse * 0.45f + sub * 0.28f + h2 * 0.16f + h3 * 0.10f + h4 * 0.05f + bubbling;
        sample = glm::clamp(sample, -1.0f, 1.0f);
        pcm[i] = static_cast<short>(sample * 28000.0f);
    }

    alGenBuffers(1, &m_engineBuffer);
    alBufferData(m_engineBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_engineSource);
    alSourcei(m_engineSource, AL_BUFFER, m_engineBuffer);
    alSourcei(m_engineSource, AL_LOOPING, AL_TRUE);
    alSourcef(m_engineSource, AL_REFERENCE_DISTANCE, 4.0f);
    alSourcef(m_engineSource, AL_MAX_DISTANCE, 120.0f);
    alSourcef(m_engineSource, AL_ROLLOFF_FACTOR, 1.0f);
    alSourcef(m_engineSource, AL_GAIN, 0.25f);
}

void AudioEngine::generateOceanSurfSound() {
    // 5.0 seconds seamless pink noise wave swell
    float duration = 5.0f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    // Paul Kellet's filtered pink noise generator state
    double b0 = 0.0, b1 = 0.0, b2 = 0.0, b3 = 0.0, b4 = 0.0, b5 = 0.0, b6 = 0.0;

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;

        // White noise sample [-1, 1]
        double white = ((rand() % 20000) / 10000.0) - 1.0;

        // Pink noise filter (1/f response)
        b0 = 0.99886 * b0 + white * 0.0555179;
        b1 = 0.99332 * b1 + white * 0.0750759;
        b2 = 0.96900 * b2 + white * 0.1538520;
        b3 = 0.86650 * b3 + white * 0.3104856;
        b4 = 0.55000 * b4 + white * 0.5329522;
        b5 = -0.7616 * b5 - white * 0.0168980;
        double pink = b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362;
        b6 = white * 0.115926;
        pink *= 0.11; // Normalize

        // Ocean wave rhythmic swell modulation (~0.2 Hz swell period)
        float swell = 0.45f + 0.55f * (0.5f + 0.5f * std::sin(2.0f * PI * 0.2f * t - PI * 0.5f));
        // High-frequency foamy crest wash
        float foamSpray = static_cast<float>(white) * std::pow(std::max(0.0f, std::sin(2.0f * PI * 0.2f * t)), 2.5f) * 0.20f;

        float sample = static_cast<float>(pink) * swell + foamSpray;
        pcm[i] = static_cast<short>(glm::clamp(sample, -1.0f, 1.0f) * 26000.0f);
    }

    // Circular crossfade over last 0.3s to guarantee 100% click-free loop
    int crossFadeSamples = static_cast<int>(0.3f * SAMPLE_RATE);
    for (int i = 0; i < crossFadeSamples; ++i) {
        float blend = static_cast<float>(i) / crossFadeSamples;
        int tailIdx = numSamples - crossFadeSamples + i;
        pcm[i] = static_cast<short>(pcm[i] * blend + pcm[tailIdx] * (1.0f - blend));
    }

    alGenBuffers(1, &m_oceanBuffer);
    alBufferData(m_oceanBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_oceanSource);
    alSourcei(m_oceanSource, AL_BUFFER, m_oceanBuffer);
    alSourcei(m_oceanSource, AL_LOOPING, AL_TRUE);
    alSourcei(m_oceanSource, AL_SOURCE_RELATIVE, AL_TRUE); // Ambient around listener
    alSourcef(m_oceanSource, AL_GAIN, 0.35f);
}

void AudioEngine::generateWindSound() {
    float duration = 4.0f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    float lp = 0.0f;
    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float white = ((rand() % 20000) / 10000.0f) - 1.0f;

        // Dynamic low-pass filter modulated by wind gusts
        float cutoff = 0.08f + 0.05f * std::sin(2.0f * PI * 0.4f * t);
        lp += (white - lp) * cutoff;

        pcm[i] = static_cast<short>(glm::clamp(lp * 1.8f, -1.0f, 1.0f) * 25000.0f);
    }

    // Circular crossfade
    int crossFadeSamples = static_cast<int>(0.3f * SAMPLE_RATE);
    for (int i = 0; i < crossFadeSamples; ++i) {
        float blend = static_cast<float>(i) / crossFadeSamples;
        int tailIdx = numSamples - crossFadeSamples + i;
        pcm[i] = static_cast<short>(pcm[i] * blend + pcm[tailIdx] * (1.0f - blend));
    }

    alGenBuffers(1, &m_windBuffer);
    alBufferData(m_windBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_windSource);
    alSourcei(m_windSource, AL_BUFFER, m_windBuffer);
    alSourcei(m_windSource, AL_LOOPING, AL_TRUE);
    alSourcei(m_windSource, AL_SOURCE_RELATIVE, AL_TRUE);
    alSourcef(m_windSource, AL_GAIN, 0.0f);
}

void AudioEngine::generateSplashSound() {
    float duration = 0.65f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float env = std::exp(-t * 7.5f);

        // Low hull thump impact (85 Hz)
        float thump = std::sin(2.0f * PI * 85.0f * t) * std::exp(-t * 16.0f);
        // Water spray noise
        float noise = ((rand() % 20000) / 10000.0f - 1.0f);

        float s = (thump * 0.7f + noise * 0.6f) * env;
        pcm[i] = static_cast<short>(glm::clamp(s, -1.0f, 1.0f) * 30000.0f);
    }

    alGenBuffers(1, &m_splashBuffer);
    alBufferData(m_splashBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(SPLASH_POOL_SIZE, m_splashSources);
    for (int i = 0; i < SPLASH_POOL_SIZE; ++i) {
        alSourcei(m_splashSources[i], AL_BUFFER, m_splashBuffer);
        alSourcei(m_splashSources[i], AL_LOOPING, AL_FALSE);
        alSourcef(m_splashSources[i], AL_REFERENCE_DISTANCE, 5.0f);
        alSourcef(m_splashSources[i], AL_MAX_DISTANCE, 80.0f);
    }
}

void AudioEngine::generateBuoyBellSound() {
    float duration = 2.5f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float env = std::exp(-t * 2.2f);

        // Maritime brass bell harmonics
        float bell = std::sin(2.0f * PI * 880.0f * t) * 0.40f   // Strike note (A5)
                   + std::sin(2.0f * PI * 440.0f * t) * 0.30f   // Hum note (A4)
                   + std::sin(2.0f * PI * 1056.0f * t) * 0.20f  // Minor third
                   + std::sin(2.0f * PI * 1320.0f * t) * 0.15f  // Fifth
                   + std::sin(2.0f * PI * 1760.0f * t) * 0.10f; // Octave

        pcm[i] = static_cast<short>(glm::clamp(bell * env, -1.0f, 1.0f) * 28000.0f);
    }

    alGenBuffers(1, &m_bellBuffer);
    alBufferData(m_bellBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_bellSource);
    alSourcei(m_bellSource, AL_BUFFER, m_bellBuffer);
    alSourcei(m_bellSource, AL_LOOPING, AL_FALSE);
    alSourcef(m_bellSource, AL_REFERENCE_DISTANCE, 10.0f);
    alSourcef(m_bellSource, AL_MAX_DISTANCE, 150.0f);
    alSourcef(m_bellSource, AL_ROLLOFF_FACTOR, 1.2f);
}

void AudioEngine::generateSeagullSound() {
    float duration = 0.65f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    float phase = 0.0f;
    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float progress = t / duration;

        // Characteristic seagull pitch swoop
        float freq = 1800.0f + 950.0f * std::sin(progress * PI) - 300.0f * progress;
        float flutter = 1.0f + 0.15f * std::sin(2.0f * PI * 18.0f * t);

        phase += 2.0f * PI * (freq * flutter) / SAMPLE_RATE;

        float env = std::pow(std::sin(progress * PI), 0.7f);
        float wave = std::sin(phase) + 0.3f * std::sin(phase * 2.0f) + 0.15f * std::sin(phase * 3.0f);
        pcm[i] = static_cast<short>(glm::clamp(wave * env * 0.45f, -1.0f, 1.0f) * 28000.0f);
    }

    alGenBuffers(1, &m_seagullBuffer);
    alBufferData(m_seagullBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_seagullSource);
    alSourcei(m_seagullSource, AL_BUFFER, m_seagullBuffer);
    alSourcei(m_seagullSource, AL_LOOPING, AL_FALSE);
    alSourcef(m_seagullSource, AL_REFERENCE_DISTANCE, 15.0f);
    alSourcef(m_seagullSource, AL_MAX_DISTANCE, 120.0f);
    alSourcef(m_seagullSource, AL_ROLLOFF_FACTOR, 1.0f);
}

void AudioEngine::generateRainSound() {
    // 4.0 seconds seamless rain sound loop (filtered broadband water noise)
    float duration = 4.0f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    float lp = 0.0f;
    for (int i = 0; i < numSamples; ++i) {
        float white = ((rand() % 20000) / 10000.0f) - 1.0f;
        lp += (white - lp) * 0.22f; // Low-pass filter for steady rain roar

        // Random subtle droplet impacts
        float drop = 0.0f;
        if ((rand() % 400) == 0) {
            drop = ((rand() % 2000) / 1000.0f - 1.0f) * 0.45f;
        }

        float sample = lp * 0.65f + drop;
        pcm[i] = static_cast<short>(glm::clamp(sample, -1.0f, 1.0f) * 22000.0f);
    }

    // Circular crossfade
    int crossFade = static_cast<int>(0.25f * SAMPLE_RATE);
    for (int i = 0; i < crossFade; ++i) {
        float blend = static_cast<float>(i) / crossFade;
        int tailIdx = numSamples - crossFade + i;
        pcm[i] = static_cast<short>(pcm[i] * blend + pcm[tailIdx] * (1.0f - blend));
    }

    alGenBuffers(1, &m_rainBuffer);
    alBufferData(m_rainBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_rainSource);
    alSourcei(m_rainSource, AL_BUFFER, m_rainBuffer);
    alSourcei(m_rainSource, AL_LOOPING, AL_TRUE);
    alSourcei(m_rainSource, AL_SOURCE_RELATIVE, AL_TRUE);
    alSourcef(m_rainSource, AL_GAIN, 0.0f);
}

void AudioEngine::generateThunderSound() {
    float duration = 3.2f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    double b0 = 0.0, b1 = 0.0;

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        double white = ((rand() % 20000) / 10000.0) - 1.0;

        b0 = 0.992 * b0 + white * 0.08;
        b1 = 0.985 * b1 + b0 * 0.12;

        // Sharp lightning crack onset (0 to 0.25s)
        float crack = 0.0f;
        if (t < 0.25f) {
            crack = static_cast<float>(white) * std::exp(-t * 24.0f) * 0.95f;
        }

        // Sub-bass rolling thunder rumble (38 Hz to 70 Hz)
        float rumbleFreq = 42.0f + 25.0f * std::sin(t * 3.8f);
        float subThump = std::sin(2.0f * PI * rumbleFreq * t) * std::exp(-t * 0.95f) * 0.50f;
        float tailRumble = static_cast<float>(b1) * std::exp(-t * 0.85f) * (0.85f + 0.35f * std::sin(t * 8.5f));

        float total = crack * 0.6f + subThump + tailRumble * 0.75f;
        pcm[i] = static_cast<short>(glm::clamp(total, -1.0f, 1.0f) * 28000.0f);
    }

    alGenBuffers(1, &m_thunderBuffer);
    alBufferData(m_thunderBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_thunderSource);
    alSourcei(m_thunderSource, AL_BUFFER, m_thunderBuffer);
    alSourcei(m_thunderSource, AL_LOOPING, AL_FALSE);
    alSourcei(m_thunderSource, AL_SOURCE_RELATIVE, AL_TRUE); // Shakes entire soundstage
}

void AudioEngine::generateRadarSound() {
    float duration = 0.08f;
    int numSamples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<short> pcm(numSamples);

    for (int i = 0; i < numSamples; ++i) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float env = std::exp(-t * 45.0f);
        float wave = std::sin(2.0f * PI * 1280.0f * t) * 0.4f;
        pcm[i] = static_cast<short>(glm::clamp(wave * env, -1.0f, 1.0f) * 24000.0f);
    }

    alGenBuffers(1, &m_radarBuffer);
    alBufferData(m_radarBuffer, AL_FORMAT_MONO16, pcm.data(), numSamples * sizeof(short), SAMPLE_RATE);

    alGenSources(1, &m_radarSource);
    alSourcei(m_radarSource, AL_BUFFER, m_radarBuffer);
    alSourcei(m_radarSource, AL_LOOPING, AL_FALSE);
    alSourcei(m_radarSource, AL_SOURCE_RELATIVE, AL_TRUE); // HUD 2D audio
}

#pragma clang diagnostic pop
