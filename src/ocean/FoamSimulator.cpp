#include "FoamSimulator.h"
#include <cmath>
#include <iostream>

FoamSimulator::FoamSimulator()
    : m_resolution(512),
      m_domainSize(240.0f),
      m_domainCenter(0.0f),
      m_prevCenter(0.0f),
      m_currentRead(0),
      m_currentWrite(1),
      m_quadVAO(0),
      m_quadVBO(0) {
    m_fbo[0] = m_fbo[1] = 0;
    m_colorTextures[0] = m_colorTextures[1] = 0;
}

FoamSimulator::~FoamSimulator() {
    cleanup();
}

void FoamSimulator::cleanup() {
    for (int i = 0; i < 2; ++i) {
        if (m_colorTextures[i] != 0) { glDeleteTextures(1, &m_colorTextures[i]); m_colorTextures[i] = 0; }
        if (m_fbo[i] != 0) { glDeleteFramebuffers(1, &m_fbo[i]); m_fbo[i] = 0; }
    }
    if (m_quadVBO != 0) { glDeleteBuffers(1, &m_quadVBO); m_quadVBO = 0; }
    if (m_quadVAO != 0) { glDeleteVertexArrays(1, &m_quadVAO); m_quadVAO = 0; }
}

void FoamSimulator::setupBuffers() {
    float quadVertices[] = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };

    glGenVertexArrays(1, &m_quadVAO);
    glGenBuffers(1, &m_quadVBO);

    glBindVertexArray(m_quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

bool FoamSimulator::init(int resolution, float domainSize) {
    cleanup();
    m_resolution = resolution;
    m_domainSize = domainSize;
    setupBuffers();

    if (!m_simShader.loadFromFiles("assets/shaders/foam_sim.vert", "assets/shaders/foam_sim.frag")) {
        std::cerr << "[FoamSimulator ERROR] Failed to load foam simulation shader\n";
        return false;
    }

    for (int i = 0; i < 2; ++i) {
        glGenFramebuffers(1, &m_fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[i]);

        glGenTextures(1, &m_colorTextures[i]);
        glBindTexture(GL_TEXTURE_2D, m_colorTextures[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, m_resolution, m_resolution, 0, GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTextures[i], 0);

        // Clear texture to zero
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "[FoamSimulator ERROR] Ping-pong FBO " << i << " incomplete!\n";
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            return false;
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void FoamSimulator::update(float dt, const glm::vec3 &boatPos, float boatSpeed, float boatHeading) {
    m_prevCenter = m_domainCenter;
    m_domainCenter = glm::vec2(boatPos.x, boatPos.z);
    glm::vec2 shiftWorld = m_domainCenter - m_prevCenter;
    glm::vec2 uvShift = shiftWorld / m_domainSize;

    // Calculate bow and stern in world space
    glm::vec2 forward(std::sin(boatHeading), std::cos(boatHeading));
    glm::vec2 bowPos = m_domainCenter + forward * 5.5f;
    glm::vec2 sternPos = m_domainCenter - forward * 5.0f;

    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo[m_currentWrite]);
    glViewport(0, 0, m_resolution, m_resolution);
    glDisable(GL_DEPTH_TEST);

    m_simShader.use();
    m_simShader.setFloat("dt", dt);
    m_simShader.setVec2("uUvShift", uvShift);
    m_simShader.setVec2("uDomainCenter", m_domainCenter);
    m_simShader.setFloat("uDomainSize", m_domainSize);
    m_simShader.setVec2("uBoatPos", m_domainCenter);
    m_simShader.setVec2("uBowPos", bowPos);
    m_simShader.setVec2("uSternPos", sternPos);
    m_simShader.setFloat("uBoatSpeed", boatSpeed);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_colorTextures[m_currentRead]);
    m_simShader.setInt("uPreviousFoam", 0);

    glBindVertexArray(m_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Swap buffers
    std::swap(m_currentRead, m_currentWrite);
}
