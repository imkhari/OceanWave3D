#pragma once

#include <glad/glad.h>
#include <iostream>

class Framebuffer {
public:
    Framebuffer();
    ~Framebuffer();

    bool init(int width, int height, bool useHdr = true);
    void resize(int width, int height);
    void bind() const;
    void unbind() const;

    GLuint getColorTexture() const { return m_colorTexture; }
    GLuint getDepthTexture() const { return m_depthTexture; }
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

private:
    GLuint m_fbo;
    GLuint m_colorTexture;
    GLuint m_depthTexture;
    int m_width;
    int m_height;
    bool m_useHdr;

    void cleanup();
};
