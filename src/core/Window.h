#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>

class Window {
public:
    Window(int width, int height, const std::string &title);
    ~Window();

    bool init();
    bool shouldClose() const;
    void close();
    void swapBuffers();
    void pollEvents();

    int getWindowWidth() const { return m_windowWidth; }
    int getWindowHeight() const { return m_windowHeight; }
    int getFramebufferWidth() const { return m_framebufferWidth; }
    int getFramebufferHeight() const { return m_framebufferHeight; }
    int getWidth() const { return m_framebufferWidth; }
    int getHeight() const { return m_framebufferHeight; }
    float getAspectRatio() const {
        return static_cast<float>(m_framebufferWidth) / static_cast<float>(m_framebufferHeight ? m_framebufferHeight : 1);
    }
    GLFWwindow *getGLFWWindow() const { return m_window; }
    float getTime() const { return static_cast<float>(glfwGetTime()); }

private:
    int m_windowWidth;
    int m_windowHeight;
    int m_framebufferWidth;
    int m_framebufferHeight;
    std::string m_title;
    GLFWwindow *m_window;

    static void framebufferSizeCallback(GLFWwindow *window, int width, int height);
};
