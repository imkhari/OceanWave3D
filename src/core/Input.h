#pragma once

#include <GLFW/glfw3.h>

class Input {
public:
    explicit Input(GLFWwindow *window) : m_window(window) {}

    bool isKeyDown(int key) const {
        return glfwGetKey(m_window, key) == GLFW_PRESS;
    }

private:
    GLFWwindow *m_window;
};
