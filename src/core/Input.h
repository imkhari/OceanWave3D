#pragma once

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <array>

class Input {
public:
    explicit Input(GLFWwindow *window) : m_window(window) {}

    void replay(bool enabled) { m_replay = enabled; m_keys.fill(false); }
    void setReplayKey(int key, bool down) { if (key >= 0 && key <= GLFW_KEY_LAST) m_keys[key] = down; }
    bool isKeyDown(int key) const {
        if (ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard) return false;
        if (m_replay) return key >= 0 && key <= GLFW_KEY_LAST && m_keys[key];
        return glfwGetKey(m_window, key) == GLFW_PRESS;
    }

private:
    bool m_replay = false;
    std::array<bool, GLFW_KEY_LAST + 1> m_keys{};
    GLFWwindow *m_window;
};
