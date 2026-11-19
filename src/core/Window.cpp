#include "Window.h"
#include <iostream>

Window::Window(int width, int height, const std::string &title)
    : m_windowWidth(width),
      m_windowHeight(height),
      m_framebufferWidth(width),
      m_framebufferHeight(height),
      m_title(title),
      m_window(nullptr) {}

Window::~Window() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}

bool Window::init() {
    if (!glfwInit()) {
        std::cerr << "[GLFW ERROR] Failed to initialize GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight, m_title.c_str(), nullptr, nullptr);
    if (!m_window) {
        std::cerr << "[GLFW ERROR] Failed to create window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSetWindowUserPointer(m_window, this);
    glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);

    // CRITICAL FOR RETINA ON MACOS: Query actual framebuffer pixel resolution
    glfwGetWindowSize(m_window, &m_windowWidth, &m_windowHeight);
    glfwGetFramebufferSize(m_window, &m_framebufferWidth, &m_framebufferHeight);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "[GLAD ERROR] Failed to initialize GLAD\n";
        return false;
    }

    std::cout << "[INFO] Window Points: " << m_windowWidth << "x" << m_windowHeight << "\n";
    std::cout << "[INFO] Framebuffer Pixels: " << m_framebufferWidth << "x" << m_framebufferHeight << "\n";
    std::cout << "[INFO] OpenGL Version: " << glGetString(GL_VERSION) << "\n";
    std::cout << "[INFO] GPU Renderer:   " << glGetString(GL_RENDERER) << "\n";

    glfwSwapInterval(1);
    glDisable(GL_FRAMEBUFFER_SRGB); // Resolve owns the only display conversion.
    glViewport(0, 0, m_framebufferWidth, m_framebufferHeight);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    return true;
}

void Window::framebufferSizeCallback(GLFWwindow *window, int width, int height) {
    auto *self = static_cast<Window *>(glfwGetWindowUserPointer(window));
    if (self) {
        self->m_framebufferWidth = (width > 0) ? width : 1;
        self->m_framebufferHeight = (height > 0) ? height : 1;
        glfwGetWindowSize(window, &self->m_windowWidth, &self->m_windowHeight);
        glViewport(0, 0, self->m_framebufferWidth, self->m_framebufferHeight);
    }
}

bool Window::shouldClose() const {
    return glfwWindowShouldClose(m_window);
}

void Window::close() {
    glfwSetWindowShouldClose(m_window, GLFW_TRUE);
}

void Window::swapBuffers() {
    glfwSwapBuffers(m_window);
}

void Window::pollEvents() {
    glfwPollEvents();
}
