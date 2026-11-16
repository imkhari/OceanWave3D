#include "Mesh.h"
#include <utility>

Mesh::Mesh() : m_vao(0), m_vbo(0), m_ebo(0), m_indexCount(0) {}

Mesh::~Mesh() {
    cleanup();
}

Mesh::Mesh(Mesh &&other) noexcept
    : m_vao(other.m_vao), m_vbo(other.m_vbo), m_ebo(other.m_ebo), m_indexCount(other.m_indexCount) {
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_ebo = 0;
    other.m_indexCount = 0;
}

Mesh &Mesh::operator=(Mesh &&other) noexcept {
    if (this != &other) {
        cleanup();
        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_ebo = other.m_ebo;
        m_indexCount = other.m_indexCount;
        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_ebo = 0;
        other.m_indexCount = 0;
    }
    return *this;
}

void Mesh::cleanup() {
    if (m_ebo != 0) { glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
    if (m_vbo != 0) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_vao != 0) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    m_indexCount = 0;
}

void Mesh::upload(const std::vector<glm::vec3> &vertices, const std::vector<unsigned int> &indices) {
    cleanup();

    m_indexCount = static_cast<GLsizei>(indices.size());

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void Mesh::draw() const {
    if (m_vao != 0 && m_indexCount > 0) {
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Mesh::drawLines() const {
    if (m_vao != 0 && m_indexCount > 0) {
        glBindVertexArray(m_vao);
        glDrawElements(GL_LINES, m_indexCount, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Mesh::drawWireframe() const {
    if (m_vao != 0 && m_indexCount > 0) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        draw();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void Mesh::addCube(std::vector<glm::vec3> &vertices, std::vector<unsigned int> &indices,
                   const glm::vec3 &center, const glm::vec3 &size) {
    glm::vec3 half = size * 0.5f;
    glm::vec3 v[8] = {
        center + glm::vec3(-half.x, -half.y,  half.z), // 0: BLF
        center + glm::vec3( half.x, -half.y,  half.z), // 1: BRF
        center + glm::vec3( half.x,  half.y,  half.z), // 2: TRF
        center + glm::vec3(-half.x,  half.y,  half.z), // 3: TLF
        center + glm::vec3(-half.x, -half.y, -half.z), // 4: BLB
        center + glm::vec3( half.x, -half.y, -half.z), // 5: BRB
        center + glm::vec3( half.x,  half.y, -half.z), // 6: TRB
        center + glm::vec3(-half.x,  half.y, -half.z)  // 7: TLB
    };

    unsigned int start = static_cast<unsigned int>(vertices.size());
    for (int i = 0; i < 8; i++) {
        vertices.push_back(v[i]);
    }

    unsigned int idx[] = {
        0, 1, 2, 0, 2, 3, // Front
        1, 5, 6, 1, 6, 2, // Right
        5, 4, 7, 5, 7, 6, // Back
        4, 0, 3, 4, 3, 7, // Left
        3, 2, 6, 3, 6, 7, // Top
        4, 5, 1, 4, 1, 0  // Bottom
    };

    for (int i = 0; i < 36; i++) {
        indices.push_back(start + idx[i]);
    }
}
