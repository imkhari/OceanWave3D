#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

class Mesh {
public:
    Mesh();
    ~Mesh();

    Mesh(const Mesh &) = delete;
    Mesh &operator=(const Mesh &) = delete;
    Mesh(Mesh &&other) noexcept;
    Mesh &operator=(Mesh &&other) noexcept;

    void upload(const std::vector<glm::vec3> &vertices, const std::vector<unsigned int> &indices);
    void draw() const;
    void drawLines(GLsizei maxIndices = -1) const;
    void updateVertices(const std::vector<glm::vec3> &vertices);
    void drawWireframe() const;

    GLsizei getIndexCount() const { return m_indexCount; }
    GLuint getVAO() const { return m_vao; }

    static void addCube(std::vector<glm::vec3> &vertices, std::vector<unsigned int> &indices,
                        const glm::vec3 &center, const glm::vec3 &size);

    static void addRoundedBox(std::vector<glm::vec3> &vertices, std::vector<unsigned int> &indices,
                              const glm::vec3 &center, const glm::vec3 &size, float radius);
    static void addCone(std::vector<glm::vec3> &vertices, std::vector<unsigned int> &indices,
                        const glm::vec3 &base, float radius, float height, int segments = 12);
private:
    GLuint m_vao;
    GLuint m_vbo;
    GLuint m_ebo;
    GLsizei m_indexCount;

    void cleanup();
};
