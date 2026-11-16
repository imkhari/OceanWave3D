#include "OceanMesh.h"
#include <vector>
#include <algorithm>

OceanMesh::OceanMesh(int resolution, float size)
    : m_resolution(resolution), m_size(size) {
    init(resolution, size);
}

void OceanMesh::init(int resolution, float size) {
    m_size = size;
    rebuild(resolution);
}

void OceanMesh::rebuild(int resolution) {
    m_resolution = std::max(4, resolution);

    std::vector<glm::vec3> verts;
    verts.reserve(static_cast<size_t>(m_resolution) * static_cast<size_t>(m_resolution));

    std::vector<unsigned int> idx;
    idx.reserve(static_cast<size_t>(m_resolution - 1) * static_cast<size_t>(m_resolution - 1) * 6);

    for (int z = 0; z < m_resolution; ++z) {
        for (int x = 0; x < m_resolution; ++x) {
            float fx = (static_cast<float>(x) / (m_resolution - 1) - 0.5f) * m_size;
            float fz = (static_cast<float>(z) / (m_resolution - 1) - 0.5f) * m_size;
            verts.emplace_back(fx, 0.0f, fz);
        }
    }

    for (int z = 0; z < m_resolution - 1; ++z) {
        for (int x = 0; x < m_resolution - 1; ++x) {
            int i0 = z * m_resolution + x;
            int i1 = z * m_resolution + x + 1;
            int i2 = (z + 1) * m_resolution + x;
            int i3 = (z + 1) * m_resolution + x + 1;

            idx.push_back(i0);
            idx.push_back(i2);
            idx.push_back(i1);

            idx.push_back(i1);
            idx.push_back(i2);
            idx.push_back(i3);
        }
    }

    m_mesh.upload(verts, idx);
}

void OceanMesh::render() const {
    m_mesh.draw();
}
