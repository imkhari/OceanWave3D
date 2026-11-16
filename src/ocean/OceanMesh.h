#pragma once

#include "../renderer/Mesh.h"
#include "../renderer/Shader.h"

class OceanMesh {
public:
    OceanMesh(int resolution = 240, float size = 650.0f);

    void init(int resolution = 240, float size = 650.0f);
    void rebuild(int resolution);
    void render() const;

    int getResolution() const { return m_resolution; }
    float getSize() const { return m_size; }

private:
    int m_resolution;
    float m_size;
    Mesh m_mesh;
};
