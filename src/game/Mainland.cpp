#include "Mainland.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

static constexpr float PI = 3.14159265358979323846f;

static float terrainHeight(float x, float z) {
        // Distance inland from the harbor shoreline edge
        float dx = x - 76.0f;
        float dz = -60.0f - z; // positive as z goes negative (northward)

        // Coastal contour: shoreline curves along a gentle natural bay
        float coastDist = dx + dz * 0.35f - 18.0f;
        if (coastDist <= 0.0f) {
            // Shallow tidal seabed
            return coastDist * 0.25f - 0.5f;
        }

        // Shoreline gentle rise
        float h = std::pow(coastDist / 20.0f, 1.25f) * 2.8f;

        // Rolling hill undulations
        float hill1 = std::sin(x * 0.035f + z * 0.025f) * 6.5f;
        float hill2 = std::cos(x * 0.065f - z * 0.045f) * 4.0f;
        h += (hill1 + hill2) * glm::smoothstep(0.0f, 28.0f, coastDist);

        // Northern Cliff Headland (z < -180)
        if (z < -160.0f) {
            float cliffFactor = glm::clamp((-160.0f - z) / 45.0f, 0.0f, 1.0f);
            h += cliffFactor * (18.0f + 12.0f * std::sin(x * 0.05f));
        }

        // Eastern Distant Mountains (x > 180, z < -170)
        if (x > 165.0f) {
            float mtnDist = (x - 165.0f) * 0.55f + (-120.0f - z) * 0.45f;
            if (mtnDist > 0.0f) {
                float mountainPeak = std::sin(x * 0.04f) * std::cos(z * 0.04f);
                h += (mtnDist * 0.55f) + std::max(0.0f, mountainPeak) * 28.0f;
            }
        }

        return h;
}


Mainland::Mainland()
    : m_origin(95.0f, 0.0f, -125.0f) {}

void Mainland::init(const glm::vec3 &origin) {
    m_origin = origin;
    buildTerrainGeometry();
    buildVillageGeometry();
    buildVegetationGeometry();
}

void Mainland::render(const Shader &shader) const {
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);

    // 1. Golden Sand Beaches & Shoreline
    shader.setInt("uMaterialID", 16);
    m_beachMesh.draw();

    // 2. Rolling Emerald Grassy Hills & Coastal Meadows
    shader.setInt("uMaterialID", 17);
    m_hillsMesh.draw();

    // 3. Granite Sea Cliffs, Headlands & Distant Mountain Peaks
    shader.setInt("uMaterialID", 18);
    m_cliffsMesh.draw();

    // 4. Coastal Village Cottages, Watchtower, Boathouse & Windows
    shader.setInt("uMaterialID", 19);
    m_villageMesh.draw();

    shader.setInt("uMaterialID",21); m_roofMesh.draw();
    shader.setInt("uMaterialID",22); m_windowMesh.draw();

    // 5. Coastal Pine Trees (Evergreen Conifers)
    shader.setInt("uMaterialID", 20);
    m_treesMesh.draw();
}

// -----------------------------------------------------------------------------
// Procedural Mainland Coastline Geometry
// -----------------------------------------------------------------------------

void Mainland::buildTerrainGeometry() {
    std::vector<glm::vec3> beachV, hillsV, cliffsV;
    std::vector<unsigned int> beachI, hillsI, cliffsI;

    // Coastal Mainland Grid: spans X from 76m to 310m, Z from -60m to -300m
    const int GRID_X = 80;
    const int GRID_Z = 80;
    float minX = 76.0f,  maxX = 300.0f;
    float minZ = -300.0f, maxZ = -60.0f;
    float stepX = (maxX - minX) / (GRID_X - 1);
    float stepZ = (maxZ - minZ) / (GRID_Z - 1);


    // Build grid vertices and classify into Beach, Grassy Hills, or Granite Cliffs
    struct GridPt {
        glm::vec3 pos;
        glm::vec3 normal;
        float height;
        float slope; // dot with UP vector
    };

    std::vector<GridPt> grid(GRID_X * GRID_Z);
    for (int j = 0; j < GRID_Z; ++j) {
        float z = minZ + j * stepZ;
        for (int i = 0; i < GRID_X; ++i) {
            float x = minX + i * stepX;
            float y = terrainHeight(x, z);
            grid[j * GRID_X + i].pos = glm::vec3(x, y, z);
            grid[j * GRID_X + i].height = y;
        }
    }

    // Compute normals
    for (int j = 0; j < GRID_Z; ++j) {
        for (int i = 0; i < GRID_X; ++i) {
            float x0 = (i > 0) ? grid[j * GRID_X + (i - 1)].pos.x : grid[j * GRID_X + i].pos.x - stepX;
            float y0 = (i > 0) ? grid[j * GRID_X + (i - 1)].pos.y : grid[j * GRID_X + i].pos.y;
            float x1 = (i < GRID_X - 1) ? grid[j * GRID_X + (i + 1)].pos.x : grid[j * GRID_X + i].pos.x + stepX;
            float y1 = (i < GRID_X - 1) ? grid[j * GRID_X + (i + 1)].pos.y : grid[j * GRID_X + i].pos.y;

            float z0 = (j > 0) ? grid[(j - 1) * GRID_X + i].pos.z : grid[j * GRID_X + i].pos.z - stepZ;
            float yz0 = (j > 0) ? grid[(j - 1) * GRID_X + i].pos.y : grid[j * GRID_X + i].pos.y;
            float z1 = (j < GRID_Z - 1) ? grid[(j + 1) * GRID_X + i].pos.z : grid[j * GRID_X + i].pos.z + stepZ;
            float yz1 = (j < GRID_Z - 1) ? grid[(j + 1) * GRID_X + i].pos.y : grid[j * GRID_X + i].pos.y;

            glm::vec3 dx(x1 - x0, y1 - y0, 0.0f);
            glm::vec3 dz(0.0f, yz1 - yz0, z1 - z0);
            glm::vec3 n = glm::normalize(glm::cross(dz, dx));
            grid[j * GRID_X + i].normal = n;
            grid[j * GRID_X + i].slope = n.y; // 1.0 = flat ground, 0.0 = sheer vertical
        }
    }

    // Triangulate grid quads and route to appropriate material mesh
    for (int j = 0; j < GRID_Z - 1; ++j) {
        for (int i = 0; i < GRID_X - 1; ++i) {
            int idx00 = j * GRID_X + i;
            int idx10 = j * GRID_X + (i + 1);
            int idx01 = (j + 1) * GRID_X + i;
            int idx11 = (j + 1) * GRID_X + (i + 1);

            const auto &p00 = grid[idx00];
            const auto &p10 = grid[idx10];
            const auto &p01 = grid[idx01];
            const auto &p11 = grid[idx11];

            float avgH = (p00.height + p10.height + p01.height + p11.height) * 0.25f;
            float avgSlope = (p00.slope + p10.slope + p01.slope + p11.slope) * 0.25f;

            auto emitQuad = [](std::vector<glm::vec3> &V, std::vector<unsigned int> &I,
                               const glm::vec3 &v0, const glm::vec3 &v1,
                               const glm::vec3 &v2, const glm::vec3 &v3) {
                unsigned base = static_cast<unsigned>(V.size());
                V.push_back(v0);
                V.push_back(v1);
                V.push_back(v2);
                V.push_back(v3);
                I.insert(I.end(), { base, base + 2, base + 1, base + 1, base + 2, base + 3 });
            };

            // 1. Beach Sand: low elevation near shoreline
            if (avgH < 2.2f) {
                emitQuad(beachV, beachI, p00.pos, p10.pos, p01.pos, p11.pos);
            }
            // 2. Granite Sea Cliffs & High Mountain Peaks: steep slopes or high elevation
            else if (avgSlope < 0.65f || avgH > 35.0f) {
                emitQuad(cliffsV, cliffsI, p00.pos, p10.pos, p01.pos, p11.pos);
            }
            // 3. Lush Green Grassy Hills & Meadows
            else {
                emitQuad(hillsV, hillsI, p00.pos, p10.pos, p01.pos, p11.pos);
            }
        }
    }

    // Coastal Sea Stacks (Offshore jagged rock crags rising from the surf)
    struct SeaStack { float x, z, r, h; };
    SeaStack stacks[] = {
        { 74.0f, -82.0f,  4.5f, 7.0f },
        { 68.0f, -145.0f, 6.0f, 11.5f },
        { 70.0f, -185.0f, 7.5f, 15.0f },
        { 82.0f, -220.0f, 8.0f, 18.0f }
    };
    for (const auto &st : stacks) {
        Mesh::addCone(cliffsV,cliffsI,{st.x,-3.0f,st.z},st.r,st.h+4.0f,9);
        Mesh::addCone(cliffsV,cliffsI,{st.x+st.r*0.4f,-2.5f,st.z+st.r*0.25f},st.r*0.65f,st.h*0.9f,7);
    }

    m_beachMesh.upload(beachV, beachI);
    m_hillsMesh.upload(hillsV, hillsI);
    m_cliffsMesh.upload(cliffsV, cliffsI);
}

// -----------------------------------------------------------------------------
// Coastal Fishing Village & Medieval Watchtower
// -----------------------------------------------------------------------------

void Mainland::buildVillageGeometry() {
    std::vector<glm::vec3> V, roofV, windowV;
    std::vector<unsigned int> I, roofI, windowI;

    // Helper: Add complete cottage (Walls + Roof + Chimney + Porch + Glowing Windows)
    auto addCottage = [&](glm::vec3 pos, const glm::vec3 &size, float roofAngle, bool hasPorch) {
        pos.y = terrainHeight(pos.x,pos.z) + 0.1f;
        float hw = size.x * 0.5f;
        float hh = size.y * 0.5f;
        float hd = size.z * 0.5f;

        // 1. Stone Wall Body
        Mesh::addCube(V, I, pos + glm::vec3(0.0f, hh, 0.0f), size);

        // 2. Terracotta Slanted Roof
        float roofH = size.y * 0.48f;
        unsigned roofBase=static_cast<unsigned>(roofV.size());
        for(float z : {-hd-0.4f,hd+0.4f}) {
            roofV.push_back(pos+glm::vec3(-hw-0.4f,size.y,z));
            roofV.push_back(pos+glm::vec3(hw+0.4f,size.y,z));
            roofV.push_back(pos+glm::vec3(0,size.y+roofH,z));
        }
        const unsigned faces[]={0,2,1,3,4,5,0,3,5,0,5,2,1,2,5,1,5,4,0,1,4,0,4,3};
        for(auto index:faces) roofI.push_back(roofBase+index);

        // 3. Brick Chimney
        Mesh::addCube(V, I, pos + glm::vec3(hw * 0.65f, size.y + roofH + 0.7f, -hd * 0.4f),
                      glm::vec3(0.85f, 2.2f, 0.85f));

        // 4. Glowing Windows (Emissive amber glass panes)
        float winW = 0.90f, winH = 1.10f;
        // Front windows facing sea (+Z)
        Mesh::addCube(windowV, windowI, pos + glm::vec3(-hw * 0.45f, hh + 0.2f, hd + 0.08f),
                      glm::vec3(winW, winH, 0.12f));
        Mesh::addCube(windowV, windowI, pos + glm::vec3(hw * 0.45f, hh + 0.2f, hd + 0.08f),
                      glm::vec3(winW, winH, 0.12f));
        // Side window
        Mesh::addCube(windowV, windowI, pos + glm::vec3(hw + 0.08f, hh + 0.2f, 0.0f),
                      glm::vec3(0.12f, winH, winW));

        // 5. Wooden Entrance Porch
        if (hasPorch) {
            Mesh::addCube(V, I, pos + glm::vec3(0.0f, 0.25f, hd + 1.2f),
                          glm::vec3(size.x * 0.45f, 0.25f, 2.0f)); // Porch deck
            Mesh::addCube(V, I, pos + glm::vec3(-size.x * 0.2f, 1.25f, hd + 2.0f),
                          glm::vec3(0.16f, 2.2f, 0.16f)); // Post L
            Mesh::addCube(V, I, pos + glm::vec3(size.x * 0.2f, 1.25f, hd + 2.0f),
                          glm::vec3(0.16f, 2.2f, 0.16f));  // Post R
        }
    };

    // 5 Picturesque Fisherman Cottages arranged in a coastal hamlet
    addCottage(glm::vec3(104.0f, 2.8f, -108.0f), glm::vec3(8.5f, 4.4f, 6.2f),  0.0f, true);
    addCottage(glm::vec3(118.0f, 5.2f, -122.0f), glm::vec3(9.5f, 4.8f, 6.8f),  0.0f, true);
    addCottage(glm::vec3(112.0f, 7.4f, -138.0f), glm::vec3(8.0f, 4.2f, 5.8f),  0.0f, false);
    addCottage(glm::vec3(132.0f, 10.5f, -132.0f), glm::vec3(9.2f, 4.6f, 6.5f), 0.0f, true);
    addCottage(glm::vec3(98.0f,  2.2f, -128.0f), glm::vec3(7.8f, 4.0f, 5.5f),  0.0f, false);

    // Boathouse & Slipway near the harbor waterline
    glm::vec3 boathousePos(90.0f, terrainHeight(90,-118), -118.0f);
    Mesh::addCube(V, I, boathousePos + glm::vec3(0.0f, 2.2f, 0.0f), glm::vec3(11.0f, 4.4f, 7.5f));
    Mesh::addCube(V, I, boathousePos + glm::vec3(0.0f, 4.6f, 0.0f), glm::vec3(11.8f, 1.4f, 8.2f)); // Roof
    Mesh::addCube(V, I, boathousePos + glm::vec3(-4.0f, -0.1f, 3.5f), glm::vec3(3.2f, 0.25f, 8.0f)); // Slipway ramp
    Mesh::addCube(V, I, boathousePos + glm::vec3(4.0f, -0.1f, 3.5f), glm::vec3(3.2f, 0.25f, 8.0f));

    // Ancient Coastal Fortress Watchtower (Granite bastion overlooking the bay)
    glm::vec3 towerPos(112.0f, terrainHeight(112,-86), -86.0f);
    // Base Tower Drum
    Mesh::addCube(V, I, towerPos + glm::vec3(0.0f, 7.5f, 0.0f), glm::vec3(8.0f, 15.0f, 8.0f));
    // Crown Battlements / Crenellations
    Mesh::addCube(V, I, towerPos + glm::vec3(0.0f, 15.6f, 0.0f), glm::vec3(9.2f, 1.4f, 9.2f));
    // Battlements Merlons (Fortress embrasures)
    for (int k = -1; k <= 1; k += 2) {
        Mesh::addCube(V, I, towerPos + glm::vec3(k * 4.2f, 16.8f, 0.0f), glm::vec3(0.8f, 1.2f, 2.4f));
        Mesh::addCube(V, I, towerPos + glm::vec3(0.0f, 16.8f, k * 4.2f), glm::vec3(2.4f, 1.2f, 0.8f));
    }
    // High Watchtower Glowing Lantern (Harbor overlook beacon)
    Mesh::addCube(V, I, towerPos + glm::vec3(0.0f, 17.5f, 0.0f), glm::vec3(0.45f, 1.8f, 0.45f));
    Mesh::addCube(V, I, towerPos + glm::vec3(0.0f, 18.6f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f)); // Glowing dome

    // Village Cobblestone Pathway connecting pier ramp to cottages
    float pathPoints[][2] = {
        { 82.0f, -105.0f }, { 92.0f, -107.0f }, { 104.0f, -112.0f },
        { 114.0f, -120.0f }, { 122.0f, -126.0f }
    };
    for (int p = 0; p < 4; ++p) {
        glm::vec3 p0(pathPoints[p][0], 1.6f + p * 1.0f, pathPoints[p][1]);
        glm::vec3 p1(pathPoints[p + 1][0], 1.6f + (p + 1) * 1.0f, pathPoints[p + 1][1]);
        Mesh::addCube(V, I, (p0 + p1) * 0.5f,
                      glm::vec3(glm::length(p1 - p0) + 0.4f, 0.15f, 2.4f));
    }

    m_villageMesh.upload(V, I);
    m_roofMesh.upload(roofV, roofI);
    m_windowMesh.upload(windowV, windowI);
}

// -----------------------------------------------------------------------------
// Coastal Vegetation (Pine Trees / Conifers)
// -----------------------------------------------------------------------------

void Mainland::buildVegetationGeometry() {
    std::vector<glm::vec3> V;
    std::vector<unsigned int> I;

    auto addPineTree = [&](const glm::vec3 &basePos, float scale) {
        float trunkH = 3.5f * scale;
        float trunkR = 0.45f * scale;

        // Tree Trunk (Wood Bark)
        Mesh::addCube(V, I, basePos + glm::vec3(0.0f, trunkH * 0.5f, 0.0f),
                      glm::vec3(trunkR, trunkH, trunkR));

        for (int tier=0;tier<4;++tier) {
            float r=(2.2f-tier*0.43f)*scale;
            float h=(3.6f-tier*0.35f)*scale;
            Mesh::addCone(V,I,basePos+glm::vec3(0,trunkH-0.8f+tier*1.25f*scale,0),r,h,11);
        }
    };

    // 42 Pine Trees placed around hills, bluffs, and around the village
    struct TreeSpawn { float x, y, z, s; };
    TreeSpawn trees[] = {
        // Village surroundings
        { 100.0f, 2.6f, -102.0f, 1.1f }, { 110.0f, 3.8f, -102.0f, 1.2f },
        { 124.0f, 5.8f, -114.0f, 1.3f }, { 130.0f, 7.2f, -118.0f, 1.0f },
        { 105.0f, 6.2f, -132.0f, 1.15f }, { 116.0f, 8.5f, -145.0f, 1.25f },
        { 94.0f,  2.0f, -136.0f, 0.95f }, { 88.0f,  1.4f, -126.0f, 1.05f },

        // Bluff & Ridge lines
        { 102.0f, 14.5f, -82.0f, 1.3f },  { 120.0f, 18.0f, -84.0f, 1.4f },
        { 128.0f, 19.5f, -92.0f, 1.2f },  { 138.0f, 17.5f, -100.0f, 1.35f },
        { 146.0f, 15.0f, -112.0f, 1.1f }, { 152.0f, 16.5f, -125.0f, 1.4f },

        // Inland forest grove
        { 140.0f, 12.0f, -148.0f, 1.3f }, { 148.0f, 13.5f, -156.0f, 1.25f },
        { 156.0f, 15.0f, -142.0f, 1.4f }, { 164.0f, 17.0f, -150.0f, 1.3f },
        { 172.0f, 18.5f, -162.0f, 1.2f }, { 180.0f, 21.0f, -154.0f, 1.45f },

        // Northern Cliff slopes
        { 86.0f,  6.0f, -165.0f, 1.15f }, { 92.0f,  9.5f, -178.0f, 1.3f },
        { 98.0f,  13.0f, -192.0f, 1.25f }, { 108.0f, 18.0f, -205.0f, 1.4f },
        { 116.0f, 22.0f, -218.0f, 1.35f }, { 124.0f, 25.0f, -230.0f, 1.2f },

        // Sea stack vegetation
        { 74.0f,  7.0f, -82.0f,  0.85f }, { 68.0f,  11.5f, -145.0f, 0.9f },
        { 70.0f,  15.0f, -185.0f, 1.0f },

        // High mountain approach pines
        { 188.0f, 24.0f, -175.0f, 1.3f }, { 196.0f, 28.0f, -185.0f, 1.2f },
        { 205.0f, 32.0f, -195.0f, 1.35f }, { 215.0f, 36.0f, -205.0f, 1.15f }
    };

    for (const auto &t : trees) {
        addPineTree(glm::vec3(t.x, t.x < 76 ? t.y : terrainHeight(t.x,t.z), t.z), t.s);
    }

    m_treesMesh.upload(V, I);
}
