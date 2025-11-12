#version 330 core
layout(location = 0) in vec2 aPos; // x,z base grid coords

out vec3 vNormal;
out vec3 vPos;
out vec3 vViewPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float time;

// Gerstner wave params - 8 sóng
#define N_WAVES 8
uniform vec2 waveDir[N_WAVES];
uniform float waveAmp[N_WAVES];
uniform float waveWLen[N_WAVES];
uniform float waveSpeed[N_WAVES];
uniform float steepness[N_WAVES];

const float EPS = 0.02; // tăng một chút để normal nhạy hơn

// ----- Hàm tính toán biến dạng Gerstner -----
vec3 gerstnerDisplace(vec2 pos, float t) {
    vec3 displaced = vec3(pos.x, 0.0, pos.y);

    for (int i = 0; i < N_WAVES; ++i) {
        vec2 d = normalize(waveDir[i]);
        float k = 2.0 * 3.14159265 / waveWLen[i]; // số sóng (wave number)
        float c = waveSpeed[i];                   // tốc độ pha
        float f = k * dot(d, pos) - c * k * t;    // pha sóng
        float A = waveAmp[i];                     // biên độ
        float S = steepness[i];                   // độ dốc (steepness)

        // Áp dụng công thức Gerstner:
        displaced.x += (S * A * d.x) * sin(f);
        displaced.z += (S * A * d.y) * sin(f);
        displaced.y += A * cos(f);
    }

    return displaced;
}

void main() {
    vec2 base = aPos;

    // vị trí chính
    vec3 p = gerstnerDisplace(base, time);

    // các vị trí lân cận để xấp xỉ đạo hàm
    vec3 p_dx = gerstnerDisplace(base + vec2(EPS, 0.0), time);
    vec3 p_dz = gerstnerDisplace(base + vec2(0.0, EPS), time);

    // vector tiếp tuyến và pháp tuyến
    vec3 tangentX = p_dx - p;
    vec3 tangentZ = p_dz - p;
    vec3 normal = normalize(cross(tangentZ, tangentX));

    // transform sang world space
    vNormal = mat3(transpose(inverse(model))) * normal;
    vec4 worldPos = model * vec4(p, 1.0);
    vPos = worldPos.xyz;
    vViewPos = (view * worldPos).xyz;

    gl_Position = projection * view * worldPos;
}
