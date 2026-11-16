#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 vPos;
out vec3 vNormal;
out float vWaveHeightVal;
out float vJacobian;
out vec2 vUV;
out vec4 vClipPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float time;
uniform float waveHeight;

// Camera-snapped infinite ocean grid
uniform vec3 uCamPos;
uniform float uOceanSize;
uniform int uResolution;

#define N_WAVES 16
uniform vec2  waveDir[N_WAVES];
uniform float waveAmp[N_WAVES];
uniform float waveWLen[N_WAVES];
uniform float waveOmega[N_WAVES];
uniform float steepness[N_WAVES];

vec3 gerstnerWithNormalAndJacobian(vec2 pos, float t, out vec3 outNormal, out float outJacobian) {
    vec3 p = vec3(pos.x, 0.0, pos.y);
    float jxx = 1.0;
    float jzz = 1.0;
    float jxz = 0.0;
    vec3 n = vec3(0.0, 1.0, 0.0);

    for (int i = 0; i < N_WAVES; ++i) {
        vec2  d = normalize(waveDir[i]);
        float k = 6.2831853 / waveWLen[i];
        float f = k * dot(d, pos) - waveOmega[i] * t;
        float A = waveAmp[i] * waveHeight;
        float S = steepness[i];
        float cosF = cos(f);
        float sinF = sin(f);

        // Trochoidal horizontal and vertical displacements
        p.x += (S * A * d.x) * sinF;
        p.z += (S * A * d.y) * sinF;
        p.y += A * cosF;

        // Analytical Gerstner wave normal derivation
        float WA = S * A * k;
        n.x -= d.x * (A * k) * sinF;
        n.z -= d.y * (A * k) * sinF;
        n.y -= WA * cosF;

        // Jacobian determinant accumulation for wave folding whitecaps
        jxx -= d.x * d.x * WA * cosF;
        jzz -= d.y * d.y * WA * cosF;
        jxz -= d.x * d.y * WA * cosF;
    }

    outNormal = normalize(n);
    outJacobian = jxx * jzz - jxz * jxz;
    return p;
}

void main() {
    // Snap grid center to camera position in discrete steps to prevent vertex swimming
    float gridStep = uOceanSize / float(max(uResolution - 1, 1));
    vec2 snappedOffset = floor(uCamPos.xz / gridStep) * gridStep;

    vec2 samplePos = aPos.xz + snappedOffset;
    vec3 normal = vec3(0.0, 1.0, 0.0);
    float jacobianDet = 1.0;
    vec3 p = gerstnerWithNormalAndJacobian(samplePos, time, normal, jacobianDet);

    vec3 worldP = (model * vec4(p, 1.0)).xyz;
    
    vPos = worldP;
    vNormal = normal;
    vWaveHeightVal = p.y;
    vJacobian = jacobianDet;
    vUV = samplePos;

    vClipPos = projection * view * vec4(worldP, 1.0);
    gl_Position = vClipPos;
}
