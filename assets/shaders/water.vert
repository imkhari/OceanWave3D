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
uniform int uGridResolution;
uniform sampler2D uFoamTexture;
uniform vec2 uFoamCenter;
uniform float uFoamSize;

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
    vec3 tangentX = vec3(1,0,0);
    vec3 tangentZ = vec3(0,0,1);

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
        tangentX += vec3(d.x*d.x*WA*cosF, -d.x*A*k*sinF, d.x*d.y*WA*cosF);
        tangentZ += vec3(d.x*d.y*WA*cosF, -d.y*A*k*sinF, d.y*d.y*WA*cosF);

        // Jacobian determinant accumulation for wave folding whitecaps
        jxx += d.x * d.x * WA * cosF;
        jzz += d.y * d.y * WA * cosF;
        jxz += d.x * d.y * WA * cosF;
    }

    outNormal = normalize(cross(tangentZ,tangentX));
    outJacobian = jxx * jzz - jxz * jxz;
    return p;
}

void main() {
    // Snap grid center to camera position in discrete steps to prevent vertex swimming
    float gridStep = uOceanSize / float(max(uGridResolution - 1, 1));
    vec2 snappedOffset = floor(uCamPos.xz / gridStep) * gridStep;

    vec2 samplePos = aPos.xz + snappedOffset;
    vec3 normal = vec3(0.0, 1.0, 0.0);
    float jacobianDet = 1.0;
    vec3 p = gerstnerWithNormalAndJacobian(samplePos, time, normal, jacobianDet);

    vec3 worldP = (model * vec4(p, 1.0)).xyz;
    vec2 wakeUV=(worldP.xz-uFoamCenter)/uFoamSize+0.5;
    if (all(greaterThan(wakeUV,vec2(0.03))) && all(lessThan(wakeUV,vec2(0.97)))) {
        // A small broad crest at the two displacement fronts. Smooth over a grid cell
        // to avoid aliasing narrow foam ridges on the existing coarse ocean geometry.
        float footprint=uOceanSize/float(max(uGridResolution-1,1));
        vec2 stepUV=vec2(footprint/uFoamSize,0);
        float ridge=textureLod(uFoamTexture,wakeUV,0.0).g*0.5;
        ridge+=(textureLod(uFoamTexture,wakeUV+stepUV,0.0).g+textureLod(uFoamTexture,wakeUV-stepUV,0.0).g
               +textureLod(uFoamTexture,wakeUV+stepUV.yx,0.0).g+textureLod(uFoamTexture,wakeUV-stepUV.yx,0.0).g)*0.125;
        worldP.y+=ridge;
    }
    
    vPos = worldP;
    vNormal = normal;
    vWaveHeightVal = p.y;
    vJacobian = jacobianDet;
    vUV = samplePos;

    vClipPos = projection * view * vec4(worldP, 1.0);
    gl_Position = vClipPos;
}
