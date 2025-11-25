#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 vPos;
out vec3 vNormalVS;

uniform mat4 model, view, projection;
uniform float time;
uniform int  uIsWater;       // 1 = water, 0 = boat
uniform vec2 uWorldOffset;   // dùng khi responsive

// các define SAFE_NO_RESPONSIVE / NO_DERIVATIVES sẽ được "chèn" từ C++ trước khi compile

#define N_WAVES 8
uniform vec2  waveDir[N_WAVES];
uniform float waveAmp[N_WAVES];
uniform float waveWLen[N_WAVES];
uniform float waveOmega[N_WAVES]; // ω = sqrt(g*k)
uniform float steepness[N_WAVES];

vec3 gerstnerDisplace(vec2 pos, float t){
    vec3 p = vec3(pos.x, 0.0, pos.y);
    for(int i=0;i<N_WAVES;++i){
        vec2  d = normalize(waveDir[i]);
        float k = 6.2831853 / waveWLen[i];
        float f = k * dot(d, pos) - waveOmega[i] * t;
        float A = waveAmp[i];
        float S = steepness[i];
        p.x += (S*A*d.x) * sin(f);
        p.z += (S*A*d.y) * sin(f);
        p.y +=  A * cos(f);
    }
    return p;
}

void main(){
    vec3 worldP;
    vec3 worldN = vec3(0.0,1.0,0.0);

#if NO_DERIVATIVES
    const float EPS = 0.02;
#endif

    if(uIsWater == 1){
        vec2 samplePos = aPos.xz;
    #if !SAFE_NO_RESPONSIVE
        samplePos += uWorldOffset;   // sample theo toạ độ thế giới để "vô hạn"
    #endif

        vec3 p  = gerstnerDisplace(samplePos, time);
    #if NO_DERIVATIVES
        vec3 px = gerstnerDisplace(samplePos + vec2(EPS,0), time);
        vec3 pz = gerstnerDisplace(samplePos + vec2(0,EPS), time);
        worldN  = normalize(cross(pz - p, px - p));
    #endif

        worldP = p;
    #if !SAFE_NO_RESPONSIVE
        worldP.xz += uWorldOffset;   // đẩy cả lưới theo camera
    #endif
        worldP = (model * vec4(worldP,1.0)).xyz;
        worldN = mat3(transpose(inverse(model))) * worldN;
    } else {
        worldP = (model * vec4(aPos,1.0)).xyz;
    #if NO_DERIVATIVES
        worldN = mat3(transpose(inverse(model))) * worldN; // boat: normal "lên trên"
    #endif
    }

    vPos      = worldP;
    vNormalVS = normalize(worldN);
    gl_Position = projection * view * vec4(worldP, 1.0);
<<<<<<< HEAD
}
=======
}
>>>>>>> 1b0837f84cd16f2e7cc6ba74c499a8ef901d7956
