#version 330 core
in vec3 vPos;
in vec3 vNormalVS;
out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform float time;
uniform int  uIsWater;

// các define SAFE_NO_RESPONSIVE / NO_DERIVATIVES sẽ được "chèn" từ C++

void main(){
#if NO_DERIVATIVES
    vec3 N = normalize(vNormalVS);
#else
    vec3 N = normalize(cross(dFdx(vPos), dFdy(vPos)));
    if(!gl_FrontFacing) N = -N;
#endif

    vec3 V = normalize(viewPos - vPos);
    vec3 L = normalize(lightPos - vPos);
    vec3 H = normalize(L + V);

    float diff    = max(dot(N, L), 0.0);
    float spec    = pow(max(dot(N, H), 0.0), 64.0);
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);

    vec3 color;
    if(uIsWater == 1){
        float slope = length(N.xz);
        float waveNoise = sin(vPos.x * 0.1 + time * 0.8) * 0.5 +
                          cos(vPos.z * 0.1 + time * 0.6) * 0.5;
        float foamFactor = smoothstep(0.35, 0.85, slope + 0.3 * waveNoise);
        foamFactor = pow(foamFactor, 1.2);

        vec3 waterDeep    = vec3(0.0, 0.25, 0.45);
        vec3 waterShallow = vec3(0.1, 0.5, 0.7);
        vec3 foamColor    = vec3(0.92, 0.94, 0.98);
        vec3 baseColor    = mix(waterDeep, waterShallow, fresnel);

        color = mix(baseColor, foamColor, foamFactor);
    } else {
        color = vec3(0.85, 0.35, 0.20); // thuyền
    }

    vec3 ambient  = 0.25 * color;
    vec3 diffuse  = 0.75 * diff * color;
    vec3 specular = 0.40 * spec * vec3(1.0);
    vec3 finalColor = ambient + diffuse + specular;

    finalColor = mix(finalColor, vec3(1.0), 0.1 * fresnel); // rim nhẹ
    FragColor = vec4(finalColor, 1.0);
}
