#version 330 core
in vec3 vNormal;
in vec3 vPos;
in vec3 vViewPos;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform float time;

void main() {
    // --- Chuẩn bị vector cơ bản ---
    vec3 N = normalize(vNormal);
    vec3 V = normalize(viewPos - vPos);
    vec3 L = normalize(lightPos - vPos);
    vec3 H = normalize(L + V);

    // --- Ánh sáng cơ bản ---
    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 64.0);

    // --- Fresnel: phản chiếu mạnh ở góc xiên ---
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);

    // --- Foam: dựa theo độ dốc + nhiễu động theo thời gian ---
    float slope = length(vNormal.xz);

    // thêm nhiễu động nhỏ để foam di chuyển
    float waveNoise = sin(vPos.x * 0.1 + time * 0.8) * 0.5 +
                      cos(vPos.z * 0.1 + time * 0.6) * 0.5;

    // kết hợp độ dốc + nhiễu để foam "chuyển động"
    float foamFactor = smoothstep(0.35, 0.85, slope + 0.3 * waveNoise);
    foamFactor = pow(foamFactor, 1.2);

    // --- Màu cơ bản ---
    vec3 waterDeep = vec3(0.0, 0.25, 0.45);
    vec3 waterShallow = vec3(0.1, 0.5, 0.7);
    vec3 foamColor = vec3(0.92, 0.94, 0.98);

    // --- Trộn màu nước và foam ---
    vec3 baseColor = mix(waterDeep, waterShallow, fresnel);
    vec3 color = mix(baseColor, foamColor, foamFactor);

    // --- Ánh sáng tổng hợp ---
    vec3 ambient = 0.25 * color;
    vec3 diffuse = 0.75 * diff * color;
    vec3 specular = 0.4 * spec * vec3(1.0);

    vec3 finalColor = ambient + diffuse + specular;

    // Rim light nhẹ theo fresnel
    finalColor = mix(finalColor, vec3(1.0), 0.1 * fresnel);

    FragColor = vec4(finalColor, 1.0);
}
