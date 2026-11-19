#version 330 core
in vec2 vUV;
in float vAlpha;

out vec4 FragColor;

uniform vec3 uSunColor;

void main() {
    // Soft radial droplet falloff
    vec2 centerOffset = vUV - vec2(0.5);
    float distSq = dot(centerOffset, centerOffset);
    if (distSq > 0.25) {
        discard;
    }

    float radialAlpha = (1.0 - smoothstep(0.0, 0.25, distSq));
    float alpha = radialAlpha * vAlpha;

    // Splash particle color: white water foam with soft sun rim
    vec3 baseColor = vec3(0.95, 0.97, 1.0);
    vec3 tintedColor = mix(baseColor, uSunColor, 0.25);

    FragColor = vec4(tintedColor, alpha * 0.48);
}
