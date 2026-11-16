#version 330 core
in float vStreakFactor;
out vec4 FragColor;

uniform float uIntensity;
uniform vec3 uRainColor;

void main() {
    float alpha = clamp(uIntensity * 0.18, 0.0, 0.28);
    FragColor = vec4(uRainColor, alpha);
}
