#version 330 core
layout(location = 0) in vec3 aPos;       // Quad vertex (-0.5 to 0.5)
layout(location = 1) in vec3 aInstancePos; // Particle world position
layout(location = 2) in float aInstanceSize;
layout(location = 3) in float aInstanceAlpha;

out vec2 vUV;
out float vAlpha;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 camRight;
uniform vec3 camUp;

void main() {
    vUV = aPos.xy + vec2(0.5);
    vAlpha = aInstanceAlpha;

    // Camera-facing billboard quad
    vec3 worldPos = aInstancePos 
                  + camRight * aPos.x * aInstanceSize 
                  + camUp * aPos.y * aInstanceSize;

    gl_Position = projection * view * vec4(worldPos, 1.0);
}
