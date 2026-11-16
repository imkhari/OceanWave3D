#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 vPos;
out vec3 vLocalPos;
out vec3 vNormalVS;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    vec3 worldP = (model * vec4(aPos, 1.0)).xyz;
    vPos = worldP;
    vLocalPos = aPos;
    vNormalVS = vec3(0.0, 1.0, 0.0);
    gl_Position = projection * view * vec4(worldP, 1.0);
}
