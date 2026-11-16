#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 vPos;
out vec3 vLocalPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    vLocalPos = aPos;
    vec3 worldP = (model * vec4(aPos, 1.0)).xyz;
    vPos = worldP;
    gl_Position = projection * view * vec4(worldP, 1.0);
}
