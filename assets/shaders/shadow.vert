#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 model;
uniform mat4 uLightMatrix;
void main() { gl_Position = uLightMatrix * model * vec4(aPos, 1.0); }
