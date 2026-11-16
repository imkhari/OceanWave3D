#version 330 core
in vec3 vPos;
in vec3 vLocalPos;

out vec4 FragColor;

uniform float uIntensity;
uniform vec3 uBeamColor;

void main() {
    float zDist = vLocalPos.z;
    float distFade = exp(-zDist * 0.022);

    float maxR = mix(0.5, 24.0, clamp(zDist / 140.0, 0.0, 1.0));
    float currentR = length(vLocalPos.xy);
    float edgeFade = 1.0 - smoothstep(maxR * 0.3, maxR, currentR);

    float alpha = uIntensity * distFade * edgeFade * 0.32;
    FragColor = vec4(uBeamColor * 1.5, alpha);
}
