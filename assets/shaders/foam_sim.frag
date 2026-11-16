#version 330 core
in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uPreviousFoam;
uniform vec2 uUvShift;
uniform float dt;

// Boat injection
uniform vec2 uDomainCenter;
uniform float uDomainSize;
uniform vec2 uBoatPos;
uniform vec2 uBowPos;
uniform vec2 uSternPos;
uniform float uBoatSpeed;

void main() {
    // 1. Shift UV based on camera/domain tracking
    vec2 prevUV = vUV + uUvShift;

    float prevFoam = 0.0;
    if (prevUV.x >= 0.0 && prevUV.x <= 1.0 && prevUV.y >= 0.0 && prevUV.y <= 1.0) {
        prevFoam = texture(uPreviousFoam, prevUV).r;
    }

    // 2. Slow natural dissipation (lingers 7 - 10 seconds)
    float decayRate = 1.0 / 8.5;
    float currentFoam = prevFoam * exp(-dt * decayRate);

    // 3. World position of this texel
    vec2 worldXZ = uDomainCenter + (vUV - 0.5) * uDomainSize;

    // 4. Inject foam from Propeller Wash (Stern)
    float speedRatio = clamp(abs(uBoatSpeed) / 4.0, 0.0, 1.5);
    if (speedRatio > 0.1) {
        float distStern = length(worldXZ - uSternPos);
        float sternInjection = smoothstep(2.8, 0.2, distStern) * (speedRatio * 0.95);
        currentFoam = max(currentFoam, sternInjection);

        // Wake V-wings close behind boat
        vec2 dStern = worldXZ - uSternPos;
        float dBehind = -dStern.y; // approximate
    }

    // 5. Inject foam from Bow Wave breaking
    if (speedRatio > 0.5) {
        float distBow = length(worldXZ - uBowPos);
        float bowInjection = smoothstep(2.0, 0.2, distBow) * (speedRatio * 0.7);
        currentFoam = max(currentFoam, bowInjection);
    }

    currentFoam = clamp(currentFoam, 0.0, 1.0);
    FragColor = vec4(currentFoam, currentFoam * 0.5, 0.0, 1.0);
}
