#version 330 core

in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uSceneDepth;
uniform vec2 uResolution;
uniform float time;

// Dynamic Weather & Droplets (Feature 1)
uniform float uWetness;          // 0.0 to 1.0
uniform float uWiperAngle;       // radians (-1.1 to 1.1)
uniform int uWiperActive;        // 1 = sweeping
uniform vec2 uWiperPivot;        // typically (0.5, 0.05)
uniform float uWiperLength;      // length of wiper blade (e.g. 0.85)
uniform vec2 uGlassQuad[4];
uniform int uGlassVisible;
uniform int uIsCockpit;          // 1 = first-person cockpit view, 0 = 3rd-person / orbit

// Underwater Submersion (Feature 3)
uniform int uIsUnderwater;       // 1 = submerged
uniform float uSubmergedDepth;   // meters below wave surface
uniform vec3 uSunDir;

uniform float uExposure;
uniform int uAntialiasing;

vec3 displayTransform(vec3 linearColor) {
    vec3 x = max(linearColor * uExposure, vec3(0.0));
    vec3 mapped = clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
    return mix(mapped * 12.92, 1.055 * pow(mapped, vec3(1.0 / 2.4)) - 0.055, step(vec3(0.0031308), mapped));
}
vec3 sceneSample(vec2 uv) {
    vec3 center = texture(uSceneColor, uv).rgb;
    if (uAntialiasing == 0) return center;
    vec2 texel = 1.0 / max(uResolution, vec2(1.0));
    vec3 n = texture(uSceneColor, uv + vec2(0.0, texel.y)).rgb;
    vec3 s = texture(uSceneColor, uv - vec2(0.0, texel.y)).rgb;
    vec3 e = texture(uSceneColor, uv + vec2(texel.x, 0.0)).rgb;
    vec3 w = texture(uSceneColor, uv - vec2(texel.x, 0.0)).rgb;
    vec3 luma = vec3(0.2126, 0.7152, 0.0722);
    float c = dot(center, luma);
    float lo = min(c, min(min(dot(n,luma),dot(s,luma)),min(dot(e,luma),dot(w,luma))));
    float hi = max(c, max(max(dot(n,luma),dot(s,luma)),max(dot(e,luma),dot(w,luma))));
    float edge = smoothstep(0.06, 0.22, (hi - lo) / max(hi, 0.1));
    return mix(center, (n+s+e+w+center*4.0)/8.0, edge * 0.65);
}

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    float a = hash21(i);
    float b = hash21(i + vec2(1.0, 0.0));
    float c = hash21(i + vec2(0.0, 1.0));
    float d = hash21(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Distance from point P to line segment AB
float distToSegment(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h);
}

void main() {
    vec2 uv = vUV;
    float aspect = uResolution.x / max(uResolution.y, 1.0);

    // =========================================================================
    // FEATURE 3: UNDERWATER SUBMERSION / DIVING VIEW
    // =========================================================================
    if (uIsUnderwater == 1) {
        // 1. Chromatic wave optical distortion from water medium
        vec2 waveOffset = vec2(
            sin(uv.y * 36.0 + time * 3.2) * 0.0035,
            cos(uv.x * 30.0 + time * 2.8) * 0.0030
        );
        vec2 distortUV = clamp(uv + waveOffset, 0.001, 0.999);

        vec3 sceneCol = sceneSample(distortUV);

        // 2. Volumetric depth color extinction (Beer-Lambert ocean absorption)
        vec3 deepOceanTurquoise = vec3(0.003, 0.038, 0.085);
        float depthAbsorption = clamp(uSubmergedDepth * 0.15 + 0.38, 0.0, 0.88);
        vec3 waterGraded = mix(sceneCol, deepOceanTurquoise, depthAbsorption);

        // 3. Dancing Sunlight Caustics
        vec2 cUV1 = uv * vec2(aspect * 8.0, 8.0) + vec2(time * 0.35, time * 0.20);
        vec2 cUV2 = uv * vec2(aspect * 12.0, 12.0) + vec2(-time * 0.28, time * 0.42);
        float caust1 = noise(cUV1);
        float caust2 = noise(cUV2);
        float causticPattern = pow(caust1 * caust2 * 2.4, 2.8) * max(uSunDir.y, 0.05);
        vec3 causticTint = vec3(0.08, 0.38, 0.48) * (causticPattern * 1.8);

        // 4. Floating Micro Bubbles
        vec2 bubbleUV = uv * vec2(aspect * 35.0, 35.0) - vec2(0.0, time * 2.5);
        vec2 bubbleCell = floor(bubbleUV);
        vec2 bubbleFrac = fract(bubbleUV) - 0.5;
        float bHash = hash21(bubbleCell);
        float bubble = 0.0;
        if (bHash > 0.94) {
            float bDist = length(bubbleFrac);
            bubble = (1.0 - smoothstep(0.16, 0.22, bDist)) * 0.65;
        }

        // 5. Diving Goggle Vignette
        vec2 vigCoord = (uv - 0.5) * vec2(aspect, 1.0);
        float vig = smoothstep(0.95, 0.40, length(vigCoord));

        vec3 finalUnderwater = (waterGraded + causticTint + vec3(bubble)) * vig;
        FragColor = vec4(displayTransform(finalUnderwater), 1.0);
        return;
    }

    // =========================================================================
    // FEATURE 1: SCREEN-SPACE WATER DROPLETS & WINDSHIELD WIPER
    // =========================================================================
    // Effective wetness: in 3rd person, keep wetness very faint (lens condensation only)
    bool positive=false,negative=false;
    for(int i=0;i<4;++i) {
        vec2 edge=uGlassQuad[(i+1)%4]-uGlassQuad[i], offset=uv-uGlassQuad[i];
        float side=edge.x*offset.y-edge.y*offset.x;
        positive=positive || side>0.0; negative=negative || side<0.0;
    }
    bool onWindshield=uGlassVisible==1 && !(positive && negative);
    float effectiveWetness = onWindshield ? uWetness : 0.0;

    vec2 wiperPivot = uWiperPivot;
    vec2 bladeTip = wiperPivot + vec2(sin(uWiperAngle), cos(uWiperAngle)) * uWiperLength;

    // Check if current fragment lies on the physical wiper blade (ONLY in cockpit view when active!)
    float bladeThickness = 0.0032;
    float distBlade = distToSegment(uv, wiperPivot, bladeTip);

    // Wiper sweep sector clearance
    vec2 toPixel = uv - wiperPivot;
    float pixelAngle = atan(toPixel.x, toPixel.y);
    float pixelDist = length(toPixel);
    bool inWiperArc = (abs(pixelAngle) < 1.15 && pixelDist < uWiperLength && pixelDist > 0.08);

    // Clean wiped arc if wiper is active
    float localWetness = effectiveWetness;
    if (inWiperArc && uWiperActive == 1 && uIsCockpit == 1) {
        localWetness *= 0.06;
    }

    // Accumulate droplet normals and refraction
    vec2 dropNormal = vec2(0.0);
    float dropMask = 0.0;
    float dropGlint = 0.0;

    if (localWetness > 0.015) {
        // --- A. Natural Static Bead Droplets ---
        vec2 beadUV = uv * vec2(aspect * 22.0, 22.0);
        vec2 beadID = floor(beadUV);
        vec2 beadFrac = fract(beadUV);

        for (int y = -1; y <= 1; ++y) {
            for (int x = -1; x <= 1; ++x) {
                vec2 neighbor = vec2(float(x), float(y));
                vec2 cell = beadID + neighbor;
                float h = hash21(cell);

                if (h < (localWetness * 0.75)) {
                    vec2 center = neighbor + vec2(hash21(cell + 1.3), hash21(cell + 2.7)) * 0.5 + 0.25;
                    vec2 diff = beadFrac - center;
                    float d = length(diff);
                    float radius = 0.10 + hash21(cell + 5.9) * 0.18;

                    if (d < radius) {
                        float profile = sqrt(max(radius * radius - d * d, 0.0)) / radius;
                        vec2 norm2D = normalize(diff) * (1.0 - profile);
                        dropNormal += norm2D * 0.024;
                        dropMask = max(dropMask, smoothstep(radius, radius * 0.85, d));

                        vec2 lightDir2D = normalize(vec2(uSunDir.x, uSunDir.y + 0.3));
                        float glint = pow(max(dot(norm2D, lightDir2D), 0.0), 16.0);
                        dropGlint += glint * 0.65;
                    }
                }
            }
        }

        // --- B. Running Gravity Droplets (Only when heavy wetness in Cockpit) ---
        if (uIsCockpit == 1 && localWetness > 0.35) {
            float colX = floor(uv.x * aspect * 12.0);
            float colHash = hash21(vec2(colX, 83.1));
            if (colHash < (localWetness * 0.5)) {
                float streakSpeed = 0.45 + colHash * 0.35;
                float streakY = fract(uv.y * 1.8 + time * streakSpeed + colHash);
                float streakX = (fract(uv.x * aspect * 12.0) - 0.5) * 2.0;

                float streakDist = length(vec2(streakX, (streakY - 0.5) * 4.0));
                if (streakDist < 0.28) {
                    float sFade = (1.0 - smoothstep(0.08, 0.28, streakDist));
                    vec2 sNorm = normalize(vec2(streakX, streakY - 0.5)) * sFade;
                    dropNormal += sNorm * 0.020;
                    dropMask = max(dropMask, sFade);
                    dropGlint += pow(max(dot(sNorm, vec2(0.3, 0.7)), 0.0), 12.0) * 0.4;
                }
            }
        }
    }

    // Sample background scene with delicate droplet refraction
    vec2 finalUV = clamp(uv + dropNormal, 0.001, 0.999);
    vec3 sceneCol = sceneSample(finalUV);

    // Add droplet gloss & glint highlights
    vec3 outputCol = sceneCol + vec3(0.95, 0.96, 1.0) * (dropGlint * dropMask);

    // --- Render Windshield Wiper Blade (ONLY in Cockpit View when active!) ---
    if (onWindshield && uWiperActive == 1) {
        if (distBlade < bladeThickness && uv.y > (wiperPivot.y + 0.02) && pixelDist < uWiperLength) {
            float bladeEdge = smoothstep(bladeThickness, bladeThickness * 0.4, distBlade);
            vec3 rubberCol = vec3(0.08, 0.08, 0.09);
            vec3 metalHighlight = vec3(0.28, 0.30, 0.32) * bladeEdge;
            outputCol = rubberCol + metalHighlight;
        }
    }

    FragColor = vec4(displayTransform(outputCol), 1.0);
}
