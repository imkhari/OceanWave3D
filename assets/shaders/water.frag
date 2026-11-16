#version 330 core

in vec3 vPos;
in vec3 vNormal;
in float vWaveHeightVal;
in float vJacobian;
in vec2 vUV;
in vec4 vClipPos;

out vec4 FragColor;

// --- Uniforms ---
uniform vec3 viewPos;
uniform float time;
uniform mat4 view;
uniform mat4 projection;
uniform vec2 uResolution;

// Scene G-Buffer for SSR and Refraction
uniform sampler2D uSceneColor;
uniform sampler2D uSceneDepth;

// Persistent Foam Buffer
uniform sampler2D uFoamTexture;
uniform vec2 uFoamCenter;
uniform float uFoamSize;

// Day-Night & Atmosphere
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uMoonDir;
uniform vec3 uMoonColor;
uniform vec3 uHorizonColor;
uniform vec3 uZenithColor;
uniform float uTimeOfDay;

// Dynamic Weather & Environment
uniform float uRainIntensity;
uniform vec3 uLightningColor;
uniform int uIsUnderwater;

// Dynamic Boat Interaction
uniform vec3 uBoatPos;
uniform float uBoatSpeed;
uniform float uBoatHeading;

// Boat Bow Spotlight
uniform int uSpotlightOn;
uniform vec3 uSpotlightPos;
uniform vec3 uSpotlightDir;

// Lighthouse Sweeping Beam
uniform int uLighthouseActive;
uniform vec3 uLighthousePos;
uniform vec3 uLighthouseBeamDir;
uniform float uLighthouseIntensity;

// --- Constants ---
const float PI = 3.14159265359;
const float WATER_F0 = 0.02; // IOR ~ 1.333 for ocean water
const float NEAR_PLANE = 0.1;
const float FAR_PLANE = 800.0;

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

float cellularFoam(vec2 p) {
    vec2 ip = floor(p);
    vec2 fp = fract(p);
    float minDist = 1.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            vec2 neighbor = vec2(float(x), float(y));
            vec2 point = vec2(hash21(ip + neighbor), hash21(ip + neighbor + 57.1));
            vec2 diff = neighbor + point - fp;
            minDist = min(minDist, length(diff));
        }
    }
    return minDist;
}

// Realistic organic rain micro-capillary ripples (smooth noise, no repeating tiles)
vec3 calculateRainCapillaries(vec2 pos, float t, float intensity) {
    if (intensity < 0.02) return vec3(0.0);
    vec2 p1 = pos * 6.5 + vec2(t * 6.0, -t * 4.5);
    vec2 p2 = pos * 14.0 + vec2(-t * 8.5, t * 7.0);
    float n1 = noise(p1);
    float n2 = noise(p2);
    vec2 dN = vec2(n1 - 0.5, n2 - 0.5) * (0.042 * intensity);
    return vec3(dN.x, 0.0, dN.y);
}

float linearizeDepth(float depth) {
    float z = depth * 2.0 - 1.0;
    return (2.0 * NEAR_PLANE * FAR_PLANE) / (FAR_PLANE + NEAR_PLANE - z * (FAR_PLANE - NEAR_PLANE));
}

// PBR Microfacet Functions
float D_GGX(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float denom = (NdotH * NdotH * (a2 - 1.0) + 1.0);
    return a2 / (PI * denom * denom);
}

float G_SchlickSmith(float NdotV, float NdotL, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    float gV = NdotV / (NdotV * (1.0 - k) + k);
    float gL = NdotL / (NdotL * (1.0 - k) + k);
    return gV * gL;
}

float Fresnel_Schlick(float cosTheta, float F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 ACESFilm(vec3 x) {
    float a = 2.51;
    float b = 0.03;
    float c = 2.43;
    float d = 0.59;
    float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// Advanced Screen-Space Reflections (SSR) with Binary Search Refinement
vec3 computeSSR(vec3 vPosVS, vec3 rayDirVS, vec3 fallbackSky, float distToBoat, float distToLighthouse) {
    // Only trace rays for reflective structures within relevant range
    bool nearReflector = (distToBoat < 42.0) || (distToLighthouse < 75.0);
    if (!nearReflector || rayDirVS.z >= 0.0) {
        return fallbackSky;
    }

    const int MAX_STEPS = 12;
    const float STEP_SIZE = 1.4;
    float currentStep = 0.7;

    for (int i = 0; i < MAX_STEPS; ++i) {
        vec3 marchPosVS = vPosVS + rayDirVS * currentStep;
        if (marchPosVS.z > -NEAR_PLANE) break;

        vec4 marchClip = projection * vec4(marchPosVS, 1.0);
        vec3 marchNDC = marchClip.xyz / marchClip.w;
        vec2 sampleUV = marchNDC.xy * 0.5 + 0.5;

        if (sampleUV.x < 0.0 || sampleUV.x > 1.0 || sampleUV.y < 0.0 || sampleUV.y > 1.0) {
            break;
        }

        float sampledDepth = texture(uSceneDepth, sampleUV).r;
        float sampledDepthLinear = linearizeDepth(sampledDepth);
        float rayDepthLinear = -marchPosVS.z;

        float depthDiff = rayDepthLinear - sampledDepthLinear;
        if (depthDiff > 0.03 && depthDiff < 2.5) {
            // Binary search refinement for sub-pixel reflection accuracy
            float minT = currentStep - STEP_SIZE;
            float maxT = currentStep;
            for (int b = 0; b < 4; ++b) {
                float midT = (minT + maxT) * 0.5;
                vec3 midPosVS = vPosVS + rayDirVS * midT;
                vec4 midClip = projection * vec4(midPosVS, 1.0);
                vec2 midUV = (midClip.xy / midClip.w) * 0.5 + 0.5;
                float midSampleDepth = linearizeDepth(texture(uSceneDepth, midUV).r);
                if (-midPosVS.z > midSampleDepth) {
                    maxT = midT;
                    sampleUV = midUV;
                } else {
                    minT = midT;
                }
            }

            float edgeFade = smoothstep(0.0, 0.12, sampleUV.x) * smoothstep(1.0, 0.88, sampleUV.x)
                           * smoothstep(0.0, 0.12, sampleUV.y) * smoothstep(1.0, 0.88, sampleUV.y);
            float distFade = max(smoothstep(42.0, 5.0, distToBoat), smoothstep(75.0, 15.0, distToLighthouse));
            vec3 hitColor = texture(uSceneColor, sampleUV).rgb;
            return mix(fallbackSky, hitColor, edgeFade * distFade * 0.88);
        }

        currentStep += STEP_SIZE;
    }

    return fallbackSky;
}

void main() {
    vec2 screenUV = (vClipPos.xy / vClipPos.w) * 0.5 + 0.5;

    // -------------------------------------------------------------------------
    // 1. Analytical Gerstner Normal + Dual-Octave Micro-Capillary Ripples
    // -------------------------------------------------------------------------
    vec3 baseN = normalize(vNormal);

    // High-frequency capillary ripples moving in wind directions
    vec2 uvCap1 = vPos.xz * 1.6 + vec2(time * 0.12, time * 0.09);
    vec2 uvCap2 = vPos.xz * 4.2 + vec2(-time * 0.16, time * 0.14);

    float dRx = (noise(uvCap1 + vec2(0.04, 0.0)) - noise(uvCap1 - vec2(0.04, 0.0))) * 0.08
              + (noise(uvCap2 + vec2(0.03, 0.0)) - noise(uvCap2 - vec2(0.03, 0.0))) * 0.035;
    float dRz = (noise(uvCap1 + vec2(0.0, 0.04)) - noise(uvCap1 - vec2(0.0, 0.04))) * 0.08
              + (noise(uvCap2 + vec2(0.0, 0.03)) - noise(uvCap2 - vec2(0.0, 0.03))) * 0.035;

    vec3 rainRipple = calculateRainCapillaries(vPos.xz, time, uRainIntensity);
    vec3 N = normalize(baseN + vec3(-dRx, 0.0, -dRz) + rainRipple);
    if (!gl_FrontFacing || uIsUnderwater == 1) N = -N;

    vec3 V = normalize(viewPos - vPos);
    float NdotV = max(dot(N, V), 0.001);

    // -------------------------------------------------------------------------
    // 2. Day-Night Celestial Illumination
    // -------------------------------------------------------------------------
    bool isSunActive = (uSunDir.y > -0.10);
    vec3 mainLightDir = isSunActive ? normalize(uSunDir) : normalize(uMoonDir);
    vec3 mainLightCol = (isSunActive ? uSunColor : uMoonColor) + uLightningColor;
    float nightFactor = clamp(1.0 - (uSunDir.y + 0.1) * 3.0, 0.0, 1.0);

    vec3 L = mainLightDir;
    vec3 H = normalize(L + V);
    float NdotL = max(dot(N, L), 0.001);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    // -------------------------------------------------------------------------
    // 3. Physical Water Absorption (Beer-Lambert) & Chromatic Depth Scattering
    // -------------------------------------------------------------------------
    float sceneDepthLinear = linearizeDepth(texture(uSceneDepth, screenUV).r);
    float waterDepthLinear = linearizeDepth(gl_FragCoord.z);
    float waterThickness = max(0.0, sceneDepthLinear - waterDepthLinear);

    // Wave refraction distortion
    vec2 refrOffset = N.xz * clamp(waterThickness * 0.022, 0.004, 0.030);
    vec2 refrUV = clamp(screenUV + refrOffset, 0.001, 0.999);
    vec3 sceneRefractedColor = texture(uSceneColor, refrUV).rgb;

    // Physical absorption: Red is absorbed fastest, Blue/Green penetrate deeply
    vec3 absorptionCoeff = vec3(0.38, 0.10, 0.025);
    vec3 waterTransmittance = exp(-absorptionCoeff * waterThickness * 1.6);

    // Photorealistic Deep Ocean Optical Palette (Natural Deep Atlantic / Pacific)
    vec3 dayDeepClear = vec3(0.002, 0.014, 0.038); // Deep oceanic abyss
    vec3 dayDeepRain  = vec3(0.008, 0.018, 0.030); // Stormy slate navy
    vec3 dayDeep = mix(dayDeepClear, dayDeepRain, uRainIntensity);

    vec3 dayCrestClear = vec3(0.012, 0.085, 0.120); // Natural coastal emerald teal on wave crests
    vec3 dayCrestRain  = vec3(0.015, 0.045, 0.065);
    vec3 dayCrest = mix(dayCrestClear, dayCrestRain, uRainIntensity);

    vec3 nightDeep    = vec3(0.0005, 0.002, 0.008);
    vec3 nightCrest   = vec3(0.003,  0.010, 0.022);

    vec3 deepWaterCol = mix(dayDeep, nightDeep, nightFactor);
    vec3 crestWaterCol = mix(dayCrest, nightCrest, nightFactor);

    // Wave crest transmission (translucent wave tops)
    float crestFactor = smoothstep(-0.2, 1.2, vWaveHeightVal);
    vec3 opticalWaterBody = mix(deepWaterCol, crestWaterCol, crestFactor * 0.45);

    // Scene refraction: only blend if there's actually geometry under water (boat hull, cargo, buoys)
    float hasSubmergedGeometry = smoothstep(0.05, 0.8, waterThickness) * (sceneDepthLinear < 400.0 ? 1.0 : 0.0);
    vec3 finalRefraction = mix(opticalWaterBody, sceneRefractedColor, waterTransmittance * hasSubmergedGeometry);

    // Subsurface Scattering (SSS): subtle emerald glow ONLY when looking directly toward the sun through wave peaks
    if (isSunActive) {
        float sssBacklight = pow(max(dot(V, -L), 0.0), 5.0);
        float sssCrest = smoothstep(0.3, 1.5, vWaveHeightVal) * (1.0 - N.y);
        vec3 sssColor = vec3(0.02, 0.14, 0.12) * (sssBacklight * sssCrest * 1.2 * (1.0 - nightFactor) * (1.0 - uRainIntensity * 0.8));
        finalRefraction += sssColor;
    }

    // -------------------------------------------------------------------------
    // 4. Fresnel & Screen-Space Reflections (SSR)
    // -------------------------------------------------------------------------
    float fresnel = Fresnel_Schlick(NdotV, WATER_F0);
    vec3 R = reflect(-V, N);
    float skyMix = smoothstep(0.0, 0.65, R.y);
    vec3 skyReflection = mix(uHorizonColor, uZenithColor, skyMix);

    // Screen-Space Reflections (SSR) with micro-ripple wobble
    vec3 vPosVS = (view * vec4(vPos, 1.0)).xyz;
    vec3 rayDirVS = normalize(mat3(view) * R);
    float distToBoat = length(vPos.xz - uBoatPos.xz);
    float distToLighthouse = length(vPos.xz - vec2(85.0, -105.0));

    vec3 actualReflection = computeSSR(vPosVS, rayDirVS, skyReflection, distToBoat, distToLighthouse);

    // Total Internal Reflection & Snell's Window when underwater
    if (uIsUnderwater == 1) {
        float sinTheta = length(cross(V, N));
        if (sinTheta > 0.75) {
            // Total Internal Reflection (TIR) mirrors the dark oceanic depths below
            actualReflection = mix(vec3(0.003, 0.028, 0.065), actualReflection, 0.15);
            fresnel = 1.0;
        } else {
            fresnel = clamp(Fresnel_Schlick(NdotV, 0.02) * 1.5, 0.05, 0.95);
        }
    }

    // -------------------------------------------------------------------------
    // 5. Dual-Lobe GGX Specular & Dancing Sun Glitter (Sparkles)
    // -------------------------------------------------------------------------
    // Lobe 1: Broad specular sheen
    float roughness1 = 0.085;
    float D1 = D_GGX(NdotH, roughness1);
    float G1 = G_SchlickSmith(NdotV, NdotL, roughness1);
    float F_spec = Fresnel_Schlick(VdotH, WATER_F0);
    vec3 broadSpecular = (D1 * G1 * F_spec / max(4.0 * NdotV * NdotL, 0.001)) * mainLightCol * 3.2;

    // Lobe 2: High-intensity dancing sun sparkles (glitter path)
    float glitterSparkle = 0.0;
    if (isSunActive && NdotL > 0.01) {
        float sparkDist = pow(max(dot(N, H), 0.0), 320.0);
        float sparkMod = sin(dot(vPos.xz, vec2(95.0, 65.0)) + time * 14.0) * 0.5 + 0.5;
        glitterSparkle = sparkDist * sparkMod * 16.0;
    }
    vec3 specularGlitter = mainLightCol * glitterSparkle * F_spec;

    vec3 finalColor = mix(finalRefraction, actualReflection, fresnel) + broadSpecular + specularGlitter;

    // -------------------------------------------------------------------------
    // 6. Boat Bow Spotlight Illumination
    // -------------------------------------------------------------------------
    if (uSpotlightOn == 1) {
        vec3 spotToFrag = vPos - uSpotlightPos;
        float spotDist = length(spotToFrag);
        if (spotDist < 75.0) {
            vec3 spotDirNorm = normalize(spotToFrag);
            float angleCos = dot(spotDirNorm, normalize(uSpotlightDir));
            float spotCutoff = cos(radians(28.0));
            float spotFeather = cos(radians(20.0));

            if (angleCos > spotCutoff) {
                float spotIntensity = smoothstep(spotCutoff, spotFeather, angleCos);
                float attenuation = 1.0 / (1.0 + 0.06 * spotDist + 0.002 * spotDist * spotDist);

                vec3 spotH = normalize(-spotDirNorm + V);
                float spotSpec = pow(max(dot(N, spotH), 0.0), 64.0);
                vec3 spotColor = vec3(1.0, 0.95, 0.85) * (spotIntensity * attenuation * 5.0);
                finalColor += spotColor * (max(dot(N, -spotDirNorm), 0.0) * 0.6 + spotSpec * 1.5);
            }
        }
    }

    // -------------------------------------------------------------------------
    // 7. Lighthouse Sweeping Searchlight Beam Illumination
    // -------------------------------------------------------------------------
    if (uLighthouseActive == 1 && uLighthouseIntensity > 0.05) {
        vec3 lhToFrag = vPos - uLighthousePos;
        float lhDist = length(lhToFrag);
        if (lhDist < 260.0) {
            vec3 lhDirNorm = normalize(lhToFrag);
            float angleCos = dot(lhDirNorm, normalize(uLighthouseBeamDir));
            float beamCutoff = cos(radians(18.0));
            float beamFeather = cos(radians(11.0));

            if (angleCos > beamCutoff) {
                float beamIntensity = smoothstep(beamCutoff, beamFeather, angleCos);
                float attenuation = 1.0 / (1.0 + 0.02 * lhDist + 0.0003 * lhDist * lhDist);

                vec3 lhH = normalize(-lhDirNorm + V);
                float lhSpec = pow(max(dot(N, lhH), 0.0), 96.0);
                vec3 beamColor = vec3(1.0, 0.96, 0.82) * (beamIntensity * attenuation * uLighthouseIntensity * 8.5);
                finalColor += beamColor * (max(dot(N, -lhDirNorm), 0.0) * 0.45 + lhSpec * 2.2);
            }
        }
    }

    // -------------------------------------------------------------------------
    // 8. Multi-Source Physical Foam Simulation & 3D Shading
    // -------------------------------------------------------------------------
    // 8a. Wave crest folding whitecaps (Jacobian determinant)
    float jacobianFoam = smoothstep(0.32, -0.18, vJacobian);

    // 8b. Persistent wake & bow spray from FoamSimulator FBO
    vec2 foamUV = (vPos.xz - uFoamCenter) / uFoamSize + 0.5;
    float persistentFoam = 0.0;
    if (foamUV.x >= 0.0 && foamUV.x <= 1.0 && foamUV.y >= 0.0 && foamUV.y <= 1.0) {
        persistentFoam = texture(uFoamTexture, foamUV).r;
    }

    // 8c. Island shoreline surf (crashing waves on rocky islet at 85, -105)
    float isletShoreDist = length(vPos.xz - vec2(85.0, -105.0));
    float shoreSurf = 0.0;
    if (isletShoreDist < 18.0 && isletShoreDist > 5.0) {
        float shoreProximity = smoothstep(18.0, 9.0, isletShoreDist);
        float shoreWavePulse = smoothstep(0.2, 0.8, sin(time * 1.5 - isletShoreDist * 1.2) * 0.5 + 0.5);
        shoreSurf = shoreProximity * shoreWavePulse * 1.4;
    }

    // 8d. Outboard motor propeller froth cone
    vec2 boatLocal = vPos.xz - uBoatPos.xz;
    float cosH = cos(-uBoatHeading);
    float sinH = sin(-uBoatHeading);
    vec2 rotatedBoatLocal = vec2(boatLocal.x * cosH - boatLocal.y * sinH, boatLocal.x * sinH + boatLocal.y * cosH);
    float motorWake = 0.0;
    if (rotatedBoatLocal.y < -1.2 && rotatedBoatLocal.y > -18.0 && abs(rotatedBoatLocal.x) < (-rotatedBoatLocal.y * 0.28 + 0.8)) {
        float motorIntensity = clamp(uBoatSpeed / 5.0, 0.0, 1.5);
        motorWake = smoothstep(-18.0, -2.0, rotatedBoatLocal.y) * motorIntensity * 1.3;
    }

    // Cellular bubble micro-structure
    vec2 microUV = vPos.xz * 0.45 + vec2(time * 0.04, time * 0.02);
    float bubblePattern = (1.0 - cellularFoam(microUV * 2.8));

    float totalFoam = clamp(jacobianFoam * 0.85 + persistentFoam * 1.25 + shoreSurf + motorWake, 0.0, 1.0) * bubblePattern;

    // 3D Foam lighting (self-shadowing ambient occlusion)
    float foamAO = clamp(N.y * 0.6 + 0.4, 0.3, 1.0);
    vec3 foamDayColor = vec3(0.97, 0.96, 0.95) * (foamAO * 0.75 + 0.25 * mainLightCol);
    vec3 foamNightColor = vec3(0.12, 0.16, 0.22);
    vec3 foamColor = mix(foamDayColor, foamNightColor, nightFactor * 0.8);

    finalColor = mix(finalColor, foamColor, totalFoam);

    // -------------------------------------------------------------------------
    // 9. Horizon Atmospheric Sea Mist & Distance Blending
    // -------------------------------------------------------------------------
    float dist = length(viewPos - vPos);
    float fogDensity = 0.0009 + uRainIntensity * 0.0022;
    float fogFactor = clamp(1.0 - exp(-dist * fogDensity), 0.0, 1.0);

    float sunHaze = isSunActive ? pow(max(dot(V, -L), 0.0), 4.0) * 0.45 : 0.0;
    vec3 ambientFog = mix(uHorizonColor, uSunColor * 0.6, sunHaze);
    finalColor = mix(finalColor, ambientFog, fogFactor);

    // -------------------------------------------------------------------------
    // 10. ACES Tone Mapping & Gamma 2.2 Correction
    // -------------------------------------------------------------------------
    vec3 tonemapped = ACESFilm(finalColor);
    vec3 sRGB = pow(tonemapped, vec3(1.0 / 2.2));

    FragColor = vec4(sRGB, 1.0);
}
