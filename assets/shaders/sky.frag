#version 330 core
out vec4 FragColor;

uniform mat4 uInvProj;
uniform mat3 uInvViewRot;
uniform vec2 uResolution;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uMoonDir;
uniform vec3 uMoonColor;
uniform vec3 uHorizonColor;
uniform vec3 uZenithColor;
uniform float uTimeOfDay;
uniform float time;
uniform float uRainIntensity;
uniform vec3 uLightningColor;

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

float fbm(vec2 p) {
    float v = 0.0;
    float a = 0.5;
    mat2 m = mat2(1.6, 1.2, -1.2, 1.6);
    for (int i = 0; i < 5; i++) {
        v += a * noise(p);
        p = m * p;
        a *= 0.5;
    }
    return v;
}

// Procedural Starfield for Night
float stars(vec3 dir, float nightFactor) {
    if (nightFactor < 0.01) return 0.0;
    vec2 skyUV = dir.xz / (abs(dir.y) + 0.15) * 85.0;
    float starGrid = hash21(floor(skyUV));
    if (starGrid > 0.985) {
        float twinkle = sin(time * 3.0 + starGrid * 62.8) * 0.5 + 0.5;
        vec2 p=fract(skyUV)-0.5;
        float footprint=max(length(dFdx(skyUV)),length(dFdy(skyUV)));
        float point=1.0-smoothstep(0.03,0.10+min(footprint,0.15),length(p));
        return point * smoothstep(0.985,1.0,starGrid) * (0.65+0.35*twinkle) * nightFactor;
    }
    return 0.0;
}

// Linear HDR output; display transform is applied once in postprocess.


void main() {
    vec2 ndc = (gl_FragCoord.xy / uResolution) * 2.0 - 1.0;
    vec4 clip = vec4(ndc, 1.0, 1.0);
    vec4 view = uInvProj * clip;
    vec3 dirV = normalize(view.xyz / view.w);
    vec3 dirW = normalize(uInvViewRot * dirV);

    float v = clamp(dirW.y * 0.5 + 0.5, 0.0, 1.0);
    float nightFactor = clamp(1.0 - (uSunDir.y + 0.1) * 3.0, 0.0, 1.0);

    // 1. Rayleigh & Mie Atmosphere Gradient
    vec3 sky = mix(uHorizonColor, uZenithColor, smoothstep(0.05, 0.82, v));

    // Sunset / Golden Hour rich horizon coloring
    if (uSunDir.y > -0.08 && uSunDir.y < 0.35) {
        float sunsetGlow = (1.0 - smoothstep(-0.05, 0.35, uSunDir.y)) * pow(max(dot(dirW, normalize(uSunDir)), 0.0), 3.0);
        sky += vec3(0.95, 0.38, 0.12) * (sunsetGlow * (1.0 - dirW.y * 0.8) * 1.5);
    }

    // 2. Night Starfield
    float starBrightness = stars(dirW, nightFactor) * 4.5;
    sky += vec3(0.92, 0.96, 1.0) * starBrightness;

    // 3. Sun (Daytime / Sunset)
    if (uSunDir.y > -0.15) {
        vec3 sunDir = normalize(uSunDir);
        float dSun = dot(dirW, sunDir);
        float disk = smoothstep(cos(radians(1.2)), cos(radians(0.6)), dSun);
        float corona = smoothstep(cos(radians(12.0)), cos(radians(1.5)), dSun);
        float wideGlare = smoothstep(cos(radians(40.0)), cos(radians(10.0)), dSun);
        sky += uSunColor * (wideGlare * 0.25 + corona * 0.65 + disk * 3.0);
    }

    // 4. Moon (Nighttime)
    if (uMoonDir.y > 0.0) {
        vec3 moonDir = normalize(uMoonDir);
        float dMoon = dot(dirW, moonDir);
        float moonDisk = smoothstep(cos(radians(1.5)), cos(radians(1.1)), dMoon);
        float moonHalo = smoothstep(cos(radians(14.0)), cos(radians(1.5)), dMoon);
        vec3 moonBody = vec3(0.90, 0.95, 1.0) * (1.0 - noise(dirW.xz * 40.0) * 0.2);
        sky += moonBody * moonDisk * 2.8 + uMoonColor * moonHalo * 0.7;
    }

    // 5. Planar Horizontal Cloud Ceiling Projection
    if (dirW.y > 0.01) {
        vec2 cloudUV = (dirW.xz / max(dirW.y, 0.04)) * 0.22 + vec2(time * 0.008, time * 0.004);
        float clouds = fbm(cloudUV) * 0.62 + fbm(cloudUV * 2.4) * 0.38;

        float cloudCoverage = 0.52;
        float cloudAlpha = smoothstep(cloudCoverage - 0.10, cloudCoverage + 0.12, clouds);
        cloudAlpha *= smoothstep(0.01, 0.12, dirW.y); // Fade out right at horizon

        // Silver lining / Forward scattering on cloud rims facing the sun
        float sunForwardScatter = 0.0;
        if (uSunDir.y > -0.10) {
            float dSun = max(dot(dirW, normalize(uSunDir)), 0.0);
            sunForwardScatter = pow(dSun, 6.0) * smoothstep(cloudCoverage - 0.08, cloudCoverage + 0.02, clouds) * (1.0 - cloudAlpha);
        }

        vec3 cloudDayColor = mix(vec3(0.95, 0.96, 0.98), uSunColor * 0.6, 0.35);
        // Overcast / Storm cloud darkening
        cloudDayColor = mix(cloudDayColor, vec3(0.20, 0.22, 0.26), uRainIntensity * 0.85);

        vec3 cloudNightColor = vec3(0.04, 0.06, 0.10) + uMoonColor * 0.25;
        vec3 cloudBaseColor = mix(cloudDayColor, cloudNightColor, nightFactor);
        cloudBaseColor += uSunColor * (sunForwardScatter * 2.5);
        // Lightning illumination inside storm clouds
        cloudBaseColor += uLightningColor * (0.65 + clouds * 0.45);

        sky = mix(sky, cloudBaseColor, cloudAlpha * 0.82);
    }

    // 6. Horizon Atmospheric Haze (Guarantees zero seam with ocean horizon fog)
    float horizonHaze = pow(1.0 - clamp(dirW.y, 0.0, 1.0), 4.0);
    sky = mix(sky, uHorizonColor, horizonHaze * 0.85);

    // 7. Linear HDR output
    FragColor = vec4(max(sky, vec3(0.0)), 1.0);
}
