#version 330 core
in vec3 vPos;
in vec3 vLocalPos;
in vec3 vNormalVS;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 fogColor;
uniform vec3 uColorOverride;

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uMoonDir;
uniform vec3 uMoonColor;
uniform vec3 uHorizonColor;

uniform int uSpotlightOn;
uniform vec3 uSpotlightPos;
uniform vec3 uSpotlightDir;

vec3 ACESFilm(vec3 x) {
    float a = 2.51;
    float b = 0.03;
    float c = 2.43;
    float d = 0.59;
    float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main() {
    vec3 N = normalize(cross(dFdx(vPos), dFdy(vPos)));
    if (!gl_FrontFacing) N = -N;

    vec3 V = normalize(viewPos - vPos);

    bool isSunActive = (uSunDir.y > -0.10);
    vec3 L = isSunActive ? normalize(uSunDir) : normalize(uMoonDir);
    vec3 lightCol = isSunActive ? uSunColor : uMoonColor;
    float nightFactor = clamp(1.0 - (uSunDir.y + 0.1) * 3.0, 0.0, 1.0);

    vec3 H = normalize(L + V);
    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);

    vec3 albedo;
    vec3 emissive = vec3(0.0);

    if (length(uColorOverride) > 0.1) {
        // Floating cargo crate
        albedo = uColorOverride;
    } else {
        // --- BOAT PROCEDURAL PBR MATERIALS ---
        
        // 1. Navigation Lights Emissive
        // Port Light (Red) - Left Bow
        if (vLocalPos.x < -0.55 && vLocalPos.z > 0.65 && vLocalPos.z < 0.95 && vLocalPos.y > 0.22) {
            albedo = vec3(0.9, 0.05, 0.02);
            emissive = vec3(1.2, 0.08, 0.03) * 4.5;
        }
        // Starboard Light (Green) - Right Bow
        else if (vLocalPos.x > 0.55 && vLocalPos.z > 0.65 && vLocalPos.z < 0.95 && vLocalPos.y > 0.22) {
            albedo = vec3(0.05, 0.9, 0.1);
            emissive = vec3(0.08, 1.2, 0.15) * 4.5;
        }
        // Masthead Light (White) - Mast top
        else if (vLocalPos.y > 1.65) {
            albedo = vec3(0.95, 0.95, 0.9);
            emissive = vec3(1.0, 1.0, 0.9) * 4.0;
        }
        // Bow Spotlight Housing
        else if (vLocalPos.z > 1.30 && vLocalPos.y > 0.25) {
            albedo = vec3(0.2, 0.22, 0.25);
            if (uSpotlightOn == 1) {
                emissive = vec3(1.2, 1.1, 0.9) * 6.0;
            }
        }
        // 2. Outboard Motor & Propeller (Stern)
        else if (vLocalPos.z < -1.45) {
            albedo = vec3(0.12, 0.14, 0.16); // Charcoal metallic casing
        }
        // 3. Cockpit Dashboard Instruments (Illuminated Analog Dials & Compass)
        else if (vLocalPos.y > 0.40 && vLocalPos.y < 0.52 && vLocalPos.z > -0.36 && vLocalPos.z < -0.20 && abs(vLocalPos.x) < 0.32) {
            // Analog gauge dial faces
            vec3 dialFace = vec3(0.04, 0.05, 0.06);
            vec3 bezelRing = vec3(0.85, 0.88, 0.92); // Chrome bezel ring
            float isRing = smoothstep(0.02, 0.01, abs(vLocalPos.y - 0.46));

            // Backlit cyan/amber numbers and glowing orange needle
            vec3 backlighting = vec3(0.25, 0.88, 0.95) * 1.5;
            vec3 needleGlow   = vec3(1.2, 0.35, 0.08) * 2.5;

            albedo = mix(dialFace, bezelRing, isRing);
            emissive = mix(backlighting, needleGlow, step(0.0, vLocalPos.x * vLocalPos.y));
            if (nightFactor > 0.1) emissive *= 1.8;
        }
        // 4. Steering Wheel (Stainless Steel Spokes & Mahogany Rim)
        else if (vLocalPos.y > 0.38 && vLocalPos.y < 0.56 && vLocalPos.z > -0.44 && vLocalPos.z < -0.32 && abs(vLocalPos.x) < 0.18) {
            float distFromCenter = length(vec2(vLocalPos.x, vLocalPos.y - 0.46));
            if (distFromCenter > 0.10) {
                // Outer wheel rim: Wood / leather
                albedo = vec3(0.26, 0.15, 0.09);
            } else {
                // Center hub and 3 spokes: Stainless steel
                albedo = vec3(0.88, 0.90, 0.94);
            }
        }
        // 5. Throttle Handle (Starboard Control Quadrant)
        else if (vLocalPos.x > 0.22 && vLocalPos.x < 0.34 && vLocalPos.z > -0.42 && vLocalPos.z < -0.28 && vLocalPos.y > 0.38 && vLocalPos.y < 0.54) {
            if (vLocalPos.y > 0.48) {
                albedo = vec3(0.88, 0.15, 0.10); // Red ergonomic handle knob
            } else {
                albedo = vec3(0.75, 0.78, 0.82); // Chrome lever shaft
            }
        }
        // 6. Cockpit Windshield (Frame & Tinted Marine Glass)
        else if (vLocalPos.y > 0.48 && vLocalPos.y < 0.74 && vLocalPos.z > -0.24 && vLocalPos.z < 0.05 && abs(vLocalPos.x) < 0.54) {
            float frameEdge = max(smoothstep(0.46, 0.52, abs(vLocalPos.x)), smoothstep(0.68, 0.72, vLocalPos.y));
            frameEdge = max(frameEdge, smoothstep(0.03, 0.0, abs(vLocalPos.x))); // Center pillar
            if (frameEdge > 0.5) {
                albedo = vec3(0.12, 0.13, 0.14); // Matte black aluminum frame
            } else {
                albedo = vec3(0.06, 0.22, 0.30); // Tinted azure glass
            }
        }
        // 7. Captain & Passenger Bucket Seats
        else if (vLocalPos.z > -0.78 && vLocalPos.z < -0.45 && vLocalPos.y > 0.24 && vLocalPos.y < 0.65 && abs(vLocalPos.x) < 0.46) {
            albedo = vec3(0.42, 0.28, 0.18); // Saddle tan marine leather
        }
        // 8. Railings & Mast (Chrome metal)
        else if (vLocalPos.y > 0.32 && (abs(vLocalPos.x) > 0.58 || abs(vLocalPos.x) < 0.06)) {
            albedo = vec3(0.75, 0.78, 0.82); // Bright chrome
        }
        // 9. Deck (Procedural Teak Wood Planks)
        else if (vLocalPos.y > 0.16 && vLocalPos.y < 0.24 && vLocalPos.z > -0.6 && vLocalPos.z < 1.15) {
            float plankX = fract(vLocalPos.x * 12.0);
            float seam = smoothstep(0.06, 0.10, plankX) * smoothstep(0.94, 0.90, plankX);
            vec3 teakWood = vec3(0.50, 0.32, 0.18);
            vec3 darkSeam = vec3(0.10, 0.08, 0.06);
            albedo = mix(darkSeam, teakWood, seam);
        }
        // 10. Main Hull & Cabin Paint (Gloss Marine White with Crimson Trim)
        else {
            vec3 marineWhite = vec3(0.88, 0.89, 0.91);
            vec3 hullTrim    = vec3(0.72, 0.18, 0.12); // Crimson stripe
            float isTrim = smoothstep(0.04, 0.08, abs(vLocalPos.y - 0.10));
            albedo = mix(hullTrim, marineWhite, isTrim);
        }
    }

    float ao = pow(N.y * 0.5 + 0.5, 0.5);
    albedo *= (ao * 0.7 + 0.3);

    // Waterline wetness gloss
    float wetness = smoothstep(0.24, 0.05, vLocalPos.y);
    albedo = mix(albedo, albedo * 0.75, wetness * 0.5); // Wet darkening
    float specPower = mix(48.0, 160.0, wetness);
    float specIntensity = mix(1.4, 2.8, wetness);

    // Ambient + Diffuse + Specular
    vec3 ambient = mix(vec3(0.28), vec3(0.06), nightFactor) * albedo;
    vec3 diffuse = lightCol * 1.4 * NdotL * albedo;
    vec3 specular = lightCol * specIntensity * pow(NdotH, specPower);
    vec3 finalColor = ambient + diffuse + specular + emissive;

    // Atmospheric distance fog
    float dist = length(viewPos - vPos);
    float fogFactor = 1.0 - exp(-dist * 0.0035);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    finalColor = mix(finalColor, uHorizonColor, fogFactor);

    // ACES Tone Mapping & Gamma 2.2
    vec3 tonemapped = ACESFilm(finalColor);
    vec3 sRGB = pow(tonemapped, vec3(1.0 / 2.2));

    FragColor = vec4(sRGB, 1.0);
}
