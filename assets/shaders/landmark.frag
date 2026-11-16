#version 330 core
in vec3 vPos;
in vec3 vLocalPos;

out vec4 FragColor;

uniform vec3 viewPos;
uniform vec3 fogColor;

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uMoonDir;
uniform vec3 uMoonColor;
uniform vec3 uHorizonColor;

uniform int uMaterialID;
uniform int uTopBeaconOn;
uniform float uBeamIntensity;
uniform int uBuoyLightOn;
uniform vec3 uBuoyLightColor;

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

    vec3 albedo = vec3(0.5);
    vec3 emissive = vec3(0.0);
    float roughness = 0.7;

    if (uMaterialID == 0) {
        // --- 0. Rocky Islet Base ---
        vec3 darkRock = vec3(0.18, 0.19, 0.20);
        vec3 wetRock  = vec3(0.08, 0.09, 0.10);
        vec3 mossRock = vec3(0.14, 0.22, 0.16);

        float wetness = smoothstep(1.5, -0.5, vPos.y);
        float mossiness = clamp(sin(vPos.x * 0.4) * cos(vPos.z * 0.4), 0.0, 1.0);

        albedo = mix(darkRock, wetRock, wetness);
        albedo = mix(albedo, mossRock, mossiness * 0.35);
        roughness = mix(0.9, 0.25, wetness);
    }
    else if (uMaterialID == 1) {
        // --- 1. Lighthouse Striped Tower ---
        float y = vLocalPos.y;
        vec3 redColor   = vec3(0.78, 0.12, 0.10);
        vec3 whiteColor = vec3(0.88, 0.88, 0.86);

        if (y < 8.2) {
            albedo = whiteColor;
        } else if (y < 12.8) {
            albedo = redColor;
        } else if (y < 17.2) {
            albedo = whiteColor;
        } else if (y < 21.4) {
            albedo = redColor;
        } else {
            // Gallery observation platform & iron railings
            albedo = vec3(0.18, 0.20, 0.22);
            roughness = 0.4;
        }
    }
    else if (uMaterialID == 2) {
        // --- 2. Lantern Room, Dome & Spire ---
        float y = vLocalPos.y;
        if (y < 25.3) {
            // Lantern room: Glass cage or central Fresnel lamp core
            float r = length(vLocalPos.xz);
            if (r < 0.9) {
                // Central rotating lamp core
                albedo = vec3(1.0, 0.95, 0.85);
                emissive = vec3(1.2, 1.05, 0.80) * (5.0 * clamp(uBeamIntensity, 0.2, 1.0));
            } else {
                // Outer bronze/iron glass frames
                albedo = vec3(0.15, 0.15, 0.16);
                roughness = 0.3;
            }
        } else if (y < 27.2) {
            // Copper dome roof (Verdigris patina)
            albedo = vec3(0.24, 0.50, 0.42);
            roughness = 0.6;
        } else {
            // Spire
            albedo = vec3(0.25, 0.26, 0.28);
            if (y > 28.6 && uTopBeaconOn == 1) {
                // Red aviation hazard beacon
                emissive = vec3(1.6, 0.08, 0.04) * 6.0;
            }
        }
    }
    else if (uMaterialID == 7) {
        // --- 7. Port Buoy (Red) ---
        float y = vLocalPos.y;
        if (y > 2.2) {
            // Top strobe light
            albedo = vec3(0.3);
            if (uBuoyLightOn == 1) {
                emissive = vec3(1.8, 0.08, 0.04) * 6.0;
            }
        } else if (y > 0.8) {
            // Metal cage superstructure
            albedo = vec3(0.25, 0.26, 0.28);
        } else {
            // Main red buoyancy collar
            albedo = vec3(0.82, 0.10, 0.08);
            roughness = 0.45;
        }
    }
    else if (uMaterialID == 8) {
        // --- 8. Starboard Buoy (Green) ---
        float y = vLocalPos.y;
        if (y > 2.3) {
            // Top strobe light
            albedo = vec3(0.3);
            if (uBuoyLightOn == 1) {
                emissive = vec3(0.08, 1.8, 0.15) * 6.0;
            }
        } else if (y > 0.6) {
            // Conical upper body
            albedo = vec3(0.08, 0.76, 0.20);
            roughness = 0.45;
        } else {
            // Main green collar
            albedo = vec3(0.06, 0.65, 0.18);
            roughness = 0.45;
        }
    }
    else if (uMaterialID == 9) {
        // --- 9. Danger / Cardinal Buoy (Yellow & Black) ---
        float y = vLocalPos.y;
        if (y > 2.4) {
            // Top strobe
            albedo = vec3(0.3);
            if (uBuoyLightOn == 1) {
                emissive = vec3(1.8, 1.4, 0.1) * 6.0;
            }
        } else {
            // Yellow and black stripes
            float stripe = step(0.0, sin(y * 4.0));
            vec3 yellow = vec3(0.92, 0.78, 0.08);
            vec3 black  = vec3(0.12, 0.12, 0.14);
            albedo = mix(black, yellow, stripe);
            roughness = 0.5;
        }
    }
    else if (uMaterialID == 11) {
        // --- 11. Seagull Feathers & Beak ---
        if (vLocalPos.z > 0.35 && vLocalPos.y > -0.05) {
            // Yellow beak
            albedo = vec3(0.95, 0.75, 0.08);
            roughness = 0.3;
        } else if (abs(vLocalPos.x) > 0.45) {
            // Dark slate wingtips
            albedo = vec3(0.24, 0.26, 0.28);
            roughness = 0.8;
        } else {
            // White body feathers
            albedo = vec3(0.92, 0.93, 0.95);
            roughness = 0.85;
        }
    }
    else if (uMaterialID == 12) {
        // --- 12. Harbor Pier Weathered Wood Planks & Pilings ---
        float plank = fract(vLocalPos.x * 0.8 + vLocalPos.z * 0.2);
        float seam = smoothstep(0.04, 0.08, plank) * smoothstep(0.96, 0.92, plank);
        vec3 woodDeck = vec3(0.42, 0.30, 0.19); // Weathered maritime oak
        vec3 darkSeam = vec3(0.14, 0.11, 0.08);
        albedo = mix(darkSeam, woodDeck, seam);
        roughness = 0.85;

        // Waterline dampness & green algae on pilings
        if (vPos.y < 0.6) {
            float damp = smoothstep(0.6, -1.0, vPos.y);
            albedo = mix(albedo, vec3(0.12, 0.18, 0.10), damp * 0.75); // Wet algae
            roughness = mix(roughness, 0.30, damp);
        }
    }
    else if (uMaterialID == 13) {
        // --- 13. Mooring Bollards & Rubber Dock Bumpers ---
        if (vLocalPos.y > 0.8) {
            // Cast iron mooring bollards (Dark charcoal metal)
            albedo = vec3(0.14, 0.15, 0.16);
            roughness = 0.40;
        } else {
            // Black rubber fender tires cushioning pier side
            albedo = vec3(0.08, 0.08, 0.09);
            roughness = 0.90;
        }
    }
    else if (uMaterialID == 14) {
        // --- 14. Pierhead Navigation Lanterns & Warm Boardwalk Lamps ---
        if (vLocalPos.x < -10.0) {
            // Pierhead Starboard Marker (Green harbor light)
            albedo = vec3(0.1, 0.8, 0.2);
            emissive = vec3(0.12, 1.8, 0.25) * 5.5;
        } else {
            // Warm Amber Boardwalk Post Lamps
            albedo = vec3(1.0, 0.9, 0.6);
            emissive = vec3(1.6, 1.15, 0.55) * 4.5;
        }
    }
    else if (uMaterialID == 15) {
        // --- 15. Mooring Ropes (Braided Nautical Hemp Cord) ---
        float twist = sin(vLocalPos.x * 35.0 + vLocalPos.y * 25.0) * 0.5 + 0.5;
        vec3 hempBase = vec3(0.62, 0.52, 0.36);
        albedo = mix(hempBase * 0.85, hempBase * 1.15, twist);
        roughness = 0.95;
    }
    else if (uMaterialID == 16) {
        // --- 16. Mainland Beach Sand & Shoreline ---
        vec3 sandDry = vec3(0.76, 0.68, 0.52);
        vec3 sandWet = vec3(0.48, 0.40, 0.28);
        float wet = smoothstep(2.0, 0.2, vPos.y);
        albedo = mix(sandDry, sandWet, wet);
        roughness = mix(0.85, 0.25, wet); // Wet glistening sand near surf
    }
    else if (uMaterialID == 17) {
        // --- 17. Mainland Rolling Grassy Hills & Coastal Meadows ---
        vec3 grassLush = vec3(0.18, 0.45, 0.14);
        vec3 grassMeadow = vec3(0.28, 0.52, 0.18);
        float patch = sin(vPos.x * 0.15) * cos(vPos.z * 0.15) * 0.5 + 0.5;
        albedo = mix(grassLush, grassMeadow, patch);
        roughness = 0.90;
    }
    else if (uMaterialID == 18) {
        // --- 18. Granite Sea Cliffs, Headlands & Distant Mountain Peaks ---
        vec3 graniteDark = vec3(0.24, 0.25, 0.27);
        vec3 graniteLight = vec3(0.38, 0.40, 0.42);
        float rockNoise = sin(vPos.y * 0.4 + vPos.x * 0.1) * 0.5 + 0.5;
        albedo = mix(graniteDark, graniteLight, rockNoise);
        // Waterline dampness & algae at surf base
        if (vPos.y < 2.0) {
            float damp = smoothstep(2.0, -1.0, vPos.y);
            albedo = mix(albedo, vec3(0.10, 0.16, 0.08), damp * 0.65);
            roughness = mix(roughness, 0.25, damp);
        } else {
            roughness = 0.85;
        }
    }
    else if (uMaterialID == 19) {
        // --- 19. Coastal Village Cottages, Watchtower & Glowing Windows ---
        bool isWindow = (fract(vPos.x * 0.8) > 0.65 && fract(vPos.y * 0.8) > 0.45 && vPos.y < 22.0 && vPos.y > 2.0);
        if (isWindow) {
            vec3 windowGlow = vec3(1.5, 1.1, 0.45);
            albedo = vec3(0.9, 0.8, 0.4);
            emissive = windowGlow * mix(1.2, 4.5, nightFactor);
            roughness = 0.15;
        } else if (vPos.y > 18.0 && abs(vPos.x - 112.0) < 1.0) {
            // Watchtower Beacon Light
            albedo = vec3(1.0, 0.9, 0.7);
            emissive = vec3(1.8, 1.3, 0.5) * 6.0;
        } else if (fract(vPos.y * 0.5) > 0.35 && vPos.y < 16.0) {
            // Terracotta orange roof tiles
            albedo = vec3(0.72, 0.28, 0.14);
            roughness = 0.75;
        } else {
            // Whitewashed stone walls / timber framing
            albedo = vec3(0.68, 0.70, 0.72);
            roughness = 0.80;
        }
    }
    else if (uMaterialID == 20) {
        // --- 20. Coastal Pine Trees (Evergreen Conifers) ---
        if (fract(vPos.y * 0.3) < 0.15) {
            // Pine Bark Trunk
            albedo = vec3(0.24, 0.16, 0.10);
            roughness = 0.95;
        } else {
            // Deep Evergreen Conifer Foliage
            vec3 pineGreen = vec3(0.08, 0.26, 0.10);
            vec3 tipHighlight = vec3(0.14, 0.35, 0.15);
            albedo = mix(pineGreen, tipHighlight, fract(vPos.y * 0.8));
            roughness = 0.85;
        }
    }

    // Direct Sun / Moon Lighting
    float NdotL = max(dot(N, L), 0.0);
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    float spec = pow(NdotH, 32.0) * (1.0 - roughness);

    vec3 diffuse = albedo * lightCol * NdotL;
    vec3 specular = lightCol * spec * 0.4;
    vec3 ambient = albedo * mix(uHorizonColor, vec3(0.02, 0.03, 0.06), nightFactor) * 0.35;

    vec3 finalColor = diffuse + specular + ambient + emissive;

    // Boat Spotlight illumination on Landmark
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
                float spotNdotL = max(dot(N, -spotDirNorm), 0.0);
                finalColor += albedo * vec3(1.0, 0.95, 0.85) * (spotIntensity * attenuation * spotNdotL * 6.0);
            }
        }
    }

    // Atmospheric Distance Fog
    float dist = length(viewPos - vPos);
    float fogFactor = 1.0 - exp(-dist * 0.0035);
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    finalColor = mix(finalColor, uHorizonColor, fogFactor);

    // ACES Film Tone Mapping
    vec3 tonemapped = ACESFilm(finalColor);
    vec3 sRGB = pow(tonemapped, vec3(1.0 / 2.2));

    FragColor = vec4(sRGB, 1.0);
}
