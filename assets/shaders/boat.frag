#version 330 core
in vec3 vPos;
in vec3 vLocalPos;
in vec3 vNormalVS;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 fogColor;
uniform float uRainIntensity;
uniform vec3 uColorOverride;
uniform int uPart;
uniform float uSpeedKnots;
uniform float uHeading;

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uMoonDir;
uniform vec3 uMoonColor;
uniform vec3 uHorizonColor;

uniform int uSpotlightOn;
uniform vec3 uSpotlightPos;
uniform vec3 uSpotlightDir;




uniform sampler2D uShadowMap;
uniform mat4 uLightMatrix;
uniform int uShadows;
float visibility(vec3 N, vec3 L) {
    if (uShadows == 0) return 1.0;
    vec4 clip = uLightMatrix * vec4(vPos, 1.0);
    vec3 p = clip.xyz / clip.w * 0.5 + 0.5;
    if (p.z >= 1.0 || p.z <= 0.0 || any(lessThan(p.xy,vec2(0))) || any(greaterThan(p.xy,vec2(1)))) return 1.0;
    vec2 texel = 1.0 / vec2(textureSize(uShadowMap, 0));
    float bias = max(0.0006, 0.0018 * sqrt(max(0.0,1.0-dot(N,L)*dot(N,L))) / max(abs(dot(N,L)),0.15));
    float visible = 0.0;
    for (int y=-1;y<=1;++y) for(int x=-1;x<=1;++x)
        visible += p.z - bias <= texture(uShadowMap,p.xy+vec2(x,y)*texel).r ? 1.0 : 0.0;
    return visible / 9.0;
}

void main() {
    vec3 N = normalize(cross(dFdx(vPos), dFdy(vPos)));
    if (!gl_FrontFacing) N = -N;

    vec3 V = normalize(viewPos - vPos);

    bool isSunActive = (uSunDir.y > -0.10);
    vec3 L = isSunActive ? normalize(uSunDir) : normalize(uMoonDir);
    vec3 lightCol = isSunActive ? uSunColor : uMoonColor;
    float nightFactor = clamp(1.0 - (uSunDir.y + 0.1) * 3.0, 0.0, 1.0);

    if (uPart == 2) {
        float fresnel = pow(1.0 - abs(dot(N,V)),5.0);
        vec3 glass = mix(vec3(0.025,0.11,0.13),uHorizonColor,fresnel);
        FragColor = vec4(glass,0.07+fresnel*0.24); return;
    }
    if (uPart == 3) {
        float cx = vLocalPos.x < -0.085 ? -0.17 : (vLocalPos.x > 0.085 ? 0.17 : 0.0);
        vec2 p = vec2((vLocalPos.x-cx)/0.065,(vLocalPos.y-0.54)/0.093);
        float r=length(p), angle=atan(p.y,p.x);
        float needleAngle = cx < -0.08 ? mix(3.9,-0.75,clamp(abs(uSpeedKnots)/35.0,0.0,1.0)) :
                            (cx > 0.08 ? mix(3.9,-0.75,clamp(abs(uSpeedKnots)/30.0,0.0,1.0)) : 1.5708-uHeading);
        vec2 axis=vec2(cos(needleAngle),sin(needleAngle));
        float needle=(1.0-smoothstep(0.025,0.055,abs(p.x*axis.y-p.y*axis.x)))*step(0.0,dot(p,axis))*(1.0-smoothstep(0.65,0.78,r));
        float ticks=step(0.91,cos(angle*24.0))*smoothstep(0.65,0.73,r)*(1.0-smoothstep(0.84,0.9,r));
        vec3 dial=mix(vec3(0.012,0.02,0.024),vec3(0.22,0.28,0.30),smoothstep(0.84,0.9,r));
        dial=mix(dial,vec3(0.40,0.72,0.65)*(0.5+nightFactor*0.3),ticks);
        dial=mix(dial,vec3(0.90,0.30,0.08)*(0.7+nightFactor*0.3),needle);
        FragColor=vec4(dial,1); return;
    }
    vec3 H = normalize(L + V);
    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);

    vec3 albedo;
    vec3 emissive = vec3(0.0);

    if (uPart == 1) {
        float r=length(vLocalPos.xy*vec2(3.0,2.1));
        float grain=sin(vLocalPos.x*500.0+sin(vLocalPos.y*95.0))*0.025;
        albedo=r>0.32 ? vec3(0.20+grain,0.075+grain*0.5,0.028) : vec3(0.32,0.37,0.4);
    } else if (uPart == 4) {
        albedo=vLocalPos.y>0.10 ? vec3(0.45,0.055,0.025) : vec3(0.32,0.37,0.4);
    } else if (length(uColorOverride) > 0.1) {
        // Floating cargo crate
        albedo = uColorOverride;
    } else {
        // --- BOAT PROCEDURAL PBR MATERIALS ---
        
        // 1. Navigation Lights Emissive
        // Port Light (Red) - Left Bow
        if (vLocalPos.x < -0.55 && vLocalPos.z > 0.65 && vLocalPos.z < 0.95 && vLocalPos.y > 0.22) {
            albedo = vec3(0.9, 0.05, 0.02);
            emissive = vec3(1.2, 0.08, 0.03) * mix(0.12, 1.5, nightFactor);
        }
        // Starboard Light (Green) - Right Bow
        else if (vLocalPos.x > 0.55 && vLocalPos.z > 0.65 && vLocalPos.z < 0.95 && vLocalPos.y > 0.22) {
            albedo = vec3(0.05, 0.9, 0.1);
            emissive = vec3(0.08, 1.2, 0.15) * mix(0.12, 1.5, nightFactor);
        }
        // Masthead Light (White) - Mast top
        else if (vLocalPos.y > 1.65) {
            albedo = vec3(0.95, 0.95, 0.9);
            emissive = vec3(1.0, 1.0, 0.9) * mix(0.12, 1.5, nightFactor);
        }
        // Bow Spotlight Housing
        else if (vLocalPos.z > 1.30 && vLocalPos.y > 0.25) {
            albedo = vec3(0.2, 0.22, 0.25);
            if (uSpotlightOn == 1) {
                emissive = vec3(1.2, 1.1, 0.9) * mix(0.12, 1.5, nightFactor);
            }
        }
        // 2. Outboard Motor & Propeller (Stern)
        else if (vLocalPos.z < -1.45) {
            albedo = vec3(0.12, 0.14, 0.16); // Charcoal metallic casing
        }
        else if (vLocalPos.y > 0.32 && vLocalPos.y < 0.65 && vLocalPos.z > -0.41 && vLocalPos.z < -0.13 && abs(vLocalPos.x) < 0.39) {
            float grain = sin(vLocalPos.x * 160.0 + sin(vLocalPos.y * 50.0)) * 0.025;
            albedo = vec3(0.10 + grain, 0.055 + grain*0.5, 0.028);
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
        else if ((vLocalPos.y > 0.135 && vLocalPos.y < 0.165 && N.y > 0.6) || (vLocalPos.y > 0.16 && vLocalPos.y < 0.24 && vLocalPos.z > -0.6 && vLocalPos.z < 1.15)) {
            float plankX = fract(vLocalPos.x * 12.0);
            float seam = smoothstep(0.06, 0.10, plankX) * (1.0 - smoothstep(0.90, 0.94, plankX));
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
    float wetness = (1.0 - smoothstep(0.05, 0.24, vLocalPos.y));
    albedo = mix(albedo, albedo * 0.75, wetness * 0.5); // Wet darkening
    float specPower = mix(48.0, 160.0, wetness);
    float specIntensity = mix(0.16, 0.30, wetness);

    // Ambient + Diffuse + Specular
    vec3 ambient = mix(vec3(0.16, 0.20, 0.25), vec3(0.035, 0.045, 0.07), nightFactor) * albedo * (0.55 + 0.45 * max(N.y, 0.0));
    vec3 diffuse = lightCol * 0.72 * NdotL * albedo;
    vec3 specular = lightCol * specIntensity * pow(NdotH, specPower);
    vec3 finalColor = ambient + (diffuse + specular) * visibility(N,L) + emissive;

    // Atmospheric distance fog
    float dist = length(viewPos - vPos);
    float fogFactor = 1.0 - exp(-max(dist - 25.0, 0.0) * (0.0008 + uRainIntensity * 0.003));
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    finalColor = mix(finalColor, uHorizonColor, fogFactor);

    // Linear HDR output
    FragColor = vec4(max(finalColor, vec3(0.0)), 1.0);
}
