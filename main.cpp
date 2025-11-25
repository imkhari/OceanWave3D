// === Build (Windows)
// g++ -std=c++17 -O2 .\main.cpp .\src\glad.c -I.\include -I"D:\SETUP\msys64\ucrt64\include" -L"D:\SETUP\msys64\ucrt64\lib" -o main.exe -lglfw3 -lopengl32 -lgdi32 -limm32 -luser32 -lshell32 -lversion -lwinmm -g
// .\main.exe

#define SAFE_NO_RESPONSIVE 1 // 1: lưới rộng cố định; 0: ocean bám camera (infinite look)
#define NO_DERIVATIVES 0     // 0: dFdx/dFdy tính normal nước ở FS; 1: normal từ VS

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cmath>

// Trạng thái game
enum GameState
{
    MENU,
    PLAYING,
    WON,
    LOST
};

// Cấu trúc thùng hàng
struct Coin
{
    glm::vec3 pos; // Vị trí (x, z)
    bool active;   // true = chưa ăn, false = đã ăn
    float angle;   // Góc xoay để trang trí
};

// ===================== WATER/BOAT Shaders (body) =====================
static const char *VS_BODY = R"(
layout(location = 0) in vec3 aPos;
out vec3 vPos;
out vec3 vNormalVS;

uniform mat4 model, view, projection;
uniform float time;
uniform int  uIsWater;
uniform vec2 uWorldOffset;

#define N_WAVES 8
uniform vec2  waveDir[N_WAVES];
uniform float waveAmp[N_WAVES];
uniform float waveWLen[N_WAVES];
uniform float waveOmega[N_WAVES];
uniform float steepness[N_WAVES];

vec3 gerstner(vec2 pos, float t){
    vec3 p = vec3(pos.x, 0.0, pos.y);
    for(int i=0;i<N_WAVES;++i){
        vec2  d = normalize(waveDir[i]);
        float k = 6.2831853 / waveWLen[i];
        float f = k*dot(d,pos) - waveOmega[i]*t;
        float A = waveAmp[i], S = steepness[i];
        p.x += (S*A*d.x)*sin(f);
        p.z += (S*A*d.y)*sin(f);
        p.y +=  A*cos(f);
    }
    return p;
}

void main(){
    vec3 worldP;
    vec3 worldN = vec3(0,1,0);
#if NO_DERIVATIVES
    const float EPS = 0.02;
#endif

    if(uIsWater == 1){
        vec2 samplePos = aPos.xz;
    #if !SAFE_NO_RESPONSIVE
        samplePos += uWorldOffset;
    #endif
        vec3 p = gerstner(samplePos, time);
    #if NO_DERIVATIVES
        vec3 px = gerstner(samplePos+vec2(EPS,0), time);
        vec3 pz = gerstner(samplePos+vec2(0,EPS), time);
        worldN = normalize(cross(pz-p, px-p));
    #endif
        worldP = p;
    #if !SAFE_NO_RESPONSIVE
        worldP.xz += uWorldOffset;
    #endif
        worldP = (model*vec4(worldP,1)).xyz;
        worldN = mat3(transpose(inverse(model)))*worldN;
    }else{
        worldP = (model*vec4(aPos,1)).xyz;
    #if NO_DERIVATIVES
        worldN = mat3(transpose(inverse(model)))*worldN;
    #endif
    }

    vPos = worldP;
    vNormalVS = normalize(worldN);
    gl_Position = projection*view*vec4(worldP,1.0);
}
)";

static const char *FS_BODY = R"(
in vec3 vPos;
in vec3 vNormalVS;
out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform float time;
uniform int  uIsWater;
uniform vec3 fogColor;
uniform vec3 uColorOverride;

void main(){
    // 1. CHUẨN BỊ VECTOR
#if NO_DERIVATIVES
    vec3 N = normalize(vNormalVS);
#else
    vec3 N = normalize(cross(dFdx(vPos), dFdy(vPos)));
    if(!gl_FrontFacing) N = -N;
#endif
    vec3 V = normalize(viewPos - vPos);
    vec3 L = normalize(lightPos - vPos);
    vec3 H = normalize(L + V);

    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);
    float NdotV = max(dot(N, V), 0.0);

    vec3 finalColor;

    if(uIsWater == 1){
        // ================= NƯỚC (WATER) =================

        // Fresnel (Phản chiếu mạnh khi nhìn nghiêng)
        float fresnel = 0.02 + (1.0 - 0.02) * pow(1.0 - NdotV, 5.0);

        // Màu bản thân nước (Water Body)
        vec3 deepColor    = vec3(0.00, 0.05, 0.10); 
        vec3 shallowColor = vec3(0.00, 0.25, 0.35);
        vec3 waterBody    = mix(deepColor, shallowColor, 0.5);

        // Màu phản chiếu bầu trời (Sky Reflection)
        vec3 R = reflect(-V, N); 
        vec3 horizonCol = vec3(0.95, 0.48, 0.22); // Cam
        vec3 zenithCol  = vec3(0.05, 0.10, 0.25); // Tím than
        float skyMix    = smoothstep(0.0, 0.5, R.y); 
        vec3 skyReflect = mix(horizonCol, zenithCol, skyMix);

        // Kết hợp Nước + Trời
        finalColor = mix(waterBody, skyReflect, fresnel);

        // Đốm sáng mặt trời (Sun Specular)
        // Giảm độ gắt xuống 128.0 để bớt răng cưa
        float specWater = pow(NdotH, 128.0); 
        specWater = clamp(specWater, 0.0, 1.0); // Chống cháy sáng
        finalColor += vec3(1.0, 0.9, 0.7) * specWater * 1.5;

        // Bọt trắng (Foam)
        float slope = 1.0 - N.y;
        float foamMask = smoothstep(0.15, 0.4, slope - 0.02);
        finalColor = mix(finalColor, vec3(0.95), foamMask);

    } else {
        vec3 albedo;
        // Nếu có màu đè (thùng hàng) thì dùng màu đó
        if(length(uColorOverride) > 0.1){ 
            albedo = uColorOverride; 
        } else {
            // Logic cũ: Tô màu thuyền
            vec3 paintColor = vec3(0.70, 0.25, 0.15); 
            vec3 woodColor  = vec3(0.40, 0.25, 0.18);
            float isDeck = smoothstep(0.15, 0.20, vPos.y);
            albedo = mix(paintColor, woodColor, isDeck);
        }
        
        // Tính ánh sáng (giữ nguyên logic cũ)
        float ao = pow(normalize(vNormalVS).y * 0.5 + 0.5, 0.5);
        albedo *= (ao * 0.7 + 0.3);
        vec3 ambient  = vec3(0.3) * albedo;
        vec3 diffuse  = vec3(0.9) * NdotL * albedo;
        vec3 specular = vec3(1.0, 0.9, 0.8) * pow(NdotH, 64.0);
        finalColor = ambient + diffuse + specular;
    }

    // XỬ LÝ SƯƠNG MÙ (FOG)
    float dist = length(viewPos - vPos);
    float fogFactor = 1.0 - exp(-dist * 0.0025); 
    fogFactor = clamp(fogFactor, 0.0, 1.0);
    FragColor = vec4(mix(finalColor, fogColor, fogFactor), 1.0);
}
)";

// ===================== SKY Shaders (Sunset + Clouds) =====================
static const char *SKY_VS = R"(
void main(){
    vec2 pos = vec2( (gl_VertexID==0)? -1.0 : (gl_VertexID==1)?  3.0 : -1.0,
                     (gl_VertexID==0)? -1.0 : (gl_VertexID==1)? -1.0 :  3.0 );
    gl_Position = vec4(pos, 0.0, 1.0);
}
)";

static const char *SKY_FS = R"(
out vec4 FragColor;

uniform mat4 uInvProj;
uniform mat3 uInvViewRot;
uniform vec2 uResolution;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform float time;

float hash21(vec2 p){
    p = fract(p*vec2(123.34, 456.21));
    p += dot(p, p+45.32);
    return fract(p.x*p.y);
}
float noise(vec2 p){
    vec2 i=floor(p), f=fract(p);
    float a=hash21(i);
    float b=hash21(i+vec2(1,0));
    float c=hash21(i+vec2(0,1));
    float d=hash21(i+vec2(1,1));
    vec2 u=f*f*(3.0-2.0*f);
    return mix(mix(a,b,u.x), mix(c,d,u.x), u.y);
}
float fbm(vec2 p){
    float v=0.0, a=0.5;
    mat2 m=mat2(1.6,1.2,-1.2,1.6);
    for(int i=0;i<5;i++){
        v += a*noise(p);
        p = m*p; a *= 0.5;
    }
    return v;
}

void main(){
    // ray dir (world)
    vec2 ndc = (gl_FragCoord.xy / uResolution)*2.0 - 1.0;
    vec4 clip = vec4(ndc, 1.0, 1.0);
    vec4 view = uInvProj * clip;
    vec3 dirV = normalize(view.xyz / view.w);
    vec3 dirW = normalize(uInvViewRot * dirV);

    float v = clamp(dirW.y*0.5 + 0.5, 0.0, 1.0);

    // --- Sunset gradient ---
    vec3 horizon = vec3(0.95, 0.48, 0.22);
    vec3 mid     = vec3(0.35, 0.16, 0.33);
    vec3 zenith  = vec3(0.03, 0.05, 0.12);
    vec3 sky     = mix(mix(horizon, mid, smoothstep(0.05, 0.45, v)),
                       zenith, smoothstep(0.55, 0.95, v));

    // --- Sun (low altitude) ---
    vec3 sunDir = normalize(uSunDir);
    float d = dot(dirW, sunDir);
    float disk = smoothstep(cos(radians(1.0)),  cos(radians(0.45)), d);
    float halo = smoothstep(cos(radians(6.0)),  cos(radians(1.1)),  d);
    vec3 sunCol = uSunColor;
    sky += sunCol * (halo*0.45 + disk*1.10);

    // --- Clouds (fbm)
    // ít mây ở zenith, nhiều gần horizon
    float horizonBoost = smoothstep(0.0, 0.6, 1.0 - v);
    vec2 wind = vec2(0.03, 0.00);
    vec2 P = vec2(atan(dirW.z,dirW.x)/6.2831853 + 0.5, v);
    P = vec2(P.x*4.0, P.y*1.6) + wind*time*0.6;
    float clouds = fbm(P)*0.6 + fbm(P*2.7)*0.4 + horizonBoost*0.15;
    float coverage = mix(0.58, 0.52, horizonBoost);
    float mask = smoothstep(coverage-0.07, coverage+0.07, clouds);

    float sunLit = pow(max(d, 0.0), 8.0)*1.5;
    vec3 cloudLit = mix(vec3(0.85,0.82,0.80), sunCol, clamp(sunLit, 0.0, 1.0));
    sky = mix(sky, cloudLit, mask*0.85);

    // haze chân trời
    float haze = pow(1.0 - v, 3.0);
    sky += vec3(0.25,0.23,0.26) * haze * 0.18;

    FragColor = vec4(sky, 1.0);
}
)";

// ===================== Shader utils =====================
static GLuint compile(GLenum type, const std::string &fullSrc)
{
    GLuint s = glCreateShader(type);
    const char *p = fullSrc.c_str();
    GLint n = (GLint)fullSrc.size();
    glShaderSource(s, 1, &p, &n);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetShaderInfoLog(s, 4096, NULL, log);
        std::cerr << "Shader compile error:\n"
                  << log << "\n";
        std::exit(-1);
    }
    return s;
}
static GLuint link(GLuint vs, GLuint fs)
{
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char log[4096];
        glGetProgramInfoLog(p, 4096, NULL, log);
        std::cerr << "Link error:\n"
                  << log << "\n";
        std::exit(-1);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

// ===================== Ocean helpers (CPU) =====================
static const int N_WAVES = 8;
static const float PI2 = 6.28318530718f;
static const float G = 9.8f;
static const float EPS_N = 0.02f;

static glm::vec3 gerstnerCPU(const glm::vec2 &pos, const glm::vec2 d[], const float A[],
                             const float L[], const float W[], const float S[], float t)
{
    glm::vec3 p(pos.x, 0.0f, pos.y);
    for (int i = 0; i < N_WAVES; i++)
    {
        glm::vec2 dir = glm::normalize(d[i]);
        float k = PI2 / L[i];
        float f = k * glm::dot(dir, pos) - W[i] * t;
        p.x += (S[i] * A[i] * dir.x) * std::sin(f);
        p.z += (S[i] * A[i] * dir.y) * std::sin(f);
        p.y += A[i] * std::cos(f);
    }
    return p;
}
static glm::vec3 normalCPU(const glm::vec2 &pos, const glm::vec2 d[], const float A[],
                           const float L[], const float W[], const float S[], float t)
{
    glm::vec3 p = gerstnerCPU(pos, d, A, L, W, S, t);
    glm::vec3 pDx = gerstnerCPU(pos + glm::vec2(EPS_N, 0), d, A, L, W, S, t);
    glm::vec3 pDz = gerstnerCPU(pos + glm::vec2(0, EPS_N), d, A, L, W, S, t);
    return glm::normalize(glm::cross(pDz - p, pDx - p));
}

// ===================== Boat mesh (open-deck, rim, bow sealed) =====================
// Hàm phụ trợ: Thêm hình hộp (Cube) vào lưới
static void addCube(std::vector<glm::vec3> &V, std::vector<unsigned int> &I,
                    glm::vec3 center, glm::vec3 size)
{
    glm::vec3 half = size * 0.5f;
    // 8 đỉnh của hình hộp
    glm::vec3 v[8] = {
        center + glm::vec3(-half.x, -half.y, half.z),  // 0: BLF (Bottom-Left-Front)
        center + glm::vec3(half.x, -half.y, half.z),   // 1: BRF
        center + glm::vec3(half.x, half.y, half.z),    // 2: TRF
        center + glm::vec3(-half.x, half.y, half.z),   // 3: TLF
        center + glm::vec3(-half.x, -half.y, -half.z), // 4: BLB
        center + glm::vec3(half.x, -half.y, -half.z),  // 5: BRB
        center + glm::vec3(half.x, half.y, -half.z),   // 6: TRB
        center + glm::vec3(-half.x, half.y, -half.z)   // 7: TLB
    };
    unsigned int start = (unsigned int)V.size();
    for (int i = 0; i < 8; i++)
        V.push_back(v[i]);

    // Các mặt (Index)
    unsigned int idx[] = {
        0, 1, 2, 0, 2, 3, // Front
        1, 5, 6, 1, 6, 2, // Right
        5, 4, 7, 5, 7, 6, // Back
        4, 0, 3, 4, 3, 7, // Left
        3, 2, 6, 3, 6, 7, // Top
        4, 5, 1, 4, 1, 0  // Bottom
    };
    for (int i = 0; i < 36; i++)
        I.push_back(start + idx[i]);
}

// Hàm tạo thuyền mới: Có Cabin và Ghế
static void buildBoatMeshComplex(
    int seg, float L, float halfW0, float H,
    std::vector<glm::vec3> &V, std::vector<unsigned int> &I)
{
    V.clear();
    I.clear();

    // 1. VẼ VỎ TÀU (HULL) - Giữ nguyên logic cũ cho phần thân
    auto halfWidth = [&](float t)
    {
        float w = halfW0 * pow(1.0f - t, 0.25f) + 0.02f * (1.0f - t);
        return std::max(0.02f, w);
    };
    auto yDeck = [&](float)
    { return 0.22f; };
    auto yBottom = [&](float t)
    { return -H * (0.8f - 0.4f * t); };
    const float rimInset = 0.15f; // Viền dày hơn chút
    const float rimDrop = 0.05f;
    float z0 = -L * 0.5f, z1 = L * 0.5f;

    for (int i = 0; i <= seg; i++)
    {
        float t = i / (float)seg;
        float z = z0 + (z1 - z0) * t;
        float hw = halfWidth(t);
        float yT = yDeck(t);
        float yB = yBottom(t);
        float hwInner = std::max(0.01f, hw * (1.0f - rimInset));
        float yR = yT - rimDrop;

        V.push_back({-hw, yT, z});         // 0 LT_o
        V.push_back({hw, yT, z});          // 1 RT_o
        V.push_back({-hw * 0.72f, yB, z}); // 2 LB_o
        V.push_back({hw * 0.72f, yB, z});  // 3 RB_o
        V.push_back({-hwInner, yR, z});    // 4 LT_i
        V.push_back({hwInner, yR, z});     // 5 RT_i
    }
    auto vid = [&](int i, int k)
    { return i * 6 + k; };
    for (int i = 0; i < seg; i++)
    {
        int LT0 = vid(i, 0), RT0 = vid(i, 1), LB0 = vid(i, 2), RB0 = vid(i, 3), LTi0 = vid(i, 4), RTi0 = vid(i, 5);
        int LT1 = vid(i + 1, 0), RT1 = vid(i + 1, 1), LB1 = vid(i + 1, 2), RB1 = vid(i + 1, 3), LTi1 = vid(i + 1, 4), RTi1 = vid(i + 1, 5);
        I.insert(I.end(), {(unsigned)LB0, (unsigned)LT0, (unsigned)LB1, (unsigned)LT0, (unsigned)LT1, (unsigned)LB1});
        I.insert(I.end(), {(unsigned)RT0, (unsigned)RB0, (unsigned)RT1, (unsigned)RB0, (unsigned)RB1, (unsigned)RT1});
        I.insert(I.end(), {(unsigned)LB0, (unsigned)RB1, (unsigned)RB0, (unsigned)LB0, (unsigned)LB1, (unsigned)RB1});
        I.insert(I.end(), {(unsigned)LT0, (unsigned)LT1, (unsigned)LTi1, (unsigned)LT0, (unsigned)LTi1, (unsigned)LTi0});
        I.insert(I.end(), {(unsigned)RT0, (unsigned)RTi1, (unsigned)RT1, (unsigned)RT0, (unsigned)RTi0, (unsigned)RTi1});
    }
    // Đóng nắp đuôi (Transom)
    {
        unsigned LT = vid(0, 0), RT = vid(0, 1), LB = vid(0, 2), RB = vid(0, 3), LTi = vid(0, 4), RTi = vid(0, 5);
        I.insert(I.end(), {LT, RT, RB, LT, RB, LB});
        I.insert(I.end(), {LT, LTi, RTi, LT, RTi, RT});
    }
    // Đóng nắp mũi (Bow)
    {
        unsigned LT = vid(seg, 0), RT = vid(seg, 1), LB = vid(seg, 2), RB = vid(seg, 3);
        unsigned LTi = vid(seg, 4), RTi = vid(seg, 5);
        float zTip = (-L * 0.5f + L * 0.5f) + 0.05f;
        float yB = yBottom(1.0f) - 0.02f;
        float yR = yDeck(1.0f) - 0.05f;
        unsigned botTip = (unsigned)V.size();
        V.push_back({0.0f, yB, zTip});
        unsigned rimTip = (unsigned)V.size();
        V.push_back({0.0f, yR, zTip - 0.01f});
        I.insert(I.end(), {LB, botTip, RB, LT, LB, botTip, RB, RT, botTip, LT, botTip, RT, LTi, rimTip, RTi, LT, LTi, rimTip, rimTip, RTi, RT});
    }

    // === PHẦN NÂNG CẤP: THÊM CẤU TRÚC ===

    // 2. Cabin lái (Ở phía đuôi tàu - Negative Z)
    // Tọa độ y=0.4 (cao hơn sàn 0.22)
    glm::vec3 cabinPos(0.0f, 0.45f, -0.8f);
    glm::vec3 cabinSize(1.0f, 0.5f, 0.8f);
    addCube(V, I, cabinPos, cabinSize);

    // 3. Ghế ngồi / Hộp chứa đồ (Ở giữa tàu)
    glm::vec3 seatPos(0.0f, 0.25f, 0.2f);
    glm::vec3 seatSize(0.6f, 0.15f, 0.4f);
    addCube(V, I, seatPos, seatSize);

    // 4. Kính chắn gió (Mặt phẳng nghiêng trước cabin)
    // Đơn giản là thêm 1 cái hộp mỏng nghiêng hoặc 1 hộp nhỏ nữa
    glm::vec3 windshieldPos(0.0f, 0.55f, -0.45f); // Trước cabin chút
    glm::vec3 windshieldSize(0.9f, 0.25f, 0.05f);
    addCube(V, I, windshieldPos, windshieldSize);
}

// ===================== MAIN =====================
int main()
{
    if (!glfwInit())
    {
        std::cerr << "GLFW init fail\n";
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow *win = glfwCreateWindow(1280, 720, "Ocean + Boat + Sunset Sky", nullptr, nullptr);
    if (!win)
    {
        std::cerr << "GLFW create fail\n";
        return -1;
    }
    glfwMakeContextCurrent(win);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "GLAD init fail\n";
        return -1;
    }
    std::cout << "GL_VERSION=" << glGetString(GL_VERSION) << "\n";

#if !SAFE_NO_RESPONSIVE
    glfwSetFramebufferSizeCallback(win, [](GLFWwindow *, int w, int h)
                                   { glViewport(0, 0, w ? h : 1, h ? h : 1); });
#endif

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    // programs
    auto makeSrc = [](const char *body, bool safe, bool deriv)
    {
        std::string s = "#version 330 core\n";
        s += std::string("#define SAFE_NO_RESPONSIVE ") + (safe ? "1\n" : "0\n");
        s += std::string("#define NO_DERIVATIVES ") + (deriv ? "1\n" : "0\n");
        s += body;
        return s;
    };
    std::string VS = makeSrc(VS_BODY, true, NO_DERIVATIVES);
    std::string FS = makeSrc(FS_BODY, true, NO_DERIVATIVES);
    GLuint vs = compile(GL_VERTEX_SHADER, VS);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FS);
    GLuint prog = link(vs, fs);

    std::string skyVS = std::string("#version 330 core\n") + SKY_VS;
    std::string skyFS = std::string("#version 330 core\n") + SKY_FS;
    GLuint sv = compile(GL_VERTEX_SHADER, skyVS);
    GLuint sf = compile(GL_FRAGMENT_SHADER, skyFS);
    GLuint skyProg = link(sv, sf);

    // ocean grid (dynamic resolution via ImGui)
    float guiWaveHeight = 1.0f;
    float guiTimeSpeed = 1.0f;
    float guiSunPos = 0.0f;
    bool guiShowWireframe = false;

    int guiOceanN = 200;             // độ phân giải lưới nước (N x N)
    const float OCEAN_SIZE = 150.0f; // kích thước lưới

    GLuint waterVAO = 0, waterVBO = 0, waterEBO = 0;
    GLsizei waterIndexCount = 0;

    auto rebuildOceanMesh = [&](int N)
    {
        if (N < 2) N = 2;

        std::vector<glm::vec3> verts;
        verts.reserve((size_t)N * (size_t)N);

        std::vector<unsigned int> idx;
        idx.reserve((size_t)(N - 1) * (size_t)(N - 1) * 6);

        for (int z = 0; z < N; ++z)
        {
            for (int x = 0; x < N; ++x)
            {
                float fx = ((float)x / (N - 1) - 0.5f) * OCEAN_SIZE;
                float fz = ((float)z / (N - 1) - 0.5f) * OCEAN_SIZE;
                verts.emplace_back(fx, 0.0f, fz);
            }
        }
        for (int z = 0; z < N - 1; ++z)
        {
            for (int x = 0; x < N - 1; ++x)
            {
                int i0 = z * N + x, i1 = z * N + x + 1, i2 = (z + 1) * N + x, i3 = (z + 1) * N + x + 1;
                idx.push_back(i0);
                idx.push_back(i2);
                idx.push_back(i1);
                idx.push_back(i1);
                idx.push_back(i2);
                idx.push_back(i3);
            }
        }

        if (waterVAO == 0)
        {
            glGenVertexArrays(1, &waterVAO);
            glGenBuffers(1, &waterVBO);
            glGenBuffers(1, &waterEBO);
        }

        glBindVertexArray(waterVAO);
        glBindBuffer(GL_ARRAY_BUFFER, waterVBO);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec3), verts.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, waterEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(unsigned int), idx.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);

        waterIndexCount = (GLsizei)idx.size();
    };

    // build initial ocean mesh
    rebuildOceanMesh(guiOceanN);

    // --- KHỞI TẠO THÙNG HÀNG (COIN) ---

    std::vector<glm::vec3> Cv;
    std::vector<unsigned int> Ci;
    addCube(Cv, Ci, glm::vec3(0), glm::vec3(1.0f)); // Tạo hình hộp
    GLuint cVAO, cVBO, cEBO;
    glGenVertexArrays(1, &cVAO);
    glGenBuffers(1, &cVBO);
    glGenBuffers(1, &cEBO);
    glBindVertexArray(cVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cVBO);
    glBufferData(GL_ARRAY_BUFFER, Cv.size() * sizeof(glm::vec3), Cv.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, Ci.size() * sizeof(unsigned int), Ci.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
    glEnableVertexAttribArray(0);

    // --- CÀI ĐẶT LEVEL GAME ---
    GameState gameState = MENU;
    float gameTimer = 60.0f;
    int score = 0;
    std::vector<Coin> coins;
    // Tạo 10 thùng hàng ngẫu nhiên
    for (int i = 0; i < 10; i++)
    {
        float rX = ((rand() / (float)RAND_MAX) - 0.5f) * 100.0f;
        float rZ = ((rand() / (float)RAND_MAX) - 0.5f) * 100.0f;
        coins.push_back({glm::vec3(rX, 0, rZ), true, (float)i});
    }

    // boat
    std::vector<glm::vec3> boatV;
    std::vector<unsigned int> boatI;
    buildBoatMeshComplex(24, 2.8f, 0.70f, 0.34f, boatV, boatI);
    GLuint boatVAO, boatVBO, boatEBO;
    int boatIndexCount = (int)boatI.size();
    glGenVertexArrays(1, &boatVAO);
    glGenBuffers(1, &boatVBO);
    glGenBuffers(1, &boatEBO);
    glBindVertexArray(boatVAO);
    glBindBuffer(GL_ARRAY_BUFFER, boatVBO);
    glBufferData(GL_ARRAY_BUFFER, boatV.size() * sizeof(glm::vec3), boatV.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boatEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, boatI.size() * sizeof(unsigned int), boatI.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // sky VAO
    GLuint skyVAO;
    glGenVertexArrays(1, &skyVAO);

    // uniforms (water/boat)
    glUseProgram(prog);
    auto U = [&](const char *n)
    { return glGetUniformLocation(prog, n); };
    GLint uModel = U("model"), uView = U("view"), uProj = U("projection"), uTime = U("time");
    GLint uLight = U("lightPos"), uViewPos = U("viewPos"), uIsWater = U("uIsWater"), uWorldOff = U("uWorldOffset");
    auto uWaveDir = [&](int i)
    { return glGetUniformLocation(prog, ("waveDir[" + std::to_string(i) + "]").c_str()); };
    auto uWaveAmp = [&](int i)
    { return glGetUniformLocation(prog, ("waveAmp[" + std::to_string(i) + "]").c_str()); };
    auto uWaveLen = [&](int i)
    { return glGetUniformLocation(prog, ("waveWLen[" + std::to_string(i) + "]").c_str()); };
    auto uWaveOm = [&](int i)
    { return glGetUniformLocation(prog, ("waveOmega[" + std::to_string(i) + "]").c_str()); };
    auto uSteep = [&](int i)
    { return glGetUniformLocation(prog, ("steepness[" + std::to_string(i) + "]").c_str()); };

    glm::vec2 d[N_WAVES];
    float A[N_WAVES], Lw[N_WAVES], Ww[N_WAVES], S[N_WAVES];
    std::srand(1337);
    for (int i = 0; i < N_WAVES; i++)
    {
        float ang = (std::rand() / (float)RAND_MAX) * 6.2831853f;
        d[i] = glm::normalize(glm::vec2(std::cos(ang), std::sin(ang)));
        Lw[i] = 2.0f + (std::rand() / (float)RAND_MAX) * 3.0f;
        A[i] = 0.25f / Lw[i];
        float k = 6.2831853f / Lw[i];
        Ww[i] = std::sqrt(G * k);
        S[i] = 0.6f;
        glUniform2f(uWaveDir(i), d[i].x, d[i].y);
        glUniform1f(uWaveAmp(i), A[i] * guiWaveHeight);
        glUniform1f(uWaveLen(i), Lw[i]);
        glUniform1f(uWaveOm(i), Ww[i]);
        glUniform1f(uSteep(i), S[i]);
    }

    // sky uniforms
    glUseProgram(skyProg);
    GLint sInvProj = glGetUniformLocation(skyProg, "uInvProj");
    GLint sInvViewR = glGetUniformLocation(skyProg, "uInvViewRot");
    GLint sRes = glGetUniformLocation(skyProg, "uResolution");
    GLint sSunDir = glGetUniformLocation(skyProg, "uSunDir");
    GLint sSunColor = glGetUniformLocation(skyProg, "uSunColor");
    GLint sTime = glGetUniformLocation(skyProg, "time");

    // camera/boat state
    glm::vec3 camTarget(0);
    float yaw = 180.0f, pitch = 20.0f, radius = 35.0f;

    glm::vec3 boatWorldPos(0.0f); // Vị trí thuyền (x, y, z)
    float boatHeading = 0.0f;     // Góc quay mũi thuyền
    float boatSpeed = 0.0f;       // Tốc độ hiện tại
    glm::mat4 Iden(1.0f);
    glm::vec3 boatScale(3.0f, 2.1f, 5.0f);
    glm::vec3 smoothCenter(0.0f);
    glm::quat smoothRot(1, 0, 0, 0);
    float last = (float)glfwGetTime();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(win))
    {
        float currentFrame = (float)glfwGetTime();
        float dt = currentFrame - last; // Khai báo dt ngay tại đây
        last = currentFrame;

        float t = currentFrame * guiTimeSpeed + guiSunPos;

        // THIẾT KẾ BẢNG ĐIỀU KHIỂN
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Ship Control"); // Tạo cửa sổ
        ImGui::SliderFloat("Song to (Height)", &guiWaveHeight, 0.0f, 10.0f);

        // Độ nét của sóng phụ thuộc nhiều vào độ mịn của lưới (N x N)
        ImGui::SliderInt("Do net luoi nuoc (N)", &guiOceanN, 64, 512);
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            rebuildOceanMesh(guiOceanN); // rebuild 1 lần sau khi kéo xong
        }

        ImGui::SliderFloat("Toc do thoi gian", &guiTimeSpeed, 0.0f, 5.0f);
        ImGui::SliderFloat("Mat troi", &guiSunPos, 0.0f, 100.0f);
        ImGui::Separator();
        ImGui::Text("Toc do thuyen: %.1f", boatSpeed);
        if (ImGui::Button("Reset Thuyen"))
        {
            boatWorldPos = glm::vec3(0);
            boatSpeed = 0;
        }
        ImGui::End();

        // LOGIC THẮNG/THUA
        if (gameState == PLAYING)
        {
            gameTimer -= dt;
            if (gameTimer <= 0)
            {
                gameState = LOST;
                boatSpeed = 0;
            }
            if (score >= 10)
            {
                gameState = WON;
                boatSpeed = 0;
            }
        }

        // VẼ GIAO DIỆN (HUD)
        // --- CỬA SỔ 2: MISSION CONTROL ---
        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::SetNextWindowSize(ImVec2(300, 200)); // Tăng chiều cao lên chút để đủ chỗ
        ImGui::Begin("MISSION CONTROL");

        if (gameState == MENU)
        {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "NHIEM VU: THU THAP 10 THUNG HANG");
            ImGui::Text("Ban co 90 giay.");
            ImGui::Text("Dieu khien: W/A/S/D");
            if (ImGui::Button("BAT DAU (START)", ImVec2(280, 40)))
            {
                // RESET GAME KHI BẮT ĐẦU
                gameState = PLAYING;
                gameTimer = 90.0f;
                score = 0;
                boatWorldPos = glm::vec3(0);
                boatHeading = 0.0f;
                boatSpeed = 0.0f;
                for (auto &c : coins)
                    c.active = true; // Hồi sinh thùng hàng
            }
        }
        else if (gameState == PLAYING)
        {
            ImGui::Text("THOI GIAN: %.1f s", gameTimer);
            ImGui::ProgressBar(score / 10.0f, ImVec2(280, 20));
            ImGui::Text("DA LAY: %d / 10", score);

            ImGui::Separator();
            
            if (ImGui::Button("CHOI LAI TU DAU", ImVec2(280, 30)))
            {
                gameState = MENU;
                boatSpeed = 0;
            }
        }
        else if (gameState == WON)
        {
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "CHUC MUNG! BAN DA THANG!");
            ImGui::Text("Thoi gian con du: %.1f s", gameTimer);

            if (ImGui::Button("CHOI LAI (REPLAY)", ImVec2(280, 40)))
            {
                gameState = MENU; // Quay về menu để start lại
            }
        }
        else if (gameState == LOST)
        {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "THAT BAI! HET GIO MAT ROI.");

            if (ImGui::Button("THU LAI (RETRY)", ImVec2(280, 40)))
            {
                gameState = PLAYING; 
                gameTimer = 90.0f;
                score = 0;
                boatWorldPos = glm::vec3(0);
                boatHeading = 0.0f;
                boatSpeed = 0.0f;
                for (auto &c : coins)
                    c.active = true;
            }
        }

        ImGui::Separator();
        ImGui::SliderFloat("Do cao song", &guiWaveHeight, 0.0f, 10.0f);

        ImGui::End(); 

        // Chỉ cho phép lái tàu khi đang PLAYING
        if (gameState == PLAYING)
        {
            // 1. XỬ LÝ INPUT (LÁI THUYỀN)
            if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS)
                boatHeading += 1.5f * dt;
            if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS)
                boatHeading -= 1.5f * dt;

            if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS)
                boatSpeed = glm::min(boatSpeed + 5.0f * dt, 15.0f);
            else if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS)
                boatSpeed = glm::max(boatSpeed - 5.0f * dt, -5.0f);
            else
                boatSpeed = glm::mix(boatSpeed, 0.0f, 1.0f * dt); // Ma sát nước

            // Tính vector hướng dựa trên góc quay
            glm::vec3 forwardDir(std::sin(boatHeading), 0.0f, std::cos(boatHeading));
            boatWorldPos += forwardDir * boatSpeed * dt;
        }
        // 2. XỬ LÝ CAMERA (Xoay quanh THUYỀN)
        if (glfwGetKey(win, GLFW_KEY_LEFT) == GLFW_PRESS)
            yaw += 60.f * dt;
        if (glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS)
            yaw -= 60.f * dt;
        if (glfwGetKey(win, GLFW_KEY_UP) == GLFW_PRESS)
            pitch = glm::clamp(pitch + 60.f * dt, -10.f, 80.f);
        if (glfwGetKey(win, GLFW_KEY_DOWN) == GLFW_PRESS)
            pitch = glm::clamp(pitch - 60.f * dt, -10.f, 80.f);
        if (glfwGetKey(win, GLFW_KEY_Q) == GLFW_PRESS)
            radius = glm::max(5.f, radius - 30.f * dt);
        if (glfwGetKey(win, GLFW_KEY_E) == GLFW_PRESS)
            radius += 30.f * dt;
        if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(win, 1);

        // === XỬ LÝ CAMERA THEO TRẠNG THÁI GAME ===
        glm::vec3 camPos;

        if (gameState == PLAYING) {
            // --- GÓC NHÌN THỨ 3 (THIRD PERSON VIEW) ---
            // Tính vector hướng trước mặt của thuyền (dựa vào boatHeading)
            glm::vec3 boatForward(sin(boatHeading), 0, cos(boatHeading));

            // Đặt Camera: 
            // - Nằm sau lưng thuyền: trừ đi vector Forward * khoảng cách (12m)
            // - Cao hơn thuyền: cộng thêm vector Y * độ cao (5m)
            float distBehind = 12.0f;
            float height = 5.0f;
            
            // Dùng mix để camera bám theo mượt hơn một chút (Lerp)
            glm::vec3 targetCamPos = boatWorldPos - (boatForward * distBehind) + glm::vec3(0, height, 0);
            
            // Gán trực tiếp (hoặc có thể dùng biến smooth nếu muốn mượt hơn nữa)
            camPos = targetCamPos;

            // Điểm nhìn (Target): Nhìn vào phía trước mũi tàu một chút
            camTarget = boatWorldPos + (boatForward * 5.0f) + glm::vec3(0, 2.0f, 0);
            
        } else {
            // --- GÓC NHÌN TỰ DO (MENU / GAME OVER) ---
            if (glfwGetKey(win, GLFW_KEY_LEFT) == GLFW_PRESS) yaw += 60.f * dt;
            if (glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS) yaw -= 60.f * dt;
            if (glfwGetKey(win, GLFW_KEY_UP) == GLFW_PRESS) pitch = glm::clamp(pitch + 60.f * dt, -10.f, 80.f);
            if (glfwGetKey(win, GLFW_KEY_DOWN) == GLFW_PRESS) pitch = glm::clamp(pitch - 60.f * dt, -10.f, 80.f);
            
            if (glfwGetKey(win, GLFW_KEY_Q) == GLFW_PRESS) radius = glm::max(5.f, radius - 30.f*dt);
            if (glfwGetKey(win, GLFW_KEY_E) == GLFW_PRESS) radius += 30.f*dt;

            // Tính toán vị trí cầu (Spherical coordinates)
            camTarget = boatWorldPos + glm::vec3(0, 2.0f, 0);
            camPos = camTarget + glm::vec3(
                radius * cos(glm::radians(yaw)) * cos(glm::radians(pitch)),
                radius * sin(glm::radians(pitch)),
                radius * sin(glm::radians(yaw)) * cos(glm::radians(pitch))
            );
        }

        // Tạo ma trận View cuối cùng
        glm::mat4 view = glm::lookAt(camPos, camTarget, glm::vec3(0, 1, 0));

        int w, h;
#if SAFE_NO_RESPONSIVE
        w = 1280;
        h = 720;
#else
        glfwGetFramebufferSize(win, &w, &h);
        if (w < 1)
            w = 1;
        if (h < 1)
            h = 1;
#endif
        glViewport(0, 0, w, h);
        glm::mat4 proj = glm::perspective(glm::radians(45.f), (float)w / (float)h, 0.1f, 500.f);

        // ---------- SKY PASS ----------
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glUseProgram(skyProg);

        glm::mat4 invProj = glm::inverse(proj);
        glm::mat3 invViewR = glm::mat3(glm::inverse(view));

        float az = t * 0.05f;
        glm::vec3 sunDir = glm::normalize(glm::vec3(std::cos(az) * 0.9f, 0.15f, std::sin(az) * 0.9f));
        glm::vec3 sunCol = glm::vec3(1.0f, 0.58f, 0.25f);

        glUniformMatrix4fv(sInvProj, 1, GL_FALSE, &invProj[0][0]);
        glUniformMatrix3fv(sInvViewR, 1, GL_FALSE, &invViewR[0][0]);
        glUniform2f(sRes, (float)w, (float)h);
        glUniform3f(sSunDir, sunDir.x, sunDir.y, sunDir.z);
        glUniform3f(sSunColor, sunCol.x, sunCol.y, sunCol.z);
        glUniform1f(sTime, t);

        glBindVertexArray(skyVAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);

        // ---------- SCENE PASS ----------
        glClear(GL_DEPTH_BUFFER_BIT);
        glUseProgram(prog);
        glUniformMatrix4fv(uView, 1, GL_FALSE, &view[0][0]);
        glUniformMatrix4fv(uProj, 1, GL_FALSE, &proj[0][0]);
        glUniform1f(uTime, t);
        // Sương mù nhẹ để che viền
        // Màu này phải khớp với màu 'horizon' trong shader
        glUniform3f(glGetUniformLocation(prog, "fogColor"), 0.95f, 0.48f, 0.22f);

        glm::vec3 lightPos = camPos + sunDir * 80.0f;
        glUniform3f(uLight, lightPos.x, lightPos.y, lightPos.z);
        glUniform3f(uViewPos, camPos.x, camPos.y, camPos.z);

        // water
        glUniform1i(uIsWater, 1);

        // Gửi toạ độ (0,0) để lưới nước bị đóng đinh tại chỗ
        glUniform2f(uWorldOff, 0.0f, 0.0f);

        glUniformMatrix4fv(uModel, 1, GL_FALSE, &Iden[0][0]);
        glBindVertexArray(waterVAO);
        glDrawElements(GL_TRIANGLES, waterIndexCount, GL_UNSIGNED_INT, 0);

        // --- VẼ THÙNG HÀNG & TÍNH VA CHẠM ---
        glUseProgram(prog);
        glUniform1i(glGetUniformLocation(prog, "uIsWater"), 0);
        glBindVertexArray(cVAO); // Sử dụng lưới hộp

        // Gửi màu vàng (R=1, G=0.8, B=0)
        glUniform3f(glGetUniformLocation(prog, "uColorOverride"), 1.0f, 0.8f, 0.0f);

        for(auto& c : coins) {
            if(!c.active) continue;

            // Kiểm tra va chạm 
            float dist = glm::distance(glm::vec2(boatWorldPos.x, boatWorldPos.z), glm::vec2(c.pos.x, c.pos.z));
            if(gameState == PLAYING && dist < 3.0f) { 
                c.active = false;
                score++;
            }

            glm::vec2 pos2D(c.pos.x, c.pos.z);
            glm::vec3 coinP = gerstnerCPU(pos2D, d, A, Lw, Ww, S, t);
            coinP.y *= guiWaveHeight; 

            glm::vec3 nW = normalCPU(pos2D, d, A, Lw, Ww, S, t);

            // Pha trộn 80% hướng lên trời (0,1,0) và 20% hướng sóng (nW)
            // Giúp thùng hàng luôn giữ thẳng đứng, chỉ lắc nhẹ khi sóng to
            glm::vec3 stableN = glm::normalize(glm::mix(glm::vec3(0,1,0), nW, 0.5f));

            // Tạo ma trận biến đổi (Matrix)
            glm::mat4 cM = glm::translate(glm::mat4(1), glm::vec3(c.pos.x, coinP.y - 0.2f, c.pos.z)); // -0.2 để chìm nhẹ xuống nước
            
            // Xoay theo hướng ổn định (Vật lý)
            cM = cM * glm::mat4_cast(glm::rotation(glm::vec3(0,1,0), stableN));
            
            // Xoay nhẹ theo trục Y (Trục thẳng đứng) để tạo vẻ tự nhiên
            // Thay vì xoay lung tung như cũ, giờ nó chỉ xoay ngang từ từ
            cM = glm::rotate(cM, c.angle + t * 0.5f, glm::vec3(0,1,0)); 
            
            cM = glm::scale(cM, glm::vec3(1.5f)); // Kích thước

            // Vẽ
            glUniformMatrix4fv(glGetUniformLocation(prog,"model"),1,0,&cM[0][0]);
            glDrawElements(GL_TRIANGLES,(GLsizei)Ci.size(),GL_UNSIGNED_INT,0);
        }

        // Trả lại màu bình thường để vẽ thuyền (QUAN TRỌNG)
        glUniform3f(glGetUniformLocation(prog, "uColorOverride"), 0, 0, 0);

        // boat float & tilt
        // CẬP NHẬT: Dùng vị trí THỰC của thuyền để tính độ cao sóng
        glm::vec2 pos2D(boatWorldPos.x, boatWorldPos.z);
        glm::vec3 pW = gerstnerCPU(pos2D, d, A, Lw, Ww, S, t);
        glm::vec3 nW = normalCPU(pos2D, d, A, Lw, Ww, S, t);

        glm::vec3 stableNormal = glm::mix(glm::vec3(0, 1, 0), nW, 0.3f);
        stableNormal = glm::normalize(stableNormal);

        // Target Center bây giờ là vị trí thuyền + độ cao sóng
        glm::vec3 targetC = glm::vec3(boatWorldPos.x, pW.y, boatWorldPos.z);

        glm::quat targetR = glm::rotation(glm::vec3(0, 1, 0), stableNormal);
        float k = glm::clamp(3.0f * dt, 0.0f, 1.0f);
        smoothCenter = glm::mix(smoothCenter, targetC, k);
        smoothRot = glm::normalize(glm::slerp(smoothRot, targetR, k));

        // TÍNH TOÁN MA TRẬN THUYỀN
        // Dịch chuyển đến vị trí sóng
        glm::mat4 boatM = glm::translate(glm::mat4(1), smoothCenter + glm::vec3(0, boatScale.y * 0.5f, 0));
        // Nghiêng theo sóng
        boatM = boatM * glm::mat4_cast(smoothRot);
        // Xoay theo hướng lái (MỚI)
        boatM = glm::rotate(boatM, boatHeading, glm::vec3(0, 1, 0));
        // Scale
        boatM = glm::scale(boatM, boatScale);

        glDisable(GL_CULL_FACE); // thấy cả bên trong khoang
        glUniform1i(uIsWater, 0);
        glUniformMatrix4fv(uModel, 1, GL_FALSE, &boatM[0][0]);
        glBindVertexArray(boatVAO);
        glDrawElements(GL_TRIANGLES, boatIndexCount, GL_UNSIGNED_INT, 0);
        glEnable(GL_CULL_FACE);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &waterVAO);
    glDeleteBuffers(1, &waterVBO);
    glDeleteBuffers(1, &waterEBO);

    glDeleteVertexArrays(1, &cVAO);
    glDeleteBuffers(1, &cVBO);
    glDeleteBuffers(1, &cEBO);
    glDeleteVertexArrays(1, &boatVAO);
    glDeleteBuffers(1, &boatVBO);
    glDeleteBuffers(1, &boatEBO);
    glDeleteVertexArrays(1, &skyVAO);
    glDeleteProgram(prog);
    glDeleteProgram(skyProg);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(win);
    glfwTerminate();

    return 0;
}