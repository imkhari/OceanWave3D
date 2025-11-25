// === Build (Windows)
// g++ -std=c++17 -O2 .\main.cpp .\src\glad.c -I.\include -I"D:\SETUP\msys64\ucrt64\include" -L"D:\SETUP\msys64\ucrt64\lib" -o main.exe -lglfw3 -lopengl32 -lgdi32 -limm32 -luser32 -lshell32 -lversion -lwinmm -g
// .\main.exe

#define SAFE_NO_RESPONSIVE 1  // 1: lưới rộng cố định; 0: ocean bám camera (infinite look)
#define NO_DERIVATIVES    0  // 0: dFdx/dFdy tính normal nước ở FS; 1: normal từ VS

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cmath>

// ===================== WATER/BOAT Shaders (body) =====================
static const char* VS_BODY = R"(
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

static const char* FS_BODY = R"(
in vec3 vPos;
in vec3 vNormalVS;
out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform float time;
uniform int  uIsWater;

void main(){
#if NO_DERIVATIVES
    vec3 N = normalize(vNormalVS);
#else
    vec3 N = normalize(cross(dFdx(vPos), dFdy(vPos)));
    if(!gl_FrontFacing) N = -N;
#endif
    vec3 V = normalize(viewPos - vPos);
    vec3 L = normalize(lightPos - vPos);
    vec3 H = normalize(L + V);

    float diff    = max(dot(N, L), 0.0);
    float spec    = pow(max(dot(N, H), 0.0), 64.0);
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0);

    vec3 color;
    if(uIsWater==1){
        float slope = length(N.xz);
        float waveNoise = sin(vPos.x*0.1 + time*0.8)*0.5 + cos(vPos.z*0.1 + time*0.6)*0.5;
        float foam = smoothstep(0.35, 0.85, slope + 0.3*waveNoise);
        foam = pow(foam, 1.2);

        vec3 deep    = vec3(0.0, 0.25, 0.45);
        vec3 shallow = vec3(0.1, 0.5, 0.7);
        vec3 foamCol = vec3(0.92, 0.94, 0.98);
        vec3 baseCol = mix(deep, shallow, fresnel);
        color = mix(baseCol, foamCol, foam);
    }else{
        color = vec3(0.90,0.30,0.18); // boat
    }

    vec3 ambient  = 0.25 * color;
    vec3 diffuse  = 0.75 * diff * color;
    vec3 specular = 0.40 * spec * vec3(1.0);
    vec3 outc = ambient + diffuse + specular;
    outc = mix(outc, vec3(1.0), 0.1*fresnel);
    FragColor = vec4(outc,1.0);
}
)";

// ===================== SKY Shaders (Sunset + Clouds) =====================
static const char* SKY_VS = R"(
void main(){
    vec2 pos = vec2( (gl_VertexID==0)? -1.0 : (gl_VertexID==1)?  3.0 : -1.0,
                     (gl_VertexID==0)? -1.0 : (gl_VertexID==1)? -1.0 :  3.0 );
    gl_Position = vec4(pos, 0.0, 1.0);
}
)";

static const char* SKY_FS = R"(
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
static GLuint compile(GLenum type, const std::string& fullSrc){
    GLuint s = glCreateShader(type);
    const char* p = fullSrc.c_str(); GLint n = (GLint)fullSrc.size();
    glShaderSource(s, 1, &p, &n);
    glCompileShader(s);
    GLint ok=0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if(!ok){ char log[4096]; glGetShaderInfoLog(s, 4096, NULL, log);
        std::cerr << "Shader compile error:\n" << log << "\n"; std::exit(-1); }
    return s;
}
static GLuint link(GLuint vs, GLuint fs){
    GLuint p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok=0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if(!ok){ char log[4096]; glGetProgramInfoLog(p, 4096, NULL, log);
        std::cerr << "Link error:\n" << log << "\n"; std::exit(-1); }
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ===================== Ocean helpers (CPU) =====================
static const int   N_WAVES = 8;
static const float PI2     = 6.28318530718f;
static const float G       = 9.8f;
static const float EPS_N   = 0.02f;

static glm::vec3 gerstnerCPU(const glm::vec2& pos, const glm::vec2 d[], const float A[],
                             const float L[], const float W[], const float S[], float t){
    glm::vec3 p(pos.x,0.0f,pos.y);
    for(int i=0;i<N_WAVES;i++){
        glm::vec2 dir = glm::normalize(d[i]);
        float k = PI2 / L[i];
        float f = k*glm::dot(dir, pos) - W[i]*t;
        p.x += (S[i]*A[i]*dir.x)*std::sin(f);
        p.z += (S[i]*A[i]*dir.y)*std::sin(f);
        p.y +=  A[i]*std::cos(f);
    }
    return p;
}
static glm::vec3 normalCPU(const glm::vec2& pos, const glm::vec2 d[], const float A[],
                           const float L[], const float W[], const float S[], float t){
    glm::vec3 p   = gerstnerCPU(pos,d,A,L,W,S,t);
    glm::vec3 pDx = gerstnerCPU(pos+glm::vec2(EPS_N,0),d,A,L,W,S,t);
    glm::vec3 pDz = gerstnerCPU(pos+glm::vec2(0,EPS_N),d,A,L,W,S,t);
    return glm::normalize(glm::cross(pDz - p, pDx - p));
}

// ===================== Boat mesh (open-deck, rim, bow sealed) =====================
static void buildBoatMeshOpenDeck(
    int seg, float L, float halfW0, float H,
    std::vector<glm::vec3>& V, std::vector<unsigned int>& I
){
    V.clear(); I.clear();

    auto halfWidth = [&](float t){
        float w = halfW0 * pow(1.0f - t, 0.25f) + 0.02f*(1.0f - t);
        return std::max(0.02f, w);
    };
    auto yDeck   = [&](float){ return  0.22f; };
    auto yBottom = [&](float t){ return -H * (0.8f - 0.4f*t); };

    const float rimInset = 0.10f;
    const float rimDrop  = 0.05f;

    float z0 = -L*0.5f, z1 =  L*0.5f;

    for(int i=0;i<=seg;i++){
        float t = i/(float)seg;
        float z = z0 + (z1-z0)*t;
        float hw = halfWidth(t);
        float yT = yDeck(t);
        float yB = yBottom(t);

        float hwInner = std::max(0.01f, hw*(1.0f - rimInset));
        float yR = yT - rimDrop;

        V.push_back({-hw, yT, z});      // 0 LT_o
        V.push_back({ hw, yT, z});      // 1 RT_o
        V.push_back({-hw*0.72f, yB, z});// 2 LB_o
        V.push_back({ hw*0.72f, yB, z});// 3 RB_o
        V.push_back({-hwInner, yR, z}); // 4 LT_i
        V.push_back({ hwInner, yR, z}); // 5 RT_i
    }

    auto vid = [&](int i, int k){ return i*6 + k; };

    for(int i=0;i<seg;i++){
        int LT0=vid(i,0), RT0=vid(i,1), LB0=vid(i,2), RB0=vid(i,3), LTi0=vid(i,4), RTi0=vid(i,5);
        int LT1=vid(i+1,0), RT1=vid(i+1,1), LB1=vid(i+1,2), RB1=vid(i+1,3), LTi1=vid(i+1,4), RTi1=vid(i+1,5);

        // sides
        I.insert(I.end(), { (unsigned)LB0,(unsigned)LT0,(unsigned)LB1,
                            (unsigned)LT0,(unsigned)LT1,(unsigned)LB1 });
        I.insert(I.end(), { (unsigned)RT0,(unsigned)RB0,(unsigned)RT1,
                            (unsigned)RB0,(unsigned)RB1,(unsigned)RT1 });
        // bottom
        I.insert(I.end(), { (unsigned)LB0,(unsigned)RB1,(unsigned)RB0,
                            (unsigned)LB0,(unsigned)LB1,(unsigned)RB1 });
        // rim wall
        I.insert(I.end(), { (unsigned)LT0,(unsigned)LT1,(unsigned)LTi1,
                            (unsigned)LT0,(unsigned)LTi1,(unsigned)LTi0 });
        I.insert(I.end(), { (unsigned)RT0,(unsigned)RTi1,(unsigned)RT1,
                            (unsigned)RT0,(unsigned)RTi0,(unsigned)RTi1 });
    }

    // transom
    {
        unsigned LT=vid(0,0), RT=vid(0,1), LB=vid(0,2), RB=vid(0,3);
        unsigned LTi=vid(0,4), RTi=vid(0,5);
        I.insert(I.end(), { LT, RT, RB,   LT, RB, LB });
        I.insert(I.end(), { LT, LTi, RTi,   LT, RTi, RT });
    }

    // bow sealed
    {
        unsigned LT = vid(seg,0), RT = vid(seg,1), LB = vid(seg,2), RB = vid(seg,3);
        unsigned LTi= vid(seg,4), RTi= vid(seg,5);
        float zTip  = ( -L*0.5f + L*0.5f ) + 0.05f;
        float yB    = yBottom(1.0f) - 0.02f;
        float yR    = yDeck(1.0f) - 0.05f;

        unsigned botTip = (unsigned)V.size(); V.push_back({ 0.0f, yB, zTip });
        unsigned rimTip = (unsigned)V.size(); V.push_back({ 0.0f, yR, zTip - 0.01f });

        I.insert(I.end(), { LB, botTip, RB });
        I.insert(I.end(), { LT, LB, botTip });
        I.insert(I.end(), { RB, RT, botTip });
        I.insert(I.end(), { LT, botTip, RT });
        I.insert(I.end(), { LTi, rimTip, RTi });
        I.insert(I.end(), { LT,  LTi,    rimTip });
        I.insert(I.end(), { rimTip, RTi,  RT    });
    }
}

// ===================== MAIN =====================
int main(){
    if(!glfwInit()){ std::cerr<<"GLFW init fail\n"; return -1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* win = glfwCreateWindow(1280,720,"Ocean + Boat + Sunset Sky",nullptr,nullptr);
    if(!win){ std::cerr<<"GLFW create fail\n"; return -1; }
    glfwMakeContextCurrent(win);

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cerr<<"GLAD init fail\n"; return -1;
    }
    std::cout << "GL_VERSION=" << glGetString(GL_VERSION) << "\n";

#if !SAFE_NO_RESPONSIVE
    glfwSetFramebufferSizeCallback(win, [](GLFWwindow*,int w,int h){ glViewport(0,0,w?h:1,h?h:1); });
#endif

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK); glFrontFace(GL_CCW);

    // programs
    auto makeSrc = [](const char* body, bool safe, bool deriv){
        std::string s = "#version 330 core\n";
        s += std::string("#define SAFE_NO_RESPONSIVE ") + (safe ? "1\n":"0\n");
        s += std::string("#define NO_DERIVATIVES ")    + (deriv? "1\n":"0\n");
        s += body;
        return s;
    };
    std::string VS = makeSrc(VS_BODY, SAFE_NO_RESPONSIVE, NO_DERIVATIVES);
    std::string FS = makeSrc(FS_BODY, SAFE_NO_RESPONSIVE, NO_DERIVATIVES);
    GLuint vs = compile(GL_VERTEX_SHADER, VS);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FS);
    GLuint prog = link(vs, fs);

    std::string skyVS = std::string("#version 330 core\n") + SKY_VS;
    std::string skyFS = std::string("#version 330 core\n") + SKY_FS;
    GLuint sv = compile(GL_VERTEX_SHADER, skyVS);
    GLuint sf = compile(GL_FRAGMENT_SHADER, skyFS);
    GLuint skyProg = link(sv, sf);

    // ocean grid
    const int   N    = 200;
    const float SIZE = SAFE_NO_RESPONSIVE ? 80.0f : 50.0f;
    std::vector<glm::vec3> verts; verts.reserve(N*N);
    std::vector<unsigned int> idx;
    for(int z=0; z<N; ++z){
        for(int x=0; x<N; ++x){
            float fx = ((float)x/(N-1)-0.5f)*SIZE;
            float fz = ((float)z/(N-1)-0.5f)*SIZE;
            verts.emplace_back(fx, 0.0f, fz);
        }
    }
    for(int z=0; z<N-1; ++z){
        for(int x=0; x<N-1; ++x){
            int i0=z*N+x, i1=z*N+x+1, i2=(z+1)*N+x, i3=(z+1)*N+x+1;
            idx.push_back(i0); idx.push_back(i2); idx.push_back(i1);
            idx.push_back(i1); idx.push_back(i2); idx.push_back(i3);
        }
    }
    GLuint vao,vbo,ebo;
    glGenVertexArrays(1,&vao);
    glGenBuffers(1,&vbo);
    glGenBuffers(1,&ebo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(glm::vec3), verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size()*sizeof(unsigned int), idx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // boat
    std::vector<glm::vec3> boatV; std::vector<unsigned int> boatI;
    buildBoatMeshOpenDeck(24, 2.8f, 0.70f, 0.34f, boatV, boatI);
    GLuint boatVAO, boatVBO, boatEBO; int boatIndexCount=(int)boatI.size();
    glGenVertexArrays(1,&boatVAO);
    glGenBuffers(1,&boatVBO);
    glGenBuffers(1,&boatEBO);
    glBindVertexArray(boatVAO);
    glBindBuffer(GL_ARRAY_BUFFER, boatVBO);
    glBufferData(GL_ARRAY_BUFFER, boatV.size()*sizeof(glm::vec3), boatV.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boatEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, boatI.size()*sizeof(unsigned int), boatI.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // sky VAO
    GLuint skyVAO; glGenVertexArrays(1,&skyVAO);

    // uniforms (water/boat)
    glUseProgram(prog);
    auto U = [&](const char* n){ return glGetUniformLocation(prog,n); };
    GLint uModel=U("model"), uView=U("view"), uProj=U("projection"), uTime=U("time");
    GLint uLight=U("lightPos"), uViewPos=U("viewPos"), uIsWater=U("uIsWater"), uWorldOff=U("uWorldOffset");
    auto uWaveDir=[&](int i){return glGetUniformLocation(prog,("waveDir["+std::to_string(i)+"]").c_str());};
    auto uWaveAmp=[&](int i){return glGetUniformLocation(prog,("waveAmp["+std::to_string(i)+"]").c_str());};
    auto uWaveLen=[&](int i){return glGetUniformLocation(prog,("waveWLen["+std::to_string(i)+"]").c_str());};
    auto uWaveOm =[&](int i){return glGetUniformLocation(prog,("waveOmega["+std::to_string(i)+"]").c_str());};
    auto uSteep =[&](int i){return glGetUniformLocation(prog,("steepness["+std::to_string(i)+"]").c_str());};

    glm::vec2 d[N_WAVES]; float A[N_WAVES], Lw[N_WAVES], Ww[N_WAVES], S[N_WAVES];
    std::srand(1337);
    for(int i=0;i<N_WAVES;i++){
        float ang = (std::rand()/(float)RAND_MAX)*6.2831853f;
        d[i] = glm::normalize(glm::vec2(std::cos(ang), std::sin(ang)));
        Lw[i] = 2.0f + (std::rand()/(float)RAND_MAX)*3.0f;
        A[i]  = 0.20f/Lw[i];
        float k = 6.2831853f/Lw[i];
        Ww[i] = std::sqrt(G*k);
        S[i]  = 0.6f;
        glUniform2f(uWaveDir(i), d[i].x, d[i].y);
        glUniform1f(uWaveAmp(i), A[i]);
        glUniform1f(uWaveLen(i), Lw[i]);
        glUniform1f(uWaveOm (i), Ww[i]);
        glUniform1f(uSteep  (i), S[i]);
    }

    // sky uniforms
    glUseProgram(skyProg);
    GLint sInvProj  = glGetUniformLocation(skyProg, "uInvProj");
    GLint sInvViewR = glGetUniformLocation(skyProg, "uInvViewRot");
    GLint sRes      = glGetUniformLocation(skyProg, "uResolution");
    GLint sSunDir   = glGetUniformLocation(skyProg, "uSunDir");
    GLint sSunColor = glGetUniformLocation(skyProg, "uSunColor");
    GLint sTime     = glGetUniformLocation(skyProg, "time");

    // camera/boat state
    glm::vec3 camTarget(0);
    float yaw=180.0f, pitch=20.0f, radius=35.0f;
    glm::mat4 Iden(1.0f);
    glm::vec3 boatScale(3.0f, 2.1f, 5.0f);
    glm::vec3 smoothCenter(0.0f); glm::quat smoothRot(1,0,0,0);
    float last = (float)glfwGetTime();

    while(!glfwWindowShouldClose(win)){
        float t=(float)glfwGetTime(), dt=t-last; last=t;

        if (glfwGetKey(win, GLFW_KEY_LEFT)  == GLFW_PRESS) yaw   += 60.f*dt;
        if (glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS) yaw   -= 60.f*dt;
        if (glfwGetKey(win, GLFW_KEY_UP)    == GLFW_PRESS) pitch = glm::clamp(pitch + 60.f*dt, -10.f, 80.f);
        if (glfwGetKey(win, GLFW_KEY_DOWN)  == GLFW_PRESS) pitch = glm::clamp(pitch - 60.f*dt, -10.f, 80.f);
        if (glfwGetKey(win, GLFW_KEY_W)     == GLFW_PRESS) radius = glm::max(5.f, radius - 30.f*dt);
        if (glfwGetKey(win, GLFW_KEY_S)     == GLFW_PRESS) radius += 30.f*dt;
        if (glfwGetKey(win, GLFW_KEY_ESCAPE)== GLFW_PRESS) glfwSetWindowShouldClose(win,1);

        // --- CAMERA & VIEW/PROJECTION ---
        float yr=glm::radians(yaw), pr=glm::radians(pitch);
        glm::vec3 camPos(
            radius*std::cos(yr)*std::cos(pr),
            radius*std::sin(pr),
            radius*std::sin(yr)*std::cos(pr)
        );
        glm::mat4 view = glm::lookAt(camPos, camTarget, glm::vec3(0,1,0));

        int w, h;
    #if SAFE_NO_RESPONSIVE
        w = 1280; h = 720;
    #else
        glfwGetFramebufferSize(win,&w,&h);
        if (w<1) w=1; if (h<1) h=1;
    #endif
        glViewport(0,0,w,h);
        glm::mat4 proj = glm::perspective(glm::radians(45.f), (float)w/(float)h, 0.1f, 500.f);

        // ---------- SKY PASS ----------
        glDisable(GL_DEPTH_TEST); glDepthMask(GL_FALSE);
        glUseProgram(skyProg);

        glm::mat4 invProj   = glm::inverse(proj);
        glm::mat3 invViewR  = glm::mat3(glm::inverse(view));

        // Sun low near horizon (sunset)
        float az = t*0.08f;
        glm::vec3 sunDir = glm::normalize(glm::vec3(std::cos(az)*0.9f, 0.15f, std::sin(az)*0.9f));
        glm::vec3 sunCol = glm::vec3(1.0f, 0.58f, 0.25f);

        glUniformMatrix4fv(sInvProj, 1, GL_FALSE, &invProj[0][0]);
        glUniformMatrix3fv(sInvViewR,1, GL_FALSE, &invViewR[0][0]);
        glUniform2f(sRes, (float)w, (float)h);
        glUniform3f(sSunDir, sunDir.x, sunDir.y, sunDir.z);
        glUniform3f(sSunColor, sunCol.x, sunCol.y, sunCol.z);
        glUniform1f(sTime, t);

        glBindVertexArray(skyVAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glDepthMask(GL_TRUE); glEnable(GL_DEPTH_TEST);

        // ---------- SCENE PASS ----------
        glClear(GL_DEPTH_BUFFER_BIT);
        glUseProgram(prog);
        glUniformMatrix4fv(uView,1,GL_FALSE,&view[0][0]);
        glUniformMatrix4fv(uProj,1,GL_FALSE,&proj[0][0]);
        glUniform1f(uTime,t);

        glm::vec3 lightPos = camPos + sunDir * 80.0f;
        glUniform3f(uLight, lightPos.x, lightPos.y, lightPos.z);
        glUniform3f(uViewPos,camPos.x,camPos.y,camPos.z);

        // water
        glUniform1i(uIsWater,1);
    #if SAFE_NO_RESPONSIVE
        glUniform2f(uWorldOff,0,0);
    #else
        glUniform2f(uWorldOff,camPos.x,camPos.z);
    #endif
        glUniformMatrix4fv(uModel,1,GL_FALSE,&Iden[0][0]);
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES,(GLsizei)idx.size(),GL_UNSIGNED_INT,0);

        // boat float & tilt
        glm::vec2 pos2D(0,0);
        glm::vec3 pW = gerstnerCPU(pos2D,d,A,Lw,Ww,S,t);
        glm::vec3 nW = normalCPU  (pos2D,d,A,Lw,Ww,S,t);
        glm::vec3 targetC = glm::vec3(0,pW.y,0);
        glm::quat targetR = glm::rotation(glm::vec3(0,1,0), glm::normalize(nW));
        float k = glm::clamp(12.f*dt,0.f,1.f);
        smoothCenter = glm::mix(smoothCenter, targetC, k);
        smoothRot    = glm::normalize(glm::slerp(smoothRot, targetR, k));

        glm::mat4 boatM = glm::translate(glm::mat4(1), smoothCenter + glm::vec3(0, boatScale.y*0.5f, 0));
        boatM = boatM * glm::mat4_cast(smoothRot);
        boatM = glm::scale(boatM, boatScale);

        glDisable(GL_CULL_FACE); // thấy cả bên trong khoang
        glUniform1i(uIsWater,0);
        glUniformMatrix4fv(uModel,1,GL_FALSE,&boatM[0][0]);
        glBindVertexArray(boatVAO);
        glDrawElements(GL_TRIANGLES, boatIndexCount, GL_UNSIGNED_INT, 0);
        glEnable(GL_CULL_FACE);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1,&vao); glDeleteBuffers(1,&vbo); glDeleteBuffers(1,&ebo);
    glDeleteVertexArrays(1,&boatVAO); glDeleteBuffers(1,&boatVBO); glDeleteBuffers(1,&boatEBO);
    glDeleteVertexArrays(1,&skyVAO);
    glDeleteProgram(prog); glDeleteProgram(skyProg);
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
