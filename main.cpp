#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>

const unsigned int SCR_W = 1280;
const unsigned int SCR_H = 720;

std::string loadFile(const char *path)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

GLuint compileShader(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char buf[1024];
        glGetShaderInfoLog(s, 1024, NULL, buf);
        std::cerr << "Shader compile error: " << buf << "\n";
        exit(-1);
    }
    return s;
}

GLuint createProgram(const char *vsSrc, const char *fsSrc)
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, vsSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fsSrc);
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    int ok;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        char buf[1024];
        glGetProgramInfoLog(p, 1024, NULL, buf);
        std::cerr << "Link error: " << buf << "\n";
        exit(-1);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

// -------------------- WAVE HEIGHT --------------------
float getWaveHeight(const glm::vec2& pos, const glm::vec2 dirs[8], const float amps[8],
                    const float wlens[8], const float speeds[8], const float steeps[8], float time)
{
    float y = 0.0f;
    for (int i = 0; i < 8; ++i)
    {
        float k = 2.0f * 3.14159f / wlens[i];
        float phase = k * glm::dot(dirs[i], pos) - speeds[i] * time;
        y += amps[i] * sin(phase);
    }
    return y;
}

// -------------------- WAVE NORMAL (GRADIENT) --------------------
glm::vec3 getWaveNormal(const glm::vec2& pos, const glm::vec2 dirs[8], const float amps[8],
                        const float wlens[8], const float speeds[8], const float steeps[8], float time)
{
    float dx = 0.0f, dz = 0.0f;
    for (int i = 0; i < 8; ++i)
    {
        float k = 2.0f * 3.14159f / wlens[i];
        float phase = k * glm::dot(dirs[i], pos) - speeds[i] * time;
        float c = cos(phase);
        float wa = amps[i] * k;
        dx -= dirs[i].x * wa * c;
        dz -= dirs[i].y * wa * c;
    }
    glm::vec3 n(-dx, 1.0f, -dz);
    return glm::normalize(n);
}

// -------------------- ROTATION FROM NORMAL --------------------
glm::mat4 rotationFromNormal(const glm::vec3& normal)
{
    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(up, normal));
    glm::vec3 newUp = glm::cross(normal, right);

    glm::mat4 rot(1.0f);
    rot[0] = glm::vec4(right, 0.0f);
    rot[1] = glm::vec4(newUp, 0.0f);
    rot[2] = glm::vec4(normal, 0.0f);
    return rot;
}

// -------------------- MAIN --------------------
int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow *window = glfwCreateWindow(SCR_W, SCR_H, "Gerstner Ocean with Floating Cube", NULL, NULL);
    if (!window)
    {
        std::cerr << "GLFW create fail\n";
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "GLAD init fail\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // load shaders
    std::string vsSrc = loadFile("vertex.glsl");
    std::string fsSrc = loadFile("fragment.glsl");
    GLuint program = createProgram(vsSrc.c_str(), fsSrc.c_str());

    // -------------------- OCEAN GRID --------------------
    const int N = 200;
    const float SIZE = 50.0f;
    std::vector<glm::vec2> verts;
    std::vector<unsigned int> idx;
    for (int z = 0; z < N; ++z)
        for (int x = 0; x < N; ++x)
        {
            float fx = ((float)x / (N - 1) - 0.5f) * SIZE;
            float fz = ((float)z / (N - 1) - 0.5f) * SIZE;
            verts.emplace_back(fx, fz);
        }
    for (int z = 0; z < N - 1; ++z)
        for (int x = 0; x < N - 1; ++x)
        {
            int i0 = z * N + x;
            int i1 = z * N + (x + 1);
            int i2 = (z + 1) * N + x;
            int i3 = (z + 1) * N + (x + 1);
            idx.push_back(i0); idx.push_back(i2); idx.push_back(i1);
            idx.push_back(i1); idx.push_back(i2); idx.push_back(i3);
        }

    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(glm::vec2), verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(unsigned int), idx.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void *)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // -------------------- UNIFORMS --------------------
    glUseProgram(program);
    GLint modelLoc = glGetUniformLocation(program, "model");
    GLint viewLoc = glGetUniformLocation(program, "view");
    GLint projLoc = glGetUniformLocation(program, "projection");
    GLint timeLoc = glGetUniformLocation(program, "time");
    GLint lightLoc = glGetUniformLocation(program, "lightPos");
    GLint viewPosLoc = glGetUniformLocation(program, "viewPos");

    // -------------------- WAVE PARAMETERS --------------------
    glm::vec2 dirs[8];
    float amps[8], wlens[8], speeds[8], steeps[8];
    for (int i = 0; i < 8; i++)
    {
        float ang = ((rand() % 100) / 100.0f - 0.5f) * 0.6f;
        dirs[i] = glm::normalize(glm::vec2(cos(ang), sin(ang)));
        wlens[i] = 2.0f + (rand() % 100) / 50.0f;
        amps[i] = 0.2f / wlens[i];
        speeds[i] = sqrt(9.8f * (2.0f * 3.14159f / wlens[i]));
        steeps[i] = 0.6f;
    }
    for (int i = 0; i < 8; i++)
    {
        std::string dname = "waveDir[" + std::to_string(i) + "]";
        std::string aname = "waveAmp[" + std::to_string(i) + "]";
        std::string wname = "waveWLen[" + std::to_string(i) + "]";
        std::string sname = "waveSpeed[" + std::to_string(i) + "]";
        std::string steep = "steepness[" + std::to_string(i) + "]";
        glUniform2f(glGetUniformLocation(program, dname.c_str()), dirs[i].x, dirs[i].y);
        glUniform1f(glGetUniformLocation(program, aname.c_str()), amps[i]);
        glUniform1f(glGetUniformLocation(program, wname.c_str()), wlens[i]);
        glUniform1f(glGetUniformLocation(program, sname.c_str()), speeds[i]);
        glUniform1f(glGetUniformLocation(program, steep.c_str()), steeps[i]);
    }

    // -------------------- CAMERA --------------------
    glm::vec3 camTarget(0.0f, 0.0f, 0.0f);
    glm::mat4 model = glm::mat4(1.0f);

    float lastTime = (float)glfwGetTime();

    float yaw = 180.0f;
    float pitch = 20.0f;
    float radius = 35.0f;

    // -------------------- CUBE --------------------
    glm::vec3 cubePos(0.0f, 0.0f, 0.0f);
    glm::vec3 cubeScale(3.0f, 3.0f, 3.0f);
    GLuint cubeVAO, cubeVBO, cubeEBO;
    {
        float vertices[] = {
            -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f,
            -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f
        };
        unsigned int indices[] = {
            0,1,2, 2,3,0, 4,5,6, 6,7,4, 0,1,5, 5,4,0, 2,3,7, 7,6,2, 0,3,7, 7,4,0, 1,2,6, 6,5,1
        };
        glGenVertexArrays(1, &cubeVAO);
        glGenBuffers(1, &cubeVBO);
        glGenBuffers(1, &cubeEBO);

        glBindVertexArray(cubeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    // -------------------- SMOOTH FLOATING (LOW-PASS FILTER) --------------------
    glm::vec3 smoothCubeCenter = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::quat smoothCubeRot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    // -------------------- RENDER LOOP --------------------
    while (!glfwWindowShouldClose(window))
    {
        float t = (float)glfwGetTime();
        float dt = t - lastTime;
        lastTime = t;

        // --- CAMERA CONTROLS ---
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) yaw += 60.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) yaw -= 60.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) pitch = glm::clamp(pitch + 60.0f*dt, -10.0f, 80.0f);
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) pitch = glm::clamp(pitch - 60.0f*dt, -10.0f, 80.0f);
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) radius = glm::max(5.0f, radius - 30.0f*dt);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) radius += 30.0f*dt;

        float yr = glm::radians(yaw);
        float pr = glm::radians(pitch);
        glm::vec3 camPos(
            radius * cos(yr) * cos(pr),
            radius * sin(pr),
            radius * sin(yr) * cos(pr)
        );

        glm::mat4 view = glm::lookAt(camPos, camTarget, glm::vec3(0.0f,1.0f,0.0f));
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)SCR_W/(float)SCR_H, 0.1f, 200.0f);

        glViewport(0,0,SCR_W,SCR_H);
        glClearColor(0.05f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(program);
        glUniformMatrix4fv(viewLoc,1,GL_FALSE,&view[0][0]);
        glUniformMatrix4fv(projLoc,1,GL_FALSE,&proj[0][0]);
        glUniform1f(timeLoc,t);
        glUniform3f(lightLoc,10.0f,20.0f,10.0f);
        glUniform3f(viewPosLoc,camPos.x,camPos.y,camPos.z);

        // --- RENDER OCEAN ---
        glUniformMatrix4fv(modelLoc,1,GL_FALSE,&model[0][0]);
        glBindVertexArray(VAO);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES,(GLsizei)idx.size(),GL_UNSIGNED_INT,0);

        // --- RENDER FLOATING CUBE (SMOOTH + TILT) ---
        {
            glm::vec2 pos2D(cubePos.x, cubePos.z);
            float height = getWaveHeight(pos2D, dirs, amps, wlens, speeds, steeps, t);
            glm::vec3 normal = getWaveNormal(pos2D, dirs, amps, wlens, speeds, steeps, t);

            glm::vec3 targetCenter = glm::vec3(cubePos.x, height, cubePos.z);
            glm::quat targetRot = glm::quat(glm::vec3(0,1,0), normal); // from up to normal

            // Smooth interpolation
            float factor = glm::clamp(12.0f * dt, 0.0f, 1.0f);
            smoothCubeCenter = glm::mix(smoothCubeCenter, targetCenter, factor);
            smoothCubeRot = glm::slerp(smoothCubeRot, targetRot, factor);
            smoothCubeRot = glm::normalize(smoothCubeRot);

            glm::mat4 cubeModel = glm::translate(glm::mat4(1.0f), smoothCubeCenter + glm::vec3(0, cubeScale.y * 0.5f, 0));
            cubeModel = cubeModel * glm::mat4_cast(smoothCubeRot);
            cubeModel = glm::scale(cubeModel, cubeScale);

            glUniformMatrix4fv(modelLoc,1,GL_FALSE,&cubeModel[0][0]);
            glBindVertexArray(cubeVAO);
            glDrawElements(GL_TRIANGLES,36,GL_UNSIGNED_INT,0);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteVertexArrays(1,&VAO); glDeleteBuffers(1,&VBO); glDeleteBuffers(1,&EBO);
    glDeleteVertexArrays(1,&cubeVAO); glDeleteBuffers(1,&cubeVBO); glDeleteBuffers(1,&cubeEBO);
    glDeleteProgram(program);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}