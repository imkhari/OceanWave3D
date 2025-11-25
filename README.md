# OceanWave3D

A simple 3D scene of a boat on an ocean with a dynamic sky, rendered using OpenGL.

This project is a single-file C++ application that uses `glad`, `GLFW`, and `glm`.

## Features

- Gerstner waves for the ocean surface.
- A simple boat that floats and tilts with the waves.
- A dynamic sky with a sunset and procedural clouds.
- The shaders are embedded directly in the C++ source code for simplicity. The separate `.vert` and `.frag` files are provided for reference and easier editing.

## How to Build and Run

The project can be built with a C++ compiler like g++. The following command shows how to build it on Windows with MSYS2/MinGW:

```bash
g++ -std=c++17 -O2 .\main.cpp .\src\glad.c -I.\include -I"path\to\your\msys64\ucrt64\include" -L"path\to\your\msys64\ucrt64\lib" -o main.exe -lglfw3 -lopengl32 -lgdi32 -limm32 -luser32 -lshell32 -lversion -lwinmm -g
```

**Note:** You will need to replace `"path\to\your\msys64\ucrt64\include"` and `"path\to\your\msys64\ucrt64\lib"` with the actual paths to your MSYS2 installation. You also need to have GLFW installed.

After building, you can run the application by executing `main.exe`.

```bash
.\main.exe
```

## Controls

- **Arrow Keys (Up/Down/Left/Right):** Control the camera's pitch and yaw.
- **W/S Keys:** Zoom in and out.
- **Escape Key:** Close the application.

```