# OceanWave3D

A real-time 3D ocean and boat simulation project created using C++ and OpenGL. This application renders a dynamic ocean surface with Gerstner waves, a procedural skybox with a day-night cycle, and a playable boat. It also includes a simple game mode where the player collects floating cargo.

## Features

*   **Realistic Water Simulation**:
    *   Dynamic ocean surface generated using Gerstner waves.
    *   Adjustable wave height and ocean mesh resolution.
    *   Advanced water shading including Fresnel effects, specular highlights, and foam.
*   **Player-Controlled Boat**:
    *   A boat that realistically floats and tilts on the waves.
    *   Includes a flag that waves in the wind.
    *   Simple controls for acceleration, deceleration, and turning.
*   **Dynamic Procedural Sky**:
    *   A beautiful sunset skybox with procedurally generated, moving clouds.
    *   Adjustable time of day and sun position.
*   **Gameplay Mode**:
    *   A "Mission Control" mini-game to collect 10 floating crates within a time limit.
    *   UI displays game state, score, and a timer.
*   **In-Game GUI**:
    *   An ImGui-based control panel to tweak simulation parameters in real-time.
## Demo 
![Demo](ss/ocean.png)
## Controls

### Camera (Menu/Game Over Screen)
*   **Arrow Keys**: Rotate the camera around the boat.
*   **Q / E**: Zoom in / out.

### Boat (During Gameplay)
*   **W**: Accelerate forward.
*   **S**: Decelerate / Reverse.
*   **A**: Turn left.
*   **D**: Turn right.

## Building and Running

### Prerequisites
*   A C++ compiler that supports C++17 (e.g., MinGW g++ on Windows).
*   GLFW library.

### Build Steps

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/imkhari/OceanWave3D.git
    cd OceanWave3D
    ```

2.  **Configure Build Script:**
    Open the `build.bat` file and modify the `MINGW_PATH` variable to point to the `bin` directory of your MinGW/MSYS2 installation (e.g., `D:\msys64\ucrt64\bin`).

3.  **Run the Build Script:**
    Execute the `build.bat` script to compile the project.
    ```bash
    .\build.bat
    ```
    This will generate a `main.exe` file.

4.  **Run the Application:**
    ```bash
    .\main.exe
    ```

## Technical Overview

*   **Gerstner Waves**: The ocean surface is simulated by summing several Gerstner wave functions. This technique modifies vertex positions on a grid to create realistic wave shapes, including sharp crests. The boat and cargo float by calculating their vertical position based on these waves.
*   **Procedural Sky**: The sky is rendered on a full-screen quad. The fragment shader calculates the colors of the sky, sun, and clouds based on the view direction, time, and procedural noise functions (fbm).

## License

This project is licensed under the MIT License.
