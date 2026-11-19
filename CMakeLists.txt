cmake_minimum_required(VERSION 3.16)
project(OceanWave3D VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include Homebrew paths for macOS (Apple Silicon & Intel)
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
    include_directories("/opt/homebrew/include" "/usr/local/include")
    link_directories("/opt/homebrew/lib" "/usr/local/lib")
endif()

# Find GLFW3
find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
    find_library(GLFW_LIB NAMES glfw glfw3 PATHS /opt/homebrew/lib /usr/local/lib)
endif()

# Find GLM
find_package(glm QUIET)
if(NOT glm_FOUND)
    find_path(GLM_INCLUDE_DIR glm/glm.hpp PATHS /opt/homebrew/include /usr/local/include include)
endif()

# Source files
set(CORE_SOURCES
    src/core/Window.cpp
    src/core/Camera.cpp
)

set(RENDERER_SOURCES
    src/renderer/Shader.cpp
    src/renderer/Mesh.cpp
    src/renderer/Sky.cpp
    src/renderer/ParticleSystem.cpp
    src/renderer/Framebuffer.cpp
)

set(OCEAN_SOURCES
    src/ocean/GerstnerWave.cpp
    src/ocean/OceanMesh.cpp
    src/ocean/FoamSimulator.cpp
)

set(AUDIO_SOURCES
    src/audio/AudioEngine.cpp
)

set(GAME_SOURCES
    src/game/Boat.cpp
    src/game/BoatPhysics.cpp
    src/game/Cargo.cpp
    src/game/Lighthouse.cpp
    src/game/Harbor.cpp
    src/game/Mainland.cpp
    src/game/Buoy.cpp
    src/game/Radar.cpp
    src/game/Wildlife.cpp
    src/game/Weather.cpp
    src/game/GameManager.cpp
)

set(IMGUI_SOURCES
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_impl_glfw.cpp
    imgui/imgui_impl_opengl3.cpp
)

set(GLAD_SOURCES
    src/glad.c
)

add_executable(OceanWave3D
    src/main.cpp
    ${CORE_SOURCES}
    ${RENDERER_SOURCES}
    ${OCEAN_SOURCES}
    ${GAME_SOURCES}
    ${AUDIO_SOURCES}
    ${IMGUI_SOURCES}
    ${GLAD_SOURCES}
)

target_compile_definitions(OceanWave3D PRIVATE GLM_ENABLE_EXPERIMENTAL)

target_include_directories(OceanWave3D PRIVATE
    include
    imgui
    src
)

if(GLM_INCLUDE_DIR)
    target_include_directories(OceanWave3D PRIVATE ${GLM_INCLUDE_DIR})
endif()

# Platform specific libraries
if(APPLE)
    target_link_libraries(OceanWave3D PRIVATE
        "-framework OpenGL"
        "-framework Cocoa"
        "-framework IOKit"
        "-framework CoreVideo"
        "-framework OpenAL"
    )
    if(TARGET glfw)
        target_link_libraries(OceanWave3D PRIVATE glfw)
    elseif(GLFW_LIB)
        target_link_libraries(OceanWave3D PRIVATE ${GLFW_LIB})
    else()
        target_link_libraries(OceanWave3D PRIVATE glfw)
    endif()
elseif(WIN32)
    target_link_libraries(OceanWave3D PRIVATE
        opengl32
        gdi32
        imm32
        user32
        shell32
        winmm
    )
    if(TARGET glfw)
        target_link_libraries(OceanWave3D PRIVATE glfw)
    else()
        target_link_libraries(OceanWave3D PRIVATE glfw3)
    endif()
else() # Linux
    find_package(OpenGL REQUIRED)
    target_link_libraries(OceanWave3D PRIVATE
        OpenGL::GL
        glfw
        pthread
        dl
    )
endif()

if(TARGET glm::glm)
    target_link_libraries(OceanWave3D PRIVATE glm::glm)
endif()

if(NOT APPLE)
    find_package(OpenAL REQUIRED)
    target_link_libraries(OceanWave3D PRIVATE OpenAL::OpenAL)
endif()

# Copy assets to build directory
add_custom_command(TARGET OceanWave3D POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/assets" "${CMAKE_CURRENT_BINARY_DIR}/assets"
    VERBATIM
)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME ocean_simulation_checks COMMAND OceanWave3D --self-test)
endif()
