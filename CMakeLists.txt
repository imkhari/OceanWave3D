cmake_minimum_required(VERSION 3.10)
project(GerstnerOcean CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)

# you must install GLFW and GLM (or use submodules)


# adjust include paths if you put glad.c locally
add_executable(GerstnerOcean main.cpp)

target_include_directories(GerstnerOcean PRIVATE .)
target_link_libraries(GerstnerOcean PRIVATE
    glfw3::glfw
    glad::glad
    glm::glm
    OpenGL::GL
)
