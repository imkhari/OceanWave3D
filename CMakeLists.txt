cmake_minimum_required(VERSION 3.10)
project(GerstnerOcean)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)

# you must install GLFW and GLM (or use submodules)
find_package(glfw3 3.3 REQUIRED)
find_package(glm REQUIRED)

# adjust include paths if you put glad.c locally
add_executable(GerstnerOcean main.cpp glad.c)

target_include_directories(GerstnerOcean PRIVATE ${GLFW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} .)
target_link_libraries(GerstnerOcean PRIVATE glfw OpenGL::GL)
