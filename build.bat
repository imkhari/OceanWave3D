@echo off
setlocal

REM Add MinGW g++ to the PATH
set "MINGW_PATH=D:\SETUP\msys64\ucrt64\bin"
set "PATH=%MINGW_PATH%;%PATH%"

echo [INFO] Compiling project...

g++ -g main.cpp src/glad.c imgui/imgui.cpp imgui/imgui_draw.cpp imgui/imgui_tables.cpp imgui/imgui_widgets.cpp imgui/imgui_impl_glfw.cpp imgui/imgui_impl_opengl3.cpp -Iinclude -Iimgui -o main.exe -lglfw3 -lopengl32 -lgdi32 -limm32

if %errorlevel% == 0 (
    echo.
    echo [SUCCESS] Compilation finished. 'main.exe' is ready.
) else (
    echo.
    echo [ERROR] Compilation failed. Please check the messages above.
)

echo.
pause
