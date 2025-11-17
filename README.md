# OceanWave3D

## Giới thiệu

OceanWave3D là một ứng dụng C++ sử dụng OpenGL để mô phỏng bề mặt đại dương với sóng Gerstner và một khối lập phương nổi trên mặt nước. Dự án này minh họa cách tạo hiệu ứng sóng nước động và tương tác vật thể với môi trường 3D.

## Hệ thống Build

Dự án này sử dụng **CMake** để quản lý quá trình build và **vcpkg** để quản lý các thư viện phụ thuộc. Đây là phương pháp hiện đại và được khuyến nghị để đảm bảo tính di động và dễ dàng tái tạo môi trường phát triển.

## Các thư viện phụ thuộc

Các thư viện chính được sử dụng trong dự án bao gồm:

*   **GLFW3**: Để tạo cửa sổ OpenGL và xử lý input.
*   **GLM (OpenGL Mathematics)**: Thư viện toán học cho OpenGL.
*   **GLAD**: Để tải các con trỏ hàm OpenGL.

Tất cả các thư viện này sẽ được vcpkg tự động tải xuống và cài đặt khi bạn cấu hình dự án bằng CMake.

## Cách Build và Chạy

Để build và chạy dự án này, bạn cần cài đặt **Git**, **CMake**, **MSYS2** (với `g++` và `pacman`) và **vcpkg**.

### 1. Cài đặt vcpkg

Nếu bạn chưa có vcpkg, hãy cài đặt nó bằng cách mở **PowerShell** và chạy các lệnh sau:

```powershell
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

### 2. Tích hợp vcpkg

Sau khi cài đặt vcpkg, hãy tích hợp nó với hệ thống của bạn:

```powershell
C:\vcpkg\vcpkg integrate install
```

### 3. Cấu hình và Build dự án

1.  **Di chuyển đến thư mục gốc của dự án:**
    ```powershell
    cd C:\Users\Admin\Projects\OceanWave3D
    ```

2.  **Tạo thư mục `build` và di chuyển vào đó:**
    ```powershell
mkdir build
cd build
    ```

3.  **Cấu hình CMake với vcpkg:**
    Lệnh này sẽ yêu cầu CMake cấu hình dự án và sử dụng vcpkg để quản lý các thư viện. vcpkg sẽ tự động tải xuống và cài đặt `glfw3`, `glm` và `glad` nếu chúng chưa có.

    ```powershell
    cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
    ```

4.  **Build dự án:**
    Sau khi CMake cấu hình xong, bạn hãy build dự án:

    ```powershell
    cmake --build .
    ```

### 4. Chạy ứng dụng

Sau khi build thành công, tệp thực thi (`.exe`) sẽ nằm trong thư mục `build` của bạn (hoặc trong `build/Debug` / `build/Release` tùy thuộc vào cấu hình build). Bạn chỉ cần mở tệp này để chạy ứng dụng.

## Lưu ý về Sublime Text Build System

Nếu bạn đang sử dụng Sublime Text, bạn có thể sử dụng cấu hình build sau (đã được cập nhật để tương thích với các thư viện đã cài đặt):

```json
{
  "shell_cmd": "g++ main.cpp -std=c++17 -O2 -o \"$file_path\$file_base_name.exe\" -I\"D:\\SETUP\\msys64\\ucrt64\\include\" -L\"D:\\SETUP\\msys64\\ucrt64\\lib\" -lglfw3 -lgdi32 -lopengl32 -lglu32 -g",
  "working_dir": "$project_path",
  "file_regex": "^(...*?):([0-9]+):?([0-9]+)?:? (.*)$",
  "env": {
    "PATH": "D:\\SETUP\\msys64\\ucrt64\\bin;%PATH%"
  },
  "variants": [
    {
      "name": "Build"
    },
    {
      "name": "Run",
      "shell_cmd": "cmd /c \"\"$file_path\$file_base_name.exe\" & pause\""
    },
    {
      "name": "Build & Run",
      "shell_cmd": "cmd /c \"g++ main.cpp -std=c++17 -O2 -o \"$file_path\$file_base_name.exe\" -I\"D:\\SETUP\\msys64\\ucrt64\\include\" -L\"D:\\SETUP\\msys64\\ucrt64\\lib\" -lglfw3 -lgdi32 -lopengl32 -lglu32 -g && \"$file_path\$file_base_name.exe\" & pause\""
    }
  ]
}
```
**Lưu ý:** Cấu hình Sublime Text này không sử dụng `vcpkg` mà dựa vào các thư viện được cài đặt toàn cục trong MSYS2 của bạn. Phương pháp CMake/vcpkg được khuyến nghị hơn.
