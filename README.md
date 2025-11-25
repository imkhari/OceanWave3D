# OceanWave3D - Mô phỏng Sóng biển 3D

Một dự án đồ họa máy tính được viết bằng C++ và OpenGL, mô phỏng cảnh biển 3D chân thực với các yếu tố chính:

*   **Mặt nước động:** Sử dụng thuật toán sóng Gerstner để tạo ra các con sóng chân thực và sống động.
*   **Thuyền nổi:** Một chiếc thuyền đơn giản có thể điều khiển, tự động nổi và nghiêng theo chuyển động của sóng.
*   **Bầu trời hoàng hôn:** Bầu trời được vẽ theo thủ tục (procedural) với hiệu ứng chuyển màu của hoàng hôn, mặt trời, và các đám mây chuyển động.

Dự án này được chứa hoàn toàn trong một tệp `main.cpp` duy nhất để dễ dàng biên dịch và chạy.

## Công nghệ sử dụng

*   **Ngôn ngữ:** C++17
*   **API đồ họa:** OpenGL 3.3+
*   **Thư viện:**
    *   `GLAD`: để tải các hàm của OpenGL.
    *   `GLFW`: để tạo cửa sổ và xử lý input.
    *   `GLM`: cho các phép toán vector và ma trận.

## Biên dịch và Chạy (Windows với MSYS2/MinGW)

Dự án có thể được biên dịch bằng `g++`. Câu lệnh sau đây được cung cấp sẵn trong mã nguồn:

1.  **Cài đặt môi trường:** Đảm bảo bạn đã có môi trường MinGW (ví dụ: MSYS2) và đã cài đặt thư viện `glfw`.
2.  **Biên dịch:** Mở terminal và chạy lệnh sau (hãy điều chỉnh đường dẫn `-I` và `-L` nếu cần):
    ```bash
    g++ -std=c++17 -O2 main.cpp src\glad.c -I./include -I"D:\path\to\msys64\ucrt64\include" -L"D:\path\to\msys64\ucrt64\lib" -o main.exe -lglfw3 -lopengl32 -lgdi32 -limm32 -luser32 -lshell32 -lversion -lwinmm
    ```
3.  **Chạy:**
    ```bash
    ./main.exe
    ```

## Điều khiển

*   **Lái thuyền:**
    *   `W`: Tăng tốc
    *   `S`: Giảm tốc / Lùi
    *   `A`: Rẽ trái
    *   `D`: Rẽ phải
*   **Camera:**
    *   `Mũi tên Lên/Xuống`: Thay đổi độ cao của camera.
    *   `Mũi tên Trái/Phải`: Xoay camera quanh thuyền.
    *   `Q`: Phóng to (Zoom in)
    *   `E`: Thu nhỏ (Zoom out)
*   **Thoát:**
    *   `ESC`: Đóng ứng dụng.
