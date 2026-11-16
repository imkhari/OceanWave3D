#!/bin/bash
set -e

# Create and enter build directory
mkdir -p build
cd build

# Configure and compile with CMake
echo "==> Configuring CMake..."
cmake ..

echo "==> Building OceanWave3D..."
cmake --build . -j$(sysctl -n hw.ncpu 2>/dev/null || echo 4)
mkdir -p assets
cp -r ../assets/* assets/
cp compile_commands.json ../compile_commands.json 2>/dev/null || true

echo "==> Build successful! Starting OceanWave3D..."
./OceanWave3D
