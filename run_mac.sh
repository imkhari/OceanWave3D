#!/bin/bash
set -euo pipefail
project_dir="$(cd "$(dirname "$0")" && pwd)"
build_dir="$project_dir/build-macos"
cmake -S "$project_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" -j "$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
cd "$build_dir"
exec ./OceanWave3D "$@"
