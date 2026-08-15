#!/usr/bin/env bash
# C++ 引擎 CMake 构建脚本
# - Linux / macOS：直接使用 g++/clang + OpenCV
# - Windows（Git Bash/msys）：推荐优先使用 build.bat（MSVC cl 直编，产线已验证）；
#   如需用 CMake，请在 vcvars64 激活后的 shell 中执行本脚本。
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_DIR/build"
THIRD_PARTY_DIR="$PROJECT_DIR/third_party"

echo "=== C++ 引擎构建脚本 ==="
echo "项目目录: $PROJECT_DIR"

# 1. 确保 nlohmann/json (header-only)
JSON_DIR="$THIRD_PARTY_DIR/nlohmann"
JSON_FILE="$JSON_DIR/json.hpp"
if [ ! -f "$JSON_FILE" ]; then
    echo "--- 下载 nlohmann/json ---"
    mkdir -p "$JSON_DIR"
    curl -sL "https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp" -o "$JSON_FILE"
    echo "    下载完成: $JSON_FILE"
else
    echo "--- nlohmann/json 已存在 ---"
fi

# 2. CMake 配置与构建
echo "--- 配置 CMake ---"
if [[ "${OS:-}" == "Windows_NT" || "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    echo "Windows 环境：使用 MSVC 生成器（需已激活 vcvars64 环境）"
    cmake "$PROJECT_DIR" -B "$BUILD_DIR" -A x64
    echo "--- 构建 ---"
    cmake --build "$BUILD_DIR" --config Release
else
    cmake "$PROJECT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
    echo "--- 构建 ---"
    nproc_bin="$(command -v nproc || true)"
    if [ -n "$nproc_bin" ]; then
        JOBS="$("$nproc_bin")"
    else
        JOBS=4
    fi
    cmake --build "$BUILD_DIR" -j "$JOBS"
fi

echo "=== 构建完成 ==="
if [[ "${OS:-}" == "Windows_NT" || "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    echo "可执行文件: $BUILD_DIR/Release/glass_engine.exe"
else
    echo "可执行文件: $BUILD_DIR/glass_engine"
fi
