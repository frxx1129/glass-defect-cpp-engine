#!/usr/bin/env bash
# C++ 引擎一键构建脚本
# 用法: bash build.bat  (Windows 下用 CMD 或 Git Bash 执行)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_DIR/build"
THIRD_PARTY_DIR="$PROJECT_DIR/third_party"

echo "=== C++ 引擎构建脚本 ==="
echo "项目目录: $PROJECT_DIR"

# 1. 下载 nlohmann/json (header-only)
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

# 2. 构建
echo "--- 配置 CMake ---"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 设置 MSVC 环境
MSVC_BAT="F:/VS2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat"
if [ -f "$MSVC_BAT" ]; then
    echo "调用 MSVC 环境: $MSVC_BAT"
    # 在 Windows 上需要先执行 vcvars64.bat
fi

cmake "$PROJECT_DIR" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$THIRD_PARTY_DIR" \
    -DCMAKE_CXX_FLAGS="/std:c++20 /utf-8"

echo "--- 构建 ---"
cmake --build . --config Release

echo "=== 构建完成 ==="
echo "可执行文件: $BUILD_DIR/Release/glass_engine.exe"
