cmake_minimum_required(VERSION 3.20)
project(glass_defect_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 中文编码支持
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# nlohmann/json (header-only)
set(NLOHMANN_JSON_DIR "${CMAKE_SOURCE_DIR}/third_party/nlohmann")
if(NOT EXISTS "${NLOHMANN_JSON_DIR}/json.hpp")
    message(FATAL_ERROR "nlohmann/json not found at ${NLOHMANN_JSON_DIR}/json.hpp. Run build.sh first.")
endif()

# OpenCV 依赖
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

# 源文件
file(GLOB_RECURSE ENGINE_SRCS src/*.cpp)

# 主程序 - 独立可执行文件
add_executable(glass_engine ${ENGINE_SRCS})
target_include_directories(glass_engine PRIVATE
    include
    ${NLOHMANN_JSON_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(glass_engine PRIVATE ${OpenCV_LIBS})
