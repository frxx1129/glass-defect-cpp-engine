cmake_minimum_required(VERSION 3.20)
project(glass_defect_engine VERSION 2.0.0 LANGUAGES CXX)

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

# 引擎静态库
file(GLOB_RECURSE ENGINE_SRCS src/*.cpp)
add_library(engine STATIC ${ENGINE_SRCS})
target_include_directories(engine PUBLIC include ${NLOHMANN_JSON_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(engine PUBLIC ${OpenCV_LIBS})

# 主程序
add_executable(glass_engine src/main.cpp)
target_link_libraries(glass_engine PRIVATE engine)

# 测试
enable_testing()
add_executable(engine_tests tests/engine_tests.cpp)
target_link_libraries(engine_tests PRIVATE engine)
add_test(NAME engine_tests COMMAND engine_tests)
