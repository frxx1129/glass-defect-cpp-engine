@echo off
chcp 65001 >nul
REM C++ 引擎一键构建（cl 直接编译，绕开 CMake 编译器识别卡死问题）
call F:\VS2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat
if %ERRORLEVEL% NEQ 0 (
    echo MSVC activation FAILED
    pause
    exit /b 1
)
cd /d %~dp0
if not exist build_new mkdir build_new
cl /nologo /std:c++20 /utf-8 /EHsc /O2 /DNDEBUG /I include /I third_party\nlohmann /I F:\opencv\opencv\build\include src\json_io.cpp src\preprocess.cpp src\line_merge.cpp src\classify.cpp src\static_artifact.cpp src\detector.cpp src\skew.cpp src\luminosity.cpp src\corner.cpp src\main.cpp /Fe:build_new\glass_engine.exe /link F:\opencv\opencv\build\x64\vc16\lib\opencv_world4100.lib
if %ERRORLEVEL% NEQ 0 (
    echo BUILD FAILED
    pause
    exit /b 1
)
echo.
echo BUILD OK: build_new\glass_engine.exe
pause
