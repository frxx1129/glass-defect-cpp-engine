# glass-defect-cpp-engine

玻璃缺陷检测算法（检测 E 边缘异常 / B 崩边 / L 裂纹 / Q 缺角）的 C++ 实现，逐模块镜像 Python 生产算法（`image_processor_hough.py`）。设计定位为 Python 引擎的计算模块替代品（通信/调度仍由 Python 负责，见 `include/engine/detector.h` 注释）；**当前已完成独立运行与逐图对比验证，尚未接入生产链路**。

## 交接范围

本仓库保留 C++ 引擎的源码、公共头文件、构建脚本、对比验证脚本、Python 调用封装与第三方依赖（nlohmann/json）。

不纳入仓库的内容包括构建产物（`build/`、`build_new/`、`*.obj`、`*.exe`）、测试图片、产线 ROI 模板、运行日志和本地调试脚本。这些文件以本地工作区或产线机器为准。

## License

Python 引擎仓库（`glass-defect-algo`）采用 PolyForm Noncommercial License 1.0.0（仅允许非商业用途）。本仓库是独立编写的 C++ 实现，算法逻辑镜像自该 Python 项目；**本仓库自身的许可条款尚未确定**，使用与分发前请与仓库所有者确认。

## 运行环境

- Windows 产线主机
- MSVC（Visual Studio Build Tools，`vcvars64.bat`）
- OpenCV 4.10（`opencv_world4100.lib`）
- nlohmann/json（已内置于 `third_party/`，无需单独安装）

## 主要入口

- `src/main.cpp`：可执行入口，支持单图 / stdin JSON / 批量三种模式。
- `src/detector.cpp`：ROI 检测主流程（镜像 Python `process_roi_hough_based`），含 Hough、直线合并、缺陷分类与标注图输出。
- `src/line_merge.cpp`：直线合并（角度聚类、邻近聚类、fitLine、轴向锁定、交点裁剪、ENSURE_VERTICAL_PER_CLUSTER）。
- `src/skew.cpp`：E 型边缘异常检测。
- `src/corner.cpp`：Q 缺角检测（含平行四边形验证过滤）。
- `src/luminosity.cpp`：B 崩边亮度扫描。
- `src/classify.cpp`：B 过滤链与 B→L 重分类。
- `src/static_artifact.cpp`：静态伪影抑制（跨帧）。
- `scripts/cpp_engine_wrapper.py`：Python 调用封装（已实测可用）。
- `scripts/batch_compare.py`：Python vs C++ 批量对比验证工具（已实测可用）。

## 运行文件清单

- 引擎入口：`src/main.cpp`
- 算法模块：`src/detector.cpp`、`src/line_merge.cpp`、`src/skew.cpp`、`src/corner.cpp`、`src/luminosity.cpp`、`src/classify.cpp`、`src/preprocess.cpp`、`src/static_artifact.cpp`
- 公共头文件：`include/engine/`（config / types / detector / line_merge / skew / corner / luminosity / classify / preprocess / static_artifact / json_io）
- JSON I/O：`src/json_io.cpp`
- 构建：`build.bat`（Windows，cl 直接编译，**已验证可用**）、`build.sh`（Git Bash 下调用 MSVC 的 CMake 脚本，**未验证**）、`CMakeLists.txt`（**未验证**，此前 CMake 方式曾遇到编译器识别问题）
- 验证与调用：`scripts/batch_compare.py`、`scripts/compare_engine.py`、`scripts/cpp_engine_wrapper.py`
- 依赖：`third_party/nlohmann/json.hpp`
- 测试：`tests/engine_tests.cpp`（基础单测源码：配置解析/预处理/空输入安全；未接入 build.bat 主构建，未验证）
- 存档：`legacy/`（旧版半成品）

## 构建方法（已验证路径）

Windows 下在 `cmd` 执行（`build.bat` 用 `cl` 直接编译，绕开 CMake）：

```bat
call F:\VS2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat
cl /nologo /std:c++20 /utf-8 /EHsc /O2 /DNDEBUG ^
  /I include /I third_party\nlohmann /I F:\opencv\opencv\build\include ^
  src\json_io.cpp src\preprocess.cpp src\line_merge.cpp src\classify.cpp ^
  src\static_artifact.cpp src\detector.cpp src\skew.cpp src\luminosity.cpp ^
  src\corner.cpp src\main.cpp ^
  /Fe:build_new\glass_engine.exe ^
  /link F:\opencv\opencv\build\x64\vc16\lib\opencv_world4100.lib
```

要点：

- 必须先跑 `vcvars64.bat`（PowerShell 直接执行 .bat 不保留环境变量）。
- 源文件列表共 10 个（json_io / preprocess / line_merge / classify / static_artifact / detector / skew / luminosity / corner / main），勿漏。
- 产物为 `build_new\glass_engine.exe`，构建产物不进入 Git。

## 使用方式

```bash
# 模式 1：单图（自动读同目录 config.json）
glass_engine image.png

# 模式 2：stdin JSON 请求
echo '{"image_path":"...","config":{...}}' | glass_engine

# 模式 3：批量（每行一个 JSON 请求，跨帧保持静态抑制状态）
glass_engine --batch < input.jsonl
```

请求 JSON 支持字段：

- `roi_template_file`：ROI 模板文件。
- `system_params.pixels_per_mm`：像素密度。
- `hough_inspector_params`：明场算法参数；暗场用 `hough_inspector_dark_params` 并设置 `"mode":"dark"`。
- `draw_defects` + `annotated_output_dir`：输出标注图（`<原文件名>_annotated.jpg`）。
- `cam_key`、`static_artifact_enabled`：静态抑制按相机隔离的状态开关。

标注图内容：Q 区域填充 + 射线箭头 + 玻璃轮廓黄线，L/B/E/X 框填充 + 轮廓，文本标注（ASCII 标签）。镜像 Python `_draw_defect_annotations`（过滤后重绘，只标注保留的缺陷）。

## 验证

对比工具 `scripts/batch_compare.py` 对同一图片分别运行 Python 与 C++，比对缺陷数量与类型@位置。以下结果为本仓库开发期间实测（2026-08）：

| 数据集 | ROI 模板 | 结果 |
|--------|---------|------|
| cam1（bugs 27 + cam1 部分，80 图） | `cam1_roi_averaged_by_group.json` | 80/80 全一致 |
| line3 original（130 图） | `roi_averaged_by_group_CORRECTED.json` | 129/130 一致 |

line3 剩余 1 张为 cam2 数据混入（cam2 图使用 cam1 ROI 验证）：C++ 已报出与 Python 匹配的缺陷，多出 1 个 E 源于 Canny snap 尚未镜像，需 cam2 ROI 模板单独验证。

待验证：line2、cam5、暗场模式（需对应 ROI 模板与测试数据，以现场机器为准）。

调试编译宏：

| 宏 | 输出 |
|----|------|
| `-DCPP_DEBUG_MERGED` | merge 分组/结果（[CLUSTER] / [PROX] / [FIT]） |
| `-DCPP_DEBUG_RAW` | Hough 原始线段 |
| `-DCPP_DEBUG_E` | E 检测中间结果 |

## 与 Python 引擎的关系

- Python 引擎（`image_processor_hough.py`，约 6700 行）是生产参考实现；C++ 引擎逐模块镜像其算法与阈值。
- **当前状态**：C++ 引擎仅完成独立运行与对比验证，未接入生产链路，也未用于生产替换。
- 已知未镜像项：merge 的 Canny snap（`_snap_line_to_canny`），影响主边位置 1~7px，计数验证不受影响。
- 配置两套：`hough_inspector_params`（明场，生产使用）/ `hough_inspector_dark_params`（暗场，冒烟测试通过，无真实暗场数据验证）。

## 目录维护约定

- 源码、头文件、构建脚本、验证脚本、依赖与文档进入 Git。
- 构建产物（`build/`、`build_new/`、`*.obj`、`*.exe`）、测试图片、日志与本地调试脚本不进入 Git。
- 产线 ROI 模板与硬件参数以产线机器当前文件为准。
