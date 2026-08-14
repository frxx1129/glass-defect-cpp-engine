# 玻璃缺陷检测 C++ 引擎（glass_engine）

面向玻璃产线的缺陷检测系统 **C++ 镜像实现**，与 Python 生产算法（`image_processor_hough.py`）逐模块对齐，可作为 Python 引擎的计算替代品（通信/调度仍由 Python 负责），也支持独立运行验证。

## 检测缺陷类型

| 类型 | 含义 |
|------|------|
| Q | 缺角（角点配对 + 射线求交 + 三角形判定 + 平行四边形验证过滤） |
| B | 崩边（主边亮度扫描 + MORPH_CLOSE 合并 + 过滤链） |
| E | 边缘异常（主边斜线直报 + 轮廓分析） |
| L | 裂纹（B→L 重分类：长宽比>7.5 且长边垂直主边，无独立检测） |
| X | 斜边（Python 侧已禁用，C++ 未实现） |

## 架构与模块映射

编译产物：静态库 `engine` + 可执行 `glass_engine`。核心流程：预处理 → Hough → 直线合并 → 缺陷分类。

| C++ 文件 | 镜像 Python 函数 | 功能 |
|---------|------------------|------|
| `preprocess.cpp` | `preprocess_for_hough_enhanced` / `preprocess_for_defect_edges` | 预处理（medianBlur + CLAHE + Canny） |
| `line_merge.cpp` | `merge_lines_and_get_main_edges` | 直线合并（角度聚类 + 邻近聚类 + fitLine + 轴向锁定 + 交点裁剪 + ENSURE_VERTICAL_PER_CLUSTER） |
| `luminosity.cpp` | `scan_edge_for_luminosity_defects` | B 崩边亮度扫描 |
| `skew.cpp` | E 检测（find_and_analyze_defects 内） | E 型边缘异常 |
| `corner.cpp` | `corner_contour_q_defects` | Q 缺角检测（含平行四边形验证过滤） |
| `classify.cpp` | B 过滤链 / L 重分类 | 缺陷分类与过滤 |
| `static_artifact.cpp` | `filter_static_artifact_defects` | 静态伪影抑制（跨帧） |
| `detector.cpp` | `process_roi_hough_based` | ROI 检测主流程 + draw_defects 标注图 |
| `json_io.cpp` | 配置/结果解析 | JSON I/O + Windows 中文路径 |

## 构建方法

Windows + MSVC（Visual Studio Build Tools），`build.bat` 用 `cl` 直接编译（绕开 CMake）：

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

依赖：OpenCV 4.10（`opencv_world4100.lib`）、nlohmann/json（已内置于 `third_party/`）、C++20。

> 注意：源文件列表共 10 个（json_io / preprocess / line_merge / classify / static_artifact / detector / skew / luminosity / corner / main），勿漏。

## 使用方式

三种运行模式（`main.cpp`）：

```bash
# 模式 1：单图（自动读同目录 config.json）
glass_engine image.png

# 模式 2：stdin JSON 请求
echo '{"image_path":"...","config":{...}}' | glass_engine

# 模式 3：批量（每行一个 JSON 请求，跨帧保持静态抑制状态）
glass_engine --batch < input.jsonl
```

请求 JSON 支持字段：`roi_template_file`、`system_params.pixels_per_mm`、`hough_inspector_params`（明场）/ `hough_inspector_dark_params`（暗场，需 `"mode":"dark"`）、`draw_defects` + `annotated_output_dir`（标注图输出）、`cam_key`、`static_artifact_enabled` 等。

Python 调用封装见 `scripts/cpp_engine_wrapper.py`（`CppEngine.detect / detect_batch`）。

### 标注图输出（draw_defects）

请求携带 `annotated_output_dir` 时输出 `<原文件名>_annotated.jpg`：Q 区域填充 + 射线箭头 + 玻璃轮廓黄线，L/B/E/X 框填充，文本标注（ASCII 标签）。镜像 Python `_draw_defect_annotations`。

### 调试编译宏

- `-DCPP_DEBUG_MERGED`：打印 merge 分组/结果（[CLUSTER]/[PROX]/[FIT]）
- `-DCPP_DEBUG_RAW`：打印 Hough 原始线段
- `-DCPP_DEBUG_E`：打印 E 检测中间结果

## 验证结果

对比工具：`scripts/batch_compare.py`（Python vs C++ 批量对比，比对缺陷数量）。

| 数据集 | ROI 模板 | 结果 |
|--------|---------|------|
| cam1（bugs 27 + cam1 部分，80 图） | `cam1_roi_averaged_by_group.json` | **80/80 全一致** |
| line3 original（130 图） | `roi_averaged_by_group_CORRECTED.json` | **129/130 一致**（剩余 1 张为 cam2 数据混入，需 cam2 ROI 模板单独验证；C++ 已报出匹配的 E，多出 1 个 E 源于 Canny snap 未镜像） |

待验证：line2、cam5、暗场模式（需对应 ROI 模板与测试数据）。

## 开发历史

| Commit | 阶段 |
|--------|------|
| `44d7f8d` | 重做：新架构（完整配置模型/多 ROI/核心检测链）+ 对比验证工具 |
| `871e7aa` | 第二阶段：E 型边缘异常 + 亮度扫描 + Hough 参数对齐 |
| `46a55f7` | 第三阶段：Q 缺角 + B 过滤链 + 主边交点对齐，cam1 80 图全一致 |
| `82c5ccb` | 第四阶段：line3 E 误报修复（merge 角度归一化 + ENSURE_VERTICAL_PER_CLUSTER + Hough 舍入对齐） |
| `1698982` | draw_defects 缺陷标注图输出 + json_io 解析修复 |
| `8dca48c` | 暗场模式选择（mode=dark） |
| `266bdb6` | merge 排序对齐 Python score + Q 平行四边形验证过滤 + 暗场标注 alpha 修复 |

## 目录结构

```
├── src/          源码（10 个模块 + main）
├── include/engine/ 公共头文件
├── scripts/      对比验证工具 / Python 调用封装
├── legacy/       旧版半成品（存档）
├── tests/        单元测试
├── third_party/  nlohmann/json
├── docs/         文档
├── build.bat     Windows 构建脚本（cl 直接编译）
└── build.sh      Linux 构建脚本
```

## 与 Python 引擎的关系

- Python 引擎是生产参考实现（约 6738 行，`image_processor_hough.py`），C++ 引擎逐模块镜像其算法
- 已知未镜像项：merge 的 Canny snap（`_snap_line_to_canny`）——影响主边位置 1~7px（计数验证不受影响）
- 配置两套：`hough_inspector_params`（明场，生产）/ `hough_inspector_dark_params`（暗场）
