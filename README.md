# glass-defect-cpp-engine

玻璃缺陷检测算法（检测 E 边缘异常 / B 崩边 / L 裂纹 / Q 缺角）的 C++ 实现，逐模块镜像 Python 生产算法（`image_processor_hough.py`）。设计定位为 Python 引擎的计算模块替代品（通信/调度仍由 Python 负责，见 `include/engine/detector.h` 注释）；**当前已完成独立运行与逐图对比验证，尚未接入生产链路**。

## 交接范围

本仓库保留 C++ 引擎的源码、公共头文件、构建脚本、对比验证脚本、Python 调用封装与第三方依赖（nlohmann/json）。

不纳入仓库的内容包括构建产物（`build/`、`build_new/`、`*.obj`、`*.exe`）、测试图片、产线 ROI 模板、运行日志和本地调试脚本。这些文件以本地工作区或产线机器为准。

## License

Python 引擎仓库（`glass-defect-algo`）采用 PolyForm Noncommercial License 1.0.0（仅允许非商业用途）。本仓库是独立编写的 C++ 实现，算法逻辑镜像自该 Python 项目；**本仓库自身的许可条款尚未确定**，使用与分发前请与仓库所有者确认。

## 运行环境

- Windows 产线主机：MSVC（Visual Studio Build Tools，`vcvars64.bat`）+ OpenCV 4.10（`opencv_world4100.lib`）
- Linux/WSL（Ubuntu）：g++ 13 + CMake 3.28 + OpenCV 4.6，已在本仓库复跑验证
- nlohmann/json（已内置于 `third_party/`，无需单独安装）

## 主要入口

- `src/main.cpp`：可执行入口，支持单图 / stdin JSON / 批量三种模式。
- `src/detector.cpp`：ROI 检测主流程（镜像 Python `process_roi_hough_based`），含 Hough、直线合并、B 过滤链与 B→L 重分类、E/Q 汇总、exclusion zones 与标注图输出。
- `src/line_merge.cpp`：直线合并（角度聚类、邻近聚类、fitLine、轴向锁定 + Canny 拟合（HORIZONTAL_LOCK_FIT）、Canny snap 细调、支持度过滤、重复线段去重、近竖直过近合并、交点裁剪、ENSURE_VERTICAL_PER_CLUSTER）。
- `src/skew.cpp`：E 型边缘异常检测。
- `src/corner.cpp`：Q 缺角检测（含平行四边形验证过滤）。
- `src/luminosity.cpp`：B 崩边亮度扫描。
- `src/classify.cpp`：历史遗留桩（**当前未被调用**；实际 B 过滤链/B→L 在 `detector.cpp` 内联实现，勿以本文件为准）。
- `src/static_artifact.cpp`：静态伪影抑制（跨帧，内存态；持久化状态文件待补）。
- `scripts/cpp_engine_wrapper.py`：Python 调用封装（已实测可用）。
- `scripts/batch_compare.py`：Python vs C++ 批量对比验证工具（已实测可用）。

## 运行文件清单

- 引擎入口：`src/main.cpp`
- 算法模块：`src/detector.cpp`、`src/line_merge.cpp`、`src/skew.cpp`、`src/corner.cpp`、`src/luminosity.cpp`、`src/classify.cpp`、`src/preprocess.cpp`、`src/static_artifact.cpp`
- 公共头文件：`include/engine/`（config / types / detector / line_merge / skew / corner / luminosity / classify / preprocess / static_artifact / json_io）
- JSON I/O：`src/json_io.cpp`
- 构建：`build.bat`（Windows，cl 直接编译，**已验证可用**）、`build.sh` + `CMakeLists.txt`（Windows/Git Bash 及 Linux/WSL 均可，Linux 路径**已验证可用**）
- 验证与调用：`scripts/batch_compare.py`（支持 Windows/WSL 路径与 `--dark` 暗场对比）、`scripts/compare_engine.py`、`scripts/cpp_engine_wrapper.py`
- 依赖：`third_party/nlohmann/json.hpp`
- 测试：`tests/engine_tests.cpp`（基础单测：配置解析/预处理/空输入安全；CMake 构建后 `ctest` **已验证通过**）
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

Linux / WSL（Ubuntu）下直接运行：

```bash
./build.sh
# 产物：build/glass_engine；单元测试：ctest --test-dir build --output-on-failure
```

`build.sh` 会自动查找系统 OpenCV 并生成 `build/glass_engine`。Windows Git Bash 也可调用同一脚本（需先在 vcvars64 激活后的 shell 中执行）；产线 Windows 推荐继续使用 `build.bat`。

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

对比工具 `scripts/batch_compare.py` 对同一图片分别运行 Python 与 C++，比对缺陷数量与类型@位置（数量不一致即 DIFF）。以下结果为本仓库开发期间实测（2026-08，对照修复后的本地 Python 参考，见下文“与 Python 引擎的关系”）：

| 数据集 | 配置 | ROI 模板 | 结果 |
|--------|------|---------|------|
| line3 original（139 图，含 cam-1/cam2/cam3/cam4 混入） | `config.yaml`（Line3） | `cam1_roi_averaged_by_group.json` | 139/139 全一致 |
| cam1（77 图） | `config.yaml`（Line3） | `cam1_roi_averaged_by_group.json` | 77/77 全一致 |
| line2 0119 original（45 图） | `config2.yaml`（Line2） | `roi_averaged_by_group_CORRECTED.json` | 45/45 全一致 |
| bugs（27 图，含 cam3 混入） | `config.yaml`（Line3） | `cam1_roi_averaged_by_group.json` | 27/27 全一致 |
| cam5（naobo_line2 instant_replay，等间隔抽 40 帧） | `config2.yaml`（Line2） | `cam5_roi_averaged_by_group.json`（cam=2） | 40/40 全一致 |
| 暗场模式冒烟（cam1 明场图 + `hough_inspector_dark_params`，等间隔抽 40 张） | `config.yaml` | `cam1_roi_averaged_by_group.json`（cam=4） | 40/40 全一致 |

合计明场基线 **288 张，DIFF=0**（本次在 Linux/WSL 构建上复跑）。批量对比时工具会对每张图深拷贝配置并重置 Python 全局缓存（`reset_global_caches`），避免 Python 参考的跨帧稳定器（栅栏/玻璃边界/理想竖直缓存）在 ROI_ID 未设置时跨图串扰导致结果不稳定。

待验证：真实暗场图片（当前暗场冒烟只能证明“暗场参数 + 暗场入口”在明场图上的 C++/Python 一致性，不能替代产线暗场图验证）。

调试编译宏：

| 宏 | 输出 |
|----|------|
| `-DCPP_DEBUG_MERGED` | merge 分组/结果（[CLUSTER] / [PROX] / [FIT] / [SNAP]） |
| `-DCPP_DEBUG_RAW` | Hough 原始线段 |
| `-DCPP_DEBUG_E` | E 检测中间结果 |
| `-DCPP_DEBUG_B` | B 亮度扫描中间结果 |
| `-DCPP_DEBUG_Q` | Q 角点/射线中间结果（[Q-CORNER] 等） |

## 与 Python 引擎的关系

- Python 引擎（`image_processor_hough.py`，约 6700 行）是生产参考实现；C++ 引擎逐模块镜像其算法与阈值。
- **Python 参考侧修复（仅本地验证基准，未改动 GitHub fork）**：`find_and_analyze_defects` 的 Q 检测（corner_contour 块）存在两处未定义名称（`_q_enabled_runtime`、`_angle_to_x_axis_deg`），被外层 `except: pass` 吞掉后 Q 检测从未真正执行（上游 zay002 原版与 fork 均如此）。本地验证基准补上这两处定义后，Python 才按算法意图输出 Q，C++ 的 Q 引擎与之逐图对齐。
- **Python 参考侧另发现一处同类死代码（未修复基准）**：`merge_lines_and_get_main_edges` 的“竖直线跨角度簇二次合并”块（`VERTICAL_ACROSS_ANGLE_MERGE_ENABLE`）在块内引用了定义在块之后的 `_angle_deg`，执行时 NameError 被外层 `except: pass` 吞掉，因此该功能在本地基准中从未实际运行。C++ 已移植该实现但按基准可观测行为保持关闭（`line_merge.cpp` 内 `vertical_across_angle_actually_runs_in_python=false`），上游修复该前向引用后应同步打开。
- C++ 侧对齐项（均镜像 Python 语义，非调阈值）：亮度扫描掩膜取整（np.int32 截断）、E 重叠合并含端点宽度、E_FROM_MAIN/主边屏蔽带近水平容差（HORIZONTAL_ANGLE_TOL_DEG 缺省 10，与轮廓路径的 v_tol 分离）、水平轴向锁定 Canny 拟合（HORIZONTAL_LOCK_FIT_*）、Q 主体聚类 X/Y 回退二分、射线求交 u 符号、角点 (竖直,水平) 顺序、三角形顶点/box_points 取整、平行四边形条带剔除（保留抗锯齿像素）、射线未命中哨兵判定、Q 单边命中分支全图亮度均值（替换原网格采样）、merge score/support 使用浮点原始长度和（此前 int 舍入会在暗场 0.8° 容差下造成 topN 差 1 条）、B 非 Q 交点附近过滤、B 重叠旋转矩形并查集合并、暗场跳过跨 ROI 共享竖直边预收集（镜像 `image_processor_hough_dark` 薄封装）、B 过滤链顺序与最近主边/角度修约、L 端点扫描带与近主边过滤、默认值逐键对齐、Line2/Line3 cam3 exclusion zones（边缘遮罩/主边裁剪/E 过滤）。
- **当前状态**：C++ 引擎仅完成独立运行与对比验证，未接入生产链路，也未用于生产替换。
- 已知未实现（当前测试数据上最终缺陷数量无差异）：`_detect_edge_notches`（Python 会生成原始 Q 再被后续长度/宽度过滤全部滤除）、`scan_edge_for_chipping_blocks`（配置默认关闭，测试数据 0 调用）、X 型缺陷（Python 硬编码 `disable_x_defects=True`）、跨帧栅栏/玻璃边界/理想竖直缓存（batch 验证每图重置缓存，产线连续帧行为未镜像）、理想竖直 Canny 列峰（全 288 张实测 `rois_added_ideal=0`）。
- 暗场：C++ 按请求 `mode=dark` 选择 `hough_inspector_dark_params`，并镜像 Python 暗场入口 `image_processor_hough_dark.py` 的行为（不预收集跨 ROI 共享竖直边）。`batch_compare.py --dark` 已修正为“Python 暗场入口 vs C++ 暗场参数”的同一口径；明场图暗场参数冒烟 40 张 DIFF=0，真实暗场图仍待产线数据。

## 目录维护约定

- 源码、头文件、构建脚本、验证脚本、依赖与文档进入 Git。
- 构建产物（`build/`、`build_new/`、`*.obj`、`*.exe`）、测试图片、日志与本地调试脚本不进入 Git。
- 产线 ROI 模板与硬件参数以产线机器当前文件为准。
