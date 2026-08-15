# C++ 引擎全盘复查指示（对照 Python 参考）

> 本文件用于让一个**全新对话**对 C++ 引擎做全盘复查：逐模块对照 Python 生产算法，找出所有功能差异/未镜像项/近似实现，并验证既有结果是否真实可靠。复查者无历史上下文，本文件自包含。

## 0. 一句话目标

确认 `F:\玻璃检测\cpp_engine` 的 C++ 引擎与 Python 参考 `image_processor_hough.py` 在**算法语义**上逐模块一致（阈值、取整、角度约定、过滤顺序、默认值），列出全部差异，能修的按"镜像 Python"原则修掉，不能修的如实记录。

## 1. 关键路径

| 项 | 路径 |
|----|------|
| C++ 引擎仓库 | `F:\玻璃检测\cpp_engine`（git remote: `git@github.com:frxx1129/glass-defect-cpp-engine.git`，main 分支） |
| C++ 源码 | `F:\玻璃检测\cpp_engine\src\*.cpp`（10 个：json_io/preprocess/line_merge/classify/static_artifact/detector/skew/luminosity/corner/main） |
| C++ 头文件 | `F:\玻璃检测\cpp_engine\include\engine\*.h` |
| **Python 参考** | `F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main\image_processor_hough.py`（约 6770 行，**本地验证基准，已含两处 Q 修复**） |
| Python 配置 | 同目录 `config.yaml`（Line3）、`config2.yaml`（Line2） |
| ROI 模板 | 同目录 `cam1_roi_averaged_by_group.json`、`cam5_roi_averaged_by_group.json`、`line3cam3_roi_averaged_by_group.json`、`roi_averaged_by_group_CORRECTED.json` |
| 测试图片 | `F:\玻璃检测\资料\inputs\inputs\`（cam1/line2/line3/bugs 子目录） |
| 验证脚本 | `F:\玻璃检测\cpp_engine\scripts\batch_compare.py` |
| Python 调用封装 | `F:\玻璃检测\cpp_engine\scripts\cpp_engine_wrapper.py` |
| 交接文档 | `C:\Users\Admin\玻璃检测C++引擎_项目交接文档.md` |
| 任务清单 | `C:\Users\Admin\下一步任务.md` |

⚠️ **Python 参考的 GitHub fork 与本地不同**：本地 `image_processor_hough.py` 比 fork（github.com/frxx1129/glass-defect-algo）多了：(a) 标注绘制重构（draw_defects 开关 + 独立 `_draw_defect_annotations` 重绘函数）；(b) 两处 Q 死代码修复（见 §3.2）。**复查以本地文件为准**，fork 未动。

## 2. 环境与运行

- 构建（Windows cmd，必须走 vcvars64）：
  ```bat
  call F:\VS2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat
  cd /d F:\玻璃检测\cpp_engine
  cl /nologo /std:c++20 /utf-8 /EHsc /O2 /DNDEBUG /I include /I third_party\nlohmann /I F:\opencv\opencv\build\include src\json_io.cpp src\preprocess.cpp src\line_merge.cpp src\classify.cpp src\static_artifact.cpp src\detector.cpp src\skew.cpp src\luminosity.cpp src\corner.cpp src\main.cpp /Fe:build_new\glass_engine.exe /link F:\opencv\opencv\build\x64\vc16\lib\opencv_world4100.lib
  ```
- Python 解释器：`C:\Users\Admin\AppData\Local\Programs\Python\Python312\python.exe`；**运行含中文路径的脚本前必须设 `$env:PYTHONUTF8="1"`**（否则 argv 中文乱码）。
- 单图调试（stdin JSON 请求 → release exe；`glass_engine_dbg.exe` 为带调试宏的构建）：
  ```python
  import json, yaml, subprocess
  cfg = yaml.safe_load(open(r'F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main\config.yaml', encoding='utf-8'))
  hip = dict(cfg['hough_inspector_params'])
  hip['_RUNTIME_LINE_NAME'] = cfg.get('lineName',''); hip['_RUNTIME_CAM_INDEX'] = 4
  req = {"image_path": r"<图片路径>", "mode":"light", "config": {
      "roi_template_file": r"<ROI模板路径>",
      "system_params": {"pixels_per_mm": cfg['system_params']['pixels_per_mm']},
      "hough_inspector_params": hip}}
  p = subprocess.run([r"F:\玻璃检测\cpp_engine\build_new\glass_engine_dbg.exe"],
                     input=json.dumps(req), capture_output=True, text=True,
                     encoding='utf-8', errors='replace', timeout=60)
  print(p.stdout)  # 结果 JSON；调试输出在 p.stderr
  ```

## 3. 当前状态快照（复查前先读，别推翻已验证结论）

### 3.1 验证结果（2026-08，全部 DIFF=0）

| 批次 | 配置 | ROI | 结果 |
|------|------|-----|------|
| line3（139 图，含 cam-1/cam2/cam3/cam4 混入） | config.yaml | cam1_roi_averaged_by_group.json | 139/139 |
| cam1（77 图） | config.yaml | cam1_roi_averaged_by_group.json | 77/77 |
| line2（45 图，0119\original） | config2.yaml | roi_averaged_by_group_CORRECTED.json | 45/45 |
| bugs（27 图，含 cam3 混入） | config.yaml | cam1_roi_averaged_by_group.json | 27/27 |
| **合计** | | | **288 张 DIFF=0** |

比对口径：`batch_compare.py` 逐图跑 Python 与 C++，比对缺陷**数量**（类型@位置用于定位）。batch 模式每图深拷贝配置 + `iph.reset_global_caches()`，消除 Python 跨图状态泄漏（见 §6.8）。

### 3.2 Python 参考的两处 Q 修复（本地基准独有）

Python 的 Q 检测（`find_and_analyze_defects` 内 corner_contour 块）原本引用两个**未定义名称**，NameError 被外层 `except: pass` 吞掉 → Q 检测从未执行（上游 zay002 原版与 fork 均如此）：
1. `_q_enabled_runtime`：块开头 `if not _q_enabled_runtime:` 未定义 → 整块跳过。修复：块前补 `try: _q_enabled_runtime = bool(params.get('DEFECT_DETECTION', {}).get('Q_ENABLED', True)) except: True`。
2. `_angle_to_x_axis_deg`：supplement 角点收集（`_is_vertical_aug`/`_is_horizontal_aug`）调用未定义 → 二次跳过。修复：块内补该函数（镜像 `_angle_to_x_axis_deg_lm` 语义）。

**C++ 的 Q 引擎是按"修复后 Python"对齐的**（即按算法意图）。复查时不要以"Python 本来不出 Q"为由认为 C++ 多报 Q。

### 3.3 已提交（HEAD=247cfcb，工作区干净）

- `112af77` 对齐修复后 Python 参考（9 文件，详见交接文档 §6 第六阶段 8 项）
- `247cfcb` README 更新

## 4. 模块映射表（复查主线）

| C++ 文件 | 镜像的 Python 函数（行号） | 状态 |
|----------|---------------------------|------|
| `preprocess.cpp` | `preprocess_for_hough_enhanced`(997)、`preprocess_for_defect_edges`(1060) | ✅ 已镜像（边缘图与 Python 逐像素一致验证过） |
| `line_merge.cpp` | `merge_lines_and_get_main_edges`(1301)、`_snap_line_to_canny`(1079)、`_fit_line_from_edges`(1143) | ✅ 已镜像（角度聚类/邻近聚类/fitLine/轴向锁定+Canny 拟合/Canny snap/交点裁剪/ENSURE_VERTICAL_PER_CLUSTER/topN score 排序） |
| `detector.cpp` | `process_roi_hough_based`(4679) 的 Hough 段 + B 扫描合并 + 竖直边延长 + 跨 ROI 共享竖直边注入 | ✅ 已镜像 |
| `luminosity.cpp` | `scan_edge_for_luminosity_defects`(616) | ✅ 已镜像（掩膜取整已对齐 np.int32 截断） |
| `classify.cpp` | B 过滤链 + B→L 重分类（find_and_analyze_defects 内 ~4333-4513） | ✅ 已镜像（复查确认过滤顺序） |
| `skew.cpp` | E 检测（find_and_analyze_defects 内 ~3000-3247，含 E_INCLUDE_SKEW_MAIN_EDGES + 轮廓 + 并查集合并） | ✅ 已镜像 |
| `corner.cpp` | Q 检测 corner_contour 块（3263-3803）+ `_q_parallelogram_cluster_ok`(3856) + 主体聚类 | ✅ 已镜像（见 §5 已知简化项） |
| `static_artifact.cpp` | `filter_static_artifact_defects`(6159) + 状态文件(6128-6149) | ✅ 已镜像 |
| `json_io.cpp` | 配置解析（对照 Python 各 params 读取点） | ✅ 已镜像主要键；**复查核对默认值** |
| `main.cpp` | 单图/stdin/批量入口 | ✅ |

## 5. 已知差异清单（复查时逐项确认/修复）

### 5.1 已知近似（已记录，当前数据无影响）

1. **Q 单边命中分支（corner.cpp ~500-560）**：当"竖直边射线未命中、水平边命中"时，Python 用**全图点积掩膜平均亮度**选暗侧端点（`image_processor_hough.py` 3575-3594），C++ 用**网格采样**（`step = roi_h/60`）近似。当前 288 张未触发差异；复查建议改为全图精确计算（注意性能与结果回归）。

### 5.2 未实现功能（Python 有、C++ 无；当前数据/配置下不触发，须确认并如实记录）

| Python 功能 | 位置 | C++ 状态 | 说明 |
|-------------|------|----------|------|
| 凹口缺陷 `_detect_edge_notches`(1192) → notch_defects(4509) | find_and_analyze_defects | ❌ 未实现 | 复查确认 Python 在测试数据上是否产出（当前 288 张两侧均无） |
| 块状崩边 `scan_edge_for_chipping_blocks`(730) | find_and_analyze_defects | ❌ 未实现 | 配置 `BLOCK_BASED_CHIPPING_ENABLED` 未设置（默认关）；确认 Python 默认关 |
| 不检测区域 `get_exclusion_zones`(102)/`apply_exclusion_zones_to_edges`(120) | process_roi_hough_based 4710-4718、5193-5195 | ❌ 未实现 | 已验证 Line3/cam4、Line2/cam1 返回空列表 → 当前数据无影响；**产线 Line2/Line3 的 cam3 有 zones，若需支持须实现** |
| 跨帧栅栏/玻璃边界 `_update_vertical_fences`(412)/`_update_glass_boundaries`(368) | 配对阶段 | ❌ 未实现 | 多帧稳定器，C++ 用帧内聚类替代；当前验证通过 |
| 理想竖直缓存（跨帧保留）`_roi_ideal_vertical_cache` | process_roi_hough_based | ❌ 未实现 | 同上 |
| X 型缺陷（虚拟角点）`corner_defects.append({"type":"X"...})`(4168) | find_and_analyze_defects | ❌ 未实现 | 当前数据两侧均未产出 X；若新数据出现须补 |

### 5.3 配置键差异（复查重点）

- 对照 `src/json_io.cpp` 解析的键 vs Python 各函数 `params.get(...)` 读取的键，逐一核对**默认值**一致（C++ 默认值在 `include/engine/config.h`）。
- 已知对齐项：CANNY_SNAP_*、HORIZONTAL_LOCK_FIT_*（0=跟随 h_tol）、CROSS_ROI_VERTICAL_*、E_LINE_SUPPRESS_*、E_FAST_*、Q_PARALLELOGRAM_*、LUMINOSITY_*、HORIZONTAL_ANGLE_TOL_DEG（默认 10，与 v_tol=15 分离，用于 E_FROM_MAIN）。
- 复查新增：是否有 Python 读取但 C++ 未解析、且生产配置里**非默认值**的键（如 GLASS_CLUSTER_GAP_MM、Q_MAX_SIDE_MM、Q_DEDUP_CENTER_DIST_PX 等，当前配置多为 None/默认）。

## 6. 历史踩坑清单（这些 bug 类别最容易复发，复查时重点扫）

1. **取整语义**：Python `np.int32`/`astype(int32)` 是**向零截断**，`np.round`/`int(round())` 是银行家舍入；C++ 用 `std::lround`（四舍五入）会差 1px → 掩膜/多边形/顶点差 1px → 阈值翻转。已修：luminosity 掩膜、corner 三角顶点。**复查所有 fillPoly/line/circle 点坐标的取整方式**。
2. **角度约定**：Python 角度 = `abs(atan2(dy,dx))` 折叠或 `atan2` +180 归一化到 [0,180)，消费方再折叠 [0,90]；不同函数默认容差不同（E_FROM_MAIN 用 10、轮廓路径用 v_tol=15）。**复查每个角度比较点的折叠/容差来源**。
3. **射线/线段求交**：C++ `ray_intersect_contour` 的 u 分母符号曾反（det 为负共轭）→ 命中边错误。**复查所有 t/u 求解公式与 Python 逐符号一致**。
4. **哨兵判定**：用 `dir.x != 0` 判断"未命中"会误伤垂直射线 (0,1)。**复查所有 (0,0) 哨兵检查是否同时看 x/y**。
5. **状态泄漏**：Python `process_image_from_memory_parallel` 会改共享 cfg 的运行时字段（`_RUNTIME_ROI_X/Y`）且跨帧缓存（栅栏/玻璃边界/理想竖直）在 ROI_ID 未设时跨图串扰 → batch 结果与单张不一致。**复查任何"逐图跑"场景都须深拷贝配置 + `reset_global_caches()`**。
6. **Python 死代码**：名称未定义被 `except: pass` 吞掉导致功能静默失效（Q 检测就是）。**复查 Python 侧"功能从未执行"的假设，确认其真实意图后再对齐 C++**。
7. **合并/去重的宽度语义**：`cv2.boundingRect` 宽度是**含端点**（x2-x1+1），1px 相接算重叠；C++ 曾用排他宽度导致 E 合并分 2 组。**复查所有 boundingRect/重叠判断的 +1**。
8. **中位数**：Python `np.median` 偶数个数取中间两数平均；C++ 用 `true_median`，勿用 `xs[n/2]` 下中位数。

## 7. 验证方法（复查流程）

1. **基线回归**（任何改动前后必跑，4 个批次全 DIFF=0 才算没改坏）：
   ```powershell
   $env:PYTHONUTF8="1"
   $py = "C:\Users\Admin\AppData\Local\Programs\Python\Python312\python.exe"
   # line3
   & $py F:\玻璃检测\cpp_engine\scripts\batch_compare.py --roi "F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main\cam1_roi_averaged_by_group.json" --dir "F:\玻璃检测\资料\inputs\inputs\line3" --config "F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main\config.yaml" --cam 4
   # cam1
   & $py ... --dir "F:\玻璃检测\资料\inputs\inputs\cam1" ...
   # line2
   & $py ... --roi "...\roi_averaged_by_group_CORRECTED.json" --dir "F:\玻璃检测\资料\inputs\inputs\line2" --config "...\config2.yaml" --cam 4
   # bugs
   & $py ... --dir "F:\玻璃检测\资料\inputs\inputs\bugs" ...
   ```
2. **单张定位**：DIFF 出现后，用 §2 的 stdin JSON 跑 debug exe（`glass_engine_dbg.exe`，编译时加 `-DCPP_DEBUG_MERGED/-DCPP_DEBUG_E/-DCPP_DEBUG_B/-DCPP_DEBUG_Q/-DCPP_DEBUG_RAW`），对比 Python 侧 monkeypatch（`iph.find_and_analyze_defects` / `iph.scan_edge_for_luminosity_defects` 等）打印中间量。
3. **模块级对照**：对每个模块，把 C++ 中间量（merged 线、阈值、轮廓）与 Python 逐项对比，找到第一个分叉点即根因。
4. **修复原则**：只做"镜像 Python 语义"的修改（取整、符号、顺序、默认值），**禁止调阈值/加魔数 workaround**。

## 8. 复查清单（输出物）

复查完成后输出一份差异清单，每条含：模块 / C++ 行为 / Python 行为 / 影响（当前数据是否触发）/ 处置（已修/记录待办/无需处理）。至少覆盖：

- [ ] 10 个源文件 × Python 对应函数逐项过一遍（§4 表）
- [ ] json_io 默认值 vs Python 默认值逐键核对（§5.3）
- [ ] 全部 fillPoly/line/circle/顶点坐标取整方式（§6.1）
- [ ] 全部角度比较的折叠与容差来源（§6.2）
- [ ] 全部求交 t/u 公式符号（§6.3）
- [ ] 全部哨兵判定（§6.4）
- [ ] 全部 boundingRect/重叠宽度 +1（§6.7）
- [ ] 全部中位数实现（§6.8）
- [ ] §5.2 未实现功能在测试数据上的触发确认（Python 侧实测输出）
- [ ] §5.1 单边命中近似的精确化评估（改 vs 不改 + 回归）
- [ ] 既有 288 张基线复跑确认（若你改了任何代码）
- [ ] 新数据（若用户提供 cam5/暗场/cam2 资产）批次验证

## 9. 受阻批次（资产到位后再验）

- **cam5**：ROI 模板已有（`cam5_roi_averaged_by_group.json`），cam5 相机索引 = **2**（config.yaml `camera_rois`：0=cam1, 1=cam2, 2=cam5, 3=cam4, 4=cam3）；需产线 cam5 测试图。
- **暗场**：`mode="dark"` + `hough_inspector_dark_params`；需暗场图 + 暗场 ROI。
- **cam2 专属**：需 `cam2_roi_averaged_by_group.json`（产线机 `C:\line3`），cam2 索引 = 1。

## 10. 约束与验收

- 复用并保持 `batch_compare.py` 的用法（--roi/--dir/--config/--cam/--dark/--exe/--pat）。
- 修复提交到 `F:\玻璃检测\cpp_engine`（git），提交信息说明改了什么、为什么（对齐 Python 哪一处）。
- README 与交接文档（`C:\Users\Admin\玻璃检测C++引擎_项目交接文档.md`）如有行为变化须同步更新，保持实事求是（已测/未测、已实现/未实现严格区分）。
- 调试输出用 `CPP_DEBUG_*` 宏保护，不留裸 print。
