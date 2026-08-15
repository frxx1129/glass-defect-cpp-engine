#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>

// ============================================================
// 完整配置模型（对齐 Python config.yaml 的 hough_inspector_params
// 与 hough_inspector_dark_params 全部键）
// ============================================================

struct PreprocessParams {
    int median_blur_ksize = 17;
    double clahe_clip_limit = 1.78;
    int clahe_grid_x = 8;
    int clahe_grid_y = 8;
    // Python preprocess_for_hough_enhanced / preprocess_for_defect_edges：
    // CANNY_THRESHOLD_LOW 默认 20、CANNY_THRESHOLD_HIGH 默认 80
    double canny_low = 20.0;
    double canny_high = 80.0;
    // Hough 兜底增强（Python HOUGH_MIN_EDGE_RATIO 等键）
    double hough_min_edge_ratio = 0.002;      // 边缘占比低于此值触发兜底
    double hough_canny_fallback_scale = 0.75; // 降阈值缩放
    int hough_edge_dilate_iter = 1;           // 膨胀迭代次数
    std::vector<int> hough_edge_dilate_kernel = {3, 3};
};

struct HoughParams {
    // Python process_roi_hough_based：THRESHOLD 默认 50；长度用
    // MIN_LINE_LENGTH_MODE（默认 min）+ MIN_LINE_LENGTH_RATIO（默认 0.05）乘 ROI 基准。
    int threshold = 50;
    double min_line_length = 0.0;       // Python 当前算法不读该键（保留兼容）
    double min_line_length_ratio = 0.05;
    std::string min_line_length_mode = "min"; // min/height/diag/width
    double downsample_scale = 0.85;     // Python DOWNSAMPLE_SCALE 默认 0.85，越界回 0.85
    double max_line_gap_mm = 0.0;       // 毫米版（0=未设置）
    double max_line_gap = 0.0;          // 像素版；Python 两键均缺省时为 0
    double rho = 1.0;                   // Python 实际调用 HoughLinesP 硬编码 rho=1/theta=π/180
    double theta_deg = 1.0;
};

struct LineMergingParams {
    double angle_tolerance = 5.0;
    // Python _get_dist_px：MM 键存在则只用 MM*ppm；否则用 PX；均无默认按调用点。
    // 此结构体 0 表示"未配置"。
    double max_lateral_distance_mm = 0.0;
    int max_lateral_distance = 40;
    int top_n_edges = 6;
    bool ensure_vertical_per_cluster = false;   // Python 默认 False（生产 config 显式 true）
    bool ensure_vertical_presence = false;      // ENSURE_VERTICAL_PRESENCE 默认 False
    double min_vertical_edge_gap_px = 10.0;     // Python 默认 10（生产 config=6）
    double vertical_thick_merge_mm = 10.0;
    double vertical_thick_merge_scale = 1.5;    // VERTICAL_THICK_MERGE_SCALE 默认 1.5
    double vertical_thick_merge_max_mm = 30.0;  // Python 默认 30（生产 config=20）
    double vertical_thick_min_overlap_ratio = 0.2; // Python 默认 0.2（生产 config=0.05）
    bool vertical_across_angle_merge_enable = true;
    double merged_min_support_mm = 5.0;         // MERGED_MIN_SUPPORT_MM 默认 5
    int merged_min_support_px = 0;
    double glass_cluster_gap_mm = 40.0;         // GLASS_CLUSTER_GAP_MM 默认 40
    int glass_cluster_gap_px = 0;
    double duplicate_merge_max_offset_px = 2.0;
    double duplicate_merge_angle_tol_deg = 3.0;
    double horizontal_dup_merge_max_offset_mm = 2.0;
    double horizontal_dup_merge_max_offset_px = 4.0;
    double horizontal_dup_min_overlap_ratio = 0.2;
    bool vertical_lock_axis = true;
    bool horizontal_lock_axis = true;
    int canny_axis_refine_half_px = 3;
    // Canny snap（镜像 Python CANNY_SNAP_*，image_processor_hough.py:1079-1140）
    bool canny_snap_enable = true;
    int canny_snap_half_stripe_px = 4;
    int canny_snap_min_points = 12;
    double canny_snap_max_angle_diff_deg = 8.0;
    // 水平轴向锁定先做 Canny 拟合（镜像 Python HORIZONTAL_LOCK_FIT_*，image_processor_hough.py:1583-1600）
    bool horizontal_lock_fit_enable = true;
    int horizontal_lock_fit_half_px = 4;
    int horizontal_lock_fit_min_points = 18;
    // 0 = 跟随 h_tol（DEFECT_DETECTION.VERTICAL_ANGLE_TOL_DEG，Python 默认 h_tol_deg）
    double horizontal_lock_fit_max_angle_deg = 0.0;
    // 水平连接性校验（Python HORIZONTAL_CONNECTIVITY_ENABLE / HORIZ_CONNECT_*）
    bool horizontal_connectivity_enable = true;
    double horiz_connect_min_gap_px = 8.0;
    double horiz_connect_max_no_edge_run_px = 25.0;
    int horiz_connect_stripe_half_px = 2;
};

struct CrackClassifyParams {
    double endpoint_shield_ratio = 0.1;
    double endpoint_proximity_threshold_mm = 5.49;
    // 暗场特有
    bool parallel_crack_filter_enable = false;
    double parallel_crack_filter_angle_tolerance = 5.0;
    double parallel_crack_filter_max_distance_px = 10.0;
};

struct DefectDetectParams {
    // 基础阈值（Python 默认 MIN_DEFECT_SIZE_MM=3.0；生产 config=5.0）
    double min_defect_size_mm = 3.0;
    double min_width_mm = 5.0;
    double max_defect_size_mm = 20.0;
    double false_defect_max_width = 8.0;
    double false_defect_min_aspect_ratio = 5.0;
    double angle_deviation_tolerance = 15.0;

    // 缺陷合并
    // Python：MERGE_DEFECTS_KERNEL_MM 未设置时回退 MERGE_DEFECTS_KERNEL_SIZE（默认 [5,5]）
    std::vector<double> merge_defects_kernel_mm = {}; // 空 = 未设置（镜像 Python 回退语义）
    std::vector<int> merge_defects_kernel_size = {5, 5};

    // Q 角点
    double q_defect_gradient_threshold = 5.0;
    int q_defect_search_width_px = 3;
    double q_triangle_brightness_margin = 10.0;
    int q_canny_stripe_half_width_px = 2;
    double q_triangle_min_area_mm2 = 15.0;
    // Python _q_parallelogram_cluster_ok 默认 5；生产 config=10
    int q_parallelogram_exclude_stripe_half_px = 5;
    double q_brightness_min_diff = 25.0;
    double q_length_min_mm = 5.0;
    double q_length_max_mm = 30.0;
    // Python 键缺省时 bool(...) 得 False（生产 config=True）
    bool q_length_range_filter_enable = false;
    double q_corner_contour_min_dist_px = 16.0;  // Python 默认 16（生产 config=10）
    double q_angle_threshold_deg = 5.0;
    double q_max_side_mm = 0.0;                  // <=0 不限制
    double q_dedup_center_dist_px = 12.0;
    int q_parallelogram_min_edge_pixels = 50;
    double q_parallelogram_min_span_frac = 0.25;
    bool q_parallelogram_use_dilate = true;
    std::string q_tri_vertical_clip_side_mode = "vertical";
    double q_ray_inward_deg = 1.8;

    // E line suppression + fast prefilter
    // Python E_LINE_SUPPRESS_WIDTH_PX 默认 18（生产 config=40）
    int e_line_suppress_width_px = 18;
    bool e_fast_prefilter_enable = true;
    double e_fast_prefilter_relax = 1.0;         // Python 默认 1.0（生产 config=0.7）
    int e_fast_min_edge_pixels = 120;
    double e_fast_min_edge_ratio = 0.0008;
    double e_fast_roi_std_threshold = 10.0;

    // E 型边缘异常
    double e_min_area_mm2 = 4.0;
    double e_min_side_mm = 20.0;
    double e_border_touch_mm = 5.0;
    int e_canny_dilate_iter = 1;
    int e_canny_close_iter = 1;
    int e_canny_close_kernel_size = 5;
    int e_line_suppress_extra_px = 4;
    bool e_include_skew_main_edges = true;
    double e_from_main_edge_min_len_mm = 20.0;

    // 斜边（skew/curved）
    double skew_curved_remove_line_thickness_px = 24.0;
    double skew_curved_min_arc_len_px = 700.0;
    double skew_curved_max_dev_ratio = 0.08;

    // 亮度扫描（Python：mm/px 键均缺省时 scan_width=max(1,0)=1、min_area=0 不过滤面积）
    double luminosity_scan_width_mm = 0.0;
    int luminosity_scan_width = 0;
    double luminosity_max_brightness_threshold = 66.0;
    double luminosity_std_dev_multiplier = 3.0;
    double luminosity_min_area_mm2 = 0.0;
    int luminosity_min_area = 0;
    double luminosity_edge_ignore_width_mm = 3.0;
    int luminosity_edge_ignore_width = 6;
    // Python LUMINOSITY_MIN_GRADIENT 默认 15（生产 config=20）
    double luminosity_min_gradient = 15.0;
    // 0 = 未设置 → 默认 max(3, int(min(0.3*scan_width,15)))
    double luminosity_endpoint_exclude_radius_mm = 0.0;

    // chipping（Python 当前算法未读这些键，保留解析）
    double chipping_endpoint_shield_radius_mm = 0.0;
    int chipping_endpoint_shield_radius = 0;

    // 角点/交点过滤
    double corner_max_physical_gap_mm = 5.0;
    int corner_max_physical_gap = 7;
    double corner_min_virtual_gap = 7.0;
    double corner_max_extension_normal_mm = 0.0;
    double corner_max_extension_perp_mm = 45.0;
    int corner_max_extension_perp = 200;
    double perpendicular_angle_tolerance = 10.0;
    // Python VERTICAL_ANGLE_TOL_DEG 默认 10（生产 config=15）
    double vertical_angle_tol_deg = 10.0;
    // 0 = 键未设置。Python 各调用点默认不同：
    //   E 屏蔽带 / E_FROM_MAIN 用 10；其余（轮廓路径/配对/merge 的 h_tol）用 v_tol。
    double horizontal_angle_tol_deg = 0.0;
    double l_endpoint_belt_length_mm = 20.0;

    // 阴影 / B / L 过滤
    double shadow_filter_min_extent_ratio = 0.25;
    // 0 = 未设置 → Python 回退 MIN_WIDTH_MM → 3.0
    double prefilter_min_width_mm = 0.0;
    double b_max_distance_to_edge_mm = 5.0;
    double b_merge_min_side_mm = 2.0;
    double b_filter_distance_to_edge_max_mm = 5.0;
    double b_filter_parallel_tolerance_deg = 10.0;
    double b_filter_near_nonq_intersection_radius_mm = 20.0;
    double b_to_l_min_ar = 7.5;
    double b_to_l_perp_tolerance_deg = 10.0;
    double l_filter_max_distance_to_edge_mm = 5.0;  // Python 默认 5（config.h 原默认 15 已改）
    double b_filter_parallel_ar_min = 10.0;
    double b_filter_parallel_min_side_mm = 2.0;
    bool b_filter_circularity_enabled = false;      // Python 默认关
    double b_circularity_threshold = 0.85;

    // B -> L 重分类
    bool reclassify_b_as_l_enable = true;
    double reclassify_b_as_l_min_ar = 3.2;
    double reclassify_b_as_l_max_distance_mm = 73.14;
    int reclassify_b_as_l_max_distance_px = 200;
    double reclassify_b_as_l_angle_tolerance = 15.0;
    double reclassify_b_as_l_endpoint_shield_ratio = 0.1;

    // 跨 ROI 共享竖直边（镜像 Python CROSS_ROI_VERTICAL_*，image_processor_hough.py:6313-6541）
    bool cross_roi_vertical_enabled = true;
    double cross_roi_vertical_min_len_mm = 5.0;
    double cross_roi_vertical_cluster_xpx = 12.0;
    double cross_roi_vertical_slant_bias = 0.5;
    bool cross_roi_vertical_prefer_global = true;
    double cross_roi_vertical_replace_max_dist_mm = 10.0;
    double cross_roi_vertical_replace_min_y_overlap_ratio = 0.15;

    // L / 裂纹分类
    double l_min_length_mm = 10.0;

    // 静态抑制（Python STATIC_ARTIFACT_REPORT_INTERVAL 默认 500；生产 config=0）
    bool static_artifact_enabled = true;
    int static_artifact_min_consecutive = 100;
    int static_artifact_grid_size = 60;
    int static_artifact_cooldown_frames = 500;
    int static_artifact_report_interval = 500;

    // 日志
    bool filter_log_enable = true;
    int filter_log_interval_s = 30;
};

struct VisualizationParams {
    double defect_overlay_alpha = 0.25;
    double retreat_distance_threshold = 150.0;
    double edge_endpoint_fixed_length = 20.0;
};

// 单套检测参数（明场或暗场）
struct InspectorParams {
    PreprocessParams preprocessing;
    HoughParams hough;
    LineMergingParams line_merging;
    CrackClassifyParams crack_classify;
    DefectDetectParams defect_detection;
    VisualizationParams visualization;

    std::string roi_template_file;   // 该套参数使用的 ROI 模板文件
    std::string mode;                // "light" / "dark"

    // 运行期字段（镜像 Python _RUNTIME_LINE_NAME / _RUNTIME_CAM_INDEX，用于 exclusion zones）
    std::string runtime_line_name;
    int runtime_cam_index = -1;
};

// 引擎总配置
struct EngineConfig {
    // Python system_params.pixels_per_mm 默认 1.0
    double px_per_mm = 1.0;
    std::string line_name = "Line3";
    std::string storage_path = "inspection_results";

    InspectorParams light;    // hough_inspector_params
    InspectorParams dark;     // hough_inspector_dark_params

    // 运行期字段（由请求注入）
    int total_frames = 0;
    std::string cam_key = "cam0";       // 静态抑制按相机区分状态
    bool static_artifact_enabled = true;
    std::string mode = "light";         // "light"=hough_inspector_params（明场，生产） / "dark"=hough_inspector_dark_params（暗场）

    // 输出标注图开关/路径
    bool draw_defects = true;
    std::string annotated_output_dir;   // 空 = 不保存标注图
};
