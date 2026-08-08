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
    double canny_low = 35.0;
    double canny_high = 90.0;
    // Hough 兜底增强（Python HOUGH_MIN_EDGE_RATIO 等键）
    double hough_min_edge_ratio = 0.002;      // 边缘占比低于此值触发兜底
    double hough_canny_fallback_scale = 0.75; // 降阈值缩放
    int hough_edge_dilate_iter = 1;           // 膨胀迭代次数
    std::vector<int> hough_edge_dilate_kernel = {3, 3};
};

struct HoughParams {
    int threshold = 35;
    double min_line_length = 30.0;      // 像素；未配置时用 MIN_LINE_LENGTH_RATIO
    double min_line_length_ratio = 0.0; // 相对对角线的比例（0=未设置）
    double max_line_gap_mm = 0.0;       // 毫米版（0=未设置）
    double max_line_gap = 30.0;         // 像素版
    double rho = 1.0;
    double theta_deg = 1.0;
};

struct LineMergingParams {
    double angle_tolerance = 5.0;
    double max_lateral_distance_mm = 5.0;
    int max_lateral_distance = 40;
    int top_n_edges = 6;
    bool ensure_vertical_per_cluster = true;
    double min_vertical_edge_gap_px = 6.0;
    double vertical_thick_merge_mm = 10.0;
    double vertical_thick_merge_max_mm = 20.0;
    double vertical_thick_min_overlap_ratio = 0.05;
    bool vertical_across_angle_merge_enable = true;
    double duplicate_merge_max_offset_px = 2.0;
    double duplicate_merge_angle_tol_deg = 3.0;
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
    // 基础阈值
    double min_defect_size_mm = 5.0;
    double min_width_mm = 5.0;
    double max_defect_size_mm = 20.0;
    double false_defect_max_width = 8.0;
    double false_defect_min_aspect_ratio = 5.0;
    double angle_deviation_tolerance = 15.0;

    // 缺陷合并
    std::vector<double> merge_defects_kernel_mm = {3.29, 3.29};
    std::vector<int> merge_defects_kernel_size = {9, 9};

    // Q 角点
    double q_defect_gradient_threshold = 5.0;
    int q_defect_search_width_px = 3;
    double q_triangle_brightness_margin = 10.0;
    int q_canny_stripe_half_width_px = 3;
    double q_triangle_min_area_mm2 = 15.0;
    int q_parallelogram_exclude_stripe_half_px = 10;
    double q_brightness_min_diff = 25.0;
    double q_length_min_mm = 5.0;
    double q_length_max_mm = 30.0;
    bool q_length_range_filter_enable = true;
    double q_corner_contour_min_dist_px = 10.0;
    double q_angle_threshold_deg = 5.0;

    // E line suppression + fast prefilter
    int e_line_suppress_width_px = 40;
    bool e_fast_prefilter_enable = true;
    double e_fast_prefilter_relax = 0.7;
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

    // 亮度扫描
    double luminosity_scan_width_mm = 75.0;
    int luminosity_scan_width = 200;
    double luminosity_max_brightness_threshold = 66.0;
    double luminosity_std_dev_multiplier = 3.0;
    double luminosity_min_area_mm2 = 15.0;
    int luminosity_min_area = 30;
    double luminosity_edge_ignore_width_mm = 3.0;
    int luminosity_edge_ignore_width = 6;
    double luminosity_min_gradient = 20.0;
    double luminosity_endpoint_exclude_radius_mm = 6.0;

    // chipping
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
    double vertical_angle_tol_deg = 15.0;
    double l_endpoint_belt_length_mm = 20.0;

    // 阴影 / B / L 过滤
    double shadow_filter_min_extent_ratio = 0.25;
    double prefilter_min_width_mm = 5.0;
    double b_max_distance_to_edge_mm = 5.0;
    double b_merge_min_side_mm = 2.0;
    double b_filter_distance_to_edge_max_mm = 5.0;
    double b_filter_parallel_tolerance_deg = 10.0;
    double b_filter_near_nonq_intersection_radius_mm = 20.0;
    double b_to_l_min_ar = 7.5;
    double b_to_l_perp_tolerance_deg = 10.0;
    double l_filter_max_distance_to_edge_mm = 15.0;
    double b_filter_parallel_ar_min = 10.0;
    double b_filter_parallel_min_side_mm = 2.0;
    double b_circularity_threshold = 0.85;

    // B -> L 重分类
    bool reclassify_b_as_l_enable = true;
    double reclassify_b_as_l_min_ar = 3.2;
    double reclassify_b_as_l_max_distance_mm = 73.14;
    int reclassify_b_as_l_max_distance_px = 200;
    double reclassify_b_as_l_angle_tolerance = 15.0;
    double reclassify_b_as_l_endpoint_shield_ratio = 0.1;

    // L / 裂纹分类
    double l_min_length_mm = 10.0;

    // 静态抑制
    bool static_artifact_enabled = true;
    int static_artifact_min_consecutive = 100;
    int static_artifact_grid_size = 60;
    int static_artifact_cooldown_frames = 500;
    int static_artifact_report_interval = 0;

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
};

// 引擎总配置
struct EngineConfig {
    double px_per_mm = 2.44;
    std::string line_name = "Line3";
    std::string storage_path = "inspection_results";

    InspectorParams light;    // hough_inspector_params
    InspectorParams dark;     // hough_inspector_dark_params

    // 运行期字段（由请求注入）
    int total_frames = 0;
    std::string cam_key = "cam0";       // 静态抑制按相机区分状态
    bool static_artifact_enabled = true;

    // 输出标注图开关/路径
    bool draw_defects = true;
    std::string annotated_output_dir;   // 空 = 不保存标注图
};
