#pragma once
#include <string>
#include <vector>
#include <unordered_map>

struct Point {
    int x = 0;
    int y = 0;
};

struct Defect {
    std::string type;       // B / L / Q / E / S
    double confidence = 0;
    double size_mm = 0;
    Point location;
    double width_mm = 0;
    double height_mm = 0;

    // 原始检测参数
    double angle_deg = 0;
    double length_mm = 0;
    double circularity = 0;
};

struct DetectionResult {
    std::string image_status = "OK";   // OK / NG
    std::vector<Defect> defects;
    double process_time_ms = 0;
    int total_defects = 0;
    int filtered_static = 0;
};

// 单个 ROI 区域
struct RoiRect {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// config from JSON
struct EngineConfig {
    // 图像
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;

    // 预处理
    int median_blur_ksize = 17;
    double clahe_clip_limit = 1.78;
    int clahe_grid_size_x = 8;
    int clahe_grid_size_y = 8;

    // Canny
    double canny_low = 35;
    double canny_high = 90;

    // Hough
    double hough_rho = 1.0;
    double hough_theta_deg = 1.0;
    int hough_threshold = 50;
    double hough_min_line_length = 30;
    double hough_min_line_length_ratio = 0.0;  // 0=未设置, >0则根据图像对角线计算
    double hough_max_line_gap = 10;

    // 缺陷分类阈值
    double min_width_mm = 3.0;
    double min_defect_size_mm = 5.0;
    double max_defect_size_mm = 20.0;
    double b_circularity_threshold = 0.85;
    double q_angle_threshold = 5.0;
    double l_min_length_mm = 10.0;

    // 像素/毫米 比例
    double px_per_mm = 10.0;

    // 直线合并（LINE_MERGING）
    double lm_angle_tolerance = 5.0;
    double lm_max_lateral_distance_mm = 5.0;
    int lm_max_lateral_distance = 40;
    int lm_top_n_edges = 6;
    bool lm_ensure_vertical_per_cluster = true;
    double lm_min_vertical_edge_gap_px = 6.0;
    double lm_vertical_thick_merge_mm = 10.0;
    double lm_vertical_thick_merge_max_mm = 20.0;
    double lm_vertical_thick_min_overlap_ratio = 0.05;
    bool lm_vertical_across_angle_merge_enable = true;
    double lm_dup_offset_px = 2.0;
    double lm_dup_angle_tol_deg = 3.0;

    // 裂纹分类（CRACK_CLASSIFICATION）
    double cc_endpoint_shield_ratio = 0.1;
    double cc_endpoint_proximity_threshold_mm = 5.49;

    // Q类型角点检测参数
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

    // E line suppression
    int e_line_suppress_width_px = 40;
    bool e_fast_prefilter_enable = true;
    double e_fast_prefilter_relax = 0.7;
    int e_fast_min_edge_pixels = 120;
    double e_fast_min_edge_ratio = 0.0008;
    double e_fast_roi_std_threshold = 10.0;

    // Luminosity 扫描检测
    double luminosity_scan_width_mm = 75.0;
    int luminosity_scan_width = 200;
    double luminosity_max_brightness_threshold = 66;
    double luminosity_std_dev_multiplier = 3.0;
    double luminosity_min_area_mm2 = 15;
    int luminosity_min_area = 30;
    double luminosity_edge_ignore_width_mm = 3.0;
    int luminosity_edge_ignore_width = 6;
    double luminosity_min_gradient = 20;

    // B/L 重分类参数
    double b_filter_distance_to_edge_max_mm = 5.0;
    double b_filter_parallel_tolerance_deg = 10.0;
    double b_filter_near_nonq_intersection_radius_mm = 20.0;
    double b_to_l_min_ar = 7.5;
    double b_to_l_perp_tolerance_deg = 10.0;
    double l_filter_max_distance_to_edge_mm = 15.0;
    double b_filter_parallel_ar_min = 10.0;
    double b_filter_parallel_min_side_mm = 2.0;
    bool reclassify_b_as_l_enable = true;
    double reclassify_b_as_l_min_ar = 3.2;
    double reclassify_b_as_l_max_distance_mm = 73.14;
    double reclassify_b_as_l_max_distance_px = 200;
    double reclassify_b_as_l_angle_tolerance = 15.0;
    double reclassify_b_as_l_endpoint_shield_ratio = 0.1;

    // 其他过滤参数
    double shadow_filter_min_extent_ratio = 0.25;
    double false_defect_max_width = 8.0;
    double false_defect_min_aspect_ratio = 5.0;
    double angle_deviation_tolerance = 15.0;
    double corner_max_physical_gap_mm = 5.0;
    int corner_max_physical_gap = 7;
    double corner_min_virtual_gap = 7.0;
    double corner_max_extension_normal_mm = 0.0;
    double corner_max_extension_perp_mm = 45;
    int corner_max_extension_perp = 200;
    double perpendicular_angle_tolerance = 10.0;
    double l_endpoint_belt_length_mm = 20.0;
    double chipping_endpoint_shield_radius_mm = 0.0;
    int chipping_endpoint_shield_radius = 0;
    double skew_curved_remove_line_thickness_px = 24;
    double skew_curved_min_arc_len_px = 700.0;
    double skew_curved_max_dev_ratio = 0.08;
    double prefilter_min_width_mm = 5.0;
    double vertical_angle_tol_deg = 15.0;
    double q_corner_contour_min_dist_px = 10.0;
    bool filter_log_enable = true;
    int filter_log_interval_s = 30;
    double b_max_distance_to_edge_mm = 5.0;
    double b_merge_min_side_mm = 2.0;
    double luminosity_endpoint_exclude_radius_mm = 6.0;

    // E 型边缘异常检测参数
    double e_min_area_mm2 = 4.0;
    double e_min_side_mm = 20.0;
    double e_border_touch_mm = 5.0;
    int e_canny_dilate_iter = 1;
    int e_canny_close_iter = 1;
    int e_canny_close_kernel_size = 5;
    int e_line_suppress_extra_px = 4;
    bool e_include_skew_main_edges = true;
    double e_from_main_edge_min_len_mm = 20.0;

    // ROI 配置
    std::string roi_file_path;  // ROI JSON 文件路径

    // 静态抑制
    bool static_artifact_enabled = true;
    int static_artifact_min_consecutive = 100;
    int static_artifact_grid_size = 60;
    int static_artifact_cooldown_frames = 500;
    int static_artifact_report_interval = 0;
};
