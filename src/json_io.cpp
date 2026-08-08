#include "json_io.h"
#include <iostream>
#include <fstream>

EngineConfig parse_config(const json& j) {
    EngineConfig cfg;

    // 读取 px_per_mm: 从 system_params.pixels_per_mm
    auto sys = j.value("system_params", json::object());
    cfg.px_per_mm = sys.value("pixels_per_mm", sys.value("PX_PER_MM", 10.0));

    // 所有检测参数都在 hough_inspector_params 下
    auto hip = j.value("hough_inspector_params", j);
    auto pp = hip.value("PREPROCESSING", json::object());
    auto dd = hip.value("DEFECT_DETECTION", json::object());
    auto ht = hip.value("HOUGH_TRANSFORM", json::object());

    // ROI
    auto roi = hip.value("ROI", json::object());
    cfg.roi_x = roi.value("x", 0);
    cfg.roi_y = roi.value("y", 0);
    cfg.roi_w = roi.value("width", 0);
    cfg.roi_h = roi.value("height", 0);

    // 预处理
    cfg.median_blur_ksize = pp.value("MEDIAN_BLUR_KSIZE", 17);
    cfg.clahe_clip_limit = pp.value("CLAHE_CLIP_LIMIT", 1.78);
    auto clahe_grid = pp.value("CLAHE_GRID_SIZE", json::array({8, 8}));
    if (clahe_grid.is_array() && clahe_grid.size() >= 2) {
        cfg.clahe_grid_size_x = clahe_grid[0].get<int>();
        cfg.clahe_grid_size_y = clahe_grid[1].get<int>();
    }

    // Canny
    cfg.canny_low = pp.value("CANNY_THRESHOLD_LOW", 35.0);
    cfg.canny_high = pp.value("CANNY_THRESHOLD_HIGH", 90.0);

    // Hough（Python 结构: hough_inspector_params.HOUGH_TRANSFORM）
    cfg.hough_rho = ht.value("RHO", 1.0);
    cfg.hough_theta_deg = ht.value("THETA_DEG", 1.0);
    cfg.hough_threshold = ht.value("THRESHOLD", 35);
    cfg.hough_min_line_length = ht.value("MIN_LINE_LENGTH", 30.0);
    cfg.hough_min_line_length_ratio = ht.value("MIN_LINE_LENGTH_RATIO", 0.0);
    cfg.hough_max_line_gap = ht.value("MAX_LINE_GAP", 10.0);

    // 直线合并（Python 结构: hough_inspector_params.LINE_MERGING）
    auto lm = hip.value("LINE_MERGING", json::object());
    cfg.lm_angle_tolerance = lm.value("ANGLE_TOLERANCE", 5.0);
    cfg.lm_max_lateral_distance_mm = lm.value("MAX_LATERAL_DISTANCE_MM", 5.0);
    cfg.lm_max_lateral_distance = lm.value("MAX_LATERAL_DISTANCE", 40);
    cfg.lm_top_n_edges = lm.value("TOP_N_EDGES", 6);
    cfg.lm_ensure_vertical_per_cluster = lm.value("ENSURE_VERTICAL_PER_CLUSTER", true);
    cfg.lm_min_vertical_edge_gap_px = lm.value("MIN_VERTICAL_EDGE_GAP_PX", 6.0);
    cfg.lm_vertical_thick_merge_mm = lm.value("VERTICAL_THICK_MERGE_MM", 10.0);
    cfg.lm_vertical_thick_merge_max_mm = lm.value("VERTICAL_THICK_MERGE_MAX_MM", 20.0);
    cfg.lm_vertical_thick_min_overlap_ratio = lm.value("VERTICAL_THICK_MIN_OVERLAP_RATIO", 0.05);
    cfg.lm_vertical_across_angle_merge_enable = lm.value("VERTICAL_ACROSS_ANGLE_MERGE_ENABLE", true);
    cfg.lm_dup_offset_px = lm.value("DUPLICATE_MERGE_MAX_OFFSET_PX", 2.0);
    cfg.lm_dup_angle_tol_deg = lm.value("DUPLICATE_MERGE_ANGLE_TOL_DEG", 3.0);

    // 裂纹分类（Python 结构: hough_inspector_params.CRACK_CLASSIFICATION）
    auto cc = hip.value("CRACK_CLASSIFICATION", json::object());
    cfg.cc_endpoint_shield_ratio = cc.value("ENDPOINT_SHIELD_RATIO", 0.1);
    cfg.cc_endpoint_proximity_threshold_mm = cc.value("ENDPOINT_PROXIMITY_THRESHOLD_MM", 5.49);

    // Q类型角点检测参数
    cfg.q_defect_gradient_threshold = dd.value("Q_DEFECT_GRADIENT_THRESHOLD", 5.0);
    cfg.q_defect_search_width_px = dd.value("Q_DEFECT_SEARCH_WIDTH_PX", 3);
    cfg.q_triangle_brightness_margin = dd.value("Q_TRIANGLE_BRIGHTNESS_MARGIN", 10.0);
    cfg.q_canny_stripe_half_width_px = dd.value("Q_CANNY_STRIPE_HALF_WIDTH_PX", 3);
    cfg.q_triangle_min_area_mm2 = dd.value("Q_TRIANGLE_MIN_AREA_MM2", 15.0);
    cfg.q_parallelogram_exclude_stripe_half_px = dd.value("Q_PARALLELOGRAM_EXCLUDE_STRIPE_HALF_PX", 10);
    cfg.q_brightness_min_diff = dd.value("Q_BRIGHTNESS_MIN_DIFF", 25.0);
    cfg.q_length_min_mm = dd.value("Q_LENGTH_MIN_MM", 5.0);
    cfg.q_length_max_mm = dd.value("Q_LENGTH_MAX_MM", 30.0);
    cfg.q_length_range_filter_enable = dd.value("Q_LENGTH_RANGE_FILTER_ENABLE", true);

    // E line suppression
    cfg.e_line_suppress_width_px = dd.value("E_LINE_SUPPRESS_WIDTH_PX", 40);
    cfg.e_fast_prefilter_enable = dd.value("E_FAST_PREFILTER_ENABLE", true);
    cfg.e_fast_prefilter_relax = dd.value("E_FAST_PREFILTER_RELAX", 0.7);
    cfg.e_fast_min_edge_pixels = dd.value("E_FAST_MIN_EDGE_PIXELS", 120);
    cfg.e_fast_min_edge_ratio = dd.value("E_FAST_MIN_EDGE_RATIO", 0.0008);
    cfg.e_fast_roi_std_threshold = dd.value("E_FAST_ROI_STD_THRESHOLD", 10.0);

    // Luminosity 扫描
    cfg.luminosity_scan_width_mm = dd.value("LUMINOSITY_SCAN_WIDTH_MM", 75.0);
    cfg.luminosity_scan_width = dd.value("LUMINOSITY_SCAN_WIDTH", 200);
    cfg.luminosity_max_brightness_threshold = dd.value("LUMINOSITY_MAX_BRIGHTNESS_THRESHOLD", 66);
    cfg.luminosity_std_dev_multiplier = dd.value("LUMINOSITY_STD_DEV_MULTIPLIER", 3.0);
    cfg.luminosity_min_area_mm2 = dd.value("LUMINOSITY_MIN_AREA_MM2", 15);
    cfg.luminosity_min_area = dd.value("LUMINOSITY_MIN_AREA", 30);
    cfg.luminosity_edge_ignore_width_mm = dd.value("LUMINOSITY_EDGE_IGNORE_WIDTH_MM", 3.0);
    cfg.luminosity_edge_ignore_width = dd.value("LUMINOSITY_EDGE_IGNORE_WIDTH", 6);
    cfg.luminosity_min_gradient = dd.value("LUMINOSITY_MIN_GRADIENT", 20);

    // B/L 重分类参数
    cfg.b_filter_distance_to_edge_max_mm = dd.value("B_FILTER_DISTANCE_TO_EDGE_MAX_MM", 5.0);
    cfg.b_filter_parallel_tolerance_deg = dd.value("B_FILTER_PARALLEL_TOLERANCE_DEG", 10.0);
    cfg.b_filter_near_nonq_intersection_radius_mm = dd.value("B_FILTER_NEAR_NONQ_INTERSECTION_RADIUS_MM", 20.0);
    cfg.b_to_l_min_ar = dd.value("B_TO_L_MIN_AR", 7.5);
    cfg.b_to_l_perp_tolerance_deg = dd.value("B_TO_L_PERP_TOLERANCE_DEG", 10.0);
    cfg.l_filter_max_distance_to_edge_mm = dd.value("L_FILTER_MAX_DISTANCE_TO_EDGE_MM", 15.0);
    cfg.b_filter_parallel_ar_min = dd.value("B_FILTER_PARALLEL_AR_MIN", 10.0);
    cfg.b_filter_parallel_min_side_mm = dd.value("B_FILTER_PARALLEL_MIN_SIDE_MM", 2.0);
    cfg.reclassify_b_as_l_enable = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("ENABLE", true);
    cfg.reclassify_b_as_l_min_ar = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("MIN_ASPECT_RATIO", 3.2);
    cfg.reclassify_b_as_l_max_distance_mm = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("MAX_DISTANCE_MM", 73.14);
    cfg.reclassify_b_as_l_max_distance_px = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("MAX_DISTANCE_PX", 200);
    cfg.reclassify_b_as_l_angle_tolerance = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("ANGLE_TOLERANCE", 15.0);
    cfg.reclassify_b_as_l_endpoint_shield_ratio = dd.value("RECLASSIFY_B_AS_L_PARAMS", json::object()).value("ENDPOINT_SHIELD_RATIO_FOR_EXTENDED", 0.1);

    // 其他过滤参数
    cfg.shadow_filter_min_extent_ratio = dd.value("SHADOW_FILTER_MIN_EXTENT_RATIO", 0.25);
    cfg.false_defect_max_width = dd.value("FALSE_DEFECT_MAX_WIDTH", 8.0);
    cfg.false_defect_min_aspect_ratio = dd.value("FALSE_DEFECT_MIN_ASPECT_RATIO", 5.0);
    cfg.angle_deviation_tolerance = dd.value("ANGLE_DEVIATION_TOLERANCE", 15.0);
    cfg.corner_max_physical_gap_mm = dd.value("CORNER_MAX_PHYSICAL_GAP_MM", 5.0);
    cfg.corner_max_physical_gap = dd.value("CORNER_MAX_PHYSICAL_GAP", 7);
    cfg.corner_min_virtual_gap = dd.value("CORNER_MIN_VIRTUAL_GAP", 7.0);
    cfg.corner_max_extension_normal_mm = dd.value("CORNER_MAX_EXTENSION_DIST_NORMAL_MM", 0.0);
    cfg.corner_max_extension_perp_mm = dd.value("CORNER_MAX_EXTENSION_DIST_PERPENDICULAR_MM", 45);
    cfg.corner_max_extension_perp = dd.value("CORNER_MAX_EXTENSION_DIST_PERPENDICULAR", 200);
    cfg.perpendicular_angle_tolerance = dd.value("PERPENDICULAR_ANGLE_TOLERANCE", 10);
    cfg.l_endpoint_belt_length_mm = dd.value("L_ENDPOINT_BELT_LENGTH_MM", 20.0);
    cfg.chipping_endpoint_shield_radius_mm = dd.value("CHIPPING_ENDPOINT_SHIELD_RADIUS_MM", 0.0);
    cfg.chipping_endpoint_shield_radius = dd.value("CHIPPING_ENDPOINT_SHIELD_RADIUS", 0);
    cfg.skew_curved_remove_line_thickness_px = dd.value("SKEW_CURVED_REMOVE_LINE_THICKNESS_PX", 24);
    cfg.skew_curved_min_arc_len_px = dd.value("SKEW_CURVED_MIN_ARC_LEN_PX", 700.0);
    cfg.skew_curved_max_dev_ratio = dd.value("SKEW_CURVED_MAX_DEV_RATIO", 0.08);
    cfg.prefilter_min_width_mm = dd.value("PREFILTER_MIN_WIDTH_MM", 5.0);
    cfg.vertical_angle_tol_deg = dd.value("VERTICAL_ANGLE_TOL_DEG", 15.0);
    cfg.q_corner_contour_min_dist_px = dd.value("Q_CORNER_CONTOUR_MIN_DIST_PX", 10.0);
    cfg.filter_log_enable = dd.value("FILTER_LOG_ENABLE", true);
    cfg.filter_log_interval_s = dd.value("FILTER_LOG_INTERVAL_S", 30);
    cfg.b_max_distance_to_edge_mm = dd.value("B_MAX_DISTANCE_TO_EDGE_MM", 5.0);
    cfg.b_merge_min_side_mm = dd.value("B_MERGE_MIN_SIDE_MM", 2.0);
    cfg.luminosity_endpoint_exclude_radius_mm = dd.value("LUMINOSITY_ENDPOINT_EXCLUDE_RADIUS_MM", 6.0);

    // 缺陷阈值
    cfg.min_width_mm = dd.value("MIN_WIDTH_MM", 3.0);
    cfg.min_defect_size_mm = dd.value("MIN_DEFECT_SIZE_MM", 5.0);
    cfg.max_defect_size_mm = dd.value("MAX_DEFECT_SIZE_MM", 20.0);
    cfg.b_circularity_threshold = dd.value("B_FILTER_CIRCULARITY_THRESHOLD", 0.85);
    cfg.q_angle_threshold = dd.value("Q_ANGLE_THRESHOLD_DEG", 5.0);
    cfg.l_min_length_mm = dd.value("L_MIN_LENGTH_MM", 10.0);
    // px_per_mm 已在上面从 system_params 读取

    // 静态抑制
    cfg.static_artifact_enabled = dd.value("STATIC_ARTIFACT_ENABLED", true);
    cfg.static_artifact_min_consecutive = dd.value("STATIC_ARTIFACT_MIN_CONSECUTIVE", 100);
    cfg.static_artifact_grid_size = dd.value("STATIC_ARTIFACT_GRID_SIZE", 60);
    cfg.static_artifact_cooldown_frames = dd.value("STATIC_ARTIFACT_COOLDOWN_FRAMES", 500);
    cfg.static_artifact_report_interval = dd.value("STATIC_ARTIFACT_REPORT_INTERVAL", 0);

    // E 型边缘异常检测参数
    cfg.e_min_area_mm2 = dd.value("E_MIN_AREA_MM2", 4.0);
    cfg.e_min_side_mm = dd.value("E_MIN_SIDE_MM", 20.0);
    cfg.e_border_touch_mm = dd.value("E_BORDER_TOUCH_MM", 5.0);
    cfg.e_canny_dilate_iter = dd.value("E_CANNY_DILATE_ITER", 1);
    cfg.e_canny_close_iter = dd.value("E_CANNY_CLOSE_ITER", 1);
    cfg.e_canny_close_kernel_size = dd.value("E_CANNY_CLOSE_KERNEL_SIZE", 5);
    cfg.e_line_suppress_extra_px = dd.value("E_LINE_SUPPRESS_EXTRA_PX_PER_DILATE_ITER", 4);
    cfg.e_include_skew_main_edges = dd.value("E_INCLUDE_SKEW_MAIN_EDGES", true);
    cfg.e_from_main_edge_min_len_mm = dd.value("E_FROM_MAIN_EDGE_MIN_LEN_MM", 20.0);

    // ROI 文件路径（顶层系统配置）
    cfg.roi_file_path = j.value("roi_template_file", "");

    return cfg;
}

json result_to_json(const DetectionResult& result) {
    json j;
    j["image_status"] = result.image_status;
    j["process_time_ms"] = result.process_time_ms;
    j["total_defects"] = result.total_defects;

    json defects_json = json::array();
    for (const auto& d : result.defects) {
        json dj;
        dj["type"] = d.type;
        dj["size_mm"] = d.size_mm;
        dj["width_mm"] = d.width_mm;
        dj["height_mm"] = d.height_mm;
        dj["length_mm"] = d.length_mm;
        dj["angle_deg"] = d.angle_deg;
        dj["location"] = {{"x", d.location.x}, {"y", d.location.y}};
        dj["confidence"] = d.confidence;
        defects_json.push_back(dj);
    }
    j["defects"] = defects_json;
    j["filtered_static"] = result.filtered_static;
    return j;
}

DetectionRequest parse_request(const std::string& json_str) {
    json j = json::parse(json_str);
    DetectionRequest req;
    req.image_path = j.value("image_path", "");
    req.total_frames = j.value("total_frames", 0);
    req.config = parse_config(j.value("config", json::object()));
    return req;
}

std::vector<RoiRect> load_rois_from_file(const std::string& file_path) {
    std::vector<RoiRect> rois;
    std::ifstream f(file_path);
    if (!f.is_open()) {
        return rois;
    }
    json j;
    f >> j;
    // 取 source_image_count 最大的 group
    std::string best_group;
    int max_count = -1;
    for (auto& [key, val] : j.items()) {
        int cnt = val.value("source_image_count", 0);
        if (cnt > max_count) {
            max_count = cnt;
            best_group = key;
        }
    }
    if (best_group.empty()) return rois;
    auto& avg_rois = j[best_group]["averaged_rois"];
    for (auto& r : avg_rois) {
        RoiRect roi;
        roi.x = r.value("x", 0);
        roi.y = r.value("y", 0);
        roi.width = r.value("width", 0);
        roi.height = r.value("height", 0);
        if (roi.width > 0 && roi.height > 0) {
            rois.push_back(roi);
        }
    }
    return rois;
}
