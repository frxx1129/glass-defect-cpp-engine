#include "engine/json_io.h"
#include <fstream>

// ---------- 内部辅助 ----------
static double get_d(const json& j, const char* key, double def) {
    return j.contains(key) ? j[key].get<double>() : def;
}
static int get_i(const json& j, const char* key, int def) {
    return j.contains(key) ? j[key].get<int>() : def;
}
static bool get_b(const json& j, const char* key, bool def) {
    return j.contains(key) ? j[key].get<bool>() : def;
}
static std::string get_s(const json& j, const char* key, const std::string& def) {
    return j.contains(key) ? j[key].get<std::string>() : def;
}

// ---------- 各段解析 ----------
static PreprocessParams parse_preprocessing(const json& pp) {
    PreprocessParams p;
    p.median_blur_ksize = get_i(pp, "MEDIAN_BLUR_KSIZE", 17);
    p.clahe_clip_limit = get_d(pp, "CLAHE_CLIP_LIMIT", 1.78);
    if (pp.contains("CLAHE_GRID_SIZE") && pp["CLAHE_GRID_SIZE"].is_array()) {
        auto& g = pp["CLAHE_GRID_SIZE"];
        if (g.size() >= 2) { p.clahe_grid_x = g[0].get<int>(); p.clahe_grid_y = g[1].get<int>(); }
    }
    p.canny_low = get_d(pp, "CANNY_THRESHOLD_LOW", 35.0);
    p.canny_high = get_d(pp, "CANNY_THRESHOLD_HIGH", 90.0);
    return p;
}

static HoughParams parse_hough(const json& ht) {
    HoughParams h;
    h.threshold = get_i(ht, "THRESHOLD", 35);
    h.min_line_length = get_d(ht, "MIN_LINE_LENGTH", 30.0);
    h.min_line_length_ratio = get_d(ht, "MIN_LINE_LENGTH_RATIO", 0.0);
    h.max_line_gap_mm = get_d(ht, "MAX_LINE_GAP_MM", 0.0);
    h.max_line_gap = get_d(ht, "MAX_LINE_GAP", 30.0);
    h.rho = get_d(ht, "RHO", 1.0);
    h.theta_deg = get_d(ht, "THETA_DEG", 1.0);
    return h;
}

static LineMergingParams parse_line_merging(const json& lm) {
    LineMergingParams m;
    m.angle_tolerance = get_d(lm, "ANGLE_TOLERANCE", 5.0);
    m.max_lateral_distance_mm = get_d(lm, "MAX_LATERAL_DISTANCE_MM", 5.0);
    m.max_lateral_distance = get_i(lm, "MAX_LATERAL_DISTANCE", 40);
    m.top_n_edges = get_i(lm, "TOP_N_EDGES", 6);
    m.ensure_vertical_per_cluster = get_b(lm, "ENSURE_VERTICAL_PER_CLUSTER", true);
    m.min_vertical_edge_gap_px = get_d(lm, "MIN_VERTICAL_EDGE_GAP_PX", 6.0);
    m.vertical_thick_merge_mm = get_d(lm, "VERTICAL_THICK_MERGE_MM", 10.0);
    m.vertical_thick_merge_max_mm = get_d(lm, "VERTICAL_THICK_MERGE_MAX_MM", 20.0);
    m.vertical_thick_min_overlap_ratio = get_d(lm, "VERTICAL_THICK_MIN_OVERLAP_RATIO", 0.05);
    m.vertical_across_angle_merge_enable = get_b(lm, "VERTICAL_ACROSS_ANGLE_MERGE_ENABLE", true);
    m.duplicate_merge_max_offset_px = get_d(lm, "DUPLICATE_MERGE_MAX_OFFSET_PX", 2.0);
    m.duplicate_merge_angle_tol_deg = get_d(lm, "DUPLICATE_MERGE_ANGLE_TOL_DEG", 3.0);
    return m;
}

static CrackClassifyParams parse_crack_classify(const json& cc) {
    CrackClassifyParams c;
    c.endpoint_shield_ratio = get_d(cc, "ENDPOINT_SHIELD_RATIO", 0.1);
    c.endpoint_proximity_threshold_mm = get_d(cc, "ENDPOINT_PROXIMITY_THRESHOLD_MM", 5.49);
    c.parallel_crack_filter_enable = get_b(cc, "PARALLEL_CRACK_FILTER_ENABLE", false);
    c.parallel_crack_filter_angle_tolerance = get_d(cc, "PARALLEL_CRACK_FILTER_ANGLE_TOLERANCE", 5.0);
    c.parallel_crack_filter_max_distance_px = get_d(cc, "PARALLEL_CRACK_FILTER_MAX_DISTANCE_PX", 10.0);
    return c;
}

static DefectDetectParams parse_defect_detection(const json& dd) {
    DefectDetectParams d;
    d.min_defect_size_mm = get_d(dd, "MIN_DEFECT_SIZE_MM", 5.0);
    d.min_width_mm = get_d(dd, "MIN_WIDTH_MM", 5.0);
    d.max_defect_size_mm = get_d(dd, "MAX_DEFECT_SIZE_MM", 20.0);
    d.false_defect_max_width = get_d(dd, "FALSE_DEFECT_MAX_WIDTH", 8.0);
    d.false_defect_min_aspect_ratio = get_d(dd, "FALSE_DEFECT_MIN_ASPECT_RATIO", 5.0);
    d.angle_deviation_tolerance = get_d(dd, "ANGLE_DEVIATION_TOLERANCE", 15.0);
    if (dd.contains("MERGE_DEFECTS_KERNEL_MM") && dd["MERGE_DEFECTS_KERNEL_MM"].is_array()) {
        d.merge_defects_kernel_mm = dd["MERGE_DEFECTS_KERNEL_MM"].get<std::vector<double>>();
    }
    if (dd.contains("MERGE_DEFECTS_KERNEL_SIZE") && dd["MERGE_DEFECTS_KERNEL_SIZE"].is_array()) {
        d.merge_defects_kernel_size = dd["MERGE_DEFECTS_KERNEL_SIZE"].get<std::vector<int>>();
    }
    d.q_defect_gradient_threshold = get_d(dd, "Q_DEFECT_GRADIENT_THRESHOLD", 5.0);
    d.q_defect_search_width_px = get_i(dd, "Q_DEFECT_SEARCH_WIDTH_PX", 3);
    d.q_triangle_brightness_margin = get_d(dd, "Q_TRIANGLE_BRIGHTNESS_MARGIN", 10.0);
    d.q_canny_stripe_half_width_px = get_i(dd, "Q_CANNY_STRIPE_HALF_WIDTH_PX", 3);
    d.q_triangle_min_area_mm2 = get_d(dd, "Q_TRIANGLE_MIN_AREA_MM2", 15.0);
    d.q_parallelogram_exclude_stripe_half_px = get_i(dd, "Q_PARALLELOGRAM_EXCLUDE_STRIPE_HALF_PX", 10);
    d.q_brightness_min_diff = get_d(dd, "Q_BRIGHTNESS_MIN_DIFF", 25.0);
    d.q_length_min_mm = get_d(dd, "Q_LENGTH_MIN_MM", 5.0);
    d.q_length_max_mm = get_d(dd, "Q_LENGTH_MAX_MM", 30.0);
    d.q_length_range_filter_enable = get_b(dd, "Q_LENGTH_RANGE_FILTER_ENABLE", true);
    d.q_corner_contour_min_dist_px = get_d(dd, "Q_CORNER_CONTOUR_MIN_DIST_PX", 10.0);
    d.q_angle_threshold_deg = get_d(dd, "Q_ANGLE_THRESHOLD_DEG", 5.0);
    d.e_line_suppress_width_px = get_i(dd, "E_LINE_SUPPRESS_WIDTH_PX", 40);
    d.e_fast_prefilter_enable = get_b(dd, "E_FAST_PREFILTER_ENABLE", true);
    d.e_fast_prefilter_relax = get_d(dd, "E_FAST_PREFILTER_RELAX", 0.7);
    d.e_fast_min_edge_pixels = get_i(dd, "E_FAST_MIN_EDGE_PIXELS", 120);
    d.e_fast_min_edge_ratio = get_d(dd, "E_FAST_MIN_EDGE_RATIO", 0.0008);
    d.e_fast_roi_std_threshold = get_d(dd, "E_FAST_ROI_STD_THRESHOLD", 10.0);
    d.e_min_area_mm2 = get_d(dd, "E_MIN_AREA_MM2", 4.0);
    d.e_min_side_mm = get_d(dd, "E_MIN_SIDE_MM", 20.0);
    d.e_border_touch_mm = get_d(dd, "E_BORDER_TOUCH_MM", 5.0);
    d.e_canny_dilate_iter = get_i(dd, "E_CANNY_DILATE_ITER", 1);
    d.e_canny_close_iter = get_i(dd, "E_CANNY_CLOSE_ITER", 1);
    d.e_canny_close_kernel_size = get_i(dd, "E_CANNY_CLOSE_KERNEL_SIZE", 5);
    d.e_line_suppress_extra_px = get_i(dd, "E_LINE_SUPPRESS_EXTRA_PX_PER_DILATE_ITER", 4);
    d.e_include_skew_main_edges = get_b(dd, "E_INCLUDE_SKEW_MAIN_EDGES", true);
    d.e_from_main_edge_min_len_mm = get_d(dd, "E_FROM_MAIN_EDGE_MIN_LEN_MM", 20.0);
    d.skew_curved_remove_line_thickness_px = get_d(dd, "SKEW_CURVED_REMOVE_LINE_THICKNESS_PX", 24.0);
    d.skew_curved_min_arc_len_px = get_d(dd, "SKEW_CURVED_MIN_ARC_LEN_PX", 700.0);
    d.skew_curved_max_dev_ratio = get_d(dd, "SKEW_CURVED_MAX_DEV_RATIO", 0.08);
    d.luminosity_scan_width_mm = get_d(dd, "LUMINOSITY_SCAN_WIDTH_MM", 75.0);
    d.luminosity_scan_width = get_i(dd, "LUMINOSITY_SCAN_WIDTH", 200);
    d.luminosity_max_brightness_threshold = get_d(dd, "LUMINOSITY_MAX_BRIGHTNESS_THRESHOLD", 66.0);
    d.luminosity_std_dev_multiplier = get_d(dd, "LUMINOSITY_STD_DEV_MULTIPLIER", 3.0);
    d.luminosity_min_area_mm2 = get_d(dd, "LUMINOSITY_MIN_AREA_MM2", 15.0);
    d.luminosity_min_area = get_i(dd, "LUMINOSITY_MIN_AREA", 30);
    d.luminosity_edge_ignore_width_mm = get_d(dd, "LUMINOSITY_EDGE_IGNORE_WIDTH_MM", 3.0);
    d.luminosity_edge_ignore_width = get_i(dd, "LUMINOSITY_EDGE_IGNORE_WIDTH", 6);
    d.luminosity_min_gradient = get_d(dd, "LUMINOSITY_MIN_GRADIENT", 20.0);
    d.luminosity_endpoint_exclude_radius_mm = get_d(dd, "LUMINOSITY_ENDPOINT_EXCLUDE_RADIUS_MM", 6.0);
    d.chipping_endpoint_shield_radius_mm = get_d(dd, "CHIPPING_ENDPOINT_SHIELD_RADIUS_MM", 0.0);
    d.chipping_endpoint_shield_radius = get_i(dd, "CHIPPING_ENDPOINT_SHIELD_RADIUS", 0);
    d.corner_max_physical_gap_mm = get_d(dd, "CORNER_MAX_PHYSICAL_GAP_MM", 5.0);
    d.corner_max_physical_gap = get_i(dd, "CORNER_MAX_PHYSICAL_GAP", 7);
    d.corner_min_virtual_gap = get_d(dd, "CORNER_MIN_VIRTUAL_GAP", 7.0);
    d.corner_max_extension_normal_mm = get_d(dd, "CORNER_MAX_EXTENSION_DIST_NORMAL_MM", 0.0);
    d.corner_max_extension_perp_mm = get_d(dd, "CORNER_MAX_EXTENSION_DIST_PERPENDICULAR_MM", 45.0);
    d.corner_max_extension_perp = get_i(dd, "CORNER_MAX_EXTENSION_DIST_PERPENDICULAR", 200);
    d.perpendicular_angle_tolerance = get_d(dd, "PERPENDICULAR_ANGLE_TOLERANCE", 10.0);
    d.vertical_angle_tol_deg = get_d(dd, "VERTICAL_ANGLE_TOL_DEG", 15.0);
    d.l_endpoint_belt_length_mm = get_d(dd, "L_ENDPOINT_BELT_LENGTH_MM", 20.0);
    d.shadow_filter_min_extent_ratio = get_d(dd, "SHADOW_FILTER_MIN_EXTENT_RATIO", 0.25);
    d.prefilter_min_width_mm = get_d(dd, "PREFILTER_MIN_WIDTH_MM", 5.0);
    d.b_max_distance_to_edge_mm = get_d(dd, "B_MAX_DISTANCE_TO_EDGE_MM", 5.0);
    d.b_merge_min_side_mm = get_d(dd, "B_MERGE_MIN_SIDE_MM", 2.0);
    d.b_filter_distance_to_edge_max_mm = get_d(dd, "B_FILTER_DISTANCE_TO_EDGE_MAX_MM", 5.0);
    d.b_filter_parallel_tolerance_deg = get_d(dd, "B_FILTER_PARALLEL_TOLERANCE_DEG", 10.0);
    d.b_filter_near_nonq_intersection_radius_mm = get_d(dd, "B_FILTER_NEAR_NONQ_INTERSECTION_RADIUS_MM", 20.0);
    d.b_to_l_min_ar = get_d(dd, "B_TO_L_MIN_AR", 7.5);
    d.b_to_l_perp_tolerance_deg = get_d(dd, "B_TO_L_PERP_TOLERANCE_DEG", 10.0);
    d.l_filter_max_distance_to_edge_mm = get_d(dd, "L_FILTER_MAX_DISTANCE_TO_EDGE_MM", 15.0);
    d.b_filter_parallel_ar_min = get_d(dd, "B_FILTER_PARALLEL_AR_MIN", 10.0);
    d.b_filter_parallel_min_side_mm = get_d(dd, "B_FILTER_PARALLEL_MIN_SIDE_MM", 2.0);
    d.b_circularity_threshold = get_d(dd, "B_FILTER_CIRCULARITY_THRESHOLD", 0.85);
    d.reclassify_b_as_l_enable = dd.contains("RECLASSIFY_B_AS_L_PARAMS")
        ? dd["RECLASSIFY_B_AS_L_PARAMS"].value("ENABLE", true) : true;
    if (dd.contains("RECLASSIFY_B_AS_L_PARAMS")) {
        const json& rb = dd["RECLASSIFY_B_AS_L_PARAMS"];
        d.reclassify_b_as_l_min_ar = get_d(rb, "MIN_ASPECT_RATIO", 3.2);
        d.reclassify_b_as_l_max_distance_mm = get_d(rb, "MAX_DISTANCE_MM", 73.14);
        d.reclassify_b_as_l_max_distance_px = get_i(rb, "MAX_DISTANCE_PX", 200);
        d.reclassify_b_as_l_angle_tolerance = get_d(rb, "ANGLE_TOLERANCE", 15.0);
        d.reclassify_b_as_l_endpoint_shield_ratio = get_d(rb, "ENDPOINT_SHIELD_RATIO_FOR_EXTENDED", 0.1);
    }
    d.l_min_length_mm = get_d(dd, "L_MIN_LENGTH_MM", 10.0);
    d.static_artifact_enabled = get_b(dd, "STATIC_ARTIFACT_ENABLED", true);
    d.static_artifact_min_consecutive = get_i(dd, "STATIC_ARTIFACT_MIN_CONSECUTIVE", 100);
    d.static_artifact_grid_size = get_i(dd, "STATIC_ARTIFACT_GRID_SIZE", 60);
    d.static_artifact_cooldown_frames = get_i(dd, "STATIC_ARTIFACT_COOLDOWN_FRAMES", 500);
    d.static_artifact_report_interval = get_i(dd, "STATIC_ARTIFACT_REPORT_INTERVAL", 0);
    d.filter_log_enable = get_b(dd, "FILTER_LOG_ENABLE", true);
    d.filter_log_interval_s = get_i(dd, "FILTER_LOG_INTERVAL_S", 30);
    return d;
}

static VisualizationParams parse_visualization(const json& v) {
    VisualizationParams p;
    p.defect_overlay_alpha = get_d(v, "DEFECT_OVERLAY_ALPHA", 0.25);
    p.retreat_distance_threshold = get_d(v, "RETREAT_DISTANCE_THRESHOLD", 150.0);
    p.edge_endpoint_fixed_length = get_d(v, "EDGE_ENDPOINT_FIXED_LENGTH", 20.0);
    return p;
}

// ---------- 单套 InspectorParams ----------
static InspectorParams parse_inspector(const json& hip, const std::string& mode) {
    InspectorParams p;
    p.mode = mode;
    p.preprocessing = parse_preprocessing(hip.value("PREPROCESSING", json::object()));
    p.hough = parse_hough(hip.value("HOUGH_TRANSFORM", json::object()));
    p.line_merging = parse_line_merging(hip.value("LINE_MERGING", json::object()));
    p.crack_classify = parse_crack_classify(hip.value("CRACK_CLASSIFICATION", json::object()));
    p.defect_detection = parse_defect_detection(hip.value("DEFECT_DETECTION", json::object()));
    p.visualization = parse_visualization(hip.value("VISUALIZATION", json::object()));
    return p;
}

// ---------- 总配置 ----------
EngineConfig parse_config(const json& j) {
    EngineConfig cfg;
    auto sys = j.value("system_params", json::object());
    cfg.px_per_mm = get_d(sys, "pixels_per_mm", 2.44);
    cfg.line_name = get_s(j, "lineName", "Line3");
    cfg.storage_path = get_s(j, "storage_path", "inspection_results");
    cfg.light = parse_inspector(j.value("hough_inspector_params", json::object()), "light");
    cfg.dark = parse_inspector(j.value("hough_inspector_dark_params", json::object()), "dark");
    cfg.light.roi_template_file = get_s(j, "roi_template_file", "");
    cfg.dark.roi_template_file = cfg.light.roi_template_file;
    cfg.draw_defects = get_b(j, "draw_defects", true);
    cfg.annotated_output_dir = get_s(j, "annotated_output_dir", "");
    return cfg;
}

// ---------- 结果输出 ----------
static json defect_to_json(const Defect& d, bool with_visual) {
    json dj;
    dj["type"] = d.type;
    dj["confidence"] = d.confidence;
    dj["size_mm"] = d.size_mm;
    dj["width_mm"] = d.width_mm;
    dj["height_mm"] = d.height_mm;
    dj["length_mm"] = d.length_mm;
    dj["angle_deg"] = d.angle_deg;
    dj["circularity"] = d.circularity;
    dj["roi_index"] = d.roi_index;
    dj["location"] = {
        {"x", d.x}, {"y", d.y},
        {"angle", d.angle_deg},
        {"length_mm", d.length_mm},
        {"width_mm", d.width_mm},
        {"subtype", d.subtype}
    };
    if (d.pixel_area > 0) dj["location"]["pixel_area"] = d.pixel_area;
    if (with_visual) {
        if (!d.box_points.empty()) {
            json bp = json::array();
            for (auto& pt : d.box_points) bp.push_back({pt.x, pt.y});
            dj["raw_defect"]["box_points"] = bp;
        }
        if (!d.region_contour.empty()) {
            json rc = json::array();
            for (auto& pt : d.region_contour) rc.push_back({pt.x, pt.y});
            dj["raw_defect"]["region_contour"] = rc;
        }
        if (!d.ray_segments.empty()) {
            json rs = json::array();
            for (auto& [p0, p1] : d.ray_segments)
                rs.push_back(json::array({json::array({p0.x, p0.y}), json::array({p1.x, p1.y})}));
            dj["raw_defect"]["ray_segments"] = rs;
        }
        if (d.center.has_value()) {
            dj["raw_defect"]["center"] = {d.center->x, d.center->y};
        }
        if (!d.barrier_contour.empty()) {
            json bc = json::array();
            for (auto& pt : d.barrier_contour) bc.push_back({pt.x, pt.y});
            dj["raw_defect"]["barrier_contour"] = bc;
        }
        if (!d.skew_subtype.empty()) dj["raw_defect"]["skew_subtype"] = d.skew_subtype;
    }
    return dj;
}

json result_to_json(const DetectionResult& result) {
    json j;
    j["image_status"] = result.image_status;
    j["process_time_ms"] = result.process_time_ms;
    j["total_defects"] = result.total_defects;
    j["filtered_static"] = result.filtered_static;
    if (!result.annotated_image_path.empty()) {
        j["annotated_image_path"] = result.annotated_image_path;
    }

    json defects_json = json::array();
    for (const auto& d : result.defects) {
        defects_json.push_back(defect_to_json(d, true));
    }
    j["defects"] = defects_json;

    json rois_json = json::array();
    for (const auto& rr : result.roi_results) {
        json rj;
        rj["roi"] = {{"x", rr.roi.x}, {"y", rr.roi.y},
                     {"width", rr.roi.width}, {"height", rr.roi.height}};
        json dlist = json::array();
        for (const auto& d : rr.defects) dlist.push_back(defect_to_json(d, true));
        rj["defects"] = dlist;
        rois_json.push_back(rj);
    }
    j["rois"] = rois_json;
    return j;
}

// ---------- ROI 模板加载 ----------
#ifdef _WIN32
#include <windows.h>
#endif

std::vector<RoiRect> load_rois_from_file(const std::string& file_path) {
    std::vector<RoiRect> rois;
#ifdef _WIN32
    // UTF-8 path -> UTF-16 open (Chinese path support)
    int wlen = MultiByteToWideChar(CP_UTF8, 0, file_path.c_str(), -1, nullptr, 0);
    if (wlen <= 0) return rois;
    std::wstring wpath(wlen, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, file_path.c_str(), -1, &wpath[0], wlen);
    std::ifstream f(wpath, std::ios::binary);
#else
    std::ifstream f(file_path, std::ios::binary);
#endif
    if (!f.is_open()) return rois;
    json j;
    try { f >> j; } catch (...) { return rois; }

    // 取 source_image_count 最大的 group（与 Python 端模板选择一致）
    std::string best_group;
    int max_count = -1;
    for (auto& [key, val] : j.items()) {
        if (!val.is_object()) continue;
        int cnt = val.value("source_image_count", 0);
        if (cnt > max_count) { max_count = cnt; best_group = key; }
    }
    if (best_group.empty()) return rois;
    auto& avg_rois = j[best_group]["averaged_rois"];
    for (auto& r : avg_rois) {
        RoiRect roi;
        roi.x = r.value("x", 0);
        roi.y = r.value("y", 0);
        roi.width = r.value("width", r.value("w", 0));
        roi.height = r.value("height", r.value("h", 0));
        if (roi.width > 0 && roi.height > 0) rois.push_back(roi);
    }
    return rois;
}

// ---------- 请求解析 ----------
DetectionRequest parse_request(const std::string& json_str) {
    json j = json::parse(json_str);
    DetectionRequest req;
    req.image_path = j.value("image_path", "");
    req.total_frames = j.value("total_frames", 0);
    auto cfg_json = j.value("config", json::object());
    // 若传入完整 config（含 hough_inspector_params），整体解析；否则按平铺键解析
    if (cfg_json.contains("hough_inspector_params") || cfg_json.contains("system_params")) {
        req.config = parse_config(cfg_json);
    } else {
        // 平铺模式：把整份 config 当作 hough_inspector_params（兼容旧 wrapper）
        json wrapped = {{"hough_inspector_params", cfg_json},
                        {"system_params", {{"pixels_per_mm", 2.44}}}};
        req.config = parse_config(wrapped);
    }
    // 运行期字段
    req.config.total_frames = req.total_frames;
    req.config.cam_key = j.value("cam_key", req.config.cam_key);
    req.config.static_artifact_enabled = j.value("static_artifact_enabled", req.config.static_artifact_enabled);
    req.config.draw_defects = j.value("draw_defects", req.config.draw_defects);
    // 顶层键存在才覆盖（否则保留 config 内的 annotated_output_dir，避免误清空）
    if (j.contains("annotated_output_dir")) {
        req.config.annotated_output_dir = j.value("annotated_output_dir", "");
    }
    return req;
}
