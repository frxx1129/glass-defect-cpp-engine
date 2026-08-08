// 引擎单元测试（配置解析 + ROI 加载 + 空图安全）
#include "engine/config.h"
#include "engine/types.h"
#include "engine/json_io.h"
#include "engine/preprocess.h"
#include "engine/line_merge.h"
#include <cassert>
#include <iostream>

int main() {
    // 1. 配置解析
    json cfg = {
        {"lineName", "Line3"},
        {"roi_template_file", "roi.json"},
        {"system_params", {{"pixels_per_mm", 2.44}}},
        {"hough_inspector_params", {
            {"PREPROCESSING", {{"MEDIAN_BLUR_KSIZE", 17}, {"CANNY_THRESHOLD_LOW", 35}, {"CANNY_THRESHOLD_HIGH", 90}}},
            {"HOUGH_TRANSFORM", {{"THRESHOLD", 35}, {"MIN_LINE_LENGTH_RATIO", 0.03}, {"MAX_LINE_GAP", 30}}},
            {"LINE_MERGING", {{"ANGLE_TOLERANCE", 5.0}, {"TOP_N_EDGES", 6}}},
            {"DEFECT_DETECTION", {{"MIN_DEFECT_SIZE_MM", 5.0}, {"STATIC_ARTIFACT_REPORT_INTERVAL", 0}}},
            {"VISUALIZATION", {{"DEFECT_OVERLAY_ALPHA", 0.25}}}
        }}
    };
    auto ecfg = parse_config(cfg);
    assert(ecfg.px_per_mm == 2.44);
    assert(ecfg.light.preprocessing.median_blur_ksize == 17);
    assert(ecfg.light.hough.min_line_length_ratio == 0.03);
    assert(ecfg.light.visualization.defect_overlay_alpha == 0.25);
    std::cout << "[PASS] config parse\n";

    // 2. 预处理空图安全
    cv::Mat gray(100, 100, CV_8UC1, cv::Scalar(128));
    auto edges = preprocess_for_hough_enhanced(gray, ecfg.light.preprocessing);
    assert(!edges.empty());
    std::cout << "[PASS] preprocess\n";

    // 3. 直线合并空输入
    std::vector<cv::Vec4i> lines;
    auto merged = merge_lines_and_get_main_edges(lines, ecfg.light, 2.44, edges);
    assert(merged.empty());
    std::cout << "[PASS] line merge empty\n";

    // 4. 直线合并单线
    lines.push_back(cv::Vec4i(10, 50, 90, 50)); // 水平长线
    merged = merge_lines_and_get_main_edges(lines, ecfg.light, 2.44, edges);
    assert(!merged.empty());
    std::cout << "[PASS] line merge single\n";

    std::cout << "ALL TESTS PASSED" << std::endl;
    return 0;
}
