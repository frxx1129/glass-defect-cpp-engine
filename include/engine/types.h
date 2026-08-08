#pragma once
#include <string>
#include <vector>
#include <optional>
#include <opencv2/core.hpp>

// ============================================================
// 核心类型定义（对齐 Python 端 raw_defect / location / ROI）
// ============================================================

// ROI 矩形
struct RoiRect {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// 缺陷类型（Python defect_type_map）
//   Q 缺角 / B 崩边 / E 边缘异常 / X 斜边 / L 裂纹 / S 静态心跳
// 可视化颜色（DEFECT_COLORS_BGR）
//   Q:(0,0,255) 红  E:(0,0,255) 红  X:(255,0,0) 蓝  L:(255,0,255) 紫  B:(0,165,255) 橙

struct Defect {
    std::string type;          // B / L / Q / E / S / X
    double confidence = 0.0;

    // ---- location ----
    int x = 0;                 // location.x
    int y = 0;                 // location.y
    double angle_deg = 0.0;    // location.angle（E/X 显示角度/曲度用）
    double length_mm = 0.0;    // location.length_mm
    double height_mm = 0.0;    // 高度（mm）
    double width_mm = 0.0;     // location.width_mm
    int pixel_area = 0;        // location.pixel_area（Q 用）
    std::string subtype;       // location.subtype == "curved" 时显示"曲度"
    std::string skew_subtype;  // raw_defect.skew_subtype == "curved"

    // ---- 可视化 / 轮廓数据（raw_defect）----
    std::vector<cv::Point> box_points;      // L/B/X/E 的四边形轮廓（画 fillPoly+drawContours）
    std::vector<cv::Point> region_contour;  // Q 的真实像素轮廓
    std::vector<std::pair<cv::Point, cv::Point>> ray_segments; // Q 检测条带射线（画箭头）
    std::optional<cv::Point> center;        // X 的中心（画圆）
    std::vector<cv::Point> barrier_contour; // Q 的玻璃轮廓（画黄线）

    // ---- 原始检测参数 ----
    double circularity = 0.0;
    double size_mm = 0.0;      // max(width, height)

    // 该缺陷属于哪个 ROI（多 ROI 汇总时用）
    int roi_index = -1;
};

// 单 ROI 检测结果
struct RoiResult {
    RoiRect roi;
    std::vector<Defect> defects;
};

// 整图检测结果
struct DetectionResult {
    std::string image_status = "OK";   // OK / NG
    std::vector<Defect> defects;       // 全图汇总（静态抑制后）
    std::vector<RoiResult> roi_results; // 各 ROI 结果（含被抑制前，供可视化）
    double process_time_ms = 0.0;
    int total_defects = 0;
    int filtered_static = 0;
    // 标注图输出：非空表示已保存
    std::string annotated_image_path;
};
