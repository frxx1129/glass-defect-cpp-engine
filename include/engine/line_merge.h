#pragma once
#include "engine/config.h"
#include <vector>
#include <opencv2/core.hpp>

// 合并后的直线（镜像 Python merge_lines_and_get_main_edges 输出）
struct MergedLine {
    cv::Vec4d line;          // 合并后直线 (x1,y1,x2,y2)
    double angle_deg = 0.0;  // 0°=水平, 90°=竖直（[0,180) 真实角度）
    double length_px = 0.0;
    int support = 0;         // 镜像 Python score：组内原始线段长度之和（用于 topN 排序/强度比较）
    bool near_vertical = false;
    bool near_horizontal = false;
};

// 主合并函数：角度聚类 + 邻近聚类（Canny 缝隙检查）+ 竖直粗边二次合并 + fitLine + 轴向锁定
// lines: HoughLinesP 输出；edge_img: 用于缝隙检查和轴向锁定的 Canny 边缘图（可为空）
std::vector<MergedLine> merge_lines_and_get_main_edges(
    const std::vector<cv::Vec4i>& lines,
    const InspectorParams& params,
    double px_per_mm,
    const cv::Mat& edge_img);

// 工具：线段长度/角度（供其他模块复用）
double line_length_px(const cv::Vec4i& l);
double line_angle_deg(const cv::Vec4i& l);
double line_angle_deg(const cv::Vec4d& l);
