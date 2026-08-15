#pragma once
#include "engine/types.h"
#include "engine/config.h"
#include "engine/line_merge.h"
#include <vector>
#include <opencv2/core.hpp>

// Q 角点（缺角）检测（镜像 Python corner_contour_q_defects 流程）
// true_edges: 合并后的主直线；binary_edges: 缺陷边缘图（用于提取玻璃主体轮廓）
std::vector<Defect> detect_q_defects(
    const cv::Mat& roi_gray,
    const std::vector<MergedLine>& true_edges,
    const cv::Mat& binary_edges,
    const InspectorParams& params,
    double px_per_mm);

// 收集“未产生 Q 的直线配对交点”（局部 ROI 坐标）。
// 镜像 Python find_and_analyze_defects 的 potential_pairs 配对循环：
// 每主体 1V+1H -> 兼容度/端点距离排序 -> 交点接受规则 -> endpoint_paired_status。
// detector 用这些交点过滤其附近的 B 误检（B_FILTER_NEAR_NONQ_INTERSECTION_RADIUS_MM）。
std::vector<cv::Point2d> collect_non_q_intersections(
    const std::vector<MergedLine>& true_edges,
    const InspectorParams& params,
    double px_per_mm,
    int roi_w,
    int roi_h);
