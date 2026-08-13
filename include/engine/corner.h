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
