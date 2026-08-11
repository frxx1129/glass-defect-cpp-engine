#pragma once
#include "engine/types.h"
#include "engine/config.h"
#include "engine/line_merge.h"
#include <opencv2/core.hpp>

// E 型边缘异常检测（镜像 Python find_and_analyze_defects 内 skew_line_defects 流程）
// defect_edges: 缺陷边缘图（preprocess_for_defect_edges 输出，未膨胀）
// true_edges:  合并后的主直线（近竖/近水平用于屏蔽带；斜向主边直接成为 E 候选）
std::vector<Defect> detect_e_defects(
    const cv::Mat& roi_gray,
    const cv::Mat& defect_edges,
    const std::vector<MergedLine>& true_edges,
    const InspectorParams& params,
    double px_per_mm);
