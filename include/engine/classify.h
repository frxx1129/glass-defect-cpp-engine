#pragma once
#include "engine/types.h"
#include "engine/config.h"
#include "engine/line_merge.h"
#include <opencv2/core.hpp>

// 镜像 Python find_and_analyze_defects 的核心分类流程。
// edges: 合并后的主直线；roi_gray: ROI 灰度图；roi_dims: (h, w)
std::vector<Defect> find_and_analyze_defects(
    const std::vector<MergedLine>& edges,
    const cv::Mat& roi_gray,
    const cv::Size& roi_dims,
    const InspectorParams& params,
    double px_per_mm);
