#pragma once
#include "engine/types.h"
#include "engine/config.h"
#include <vector>
#include <opencv2/core.hpp>

// 亮度扫描（镜像 Python scan_edge_for_luminosity_defects）：
// 沿主边构建扫描带，取亮度更低的一侧，以 mean - k*std 为阈值找暗区轮廓。
// 返回轮廓列表（后续由调用方转为 B 缺陷）。
std::vector<std::vector<cv::Point>> scan_edge_for_luminosity_defects(
    const cv::Mat& roi_gray,
    const cv::Vec4d& edge,
    const InspectorParams& params,
    double px_per_mm);
