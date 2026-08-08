#pragma once
#include "engine/config.h"
#include <opencv2/core.hpp>

// Hough 增强预处理（镜像 Python preprocess_for_hough_enhanced）：
// medianBlur + CLAHE + Canny；若边缘过稀则降阈值补跑一次并合并，再轻微膨胀。
cv::Mat preprocess_for_hough_enhanced(const cv::Mat& gray, const PreprocessParams& p);

// 缺陷边缘预处理（镜像 Python preprocess_for_defect_edges）：
// 仅 medianBlur + CLAHE + Canny，不做兜底增强。
cv::Mat preprocess_for_defect_edges(const cv::Mat& gray, const PreprocessParams& p);

// 基础预处理（中值滤波 + CLAHE），供各模块复用
cv::Mat enhance_contrast(const cv::Mat& gray, const PreprocessParams& p);
