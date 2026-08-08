#include "engine/preprocess.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>

cv::Mat enhance_contrast(const cv::Mat& gray, const PreprocessParams& p) {
    cv::Mat blurred, out;
    int ksize = p.median_blur_ksize;
    if (ksize % 2 == 0) ksize += 1;
    if (ksize <= 1) ksize = 3;
    cv::medianBlur(gray, blurred, ksize);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        p.clahe_clip_limit,
        cv::Size(std::max(1, p.clahe_grid_x), std::max(1, p.clahe_grid_y)));
    clahe->apply(blurred, out);
    return out;
}

static cv::Mat canny_once(const cv::Mat& contrast, double low, double high) {
    int l = static_cast<int>(low);
    int h = static_cast<int>(high);
    if (h <= l) h = l + 1;
    cv::Mat edges;
    cv::Canny(contrast, edges, l, h);
    return edges;
}

cv::Mat preprocess_for_defect_edges(const cv::Mat& gray, const PreprocessParams& p) {
    cv::Mat contrast = enhance_contrast(gray, p);
    return canny_once(contrast, p.canny_low, p.canny_high);
}

cv::Mat preprocess_for_hough_enhanced(const cv::Mat& gray, const PreprocessParams& p) {
    cv::Mat contrast = enhance_contrast(gray, p);
    cv::Mat edges = canny_once(contrast, p.canny_low, p.canny_high);

    // 兜底增强：边缘过稀时降阈值补跑 + 膨胀
    double edge_ratio = 0.0;
    if (!edges.empty()) {
        int non_zero = cv::countNonZero(edges);
        edge_ratio = static_cast<double>(non_zero) / static_cast<double>(edges.total());
    }
    if (edge_ratio < p.hough_min_edge_ratio) {
        // 1) 降阈值补跑
        double scale = p.hough_canny_fallback_scale;
        double low2 = std::max(0.0, p.canny_low * scale);
        double high2 = std::max(low2 + 1.0, p.canny_high * scale);
        cv::Mat edges2 = canny_once(contrast, low2, high2);
        cv::bitwise_or(edges, edges2, edges);

        // 2) 轻微膨胀连接断裂边缘
        if (p.hough_edge_dilate_iter > 0) {
            int kx = p.hough_edge_dilate_kernel.size() >= 1 ? p.hough_edge_dilate_kernel[0] : 3;
            int ky = p.hough_edge_dilate_kernel.size() >= 2 ? p.hough_edge_dilate_kernel[1] : 3;
            if (kx < 1) kx = 1;
            if (ky < 1) ky = 1;
            if (kx % 2 == 0) kx += 1;
            if (ky % 2 == 0) ky += 1;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kx, ky));
            cv::dilate(edges, edges, kernel, cv::Point(-1, -1), p.hough_edge_dilate_iter);
        }
    }
    return edges;
}
