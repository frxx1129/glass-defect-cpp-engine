#include "engine/luminosity.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>

std::vector<std::vector<cv::Point>> scan_edge_for_luminosity_defects(
    const cv::Mat& roi_gray,
    const cv::Vec4d& edge,
    const InspectorParams& params,
    double px_per_mm)
{
    std::vector<std::vector<cv::Point>> result;
    if (roi_gray.empty()) return result;

    const DefectDetectParams& p = params.defect_detection;
    cv::Point2d p1(edge[0], edge[1]), p2(edge[2], edge[3]);
    double line_vec_x = p2.x - p1.x, line_vec_y = p2.y - p1.y;
    double line_length = std::hypot(line_vec_x, line_vec_y);
    if (line_length < 1e-6) return result;

    double scan_width = p.luminosity_scan_width_mm > 0
        ? p.luminosity_scan_width_mm * px_per_mm : p.luminosity_scan_width;
    scan_width = std::max(1.0, scan_width);

    double ux = line_vec_x / line_length, uy = line_vec_y / line_length;
    double nx = -uy, ny = ux; // 法线
    double hwx = (scan_width / 2.0) * nx, hwy = (scan_width / 2.0) * ny;

    cv::Point2d c1(p1.x + hwx, p1.y + hwy), c2(p2.x + hwx, p2.y + hwy);
    cv::Point2d c3(p2.x - hwx, p2.y - hwy), c4(p1.x - hwx, p1.y - hwy);

    // 两侧掩膜，取亮度更低一侧
    cv::Mat mask_plus = cv::Mat::zeros(roi_gray.size(), CV_8U);
    cv::Mat mask_minus = cv::Mat::zeros(roi_gray.size(), CV_8U);
    auto to_pts = [](std::initializer_list<cv::Point2d> pts) {
        std::vector<cv::Point> v;
        for (auto& pt : pts) v.push_back(cv::Point((int)std::lround(pt.x), (int)std::lround(pt.y)));
        return v;
    };
    cv::fillPoly(mask_plus, std::vector<std::vector<cv::Point>>{
        to_pts({c1, c2, p2, p1})}, cv::Scalar(255));
    cv::fillPoly(mask_minus, std::vector<std::vector<cv::Point>>{
        to_pts({p1, p2, c3, c4})}, cv::Scalar(255));

    double mean_plus = cv::mean(roi_gray, mask_plus)[0];
    double mean_minus = cv::mean(roi_gray, mask_minus)[0];
    cv::Mat scan_mask = mean_plus < mean_minus ? mask_plus : mask_minus;

    // 动态忽略宽度：边中点到整图中心距离线性映射（0px->0mm, 2000px->4mm）
    cv::Point2d edge_mid((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
    // 无 FRAME_WIDTH/ROI_OFFSET 注入时，用 ROI 自身中心近似
    cv::Point2d global_center(roi_gray.cols / 2.0, roi_gray.rows / 2.0);
    double dist_px_dynamic = std::hypot(edge_mid.x - global_center.x, edge_mid.y - global_center.y);
    double ignore_width_mm = std::min(4.0, std::max(0.0, dist_px_dynamic * 0.002));
    double edge_ignore_px = ignore_width_mm * px_per_mm;

    // 端点排除半径：默认 max(3, min(0.3*scan_width, 15))
    double endpoint_exclude_r = p.luminosity_endpoint_exclude_radius_mm > 0
        ? p.luminosity_endpoint_exclude_radius_mm * px_per_mm : 0.0;
    if (endpoint_exclude_r <= 0) {
        endpoint_exclude_r = std::max(3, (int)std::min(0.3 * scan_width, 15.0));
    }

    cv::Mat ignore_mask = cv::Mat::zeros(roi_gray.size(), CV_8U);
    if (edge_ignore_px > 0.5) {
        int thickness = (int)std::ceil(edge_ignore_px);
        thickness = std::max(1, std::min(256, thickness));
        cv::line(ignore_mask, cv::Point((int)std::lround(p1.x), (int)std::lround(p1.y)),
                 cv::Point((int)std::lround(p2.x), (int)std::lround(p2.y)),
                 cv::Scalar(255), thickness);
    }
    cv::circle(ignore_mask, cv::Point((int)std::lround(p1.x), (int)std::lround(p1.y)),
               (int)std::lround(endpoint_exclude_r), cv::Scalar(255), -1);
    cv::circle(ignore_mask, cv::Point((int)std::lround(p2.x), (int)std::lround(p2.y)),
               (int)std::lround(endpoint_exclude_r), cv::Scalar(255), -1);

    cv::Scalar mean, stddev;
    cv::meanStdDev(roi_gray, mean, stddev, scan_mask);
    double std_val = stddev[0];
    if (std_val <= 3.0) return result;

    double threshold_low = mean[0] - p.luminosity_std_dev_multiplier * std_val;
    double min_area_px2 = p.luminosity_min_area_mm2 > 0
        ? p.luminosity_min_area_mm2 * px_per_mm * px_per_mm : p.luminosity_min_area;
    double min_gradient = p.luminosity_min_gradient > 0 ? p.luminosity_min_gradient : 15.0;

    cv::Mat potential;
    cv::compare(roi_gray, threshold_low, potential, cv::CMP_LT);
    potential.convertTo(potential, CV_8U, 255.0, 0.0);
    cv::Mat defect_mask;
    cv::bitwise_and(potential, scan_mask, defect_mask);
    cv::subtract(defect_mask, ignore_mask, defect_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(defect_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    // 梯度均值过滤
    cv::Mat blurred, grad_x, grad_y, grad_mag;
    cv::medianBlur(roi_gray, blurred, 3);
    cv::Sobel(blurred, grad_x, CV_64F, 1, 0, 3);
    cv::Sobel(blurred, grad_y, CV_64F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, grad_mag);

    for (auto& cnt : contours) {
        double area = std::abs(cv::contourArea(cnt));
        if (area <= min_area_px2) continue;
        cv::Mat contour_mask = cv::Mat::zeros(roi_gray.size(), CV_8U);
        cv::drawContours(contour_mask, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(255), -1);
        double mean_grad = cv::mean(grad_mag, contour_mask)[0];
        if (mean_grad > min_gradient) {
            result.push_back(cnt);
        }
    }
    return result;
}
