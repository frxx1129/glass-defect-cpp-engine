#include "engine/detector.h"
#include "engine/preprocess.h"
#include "engine/line_merge.h"
#include "engine/classify.h"
#include "engine/skew.h"
#include "engine/luminosity.h"
#include "engine/json_io.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <cwchar>
#endif
#include <filesystem>

Detector::Detector(const EngineConfig& config) : config_(config) {
    px_per_mm_ = config_.px_per_mm > 0 ? config_.px_per_mm : 2.44;
    cam_key_ = config_.cam_key.empty() ? "cam0" : config_.cam_key;
}

cv::Mat Detector::load_image(const std::string& image_path) {
    std::vector<uchar> buffer;
#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, 0, image_path.c_str(), -1, nullptr, 0);
    if (wlen <= 0) throw std::runtime_error("Failed to convert path to UTF-16: " + image_path);
    std::wstring wpath(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, image_path.c_str(), -1, &wpath[0], wlen);
    FILE* fp = nullptr;
    if (_wfopen_s(&fp, wpath.c_str(), L"rb") != 0 || !fp)
        throw std::runtime_error("Failed to open image file: " + image_path);
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buffer.resize(static_cast<size_t>(fsize));
    size_t read = fread(buffer.data(), 1, buffer.size(), fp);
    fclose(fp);
    if (read != buffer.size())
        throw std::runtime_error("Failed to read image file: " + image_path);
#else
    std::ifstream file(image_path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Failed to open image file: " + image_path);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    buffer.resize(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size))
        throw std::runtime_error("Failed to read image file: " + image_path);
#endif
    cv::Mat img = cv::imdecode(cv::Mat(buffer), cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        // 回退：彩色解码再转灰度
        cv::Mat color = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
        if (color.empty()) throw std::runtime_error("Failed to decode image: " + image_path);
        cv::cvtColor(color, img, cv::COLOR_BGR2GRAY);
    }
    return img;
}

std::vector<Defect> Detector::process_roi(
    const cv::Mat& image_gray,
    const RoiRect& roi,
    const InspectorParams& params,
    int roi_idx)
{
    // 裁剪 ROI（越界保护）
    int x0 = std::max(0, roi.x);
    int y0 = std::max(0, roi.y);
    int x1 = std::min(image_gray.cols, roi.x + roi.width);
    int y1 = std::min(image_gray.rows, roi.y + roi.height);
    if (x1 <= x0 || y1 <= y0) return {};
    cv::Mat roi_gray = image_gray(cv::Rect(x0, y0, x1 - x0, y1 - y0));

    // 预处理（Hough 版，含兜底增强）
    cv::Mat edges = preprocess_for_hough_enhanced(roi_gray, params.preprocessing);

    // Hough 直线（镜像 Python：先按 DOWNSAMPLE_SCALE 降采样再放大回原图）
    std::vector<cv::Vec4i> lines;
    {
        // 最小线段长度：镜像 Python MIN_LINE_LENGTH_MODE="min"（min(ROI宽,高) x 比例，不设下限）
        double min_len = params.hough.min_line_length;
        if (params.hough.min_line_length_ratio > 0) {
            double min_base = std::min(roi_gray.cols, roi_gray.rows);
            min_len = min_base * params.hough.min_line_length_ratio;
        }
        // Python DOWNSAMPLE_SCALE 默认 0.85
        double ds_scale = 0.85;
        if (ds_scale < 0.999) {
            int w_full = edges.cols, h_full = edges.rows;
            int w_small = std::max(1, (int)std::lround(w_full * ds_scale));
            int h_small = std::max(1, (int)std::lround(h_full * ds_scale));
            cv::Mat edges_small;
            cv::resize(edges, edges_small, cv::Size(w_small, h_small), 0, 0, cv::INTER_AREA);
            double scale_x = (double)w_full / w_small;
            double scale_y = (double)h_full / h_small;
            double scale_len = std::min(1.0, std::min(w_small / (double)std::max(1, w_full),
                                                      h_small / (double)std::max(1, h_full)));
            int min_len_small = std::max(1, (int)std::lround(min_len * scale_len));
            double max_gap_full = params.hough.max_line_gap_mm > 0
                ? params.hough.max_line_gap_mm * px_per_mm_ : params.hough.max_line_gap;
            int max_gap_small = std::max(0, (int)std::lround(max_gap_full * scale_len));
            std::vector<cv::Vec4i> raw_small;
            cv::HoughLinesP(edges_small, raw_small,
                            params.hough.rho,
                            params.hough.theta_deg * CV_PI / 180.0,
                            params.hough.threshold,
                            min_len_small,
                            max_gap_small);
            // 坐标放大回原图
            for (auto& l : raw_small) {
                l[0] = (int)std::lround(l[0] * scale_x);
                l[1] = (int)std::lround(l[1] * scale_y);
                l[2] = (int)std::lround(l[2] * scale_x);
                l[3] = (int)std::lround(l[3] * scale_y);
            }
            lines = raw_small;
        } else {
            double max_gap_full = params.hough.max_line_gap_mm > 0
                ? params.hough.max_line_gap_mm * px_per_mm_ : params.hough.max_line_gap;
            cv::HoughLinesP(edges, lines,
                            params.hough.rho,
                            params.hough.theta_deg * CV_PI / 180.0,
                            params.hough.threshold,
                            min_len,
                            max_gap_full);
        }
    }

    // 直线合并
    auto merged = merge_lines_and_get_main_edges(lines, params, px_per_mm_, edges);

    // 缺陷来源 1：亮度扫描（沿主边扫描暗区 → B 崩边候选，镜像 Python scan_edge_for_luminosity_defects）
    std::vector<Defect> defects;
    for (auto& ml : merged) {
        auto contours = scan_edge_for_luminosity_defects(roi_gray, ml.line, params, px_per_mm_);
        for (auto& cnt : contours) {
            cv::RotatedRect rr = cv::minAreaRect(cnt);
            cv::Point2f pts[4];
            rr.points(pts);
            Defect d;
            d.type = "B";
            d.confidence = 0.65;
            for (auto& pt : pts)
                d.box_points.push_back(cv::Point((int)std::lround(pt.x), (int)std::lround(pt.y)));
            d.x = (int)std::lround(rr.center.x);
            d.y = (int)std::lround(rr.center.y);
            double w_px = std::max(rr.size.width, rr.size.height);
            double h_px = std::min(rr.size.width, rr.size.height);
            d.length_mm = w_px / px_per_mm_;
            d.width_mm = h_px / px_per_mm_;
            d.size_mm = d.length_mm;
            d.roi_index = roi_idx;
            defects.push_back(d);
        }
    }
    for (auto& d : defects) d.roi_index = roi_idx;

    // 缺陷来源 2：E 型边缘异常（缺陷边缘图 + 主边屏蔽带 + 轮廓分析）
    cv::Mat defect_edges = preprocess_for_defect_edges(roi_gray, params.preprocessing);
    auto e_defects = detect_e_defects(roi_gray, defect_edges, merged, params, px_per_mm_);
    for (auto& d : e_defects) d.roi_index = roi_idx;
    defects.insert(defects.end(), e_defects.begin(), e_defects.end());

    // 预过滤（镜像 Python QBL_PREFILTER_MIN_WIDTH）：Q/B/L 宽度 < PREFILTER_MIN_WIDTH_MM 过滤
    double prefilter_min_width = params.defect_detection.prefilter_min_width_mm > 0
        ? params.defect_detection.prefilter_min_width_mm
        : params.defect_detection.min_width_mm;
    if (prefilter_min_width > 0) {
        std::vector<Defect> kept;
        for (auto& d : defects) {
            if (d.type == "Q" || d.type == "B" || d.type == "L") {
                if (d.width_mm < prefilter_min_width) continue;
                // Q 长度范围过滤
                if (d.type == "Q" && params.defect_detection.q_length_range_filter_enable) {
                    if (d.length_mm < params.defect_detection.q_length_min_mm ||
                        d.length_mm > params.defect_detection.q_length_max_mm) continue;
                }
            }
            kept.push_back(d);
        }
        defects = kept;
    }

    // 坐标还原到全图
    for (auto& d : defects) { d.x += roi.x; d.y += roi.y; }
    for (auto& d : defects) {
        for (auto& pt : d.box_points) { pt.x += roi.x; pt.y += roi.y; }
        for (auto& pt : d.region_contour) { pt.x += roi.x; pt.y += roi.y; }
        for (auto& seg : d.ray_segments) { seg.first += cv::Point(roi.x, roi.y); seg.second += cv::Point(roi.x, roi.y); }
        for (auto& pt : d.barrier_contour) { pt.x += roi.x; pt.y += roi.y; }
        if (d.center) *d.center += cv::Point(roi.x, roi.y);
    }
    return defects;
}

std::vector<Defect> Detector::apply_filter_chain(std::vector<Defect> defects) {
    // 阶段 1：静态干扰抑制（镜像 Python filter_static_artifact_defects）
    // 阶段 2+：阴影过滤 / B-L 重分类 / 尺寸过滤（随算法镜像逐步补全）
    if (!config_.static_artifact_enabled) return defects;

    auto& tracker = trackers_[cam_key_];
    // StaticArtifactTracker::filter 返回过滤后缺陷；同时生成 S 心跳
    return tracker.filter(defects, cam_key_);
}

DetectionResult Detector::detect(const cv::Mat& image_gray) {
    DetectionResult result;
    auto start = cv::getTickCount();

    // 选择参数（明场默认；暗场由请求注入 mode）
    const InspectorParams* params = &config_.light;

    // 加载 ROI
    std::vector<RoiRect> rois = external_rois_;
    if (rois.empty() && !params->roi_template_file.empty()) {
        rois = load_rois_from_file(params->roi_template_file);
    }

    // 逐 ROI 检测
    std::vector<Defect> all_defects;
    for (size_t i = 0; i < rois.size(); i++) {
        auto defects = process_roi(image_gray, rois[i], *params, (int)i);
        all_defects.insert(all_defects.end(), defects.begin(), defects.end());
        RoiResult rr;
        rr.roi = rois[i];
        rr.defects = defects;
        result.roi_results.push_back(std::move(rr));
    }

    // 过滤链
    auto filtered = apply_filter_chain(all_defects);

    auto end = cv::getTickCount();
    result.process_time_ms = (end - start) * 1000.0 / cv::getTickFrequency();
    result.defects = filtered;
    result.total_defects = (int)filtered.size();
    int suppressed = (int)all_defects.size() - (int)filtered.size();
    result.filtered_static = suppressed > 0 ? suppressed : 0;
    if (!result.defects.empty()) {
        result.image_status = "NG";
    }
    return result;
}

DetectionResult Detector::detect_file(const std::string& image_path) {
    cv::Mat gray = load_image(image_path);
    return detect(gray);
}
