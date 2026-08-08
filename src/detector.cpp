#include "engine/detector.h"
#include "engine/preprocess.h"
#include "engine/line_merge.h"
#include "engine/classify.h"
#include "engine/json_io.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
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

    // Hough 直线
    std::vector<cv::Vec4i> lines;
    {
        double min_len = params.hough.min_line_length;
        if (params.hough.min_line_length_ratio > 0) {
            double diag = std::sqrt(double(roi_gray.cols) * double(roi_gray.cols) +
                                    double(roi_gray.rows) * double(roi_gray.rows));
            min_len = std::max(min_len, diag * params.hough.min_line_length_ratio);
        }
        if (params.hough.max_line_gap_mm > 0) {
            // MAX_LINE_GAP_MM 优先于像素版
        }
        cv::HoughLinesP(edges, lines,
                        params.hough.rho,
                        params.hough.theta_deg * CV_PI / 180.0,
                        params.hough.threshold,
                        min_len,
                        params.hough.max_line_gap);
    }

    // 直线合并
    auto merged = merge_lines_and_get_main_edges(lines, params, px_per_mm_, edges);

    // 缺陷分类
    auto defects = find_and_analyze_defects(merged, roi_gray, roi_gray.size(), params, px_per_mm_);
    for (auto& d : defects) d.roi_index = roi_idx;

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
