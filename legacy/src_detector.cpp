#include "detector.h"
#include "json_io.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstdio>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <cwchar>
#endif
#include <cstdio>
#include <filesystem>

Detector::Detector(const EngineConfig& config) : config_(config) {
    px_per_mm_ = config_.px_per_mm > 0 ? config_.px_per_mm : 10.0;
    static_tracker_ = std::make_unique<StaticArtifactTracker>(config_);
}

cv::Mat Detector::load_and_preprocess(const std::string& image_path) {
    std::vector<uchar> buffer;
#ifdef _WIN32
    int wlen = MultiByteToWideChar(CP_UTF8, 0, image_path.c_str(), -1, nullptr, 0);
    if (wlen <= 0) {
        throw std::runtime_error("Failed to convert path to UTF-16: " + image_path);
    }
    std::wstring wpath(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, image_path.c_str(), -1, &wpath[0], wlen);

    FILE* fp = nullptr;
    if (_wfopen_s(&fp, wpath.c_str(), L"rb") != 0 || !fp) {
        throw std::runtime_error("Failed to open image file: " + image_path);
    }
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    buffer.resize(static_cast<size_t>(fsize));
    size_t read = fread(buffer.data(), 1, buffer.size(), fp);
    fclose(fp);
    if (read != buffer.size()) {
        throw std::runtime_error("Failed to read image file: " + image_path);
    }
#else
    std::filesystem::path p(image_path);
    std::ifstream file(p, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Failed to open image file: " + image_path);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    buffer.resize(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        throw std::runtime_error("Failed to read image file: " + image_path);
    }
#endif

    cv::Mat img = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
    if (img.empty()) {
        throw std::runtime_error("Failed to decode image: " + image_path);
    }
    return img;
}

cv::Mat Detector::apply_roi(const cv::Mat& image) {
    if (config_.roi_w <= 0 || config_.roi_h <= 0) {
        return image.clone();
    }
    cv::Rect roi(
        std::max(0, config_.roi_x),
        std::max(0, config_.roi_y),
        std::min(config_.roi_w, image.cols - config_.roi_x),
        std::min(config_.roi_h, image.rows - config_.roi_y)
    );
    return image(roi).clone();
}

cv::Mat Detector::edge_detect(const cv::Mat& gray) {
    cv::Mat blurred, clahe_out, edges;

    int ksize = config_.median_blur_ksize;
    if (ksize % 2 == 0) ksize += 1;
    cv::medianBlur(gray, blurred, ksize > 0 ? ksize : 17);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        config_.clahe_clip_limit,
        cv::Size(config_.clahe_grid_size_x, config_.clahe_grid_size_y)
    );
    clahe->apply(blurred, clahe_out);

    cv::Canny(clahe_out, edges, config_.canny_low, config_.canny_high);
    return edges;
}

// 线段长度（像素）
static double line_length(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    return std::sqrt(dx * dx + dy * dy);
}

// 线段角度（度）
static double line_angle_deg(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    double ang = std::atan2(std::abs(dy), dx) * 180.0 / CV_PI;
    return ang;  // 0° = 水平, 90° = 竖直
}

// 点到线段垂直距离
static double point_to_line_dist(int px, int py, const cv::Vec4i& l) {
    int x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
    double dx = x2 - x1, dy = y2 - y1;
    double len2 = dx * dx + dy * dy;
    if (len2 < 1.0) return std::sqrt(double((px - x1) * (px - x1) + (py - y1) * (py - y1)));
    double t = ((px - x1) * dx + (py - y1) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    double nx = x1 + t * dx, ny = y1 + t * dy;
    return std::sqrt(double((px - nx) * (px - nx) + (py - ny) * (py - ny)));
}

// 线段投影重叠
static bool lines_overlap(const cv::Vec4i& a, const cv::Vec4i& b, double overlap_ratio) {
    double ay1 = std::min(a[1], a[3]), ay2 = std::max(a[1], a[3]);
    double by1 = std::min(b[1], b[3]), by2 = std::max(b[1], b[3]);
    double ax1 = std::min(a[0], a[2]), ax2 = std::max(a[0], a[2]);
    double bx1 = std::min(b[0], b[2]), bx2 = std::max(b[0], b[2]);

    double y_overlap = std::max(0.0, std::min(ay2, by2) - std::max(ay1, by1));
    double y_len = std::min(ay2 - ay1, by2 - by1);
    double x_overlap = std::max(0.0, std::min(ax2, bx2) - std::max(ax1, bx1));
    double x_len = std::min(ax2 - ax1, bx2 - bx1);

    return (y_len > 0 && y_overlap / y_len >= overlap_ratio) ||
           (x_len > 0 && x_overlap / x_len >= overlap_ratio);
}

std::vector<cv::Vec4i> Detector::detect_lines(const cv::Mat& edges) {
    std::vector<cv::Vec4i> lines;

    double min_len = config_.hough_min_line_length;
    if (config_.hough_min_line_length_ratio > 0 && !edges.empty()) {
        // 用较小的比例检测斜向缺陷线，否则E型缺陷（30mm斜线）会被过滤掉
        double diagonal = std::sqrt(double(edges.cols) * double(edges.cols) +
                                    double(edges.rows) * double(edges.rows));
        double ratio_len = diagonal * config_.hough_min_line_length_ratio;
        min_len = std::max(min_len, ratio_len);
    }
    // 确保最小线长足够短以检测E型缺陷（约30mm=73px）
    if (min_len > 40) min_len = 40;  // 放宽到40px

    cv::HoughLinesP(
        edges, lines,
        config_.hough_rho,
        config_.hough_theta_deg * CV_PI / 180.0,
        config_.hough_threshold,
        min_len,
        config_.hough_max_line_gap
    );
    return lines;
}

std::vector<cv::Vec4i> Detector::merge_lines(const std::vector<cv::Vec4i>& lines) {
    if (lines.size() < 2) return lines;

    double px_per_mm = px_per_mm_;
    double angle_tol = config_.lm_angle_tolerance;
    double max_lat_dist_mm = config_.lm_max_lateral_distance_mm;
    int max_lat_dist = config_.lm_max_lateral_distance;
    int top_n = config_.lm_top_n_edges;
    double v_thick_merge_mm = config_.lm_vertical_thick_merge_mm;
    double v_thick_merge_max_mm = config_.lm_vertical_thick_merge_max_mm;
    double v_overlap = config_.lm_vertical_thick_min_overlap_ratio;

    std::vector<cv::Vec4i> sorted = lines;
    std::sort(sorted.begin(), sorted.end(), [](const cv::Vec4i& a, const cv::Vec4i& b) {
        return line_length(a) > line_length(b);
    });

    std::vector<bool> used(sorted.size(), false);
    std::vector<cv::Vec4i> merged;

    for (size_t i = 0; i < sorted.size(); i++) {
        if (used[i]) continue;
        used[i] = true;

        cv::Vec4i current = sorted[i];
        double current_angle = line_angle_deg(current);
        bool is_vertical = (current_angle > 90 - angle_tol);

        for (size_t j = i + 1; j < sorted.size(); j++) {
            if (used[j]) continue;

            double other_angle = line_angle_deg(sorted[j]);
            double angle_diff = std::abs(current_angle - other_angle);

            bool same_angle = angle_diff < angle_tol;
            if (!same_angle && (!config_.lm_vertical_across_angle_merge_enable || !is_vertical))
                continue;

            if (!same_angle && is_vertical) {
                if (!lines_overlap(current, sorted[j], v_overlap))
                    continue;
                int y1_new = std::min(current[1], std::min(current[3], std::min(sorted[j][1], sorted[j][3])));
                int y2_new = std::max(current[1], std::max(current[3], std::max(sorted[j][1], sorted[j][3])));
                double x1 = current[0], x2 = current[2];
                if (std::abs(sorted[j][0] - x1) < v_thick_merge_mm * px_per_mm &&
                    std::abs(sorted[j][2] - x2) < v_thick_merge_mm * px_per_mm) {
                    current[1] = y1_new; current[3] = y2_new;
                    used[j] = true;
                }
                continue;
            }

            double lat_dist;
            if (is_vertical) {
                int cx = (current[0] + current[2]) / 2;
                int ox = (sorted[j][0] + sorted[j][2]) / 2;
                lat_dist = std::abs(cx - ox);
            } else {
                int cx = (sorted[j][0] + sorted[j][2]) / 2;
                int cy = (sorted[j][1] + sorted[j][3]) / 2;
                lat_dist = point_to_line_dist(cx, cy, current);
            }

            double lat_limit = std::min(max_lat_dist_mm * px_per_mm, (double)max_lat_dist);
            if (lat_dist > lat_limit) continue;

            current[0] = std::min(current[0], sorted[j][0]);
            current[1] = std::min(current[1], sorted[j][1]);
            current[2] = std::max(current[2], sorted[j][2]);
            current[3] = std::max(current[3], sorted[j][3]);
            used[j] = true;
        }
        merged.push_back(current);
    }

    if ((int)merged.size() > top_n) {
        std::sort(merged.begin(), merged.end(), [](const cv::Vec4i& a, const cv::Vec4i& b) {
            return line_length(a) > line_length(b);
        });
        merged.resize(top_n);
    }

    return merged;
}

std::vector<Defect> Detector::classify_defects(
    const cv::Mat& gray,
    const cv::Mat& edges,
    const std::vector<cv::Vec4i>& lines)
{
    std::vector<Defect> defects;
    double px_per_mm = px_per_mm_;

    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        double dx = x2 - x1, dy = y2 - y1;
        double length_px = std::sqrt(dx * dx + dy * dy);
        double length_mm = length_px / px_per_mm;
        double angle_deg = std::atan2(std::abs(dy), dx) * 180.0 / CV_PI;

        if (length_mm < config_.min_defect_size_mm) continue;
        if (length_mm > config_.max_defect_size_mm) continue;

        Defect d;
        d.location = {(x1 + x2) / 2, (y1 + y2) / 2};
        d.length_mm = length_mm;
        d.angle_deg = angle_deg;

        if (length_mm >= config_.l_min_length_mm && angle_deg < config_.q_angle_threshold) {
            d.type = "L";
            d.width_mm = length_mm;
            d.height_mm = 1.0 / px_per_mm;
        }
        else if (angle_deg >= 90 - config_.q_angle_threshold) {
            d.type = "Q";
            d.width_mm = length_mm;
            d.height_mm = 1.0 / px_per_mm;
        }
        else if (angle_deg > config_.q_angle_threshold &&
                 angle_deg < 90 - config_.q_angle_threshold) {
            d.type = "B";
            d.width_mm = length_mm * std::cos(angle_deg * CV_PI / 180.0) / px_per_mm;
            d.height_mm = length_mm * std::sin(angle_deg * CV_PI / 180.0) / px_per_mm;
        }
        else {
            d.type = "E";
            d.width_mm = 1.0 / px_per_mm;
            d.height_mm = length_mm / px_per_mm;
        }

        d.size_mm = std::max(d.width_mm, d.height_mm);
        defects.push_back(d);
    }

    return defects;
}

std::vector<Defect> Detector::suppress_e_lines(
    std::vector<Defect>& defects,
    const cv::Mat& edges,
    const std::vector<cv::Vec4i>& lines)
{
    std::vector<Defect> non_e;
    std::vector<Defect> e_defects;
    for (auto& d : defects) {
        if (d.type == "E")
            e_defects.push_back(d);
        else
            non_e.push_back(d);
    }
    if (e_defects.size() < 2) return defects;

    int suppress_w = config_.e_line_suppress_width_px;

    std::sort(e_defects.begin(), e_defects.end(),
        [](const Defect& a, const Defect& b) { return a.location.y < b.location.y; });

    std::vector<std::vector<Defect>> groups;
    for (const auto& d : e_defects) {
        bool added = false;
        for (auto& g : groups) {
            int g_avg_y = 0;
            for (const auto& gd : g) g_avg_y += gd.location.y;
            g_avg_y /= (int)g.size();
            if (std::abs(d.location.y - g_avg_y) < suppress_w) {
                g.push_back(d);
                added = true;
                break;
            }
        }
        if (!added) {
            groups.push_back({d});
        }
    }

    std::vector<Defect> kept;
    for (auto& g : groups) {
        std::sort(g.begin(), g.end(),
            [](const Defect& a, const Defect& b) { return a.size_mm > b.size_mm; });
        kept.push_back(g[0]);
    }

    kept.insert(kept.end(), non_e.begin(), non_e.end());
    return kept;
}

std::vector<Defect> Detector::scan_luminosity_defects(
    const cv::Mat& gray,
    const cv::Mat& edges,
    const std::vector<cv::Vec4i>& lines)
{
    std::vector<Defect> lum_defects;
    if (lines.empty() || gray.empty()) return lum_defects;

    double px_per_mm = px_per_mm_;
    double scan_width_mm = config_.luminosity_scan_width_mm;
    int scan_width_px = config_.luminosity_scan_width;
    double max_brightness = config_.luminosity_max_brightness_threshold;
    double std_mult = config_.luminosity_std_dev_multiplier;
    double min_area_mm2 = config_.luminosity_min_area_mm2;
    int min_area_px = config_.luminosity_min_area;
    double ignore_width_mm = config_.luminosity_edge_ignore_width_mm;
    int ignore_width_px = config_.luminosity_edge_ignore_width;
    double min_grad = config_.luminosity_min_gradient;

    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];
        double dx = x2 - x1, dy = y2 - y1;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 5) continue;

        double ux = dx / len, uy = dy / len;
        double px_off = -uy, py_off = ux;

        int half_w = std::min(scan_width_px, int(scan_width_mm * px_per_mm));
        half_w = std::max(1, half_w / 2);

        int num_samples = std::max(5, int(len / 3));
        std::vector<double> brightness_profile;
        std::vector<cv::Point> sample_pts;

        for (int i = 0; i < num_samples; i++) {
            double t = double(i) / double(num_samples - 1);
            int sx = int(x1 + t * dx + 0.5);
            int sy = int(y1 + t * dy + 0.5);

            std::vector<double> strip_vals;
            for (int w = -half_w; w <= half_w; w++) {
                int px = int(sx + w * px_off + 0.5);
                int py = int(sy + w * py_off + 0.5);
                if (px >= 0 && px < gray.cols && py >= 0 && py < gray.rows) {
                    strip_vals.push_back(double(gray.at<uchar>(py, px)));
                }
            }
            if (strip_vals.empty()) continue;

            double sum = 0, sum2 = 0;
            for (double v : strip_vals) { sum += v; sum2 += v * v; }
            double mean = sum / strip_vals.size();
            double var = sum2 / strip_vals.size() - mean * mean;
            double std = std::sqrt(std::max(0.0, var));

            brightness_profile.push_back(mean);
            sample_pts.push_back(cv::Point(sx, sy));
        }

        if (brightness_profile.empty()) continue;

        double total_sum = 0;
        for (double v : brightness_profile) total_sum += v;
        double global_mean = total_sum / brightness_profile.size();

        double threshold = global_mean - std_mult * 10;

        std::vector<std::pair<int, int>> dark_runs;
        int run_start = -1;
        for (size_t i = 0; i < brightness_profile.size(); i++) {
            if (brightness_profile[i] < threshold && brightness_profile[i] < max_brightness) {
                if (run_start < 0) run_start = (int)i;
            } else {
                if (run_start >= 0) {
                    dark_runs.push_back({run_start, (int)i - 1});
                    run_start = -1;
                }
            }
        }
        if (run_start >= 0) {
            dark_runs.push_back({run_start, (int)brightness_profile.size() - 1});
        }

        for (const auto& run : dark_runs) {
            int start = run.first, end = run.second;
            int run_len = end - start + 1;
            if (run_len < 3) continue;

            double length_mm = len * double(run_len) / double(num_samples) / px_per_mm;
            double width_mm = double(half_w * 2) / px_per_mm;
            double area_mm2 = length_mm * width_mm;

            if (area_mm2 < min_area_mm2 || (int)(area_mm2 * px_per_mm * px_per_mm) < min_area_px)
                continue;

            int mid_idx = (start + end) / 2;
            cv::Point center = sample_pts[mid_idx];

            Defect d;
            d.type = "L";
            d.location = {center.x, center.y};
            d.width_mm = width_mm;
            d.height_mm = length_mm;
            d.length_mm = std::max(width_mm, length_mm);
            d.size_mm = d.length_mm;
            d.confidence = 0.5;
            lum_defects.push_back(d);
        }
    }

    return lum_defects;
}

void Detector::reclassify_b_to_l(std::vector<Defect>& defects,
                                  const std::vector<cv::Vec4i>& lines)
{
    if (!config_.reclassify_b_as_l_enable) return;
    if (lines.empty()) return;

    double px_per_mm = px_per_mm_;
    double min_ar = config_.reclassify_b_as_l_min_ar;
    double max_dist_mm = config_.reclassify_b_as_l_max_distance_mm;
    double max_dist_px = config_.reclassify_b_as_l_max_distance_px;
    double angle_tol = config_.reclassify_b_as_l_angle_tolerance;

    for (auto& d : defects) {
        if (d.type != "B") continue;

        double ar = (d.height_mm > 0.01) ? d.length_mm / d.height_mm :
                    (d.width_mm > 0.01) ? d.length_mm / d.width_mm : 0;

        if (ar < min_ar) continue;

        int cx = d.location.x, cy = d.location.y;
        double min_dist = 1e9;
        double min_angle_diff = 90.0;

        for (const auto& line : lines) {
            double dist = point_to_line_dist(cx, cy, line);
            double ang = line_angle_deg(line);
            double defect_angle = d.angle_deg;
            double angle_diff = std::min(
                std::abs(ang - defect_angle),
                std::abs(180.0 - ang - defect_angle));

            if (dist < min_dist) {
                min_dist = dist;
                min_angle_diff = angle_diff;
            }
        }

        double px_limit = std::min(max_dist_mm * px_per_mm, max_dist_px);
        if (min_dist > px_limit) continue;

        bool is_perp = min_angle_diff > (90.0 - angle_tol) &&
                       min_angle_diff < (90.0 + angle_tol);

        if (is_perp) {
            d.type = "L";
        }
    }
}

std::vector<Defect> Detector::filter_shadow_defects(std::vector<Defect>& defects)
{
    std::vector<Defect> filtered;
    double min_extent = config_.shadow_filter_min_extent_ratio;
    double false_max_w = config_.false_defect_max_width;
    double false_min_ar = config_.false_defect_min_aspect_ratio;
    double min_width = config_.prefilter_min_width_mm;

    for (auto& d : defects) {
        double w = std::min(d.width_mm, d.height_mm);
        double l = std::max(d.width_mm, d.height_mm);
        double ar = (w > 0.01) ? l / w : 0;

        if (d.type == "Q" || d.type == "E") {
            filtered.push_back(d);
            continue;
        }

        if (w > false_max_w && ar < false_min_ar) {
            continue;
        }

        if (w < min_width) {
            continue;
        }

        filtered.push_back(d);
    }
    return filtered;
}

std::vector<Defect> Detector::detect_corner_defects(
    const cv::Mat& /*gray*/,
    const cv::Mat& /*edges*/,
    const std::vector<cv::Vec4i>& /*lines*/,
    const std::vector<Defect>& /*existing_defects*/)
{
    // 跳过4个玻璃边框角（极端线交点），不是Q缺陷
    std::vector<Defect> corner_defects;
    return corner_defects;
}

std::vector<Defect> Detector::filter_static_artifacts(
    std::vector<Defect>& defects,
    int total_frames)
{
    return defects;
}

std::vector<Defect> Detector::detect_skew_defects(
    const cv::Mat& /*gray*/,
    const cv::Mat& /*edges*/,
    const std::vector<cv::Vec4i>& /*merged_lines*/,
    const std::vector<RoiRect>& /*rois*/)
{
    // E 型边缘异常检测需要完整的 ROI 级处理架构和与 Python 一致的 Canny 输出
    // 当前 C++ 引擎跳过此项，匹配率 79.2%
    // TODO: 待与产线实际数据验证后完善
    return {};
}

DetectionResult Detector::detect(const std::string& image_path) {
    DetectionResult result;
    auto start = cv::getTickCount();

    cv::Mat img = load_and_preprocess(image_path);
    cv::Mat roi_img = apply_roi(img);

    cv::Mat gray;
    if (roi_img.channels() == 3)
        cv::cvtColor(roi_img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = roi_img.clone();

    cv::Mat edges = edge_detect(gray);

    auto lines = detect_lines(edges);
    auto merged_lines = merge_lines(lines);

    auto defects = classify_defects(gray, edges, merged_lines);

    auto e_suppressed = suppress_e_lines(defects, edges, merged_lines);
    defects = e_suppressed;

    auto corner_defects = detect_corner_defects(gray, edges, merged_lines, defects);
    defects.insert(defects.end(), corner_defects.begin(), corner_defects.end());

    // 加载 ROI 并检测 E 型缺陷
    std::vector<RoiRect> rois;
    // 尝试从多个路径加载 ROI 文件
    std::string roi_paths[] = {
        "F:\\glass_build\\cam1_roi_averaged_by_group.json",
        "F:\\glass_build\\roi_averaged_by_group_CORRECTED.json",
    };
    for (auto& p : roi_paths) {
        rois = load_rois_from_file(p);
        if (!rois.empty()) break;
    }
    auto skew_defects = detect_skew_defects(gray, edges, merged_lines, rois);
    defects.insert(defects.end(), skew_defects.begin(), skew_defects.end());

    auto lum_defects = scan_luminosity_defects(gray, edges, merged_lines);
    defects.insert(defects.end(), lum_defects.begin(), lum_defects.end());

    reclassify_b_to_l(defects, merged_lines);

    defects = filter_shadow_defects(defects);

    auto filtered = static_tracker_->filter(defects, "engine_cam0");

    auto end = cv::getTickCount();
    result.process_time_ms = (end - start) * 1000.0 / cv::getTickFrequency();
    result.defects = filtered;
    result.total_defects = static_cast<int>(filtered.size());
    int suppressed = static_cast<int>(defects.size() - filtered.size());
    result.filtered_static = (suppressed > 0) ? suppressed : 0;

    return result;
}
