#include "engine/detector.h"
#include "engine/preprocess.h"
#include "engine/line_merge.h"
#include "engine/classify.h"
#include "engine/skew.h"
#include "engine/luminosity.h"
#include "engine/corner.h"
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

namespace {

// 缺陷标注绘制（镜像 Python _draw_defect_annotations，image_processor_hough.py:6013）。
// defects 坐标为全图坐标（process_roi 已平移还原）；文本用 ASCII（cv::putText 不支持中文，
// 中文标注需 FreeType 字体，暂以英文标签替代，格式与 Python 一致）。
cv::Mat draw_defect_annotations(const cv::Mat& gray,
                                const std::vector<Defect>& defects,
                                const std::vector<RoiRect>& rois,
                                double alpha) {
    cv::Mat roi_bgr;
    cv::cvtColor(gray, roi_bgr, cv::COLOR_GRAY2BGR);
    const double beta = 1.0 - alpha;
    const int THICKNESS = 1;

    auto color_of = [](const std::string& type) -> cv::Scalar {
        if (type == "Q" || type == "E") return cv::Scalar(0, 0, 255);      // 红
        if (type == "X") return cv::Scalar(255, 0, 0);                     // 蓝
        if (type == "L") return cv::Scalar(255, 0, 255);                   // 品红
        if (type == "B") return cv::Scalar(0, 165, 255);                   // 橙
        return cv::Scalar(255, 255, 255);
    };

    for (const auto& d : defects) {
        const cv::Scalar color = color_of(d.type);

        // Q：射线箭头
        if (d.type == "Q" && !d.ray_segments.empty()) {
            for (const auto& seg : d.ray_segments) {
                cv::arrowedLine(roi_bgr, seg.first, seg.second, cv::Scalar(0, 255, 255), 1,
                                cv::LINE_8, 0, 0.25);
            }
        }

        if (d.type == "Q" && !d.region_contour.empty()) {
            std::vector<std::vector<cv::Point>> poly{d.region_contour};
            cv::Mat overlay = roi_bgr.clone();
            cv::fillPoly(overlay, poly, color);
            cv::addWeighted(overlay, alpha, roi_bgr, beta, 0, roi_bgr);
            cv::drawContours(roi_bgr, poly, 0, color, THICKNESS);
            if (!d.barrier_contour.empty()) {
                cv::polylines(roi_bgr, d.barrier_contour, true, cv::Scalar(0, 255, 255), 1);
            }
        } else if (d.type == "X" && d.center.has_value()) {
            cv::circle(roi_bgr, *d.center, 15, color, THICKNESS);
        } else if ((d.type == "L" || d.type == "B" || d.type == "X" || d.type == "E") &&
                   !d.box_points.empty()) {
            std::vector<std::vector<cv::Point>> poly{d.box_points};
            cv::Mat overlay = roi_bgr.clone();
            cv::fillPoly(overlay, poly, color);
            cv::addWeighted(overlay, alpha, roi_bgr, beta, 0, roi_bgr);
            cv::drawContours(roi_bgr, poly, 0, color, THICKNESS);
        }

        // 文本标注（ROI 右上角，向下堆叠；镜像 Python 文本内容，英文标签）
        char buf[160];
        if (d.type == "E" || d.type == "X") {
            const char* angle_kind = (d.subtype == "curved" || d.skew_subtype == "curved") ? "curv" : "ang";
            std::snprintf(buf, sizeof(buf), "%s: (%d,%d), %s %.1f deg, size %.1fx%.1f mm",
                          d.type.c_str(), d.x, d.y, angle_kind, d.angle_deg, d.length_mm, d.width_mm);
        } else {
            std::snprintf(buf, sizeof(buf), "%s: (%d,%d), size %.1fx%.1f mm",
                          d.type.c_str(), d.x, d.y, d.length_mm, d.width_mm);
        }
        const std::string text(buf);

        int base_x = 0, base_y = 10, roi_w = (int)roi_bgr.cols;
        if (d.roi_index >= 0 && d.roi_index < (int)rois.size()) {
            base_x = rois[d.roi_index].x;
            base_y = rois[d.roi_index].y + 10;
            roi_w = rois[d.roi_index].width;   // 文本放在 ROI 右上角（镜像 Python）
        }
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.5;
        int thickness = 1;
        int baseline = 0;
        cv::Size ts = cv::getTextSize(text, font, scale, thickness, &baseline);
        int x_text = std::max(0, base_x + roi_w - ts.width - 10);
        int y_text = base_y;
        cv::putText(roi_bgr, text, cv::Point(x_text, y_text), font, scale, color, thickness, cv::LINE_AA);
        (void)baseline;
    }
    return roi_bgr;
}

#ifdef _WIN32
// UTF-8 路径安全保存（镜像 load_image 的 UTF-16 打开方式；cv::imwrite 不支持中文路径）
bool save_image_utf8(const std::string& path, const cv::Mat& img) {
    int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
    if (wlen <= 0) return false;
    std::wstring wpath(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, &wpath[0], wlen);
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", img, buf)) return false;
    FILE* fp = nullptr;
    if (_wfopen_s(&fp, wpath.c_str(), L"wb") != 0 || !fp) return false;
    size_t written = fwrite(buf.data(), 1, buf.size(), fp);
    fclose(fp);
    return written == buf.size();
}
#endif

} // namespace

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
        // 镜像 Python round()/np.round()：银行家舍入（.5 时取偶数），避免 0.5 边界 1px 差异
        auto round_banker = [](double x) -> int {
            double r = std::floor(x);
            double frac = x - r;
            if (frac > 0.5 || (frac == 0.5 && std::fmod(r, 2.0) != 0.0)) r += 1.0;
            return (int)r;
        };
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
            int w_small = std::max(1, round_banker(w_full * ds_scale));
            int h_small = std::max(1, round_banker(h_full * ds_scale));
            cv::Mat edges_small;
            cv::resize(edges, edges_small, cv::Size(w_small, h_small), 0, 0, cv::INTER_AREA);
            double scale_x = (double)w_full / w_small;
            double scale_y = (double)h_full / h_small;
            double scale_len = std::min(1.0, std::min(w_small / (double)std::max(1, w_full),
                                                      h_small / (double)std::max(1, h_full)));
            int min_len_small = std::max(1, round_banker(min_len * scale_len));
            double max_gap_full = params.hough.max_line_gap_mm > 0
                ? params.hough.max_line_gap_mm * px_per_mm_ : params.hough.max_line_gap;
            int max_gap_small = std::max(0, round_banker(max_gap_full * scale_len));
            std::vector<cv::Vec4i> raw_small;
            cv::HoughLinesP(edges_small, raw_small,
                            params.hough.rho,
                            params.hough.theta_deg * CV_PI / 180.0,
                            params.hough.threshold,
                            min_len_small,
                            max_gap_small);
            // 坐标放大回原图（银行家舍入，镜像 Python np.round().astype(int32)）
            for (auto& l : raw_small) {
                l[0] = round_banker(l[0] * scale_x);
                l[1] = round_banker(l[1] * scale_y);
                l[2] = round_banker(l[2] * scale_x);
                l[3] = round_banker(l[3] * scale_y);
            }
            lines = raw_small;
        } else {
            double max_gap_full = params.hough.max_line_gap_mm > 0
                ? params.hough.max_line_gap_mm * px_per_mm_ : params.hough.max_line_gap;
            int min_len_i = std::max(1, round_banker(min_len));
            int max_gap_i = std::max(0, round_banker(max_gap_full));
            cv::HoughLinesP(edges, lines,
                            params.hough.rho,
                            params.hough.theta_deg * CV_PI / 180.0,
                            params.hough.threshold,
                            min_len_i,
                            max_gap_i);
        }
    }

    // 直线合并
    auto merged = merge_lines_and_get_main_edges(lines, params, px_per_mm_, edges);

#ifdef CPP_DEBUG_RAW
    std::cerr << "[RAW] roi=" << roi_idx << " n=" << lines.size() << std::endl;
    for (auto& l : lines) {
        std::cerr << "  (" << l[0] << "," << l[1] << ")->(" << l[2] << "," << l[3]
                  << ") len=" << std::hypot(l[2]-l[0], l[3]-l[1]) << std::endl;
    }
#endif

#ifdef CPP_DEBUG_MERGED
    for (auto& ml : merged) {
        std::cerr << "[MERGED] roi=" << roi_idx << " (" << ml.line[0] << "," << ml.line[1]
                  << ")->(" << ml.line[2] << "," << ml.line[3] << ") ang=" << ml.angle_deg
                  << " len=" << ml.length_px << std::endl;
    }
#endif

    // 缺陷来源 1：亮度扫描（沿主边扫描暗区 → B 崩边候选，镜像 Python scan_edge_for_luminosity_defects）
    // Python 流程：收集所有主边的扫描轮廓 -> 填充画布 -> MORPH_CLOSE 闭运算合并 -> 重新 findContours
    // -> 面积过滤 -> 逐轮廓生成 B。此处 C++ 同步镜像，否则相邻小轮廓会各自成 B，与 Python 不一致。
    // Vertical edge extension to ROI boundary (mirror Python ENABLE_VERTICAL_EXTENSION_FOR_Q + _clip_infinite_line_to_roi)
    {
        double v_ext_min_len_px = 5.0 * px_per_mm_;
        double v_tol = params.defect_detection.vertical_angle_tol_deg;
        for (auto& ml : merged) {
            double dx = ml.line[2] - ml.line[0];
            double dy = ml.line[3] - ml.line[1];
            double len = std::hypot(dx, dy);
            if (len < v_ext_min_len_px) continue;
            double ang = std::abs(std::atan2(dy, dx)) * 180.0 / CV_PI;
            if (ang > 90.0) ang = 180.0 - ang;
            if (ang >= (90.0 - v_tol)) {
                double x1 = ml.line[0], y1 = ml.line[1];
                double x2 = ml.line[2], y2 = ml.line[3];
                double ddx = x2 - x1, ddy = y2 - y1;
                if (!(std::abs(ddx) < 1e-9 && std::abs(ddy) < 1e-9)) {
                    std::vector<cv::Point2d> cand;
                    double W = (double)(roi_gray.cols - 1);
                    double H = (double)(roi_gray.rows - 1);
                    if (std::abs(ddx) >= 1e-9) {
                        for (double xk : {0.0, W}) {
                            double t = (xk - x1) / ddx;
                            double yk = y1 + t * ddy;
                            if (yk >= 0.0 && yk <= H) cand.push_back(cv::Point2d(xk, yk));
                        }
                    }
                    if (std::abs(ddy) >= 1e-9) {
                        for (double yk : {0.0, H}) {
                            double t = (yk - y1) / ddy;
                            double xk = x1 + t * ddx;
                            if (xk >= 0.0 && xk <= W) cand.push_back(cv::Point2d(xk, yk));
                        }
                    }
                    std::vector<cv::Point2d> uniq;
                    for (auto& p : cand) {
                        bool dup = false;
                        for (auto& q : uniq) {
                            if (std::abs(p.x - q.x) < 0.5 && std::abs(p.y - q.y) < 0.5) { dup = true; break; }
                        }
                        if (!dup) uniq.push_back(p);
                    }
                    if (uniq.size() >= 2) {
                        int ia = 0, ib = 1;
                        double best = -1;
                        for (size_t a = 0; a < uniq.size(); a++) {
                            for (size_t b = a + 1; b < uniq.size(); b++) {
                                double d = std::hypot(uniq[b].x - uniq[a].x, uniq[b].y - uniq[a].y);
                                if (d > best) { best = d; ia = (int)a; ib = (int)b; }
                            }
                        }
                        ml.line = cv::Vec4d(uniq[ia].x, uniq[ia].y, uniq[ib].x, uniq[ib].y);
                        ml.length_px = std::hypot(ml.line[2] - ml.line[0], ml.line[3] - ml.line[1]);
                        double na = std::abs(std::atan2(ml.line[3] - ml.line[1], ml.line[2] - ml.line[0])) * 180.0 / CV_PI;
                        ml.angle_deg = na > 90.0 ? 180.0 - na : na;
                    }
                }
            }
        }
    }

    std::vector<Defect> defects;
    {
        std::vector<std::vector<cv::Point>> all_chipping_contours;
        for (auto& ml : merged) {
            auto contours = scan_edge_for_luminosity_defects(roi_gray, ml.line, params, px_per_mm_);
            for (auto& cnt : contours) all_chipping_contours.push_back(cnt);
        }
        if (!all_chipping_contours.empty()) {
            cv::Mat defect_canvas = cv::Mat::zeros(roi_gray.size(), CV_8U);
            cv::drawContours(defect_canvas, all_chipping_contours, -1, cv::Scalar(255), -1);

            // MORPH_CLOSE 核尺寸：优先 MERGE_DEFECTS_KERNEL_MM * px_per_mm 转奇数，否则用 KERNEL_SIZE
            int kx = params.defect_detection.merge_defects_kernel_size.size() > 0
                ? params.defect_detection.merge_defects_kernel_size[0] : 9;
            int ky = params.defect_detection.merge_defects_kernel_size.size() > 1
                ? params.defect_detection.merge_defects_kernel_size[1] : 9;
            if (params.defect_detection.merge_defects_kernel_mm.size() >= 2) {
                int kx_mm = (int)std::lround(params.defect_detection.merge_defects_kernel_mm[0] * px_per_mm_);
                int ky_mm = (int)std::lround(params.defect_detection.merge_defects_kernel_mm[1] * px_per_mm_);
                if (kx_mm >= 1) kx = (kx_mm % 2 == 0) ? kx_mm + 1 : kx_mm;
                if (ky_mm >= 1) ky = (ky_mm % 2 == 0) ? ky_mm + 1 : ky_mm;
            }
            cv::Mat kernel = cv::Mat::ones(kx, ky, CV_8U);
            cv::Mat merged_mask;
            cv::morphologyEx(defect_canvas, merged_mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

            std::vector<std::vector<cv::Point>> final_contours;
            cv::findContours(merged_mask, final_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double min_area_px2 = params.defect_detection.luminosity_min_area_mm2 > 0
                ? params.defect_detection.luminosity_min_area_mm2 * px_per_mm_ * px_per_mm_
                : params.defect_detection.luminosity_min_area;

            for (auto& cnt : final_contours) {
                if (std::abs(cv::contourArea(cnt)) < min_area_px2) continue;
                cv::RotatedRect rr = cv::minAreaRect(cnt);
                cv::Point2f pts[4];
                rr.points(pts);
                Defect d;
                d.type = "B";
                d.confidence = 0.65;
                for (auto& pt : pts)
                    d.box_points.push_back(cv::Point((int)std::lround(pt.x), (int)std::lround(pt.y)));
                d.region_contour = cnt;   // 保留合并后轮廓（阴影过滤用）
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
    }
    for (auto& d : defects) d.roi_index = roi_idx;

    // B 过滤链（镜像 Python final_chipping_defects / report 阶段 B 规则）
    {
        std::vector<Defect> b_filtered;
        for (auto& d : defects) {
            if (d.type != "B") { b_filtered.push_back(d); continue; }
            // 从 box_points 重算 minAreaRect
            cv::RotatedRect rr = cv::minAreaRect(d.box_points);
            double w_px = std::max(rr.size.width, rr.size.height);
            double h_px = std::min(rr.size.width, rr.size.height);
            double width_mm = h_px / px_per_mm_;
            double length_mm = w_px / px_per_mm_;
            double ar = h_px > 1e-6 ? w_px / h_px : 1e9;

            // 1) 细长窄门控：ar>10 且宽<2mm → 距主边 >5mm 过滤
            double ar_min_gate = params.defect_detection.b_filter_parallel_ar_min > 0
                ? params.defect_detection.b_filter_parallel_ar_min : 10.0;
            double min_side_gate = params.defect_detection.b_filter_parallel_min_side_mm > 0
                ? params.defect_detection.b_filter_parallel_min_side_mm : 2.0;
            double max_edge_dist = params.defect_detection.b_max_distance_to_edge_mm > 0
                ? params.defect_detection.b_max_distance_to_edge_mm : 5.0;
            if (ar > ar_min_gate && width_mm < min_side_gate) {
                cv::Point2d center(rr.center.x, rr.center.y);
                double min_d = 1e18;
                for (auto& ml : merged) {
                    double dpx = std::abs((ml.line[2]-ml.line[0]) * (ml.line[1]-center.y) -
                                          (ml.line[3]-ml.line[1]) * (ml.line[0]-center.x)) /
                                 std::max(1e-6, ml.length_px);
                    min_d = std::min(min_d, dpx);
                }
                if (min_d > max_edge_dist * px_per_mm_) continue; // 距主边过远，过滤
            }
            // 2) 阴影过滤：contour 面积 / minAreaRect 面积 < SHADOW_FILTER_MIN_EXTENT_RATIO
            if (!d.region_contour.empty()) {
                double contour_area = std::abs(cv::contourArea(d.region_contour));
                double rect_area = w_px * h_px;
                double min_extent = params.defect_detection.shadow_filter_min_extent_ratio > 0
                    ? params.defect_detection.shadow_filter_min_extent_ratio : 0.25;
                if (rect_area > 1e-6 && (contour_area / rect_area) < min_extent) continue;
            }
            // 3) MIN_WIDTH 过滤（未转 L 的 B）：宽 < MIN_WIDTH_MM 过滤
            double min_width_rule = params.defect_detection.min_width_mm > 0
                ? params.defect_detection.min_width_mm : 5.0;
            if (width_mm < min_width_rule) continue;
            // 4) 面积/宽度启发式：面积<25mm2 且宽<5mm 过滤
            double area_mm2 = length_mm * width_mm;
            if (area_mm2 < 25.0 && width_mm < params.defect_detection.min_defect_size_mm) continue;

            // 5) B→L 重分类：ar>7.5 且长边近似垂直主边（角度差 > 90-tol）
            double min_ar_l = params.defect_detection.b_to_l_min_ar > 0
                ? params.defect_detection.b_to_l_min_ar : 7.5;
            double perp_tol = params.defect_detection.b_to_l_perp_tolerance_deg > 0
                ? params.defect_detection.b_to_l_perp_tolerance_deg : 10.0;
            if (ar > min_ar_l) {
                // 长边方向（minAreaRect 长边角度）
                double long_ang = rr.angle;
                if (w_px < h_px) long_ang += 90.0;
                long_ang = std::fmod(std::abs(long_ang), 180.0);
                if (long_ang > 90.0) long_ang = 180.0 - long_ang;
                bool is_perp = false;
                for (auto& ml : merged) {
                    // ml.angle_deg 为 [0,180) 真实角度，先折叠到 [0,90] 再比较
                    double mla = std::fmod(std::abs(ml.angle_deg), 180.0);
                    if (mla > 90.0) mla = 180.0 - mla;
                    double ang_diff = std::abs(mla - long_ang);
                    ang_diff = std::min(ang_diff, 180.0 - ang_diff);
                    if (ang_diff >= (90.0 - perp_tol)) { is_perp = true; break; }
                }
                if (is_perp) d.type = "L";
            }
            d.width_mm = width_mm;
            d.length_mm = length_mm;
            d.size_mm = std::max(width_mm, length_mm);
            b_filtered.push_back(d);
        }
        defects = b_filtered;
    }

    // 缺陷来源 2：E 型边缘异常（缺陷边缘图 + 主边屏蔽带 + 轮廓分析）
    cv::Mat defect_edges = preprocess_for_defect_edges(roi_gray, params.preprocessing);
    auto e_defects = detect_e_defects(roi_gray, defect_edges, merged, params, px_per_mm_);
    for (auto& d : e_defects) d.roi_index = roi_idx;
    defects.insert(defects.end(), e_defects.begin(), e_defects.end());

    // 缺陷来源 3：Q 角点（缺角）检测（角点配对 + 射线求交 + 三角形判定）
    auto q_defects = detect_q_defects(roi_gray, merged, defect_edges, params, px_per_mm_);
    for (auto& d : q_defects) d.roi_index = roi_idx;
    defects.insert(defects.end(), q_defects.begin(), q_defects.end());

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

    // 选择参数（明场默认；暗场由请求注入 mode="dark"）
    const InspectorParams* params = (config_.mode == "dark") ? &config_.dark : &config_.light;

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
    DetectionResult result = detect(gray);

    // 标注图输出（镜像 Python：静态抑制后对保留缺陷重绘）
    if (config_.draw_defects && !config_.annotated_output_dir.empty()) {
        try {
            std::vector<RoiRect> rois = external_rois_;
            if (rois.empty() && !config_.light.roi_template_file.empty()) {
                rois = load_rois_from_file(config_.light.roi_template_file);
            }
            double alpha = config_.light.visualization.defect_overlay_alpha > 0
                ? config_.light.visualization.defect_overlay_alpha : 0.25;
            cv::Mat annotated = draw_defect_annotations(gray, result.defects, rois, alpha);

            // 输出文件名：<原文件名去扩展名>_annotated.jpg
            std::filesystem::path p(image_path);
            std::string base = p.stem().string();
            std::string out_path = config_.annotated_output_dir + "\\" + base + "_annotated.jpg";
#ifdef _WIN32
            if (!save_image_utf8(out_path, annotated)) {
                std::cerr << "[WARN] annotated image save failed: " << out_path << std::endl;
            }
#else
            if (!cv::imwrite(out_path, annotated)) {
                std::cerr << "[WARN] annotated image save failed: " << out_path << std::endl;
            }
#endif
            result.annotated_image_path = out_path;
        } catch (const std::exception& e) {
            std::cerr << "[WARN] draw_defects error: " << e.what() << std::endl;
        }
    }
    return result;
}
