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

namespace {

// 单 ROI：预处理 + Hough + 直线合并（镜像 Python process_roi_hough_based 的 Hough 段）
std::vector<MergedLine> hough_merge_roi(const cv::Mat& roi_gray, const InspectorParams& params, double px_per_mm) {
    cv::Mat edges = preprocess_for_hough_enhanced(roi_gray, params.preprocessing);
    std::vector<cv::Vec4i> lines;
    {
        auto round_banker = [](double x) -> int {
            double r = std::floor(x);
            double frac = x - r;
            if (frac > 0.5 || (frac == 0.5 && std::fmod(r, 2.0) != 0.0)) r += 1.0;
            return (int)r;
        };
        double min_len = params.hough.min_line_length;
        if (params.hough.min_line_length_ratio > 0) {
            double min_base = std::min(roi_gray.cols, roi_gray.rows);
            min_len = min_base * params.hough.min_line_length_ratio;
        }
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
                ? params.hough.max_line_gap_mm * px_per_mm : params.hough.max_line_gap;
            int max_gap_small = std::max(0, round_banker(max_gap_full * scale_len));
            std::vector<cv::Vec4i> raw_small;
            cv::HoughLinesP(edges_small, raw_small,
                            params.hough.rho,
                            params.hough.theta_deg * CV_PI / 180.0,
                            params.hough.threshold,
                            min_len_small,
                            max_gap_small);
            for (auto& l : raw_small) {
                l[0] = round_banker(l[0] * scale_x);
                l[1] = round_banker(l[1] * scale_y);
                l[2] = round_banker(l[2] * scale_x);
                l[3] = round_banker(l[3] * scale_y);
            }
            lines = raw_small;
        } else {
            double max_gap_full = params.hough.max_line_gap_mm > 0
                ? params.hough.max_line_gap_mm * px_per_mm : params.hough.max_line_gap;
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
#ifdef CPP_DEBUG_RAW
    {
        static int g_roi_counter = 0;
        std::string path = "E:\\dsh\\cpp_edges_roi" + std::to_string(g_roi_counter) + ".png";
        try { cv::imwrite(path, edges); } catch (...) {}
        std::cerr << "[EDGE] roi" << g_roi_counter << " nonzero=" << cv::countNonZero(edges)
                  << " total=" << edges.total() << std::endl;
        g_roi_counter++;
        std::cerr << "[RAW] n=" << lines.size() << std::endl;
        for (auto& l : lines) {
            std::cerr << "  (" << l[0] << "," << l[1] << ")->(" << l[2] << "," << l[3]
                      << ") len=" << std::hypot(l[2]-l[0], l[3]-l[1]) << std::endl;
        }
    }
#endif
    return merge_lines_and_get_main_edges(lines, params, px_per_mm, edges);
}

inline double fold_angle_deg_abs(double a) {
    a = std::fmod(std::abs(a), 180.0);
    return a <= 90.0 ? a : 180.0 - a;
}

// 无限直线裁剪到 ROI 局部矩形（镜像 Python _clip_infinite_line_to_roi_local，4835-4871）
bool clip_infinite_line_to_roi_local(const cv::Vec4d& seg, int w, int h, cv::Vec4d& out) {
    double x1 = seg[0], y1 = seg[1], x2 = seg[2], y2 = seg[3];
    double dx = x2 - x1, dy = y2 - y1;
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) return false;
    std::vector<cv::Point2d> cand;
    if (std::abs(dx) >= 1e-9) {
        for (double xk : {0.0, (double)(w - 1)}) {
            double t = (xk - x1) / dx;
            double yk = y1 + t * dy;
            if (yk >= 0.0 && yk <= (double)(h - 1)) cand.push_back({xk, yk});
        }
    }
    if (std::abs(dy) >= 1e-9) {
        for (double yk : {0.0, (double)(h - 1)}) {
            double t = (yk - y1) / dy;
            double xk = x1 + t * dx;
            if (xk >= 0.0 && xk <= (double)(w - 1)) cand.push_back({xk, yk});
        }
    }
    std::vector<cv::Point2d> uniq;
    for (auto& p : cand) {
        bool dup = false;
        for (auto& q : uniq)
            if (std::abs(p.x - q.x) < 0.5 && std::abs(p.y - q.y) < 0.5) { dup = true; break; }
        if (!dup) uniq.push_back(p);
    }
    if (uniq.size() < 2) return false;
    cv::Point2d a = uniq[0], b = uniq[1];
    double maxd = -1.0;
    for (size_t i = 0; i < uniq.size(); i++)
        for (size_t j = i + 1; j < uniq.size(); j++) {
            double d = std::hypot(uniq[j].x - uniq[i].x, uniq[j].y - uniq[i].y);
            if (d > maxd) { maxd = d; a = uniq[i]; b = uniq[j]; }
        }
    out = cv::Vec4d(a.x, a.y, b.x, b.y);
    return true;
}

// 跨 ROI 共享竖直边收集（镜像 Python process_image_from_memory_parallel 6375-6533）
std::vector<cv::Vec4d> collect_shared_vertical_edges(
    const std::vector<RoiRect>& rois,
    const std::vector<std::vector<MergedLine>>& per_roi_merged,
    const InspectorParams& params, double px_per_mm) {
    std::vector<cv::Vec4d> shared;
    const DefectDetectParams& dd = params.defect_detection;
    if (!dd.cross_roi_vertical_enabled) return shared;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    double min_len_px = dd.cross_roi_vertical_min_len_mm > 0
        ? dd.cross_roi_vertical_min_len_mm * px_per_mm : 5.0 * px_per_mm;
    double cluster_x = dd.cross_roi_vertical_cluster_xpx > 0 ? dd.cross_roi_vertical_cluster_xpx : 12.0;
    double slant_bias = dd.cross_roi_vertical_slant_bias;

    // 1) 各 ROI 合并主边中收集近竖直（全局坐标）
    std::vector<cv::Vec4d> cand_global;
    for (size_t i = 0; i < rois.size() && i < per_roi_merged.size(); i++) {
        const RoiRect& roi = rois[i];
        for (auto& ml : per_roi_merged[i]) {
            if (!ml.near_vertical) continue;
            if (ml.length_px < min_len_px) continue;
            cand_global.push_back(cv::Vec4d(ml.line[0] + roi.x, ml.line[1] + roi.y,
                                            ml.line[2] + roi.x, ml.line[3] + roi.y));
        }
    }
    if (cand_global.empty()) return shared;

    // 2) 按 x 中心聚类（镜像 Python：|xm - mean(group xs)| <= cluster_x）
    struct Cand { double xm, ang, len; cv::Vec4d seg; };
    std::vector<Cand> cands;
    for (auto& s : cand_global) {
        Cand c;
        c.xm = 0.5 * (s[0] + s[2]);
        c.seg = s;
        c.ang = fold_angle_deg_abs(std::atan2(s[3] - s[1], s[2] - s[0]) * 180.0 / CV_PI);
        c.len = std::hypot(s[2] - s[0], s[3] - s[1]);
        cands.push_back(c);
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.xm < b.xm; });
    std::vector<std::vector<Cand>> groups;
    for (auto& c : cands) {
        bool placed = false;
        for (auto& g : groups) {
            double mean_x = 0.0;
            for (auto& gc : g) mean_x += gc.xm;
            mean_x /= (double)g.size();
            if (std::abs(c.xm - mean_x) <= cluster_x) { g.push_back(c); placed = true; break; }
        }
        if (!placed) groups.push_back({c});
    }

    // 3) 每组 fitLine + 投影 + 得分（镜像 Python 6494-6524）
    struct Scored { double score; cv::Vec4d line; };
    std::vector<Scored> scored_lines;
    for (auto& g : groups) {
        std::vector<cv::Point2f> pts;
        double total_len = 0.0;
        double y_min = 1e18, y_max = -1e18;
        for (auto& c : g) {
            pts.emplace_back((float)c.seg[0], (float)c.seg[1]);
            pts.emplace_back((float)c.seg[2], (float)c.seg[3]);
            total_len += c.len;
            y_min = std::min(y_min, std::min(c.seg[1], c.seg[3]));
            y_max = std::max(y_max, std::max(c.seg[1], c.seg[3]));
        }
        if (y_max - y_min < 1.0) continue;
        cv::Vec4f fit;
        try { cv::fitLine(pts, fit, cv::DIST_L2, 0, 0.01, 0.01); }
        catch (...) { continue; }
        double vx = fit[0], vy = fit[1], x0 = fit[2], y0 = fit[3];
        double norm = std::hypot(vx, vy);
        if (norm < 1e-6) { vx = 0.0; vy = 1.0; }
        else { vx /= norm; vy /= norm; }
        double ang_fit = fold_angle_deg_abs(std::atan2(vy, vx) * 180.0 / CV_PI);
        double x_min, x_max;
        if (std::abs(vy) < 1e-3) {
            double mean_x = 0.0;
            for (auto& c : g) mean_x += c.xm;
            x_min = x_max = mean_x / (double)g.size();
        } else {
            x_min = x0 + vx / vy * (y_min - y0);
            x_max = x0 + vx / vy * (y_max - y0);
        }
        double ang_dev = std::max(0.0, std::abs(90.0 - ang_fit));
        double score = total_len * (1.0 + slant_bias * (ang_dev / std::max(1e-3, v_tol)));
        scored_lines.push_back({score, cv::Vec4d(x_min, y_min, x_max, y_max)});
    }

    // 4) 按得分排序 + x 去重（镜像 Python 6526-6532）
    std::sort(scored_lines.begin(), scored_lines.end(),
              [](const Scored& a, const Scored& b) { return a.score > b.score; });
    for (auto& s : scored_lines) {
        double lx = 0.5 * (s.line[0] + s.line[2]);
        bool dup = false;
        for (auto& k : shared) {
            double kx = 0.5 * (k[0] + k[2]);
            if (std::abs(lx - kx) < cluster_x * 0.5) { dup = true; break; }
        }
        if (!dup) shared.push_back(s.line);
    }
    return shared;
}

// 把跨 ROI 共享竖直边注入当前 ROI 的 main_edges（镜像 Python 4873-5027）
void inject_shared_verticals(std::vector<MergedLine>& merged,
                             const std::vector<cv::Vec4d>& shared,
                             const RoiRect& roi,
                             const InspectorParams& params, double px_per_mm) {
    const DefectDetectParams& dd = params.defect_detection;
    if (!dd.cross_roi_vertical_enabled || shared.empty()) return;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    int W = roi.width, H = roi.height;
    if (W <= 0 || H <= 0) return;

    // 1) 全局 → ROI 局部，裁剪，仅保留近竖直
    std::vector<cv::Vec4d> appended;
    for (auto& g : shared) {
        cv::Vec4d lseg(g[0] - roi.x, g[1] - roi.y, g[2] - roi.x, g[3] - roi.y);
        cv::Vec4d clipped;
        if (!clip_infinite_line_to_roi_local(lseg, W, H, clipped)) continue;
        double ang = fold_angle_deg_abs(std::atan2(clipped[3] - clipped[1], clipped[2] - clipped[0]) * 180.0 / CV_PI);
        if (ang < (90.0 - v_tol)) continue;
        appended.push_back(clipped);
    }
    if (appended.empty()) return;

    // 2) 按 x 中心排序 + 相邻(<=1px)合并 y 范围（镜像 Python 4967-4981）
    auto x_center = [](const cv::Vec4d& s) { return 0.5 * (s[0] + s[2]); };
    std::sort(appended.begin(), appended.end(),
              [&](const cv::Vec4d& a, const cv::Vec4d& b) { return x_center(a) < x_center(b); });
    std::vector<cv::Vec4d> appended_uniq;
    for (auto& cseg : appended) {
        if (appended_uniq.empty()) { appended_uniq.push_back(cseg); continue; }
        if (std::abs(x_center(cseg) - x_center(appended_uniq.back())) <= 1.0) {
            double y0a = std::min(appended_uniq.back()[1], appended_uniq.back()[3]);
            double y1a = std::max(appended_uniq.back()[1], appended_uniq.back()[3]);
            double y0b = std::min(cseg[1], cseg[3]);
            double y1b = std::max(cseg[1], cseg[3]);
            double xc = x_center(appended_uniq.back());
            appended_uniq.back() = cv::Vec4d(xc, std::min(y0a, y0b), xc, std::max(y1a, y1b));
        } else {
            appended_uniq.push_back(cseg);
        }
    }

    // 3) 与现有主边垂直距离 > 3px 才加入（镜像 Python 4983-4991）
    auto perp_dist_to_merged = [&](const cv::Vec4d& s) -> double {
        double mx = 0.5 * (s[0] + s[2]), my = 0.5 * (s[1] + s[3]);
        double best = 1e18;
        for (auto& e : merged) {
            double el = e.length_px;
            if (el < 1e-6) continue;
            double cross = (e.line[2] - e.line[0]) * (e.line[1] - my) - (e.line[3] - e.line[1]) * (e.line[0] - mx);
            best = std::min(best, std::abs(cross) / el);
        }
        return best;
    };
    std::vector<cv::Vec4d> appended_to_add;
    for (auto& cseg : appended_uniq) {
        if (perp_dist_to_merged(cseg) > 3.0) appended_to_add.push_back(cseg);
    }
    if (appended_to_add.empty()) return;

    // 4) prefer_global：删除 x 接近且 y 重叠足够的现有近竖直主边（镜像 Python 4993-5021）
    if (dd.cross_roi_vertical_prefer_global) {
        double replace_max_dist = dd.cross_roi_vertical_replace_max_dist_mm > 0
            ? dd.cross_roi_vertical_replace_max_dist_mm * px_per_mm : 12.0;
        double min_overlap = dd.cross_roi_vertical_replace_min_y_overlap_ratio > 0
            ? dd.cross_roi_vertical_replace_min_y_overlap_ratio : 0.15;
        auto y_span = [](const cv::Vec4d& s) -> std::pair<double, double> {
            return {std::min(s[1], s[3]), std::max(s[1], s[3])};
        };
        auto y_overlap_ratio = [](double a0, double a1, double b0, double b1) {
            double overlap = std::max(0.0, std::min(a1, b1) - std::max(a0, b0));
            double span = std::max(1.0, std::max(a1, b1) - std::min(a0, b0));
            return overlap / span;
        };
        std::vector<MergedLine> filtered;
        for (auto& e : merged) {
            if (!e.near_vertical) { filtered.push_back(e); continue; }
            double xe = 0.5 * (e.line[0] + e.line[2]);
            auto [ye0, ye1] = y_span(e.line);
            bool drop = false;
            for (auto& cseg : appended_to_add) {
                double xc = x_center(cseg);
                if (std::abs(xe - xc) > replace_max_dist) continue;
                auto [yc0, yc1] = y_span(cseg);
                if (y_overlap_ratio(ye0, ye1, yc0, yc1) >= min_overlap) { drop = true; break; }
            }
            if (!drop) filtered.push_back(e);
        }
        merged = std::move(filtered);
    }

    // 5) 追加共享竖直边
    for (auto& cseg : appended_to_add) {
        MergedLine ml;
        ml.line = cseg;
        ml.length_px = std::hypot(cseg[2] - cseg[0], cseg[3] - cseg[1]);
        ml.angle_deg = fold_angle_deg_abs(std::atan2(cseg[3] - cseg[1], cseg[2] - cseg[0]) * 180.0 / CV_PI);
        ml.support = (int)std::lround(ml.length_px);
        ml.near_vertical = true;
        ml.near_horizontal = false;
        merged.push_back(ml);
    }
}

} // namespace

std::vector<Defect> Detector::process_roi(
    const cv::Mat& image_gray,
    const RoiRect& roi,
    const InspectorParams& params,
    int roi_idx,
    const std::vector<MergedLine>& merged_in,
    const std::vector<cv::Vec4d>& shared_verticals)
{
    // 裁剪 ROI（越界保护）
    int x0 = std::max(0, roi.x);
    int y0 = std::max(0, roi.y);
    int x1 = std::min(image_gray.cols, roi.x + roi.width);
    int y1 = std::min(image_gray.rows, roi.y + roi.height);
    if (x1 <= x0 || y1 <= y0) return {};
    cv::Mat roi_gray = image_gray(cv::Rect(x0, y0, x1 - x0, y1 - y0));

    // 预处理 + Hough + 直线合并（镜像 Python process_roi_hough_based 的 Hough 段）
    std::vector<MergedLine> merged = merged_in.empty()
        ? hough_merge_roi(roi_gray, params, px_per_mm_)
        : merged_in;

    // 跨 ROI 共享竖直边注入（镜像 Python 4873-5027）：把整图收集的竖直边并入本 ROI 主边，
    // 使 B 亮度扫描能覆盖跨 ROI 的竖直玻璃边缘
    inject_shared_verticals(merged, shared_verticals, roi, params, px_per_mm_);

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

#ifdef CPP_DEBUG_B
            std::cerr << "[B] roi=" << roi_idx << " scan_contours=" << all_chipping_contours.size()
                      << " final_contours=" << final_contours.size() << std::endl;
            for (auto& cnt : final_contours) {
                cv::RotatedRect rrt = cv::minAreaRect(cnt);
                double area_t = std::abs(cv::contourArea(cnt));
                std::cerr << "  [B] cnt center=(" << rrt.center.x << "," << rrt.center.y
                          << ") area=" << area_t << " w=" << rrt.size.width << " h=" << rrt.size.height
                          << " min_area_px2=" << min_area_px2 << std::endl;
            }
#endif

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

    // 逐 ROI 检测（两遍：先合并主边 → 收集跨 ROI 共享竖直边 → 再逐 ROI 检测注入）
    std::vector<Defect> all_defects;
    if (rois.empty()) {
        // 无 ROI 时直接返回
    } else {
        std::vector<std::vector<MergedLine>> per_roi_merged;
        per_roi_merged.reserve(rois.size());
        for (size_t i = 0; i < rois.size(); i++) {
            int x0 = std::max(0, rois[i].x);
            int y0 = std::max(0, rois[i].y);
            int x1 = std::min(image_gray.cols, rois[i].x + rois[i].width);
            int y1 = std::min(image_gray.rows, rois[i].y + rois[i].height);
            if (x1 <= x0 || y1 <= y0) { per_roi_merged.emplace_back(); continue; }
            cv::Mat roi_gray = image_gray(cv::Rect(x0, y0, x1 - x0, y1 - y0));
            per_roi_merged.push_back(hough_merge_roi(roi_gray, *params, px_per_mm_));
        }
        std::vector<cv::Vec4d> shared = collect_shared_vertical_edges(rois, per_roi_merged, *params, px_per_mm_);
        for (size_t i = 0; i < rois.size(); i++) {
            auto defects = process_roi(image_gray, rois[i], *params, (int)i, per_roi_merged[i], shared);
            all_defects.insert(all_defects.end(), defects.begin(), defects.end());
            RoiResult rr;
            rr.roi = rois[i];
            rr.defects = defects;
            result.roi_results.push_back(std::move(rr));
        }
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
            double alpha = (config_.mode == "dark" ? config_.dark : config_.light)
                               .visualization.defect_overlay_alpha;
            if (alpha <= 0) alpha = 0.25;
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
