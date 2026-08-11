#include "engine/skew.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace {

inline double angle_abs_deg(double a) {
    a = std::fmod(std::abs(a), 180.0);
    return a <= 90.0 ? a : 180.0 - a;
}

// 把直线裁剪到 ROI 边界（与 Python _clip_infinite_line_to_roi_local_for_e 一致）
cv::Vec4d clip_line_to_roi(const cv::Vec4d& seg, int w, int h) {
    double x1 = seg[0], y1 = seg[1], x2 = seg[2], y2 = seg[3];
    double dx = x2 - x1, dy = y2 - y1;
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) return seg;
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
    // 去重
    std::vector<cv::Point2d> uniq;
    for (auto& pt : cand) {
        bool dup = false;
        for (auto& q : uniq)
            if (std::abs(pt.x - q.x) < 0.5 && std::abs(pt.y - q.y) < 0.5) { dup = true; break; }
        if (!dup) uniq.push_back(pt);
    }
    if (uniq.size() < 2) return seg;
    // 取最远两点
    cv::Point2d a = uniq[0], b = uniq[1];
    double maxd = -1.0;
    for (size_t i = 0; i < uniq.size(); i++)
        for (size_t j = i + 1; j < uniq.size(); j++) {
            double d = std::hypot(uniq[j].x - uniq[i].x, uniq[j].y - uniq[i].y);
            if (d > maxd) { maxd = d; a = uniq[i]; b = uniq[j]; }
        }
    return cv::Vec4d(a.x, a.y, b.x, b.y);
}

// 点到线段距离
double pt_to_seg_dist(double px, double py, const cv::Vec4d& seg) {
    double x1 = seg[0], y1 = seg[1], x2 = seg[2], y2 = seg[3];
    double vx = x2 - x1, vy = y2 - y1;
    double denom = vx * vx + vy * vy;
    if (denom < 1e-9) return std::hypot(px - x1, py - y1);
    double t = ((px - x1) * vx + (py - y1) * vy) / denom;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double qx = x1 + t * vx, qy = y1 + t * vy;
    return std::hypot(px - qx, py - qy);
}

double min_dist_to_edges(double px, double py, const std::vector<MergedLine>& edges) {
    double best = std::numeric_limits<double>::infinity();
    for (auto& ml : edges) best = std::min(best, pt_to_seg_dist(px, py, ml.line));
    return best;
}

} // namespace

std::vector<Defect> detect_e_defects(
    const cv::Mat& roi_gray,
    const cv::Mat& defect_edges,
    const std::vector<MergedLine>& true_edges,
    const InspectorParams& params,
    double px_per_mm)
{
    std::vector<Defect> e_defects;
    if (defect_edges.empty()) return e_defects;

    const DefectDetectParams& dd = params.defect_detection;
    const int h_img = defect_edges.rows, w_img = defect_edges.cols;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    double h_tol = v_tol;

    // ---- 1. 边缘准备：膨胀 -> 屏蔽主边带 -> Closing -> 再屏蔽 ----
    cv::Mat edges_base;
    cv::compare(defect_edges, 0, edges_base, cv::CMP_GT);
    edges_base.convertTo(edges_base, CV_8U, 1.0, 0.0);

    int dil_iter = std::clamp(dd.e_canny_dilate_iter, 0, 10);
    cv::Mat edges_work;
    if (dil_iter > 0) {
        cv::Mat kernel_e = cv::Mat::ones(3, 3, CV_8U);
        cv::dilate(edges_base, edges_work, kernel_e, cv::Point(-1, -1), dil_iter);
    } else {
        edges_work = edges_base.clone();
    }

    int suppress_w = std::max(0, dd.e_line_suppress_width_px);
    int extra_per_iter = std::max(0, dd.e_line_suppress_extra_px);
    int suppress_w_eff = std::max(0, suppress_w + dil_iter * extra_per_iter);

    cv::Mat mask_lines;
    if (suppress_w_eff > 0 && !true_edges.empty()) {
        mask_lines = cv::Mat::zeros(edges_base.size(), CV_8U);
        for (auto& ml : true_edges) {
            double ang = ml.angle_deg;
            if (ang >= (90.0 - v_tol) || ang <= h_tol) {
                cv::Vec4d clipped = clip_line_to_roi(ml.line, w_img, h_img);
                cv::line(mask_lines,
                         cv::Point((int)std::lround(clipped[0]), (int)std::lround(clipped[1])),
                         cv::Point((int)std::lround(clipped[2]), (int)std::lround(clipped[3])),
                         cv::Scalar(255), suppress_w_eff);
            }
        }
        if (cv::countNonZero(mask_lines) > 0) {
            edges_work.setTo(0, mask_lines);
        }
    }

    // Closing 连接断裂斜向边缘（屏蔽带之后）
    int close_iter = std::clamp(dd.e_canny_close_iter, 0, 10);
    int close_ks = std::clamp(dd.e_canny_close_kernel_size, 1, 9);
    if (close_ks % 2 == 0) close_ks += 1;
    if (close_iter > 0) {
        cv::Mat kernel_close = cv::Mat::ones(close_ks, close_ks, CV_8U);
        cv::morphologyEx(edges_work, edges_work, cv::MORPH_CLOSE, kernel_close,
                         cv::Point(-1, -1), close_iter);
        if (!mask_lines.empty() && cv::countNonZero(mask_lines) > 0) {
            edges_work.setTo(0, mask_lines);
        }
    }

    // ---- 2. 快速预过滤 ----
    bool skip_e_contours = false;
    if (dd.e_fast_prefilter_enable) {
        double relax = std::clamp(dd.e_fast_prefilter_relax, 0.1, 1.0);
        int e_min_pixels = std::max(1, (int)std::lround(dd.e_fast_min_edge_pixels * relax));
        double e_min_ratio = dd.e_fast_min_edge_ratio * relax;
        double e_roi_std_thr = dd.e_fast_roi_std_threshold * relax;

        int e_pixels = cv::countNonZero(edges_work);
        double e_ratio = (double)e_pixels / (double)edges_work.total();

        cv::Scalar mean, stddev;
        cv::meanStdDev(roi_gray, mean, stddev);
        double roi_std = stddev[0];

        if (e_pixels < e_min_pixels && e_ratio < e_min_ratio && roi_std < e_roi_std_thr) {
            skip_e_contours = true;
        }
    }

    double min_area_px2 = dd.e_min_area_mm2 * px_per_mm * px_per_mm;
    double min_side_px = std::max(dd.e_min_side_mm * px_per_mm, 20.0 * px_per_mm);
    double border_touch_px = dd.e_border_touch_mm * px_per_mm;
    if (border_touch_px < 1.0) border_touch_px = 8.0;

    // ---- 3. 主边中的斜向线直接作为 E 候选 ----
    if (dd.e_include_skew_main_edges && !true_edges.empty()) {
        double min_len_px_for_main = dd.e_from_main_edge_min_len_mm * px_per_mm;
        for (auto& ml : true_edges) {
            double ang = ml.angle_deg;
            if (ang <= h_tol || ang >= (90.0 - v_tol)) continue;
            if (ml.length_px < min_len_px_for_main) continue;
            double xmn = std::min(ml.line[0], ml.line[2]);
            double xmx = std::max(ml.line[0], ml.line[2]);
            double ymn = std::min(ml.line[1], ml.line[3]);
            double ymx = std::max(ml.line[1], ml.line[3]);
            if (std::min({xmn, ymn, (double)(w_img - 1) - xmx, (double)(h_img - 1) - ymx}) > border_touch_px)
                continue;
            int pad = std::max(2, (int)std::lround(0.5 * std::max(1, suppress_w_eff)));
            int X = std::max(0, (int)std::floor(xmn) - pad);
            int Y = std::max(0, (int)std::floor(ymn) - pad);
            int X2 = std::min(w_img - 1, (int)std::ceil(xmx) + pad);
            int Y2 = std::min(h_img - 1, (int)std::ceil(ymx) + pad);
            Defect d;
            d.type = "E";
            d.box_points = {cv::Point(X, Y), cv::Point(X2, Y), cv::Point(X2, Y2), cv::Point(X, Y2)};
            d.angle_deg = 90.0 - ang; // skew_angle_deg
            d.x = (X + X2) / 2; d.y = (Y + Y2) / 2;
            d.length_mm = std::max(X2 - X, Y2 - Y) / px_per_mm;
            d.width_mm = std::min(X2 - X, Y2 - Y) / px_per_mm;
            d.size_mm = d.length_mm;
            d.confidence = 0.75;
            e_defects.push_back(d);
        }
    }

    // ---- 4. 轮廓分析 ----
    std::vector<Defect> contour_defects;
    if (!skip_e_contours) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges_work, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto& cnt : contours) {
            if (cnt.size() < 3) continue;
            double area = std::abs(cv::contourArea(cnt));
            cv::RotatedRect rect = cv::minAreaRect(cnt);
            double long_side = std::max(rect.size.width, rect.size.height);
            double short_side = std::min(rect.size.width, rect.size.height);
            if (long_side < std::max(1.0, min_side_px)) continue;
            if (area < std::max(1.0, min_area_px2)) {
                double perim = cv::arcLength(cnt, true);
                if (perim < std::max(8.0, 0.6 * long_side)) continue;
            }
            // 角度：minAreaRect 角度归一化到斜向
            double ang = std::abs(rect.angle);
            if (rect.size.width < rect.size.height) ang = std::abs(rect.angle + 90.0);
            if (ang > 90.0) ang -= 90.0;
            if (ang <= h_tol || ang >= (90.0 - v_tol)) continue;

            cv::Point2f box_pts[4];
            rect.points(box_pts);
            cv::Rect bb = cv::boundingRect(std::vector<cv::Point2f>(box_pts, box_pts + 4));

            // 位置过滤：靠近 ROI 边界 OR 靠近主边
            double border_dist = std::min({(double)bb.x, (double)bb.y,
                                           (double)w_img - (bb.x + bb.width),
                                           (double)h_img - (bb.y + bb.height)});
            if (border_dist > std::max(1.0, border_touch_px)) {
                double dmin_main = true_edges.empty()
                    ? std::numeric_limits<double>::infinity()
                    : min_dist_to_edges(rect.center.x, rect.center.y, true_edges);
                double near_main_thr = std::max(2.0, 0.5 * std::max(0, suppress_w_eff)
                                                  + std::max(1.0, border_touch_px) + 2.0);
                if (!(dmin_main <= near_main_thr)) continue;
            }

            // 夹紧到 ROI 内
            int x1b = std::max(0, bb.x);
            int y1b = std::max(0, bb.y);
            int x2b = std::min(w_img - 1, bb.x + bb.width - 1);
            int y2b = std::min(h_img - 1, bb.y + bb.height - 1);
            if (x2b <= x1b || y2b <= y1b) continue;

            Defect d;
            d.type = "E";
            d.box_points = {cv::Point(x1b, y1b), cv::Point(x2b, y1b),
                            cv::Point(x2b, y2b), cv::Point(x1b, y2b)};
            d.angle_deg = 90.0 - ang;
            d.x = (x1b + x2b) / 2; d.y = (y1b + y2b) / 2;
            d.length_mm = std::max(x2b - x1b + 1, y2b - y1b + 1) / px_per_mm;
            d.width_mm = std::min(x2b - x1b + 1, y2b - y1b + 1) / px_per_mm;
            d.size_mm = d.length_mm;
            d.confidence = 0.7;
            contour_defects.push_back(d);
        }
    }
    e_defects.insert(e_defects.end(), contour_defects.begin(), contour_defects.end());

    // ---- 5. 重叠框合并（并查集 + boundingRect + 面积加权角度）----
    struct RectInfo { int x, y, w, h; double angle; double area; };
    std::vector<RectInfo> rects;
    for (auto& d : e_defects) {
        int minx = INT_MAX, miny = INT_MAX, maxx = INT_MIN, maxy = INT_MIN;
        for (auto& pt : d.box_points) {
            minx = std::min(minx, pt.x); miny = std::min(miny, pt.y);
            maxx = std::max(maxx, pt.x); maxy = std::max(maxy, pt.y);
        }
        int w = std::max(1, maxx - minx), h = std::max(1, maxy - miny);
        rects.push_back({minx, miny, w, h, d.angle_deg, (double)w * h});
    }
    if (rects.size() > 1) {
        int n = (int)rects.size();
        std::vector<int> parent(n);
        for (int i = 0; i < n; i++) parent[i] = i;
        auto find = [&](auto&& self, int a) -> int {
            while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
            return a;
        };
        auto union_ = [&](int a, int b) {
            int ra = find(find, a), rb = find(find, b);
            if (ra != rb) parent[rb] = ra;
        };
        auto overlap = [](const RectInfo& r1, const RectInfo& r2) {
            int ix1 = std::max(r1.x, r2.x), iy1 = std::max(r1.y, r2.y);
            int ix2 = std::min(r1.x + r1.w, r2.x + r2.w);
            int iy2 = std::min(r1.y + r1.h, r2.y + r2.h);
            return (ix2 - ix1) > 0 && (iy2 - iy1) > 0;
        };
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (overlap(rects[i], rects[j])) union_(i, j);
        std::map<int, std::vector<int>> groups;
        for (int i = 0; i < n; i++) groups[find(find, i)].push_back(i);

        std::vector<Defect> merged;
        for (auto& [root, idxs] : groups) {
            if (idxs.size() == 1) { merged.push_back(e_defects[idxs[0]]); continue; }
            int minx = INT_MAX, miny = INT_MAX, maxx = INT_MIN, maxy = INT_MIN;
            double ang_sum = 0, area_sum = 0;
            for (int k : idxs) {
                minx = std::min(minx, rects[k].x); miny = std::min(miny, rects[k].y);
                maxx = std::max(maxx, rects[k].x + rects[k].w);
                maxy = std::max(maxy, rects[k].y + rects[k].h);
                ang_sum += rects[k].angle * rects[k].area;
                area_sum += rects[k].area;
            }
            Defect d = e_defects[idxs[0]];
            d.box_points = {cv::Point(minx, miny), cv::Point(maxx, miny),
                            cv::Point(maxx, maxy), cv::Point(minx, maxy)};
            d.angle_deg = area_sum > 1e-6 ? ang_sum / area_sum : 0.0;
            d.x = (minx + maxx) / 2; d.y = (miny + maxy) / 2;
            d.length_mm = std::max(maxx - minx, maxy - miny) / px_per_mm;
            d.width_mm = std::min(maxx - minx, maxy - miny) / px_per_mm;
            d.size_mm = d.length_mm;
            merged.push_back(d);
        }
        return merged;
    }
    return e_defects;
}
