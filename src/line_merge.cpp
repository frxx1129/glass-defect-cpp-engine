#include "engine/line_merge.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

double line_length_px(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    return std::sqrt(dx * dx + dy * dy);
}
double line_angle_deg(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    double a = std::atan2(std::abs(dy), dx) * 180.0 / CV_PI;
    return a;
}
double line_angle_deg(const cv::Vec4d& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    double a = std::atan2(std::abs(dy), dx) * 180.0 / CV_PI;
    return a;
}

namespace {

inline double dist_px_mm(const LineMergingParams& p, double px_per_mm,
                         const char* key_mm, const char* key_px,
                         double default_mm, double default_px) {
    // Python _get_dist_px: 有 MM 用 MM*ppm，否则用 PX
    // 这里 p 结构已解析，直接读字段（调用方保证传入正确字段）
    (void)p; (void)key_mm; (void)key_px;
    if (default_mm > 0) return default_mm * px_per_mm;
    return default_px;
}

inline bool is_near_vertical(double angle_deg, double tol) {
    double a = std::fmod(angle_deg, 180.0);
    if (a < 0) a += 180.0;
    return std::abs(90.0 - a) <= tol;
}
inline bool is_near_horizontal(double angle_deg, double tol) {
    double a = std::fmod(angle_deg, 180.0);
    if (a < 0) a += 180.0;
    return std::min(a, 180.0 - a) <= tol;
}

// 点到线段所在直线的垂直距离（与 Python 一致）
inline double point_line_dist(double px, double py, const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    double len2 = dx * dx + dy * dy;
    if (len2 < 1e-9) return std::hypot(px - l[0], py - l[1]);
    double cross = dx * (l[1] - py) - dy * (l[0] - px);
    return std::abs(cross) / std::sqrt(len2);
}

// 早期双主体估计：线段中点 x 最大间隙 > 阈值时划分左右两组
struct EarlyCluster {
    bool active = false;
    double x_thresh = 0.0;
};

EarlyCluster build_early_cluster(const std::vector<cv::Vec4i>& lines, const LineMergingParams& lm,
                                 double px_per_mm) {
    EarlyCluster ec;
    double cluster_gap_px = 40.0 * px_per_mm; // GLASS_CLUSTER_GAP_MM 默认 40
    if (cluster_gap_px <= 0 || lines.size() < 4) return ec;
    std::vector<double> mids;
    for (auto& l : lines) mids.push_back((l[0] + l[2]) * 0.5);
    std::sort(mids.begin(), mids.end());
    double max_gap = 0.0; int max_k = -1;
    for (size_t i = 0; i + 1 < mids.size(); i++) {
        double g = mids[i + 1] - mids[i];
        if (g > max_gap) { max_gap = g; max_k = (int)i; }
    }
    if (max_k >= 0 && max_gap >= cluster_gap_px) {
        ec.active = true;
        ec.x_thresh = 0.5 * (mids[max_k] + mids[max_k + 1]);
    }
    return ec;
}

inline int early_cluster_id(const EarlyCluster& ec, double x_mid) {
    if (!ec.active) return 0;
    return x_mid <= ec.x_thresh ? 0 : 1;
}

// 沿法线方向做 Canny 缝隙检查（镜像 Python GAP_NO_EDGE_PX=30）
bool canny_gap_blocks_merge(const cv::Mat& edge_img, const cv::Vec4i& ref,
                            double mid_x, double mid_y,
                            double stripe_half) {
    if (edge_img.empty()) return false;
    double dx = ref[2] - ref[0], dy = ref[3] - ref[1];
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return false;
    double ux = dx / len, uy = dy / len;
    // 垂足
    double t = ((mid_x - ref[0]) * ux + (mid_y - ref[1]) * uy);
    double perp_x = ref[0] + t * ux;
    double perp_y = ref[1] + t * uy;
    double d_vec_x = mid_x - perp_x, d_vec_y = mid_y - perp_y;
    double gap_len = std::hypot(d_vec_x, d_vec_y);
    if (gap_len <= 1.0) return false;

    int steps = (int)std::ceil(gap_len);
    double svx = d_vec_x / steps, svy = d_vec_y / steps;
    const int GAP_NO_EDGE_PX = 30;
    int max_run = 0, run = 0;
    int h = edge_img.rows, w = edge_img.cols;
    int sh = std::max(1, (int)stripe_half);
    for (int i = 0; i <= steps; i++) {
        int cx = (int)std::lround(perp_x + svx * i);
        int cy = (int)std::lround(perp_y + svy * i);
        bool hit = false;
        int x0 = std::max(0, cx - sh), x1 = std::min(w - 1, cx + sh);
        int y0 = std::max(0, cy - sh), y1 = std::min(h - 1, cy + sh);
        if (x0 <= x1 && y0 <= y1) {
            for (int yy = y0; yy <= y1 && !hit; yy++)
                for (int xx = x0; xx <= x1 && !hit; xx++)
                    if (edge_img.at<uchar>(yy, xx) > 0) hit = true;
        }
        if (hit) { run = 0; }
        else {
            run++;
            if (run > max_run) {
                max_run = run;
                if (max_run >= GAP_NO_EDGE_PX) return true; // 有足够大的无边缘缝隙 → 阻止合并
            }
        }
    }
    return false;
}

} // namespace

std::vector<MergedLine> merge_lines_and_get_main_edges(
    const std::vector<cv::Vec4i>& lines,
    const InspectorParams& params,
    double px_per_mm,
    const cv::Mat& edge_img)
{
    std::vector<MergedLine> result;
    if (lines.empty()) return result;

    const LineMergingParams& p = params.line_merging;
    const DefectDetectParams& dd = params.defect_detection;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    double h_tol = v_tol;

    double vertical_thick_merge_px = p.vertical_thick_merge_mm * px_per_mm;
    double vertical_thick_merge_cap_px = p.vertical_thick_merge_max_mm * px_per_mm;
    double vertical_max_ax_gap_px = 20.0 * px_per_mm; // VERTICAL_MAX_AXIAL_GAP_MM 默认 20
    double max_lat_dist_px = std::min(p.max_lateral_distance_mm * px_per_mm,
                                      (double)p.max_lateral_distance);
    double min_vertical_gap_px = p.min_vertical_edge_gap_px;
    double stripe_half = std::max(1.0, (double)dd.q_canny_stripe_half_width_px);

    EarlyCluster ec = build_early_cluster(lines, p, px_per_mm);

    // ===== 角度聚类 =====
    std::vector<std::pair<double, std::vector<cv::Vec4i>>> angle_clusters;
    for (auto& l : lines) {
        double ang = line_angle_deg(l);
        bool placed = false;
        for (auto& [ca, segs] : angle_clusters) {
            double d = std::abs(ang - ca);
            if (std::min(d, 180.0 - d) < p.angle_tolerance) {
                segs.push_back(l); placed = true; break;
            }
        }
        if (!placed) angle_clusters.push_back({ang, {l}});
    }

    // ===== 每角度簇：邻近聚类 =====
    std::vector<std::vector<std::vector<cv::Vec4i>>> final_groups; // [angle_cluster][proximity_group]
    for (auto& [angle, segments] : angle_clusters) {
        if (segments.empty()) continue;
        std::sort(segments.begin(), segments.end(),
                  [](const cv::Vec4i& a, const cv::Vec4i& b) { return line_length_px(a) > line_length_px(b); });

        std::vector<std::vector<cv::Vec4i>> proximity_groups;
        proximity_groups.push_back({segments[0]});
        for (size_t si = 1; si < segments.size(); si++) {
            auto& seg = segments[si];
            double mid_x = (seg[0] + seg[2]) / 2.0, mid_y = (seg[1] + seg[3]) / 2.0;
            bool placed = false;
            for (auto& group : proximity_groups) {
                const cv::Vec4i& ref = group[0];
                double dist = point_line_dist(mid_x, mid_y, ref);
                if (dist >= max_lat_dist_px) continue;
                bool allowed_merge = true;
                // 轴向间隙约束：竖直边允许更大轴向 gap
                double ref_ang = line_angle_deg(ref);
                if (is_near_vertical(ref_ang, v_tol)) {
                    double ux = (ref[2] - ref[0]) / std::max(1e-6, line_length_px(ref));
                    double uy = (ref[3] - ref[1]) / std::max(1e-6, line_length_px(ref));
                    double g_min = 1e18, g_max = -1e18;
                    for (auto& ln : group) {
                        for (int k = 0; k < 2; k++) {
                            double px_ = k == 0 ? ln[0] : ln[2];
                            double py_ = k == 0 ? ln[1] : ln[3];
                            double t = (px_ - ref[0]) * ux + (py_ - ref[1]) * uy;
                            g_min = std::min(g_min, t); g_max = std::max(g_max, t);
                        }
                    }
                    double t_proj = (mid_x - ref[0]) * ux + (mid_y - ref[1]) * uy;
                    if (t_proj < g_min - vertical_max_ax_gap_px || t_proj > g_max + vertical_max_ax_gap_px) {
                        allowed_merge = false;
                    }
                }
                // Canny 缝隙检查
                if (allowed_merge && !edge_img.empty()) {
                    if (canny_gap_blocks_merge(edge_img, ref, mid_x, mid_y, stripe_half)) {
                        allowed_merge = false;
                    }
                }
                if (allowed_merge) { group.push_back(seg); placed = true; break; }
            }
            if (!placed) proximity_groups.push_back({seg});
        }

        // ===== 竖直粗边二次合并（MAD 厚度 + 动态横向阈值 + 重叠比例）=====
        bool is_vert = is_near_vertical(angle, v_tol);
        if (is_vert && proximity_groups.size() > 1 && vertical_thick_merge_px > 0) {
            auto group_stats = [](const std::vector<cv::Vec4i>& g) {
                std::vector<double> xs, ys;
                for (auto& ln : g) {
                    xs.push_back(ln[0]); xs.push_back(ln[2]);
                    ys.push_back(ln[1]); ys.push_back(ln[3]);
                }
                std::sort(xs.begin(), xs.end());
                std::vector<double> devs;
                double x_med = xs[xs.size() / 2];
                for (double x : xs) devs.push_back(std::abs(x - x_med));
                std::sort(devs.begin(), devs.end());
                double x_mad = devs[devs.size() / 2] * 2.0;
                double y_min = *std::min_element(ys.begin(), ys.end());
                double y_max = *std::max_element(ys.begin(), ys.end());
                return std::make_tuple(x_med, x_mad, y_min, y_max);
            };

            std::vector<bool> used(proximity_groups.size(), false);
            std::vector<std::vector<cv::Vec4i>> merged_groups;
            for (size_t i = 0; i < proximity_groups.size(); i++) {
                if (used[i]) continue;
                auto [xi, xi_mad, yi0, yi1] = group_stats(proximity_groups[i]);
                std::vector<cv::Vec4i> cur = proximity_groups[i];
                used[i] = true;
                for (size_t j = i + 1; j < proximity_groups.size(); j++) {
                    if (used[j]) continue;
                    auto [xj, xj_mad, yj0, yj1] = group_stats(proximity_groups[j]);
                    if (ec.active && early_cluster_id(ec, xi) != early_cluster_id(ec, xj)) continue;
                    double dyn_allow = std::max(vertical_thick_merge_px, 1.5 * (xi_mad + xj_mad));
                    if (vertical_thick_merge_cap_px > 0) dyn_allow = std::min(dyn_allow, vertical_thick_merge_cap_px);
                    if (std::abs(xi - xj) <= dyn_allow) {
                        double overlap = std::max(0.0, std::min(yi1, yj1) - std::max(yi0, yj0));
                        double span = std::max(1.0, std::max(yi1, yj1) - std::min(yi0, yj0));
                        double min_overlap = p.vertical_thick_min_overlap_ratio > 0
                            ? p.vertical_thick_min_overlap_ratio : 0.2;
                        if ((overlap / span) >= min_overlap) {
                            cur.insert(cur.end(), proximity_groups[j].begin(), proximity_groups[j].end());
                            used[j] = true;
                            std::tie(xi, xi_mad, yi0, yi1) = group_stats(cur);
                        }
                    }
                }
                merged_groups.push_back(cur);
            }
            final_groups.push_back(merged_groups);
        } else {
            final_groups.push_back(proximity_groups);
        }
    }

    // ===== 每组：fitLine + 投影端点 + 轴向锁定 =====
    const double canny_refine_half_px = 3.0;
    for (auto& angle_groups : final_groups) {
        for (auto& group : angle_groups) {
            if (group.empty()) continue;
            std::vector<cv::Point2f> pts;
            for (auto& ln : group) {
                pts.emplace_back((float)ln[0], (float)ln[1]);
                pts.emplace_back((float)ln[2], (float)ln[3]);
            }
            cv::Vec4f fit;
            try {
                cv::fitLine(pts, fit, cv::DIST_L2, 0, 0.01, 0.01);
            } catch (...) {
                continue;
            }
            double vx = fit[0], vy = fit[1], x0 = fit[2], y0 = fit[3];
            double ang = std::abs(std::atan2(vy, vx) * 180.0 / CV_PI);
            if (ang > 90.0) ang = 180.0 - ang;

            // 投影端点
            double t_min = 1e18, t_max = -1e18;
            cv::Point2d p_min, p_max;
            for (auto& pt : pts) {
                double t = (pt.x - x0) * vx + (pt.y - y0) * vy;
                if (t < t_min) { t_min = t; p_min = {(double)pt.x, (double)pt.y}; }
                if (t > t_max) { t_max = t; p_max = {(double)pt.x, (double)pt.y}; }
            }
            cv::Vec4d final_line(p_min.x, p_min.y, p_max.x, p_max.y);

            bool nv = is_near_vertical(ang, v_tol);
            bool nh = is_near_horizontal(ang, h_tol);

            // 轴向锁定：利用 Canny 边缘图稳定位置
            if (!edge_img.empty()) {
                int h_img = edge_img.rows, w_img = edge_img.cols;
                if (nv) {
                    int y0s = std::max(0, (int)std::floor(p_min.y));
                    int y1s = std::min(h_img - 1, (int)std::ceil(p_max.y));
                    std::vector<double> xs;
                    for (auto& ln : group) { xs.push_back(ln[0]); xs.push_back(ln[2]); }
                    std::sort(xs.begin(), xs.end());
                    int x_med = (int)std::lround(xs[xs.size() / 2]);
                    int cx_best = x_med; int best_sum = -1;
                    for (int cx = std::max(0, x_med - (int)canny_refine_half_px);
                         cx <= std::min(w_img - 1, x_med + (int)canny_refine_half_px); cx++) {
                        int col_sum = 0;
                        for (int yy = y0s; yy <= y1s; yy++)
                            if (edge_img.at<uchar>(yy, cx) > 0) col_sum++;
                        if (col_sum > best_sum) { best_sum = col_sum; cx_best = cx; }
                    }
                    final_line = cv::Vec4d(cx_best, y0s, cx_best, y1s);
                } else if (nh) {
                    int x0s = std::max(0, (int)std::floor(p_min.x));
                    int x1s = std::min(w_img - 1, (int)std::ceil(p_max.x));
                    std::vector<double> ys;
                    for (auto& ln : group) { ys.push_back(ln[1]); ys.push_back(ln[3]); }
                    std::sort(ys.begin(), ys.end());
                    int y_med = (int)std::lround(ys[ys.size() / 2]);
                    int cy_best = y_med; int best_sum = -1;
                    for (int cy = std::max(0, y_med - (int)canny_refine_half_px);
                         cy <= std::min(h_img - 1, y_med + (int)canny_refine_half_px); cy++) {
                        int row_sum = 0;
                        for (int xx = x0s; xx <= x1s; xx++)
                            if (edge_img.at<uchar>(cy, xx) > 0) row_sum++;
                        if (row_sum > best_sum) { best_sum = row_sum; cy_best = cy; }
                    }
                    final_line = cv::Vec4d(x0s, cy_best, x1s, cy_best);
                }
            }

            MergedLine ml;
            ml.line = final_line;
            ml.angle_deg = line_angle_deg(final_line);
            ml.length_px = std::hypot(final_line[2] - final_line[0], final_line[3] - final_line[1]);
            ml.support = (int)pts.size();
            ml.near_vertical = is_near_vertical(ml.angle_deg, v_tol);
            ml.near_horizontal = is_near_horizontal(ml.angle_deg, h_tol);
            result.push_back(ml);
        }
    }

    // 按长度排序取 top_n
    std::sort(result.begin(), result.end(),
              [](const MergedLine& a, const MergedLine& b) { return a.length_px > b.length_px; });
    if ((int)result.size() > p.top_n_edges) result.resize(p.top_n_edges);
    return result;
}
