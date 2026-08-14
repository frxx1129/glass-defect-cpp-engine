#include "engine/line_merge.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

double line_length_px(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    return std::sqrt(dx * dx + dy * dy);
}
double line_angle_deg(const cv::Vec4i& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    // 镜像 Python：atan2(dy,dx) 负值 +180，归一化到 [0,180)。
    // 不能用 atan2(|dy|,dx)：会抹掉 dy 符号，导致向下倾斜与向上倾斜的线被折叠到同一角度，
    // 使角度簇合并错误（如 178.99° 与 4.19° 的真实夹角 5.2° 被误算成 3.18°）。
    double a = std::atan2(dy, dx) * 180.0 / CV_PI;
    if (a < 0) a += 180.0;
    return a;
}
double line_angle_deg(const cv::Vec4d& l) {
    double dx = l[2] - l[0], dy = l[3] - l[1];
    double a = std::atan2(dy, dx) * 180.0 / CV_PI;
    if (a < 0) a += 180.0;
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

// 两直线交点（无限延长），平行返回 false（镜像 Python find_line_intersection）
inline bool line_intersection_inf(const cv::Vec4d& a, const cv::Vec4d& b, cv::Point2d& out) {
    double ax = a[2] - a[0], ay = a[3] - a[1];
    double bx = b[2] - b[0], by = b[3] - b[1];
    double denom = ax * by - ay * bx;
    if (std::abs(denom) < 1e-9) return false;
    double t = ((b[0] - a[0]) * by - (b[1] - a[1]) * bx) / denom;
    out = cv::Point2d(a[0] + t * ax, a[1] + t * ay);
    return true;
}

// 点在直线上的 t 参数、垂距、线段长度（镜像 Python _t_param_and_perp_dist）
inline void t_param_and_perp_dist(const cv::Point2d& pt, const cv::Point2d& a, const cv::Point2d& b,
                                  double& t, double& d_perp, double& len) {
    cv::Point2d v = b - a;
    double denom = v.x * v.x + v.y * v.y;
    if (denom < 1e-8) { t = 0.0; d_perp = std::hypot(pt.x - a.x, pt.y - a.y); len = 0.0; return; }
    t = ((pt.x - a.x) * v.x + (pt.y - a.y) * v.y) / denom;
    cv::Point2d proj = a + t * v;
    d_perp = std::hypot(pt.x - proj.x, pt.y - proj.y);
    len = std::sqrt(denom);
}

// 两直线角度差（归一化到 [0, 90]）（镜像 Python diff_deg）
inline double angle_diff_deg(const cv::Vec4d& a, const cv::Vec4d& b) {
    auto ang = [](const cv::Vec4d& l) {
        double dx = l[2] - l[0], dy = l[3] - l[1];
        double aa = std::atan2(dy, dx) * 180.0 / CV_PI;
        if (aa < 0) aa += 180.0;
        return aa;
    };
    double da = std::abs(ang(a) - ang(b));
    return std::min(da, 180.0 - da);
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

#ifdef CPP_DEBUG_MERGED
        {
            std::cerr << "[CLUSTER] ang=" << angle << " n=" << segments.size() << std::endl;
            for (auto& s : segments) {
                std::cerr << "  seg (" << s[0] << "," << s[1] << ")->(" << s[2] << "," << s[3]
                          << ") len=" << line_length_px(s) << std::endl;
            }
        }
#endif

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

#ifdef CPP_DEBUG_MERGED
        {
            std::cerr << "  [PROX] ang=" << angle << " groups=" << proximity_groups.size() << std::endl;
            for (size_t gi = 0; gi < proximity_groups.size(); gi++) {
                std::cerr << "    group" << gi << ":";
                for (auto& s : proximity_groups[gi]) {
                    std::cerr << " (" << s[0] << "," << s[1] << ")->(" << s[2] << "," << s[3] << ")";
                }
                std::cerr << std::endl;
            }
        }
#endif

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

                    // 水平连接性校验（镜像 Python HORIZONTAL_CONNECTIVITY）：
                    // group 内线段按 x 投影排序，相邻段 gap>=8px 且 Canny 无边缘连续>=25px -> 拆回最长段
                    if (group.size() >= 2) {
                        struct XSeg { double a, b; cv::Vec4i ln; };
                        std::vector<XSeg> segs_proj;
                        for (auto& ln : group) {
                            double xa = std::min((double)ln[0], (double)ln[2]);
                            double xb = std::max((double)ln[0], (double)ln[2]);
                            segs_proj.push_back({xa, xb, ln});
                        }
                        std::sort(segs_proj.begin(), segs_proj.end(),
                                  [](const XSeg& u, const XSeg& v) { return u.a < v.a; });
                        const double gap_min_px = 8.0;
                        const double gap_no_edge_allow = 25.0;
                        const int stripe_half = 2;
                        bool connectivity_ok = true;
                        for (size_t k = 0; k + 1 < segs_proj.size() && connectivity_ok; k++) {
                            double a1 = segs_proj[k].b;
                            double b0 = segs_proj[k + 1].a;
                            double gap_len = b0 - a1;
                            if (gap_len < gap_min_px) continue;
                            int steps = std::max(1, (int)gap_len);
                            int no_edge_run = 0;
                            for (int s = 0; s <= steps; s++) {
                                double xpos = a1 + gap_len * (double)s / std::max(1, steps);
                                int cx = (int)std::lround(xpos);
                                int xa = std::max(0, cx - stripe_half);
                                int xb = std::min(w_img - 1, cx + stripe_half);
                                int ya = std::max(0, cy_best - stripe_half);
                                int yb = std::min(h_img - 1, cy_best + stripe_half);
                                bool hit = false;
                                if (xa <= xb && ya <= yb) {
                                    for (int yy = ya; yy <= yb && !hit; yy++)
                                        for (int xx = xa; xx <= xb && !hit; xx++)
                                            if (edge_img.at<uchar>(yy, xx) > 0) hit = true;
                                }
                                if (hit) no_edge_run = 0;
                                else {
                                    no_edge_run++;
                                    if (no_edge_run >= gap_no_edge_allow) { connectivity_ok = false; break; }
                                }
                            }
                        }
                        if (!connectivity_ok) {
                            // 回退：保留 group 中支持度最高的水平线段（最长）
                            const cv::Vec4i* best_ln = &group[0];
                            double best_len = -1;
                            for (auto& ln : group) {
                                double L = std::hypot(ln[2] - ln[0], ln[3] - ln[1]);
                                if (L > best_len) { best_len = L; best_ln = &ln; }
                            }
                            final_line = cv::Vec4d((*best_ln)[0], (*best_ln)[1], (*best_ln)[2], (*best_ln)[3]);
                        }
                    }
                }
            }

            MergedLine ml;
            ml.line = final_line;
            ml.angle_deg = line_angle_deg(final_line);
            ml.length_px = std::hypot(final_line[2] - final_line[0], final_line[3] - final_line[1]);
            ml.support = (int)pts.size();
            ml.near_vertical = is_near_vertical(ml.angle_deg, v_tol);
            ml.near_horizontal = is_near_horizontal(ml.angle_deg, h_tol);
#ifdef CPP_DEBUG_MERGED
            std::cerr << "  [FIT] (" << final_line[0] << "," << final_line[1] << ")->("
                      << final_line[2] << "," << final_line[3] << ") ang=" << ml.angle_deg
                      << " len=" << ml.length_px << " npts=" << pts.size() << std::endl;
#endif
            result.push_back(ml);
        }
    }

    // 按长度排序取 top_n
    std::vector<MergedLine> all_lines = result; // topN 前全部候选（镜像 Python filtered）
    std::sort(result.begin(), result.end(),
              [](const MergedLine& a, const MergedLine& b) { return a.length_px > b.length_px; });
    if ((int)result.size() > p.top_n_edges) result.resize(p.top_n_edges);

    // ===== 多玻璃/多主体：每个 cluster 至少保留一条近竖直主边（镜像 Python ENSURE_VERTICAL_PER_CLUSTER，1961-2055）=====
    // 当全部候选按 x 中点存在大间隙（>= GLASS_CLUSTER_GAP_MM）时划分左右两 cluster；
    // 若某 cluster 的 top_n 内没有竖直主边，则用该 cluster 候选集中最强的竖直边替换其最弱的非竖直边。
    // 典型场景：多玻璃 ROI 中右侧玻璃只有斜边噪声进入 top_n，缺少竖直主边 → E 误报。
    if (p.ensure_vertical_per_cluster && !all_lines.empty() && !result.empty()) {
        double cluster_gap_px = 40.0 * px_per_mm; // GLASS_CLUSTER_GAP_MM 默认 40
        if (cluster_gap_px > 0 && (int)all_lines.size() >= 4) {
            std::vector<double> mids;
            for (auto& ml : all_lines) mids.push_back((ml.line[0] + ml.line[2]) * 0.5);
            std::sort(mids.begin(), mids.end());
            double max_gap = 0.0; int k = -1;
            for (size_t i = 0; i + 1 < mids.size(); i++) {
                double g = mids[i + 1] - mids[i];
                if (g > max_gap) { max_gap = g; k = (int)i; }
            }
            if (k >= 0 && max_gap >= cluster_gap_px) {
                double x_thresh = 0.5 * (mids[k] + mids[k + 1]);
                auto cluster_id = [x_thresh](const MergedLine& ml) -> int {
                    double mx = (ml.line[0] + ml.line[2]) * 0.5;
                    return mx <= x_thresh ? 0 : 1;
                };
                int m = (int)result.size();
                std::vector<int> edges_cluster(m);
                bool has_v[2] = {false, false};
                for (int i = 0; i < m; i++) {
                    edges_cluster[i] = cluster_id(result[i]);
                    if (result[i].near_vertical) has_v[edges_cluster[i]] = true;
                }
                // 候选集（all_lines）中每个 cluster 的最强竖直候选（不在 top_n 内）
                MergedLine* best_v[2] = {nullptr, nullptr};
                for (auto& ml : all_lines) {
                    if (!ml.near_vertical) continue;
                    int cid = cluster_id(ml);
                    bool in_edges = false;
                    for (int i = 0; i < m && !in_edges; i++) {
                        if (result[i].line == ml.line) in_edges = true;
                    }
                    if (in_edges) continue;
                    if (!best_v[cid] || ml.length_px > best_v[cid]->length_px) best_v[cid] = &ml;
                }
                for (int cid = 0; cid < 2; cid++) {
                    if (has_v[cid] || !best_v[cid]) continue;
                    // 优先替换该 cluster 内最弱的非竖直边；否则该 cluster 内最弱边；再否则全局最弱边
                    std::vector<int> cand_idxs;
                    for (int i = 0; i < m; i++)
                        if (edges_cluster[i] == cid && !result[i].near_vertical) cand_idxs.push_back(i);
                    if (cand_idxs.empty())
                        for (int i = 0; i < m; i++)
                            if (edges_cluster[i] == cid) cand_idxs.push_back(i);
                    if (cand_idxs.empty())
                        for (int i = 0; i < m; i++) cand_idxs.push_back(i);
                    if (!cand_idxs.empty()) {
                        int idx_min = cand_idxs[0];
                        for (int i : cand_idxs)
                            if (result[i].length_px < result[idx_min].length_px) idx_min = i;
                        result[idx_min] = *best_v[cid];
                    }
                }
            }
        }
    }

    // ===== 交点裁剪/延长（镜像 Python merge 的端点交点对齐，2060-2231）=====
    // 把线段端点延长/裁剪到与其他线段的交点，使斜边端点对齐到水平/竖直边交点。
    {
        int m = (int)result.size();
        if (m > 1) {
            const double extend_margin = 5.0 * px_per_mm;    // 5mm
            const double ortho_extend = 30.0 * px_per_mm;    // INTERSECTION_ORTHO_EXTEND_MM
            const double ortho_accept_deg = 60.0;

            struct InterRec { cv::Point2d pt; double t; };
            std::vector<InterRec> best0(m), best1(m);
            std::vector<char> has0(m, 0), has1(m, 0);
            std::vector<double> score0(m, 1e18), score1(m, 1e18);

            for (int i = 0; i < m; i++) {
                const cv::Vec4d& li = result[i].line;
                cv::Point2d ai(li[0], li[1]), bi(li[2], li[3]);
                for (int j = i + 1; j < m; j++) {
                    const cv::Vec4d& lj = result[j].line;
                    cv::Point2d aj(lj[0], lj[1]), bj(lj[2], lj[3]);
                    cv::Point2d inter;
                    if (!line_intersection_inf(li, lj, inter)) continue;

                    double t_i, d_i, len_i, t_j, d_j, len_j;
                    t_param_and_perp_dist(inter, ai, bi, t_i, d_i, len_i);
                    t_param_and_perp_dist(inter, aj, bj, t_j, d_j, len_j);
                    if (len_i < 1e-6 || len_j < 1e-6) continue;

                    double ext_tol_i = extend_margin / len_i;
                    double ext_tol_j = extend_margin / len_j;
                    double diff_deg = angle_diff_deg(li, lj);
                    bool near_line = (d_i < 1.5) && (d_j < 1.5);
                    if (diff_deg >= ortho_accept_deg) {
                        near_line = true;
                        ext_tol_i = std::max(ext_tol_i, ortho_extend / len_i);
                        ext_tol_j = std::max(ext_tol_j, ortho_extend / len_j);
                    }
                    bool within_i = (-ext_tol_i <= t_i && t_i <= 1.0 + ext_tol_i);
                    bool within_j = (-ext_tol_j <= t_j && t_j <= 1.0 + ext_tol_j);
                    if (!(near_line && within_i && within_j)) continue;

                    auto upd = [&](InterRec& rec, char& has, double& sc, double t, double end,
                                   const cv::Point2d& pt) {
                        bool inside = (t >= 0.0 && t <= 1.0);
                        double cand = (inside ? 0.0 : 1.0) * 1e6 + std::abs(t - end);
                        if (!has || cand < sc) { rec = {pt, t}; has = 1; sc = cand; }
                    };
                    if (t_i <= 0.5) upd(best0[i], has0[i], score0[i], t_i, 0.0, inter);
                    else upd(best1[i], has1[i], score1[i], t_i, 1.0, inter);
                    if (t_j <= 0.5) upd(best0[j], has0[j], score0[j], t_j, 0.0, inter);
                    else upd(best1[j], has1[j], score1[j], t_j, 1.0, inter);
                }
            }

            for (int i = 0; i < m; i++) {
                if (!has0[i] && !has1[i]) continue;
                cv::Point2d p1(result[i].line[0], result[i].line[1]);
                cv::Point2d p2(result[i].line[2], result[i].line[3]);
                if (has0[i]) p1 = best0[i].pt;
                if (has1[i]) p2 = best1[i].pt;
                double L = std::hypot(p2.x - p1.x, p2.y - p1.y);
                if (L < 1.0) continue;
                result[i].line = cv::Vec4d(p1.x, p1.y, p2.x, p2.y);
                result[i].length_px = L;
                result[i].angle_deg = line_angle_deg(result[i].line);
            }
        }
    }

    return result;
}
