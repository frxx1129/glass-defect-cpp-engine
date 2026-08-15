#include "engine/corner.h"
#include "engine/preprocess.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

namespace {

inline double angle_x_deg(const cv::Vec4d& l) {
    double a = std::atan2(std::abs(l[3] - l[1]), std::abs(l[2] - l[0])) * 180.0 / CV_PI;
    return a <= 90.0 ? a : 180.0 - a;
}
inline bool is_vert(const cv::Vec4d& l, double tol) { return angle_x_deg(l) >= (90.0 - tol); }
inline bool is_horiz(const cv::Vec4d& l, double tol) { return angle_x_deg(l) <= tol; }

// 两直线交点（无限延长）；平行返回 false
bool line_intersection(const cv::Vec4d& a, const cv::Vec4d& b, cv::Point2d& out) {
    double ax = a[2] - a[0], ay = a[3] - a[1];
    double bx = b[2] - b[0], by = b[3] - b[1];
    double denom = ax * by - ay * bx;
    if (std::abs(denom) < 1e-9) return false;
    double t = ((b[0] - a[0]) * by - (b[1] - a[1]) * bx) / denom;
    out = cv::Point2d(a[0] + t * ax, a[1] + t * ay);
    return true;
}

// 模糊 HV 交点（Python _fuzzy_hv_intersection，容差 3px）
bool fuzzy_hv_intersection(const cv::Vec4d& hseg, const cv::Vec4d& vseg, int tol, cv::Point2d& out) {
    if (angle_x_deg(hseg) > 15.0) return false;
    double ang_v = angle_x_deg(vseg);
    if (ang_v < 75.0) return false;
    double hy = (hseg[1] + hseg[3]) / 2.0;
    double vx = (vseg[0] + vseg[2]) / 2.0;
    double h_xmin = std::min(hseg[0], hseg[2]) - tol;
    double h_xmax = std::max(hseg[0], hseg[2]) + tol;
    if (!(h_xmin <= vx && vx <= h_xmax)) return false;
    double v_ymin = std::min(vseg[1], vseg[3]) - tol;
    double v_ymax = std::max(vseg[1], vseg[3]) + tol;
    if (!(v_ymin <= hy && hy <= v_ymax)) return false;
    out = cv::Point2d(vx, hy);
    return true;
}

// 点到线段（无限延长）垂直距离
double perp_dist(const cv::Point2d& pt, const cv::Vec4d& seg) {
    double dx = seg[2] - seg[0], dy = seg[3] - seg[1];
    double len2 = dx * dx + dy * dy;
    if (len2 < 1e-9) return std::hypot(pt.x - seg[0], pt.y - seg[1]);
    return std::abs(dx * (seg[1] - pt.y) - dy * (seg[0] - pt.x)) / std::sqrt(len2);
}

// 平行四边形 + 边缘点集群验证（镜像 Python _q_parallelogram_cluster_ok，image_processor_hough.py:3832-3996）
// 从 Q 三角形构造平行四边形，剔除主边射线条带后检查内部缺陷边缘连通域：
// 存在 >= Q_PARALLELOGRAM_MIN_EDGE_PIXELS 像素且沿对角投影跨度 >= 25% 的连通域才保留该 Q。
bool q_parallelogram_cluster_ok(const Defect& q, const cv::Mat& edges_img, const DefectDetectParams& dd) {
    if (q.region_contour.size() != 3) return true;   // 无法验证则放行
    if (edges_img.empty()) return true;
    const int H = edges_img.rows, W = edges_img.cols;

    cv::Point2d A(q.region_contour[0].x, q.region_contour[0].y);
    cv::Point2d B(q.region_contour[1].x, q.region_contour[1].y);
    cv::Point2d C(q.region_contour[2].x, q.region_contour[2].y);
    cv::Point2d P, Q, R;
    double dAB = cv::norm(A - B), dBC = cv::norm(B - C), dCA = cv::norm(C - A);
    if (dAB >= dBC && dAB >= dCA) { P = A; Q = B; R = C; }
    else if (dBC >= dAB && dBC >= dCA) { P = B; Q = C; R = A; }
    else { P = C; Q = A; R = B; }

    cv::Point2d M = (P + Q) * 0.5;
    cv::Point2d D = 2.0 * M - R;
    std::vector<cv::Point2d> quad = {P, Q, R, D};
    cv::Point2d cen(0, 0);
    for (auto& p : quad) { cen.x += p.x; cen.y += p.y; }
    cen.x /= 4.0; cen.y /= 4.0;
    std::sort(quad.begin(), quad.end(), [&](const cv::Point2d& a, const cv::Point2d& b) {
        return std::atan2(a.y - cen.y, a.x - cen.x) < std::atan2(b.y - cen.y, b.x - cen.x);
    });
    std::vector<cv::Point> contour;
    for (auto& p : quad) contour.push_back(cv::Point((int)std::lround(p.x), (int)std::lround(p.y)));
    std::vector<std::vector<cv::Point>> polys{contour};

    cv::Mat mask = cv::Mat::zeros(H, W, CV_8U);
    cv::fillPoly(mask, polys, cv::Scalar(255));
    cv::Mat inner;
    cv::erode(mask, inner, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1);

    // 剔除沿主边（射线）延长方向的条带，避免边界噪声干扰
    int stripe_half = std::max(0, dd.q_parallelogram_exclude_stripe_half_px);
    if (stripe_half > 0 && !q.ray_segments.empty()) {
        cv::Mat exclude = cv::Mat::zeros(H, W, CV_8U);
        double L = 2.0 * std::max(H, W);
        int thick = 2 * stripe_half + 1;
        int nrays = std::min(2, (int)q.ray_segments.size());
        for (int k = 0; k < nrays; k++) {
            cv::Point2d p1(q.ray_segments[k].first.x, q.ray_segments[k].first.y);
            cv::Point2d p2(q.ray_segments[k].second.x, q.ray_segments[k].second.y);
            cv::Point2d v = p2 - p1;
            double nrm = cv::norm(v);
            if (nrm < 1e-3) continue;
            cv::Point2d u(v.x / nrm, v.y / nrm);
            cv::Point2d mid = (p1 + p2) * 0.5;
            cv::Point2d s = mid - u * L;
            cv::Point2d e = mid + u * L;
            cv::line(exclude,
                     cv::Point((int)std::lround(s.x), (int)std::lround(s.y)),
                     cv::Point((int)std::lround(e.x), (int)std::lround(e.y)),
                     cv::Scalar(255), thick, cv::LINE_AA);
        }
        cv::Mat not_excl;
        cv::bitwise_not(exclude, not_excl);
        cv::bitwise_and(inner, not_excl, inner);   // 镜像 Python: inner &= ~exclude（保留抗锯齿部分像素）
    }

    cv::Mat cand;
    cv::bitwise_and(edges_img, edges_img, cand, inner);
    cv::dilate(cand, cand, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1);

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(cand, labels, stats, centroids, 8, CV_32S);
    if (num_labels <= 1) return false;

    const int min_pixels = 50;              // Q_PARALLELOGRAM_MIN_EDGE_PIXELS
    const double span_frac = 0.25;          // Q_PARALLELOGRAM_MIN_SPAN_FRAC
    cv::Point2d diag = Q - P;
    double diag_len = cv::norm(diag);
    if (diag_len <= 1.0) return false;
    cv::Point2d u(diag.x / diag_len, diag.y / diag_len);
    double need_span = span_frac * diag_len;
    for (int lbl = 1; lbl < num_labels; lbl++) {
        int cnt = stats.at<int>(lbl, cv::CC_STAT_AREA);
        if (cnt < min_pixels) continue;
        double min_proj = 1e18, max_proj = -1e18;
        for (int yy = 0; yy < H; yy++) {
            const int* row = labels.ptr<int>(yy);
            for (int xx = 0; xx < W; xx++) {
                if (row[xx] == lbl) {
                    double proj = (xx - P.x) * u.x + (yy - P.y) * u.y;
                    min_proj = std::min(min_proj, proj);
                    max_proj = std::max(max_proj, proj);
                }
            }
        }
        if (min_proj <= max_proj && (max_proj - min_proj) >= need_span) return true;
    }
    return false;
}

// 主体聚类：近竖直边 x 中点间隙二分（镜像 Python find_and_analyze_defects 的 cluster 划分：
// 无 >=2 条竖直边时回退到“全部边的 X/Y 中点最大间隙”二分）
std::vector<int> build_clusters(const std::vector<MergedLine>& edges, double v_tol, int roi_w) {
    std::vector<int> labels(edges.size(), 0);
    std::vector<std::pair<double, int>> xs;
    for (size_t i = 0; i < edges.size(); i++)
        if (is_vert(edges[i].line, v_tol))
            xs.push_back({(edges[i].line[0] + edges[i].line[2]) * 0.5, (int)i});
    if (xs.size() >= 2) {
        std::sort(xs.begin(), xs.end());
        double max_gap = -1; int max_k = -1;
        for (size_t k = 0; k + 1 < xs.size(); k++) {
            double g = xs[k + 1].first - xs[k].first;
            if (g > max_gap) { max_gap = g; max_k = (int)k; }
        }
        double eff_gap = 0.08 * roi_w;
        if (max_k >= 0 && max_gap >= eff_gap) {
            double th = 0.5 * (xs[max_k].first + xs[max_k + 1].first);
            for (size_t i = 0; i < edges.size(); i++) {
                double mx = (edges[i].line[0] + edges[i].line[2]) * 0.5;
                labels[i] = mx <= th ? 0 : 1;
            }
        }
    }
    // Python 回退：未能按竖直边分裂（所有边仍为 0 簇）时，按全部边的 X 中点、其次 Y 中点
    // 的最大间隙二分（Python: np.max(cluster_labels)==0 and len(edges)>=4 and eff_gap>0）
    bool any_one = false;
    for (auto l : labels) if (l == 1) { any_one = true; break; }
    double eff_gap = 0.08 * roi_w;
    if (!any_one && edges.size() >= 4 && eff_gap > 0) {
        for (int dim = 0; dim < 2; dim++) {
            std::vector<std::pair<double, int>> ord;
            for (size_t i = 0; i < edges.size(); i++) {
                double v = dim == 0 ? (edges[i].line[0] + edges[i].line[2]) * 0.5
                                    : (edges[i].line[1] + edges[i].line[3]) * 0.5;
                ord.push_back({v, (int)i});
            }
            std::sort(ord.begin(), ord.end());
            double max_gap = -1; int max_k = -1;
            for (size_t k = 0; k + 1 < ord.size(); k++) {
                double g = ord[k + 1].first - ord[k].first;
                if (g > max_gap) { max_gap = g; max_k = (int)k; }
            }
            if (max_k >= 0 && max_gap >= eff_gap) {
                for (size_t k = 0; k < ord.size(); k++)
                    labels[ord[k].second] = (int)k <= max_k ? 0 : 1;
                break;
            }
        }
    }
    return labels;
}

// 提取玻璃主体轮廓（Python：preprocess_for_defect_edges + 膨胀 + 最大轮廓）
cv::Mat extract_glass_contour(const cv::Mat& roi_gray, const InspectorParams& params,
                              std::vector<cv::Point>& contour_pts, cv::Point2d& centroid) {
    cv::Mat edges = preprocess_for_defect_edges(roi_gray, params.preprocessing);
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat dil;
    cv::dilate(edges, dil, kernel, cv::Point(-1, -1), 1);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dil, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0;
    std::vector<cv::Point> best;
    for (auto& cnt : contours) {
        double a = std::abs(cv::contourArea(cnt));
        double perim = cv::arcLength(cnt, true);
        if (a > 10.0 && perim >= 1200.0 && a > best_area) { best_area = a; best = cnt; }
    }
    if (!best.empty()) {
        contour_pts = best;
        cv::Moments m = cv::moments(best);
        if (m.m00 != 0) centroid = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
        else {
            double sx = 0, sy = 0;
            for (auto& p : best) { sx += p.x; sy += p.y; }
            centroid = cv::Point2d(sx / best.size(), sy / best.size());
        }
    } else {
        centroid = cv::Point2d(roi_gray.cols / 2.0, roi_gray.rows / 2.0);
    }
    return dil;
}

// 射线与多边形轮廓求交（Python _ray_intersect_contour）：返回最小 t 的命中点
bool ray_intersect_contour(const cv::Point2d& origin, const cv::Point2d& dir,
                           const std::vector<cv::Point>& pts, double t_min,
                           cv::Point2d& hit) {
    double dlen = std::hypot(dir.x, dir.y);
    if (dlen < 1e-9) return false;
    double dx = dir.x / dlen, dy = dir.y / dlen;
    bool found = false;
    double best_t = std::numeric_limits<double>::infinity();
    int n = (int)pts.size();
    for (int k = 0; k < n; k++) {
        cv::Point2d a(pts[k].x, pts[k].y), b(pts[(k + 1) % n].x, pts[(k + 1) % n].y);
        double ex = b.x - a.x, ey = b.y - a.y;
        double det = dx * ey - dy * ex;
        if (std::abs(det) < 1e-9) continue;
        double rhsx = a.x - origin.x, rhsy = a.y - origin.y;
        double t = (rhsx * ey - rhsy * ex) / det;
        // 镜像 Python _ray_intersect_contour：u 的分子/分母需与 Python 一致
        // （Python: u = (dx*rhsy - dy*rhsx) / (dy*ex - dx*ey)）。
        // 此处 det = dx*ey - dy*ex = -(dy*ex - dx*ey)，故 u 须取负号，否则命中边错误。
        double u = (dy * rhsx - dx * rhsy) / det;
        if (t >= t_min && u >= 0.0 && u <= 1.0 && t < best_t) {
            best_t = t;
            hit = cv::Point2d(origin.x + dx * t, origin.y + dy * t);
            found = true;
        }
    }
    return found;
}

// 射线加粗条带求交（Python _ray_intersect_contour_thick）
bool ray_intersect_contour_thick(const cv::Point2d& origin, const cv::Point2d& dir,
                                 const std::vector<cv::Point>& pts, double t_min,
                                 int stripe_half, cv::Point2d& hit) {
    cv::Mat edge_mask = cv::Mat::zeros(origin.y > 0 && origin.x > 0 ? 1 : 1, 1, CV_8U);
    // 构建轮廓细线掩膜
    int H = 0, W = 0;
    for (auto& p : pts) { H = std::max(H, p.y + 1); W = std::max(W, p.x + 1); }
    if (H <= 0 || W <= 0) return false;
    edge_mask = cv::Mat::zeros(H, W, CV_8U);
    std::vector<std::vector<cv::Point>> cc{pts};
    cv::polylines(edge_mask, cc, true, cv::Scalar(255), 1);
    double dlen = std::hypot(dir.x, dir.y);
    if (dlen < 1e-9) return false;
    double dx = dir.x / dlen, dy = dir.y / dlen;
    double nx = -dy, ny = dx;
    double T_max = std::max(H, W) * 2.0;
    for (double t = std::max(t_min, 0.0); t <= T_max; t += 1.0) {
        double px = origin.x + dx * t, py = origin.y + dy * t;
        for (int off = -stripe_half; off <= stripe_half; off++) {
            int qx = (int)std::lround(px + nx * off);
            int qy = (int)std::lround(py + ny * off);
            if (qx >= 0 && qx < W && qy >= 0 && qy < H && edge_mask.at<uchar>(qy, qx) != 0) {
                hit = cv::Point2d(qx, qy);
                return true;
            }
        }
    }
    return false;
}

// 角点到轮廓最小距离
double min_dist_to_contour(const cv::Point2d& pt, const std::vector<cv::Point>& pts) {
    double best = std::numeric_limits<double>::infinity();
    for (auto& p : pts) best = std::min(best, std::hypot(pt.x - p.x, pt.y - p.y));
    return best;
}

} // namespace

std::vector<Defect> detect_q_defects(
    const cv::Mat& roi_gray,
    const std::vector<MergedLine>& true_edges,
    const cv::Mat& binary_edges,
    const InspectorParams& params,
    double px_per_mm)
{
    std::vector<Defect> q_defects;
    if (true_edges.size() < 2 || roi_gray.empty()) return q_defects;

    const DefectDetectParams& dd = params.defect_detection;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    double h_tol = v_tol;
    int roi_w = roi_gray.cols, roi_h = roi_gray.rows;

    // ---- 1. 主体聚类 + allowed_pair_lines（每主体最长 1V + 1H）----
    auto labels = build_clusters(true_edges, v_tol, roi_w);
    std::map<int, std::pair<int, double>> best_v, best_h; // idx -> (edge_idx, len)
    for (size_t i = 0; i < true_edges.size(); i++) {
        int cid = labels[i];
        double ang = true_edges[i].angle_deg;
        double L = true_edges[i].length_px;
        if (is_vert(true_edges[i].line, v_tol)) {
            if (best_v[cid].second < L) best_v[cid] = {(int)i, L};
        } else if (is_horiz(true_edges[i].line, h_tol)) {
            if (best_h[cid].second < L) best_h[cid] = {(int)i, L};
        }
    }
    std::vector<bool> allowed(true_edges.size(), false);
    std::vector<int> pairable;
    for (auto& [cid, pick] : best_v) if (pick.first >= 0) { allowed[pick.first] = true; pairable.push_back(pick.first); }
    for (auto& [cid, pick] : best_h) if (pick.first >= 0) { allowed[pick.first] = true; pairable.push_back(pick.first); }
    std::sort(pairable.begin(), pairable.end());
    pairable.erase(std::unique(pairable.begin(), pairable.end()), pairable.end());

    // ---- 2. 角点配对 ----
    struct Corner { int i, j; cv::Point2d pt; };
    std::vector<Corner> corners;
    for (size_t a = 0; a < pairable.size(); a++) {
        for (size_t b = a + 1; b < pairable.size(); b++) {
            int i = pairable[a], j = pairable[b];
            if (labels[i] != labels[j]) continue;
            const cv::Vec4d& l1 = true_edges[i].line;
            const cv::Vec4d& l2 = true_edges[j].line;
            bool hv = (is_horiz(l1, h_tol) && is_vert(l2, v_tol)) ||
                      (is_horiz(l2, h_tol) && is_vert(l1, v_tol));
            if (!hv) continue;
            cv::Point2d inter;
            if (!line_intersection(l1, l2, inter)) {
                if (!fuzzy_hv_intersection(l1, l2, 3, inter) &&
                    !fuzzy_hv_intersection(l2, l1, 3, inter)) continue;
            }
            if (!(inter.x >= 0 && inter.x < roi_w && inter.y >= 0 && inter.y < roi_h)) continue;
            // 与 Python 对齐：corner 存储顺序固定为 (竖直边, 水平边)。
            // Python 的 supplement 收集是 idx_vertical 外层 × idx_horizontal 内层 → (V,H)；
            // 若按索引存 (H,V)，三角顶点顺序不同 → 平行四边形差 1px → 过滤结果翻转
            // （如 cam-1_ts1765692711692 的 Q@(860,619)）。
            if (is_horiz(l1, h_tol) && is_vert(l2, v_tol)) {
                corners.push_back({j, i, inter});
            } else {
                corners.push_back({i, j, inter});
            }
        }
    }

    // ---- 3. 玻璃主体轮廓 ----
    std::vector<cv::Point> cnt_pts;
    cv::Point2d cnt_center;
    cv::Mat edges_qc_dil = extract_glass_contour(roi_gray, params, cnt_pts, cnt_center);

    // ---- 4. 逐角点 Q 三角形判定 ----
    double min_corner_dist = dd.q_corner_contour_min_dist_px > 0 ? dd.q_corner_contour_min_dist_px : 16.0;
    double min_side_mm = dd.min_width_mm > 0 ? dd.min_width_mm : 5.0;      // 宽 >= 5mm
    double min_area_mm2 = 25.0;                                             // 面积 >= 25mm2
    double min_hit_dist_px = 5.0 * px_per_mm;                               // 命中点距角点 >= 5mm

    struct QCand {
        Defect d;
        double tri_area = 0;
        cv::Point2d center;
    };
    std::vector<QCand> cands;

    for (auto& c : corners) {
        const cv::Point2d& cp = c.pt;
        if (!cnt_pts.empty() && min_dist_to_contour(cp, cnt_pts) <= min_corner_dist) continue;

        // 对两条主边分别沿端点方向发射射线
        std::vector<cv::Point2d> hits;      // 命中点
        std::vector<cv::Point2d> chosen_dirs;
        std::vector<cv::Vec4d> segs = {true_edges[c.i].line, true_edges[c.j].line};
        for (auto& seg : segs) {
            cv::Point2d p1(seg[0], seg[1]), p2(seg[2], seg[3]);
            struct CandHit { double t; cv::Point2d pt; cv::Point2d dir; };
            std::vector<CandHit> cands_ray;
            for (cv::Point2d tgt : {p1, p2}) {
                cv::Point2d v = tgt - cp;
                double n = std::hypot(v.x, v.y);
                if (n <= 1e-6) continue;
                cv::Point2d u0(v.x / n, v.y / n);
                cv::Point2d hit;
                if (ray_intersect_contour(cp, u0, cnt_pts, 1.0, hit))
                    cands_ray.push_back({std::hypot(hit.x - cp.x, hit.y - cp.y), hit, u0});
            }
            if (cands_ray.empty() && !cnt_pts.empty()) {
                for (cv::Point2d tgt : {p1, p2}) {
                    cv::Point2d v = tgt - cp;
                    double n = std::hypot(v.x, v.y);
                    if (n <= 1e-6) continue;
                    cv::Point2d u0(v.x / n, v.y / n);
                    cv::Point2d hit;
                    if (ray_intersect_contour_thick(cp, u0, cnt_pts, 1.0, 7, hit))
                        cands_ray.push_back({std::hypot(hit.x - cp.x, hit.y - cp.y), hit, u0});
                }
            }
            if (!cands_ray.empty()) {
                // 优先选择指向主体中心的候选，否则最短 t
                cv::Point2d inward = cnt_center - cp;
                double in_norm = std::hypot(inward.x, inward.y);
                bool has_inward = false;
                double best_dot = -1e18; size_t best_k = 0;
                if (in_norm > 1e-6) {
                    cv::Point2d in_dir(inward.x / in_norm, inward.y / in_norm);
                    for (size_t k = 0; k < cands_ray.size(); k++) {
                        double dot = cands_ray[k].dir.x * in_dir.x + cands_ray[k].dir.y * in_dir.y;
                        if (dot > 0 && dot > best_dot) { best_dot = dot; best_k = k; has_inward = true; }
                    }
                }
                if (!has_inward) {
                    best_k = 0;
                    for (size_t k = 1; k < cands_ray.size(); k++)
                        if (cands_ray[k].t < cands_ray[best_k].t) best_k = k;
                }
                hits.push_back(cands_ray[best_k].pt);
                chosen_dirs.push_back(cands_ray[best_k].dir);
            } else {
                chosen_dirs.push_back(cv::Point2d(0, 0)); // miss
            }
        }

#ifdef CPP_DEBUG_Q
        std::cerr << "[Q-CORNER] roi cp=(" << cp.x << "," << cp.y << ") i=" << c.i << " j=" << c.j
                  << " seg0=(" << true_edges[c.i].line[0] << "," << true_edges[c.i].line[1] << ")->("
                  << true_edges[c.i].line[2] << "," << true_edges[c.i].line[3] << ")"
                  << " seg1=(" << true_edges[c.j].line[0] << "," << true_edges[c.j].line[1] << ")->("
                  << true_edges[c.j].line[2] << "," << true_edges[c.j].line[3] << ")"
                  << " hits=" << hits.size();
        for (size_t hk = 0; hk < hits.size(); hk++)
            std::cerr << " hit" << hk << "=(" << hits[hk].x << "," << hits[hk].y << ") dir=("
                      << chosen_dirs[hk].x << "," << chosen_dirs[hk].y << ")";
        std::cerr << std::endl;
#endif

        // ---- 双边命中：三角形法 ----
        // 未命中哨兵为 (0,0)，须同时检查 x/y；垂直射线方向 (0,±1) 的 x 为 0 不能算未命中
        bool hit0_ok = (chosen_dirs.size() > 0 && (chosen_dirs[0].x != 0 || chosen_dirs[0].y != 0));
        bool hit1_ok = (chosen_dirs.size() > 1 && (chosen_dirs[1].x != 0 || chosen_dirs[1].y != 0));
        if (hits.size() == 2 && hit0_ok && hit1_ok) {
            double d1 = std::hypot(hits[0].x - cp.x, hits[0].y - cp.y);
            double d2 = std::hypot(hits[1].x - cp.x, hits[1].y - cp.y);
            if (d1 < min_hit_dist_px || d2 < min_hit_dist_px) continue;

            std::vector<cv::Point> tri = {cv::Point((int)cp.x, (int)cp.y),
                                          cv::Point((int)hits[0].x, (int)hits[0].y),
                                          cv::Point((int)hits[1].x, (int)hits[1].y)};
            double tri_area_px = std::abs(cv::contourArea(tri));
            if (tri_area_px <= 1.0) continue;

            cv::RotatedRect r = cv::minAreaRect(tri);
            double width_mm = std::min(r.size.width, r.size.height) / px_per_mm;
            double length_mm = std::max(r.size.width, r.size.height) / px_per_mm;
            double area_mm2 = (r.size.width * r.size.height) / (px_per_mm * px_per_mm);
            if (width_mm >= min_side_mm && area_mm2 >= min_area_mm2) {
                cv::Point2f box[4];
                r.points(box);
                Defect d;
                d.type = "Q";
                d.x = (int)std::lround(r.center.x);
                d.y = (int)std::lround(r.center.y);
                d.width_mm = width_mm;
                d.length_mm = length_mm;
                d.size_mm = length_mm;
                d.angle_deg = 0;
                d.confidence = 0.8;
                d.pixel_area = (int)std::lround(tri_area_px);
                for (auto& p : box) d.box_points.push_back(cv::Point((int)std::lround(p.x), (int)std::lround(p.y)));
                d.region_contour = tri;
                d.ray_segments = {{tri[0], tri[1]}, {tri[0], tri[2]}};
                QCand qc;
                qc.d = d;
                qc.tri_area = tri_area_px;
                qc.center = cv::Point2d(r.center.x, r.center.y);
                cands.push_back(qc);
            }
        }
        // 单边命中（竖直 miss + 水平 hit）：垂直裁剪三角形法（简化：选亮度暗侧端点）
        else if (hits.size() == 1) {
            bool d0_ok = (chosen_dirs.size() > 0 && (chosen_dirs[0].x != 0 || chosen_dirs[0].y != 0));
            int hit_k = d0_ok ? 0 : 1;
            int miss_k = 1 - hit_k;
            const cv::Vec4d& seg_hit = segs[hit_k];
            const cv::Vec4d& seg_miss = segs[miss_k];
            if (!(is_vert(seg_miss, v_tol) && is_horiz(seg_hit, h_tol))) continue;
            cv::Point2d pt_hit = hits[hit_k];
            cv::Point2d vm_p1(seg_miss[0], seg_miss[1]), vm_p2(seg_miss[2], seg_miss[3]);
            cv::Point2d vv = vm_p2 - vm_p1;
            double vlen = std::hypot(vv.x, vv.y);
            if (vlen < 1e-6) continue;
            cv::Point2d v_dir(vv.x / vlen, vv.y / vlen);
            // 按竖直方向上下半区平均亮度选暗侧端点
            cv::Point2d v_clip;
            double t1 = (vm_p1.x - cp.x) * v_dir.x + (vm_p1.y - cp.y) * v_dir.y;
            double t2 = (vm_p2.x - cp.x) * v_dir.x + (vm_p2.y - cp.y) * v_dir.y;
            cv::Point2d v_up = t1 >= t2 ? vm_p1 : vm_p2;
            cv::Point2d v_dn = (t1 >= t2) ? vm_p2 : vm_p1;
            // 半区平均亮度（简化：以 cp 为界沿 v_dir 划分）
            double sum_up = 0, sum_dn = 0; int cnt_up = 0, cnt_dn = 0;
            int step = std::max(1, roi_h / 60);
            for (int yy = 0; yy < roi_h; yy += step) {
                for (int xx = 0; xx < roi_w; xx += step) {
                    double dot = (xx - cp.x) * v_dir.x + (yy - cp.y) * v_dir.y;
                    if (dot > 0) { sum_up += roi_gray.at<uchar>(yy, xx); cnt_up++; }
                    else { sum_dn += roi_gray.at<uchar>(yy, xx); cnt_dn++; }
                }
            }
            if (cnt_up > 0 && cnt_dn > 0) {
                v_clip = (sum_up / cnt_up < sum_dn / cnt_dn) ? v_up : v_dn;
            } else {
                v_clip = v_up;
            }
            std::vector<cv::Point> tri = {cv::Point((int)cp.x, (int)cp.y),
                                          cv::Point((int)pt_hit.x, (int)pt_hit.y),
                                          cv::Point((int)v_clip.x, (int)v_clip.y)};
            double tri_area_px = std::abs(cv::contourArea(tri));
            if (tri_area_px <= 1.0) continue;
            cv::RotatedRect r = cv::minAreaRect(tri);
            double width_mm = std::min(r.size.width, r.size.height) / px_per_mm;
            double length_mm = std::max(r.size.width, r.size.height) / px_per_mm;
            double area_mm2 = (r.size.width * r.size.height) / (px_per_mm * px_per_mm);
            if (width_mm >= min_side_mm && area_mm2 >= min_area_mm2) {
                cv::Point2f box[4];
                r.points(box);
                Defect d;
                d.type = "Q";
                d.x = (int)std::lround(r.center.x);
                d.y = (int)std::lround(r.center.y);
                d.width_mm = width_mm;
                d.length_mm = length_mm;
                d.size_mm = length_mm;
                d.confidence = 0.75;
                d.pixel_area = (int)std::lround(tri_area_px);
                for (auto& p : box) d.box_points.push_back(cv::Point((int)std::lround(p.x), (int)std::lround(p.y)));
                d.region_contour = tri;
                d.ray_segments = {{tri[0], tri[1]}};
                QCand qc;
                qc.d = d;
                qc.tri_area = tri_area_px;
                qc.center = cv::Point2d(r.center.x, r.center.y);
                cands.push_back(qc);
            }
        }
    }

    // ---- 5. 去重：中心距离 <= 12px 保留三角形面积大者 ----
    const double dedup_dist = 12.0;
    std::vector<QCand> dedup;
    for (auto& qc : cands) {
        bool picked = false;
        for (size_t k = 0; k < dedup.size(); k++) {
            if (std::hypot(qc.center.x - dedup[k].center.x, qc.center.y - dedup[k].center.y) <= dedup_dist) {
                if (qc.tri_area > dedup[k].tri_area) dedup[k] = qc;
                picked = true;
                break;
            }
        }
        if (!picked) dedup.push_back(qc);
    }
    for (auto& qc : dedup) {
        // 平行四边形 + 边缘点集群验证（镜像 Python）：内部无足够缺陷边缘的 Q 视为误报
        if (q_parallelogram_cluster_ok(qc.d, binary_edges, dd)) q_defects.push_back(qc.d);
    }
    return q_defects;
}
