#include "engine/classify.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace {

// 以线段为中心线构造四边形轮廓（box_points）
// 沿线方向半长 = length/2，法线方向半宽 = thickness/2（像素）
std::vector<cv::Point> make_box_points(const cv::Vec4d& line, double thickness_px) {
    double dx = line[2] - line[0], dy = line[3] - line[1];
    double len = std::hypot(dx, dy);
    std::vector<cv::Point> box;
    if (len < 1e-6) return box;
    double ux = dx / len, uy = dy / len;
    double nx = -uy, ny = ux;
    double hw = std::max(1.0, thickness_px * 0.5);
    // 四个角：p0±n, p1±n
    box.push_back(cv::Point((int)std::lround(line[0] + nx * hw), (int)std::lround(line[1] + ny * hw)));
    box.push_back(cv::Point((int)std::lround(line[2] + nx * hw), (int)std::lround(line[3] + ny * hw)));
    box.push_back(cv::Point((int)std::lround(line[2] - nx * hw), (int)std::lround(line[3] - ny * hw)));
    box.push_back(cv::Point((int)std::lround(line[0] - nx * hw), (int)std::lround(line[1] - ny * hw)));
    return box;
}

// 线段角度（0-180，90=竖直）
double angle_x_deg(const cv::Vec4d& l) {
    double a = std::atan2(std::abs(l[3] - l[1]), std::abs(l[2] - l[0])) * 180.0 / CV_PI;
    return a;
}

// 近竖直/近水平
bool is_vert(const cv::Vec4d& l, double tol) { return angle_x_deg(l) >= (90.0 - tol); }
bool is_horiz(const cv::Vec4d& l, double tol) { return angle_x_deg(l) <= tol; }

} // namespace

std::vector<Defect> find_and_analyze_defects(
    const std::vector<MergedLine>& edges,
    const cv::Mat& roi_gray,
    const cv::Size& roi_dims,
    const InspectorParams& params,
    double px_per_mm)
{
    std::vector<Defect> defects;
    if (edges.empty() || px_per_mm <= 0) return defects;

    const DefectDetectParams& dd = params.defect_detection;
    double v_tol = dd.vertical_angle_tol_deg > 0 ? dd.vertical_angle_tol_deg : 10.0;
    double h_tol = dd.angle_deviation_tolerance > 0 ? dd.angle_deviation_tolerance : 15.0;
    double ppm = px_per_mm;

    // 主体聚类：近竖直边 x 中点间隙二分（镜像 Python）
    std::vector<int> cluster_labels(edges.size(), 0);
    {
        std::vector<int> vert_idx;
        for (size_t i = 0; i < edges.size(); i++)
            if (is_vert(edges[i].line, v_tol)) vert_idx.push_back((int)i);
        if (vert_idx.size() >= 2) {
            std::vector<std::pair<double, int>> xs;
            for (int i : vert_idx) xs.push_back({(edges[i].line[0] + edges[i].line[2]) * 0.5, i});
            std::sort(xs.begin(), xs.end());
            double max_gap = -1; int max_k = -1;
            for (size_t k = 0; k + 1 < xs.size(); k++) {
                double g = xs[k + 1].first - xs[k].first;
                if (g > max_gap) { max_gap = g; max_k = (int)k; }
            }
            double eff_gap = 0.08 * roi_dims.width; // 默认 8% ROI 宽
            if (max_k >= 0 && max_gap >= eff_gap) {
                double x_thresh = 0.5 * (xs[max_k].first + xs[max_k + 1].first);
                for (size_t i = 0; i < edges.size(); i++) {
                    double mx = (edges[i].line[0] + edges[i].line[2]) * 0.5;
                    cluster_labels[i] = mx <= x_thresh ? 0 : 1;
                }
            }
        }
    }

    for (size_t i = 0; i < edges.size(); i++) {
        const MergedLine& ml = edges[i];
        const cv::Vec4d& line = ml.line;
        double length_mm = ml.length_px / ppm;
        double ang = angle_x_deg(line);

        // 尺寸门槛（Python: min/max defect size）
        if (length_mm < dd.min_defect_size_mm) continue;
        if (length_mm > dd.max_defect_size_mm) continue;

        Defect d;
        d.type = "B"; // 默认崩边
        d.x = (int)std::lround((line[0] + line[2]) * 0.5);
        d.y = (int)std::lround((line[1] + line[3]) * 0.5);
        d.length_mm = length_mm;
        d.angle_deg = ang;
        d.roi_index = -1; // 由调用方设置

        // 基础类型判定（镜像 Python 角度判定主路径）
        if (is_horiz(line, dd.q_angle_threshold_deg > 0 ? dd.q_angle_threshold_deg : 5.0)
            && length_mm >= dd.l_min_length_mm) {
            d.type = "L"; // 近水平长线 → 裂纹
            d.width_mm = dd.min_width_mm;
            d.height_mm = length_mm;
        } else if (is_vert(line, dd.q_angle_threshold_deg > 0 ? dd.q_angle_threshold_deg : 5.0)) {
            d.type = "Q"; // 近竖直 → 缺角候选
            d.width_mm = length_mm;
            d.height_mm = dd.min_width_mm;
        } else {
            d.type = "B"; // 斜向 → 崩边
            d.width_mm = length_mm * std::cos(ang * CV_PI / 180.0) / ppm;
            d.height_mm = length_mm * std::sin(ang * CV_PI / 180.0) / ppm;
            if (d.width_mm < 0.1) d.width_mm = dd.min_width_mm;
            if (d.height_mm < 0.1) d.height_mm = dd.min_width_mm;
        }

        d.size_mm = std::max(d.width_mm, d.height_mm);
        d.box_points = make_box_points(line, dd.min_width_mm * ppm);
        d.confidence = 0.8;
        defects.push_back(d);
    }
    return defects;
}
