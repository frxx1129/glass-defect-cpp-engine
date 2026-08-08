#pragma once
#include "defect.h"
#include "static_artifact.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>
#include <memory>

class Detector {
public:
    explicit Detector(const EngineConfig& config);
    ~Detector() = default;

    // 主检测接口
    DetectionResult detect(const std::string& image_path);

    // 获取静态抑制器引用（用于持久化状态）
    StaticArtifactTracker& get_static_tracker() { return *static_tracker_; }

private:
    EngineConfig config_;
    double px_per_mm_;
    std::unique_ptr<StaticArtifactTracker> static_tracker_;

    // 图像预处理
    cv::Mat load_and_preprocess(const std::string& image_path);

    // ROI 裁剪
    cv::Mat apply_roi(const cv::Mat& image);

    // 边缘检测
    cv::Mat edge_detect(const cv::Mat& gray);

    // 直线检测
    std::vector<cv::Vec4i> detect_lines(const cv::Mat& edges);

    // 直线合并去重（匹配 Python merge_lines_and_get_main_edges）
    std::vector<cv::Vec4i> merge_lines(const std::vector<cv::Vec4i>& lines);

    // 缺陷分类
    std::vector<Defect> classify_defects(
        const cv::Mat& gray,
        const cv::Mat& edges,
        const std::vector<cv::Vec4i>& lines);

    // Q类型角点缺陷检测
    std::vector<Defect> detect_corner_defects(
        const cv::Mat& gray,
        const cv::Mat& edges,
        const std::vector<cv::Vec4i>& lines,
        const std::vector<Defect>& existing_defects);

    // E line suppression
    std::vector<Defect> suppress_e_lines(
        std::vector<Defect>& defects,
        const cv::Mat& edges,
        const std::vector<cv::Vec4i>& lines);

    // 亮度扫描检测（沿边缘扫描暗区）
    std::vector<Defect> scan_luminosity_defects(
        const cv::Mat& gray,
        const cv::Mat& edges,
        const std::vector<cv::Vec4i>& lines);

    // B/L 重分类
    void reclassify_b_to_l(std::vector<Defect>& defects, const std::vector<cv::Vec4i>& lines);

    // 阴影过滤
    std::vector<Defect> filter_shadow_defects(std::vector<Defect>& defects);

    // E 型边缘异常检测（轮廓分析）
    std::vector<Defect> detect_skew_defects(
        const cv::Mat& gray,
        const cv::Mat& edges,
        const std::vector<cv::Vec4i>& merged_lines,
        const std::vector<RoiRect>& rois);

    // 静态干扰过滤
    std::vector<Defect> filter_static_artifacts(
        std::vector<Defect>& defects,
        int total_frames);
};
