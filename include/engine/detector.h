#pragma once
#include "engine/config.h"
#include "engine/types.h"
#include "engine/line_merge.h"
#include "engine/static_artifact.h"
#include <map>
#include <memory>

// 多 ROI 缺陷检测器
// 定位：作为 Python 端计算模块的 C++ 替代品，通信/调度仍由 Python 负责。
class Detector {
public:
    explicit Detector(const EngineConfig& config);
    ~Detector() = default;

    // 对单张灰度图执行多 ROI 检测（镜像 Python process_image_from_memory_parallel）
    DetectionResult detect(const cv::Mat& image_gray);

    // 从文件加载图像并检测（UTF-8 路径安全）
    DetectionResult detect_file(const std::string& image_path);

    // 跨帧状态（静态抑制）按相机隔离
    void set_cam_key(const std::string& key) { cam_key_ = key; }

    // 配置 ROI 模板（优先于 config.roi_template_file）
    void set_rois(const std::vector<RoiRect>& rois) { external_rois_ = rois; }

private:
    EngineConfig config_;
    double px_per_mm_ = 2.44;
    std::string cam_key_;
    std::vector<RoiRect> external_rois_;

    // 每相机一个静态抑制器（跨帧持久）
    std::map<std::string, StaticArtifactTracker> trackers_;

    // 单 ROI 检测：预处理 -> Hough -> 直线合并 -> 缺陷分类
    // merged_in: 预计算的合并主边（空则内部计算）；shared_verticals: 跨 ROI 共享竖直边（全局坐标，空则跳过注入）
    std::vector<Defect> process_roi(
        const cv::Mat& image_gray,
        const RoiRect& roi,
        const InspectorParams& params,
        int roi_idx,
        const std::vector<MergedLine>& merged_in = {},
        const std::vector<cv::Vec4d>& shared_verticals = {});

    // 加载图像（UTF-8 路径、内存解码）
    cv::Mat load_image(const std::string& image_path);

    // 缺陷过滤链（镜像 Python 各 filter 阶段；逐步补全）
    std::vector<Defect> apply_filter_chain(std::vector<Defect> defects);
};
