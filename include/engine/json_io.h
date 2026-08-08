#pragma once
#include "engine/config.h"
#include "engine/types.h"
#include <string>
#include "json.hpp"

using json = nlohmann::json;

// 从 JSON 解析引擎总配置（对齐 Python config.yaml 结构）
EngineConfig parse_config(const json& j);

// 检测结果转 JSON（含 per-ROI 与标注图路径）
json result_to_json(const DetectionResult& result);

// 从 ROI JSON 模板文件读取 ROI 列表
// 格式: { "group_name": { "source_image_count": N, "averaged_rois": [...] } }
std::vector<RoiRect> load_rois_from_file(const std::string& file_path);

// 请求结构（stdin JSON / 命令行）
struct DetectionRequest {
    std::string image_path;
    EngineConfig config;
    int total_frames = 0;
};

DetectionRequest parse_request(const std::string& json_str);
