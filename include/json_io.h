#pragma once
#include "defect.h"
#include <string>
#include "json.hpp"

using json = nlohmann::json;

// 从 JSON 解析配置
EngineConfig parse_config(const json& j);

// 检测结果转 JSON
json result_to_json(const DetectionResult& result);

// 从 stdin 读取请求
struct DetectionRequest {
    std::string image_path;
    EngineConfig config;
    int total_frames = 0;
};

DetectionRequest parse_request(const std::string& json_str);

// 从 ROI JSON 文件读取 ROI 列表
// 格式: { "group_name": { "averaged_rois": [{"x":..., "y":..., "width":..., "height":...}, ...] } }
std::vector<RoiRect> load_rois_from_file(const std::string& file_path);
