#pragma once
#include "defect.h"
#include <string>
#include <vector>
#include <unordered_map>

// 静态干扰（DNN 模板等固定位置缺陷）抑制器
// 通过网格化追踪每个缺陷位置的出现频率：
//   - streak >= min_consecutive → 认为是静态干扰，抑制输出
//   - absent > cooldown_frames  → 该网格不再追踪
//   - 全部被抑制时，按 report_interval 发 S 类型兜底心跳
class StaticArtifactTracker {
public:
    struct CellInfo {
        int streak = 0;
        int absent = 0;
        bool suppressed = false;
        int64_t first_seen = 0;
    };

    struct CameraState {
        int total = 0;
        std::unordered_map<std::string, CellInfo> cells;
    };

    explicit StaticArtifactTracker(const EngineConfig& config);

    // 对指定相机的缺陷列表进行静态抑制过滤
    // cam_key 格式："{line_name}_cam{cam_index}" 或 "cam{cam_index}"
    std::vector<Defect> filter(const std::vector<Defect>& defects, const std::string& cam_key);

    // 序列化 / 反序列化状态（JSON 持久化）
    std::unordered_map<std::string, CameraState>& get_all_states() { return state_; }
    void set_all_states(const std::unordered_map<std::string, CameraState>& states) { state_ = states; }

private:
    int min_consecutive_;
    int grid_size_;
    int cooldown_frames_;
    int report_interval_;
    std::unordered_map<std::string, CameraState> state_;

    static std::string make_cell_key(int cx, int cy);
};
