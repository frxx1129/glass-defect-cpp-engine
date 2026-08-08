#include "static_artifact.h"
#include <sstream>
#include <chrono>
#include <cstdint>
#include <unordered_set>

StaticArtifactTracker::StaticArtifactTracker(const EngineConfig& config) {
    min_consecutive_ = config.static_artifact_min_consecutive;
    grid_size_ = config.static_artifact_grid_size;
    cooldown_frames_ = config.static_artifact_cooldown_frames;
    report_interval_ = config.static_artifact_report_interval;
}

std::string StaticArtifactTracker::make_cell_key(int cx, int cy) {
    return std::to_string(cx) + "_" + std::to_string(cy);
}

std::vector<Defect> StaticArtifactTracker::filter(
    const std::vector<Defect>& defects,
    const std::string& cam_key)
{
    if (!cam_key.empty() && state_.find(cam_key) == state_.end()) {
        CameraState cs;
        cs.total = 0;
        state_[cam_key] = cs;
    }

    CameraState& tracker = state_[cam_key];
    tracker.total++;

    // ---- 当前帧缺陷位置网格化 ----
    std::unordered_set<std::string> current_cells;
    for (const auto& defect : defects) {
        int gx = defect.location.x;
        int gy = defect.location.y;
        if (gx < 0 || gy < 0) continue;
        int cx = (gx / grid_size_) * grid_size_;
        int cy = (gy / grid_size_) * grid_size_;
        current_cells.insert(make_cell_key(cx, cy));
    }

    // ---- 更新网格状态 ----
    std::vector<std::string> to_erase;
    for (auto& [cell_key, info] : tracker.cells) {
        if (current_cells.find(cell_key) != current_cells.end()) {
            info.streak += 1;
            info.absent = 0;
        } else {
            info.absent += 1;
            if (info.absent > cooldown_frames_) {
                to_erase.push_back(cell_key);
            }
        }
    }
    for (const auto& key : to_erase) {
        tracker.cells.erase(key);
    }

    // 新出现的网格
    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    for (const auto& cell_key : current_cells) {
        if (tracker.cells.find(cell_key) == tracker.cells.end()) {
            CellInfo info;
            info.streak = 1;
            info.absent = 0;
            info.suppressed = false;
            info.first_seen = static_cast<int64_t>(now_sec);
            tracker.cells[cell_key] = info;
        }
    }

    // ---- 标记需要抑制的网格 ----
    std::unordered_set<std::string> suppressed_cells;
    for (auto& [cell_key, info] : tracker.cells) {
        if (info.streak >= min_consecutive_) {
            info.suppressed = true;
            suppressed_cells.insert(cell_key);
        } else if (info.suppressed) {
            suppressed_cells.insert(cell_key);
        }
    }

    // ---- 过滤缺陷 ----
    std::vector<Defect> filtered;
    int suppressed_count = 0;
    for (const auto& defect : defects) {
        int gx = defect.location.x;
        int gy = defect.location.y;
        int cx = (gx / grid_size_) * grid_size_;
        int cy = (gy / grid_size_) * grid_size_;
        std::string cell_key = make_cell_key(cx, cy);
        if (suppressed_cells.find(cell_key) != suppressed_cells.end()) {
            suppressed_count++;
            continue;
        }
        filtered.push_back(defect);
    }

    // ---- 全部被抑制时，按间隔发 S 类型兜底心跳 ----
    int total = tracker.total;
    if (report_interval_ > 0) {
        if (filtered.empty() && !defects.empty() && suppressed_count > 0) {
            if (total % report_interval_ == 0 || total <= 5) {
                Defect s = defects[0];
                s.type = "S";
                filtered.push_back(s);
            }
        }
    }
    // report_interval_ <= 0: 不生成 S 类型心跳

    return filtered;
}
