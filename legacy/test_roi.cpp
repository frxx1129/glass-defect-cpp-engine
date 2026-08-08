#include "json_io.h"
#include <iostream>

int main() {
    auto rois = load_rois_from_file("F:/玻璃检测/glass-defect-algo-main/glass-defect-algo-main/roi_averaged_by_group_CORRECTED.json");
    std::cout << "Loaded " << rois.size() << " ROIs" << std::endl;
    for (auto& r : rois) {
        std::cout << "  ROI: x=" << r.x << " y=" << r.y << " w=" << r.width << " h=" << r.height << std::endl;
    }
    return 0;
}
