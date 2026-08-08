#include "engine/detector.h"
#include "engine/json_io.h"
#include <iostream>
#include <string>
#include <fstream>
#include <memory>

#ifdef _WIN32
#include <windows.h>
// Windows 命令行参数 ANSI(GBK) -> UTF-8
static std::string ansi_to_utf8(const std::string& s) {
    int wlen = MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, nullptr, 0);
    if (wlen <= 0) return s;
    std::wstring wstr(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, &wstr[0], wlen);
    int ulen = WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (ulen <= 0) return s;
    std::string utf8(static_cast<size_t>(ulen), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &utf8[0], ulen, nullptr, nullptr);
    if (!utf8.empty() && utf8.back() == '\0') utf8.pop_back();
    return utf8;
}
#endif

int main(int argc, char* argv[]) {
    // 模式 1：glass_engine image.png           单图（自动找同目录 config.json）
    // 模式 2：glass_engine < input.json         stdin JSON 请求
    // 模式 3：glass_engine --batch < input.jsonl 批量（跨帧保持 Detector 状态）
    try {
        if (argc > 1 && std::string(argv[1]) == "--batch") {
            // 批量模式：每行一个 JSON 请求，复用 Detector 保持静态抑制状态
            std::unique_ptr<Detector> detector;
            std::string line;
            while (std::getline(std::cin, line)) {
                if (line.empty()) continue;
                try {
                    auto req = parse_request(line);
                    if (!detector) detector = std::make_unique<Detector>(req.config);
                    detector->set_cam_key(req.config.cam_key);
                    auto result = detector->detect_file(req.image_path);
                    auto out = result_to_json(result);
                    out["status"] = "ok";
                    std::cout << out.dump() << std::endl;
                } catch (const std::exception& e) {
                    json err;
                    err["status"] = "error";
                    err["message"] = e.what();
                    std::cerr << err.dump() << std::endl;
                }
            }
        } else if (argc > 1 && std::string(argv[1]) != "--batch") {
            // 单图模式
            std::string img_path;
#ifdef _WIN32
            img_path = ansi_to_utf8(argv[1]);
#else
            img_path = argv[1];
#endif
            EngineConfig cfg;
            // 尝试从同目录 config.json 读取完整配置
            std::ifstream cfg_file("config.json");
            if (cfg_file.is_open()) {
                json cfg_json;
                cfg_file >> cfg_json;
                cfg = parse_config(cfg_json);
            }
            Detector detector(cfg);
            auto result = detector.detect_file(img_path);
            auto out = result_to_json(result);
            out["image_path"] = img_path;
            out["status"] = "ok";
            std::cout << out.dump(2) << std::endl;
        } else {
            // stdin JSON 模式
            std::string input, line;
            while (std::getline(std::cin, line)) input += line;
            if (input.empty()) {
                json help;
                help["usage"] = "glass_engine <image_path> | --batch";
                help["modes"] = {
                    "1. glass_engine image.png  (单图)",
                    "2. echo '{\"image_path\":\"...\"}' | glass_engine  (stdin JSON)",
                    "3. glass_engine --batch < input.jsonl  (批量)"
                };
                std::cout << help.dump(2) << std::endl;
                return 0;
            }
            auto req = parse_request(input);
            Detector detector(req.config);
            detector.set_cam_key(req.config.cam_key);
            auto result = detector.detect_file(req.image_path);
            auto out = result_to_json(result);
            out["status"] = "ok";
            std::cout << out.dump() << std::endl;
        }
    } catch (const std::exception& e) {
        json err;
        err["status"] = "error";
        err["message"] = e.what();
        std::cerr << err.dump() << std::endl;
        return 1;
    }
    return 0;
}
