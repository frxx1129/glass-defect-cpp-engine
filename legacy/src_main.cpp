#include "detector.h"
#include "json_io.h"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>
#include <iomanip>

#ifdef _WIN32
#include <windows.h>
// Windows 下命令行参数是 ANSI(GBK) 编码，转成 UTF-8
static std::string ansi_to_utf8(const std::string& ansi_str) {
    int wlen = MultiByteToWideChar(CP_ACP, 0, ansi_str.c_str(), -1, nullptr, 0);
    if (wlen <= 0) return ansi_str;
    std::wstring wstr(static_cast<size_t>(wlen), L'\0');
    MultiByteToWideChar(CP_ACP, 0, ansi_str.c_str(), -1, &wstr[0], wlen);
    int ulen = WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (ulen <= 0) return ansi_str;
    std::string utf8(static_cast<size_t>(ulen), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &utf8[0], ulen, nullptr, nullptr);
    if (!utf8.empty() && utf8.back() == '\0') utf8.pop_back();
    return utf8;
}
#endif

int main(int argc, char* argv[]) {
    // 从命令行参数或 stdin 读取 JSON
    std::string input_json;
    if (argc > 1 && std::string(argv[1]) == "--batch") {
        // 批量模式：从 stdin 一行一条 JSON，跨帧保持 Detector 状态
        // 使用首行的配置创建 Detector，后续复用同一个实例
        std::unique_ptr<Detector> detector_ptr;
        // 尝试从 config.json 加载完整配置
        EngineConfig file_cfg;
        {
            std::ifstream cfg_file("config.json");
            if (cfg_file.is_open()) {
                json cfg_json;
                cfg_file >> cfg_json;
                file_cfg = parse_config(cfg_json);
            }
        }
        std::string line;
        while (std::getline(std::cin, line)) {
            if (line.empty()) continue;
            try {
                auto req = parse_request(line);
                if (!detector_ptr) {
                    EngineConfig cfg = file_cfg;
                    // 合并 stdin 传入的配置（字段不为默认值时覆盖）
                    // cfg 已有完整配置，优先用文件配置
                    detector_ptr = std::make_unique<Detector>(cfg);
                }
                auto result = detector_ptr->detect(req.image_path);
                auto output = result_to_json(result);
                output["status"] = "ok";
                std::cout << output.dump() << std::endl;
            }
            catch (const std::exception& e) {
                json err;
                err["status"] = "error";
                err["message"] = e.what();
                std::cerr << err.dump() << std::endl;
            }
        }
    }
    else if (argc > 1) {
        // 单图模式：命令行参数 = 图片路径
        try {
            // 从标准配置加载（后续支持从文件读取 config）
            EngineConfig cfg;
            // 尝试从同路径 config.json 读取
            std::ifstream cfg_file("config.json");
            if (cfg_file.is_open()) {
                json cfg_json;
                cfg_file >> cfg_json;
                cfg = parse_config(cfg_json);
            }
            // 覆盖：图片路径
            std::string image_path_utf8;
#ifdef _WIN32
            image_path_utf8 = ansi_to_utf8(argv[1]);
#else
            image_path_utf8 = argv[1];
#endif
            auto req_json = json::object();
            req_json["image_path"] = image_path_utf8;
            req_json["config"] = json::object();
            auto req = parse_request(req_json.dump());
            Detector detector(cfg);
            auto result = detector.detect(req.image_path);

            // 输出标准输出结果 + stderr 日志
            auto output = result_to_json(result);
            output["image_path"] = req.image_path;
            output["status"] = "ok";
            std::cout << output.dump(2) << std::endl;
        }
        catch (const std::exception& e) {
            json err;
            err["status"] = "error";
            err["message"] = e.what();
            std::cerr << err.dump() << std::endl;
            return 1;
        }
    }
    else {
        // 无参数模式：从 stdin 读取一条 JSON 请求
        std::string input;
        std::string line;
        while (std::getline(std::cin, line)) {
            input += line;
        }
        if (input.empty()) {
            json help;
            help["usage"] = "glass_engine <image_path>";
            help["modes"] = {"1. glass_engine image.png  (单图)",
                             "2. echo '{\"image_path\":\"...\"}' | glass_engine  (stdin JSON)",
                             "3. glass_engine --batch < input.jsonl  (批量)"};
            std::cout << help.dump(2) << std::endl;
            return 0;
        }
        try {
            auto req = parse_request(input);
            Detector detector(req.config);
            auto result = detector.detect(req.image_path);
            auto output = result_to_json(result);
            output["status"] = "ok";
            std::cout << output.dump() << std::endl;
        }
        catch (const std::exception& e) {
            json err;
            err["status"] = "error";
            err["message"] = e.what();
            std::cerr << err.dump() << std::endl;
            return 1;
        }
    }
    return 0;
}
