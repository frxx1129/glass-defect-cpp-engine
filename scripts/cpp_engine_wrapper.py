#!/usr/bin/env python3
"""
C++ 引擎的 Python 调用包装器
使用方法：
    from cpp_engine_wrapper import CppEngine
    engine = CppEngine()
    result = engine.detect("image.png", config_dict)
"""
import subprocess
import json
import os
import tempfile

class CppEngine:
    def __init__(self, exe_path=None):
        if exe_path is None:
            # 默认路径：和脚本同目录的 ../build_new/glass_engine.exe（build.bat 产物）
            self.exe_path = os.path.join(
                os.path.dirname(__file__),
                "..", "build_new", "glass_engine.exe"
            )
        else:
            self.exe_path = exe_path

        if not os.path.exists(self.exe_path):
            raise FileNotFoundError(
                f"C++ engine not found at {self.exe_path}. "
                f"Build it first: 在项目根目录运行 build.bat（cl 直接编译，产物在 build_new\\glass_engine.exe）"
            )

    def detect(self, image_path, config=None, total_frames=0):
        """调用 C++ 引擎检测单张图片"""
        request = {
            "image_path": image_path,
            "config": config or {},
            "total_frames": total_frames
        }

        proc = subprocess.run(
            [self.exe_path],
            input=json.dumps(request),
            capture_output=True,
            text=True,
            timeout=30
        )

        if proc.returncode != 0:
            error_msg = proc.stderr.strip() or f"Exit code {proc.returncode}"
            raise RuntimeError(f"C++ engine failed: {error_msg}")

        return json.loads(proc.stdout)

    def detect_batch(self, requests):
        """批量检测，requests 是 list[dict]"""
        input_lines = "\n".join(json.dumps(r) for r in requests)

        proc = subprocess.run(
            [self.exe_path, "--batch"],
            input=input_lines,
            capture_output=True,
            text=True,
            timeout=60
        )

        if proc.returncode != 0:
            raise RuntimeError(f"C++ engine batch failed: {proc.stderr}")

        results = []
        for line in proc.stdout.strip().split("\n"):
            if line:
                results.append(json.loads(line))
        return results

    def __repr__(self):
        return f"CppEngine({self.exe_path})"


# 快速测试
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        engine = CppEngine()
        result = engine.detect(sys.argv[1])
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print("Usage: python cpp_engine_wrapper.py <image_path>")
