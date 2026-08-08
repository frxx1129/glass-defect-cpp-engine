import sys
if sys.stdout.encoding.lower() != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
#!/usr/bin/env python3
"""Python 引擎 vs C++ 引擎对比验证工具
用法: python scripts/compare_engine.py [图片目录]
"""
import sys, os, json, subprocess, cv2, yaml

CPP_EXE = r"F:\玻璃检测\cpp_engine\build_new\glass_engine.exe"
PROJ = r"F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main"
ROI_FILE = r"F:\glass_build\roi_averaged_by_group_CORRECTED.json"
IMAGES = [
    r"F:\glass_build\cam-1_ts1765469138953.jpg",
    r"F:\glass_build\cam-1_ts1765470764168.jpg",
    r"F:\glass_build\cam-1_ts1765472073013.jpg",
]

def load_cfg():
    cfg = yaml.safe_load(open(PROJ + r"\config.yaml", encoding="utf-8"))
    cfg["roi_template_file"] = ROI_FILE
    return cfg

def load_rois():
    roi_j = json.load(open(ROI_FILE, encoding="utf-8"))
    best = max(roi_j, key=lambda k: roi_j[k].get("source_image_count", 0))
    return [{"x": r["x"], "y": r["y"], "width": r["width"], "height": r["height"]}
            for r in roi_j[best]["averaged_rois"] if r["width"] > 0 and r["height"] > 0]

def py_detect(cfg, img_path):
    sys.path.insert(0, PROJ)
    import image_processor_hough as iph
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    rois = load_rois()
    report, _ = iph.process_image_from_memory_parallel(img, rois, cfg)
    return report.get("defects", [])

def cpp_detect(cfg, img_path):
    req = {
        "image_path": img_path,
        "config": {
            "roi_template_file": ROI_FILE,
            "system_params": {"pixels_per_mm": cfg["system_params"]["pixels_per_mm"]},
            "hough_inspector_params": cfg["hough_inspector_params"],
        },
    }
    proc = subprocess.run([CPP_EXE], input=json.dumps(req), capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr)
    return json.loads(proc.stdout).get("defects", [])

def brief(defects):
    return [f"{d['type']}@({d['location']['x']},{d['location']['y']})" for d in defects]

def main():
    cfg = load_cfg()
    print(f"{'图片':<40} {'Python':<20} {'C++':<20} {'一致'}")
    print("=" * 100)
    for img in IMAGES:
        name = os.path.basename(img)
        pd = py_detect(cfg, img)
        cd = cpp_detect(cfg, img)
        same = "OK" if len(pd) == len(cd) else "DIFF"
        print(f"{name:<40} {len(pd):<3}{str(brief(pd)):<17} {len(cd):<3}{str(brief(cd)):<17} {same}")

if __name__ == "__main__":
    main()
