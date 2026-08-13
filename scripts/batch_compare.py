#!/usr/bin/env python3
"""批量对比：Python vs C++（用 cam1 ROI 模板）"""
import sys, os, json, subprocess, cv2, yaml, glob, numpy as np

sys.path.insert(0, r'F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main')
import image_processor_hough as iph

CPP_EXE = r"F:\玻璃检测\cpp_engine\build_new\glass_engine.exe"
PROJ = r"F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main"
ROI_FILE = r"F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main\cam1_roi_averaged_by_group.json"

IMAGES = sorted(glob.glob(r'F:\玻璃检测\资料\inputs\inputs\bugs\cam-1_*.jpg') +
                glob.glob(r'F:\玻璃检测\资料\inputs\inputs\bugs\cam1_*.jpg') +
                glob.glob(r'F:\玻璃检测\资料\inputs\inputs\cam1\cam-1_*.jpg'))

cfg = yaml.safe_load(open(PROJ + r'\config.yaml', encoding='utf-8'))
cfg['roi_template_file'] = ROI_FILE
hip = cfg['hough_inspector_params']
hip['_RUNTIME_LINE_NAME'] = cfg.get('lineName', 'Line3')
hip['_RUNTIME_CAM_INDEX'] = 0
px = float(cfg['system_params']['pixels_per_mm'])

roi_j = json.load(open(ROI_FILE, encoding='utf-8'))
best = max(roi_j, key=lambda k: roi_j[k].get('source_image_count', 0))
rois = [{'x': r['x'], 'y': r['y'], 'width': r['width'], 'height': r['height']}
        for r in roi_j[best]['averaged_rois'] if r['width'] > 0 and r['height'] > 0]

def py_detect(img_path):
    img = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    report, _ = iph.process_image_from_memory_parallel(img, rois, cfg)
    return report.get('defects', [])

def cpp_detect(img_path):
    req = {"image_path": img_path, "config": {
        "roi_template_file": ROI_FILE,
        "system_params": {"pixels_per_mm": px},
        "hough_inspector_params": hip,
    }}
    proc = subprocess.run([CPP_EXE], input=json.dumps(req), capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        return 'ERR:' + proc.stderr[:80]
    return json.loads(proc.stdout).get('defects', [])

def brief(defects):
    if isinstance(defects, str): return defects
    return ','.join(f"{d['type']}@({d['location']['x']},{d['location']['y']})" for d in defects)

ok = diff = err = 0
print(f"{'图片':<35} {'Python':<30} {'C++':<30} {'结果'}")
print('=' * 120)
for img in IMAGES:
    name = os.path.basename(img)
    pd = py_detect(img)
    cd = cpp_detect(img)
    if isinstance(cd, str):
        err += 1
        res = 'ERR'
    else:
        if len(pd) == len(cd): ok += 1; res = 'OK'
        else: diff += 1; res = 'DIFF'
    print(f"{name:<35} {len(pd):<3}{brief(pd):<27} {str(len(cd)) if not isinstance(cd,str) else '?' :<3}{brief(cd):<27} {res}")

print(f"\n共 {len(IMAGES)} 张: OK={ok} DIFF={diff} ERR={err}")
