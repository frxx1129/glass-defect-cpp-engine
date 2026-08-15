#!/usr/bin/env python3
"""批量对比：Python vs C++（支持多产线/多相机/暗场）。

用法:
  python batch_compare.py --roi <ROI模板> --dir <数据目录> [--config config2.yaml] [--cam 4] [--dark] [--exe <glass_engine.exe>]

参数:
  --roi      ROI 模板 JSON 路径（必填）
  --dir      数据目录（递归收集 cam-1_/cam1_/cam2_/... 图片）
  --config   配置文件（默认 config.yaml；line2 用 config2.yaml 等，决定 hough 参数/ppm/lineName）
  --cam      相机索引 _RUNTIME_CAM_INDEX（默认 0）
  --dark     使用 hough_inspector_dark_params + mode=dark
  --exe      C++ 引擎路径（默认 build_new\\glass_engine.exe）
"""
import sys, os, json, subprocess, cv2, yaml, glob, numpy as np, argparse, copy

if sys.stdout.encoding.lower() != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

PROJ = r'F:\玻璃检测\glass-defect-algo-main\glass-defect-algo-main'
CPP_EXE_DEFAULT = r"F:\玻璃检测\cpp_engine\build_new\glass_engine.exe"

sys.path.insert(0, PROJ)
import image_processor_hough as iph


def parse_args():
    ap = argparse.ArgumentParser(description='Python vs C++ 批量对比')
    ap.add_argument('--roi', required=True, help='ROI 模板 JSON 路径')
    ap.add_argument('--dir', required=True, help='数据目录（递归收集图片）')
    ap.add_argument('--config', default=PROJ + r'\config.yaml', help='配置文件')
    ap.add_argument('--cam', type=int, default=0, help='_RUNTIME_CAM_INDEX')
    ap.add_argument('--dark', action='store_true', help='暗场模式（hough_inspector_dark_params）')
    ap.add_argument('--exe', default=CPP_EXE_DEFAULT, help='C++ 引擎路径')
    ap.add_argument('--pat', default='cam-1_*.jpg,cam1_*.jpg,cam2_*.jpg,cam3_*.jpg,cam4_*.jpg,cam5_*.jpg',
                    help='图片名模式（逗号分隔 glob）')
    return ap.parse_args()


def main():
    args = parse_args()
    if not os.path.exists(args.exe):
        print(f'ERR: C++ exe 不存在: {args.exe}')
        sys.exit(1)

    cfg = yaml.safe_load(open(args.config, encoding='utf-8'))
    cfg['roi_template_file'] = args.roi
    hip = cfg['hough_inspector_dark_params' if args.dark else 'hough_inspector_params']
    hip['_RUNTIME_LINE_NAME'] = cfg.get('lineName', '')
    hip['_RUNTIME_CAM_INDEX'] = args.cam
    px = float(cfg['system_params']['pixels_per_mm'])
    mode = 'dark' if args.dark else 'light'

    pats = [p.strip() for p in args.pat.split(',') if p.strip()]
    IMAGES = sorted({f for p in pats for f in glob.glob(os.path.join(args.dir, '**', p), recursive=True)})
    if not IMAGES:
        print(f'ERR: {args.dir} 下没有匹配 {args.pat} 的图片')
        sys.exit(1)

    roi_j = json.load(open(args.roi, encoding='utf-8'))
    best = max(roi_j, key=lambda k: roi_j[k].get('source_image_count', 0))
    rois = [{'x': r['x'], 'y': r['y'], 'width': r['width'], 'height': r['height']}
            for r in roi_j[best]['averaged_rois'] if r['width'] > 0 and r['height'] > 0]

    def py_detect(img_path):
        img = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        # 每张图用配置深拷贝，避免 Python 侧 process_image_from_memory_parallel 对共享 cfg 的
        # 运行时字段污染（_RUNTIME_ROI_X/Y 等）导致跨图状态泄漏、结果不稳定
        cfg_fresh = copy.deepcopy(cfg)
        # 每张图重置 Python 全局缓存（理想竖直/栅栏/玻璃边界均为跨帧稳定器，ROI_ID 未设置时
        # key 相同会跨图串扰，导致 batch 结果与单张不一致）
        iph.reset_global_caches()
        report, _ = iph.process_image_from_memory_parallel(img, rois, cfg_fresh)
        return report.get('defects', [])

    def cpp_detect(img_path):
        req = {"image_path": img_path, "mode": mode, "config": {
            "roi_template_file": args.roi,
            "system_params": {"pixels_per_mm": px},
            "hough_inspector_params": hip,
        }}
        proc = subprocess.run([args.exe], input=json.dumps(req), capture_output=True, text=True, timeout=180)
        if proc.returncode != 0:
            return 'ERR:' + proc.stderr[:80]
        return json.loads(proc.stdout).get('defects', [])

    def brief(defects):
        if isinstance(defects, str): return defects
        return ','.join(f"{d['type']}@({d['location']['x']},{d['location']['y']})" for d in defects)

    ok = diff = err = 0
    diffs = []
    print(f"config={os.path.basename(args.config)} mode={mode} cam={args.cam} ppm={px} roi={os.path.basename(args.roi)}")
    print(f"图片 {len(IMAGES)} 张, exe={args.exe}")
    print(f"{'图片':<38} {'Python':<32} {'C++':<32} {'结果'}")
    print('=' * 130)
    for img in IMAGES:
        name = os.path.basename(img)
        pd = py_detect(img)
        cd = cpp_detect(img)
        if isinstance(cd, str):
            err += 1; res = 'ERR'
        else:
            if len(pd) == len(cd): ok += 1; res = 'OK'
            else:
                diff += 1; res = 'DIFF'
                diffs.append((name, pd, cd))
        print(f"{name:<38} {len(pd):<3}{brief(pd):<29} {str(len(cd)) if not isinstance(cd,str) else '?' :<3}{brief(cd):<29} {res}")

    print(f"\n共 {len(IMAGES)} 张: OK={ok} DIFF={diff} ERR={err}")
    if diffs:
        print("\nDIFF 明细:")
        for name, pd, cd in diffs:
            print(f"  {name}: Python={brief(pd)} | C++={brief(cd)}")


if __name__ == '__main__':
    main()
