#!/usr/bin/env python3
"""Compare ONNX models on speed AND detection agreement over the calibration set.

Speed: median inference latency over N images.
Accuracy: detections vs the ground-truth YOLO labels (IoU>=0.5, class must match).
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np

from detect import letterbox, load_session, postprocess


def load_labels(label_path, w, h):
    """YOLO txt -> [(x1,y1,x2,y2,cls)] in pixels.

    Handles both label formats this dataset mixes:
      - detection: `cls cx cy bw bh` (5 fields, normalized)
      - segmentation: `cls x1 y1 x2 y2 ...` polygon -> bbox via min/max
    """
    if not os.path.exists(label_path):
        return []
    out = []
    for line in open(label_path):
        parts = line.split()
        if len(parts) < 5:
            continue
        c, vals = int(parts[0]), list(map(float, parts[1:]))
        if len(vals) == 4:
            cx, cy, bw, bh = vals
            x1, y1, x2, y2 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
        elif len(vals) >= 6 and len(vals) % 2 == 0:
            xs, ys = vals[0::2], vals[1::2]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
        else:
            continue
        out.append((x1 * w, y1 * h, x2 * w, y2 * h, c))
    return out


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def evaluate(model, files, label_dir, conf, iou_thres, threads):
    sess, names, imgsz = load_session(model, threads)
    inp = sess.get_inputs()[0].name

    times, tp, fp, fn = [], 0, 0, 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        h, w = img.shape[:2]
        lb, r, pad = letterbox(img, imgsz)
        blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
        blob = np.ascontiguousarray(blob, dtype=np.float32)[None] / 255.0

        t0 = time.perf_counter()
        out = sess.run(None, {inp: blob})[0]
        times.append(time.perf_counter() - t0)

        dets = postprocess(out, r, pad, conf, iou_thres, (h, w))
        gts = load_labels(os.path.join(label_dir,
                                       os.path.splitext(os.path.basename(f))[0] + ".txt"), w, h)

        matched = set()
        for d in sorted(dets, key=lambda x: -x[4]):
            best, best_i = 0.0, -1
            for i, g in enumerate(gts):
                if i in matched or g[4] != d[5]:
                    continue
                v = iou(d[:4], g[:4])
                if v > best:
                    best, best_i = v, i
            if best >= 0.5:
                matched.add(best_i)
                tp += 1
            else:
                fp += 1
        fn += len(gts) - len(matched)

    ms = np.array(times) * 1e3
    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * prec * rec / (prec + rec) if prec + rec else 0.0
    return dict(median_ms=float(np.median(ms)), fps=1000.0 / float(np.median(ms)),
                tp=tp, fp=fp, fn=fn, precision=prec, recall=rec, f1=f1,
                size_mb=os.path.getsize(model) / 1e6)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("models", nargs="+")
    p.add_argument("--calib-dir", default="calib/images")
    p.add_argument("--label-dir", default="calib/labels")
    p.add_argument("--limit", type=int, default=60)
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--threads", type=int, default=4)
    args = p.parse_args()

    files = sorted(f for ext in ("jpg", "jpeg", "png")
                   for f in glob.glob(os.path.join(args.calib_dir, f"*.{ext}")))[:args.limit]
    print(f"evaluating on {len(files)} images, conf={args.conf}\n")

    rows = []
    for m in args.models:
        print(f"running {m} ...", flush=True)
        rows.append((m, evaluate(m, files, args.label_dir, args.conf, args.iou, args.threads)))

    print(f"\n{'model':<22}{'MB':>7}{'ms':>9}{'FPS':>8}{'prec':>8}{'recall':>8}{'F1':>8}"
          f"{'TP':>6}{'FP':>6}{'FN':>6}")
    print("-" * 89)
    for m, r in rows:
        print(f"{os.path.basename(m):<22}{r['size_mb']:>7.1f}{r['median_ms']:>9.1f}"
              f"{r['fps']:>8.1f}{r['precision']:>8.3f}{r['recall']:>8.3f}{r['f1']:>8.3f}"
              f"{r['tp']:>6}{r['fp']:>6}{r['fn']:>6}")

    if len(rows) == 2:
        base, new = rows[0][1], rows[1][1]
        print(f"\nspeedup: {base['median_ms'] / new['median_ms']:.2f}x   "
              f"F1 delta: {new['f1'] - base['f1']:+.3f}")


if __name__ == "__main__":
    main()
