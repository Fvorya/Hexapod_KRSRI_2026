#!/usr/bin/env python3
"""YOLOv8 ONNX webcam detection - Raspberry Pi 5, onnxruntime + OpenCV preview."""
import argparse
import ast
import os
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort

COLORS = [(0, 220, 60), (40, 120, 255), (255, 180, 0), (200, 60, 255)]


def load_session(model_path, threads):
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = threads
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sess = ort.InferenceSession(model_path, opts, providers=["CPUExecutionProvider"])

    meta = sess.get_modelmeta().custom_metadata_map
    names = ast.literal_eval(meta["names"]) if "names" in meta else {}
    names = {int(k): v for k, v in names.items()}

    shape = sess.get_inputs()[0].shape  # (1, 3, H, W)
    imgsz = (int(shape[3]), int(shape[2]))  # (w, h)
    return sess, names, imgsz


def letterbox(img, size):
    """Resize keeping aspect ratio, pad to size. Returns image, scale, (padx, pady)."""
    h, w = img.shape[:2]
    tw, th = size
    r = min(tw / w, th / h)
    nw, nh = round(w * r), round(h * r)
    if (nw, nh) != (w, h):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padx, pady = (tw - nw) / 2, (th - nh) / 2
    top, bottom = round(pady - 0.1), round(pady + 0.1)
    left, right = round(padx - 0.1), round(padx + 0.1)
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return img, r, (left, top)


def postprocess(out, r, pad, conf_thres, iou_thres, orig_shape):
    """YOLOv8 raw output (1, 4+nc, N) -> list of (x1, y1, x2, y2, score, cls)."""
    pred = out[0]                      # (4+nc, N)
    boxes_xywh = pred[:4].T            # (N, 4)
    scores_all = pred[4:].T            # (N, nc)

    cls_ids = scores_all.argmax(1)
    scores = scores_all[np.arange(len(cls_ids)), cls_ids]
    keep = scores > conf_thres
    if not keep.any():
        return []
    boxes_xywh, scores, cls_ids = boxes_xywh[keep], scores[keep], cls_ids[keep]

    # xywh (letterboxed space) -> xyxy in original frame
    xy, wh = boxes_xywh[:, :2], boxes_xywh[:, 2:]
    x1y1 = (xy - wh / 2 - pad) / r
    x2y2 = (xy + wh / 2 - pad) / r
    boxes = np.concatenate([x1y1, x2y2], axis=1)

    h, w = orig_shape
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h)

    # class-aware NMS: offset boxes per class so classes never suppress each other
    offset = cls_ids[:, None] * (max(w, h) + 1)
    nms_boxes = (boxes + offset).astype(np.float32)
    wh_boxes = np.column_stack([nms_boxes[:, 0], nms_boxes[:, 1],
                                nms_boxes[:, 2] - nms_boxes[:, 0],
                                nms_boxes[:, 3] - nms_boxes[:, 1]])
    idx = cv2.dnn.NMSBoxes(wh_boxes.tolist(), scores.tolist(), conf_thres, iou_thres)
    if len(idx) == 0:
        return []
    idx = np.array(idx).flatten()
    return [(*boxes[i].astype(int), float(scores[i]), int(cls_ids[i])) for i in idx]


def draw(frame, dets, names, stats):
    for x1, y1, x2, y2, score, cls in dets:
        color = COLORS[cls % len(COLORS)]
        label = f"{names.get(cls, cls)} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    # HUD panel
    lines = [
        f"FPS  {stats['fps']:5.1f}  (cap {stats['cap_fps']:.0f})",
        f"cap  {stats['t_cap']:5.1f} ms",
        f"pre  {stats['t_pre']:5.1f} ms",
        f"inf  {stats['t_inf']:5.1f} ms",
        f"post {stats['t_post']:5.1f} ms",
        f"objs {len(dets)}",
    ]
    counts = {}
    for *_, cls in dets:
        counts[names.get(cls, cls)] = counts.get(names.get(cls, cls), 0) + 1
    for name, n in sorted(counts.items()):
        lines.append(f"  {name}: {n}")

    pw = 190
    ph = 18 * len(lines) + 10
    overlay = frame.copy()
    cv2.rectangle(overlay, (6, 6), (6 + pw, 6 + ph), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (14, 26 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 120), 1, cv2.LINE_AA)
    return frame


def open_camera(source, width, height, fps):
    """Open the webcam at its real maximum rate.

    Two ordering constraints, both found by measurement:

    1. CAP_PROP_FPS must be set *after* fourcc and frame size. Setting the format
       renegotiates the stream and resets the frame interval back to 30.
    2. exposure_dynamic_framerate must be cleared *after* VideoCapture opens --
       OpenCV resets it during negotiation. Left at its default of 1, the C922
       halves its rate for longer exposures and caps capture at ~15 fps no matter
       what resolution or FPS is requested.

    With both applied, MJPG 1280x720 reaches ~62 fps over plain USB 2.0.
    """
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {source}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    dev = f"/dev/video{source}" if isinstance(source, int) else str(source)
    subprocess.run(["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_dynamic_framerate=0"],
                   capture_output=True, check=False)
    return cap


class CameraThread:
    """Grabs frames in the background so inference never waits on cap.read().

    With a 32 ms capture and a 14 ms model, serial execution costs 46 ms/frame;
    overlapping them costs max(32, 14) = 32 ms. Always hands out the newest frame
    and drops stale ones -- for live detection latency matters more than seeing
    every frame.
    """

    def __init__(self, cap):
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                self.running = False
                break
            with self.lock:
                self.frame = frame
                self.seq += 1

    def read(self, last_seq):
        """Block until a frame newer than last_seq exists; return (frame, seq)."""
        while self.running:
            with self.lock:
                if self.seq != last_seq and self.frame is not None:
                    return self.frame, self.seq
            time.sleep(0.001)
        return None, last_seq

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="best_320_int8.onnx",
                   help="320 INT8 by default: ~50 fps at F1 0.990. Use best_416_int8.onnx "
                        "for F1 0.993 at ~34 fps, or best_int8.onnx (640) for 0.999 at ~14")
    p.add_argument("--source", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=60)
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--threads", type=int, default=0,
                   help="0 (default) = auto: all cores when capture runs inline, one "
                        "core fewer when it runs threaded so the grabber and the MJPG "
                        "decoder are not fighting inference for the same cores")
    p.add_argument("--headless", action="store_true", help="no window; print stats")
    p.add_argument("--capture", choices=("auto", "threaded", "serial"), default="auto",
                   help="auto (default) picks threaded only when inference is fast enough "
                        "to keep up with the camera; a background grabber that discards "
                        "most frames just steals CPU from a slow model")
    p.add_argument("--frames", type=int, default=0, help="stop after N frames (0=forever)")
    args = p.parse_args()

    ncores = os.cpu_count() or 4
    probe_threads = args.threads or ncores
    sess, names, imgsz = load_session(args.model, probe_threads)
    inp_name = sess.get_inputs()[0].name
    print(f"model={args.model} imgsz={imgsz} classes={names}")

    cap = open_camera(args.source, args.width, args.height, args.fps)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    afps = cap.get(cv2.CAP_PROP_FPS)
    print(f"camera: {aw}x{ah} @ {afps:.0f} fps")

    if args.capture == "auto":
        # Time a few real inferences, then decide. Threading only pays when the model
        # keeps up with the camera; below that the grabber thread discards most frames
        # and the CPU it burns comes straight out of inference.
        warm = np.zeros((1, 3, imgsz[1], imgsz[0]), np.float32)
        for _ in range(2):
            sess.run(None, {inp_name: warm})
        t0 = time.perf_counter()
        for _ in range(3):
            sess.run(None, {inp_name: warm})
        infer_ms = (time.perf_counter() - t0) / 3 * 1e3
        frame_ms = 1000.0 / max(afps, 1.0)
        use_thread = infer_ms < frame_ms * 1.5
        print(f"capture: {'threaded' if use_thread else 'serial'} "
              f"(inference {infer_ms:.0f} ms vs frame interval {frame_ms:.0f} ms)")
    else:
        use_thread = args.capture == "threaded"

    if args.threads == 0:
        # Threaded capture needs a core of its own; without it, inference and the
        # grabber/MJPG-decoder contend and the pipeline loses ~14% (52.0 vs 60.4 fps
        # at 320px). Running inline, nothing competes, so take every core.
        want = max(1, ncores - 1) if use_thread else ncores
        if want != probe_threads:
            sess, names, imgsz = load_session(args.model, want)
            inp_name = sess.get_inputs()[0].name
        print(f"threads: {want} of {ncores}")

    camera = CameraThread(cap) if use_thread else None
    seq = 0

    fps_hist = deque(maxlen=30)
    stats = dict(fps=0.0, cap_fps=afps, t_cap=0, t_pre=0, t_inf=0, t_post=0)
    n = 0
    try:
        while True:
            loop_start = time.perf_counter()

            t0 = time.perf_counter()
            if camera is not None:
                frame, seq = camera.read(seq)
                if frame is None:
                    print("frame grab failed")
                    break
            else:
                ok, frame = cap.read()
                if not ok:
                    print("frame grab failed")
                    break
            t_cap = time.perf_counter() - t0

            t0 = time.perf_counter()
            lb, r, pad = letterbox(frame, imgsz)
            blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
            blob = np.ascontiguousarray(blob, dtype=np.float32)[None] / 255.0
            t_pre = time.perf_counter() - t0

            t0 = time.perf_counter()
            out = sess.run(None, {inp_name: blob})[0]
            t_inf = time.perf_counter() - t0

            t0 = time.perf_counter()
            dets = postprocess(out, r, pad, args.conf, args.iou, frame.shape[:2])
            t_post = time.perf_counter() - t0

            fps_hist.append(1.0 / max(time.perf_counter() - loop_start, 1e-6))
            stats.update(fps=sum(fps_hist) / len(fps_hist), t_cap=t_cap * 1e3,
                         t_pre=t_pre * 1e3, t_inf=t_inf * 1e3, t_post=t_post * 1e3)

            if args.headless:
                if n % 10 == 0:
                    print(f"[{n:4d}] fps={stats['fps']:5.1f} inf={stats['t_inf']:5.1f}ms "
                          f"objs={len(dets)} "
                          f"{[(names.get(c, c), round(s, 2)) for *_, s, c in dets]}",
                          flush=True)
            else:
                draw(frame, dets, names, stats)
                cv2.imshow("YOLOv8 - korban/dummy", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            n += 1
            if args.frames and n >= args.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.release()
        else:
            cap.release()
        cv2.destroyAllWindows()
        print(f"\ndone: {n} frames, avg {stats['fps']:.1f} fps")


if __name__ == "__main__":
    main()
