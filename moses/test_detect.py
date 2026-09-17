#!/usr/bin/env python3
"""Self-check for letterbox + postprocess geometry. Run: python test_detect.py"""
import numpy as np
from detect import letterbox, postprocess


def test_letterbox_roundtrip():
    """A box drawn in the original frame must map back to itself after letterbox+postprocess."""
    frame = np.zeros((720, 1280, 3), np.uint8)
    lb, r, pad = letterbox(frame, (640, 640))
    assert lb.shape[:2] == (640, 640), lb.shape

    # ground-truth box in original coords
    gx1, gy1, gx2, gy2 = 200.0, 100.0, 500.0, 400.0
    # forward-transform into letterboxed space, as the model would see it
    lx1, ly1 = gx1 * r + pad[0], gy1 * r + pad[1]
    lx2, ly2 = gx2 * r + pad[0], gy2 * r + pad[1]
    cx, cy = (lx1 + lx2) / 2, (ly1 + ly2) / 2
    bw, bh = lx2 - lx1, ly2 - ly1

    # fake YOLOv8 output: (1, 4+nc, N) with one strong detection of class 1
    out = np.zeros((1, 6, 3), np.float32)
    out[0, :4, 0] = [cx, cy, bw, bh]
    out[0, 5, 0] = 0.9  # class 1 score

    dets = postprocess(out, r, pad, 0.35, 0.45, frame.shape[:2])
    assert len(dets) == 1, dets
    x1, y1, x2, y2, score, cls = dets[0]
    assert cls == 1 and abs(score - 0.9) < 1e-5
    for got, want in zip((x1, y1, x2, y2), (gx1, gy1, gx2, gy2)):
        assert abs(got - want) <= 1, f"{got} != {want}"


def test_conf_filter():
    out = np.zeros((1, 6, 2), np.float32)
    out[0, :4, 0] = [100, 100, 50, 50]
    out[0, 4, 0] = 0.10  # below threshold
    assert postprocess(out, 1.0, (0, 0), 0.35, 0.45, (640, 640)) == []


def test_class_aware_nms():
    """Two heavily-overlapping boxes of DIFFERENT classes must both survive."""
    out = np.zeros((1, 6, 2), np.float32)
    out[0, :4, 0] = [300, 300, 100, 100]
    out[0, 4, 0] = 0.9  # class 0
    out[0, :4, 1] = [302, 301, 100, 100]
    out[0, 5, 1] = 0.8  # class 1
    dets = postprocess(out, 1.0, (0, 0), 0.35, 0.45, (640, 640))
    assert len(dets) == 2, dets
    assert {d[5] for d in dets} == {0, 1}


def test_same_class_nms_suppresses():
    out = np.zeros((1, 6, 2), np.float32)
    out[0, :4, 0] = [300, 300, 100, 100]
    out[0, 4, 0] = 0.9
    out[0, :4, 1] = [302, 301, 100, 100]
    out[0, 4, 1] = 0.8  # same class, overlapping
    dets = postprocess(out, 1.0, (0, 0), 0.35, 0.45, (640, 640))
    assert len(dets) == 1, dets


def test_clipping():
    """Box running off-frame is clipped to frame bounds."""
    out = np.zeros((1, 6, 1), np.float32)
    out[0, :4, 0] = [10, 10, 100, 100]  # extends to -40,-40
    out[0, 4, 0] = 0.9
    dets = postprocess(out, 1.0, (0, 0), 0.35, 0.45, (480, 640))
    x1, y1, x2, y2, _, _ = dets[0]
    assert x1 == 0 and y1 == 0 and x2 <= 640 and y2 <= 480, dets


def test_camera_thread_returns_fresh_frames():
    """CameraThread must hand out newer frames, never repeat the same seq."""
    import time
    from detect import CameraThread

    class FakeCap:
        def __init__(self):
            self.n = 0

        def read(self):
            self.n += 1
            time.sleep(0.002)
            return True, np.full((4, 4, 3), self.n % 256, np.uint8)

        def release(self):
            pass

    cam = CameraThread(FakeCap())
    try:
        seq = 0
        seen = []
        for _ in range(5):
            frame, seq = cam.read(seq)
            assert frame is not None
            seen.append(seq)
        assert len(set(seen)) == len(seen), seen  # every read advanced
        assert seen == sorted(seen), seen
    finally:
        cam.release()


def test_camera_thread_stops_on_failure():
    """A camera that stops delivering must not hang read() forever."""
    from detect import CameraThread

    class DeadCap:
        def read(self):
            return False, None

        def release(self):
            pass

    cam = CameraThread(DeadCap())
    frame, _ = cam.read(0)
    assert frame is None
    cam.release()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok  {name}")
    print("all passed")
