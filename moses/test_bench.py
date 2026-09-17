#!/usr/bin/env python3
"""Self-check for bench.py's label parsing and IoU. Run: python test_bench.py"""
import os
import tempfile

from bench import iou, load_labels


def test_detection_format():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "a.txt")
    open(p, "w").write("1 0.5 0.5 0.25 0.5\n")
    assert load_labels(p, 640, 480) == [(240.0, 120.0, 400.0, 360.0, 1)]


def test_polygon_format():
    """Segmentation polygons must collapse to their bounding box."""
    d = tempfile.mkdtemp()
    p = os.path.join(d, "b.txt")
    open(p, "w").write("0 0.2 0.3 0.6 0.3 0.6 0.7 0.2 0.7\n")
    assert load_labels(p, 100, 100) == [(20.0, 30.0, 60.0, 70.0, 0)]


def test_real_dataset_line():
    """A real polygon line from calib/labels must parse to a sane in-frame box."""
    d = tempfile.mkdtemp()
    p = os.path.join(d, "c.txt")
    coords = [0.58984375, 0.5651041, 0.59765625, 0.4661458, 0.58203125, 0.4505208,
              0.5859375, 0.3932291, 0.572265625, 0.3802083, 0.544921875, 0.3802083,
              0.529296875, 0.40625, 0.50390625, 0.4192708, 0.5078125, 0.4713541,
              0.51953125, 0.5651041, 0.4921875, 0.6588541, 0.5390625, 0.6744791]
    open(p, "w").write("1 " + " ".join(map(str, coords)) + "\n")
    (x1, y1, x2, y2, c), = load_labels(p, 1000, 1000)
    assert c == 1
    assert 0 < x1 < x2 <= 1000 and 0 < y1 < y2 <= 1000
    assert abs(x1 - 492.1875) < 0.1 and abs(x2 - 597.65625) < 0.1, (x1, x2)


def test_malformed_lines_ignored():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "e.txt")
    open(p, "w").write("\n0 0.1 0.2\n1 0.5 0.5 0.2 0.2\n")
    assert len(load_labels(p, 100, 100)) == 1


def test_missing_file():
    assert load_labels("/nonexistent/x.txt", 10, 10) == []


def test_iou():
    assert abs(iou((0, 0, 10, 10), (0, 0, 10, 10)) - 1.0) < 1e-9
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert abs(iou((0, 0, 10, 10), (5, 0, 15, 10)) - 50 / 150) < 1e-9


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok  {name}")
    print("all passed")
