#!/usr/bin/env python3
"""Static INT8 quantization of the YOLOv8 ONNX model using real calibration images."""
import argparse
import glob
import os

import cv2
import numpy as np
from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_static
from onnxruntime.quantization.shape_inference import quant_pre_process

from detect import letterbox

TAIL_DEPTH = 4  # Concat_3 -> Mul_2 -> Concat_2 -> Div_1, per the YOLOv8 detect head


class ImageCalibrationReader(CalibrationDataReader):
    """Feeds preprocessed calibration images, matching detect.py's preprocessing exactly."""

    def __init__(self, image_dir, input_name, imgsz, limit):
        files = sorted(
            f for ext in ("jpg", "jpeg", "png")
            for f in glob.glob(os.path.join(image_dir, f"*.{ext}"))
        )
        if not files:
            raise SystemExit(f"no calibration images in {image_dir}")
        self.files = files[:limit]
        self.input_name = input_name
        self.imgsz = imgsz
        self.i = 0
        print(f"calibrating on {len(self.files)} images")

    def get_next(self):
        if self.i >= len(self.files):
            return None
        img = cv2.imread(self.files[self.i])
        self.i += 1
        if img is None:
            return self.get_next()
        if self.i % 25 == 0:
            print(f"  {self.i}/{len(self.files)}", flush=True)
        lb, _, _ = letterbox(img, self.imgsz)
        blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
        blob = np.ascontiguousarray(blob, dtype=np.float32)[None] / 255.0
        return {self.input_name: blob}

    def rewind(self):
        self.i = 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="best.onnx")
    p.add_argument("--out", default="best_int8.onnx")
    p.add_argument("--calib-dir", default="calib/images")
    p.add_argument("--limit", type=int, default=200)
    p.add_argument("--per-channel", action="store_true",
                   help="per-channel weights; needs opset>=13 (DequantizeLinear axis)")
    p.add_argument("--keep-head-fp32", action="store_true",
                   help="leave the classification head in FP32; its sigmoid outputs live in "
                        "a narrow 0-1 range that INT8 can crush to zero")
    p.add_argument("--activation", choices=("uint8", "int8"), default="int8",
                   help="int8 (signed) keeps zero centred, which matters here: the "
                        "pre-sigmoid class logits run -109..+3 and are 99.8%% negative, "
                        "so uint8 crushes the decisive near-zero band to nothing")
    args = p.parse_args()

    import onnxruntime as ort  # noqa: E402  (imported late; heavy)

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    imgsz = (int(inp.shape[3]), int(inp.shape[2]))

    prepped = args.model.replace(".onnx", "_prep.onnx")
    print("pre-processing (shape inference + optimization)...")
    quant_pre_process(args.model, prepped, skip_symbolic_shape=True)

    reader = ImageCalibrationReader(args.calib_dir, inp.name, imgsz, args.limit)

    import onnx
    graph = onnx.load(prepped).graph

    # The final Concat fuses box coords (0..imgsz) with class scores (0..1) into one
    # tensor. Quantizing it picks a single scale from the wide box range (~2.5 per
    # step at 640), which rounds every class score to exactly 0. Leave the tail in
    # FP32 -- a handful of elementwise ops, so the speed cost is negligible.
    #
    # Walk back from the graph output instead of hardcoding names: node names are an
    # exporter detail and change with imgsz/version, and a stale name list would
    # silently re-introduce the score collapse.
    producer = {o: n for n in graph.node for o in n.output}
    tail, cur, depth = [], graph.output[0].name, 0
    while cur in producer and depth < TAIL_DEPTH:
        node = producer[cur]
        tail.append(node.name)
        # the class-score branch (Sigmoid) enters the final Concat as a side input;
        # it must stay FP32 too, so pull in every immediate producer, not just [0]
        for extra in node.input[1:]:
            side = producer.get(extra)
            if side is not None and side.op_type in ("Sigmoid", "Concat", "Mul"):
                tail.append(side.name)
        cur = node.input[0]
        depth += 1
    nodes_to_exclude = list(dict.fromkeys(tail))

    if args.keep_head_fp32:
        nodes_to_exclude += [n.name for n in graph.node
                             if "/cv3." in n.name and n.name not in set(tail)]
    print(f"keeping {len(nodes_to_exclude)} output-tail nodes in FP32: {tail}")

    print("quantizing (this takes a few minutes)...")
    quantize_static(
        prepped,
        args.out,
        reader,
        nodes_to_exclude=nodes_to_exclude,
        quant_format=QuantFormat.QDQ,
        # per_channel requires opset>=13: it emits an `axis` attribute on
        # DequantizeLinear, which opset 12 rejects with INVALID_GRAPH at load time.
        per_channel=args.per_channel,
        weight_type=QuantType.QInt8,
        activation_type=QuantType.QInt8 if args.activation == "int8" else QuantType.QUInt8,
        # QDQOpTypePerChannelSupportToAxis is irrelevant here; what matters is that the
        # graph output itself is never wrapped in QuantizeLinear/DequantizeLinear.
        extra_options={"QDQKeepRemovableActivations": False,
                       "AddQDQPairToWeight": False},
    )
    os.remove(prepped)

    src_mb = os.path.getsize(args.model) / 1e6
    out_mb = os.path.getsize(args.out) / 1e6
    print(f"\n{args.model} {src_mb:.1f} MB -> {args.out} {out_mb:.1f} MB")

    # A model that loads but scores every box 0.0 is the failure mode that actually
    # bit us here, so verify on a real image and compare against the FP32 original.
    print("verifying the quantized model on a real image...")
    ref = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    new = ort.InferenceSession(args.out, providers=["CPUExecutionProvider"])

    sample = sorted(glob.glob(os.path.join(args.calib_dir, "*.jpg")))[0]
    img = cv2.imread(sample)
    lb, _, _ = letterbox(img, imgsz)
    blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
    blob = np.ascontiguousarray(blob, dtype=np.float32)[None] / 255.0

    r_cls = ref.run(None, {ref.get_inputs()[0].name: blob})[0][0, 4:].max()
    n_cls = new.run(None, {new.get_inputs()[0].name: blob})[0][0, 4:].max()
    print(f"  max class score: fp32={r_cls:.4f}  int8={n_cls:.4f}")
    if n_cls < 0.5 * r_cls:
        raise SystemExit(
            f"FAILED: class scores collapsed ({n_cls:.4f} vs {r_cls:.4f}). "
            "The quantized model is unusable; do not deploy it.")
    print("OK")


if __name__ == "__main__":
    main()
