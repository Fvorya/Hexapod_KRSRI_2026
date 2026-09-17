#!/usr/bin/env bash
# Export best.pt at several imgsz, convert to opset13, quantize each to INT8.
set -euo pipefail
cd "$(dirname "$0")"

for SZ in "$@"; do
  echo "=============== imgsz=$SZ ==============="
  .venv-export/bin/python - "$SZ" <<'PY'
import sys, shutil
from ultralytics import YOLO
sz = int(sys.argv[1])
m = YOLO("best.pt")
p = m.export(format="onnx", imgsz=sz, opset=13, simplify=True, dynamic=False)
shutil.move(p, f"best_{sz}.onnx")
print("exported", f"best_{sz}.onnx")
PY

  .venv/bin/python quantize.py \
      --model "best_${SZ}.onnx" \
      --out "best_${SZ}_int8.onnx" \
      --per-channel
done
echo "ALL BUILDS DONE"
