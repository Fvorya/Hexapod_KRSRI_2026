#!/usr/bin/env bash
# Jalankan deteksi dengan window preview.
#   Dari terminal VNC/desktop : ./run.sh
#   Dari SSH (window muncul di layar VNC) : ./run.sh
# Argumen tambahan diteruskan ke detect.py, mis: ./run.sh --model best_int8.onnx
cd "$(dirname "$0")"

# Kalau dijalankan dari SSH, sambungkan ke sesi desktop yang sedang berjalan supaya
# window-nya muncul di VNC dan bukan gagal "cannot open display".
if [ -z "$WAYLAND_DISPLAY" ] && [ -z "$DISPLAY" ]; then
  export XDG_RUNTIME_DIR="/run/user/$(id -u)"
  if [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
    export WAYLAND_DISPLAY=wayland-0
  fi
  export DISPLAY="${DISPLAY:-:0}"
fi

exec .venv/bin/python detect.py "$@"
