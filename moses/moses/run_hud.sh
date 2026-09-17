#!/usr/bin/env bash
# Jalankan HUD misi. Antarmukanya WEB -- tidak butuh VNC, tidak butuh window.
#
#   ./run_hud.sh                          # buka http://terra-core:5000/
#   ./run_hud.sh --web-port 5001          # kalau :5000 masih dipegang app lama
#   ./run_hud.sh --port /dev/ttyACM0      # paksa port Teensy (default: deteksi otomatis)
#   ./run_hud.sh --model best_320_int8.onnx
#   ./run_hud.sh --window                 # tambah window lokal di layar VNC/HDMI
#
# Argumen tambahan diteruskan apa adanya ke mission_hud.py.
set -euo pipefail
cd "$(dirname "$0")"

# Hanya dibutuhkan kalau dipanggil dengan --window. Untuk antarmuka web, blok
# ini tidak berpengaruh apa-apa -- dibiarkan supaya --window tetap bekerja dari SSH.
if [ -z "${WAYLAND_DISPLAY:-}" ] && [ -z "${DISPLAY:-}" ]; then
  export XDG_RUNTIME_DIR="/run/user/$(id -u)"
  [ -S "$XDG_RUNTIME_DIR/wayland-0" ] && export WAYLAND_DISPLAY=wayland-0
  export DISPLAY="${DISPLAY:-:0}"
fi

# pyserial hanya dibutuhkan untuk bicara ke Teensy. Tanpa itu HUD tetap jalan
# dalam mode SIMULASI -- tetap berguna untuk menguji tampilan & state machine.
if ! .venv/bin/python -c "import serial" 2>/dev/null; then
  echo "[run_hud] pyserial belum ada di .venv -- mode SIMULASI."
  echo "[run_hud] pasang: .venv/bin/pip install pyserial"
fi

# Peringatkan lebih awal kalau port webnya sudah dipakai, supaya pesannya jelas
# bukan tumpukan traceback.
PORT=5000
for ((i = 1; i <= $#; i++)); do
  if [ "${!i}" = "--web-port" ]; then j=$((i + 1)); PORT="${!j}"; fi
done
if command -v ss >/dev/null && ss -ltn "sport = :$PORT" | grep -q LISTEN; then
  echo "[run_hud] PERINGATAN: port $PORT sudah dipakai proses lain."
  echo "[run_hud]   lihat siapa : sudo ss -ltnp \"sport = :$PORT\""
  echo "[run_hud]   atau pakai  : ./run_hud.sh --web-port 5001"
fi

echo "[run_hud] buka: http://$(hostname):$PORT/"
exec .venv/bin/python -u mission_hud.py "$@"
