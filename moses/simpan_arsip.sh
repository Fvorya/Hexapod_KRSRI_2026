#!/usr/bin/env bash
# Simpan snapshot semua berkas buatan kita ke arsip/<tanggal>_<label>/.
#
# Jalankan SEBELUM mengubah apa pun, supaya versi yang sekarang jalan tidak
# hilang tertimpa -- itu persis yang terjadi pada r1..r8 dan kenapa berkas
# mentahnya sudah tidak ada.
#
#   ./simpan_arsip.sh r10
#   ./simpan_arsip.sh r10-capit-terukur
#
# Sesudah itu tulis apa yang berubah di arsip/RIWAYAT.md. Snapshot tanpa
# catatan hanya menunda kebingungan, tidak menghilangkannya.
set -euo pipefail

LABEL="${1:-}"
[ -n "$LABEL" ] || { echo "pakai: ./simpan_arsip.sh <label>   (mis. r10)"; exit 1; }

DIR="$(cd "$(dirname "$0")" && pwd)"
TUJUAN="$DIR/arsip/$(date +%Y-%m-%d)_$LABEL"

[ -e "$TUJUAN" ] && { echo "sudah ada: $TUJUAN -- pakai label lain"; exit 1; }
mkdir -p "$TUJUAN"

BERKAS=(mission_hud.py test_mission_hud.py cek_serial.py
        hemat_daya.sh install_service.sh run_hud.sh simpan_arsip.sh
        upload_to_pi.bat upload_to_pi.sh upload_teensy.bat flash_teensy.bat
        README_HUD.md)

n=0
for f in "${BERKAS[@]}"; do
  if [ -f "$DIR/$f" ]; then cp -p "$DIR/$f" "$TUJUAN/"; n=$((n+1)); fi
done

# Kalibrasi yang sedang berlaku ikut disimpan kalau ada -- tanpa itu, snapshot
# kodenya lengkap tapi angka yang membuatnya jalan hilang.
for f in "$DIR"/kalib_*.json; do
  [ -f "$f" ] && cp -p "$f" "$TUJUAN/" && n=$((n+1))
done

echo "[arsip] $n berkas -> $TUJUAN"
echo "[arsip] sekarang tulis perubahannya di arsip/RIWAYAT.md"
