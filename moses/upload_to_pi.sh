#!/usr/bin/env bash
# Unggah HUD misi ke Raspberry Pi 5 lewat Tailscale/SSH.
#
#   ./upload_to_pi.sh              # kirim berkas HUD
#   ./upload_to_pi.sh --all        # + detect.py (kalau ikut diubah)
#   PI=bima@raspi5 ./upload_to_pi.sh
#
# Dari Windows: jalankan lewat Git Bash atau WSL. Perintah setara untuk
# PowerShell polos dicetak di akhir.
set -euo pipefail

PI="${PI:-bima@terra-core}"
DIR="${DIR:-M}"

BERKAS=(mission_hud.py run_hud.sh install_service.sh test_mission_hud.py)
[ "${1:-}" = "--all" ] && BERKAS+=(detect.py)

cd "$(dirname "$0")"

echo "[upload] -> $PI:~/$DIR/"
ADA=()
for f in "${BERKAS[@]}"; do
  if [ -f "$f" ]; then echo "[upload]   $f"; ADA+=("$f")
  else echo "[upload]   LEWAT (tidak ada): $f"; fi
done

scp "${ADA[@]}" "$PI:~/$DIR/"

# chmod + jalankan uji logikanya di Pi. Kalau 44 uji lulus di sana, berarti
# pustaka dan versi Python-nya cocok, bukan cuma di laptop.
ssh "$PI" "cd ~/$DIR && chmod +x run_hud.sh install_service.sh && \
           .venv/bin/python test_mission_hud.py | tail -3"

cat <<EOF

[upload] selesai.

Sekali saja, supaya nyala sendiri tiap Raspi boot:
    ssh $PI
    cd ~/$DIR && ./install_service.sh          # lalu reboot sekali

Sesudah itu cukup nyalakan robot dan buka:
    http://${PI#*@}:5000/

Menjalankan manual (tanpa layanan):
    ssh $PI -t "cd ~/$DIR && ./run_hud.sh"

CATATAN PowerShell (tanpa Git Bash):
    scp mission_hud.py run_hud.sh install_service.sh test_mission_hud.py ${PI}:~/${DIR}/
EOF
