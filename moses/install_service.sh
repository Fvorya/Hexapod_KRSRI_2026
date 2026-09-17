#!/usr/bin/env bash
# Pasang HUD sebagai layanan systemd -- nyala sendiri saat Raspi boot.
# Setelah ini: colok power, tunggu ~25 detik, buka http://terra-core:5000/.
# Tidak perlu SSH, tidak perlu menjalankan apa pun secara manual.
#
#   ./install_service.sh              # pasang & nyalakan
#   ./install_service.sh --hapus      # matikan & lepas
#
# Jalankan DI RASPI, dengan sudo tersedia.
set -euo pipefail

NAMA=r2c-hud
UNIT=/etc/systemd/system/$NAMA.service
DIR="$(cd "$(dirname "$0")" && pwd)"
USR="$(id -un)"
PORT="${PORT:-5000}"

# Port Teensy. Dikosongkan = deteksi otomatis lewat /dev/serial/by-id (USB).
# Untuk sambungan UART GPIO, sebutkan eksplisit -- node UART SELALU ada walau
# tidak ada apa pun di ujung kabelnya, jadi deteksi otomatis akan berbohong:
#     TEENSY=/dev/ttyAMA0 ./install_service.sh
TEENSY="${TEENSY:-}"
ARG_PORT=""
[ -n "$TEENSY" ] && ARG_PORT=" --port $TEENSY"

if [ "${1:-}" = "--hapus" ]; then
  sudo systemctl disable --now $NAMA.service 2>/dev/null || true
  sudo rm -f $UNIT
  sudo systemctl daemon-reload
  echo "[service] $NAMA dilepas."
  exit 0
fi

[ -x "$DIR/.venv/bin/python" ] || { echo "[service] .venv tidak ada di $DIR"; exit 1; }

# Akses perangkat: 'dialout' untuk /dev/ttyACM* (Teensy), 'video' untuk kamera.
# Tanpa keduanya, layanan hidup tapi tidak melihat robot maupun kamera -- dan
# gejalanya cuma "SIMULASI" di pojok HUD, bukan pesan error yang jelas.
for grup in dialout video; do
  if ! id -nG "$USR" | tr ' ' '\n' | grep -qx "$grup"; then
    echo "[service] menambahkan $USR ke grup $grup"
    sudo usermod -aG "$grup" "$USR"
    PERLU_LOGOUT=1
  fi
done

# Pencuri klasik /dev/ttyACM* (Raspi 5 bawaan Bookworm menyalakan keduanya):
#   ModemManager -- menyangka Teensy modem, kirim AT dan MEMAKAN balasannya.
#                   HUD mengirim (tx naik) tapi tidak pernah menerima (rx 0).
#   brltty       -- menyangka Teensy layar braille, merebut port sampai
#                   tidak bisa dipakai sama sekali.
# Di-mask, bukan cuma disable: paket bisa menyalakannya lagi lewat update
# atau udev rule tanpa mask.
for svc in ModemManager brltty brltty-udev; do
  if systemctl list-unit-files "$svc.service" >/dev/null 2>&1; then
    echo "[service] mematikan $svc (konflik port serial Teensy)"
    sudo systemctl disable --now "$svc" 2>/dev/null || true
    sudo systemctl mask "$svc" 2>/dev/null || true
  fi
done

sudo tee $UNIT >/dev/null <<UNITEOF
[Unit]
Description=R2C Hexapod Mission HUD (web di :$PORT)
# Tidak memakai network-online.target: HUD harus tetap menyala walau Tailscale
# belum naik. Halaman webnya baru bisa dibuka setelah jaringan ada, dan itu
# terjadi sendiri tanpa layanan ini perlu menunggu.
After=multi-user.target
# StartLimit* HARUS di [Unit], bukan [Service]. Kalau salah tempat, systemd
# hanya mencatat "Unknown key name ... ignoring" lalu diam-diam memakai batas
# bawaan -- layanan berhenti sendiri sesudah beberapa kali restart cepat.
StartLimitIntervalSec=0

[Service]
Type=simple
User=$USR
WorkingDirectory=$DIR
ExecStart=$DIR/.venv/bin/python -u $DIR/mission_hud.py --web-port $PORT$ARG_PORT
# Jaring terakhir saja. Kamera yang belum tercolok TIDAK lagi mematikan HUD --
# program mencobanya sendiri tiap 3 detik dan halamannya tetap bisa dibuka,
# lengkap dengan sebabnya. Teensy juga disambung ulang sendiri tiap 2 detik.
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
UNITEOF

sudo systemctl daemon-reload
sudo systemctl enable --now $NAMA.service
sleep 2
sudo systemctl --no-pager --lines=15 status $NAMA.service || true

echo
echo "[service] terpasang. Buka: http://$(hostname):$PORT/"
echo "[service] lihat log   : journalctl -u $NAMA -f"
echo "[service] hentikan    : sudo systemctl stop $NAMA"
echo "[service] jangan nyala saat boot: sudo systemctl disable $NAMA"
if [ "${PERLU_LOGOUT:-0}" = "1" ]; then
  echo
  echo "[service] PENTING: keanggotaan grup baru berlaku setelah reboot."
  echo "[service]          reboot sekali supaya Teensy & kamera terbaca."
fi
