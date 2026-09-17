#!/usr/bin/env bash
# ===================================================================
#  hemat_daya.sh -- turunkan konsumsi daya Raspberry Pi 5 untuk robot.
#
#    ./hemat_daya.sh --status    lihat keadaan sekarang, tidak mengubah apa pun
#    ./hemat_daya.sh             terapkan langkah 1-4 (minta konfirmasi)
#    ./hemat_daya.sh --balik     kembalikan seperti semula
#
#  Yang dilakukan:
#    1. matikan overclock          (arm_freq / over_voltage dinonaktifkan)
#    2. batasi frekuensi puncak    (arm_freq_max=1800)
#    3. inferensi 2 thread + kamera 640x480@30  (layanan r2c-hud)
#    4. boot ke konsol, bukan desktop
#
#  CATATAN JUJUR: ini menggeser ambang brownout, TIDAK menghapusnya.
#  Obat sebenarnya tetap kapasitor bulk 2200-4700 uF di terminal 5 V Pi.
# ===================================================================
set -euo pipefail

CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
UNIT=/etc/systemd/system/r2c-hud.service
CADANGAN="$CFG.sebelum-hemat"
TANDA="# --- hemat_daya.sh ---"
EKSTRA="--threads 2 --width 640 --height 480 --fps 30"

status() {
  echo
  echo "=== KEADAAN SEKARANG ==="
  echo "config : $CFG"
  echo
  echo "-- baris frekuensi/tegangan yang aktif:"
  grep -nE '^\s*(arm_freq|over_voltage|gpu_freq|force_turbo)' "$CFG" || echo "   (tidak ada -- sudah default)"
  echo
  echo "-- target boot :"; systemctl get-default
  echo "-- suhu        :"; vcgencmd measure_temp 2>/dev/null || echo "   (vcgencmd tidak ada)"
  echo "-- throttled   :"; vcgencmd get_throttled 2>/dev/null || true
  echo "     0x0 = sehat | bit 0 (0x1) = under-voltage SEKARANG"
  echo "     0x10000 = pernah under-voltage (melekat sampai reboot)"
  echo "-- frekuensi   :"; vcgencmd measure_clock arm 2>/dev/null || true
  echo
  echo "-- ExecStart layanan HUD:"
  grep -m1 ExecStart "$UNIT" 2>/dev/null || echo "   (layanan belum terpasang)"
  echo
}

if [ "${1:-}" = "--status" ]; then status; exit 0; fi

if [ "${1:-}" = "--balik" ]; then
  echo "[balik] mengembalikan pengaturan..."
  if [ -f "$CADANGAN" ]; then
    sudo cp "$CADANGAN" "$CFG"; echo "  config.txt dipulihkan dari $CADANGAN"
  else
    # Tanpa cadangan, yang bisa dilakukan cuma dua: buang blok yang kita
    # tambahkan, dan hidupkan lagi baris yang kita komentari. Keduanya
    # memakai penanda sendiri, jadi baris overclock milikmu yang lain tidak
    # ikut tersentuh.
    echo "  (tidak ada cadangan; membuang blok hemat_daya & membuka komentar)"
    sudo sed -i "/$TANDA/,/# --- akhir hemat_daya ---/d" "$CFG"
    sudo sed -i -E 's/^(\s*)#hemat# /\1/' "$CFG"
  fi
  sudo systemctl set-default graphical.target
  if [ -f "$UNIT" ]; then
    sudo sed -i "s| $EKSTRA||" "$UNIT"
    sudo systemctl daemon-reload && sudo systemctl restart r2c-hud
  fi
  echo
  echo "[balik] selesai. REBOOT supaya config.txt berlaku:  sudo reboot"
  echo
  echo "  PERINGATAN: overclock kembali menyala, jadi ambang brownout kembali"
  echo "  ke posisi semula. Sesudah reboot, pantau baris 'daya' di strip HUD"
  echo "  selama satu menit di mode JEJAK. Kalau merah atau Pi mati:"
  echo "    - knob tercepat: tab Kalibrasi -> jejak_hz -> isi 8 (tanpa restart)"
  echo "    - obat sebenarnya tetap kapasitor bulk 2200-4700 uF di 5 V Pi"
  echo
  status
  exit 0
fi

status
cat <<INFO
=== YANG AKAN DIUBAH ===
 1. arm_freq / over_voltage / force_turbo  -> dinonaktifkan (overclock mati)
 2. arm_freq_max=1800                      -> puncak CPU dibatasi 1,8 GHz
 3. layanan HUD  + $EKSTRA
 4. boot           -> multi-user.target (konsol, tanpa desktop/VNC)

Cadangan config.txt disimpan ke: $CADANGAN
Bisa dibatalkan kapan saja dengan: ./hemat_daya.sh --balik
INFO
read -r -p "Lanjutkan? [y/N] " jwb
[ "${jwb,,}" = "y" ] || { echo "dibatalkan."; exit 0; }

# --- 1 & 2 : config.txt -------------------------------------------------
[ -f "$CADANGAN" ] || sudo cp "$CFG" "$CADANGAN"
# Nonaktifkan baris overclock yang ada, jangan dihapus -- supaya kelihatan
# apa yang dulu dipakai kalau nanti mau dikembalikan manual.
sudo sed -i -E 's/^(\s*)(arm_freq=|over_voltage|gpu_freq=|force_turbo)/\1#hemat# \2/' "$CFG"
sudo sed -i "/$TANDA/,/# --- akhir hemat_daya ---/d" "$CFG"
sudo tee -a "$CFG" >/dev/null <<EOF
$TANDA
# Puncak CPU dibatasi: arus puncak turun jauh lebih cepat daripada
# kecepatannya, karena daya naik kira-kira kuadrat terhadap tegangan inti.
# Robot BERDIRI DIAM saat menilai boneka, jadi inferensi yang lebih lama
# tidak ada yang menunggu.
arm_freq_max=1800
# --- akhir hemat_daya ---
EOF
echo "[1,2] config.txt diperbarui."

# --- 3 : layanan HUD ----------------------------------------------------
if [ -f "$UNIT" ]; then
  if grep -q -- "$EKSTRA" "$UNIT"; then
    echo "[3] layanan sudah memakai opsi hemat."
  else
    sudo sed -i "s|\(ExecStart=.*mission_hud.py.*\)|\1 $EKSTRA|" "$UNIT"
    sudo systemctl daemon-reload
    sudo systemctl restart r2c-hud
    echo "[3] layanan HUD: $EKSTRA"
  fi
else
  echo "[3] LEWAT -- layanan r2c-hud belum terpasang (./install_service.sh)"
fi

# --- 4 : boot ke konsol -------------------------------------------------
sudo systemctl set-default multi-user.target
echo "[4] boot berikutnya ke konsol (desktop & VNC tidak dijalankan)."

cat <<AKHIR

=== SELESAI ===
Langkah 1, 2 dan 4 baru berlaku SESUDAH REBOOT:

    sudo reboot

Sesudah reboot, buka HUD lalu tekan "Mulai JEJAK" dan perhatikan baris
"daya" di strip status selama satu menit:

  hijau  (mis. 58C)                 -> aman
  merah  under-voltage SEKARANG     -> masih kurang; kapasitor bulk
                                       2200-4700 uF di terminal 5 V Pi

Periksa kapan saja:  ./hemat_daya.sh --status
Kembalikan semula :  ./hemat_daya.sh --balik
AKHIR
