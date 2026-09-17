#!/usr/bin/env bash
# ====================================================================
# siapkan_teensy_pi.sh -- pasang alat flash Teensy di Raspberry Pi.
# SEKALI SAJA. Sesudah ini cukup flash_teensy.py.
#
#   bash ~/M/siapkan_teensy_pi.sh
#
# Yang dipasang:
#   arduino-cli        compiler
#   core teensy:avr    dari indeks PJRC (bukan indeks Arduino biasa)
#   VL53L1X, Adafruit PWM Servo Driver
#   teensy_loader_cli  yang benar-benar menulis ke chip
#   aturan udev PJRC   supaya tidak perlu sudo tiap flash
#
# TIDAK memakai 'set -e'. Sengaja: kalau satu langkah gagal, yang dibutuhkan
# itu SISA laporannya -- apa yang sudah terpasang dan apa yang belum -- bukan
# skrip yang berhenti di tengah dan meninggalkan Pi setengah siap tanpa
# memberi tahu bagian mana yang mana.
# ====================================================================
set -u

BIN="$HOME/bin"
mkdir -p "$BIN"
gagal=0

lapor() { printf '\n== %s\n' "$*"; }
adu()   { command -v "$1" >/dev/null 2>&1; }

lapor "1/6  arduino-cli"
if adu arduino-cli; then
    echo "   sudah ada: $(arduino-cli version 2>/dev/null | head -1)"
else
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        | BINDIR="$BIN" sh || gagal=1
    export PATH="$BIN:$PATH"
    grep -q 'HOME/bin' "$HOME/.bashrc" 2>/dev/null \
        || echo 'export PATH="$HOME/bin:$PATH"' >> "$HOME/.bashrc"
fi
export PATH="$BIN:$PATH"

lapor "2/6  indeks board Teensy (PJRC)"
# Teensy TIDAK ada di indeks Arduino bawaan -- harus indeks PJRC sendiri.
arduino-cli config init --overwrite >/dev/null 2>&1
arduino-cli config add board_manager.additional_urls \
    https://www.pjrc.com/teensy/package_teensy_index.json || gagal=1
arduino-cli core update-index || gagal=1

lapor "3/6  core teensy:avr  (besar, sabar)"
if arduino-cli core list 2>/dev/null | grep -qi '^teensy:avr'; then
    echo "   sudah ada"
else
    arduino-cli core install teensy:avr || gagal=1
fi

lapor "4/6  pustaka pihak ketiga"
# Daftar ini titik berangkat, bukan kebenaran tetap. Yang menentukan tetap
# flash_teensy.py: ia memindai #include versi yang SEDANG dipilih dan
# menyebut mana yang kurang. Kalau Vincent menambah pustaka baru, di situ
# ketahuannya -- bukan di sini.
for p in "VL53L1X" "Adafruit PWM Servo Driver Library" "Adafruit GFX Library" "Adafruit SSD1306"; do
    if arduino-cli lib list 2>/dev/null | grep -qi "$p"; then
        echo "   sudah ada: $p"
    else
        arduino-cli lib install "$p" || gagal=1
    fi
done

lapor "5/6  teensy_loader_cli"
if adu teensy_loader_cli; then
    echo "   sudah ada: $(command -v teensy_loader_cli)"
else
    sudo apt-get install -y libusb-dev build-essential git >/dev/null 2>&1
    t=$(mktemp -d)
    if git clone --depth 1 https://github.com/PaulStoffregen/teensy_loader_cli \
            "$t/tlc" >/dev/null 2>&1 \
       && make -C "$t/tlc" >/dev/null 2>&1; then
        cp "$t/tlc/teensy_loader_cli" "$BIN/" && echo "   terpasang di $BIN"
    else
        echo "   !! gagal membangun teensy_loader_cli"
        gagal=1
    fi
    rm -rf "$t"
fi

lapor "6/6  aturan udev PJRC"
# Tanpa ini, Teensy cuma bisa di-flash sebagai root -- dan menjalankan
# seluruh flash_teensy.py sebagai root akan membuat berkas di ~/teensy jadi
# milik root, lalu scp berikutnya dari laptop ditolak. Jadi aturan udev ini
# bukan kenyamanan, ia yang menjaga alurnya tetap bisa diulang.
if [ -f /etc/udev/rules.d/00-teensy.rules ]; then
    echo "   sudah ada"
else
    if sudo curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules \
            -o /etc/udev/rules.d/00-teensy.rules; then
        sudo udevadm control --reload-rules && sudo udevadm trigger
        echo "   terpasang. CABUT DAN COLOK ULANG kabel Teensy sekali."
    else
        echo "   !! gagal mengunduh aturan udev"
        gagal=1
    fi
fi

lapor "Uji lewat ssh NON-interaktif"
# Ini yang sebenarnya dipakai: ssh pi "perintah". Shell-nya TIDAK membaca
# ~/.bashrc -- .bashrc bawaan Raspberry Pi OS keluar di baris pertamanya
# untuk shell non-interaktif -- jadi baris export PATH di atas tidak berlaku
# di sana. Diuji DI SINI supaya ketahuan sekarang, bukan nanti saat flash.
if ssh -o BatchMode=yes localhost true >/dev/null 2>&1; then
    if ssh localhost 'command -v arduino-cli' >/dev/null 2>&1; then
        echo "   [v] arduino-cli terlihat dari ssh non-interaktif"
    else
        echo "   [i] arduino-cli TIDAK terlihat dari ssh non-interaktif."
        echo "       Itu normal untuk pemasangan di ~/bin, dan flash_teensy.py"
        echo "       sudah menanganinya sendiri (ia menambahkan ~/bin ke PATH"
        echo "       proses). Tidak perlu diperbaiki."
    fi
else
    echo "   (dilewati -- ssh ke localhost butuh kata sandi)"
fi

lapor "Ringkasan"
for c in arduino-cli teensy_loader_cli; do
    if adu "$c"; then echo "   [v] $c"; else echo "   [!] $c BELUM ADA"; fi
done
arduino-cli core list 2>/dev/null | grep -i teensy | sed 's/^/   core: /'
arduino-cli lib list 2>/dev/null | grep -iE 'vl53|adafruit' | sed 's/^/   lib : /'

echo
if [ "$gagal" -eq 0 ]; then
    echo "Siap. Uji tanpa menyentuh Teensy:"
    echo "   python3 ~/M/flash_teensy.py --coba --terbaru"
else
    echo "Ada langkah yang gagal di atas. Perbaiki yang bertanda [!] dulu;"
    echo "yang lain sudah terpasang dan tidak perlu diulang."
fi
