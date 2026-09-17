#!/usr/bin/env python3
"""
sidik_firmware.py -- ambil "sidik jari" firmware yang SEKARANG ada di Teensy.
Jalankan DI RASPI.

    cd ~/M && sudo systemctl stop r2c-hud
    cd ~/M && .venv/bin/python sidik_firmware.py
    # lalu dari laptop:  scp bima@terra-core:~/M/sidik_firmware_*.txt .

YANG PERLU KAMU TAHU LEBIH DULU
-------------------------------
Ini TIDAK mengambil kode sumber. Kode sumber tidak bisa diambil dari Teensy:

  1. Bootloader Teensy 4.x (chip MKL02) hanya bisa MENULIS dan me-reboot.
     Tidak ada perintah baca-balik. PJRC memang mendesainnya begitu.
  2. Andai bisa pun, isinya kode mesin ARM hasil kompilasi -- bukan .cpp.
     Nama variabel, komentar, struktur file, semuanya sudah hilang saat
     dikompilasi. Disassembly tidak akan mengembalikan Mission.cpp.

Yang BISA diambil: apa yang firmware itu CETAK tentang dirinya sendiri.
Vincent menulis banyak perintah introspeksi, dan gabungannya cukup untuk
menjawab "versi mana yang ada di chip ini, dan fitur apa saja yang ada".

Kalau di dump nanti muncul state seperti MISI_KE_KORBAN3 atau perintah m
yang belum kita kenal, berarti chip memang memegang firmware yang lebih baru
daripada yang ada di folder vincent -- dan itu bukti yang bisa kamu bawa ke
Vincent untuk minta sumbernya.

SEMUA perintah di bawah HANYA MENCETAK. Tidak satu pun menggerakkan robot,
menulis EEPROM, atau mengubah setelan.
"""
import glob
import os
import subprocess
import sys
import time

# (perintah, keterangan, detik menunggu jawaban)
# Urutan sengaja: 'h' duluan, karena daftar perintahnya sendiri yang paling
# banyak memberi tahu fitur apa yang ada.
PERINTAH = [
    ("h",  "daftar SELURUH perintah -- petunjuk fitur terkuat", 2.5),
    ("m",  "status misi: nama state, ambang, titik nol",        2.0),
    ("v",  "status navigasi + jarak sekitar",                   2.0),
    ("q",  "SEMUA parameter kalibrasi + nilainya",              3.0),
    ("K",  "tabel kalibrasi pivot & odometri (EEPROM 2048)",    2.0),
    ("k",  "kompas arena yang tercatat",                        2.0),
    ("l",  "tabel jarak keenam LiDAR",                          2.0),
    ("M",  "peta EEPROM + kapasitas chip",                      2.0),
    ("d",  "dump diagnostik: PWM, ServoMap, sudut per kaki",    3.0),
    ("T",  "profil medan yang berlaku      (v1.7+)",            1.5),
    ("N",  "mode kemudi dinding PD/fuzzy    (v1.7+)",           1.5),
    ("Z",  "kemudi lateral menengah/dinding (v1.7+)",           1.5),
    ("Y",  "tabel sudut badan vs dinding    (v1.7+)",           1.5),
    ("i",  "keadaan sensor depan            (v1.7+)",           1.5),
    ("j",  "statistik sebaran LiDAR",                           2.5),
]

GARIS = "=" * 70


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import serial
    except ImportError:
        print("!! pyserial tidak ada:  .venv/bin/pip install pyserial")
        return 1

    # Layanan HUD memegang portnya. Kalau masih jalan, kita TIDAK bisa membuka
    # port yang sama -- dan gejalanya cuma "Device or resource busy" yang
    # membingungkan. Lebih baik dibilang di depan.
    aktif = subprocess.run(["systemctl", "is-active", "r2c-hud"],
                           capture_output=True, text=True).stdout.strip()
    if aktif == "active" and "--paksa" not in sys.argv:
        print("!! layanan r2c-hud masih jalan dan memegang port serialnya.")
        print("   sudo systemctl stop r2c-hud")
        print("   (sesudah selesai:  sudo systemctl start r2c-hud)")
        return 1

    try:
        from mission_hud import cari_port
        port = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") \
            else cari_port()
    except Exception:                                    # noqa: BLE001
        port = (sorted(glob.glob("/dev/serial/by-id/*"))
                or sorted(glob.glob("/dev/ttyACM*")) or [None])[0]

    if not port:
        print("!! Teensy tidak ketemu. Ingat: sesudah pad VUSB-VIN dikikir,")
        print("   USB saja TIDAK menyalakan Teensy -- daya robot harus hidup.")
        return 1

    nama = f"sidik_firmware_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    print(f"port   : {port}")
    print(f"tujuan : {os.path.abspath(nama)}")
    print(f"{len(PERINTAH)} perintah, semuanya hanya MEMBACA. Robot tidak akan bergerak.\n")

    with serial.Serial(port, 115200, timeout=0.2) as ser, \
            open(nama, "w", encoding="utf-8") as fh:

        def tulis(t=""):
            print(t)
            fh.write(t + "\n")

        tulis(GARIS)
        tulis(f"SIDIK JARI FIRMWARE TEENSY  --  {time.strftime('%Y-%m-%d %H:%M:%S')}")
        tulis(f"port: {port} @ 115200")
        tulis("Semua di bawah ini adalah keluaran firmware SENDIRI, apa adanya.")
        tulis(GARIS)

        # Beberapa firmware mencetak sesuatu saat port dibuka (DTR memicu
        # reset pada sebagian board). Tunggu sebentar dan tangkap kalau ada.
        time.sleep(1.0)
        awal = ser.read(65536).decode("utf-8", "replace")
        if awal.strip():
            tulis("\n--- [saat port dibuka] ---")
            tulis(awal.rstrip())

        for cmd, ket, tunggu in PERINTAH:
            tulis(f"\n{GARIS}\n>>> '{cmd}'   ({ket})\n{'-' * 70}")
            ser.reset_input_buffer()
            ser.write((cmd + "\n").encode())
            time.sleep(tunggu)
            # Baca sampai benar-benar sepi: keluaran 'q' dan 'd' panjang dan
            # datang bertahap, jadi satu read() saja memotongnya di tengah.
            data, sepi = "", 0
            while sepi < 4:
                potong = ser.read(65536).decode("utf-8", "replace")
                if potong:
                    data += potong
                    sepi = 0
                else:
                    sepi += 1
                time.sleep(0.25)
            tulis(data.rstrip() if data.strip() else "(tidak ada jawaban)")

    print(f"\n{GARIS}")
    print(f"SELESAI -> {nama}")
    print("\nAmbil ke laptop:")
    print(f"    scp bima@terra-core:~/M/{nama} .")
    print("\nJangan lupa nyalakan lagi HUD-nya:")
    print("    sudo systemctl start r2c-hud")
    return 0


if __name__ == "__main__":
    sys.exit(main())
