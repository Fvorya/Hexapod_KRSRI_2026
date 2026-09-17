#!/usr/bin/env python3
"""sidik_imu.py -- cari tahu KENAPA IMU diam, dari Raspberry Pi.

    python3 sidik_imu.py                  # pindai semua port & baud
    python3 sidik_imu.py --port /dev/ttyUSB0
    python3 sidik_imu.py --baud 9600 --lama 5   # dengarkan satu baud, lama
    python3 sidik_imu.py --uji             # uji parser pakai frame buatan

KENAPA INI ADA
--------------
Satu kegagalan yang sama menghentikan hampir semua yang kita coba, dan
selama berhari-hari ia menyamar jadi banyak masalah berbeda:

    'C'  kalibrasi pivot   -> Navigation.cpp:460  !_imu.hasData()
    'c0' catat kompas      -> Navigation.cpp:65   !_imu.hasData()
    'O'  pivot relatif     -> pivotRelatif() kembali DIAM-DIAM
    'o0' hadap utara       -> "Gagal: Tidak ada data IMU."
    'm1' mulai misi        -> Navigation.cpp:617  !_imu.hasData()

Semuanya satu sebab: tidak ada frame WIT yang sampai ke Teensy.

YANG PALING MUNGKIN, DAN KENAPA
-------------------------------
Imu::begin() cuma MEMBUKA port. Ia tidak pernah mengirim apa pun ke modul --
tidak ada perintah pindah baud, tidak ada permintaan mulai mengirim. Jadi
seluruh sambungan bergantung pada modul yang SUDAH berada di baud yang sama.

Dan komentar Vincent sendiri di Imu.cpp:19 berbunyi:

    "Pastikan IMU_BAUD di config.h sudah diubah ke 230400"

Kata "diubah" itu petunjuknya: angka aslinya bukan 230400. Modul WIT (dan
Yahboom 10-axis memakai protokol WIT) keluar pabrik pada 9600. Kalau modulnya
tidak pernah disetel ulang, ia mengirim frame 0x55 dengan rajin pada 9600
sementara Teensy mendengarkan di 230400 -- dan hasilnya SENYAP TOTAL, tanpa
satu pun pesan error. Gejalanya sama persis dengan kabel putus.

SOAL TYPE-C YANG KOSONG ITU
---------------------------
Aku pernah bilang itu mungkin masukan daya. Kemungkinan besar aku KELIRU.
Di modul Yahboom 10-axis, Type-C itu biasanya antarmuka USB-serial (chip
CH340/CP2102 di papan yang sama) -- bukan colokan daya. Kalau benar, colok
saja ke Pi: modulnya muncul sebagai /dev/ttyUSB0 dan skrip ini bisa
mendengarkannya LANGSUNG, tanpa lewat Teensy dan tanpa alat tambahan.

Itu yang membuat pemeriksaan ini mungkin dikerjakan hari ini juga: kalau
skrip ini menemukan frame sah pada baud X, kita tahu modulnya HIDUP dan
masalahnya cuma angka -- dan kalau tidak menemukan apa pun di baud mana pun,
kita tahu masalahnya kabel atau daya, dan berhenti menyetel angka.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time

WIT_LEN = 11
WIT_HEAD = 0x55
# Tipe paket WIT yang kita pedulikan. 0x53 = sudut Euler, itu yang dipakai
# yawDeg() dan karena itu yang menentukan hidup-matinya seluruh navigasi.
TIPE = {0x50: "waktu", 0x51: "akselerasi", 0x52: "giro", 0x53: "SUDUT",
        0x54: "magnet", 0x55: "port", 0x56: "tekanan", 0x57: "GPS",
        0x58: "kecepatan GPS", 0x59: "kuaternion", 0x5A: "akurasi GPS"}

# 230400 ditaruh di urutan yang wajar, tapi 9600 SENGAJA pertama: itu bawaan
# pabrik WIT dan tersangka utama di kasus ini.
BAUD_COBA = [9600, 230400, 115200, 57600, 38400, 19200, 460800, 921600, 4800]


def frame_sah(b: bytes) -> bool:
    """Frame WIT: 11 byte, header 0x55, checksum = jumlah 10 byte pertama."""
    if len(b) != WIT_LEN or b[0] != WIT_HEAD:
        return False
    return (sum(b[:WIT_LEN - 1]) & 0xFF) == b[WIT_LEN - 1]


def cari_frame(data: bytes):
    """Sisir byte mentah, kembalikan daftar frame sah.

    Disinkronkan ulang byte per byte, persis seperti Imu::update() di Teensy.
    Sengaja meniru firmware: kalau skrip ini menemukan frame tapi firmware
    tidak, bedanya ada di tempat lain -- bukan di cara membaca.
    """
    hasil = []
    i = 0
    while i + WIT_LEN <= len(data):
        if data[i] != WIT_HEAD:
            i += 1
            continue
        f = data[i:i + WIT_LEN]
        if frame_sah(f):
            hasil.append(f)
            i += WIT_LEN
        else:
            i += 1
    return hasil


def sudut(f: bytes):
    """Roll/pitch/yaw dari frame 0x53. Skala WIT: nilai/32768*180 derajat."""
    if f[1] != 0x53:
        return None

    def s16(lo, hi):
        v = lo | (hi << 8)
        return v - 65536 if v & 0x8000 else v

    return tuple(s16(f[2 + 2 * k], f[3 + 2 * k]) / 32768.0 * 180.0
                 for k in range(3))


def daftar_port():
    p = sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                   + glob.glob("/dev/serial/by-id/*")))
    return p


def dengar(port, baud, lama):
    """Buka port pada baud itu, kumpulkan byte, kembalikan (byte, frame)."""
    try:
        import serial                                       # type: ignore
    except ImportError:
        print("!! pyserial belum ada:  pip3 install pyserial --break-system-packages")
        sys.exit(2)
    try:
        with serial.Serial(port, baud, timeout=0.1) as s:
            # Buang yang sudah tertimbun -- isinya byte dari baud sebelumnya
            # dan cuma akan jadi frame palsu.
            time.sleep(0.15)
            s.reset_input_buffer()
            data = b""
            t0 = time.time()
            while time.time() - t0 < lama:
                data += s.read(4096)
            return data, cari_frame(data)
    except Exception as e:                                  # noqa: BLE001
        return None, str(e)


def pindai(port, lama):
    print(f"\n=== {port} ===")
    temuan = []
    for baud in BAUD_COBA:
        data, hasil = dengar(port, baud, lama)
        if data is None:
            print(f"  {baud:>7} : tidak bisa dibuka -- {hasil}")
            continue
        n = len(data)
        if isinstance(hasil, str):
            print(f"  {baud:>7} : {hasil}")
            continue
        tipe = {}
        for f in hasil:
            tipe[f[1]] = tipe.get(f[1], 0) + 1
        ket = ", ".join(f"{TIPE.get(t, hex(t))}x{c}" for t, c in sorted(tipe.items()))
        tanda = "  <<<<" if hasil else ""
        print(f"  {baud:>7} : {n:>6} byte, {len(hasil):>4} frame sah"
              + (f"  [{ket}]" if ket else "") + tanda)
        if hasil:
            temuan.append((baud, len(hasil), tipe, hasil))
    return temuan


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="satu port saja")
    p.add_argument("--baud", type=int, default=None, help="satu baud saja")
    p.add_argument("--lama", type=float, default=1.2,
                   help="detik mendengarkan per baud (bawaan 1,2)")
    p.add_argument("--uji", action="store_true",
                   help="uji parser dengan frame buatan, tanpa perangkat")
    args = p.parse_args()

    if args.uji:
        return uji_sendiri()

    print("sidik_imu -- mencari frame WIT (0x55, 11 byte, checksum)")
    ports = [args.port] if args.port else daftar_port()
    if not ports:
        print("\n!! Tidak ada /dev/ttyUSB* atau /dev/ttyACM* sama sekali.")
        print("   Kalau modul IMU punya Type-C, COLOK ke Pi -- di Yahboom 10-axis")
        print("   itu biasanya antarmuka USB-serial (CH340/CP2102), bukan colokan")
        print("   daya. Sesudah dicolok, periksa 'lsusb' lalu jalankan ini lagi.")
        return 2

    print(f"port: {', '.join(ports)}")
    if args.baud:
        global BAUD_COBA
        BAUD_COBA = [args.baud]

    semua = []
    for port in ports:
        for baud, n, tipe, frames in pindai(port, args.lama):
            semua.append((port, baud, n, tipe, frames))

    print("\n" + "=" * 64)
    if not semua:
        print("TIDAK ADA frame sah di baud mana pun, di port mana pun.")
        print()
        print("Artinya masalahnya BUKAN angka baud, jadi berhenti menyetel angka.")
        print("Yang tersisa tiga: modul tidak berdaya, kabelnya putus/terbalik,")
        print("atau modulnya memang rusak.")
        print()
        print("Urutan memeriksanya:")
        print("  1. 'lsusb' -- kalau modul dicolok lewat Type-C tapi tidak muncul")
        print("     di situ, dayanya yang tidak sampai. Ganti kabel dulu: kabel")
        print("     Type-C yang cuma untuk mengecas TIDAK punya jalur data.")
        print("  2. TX/RX tertukar. Pada sambungan ke Teensy: TX modul -> RX")
        print("     Teensy (pin 7), RX modul -> TX Teensy (pin 8). Tertukar =")
        print("     senyap total, sama persis dengan yang kita lihat.")
        print("  3. Flexible connector. Ia gampang terlihat terpasang padahal")
        print("     kontaknya tidak kena. Cabut dan pasang ulang sekali.")
        return 1

    print("KETEMU. Modulnya HIDUP dan mengirim frame WIT yang sah:")
    for port, baud, n, tipe, frames in semua:
        print(f"\n  {port} @ {baud} -- {n} frame")
        for f in frames[:200]:
            a = sudut(f)
            if a:
                print(f"     sudut: roll {a[0]:+7.2f}  pitch {a[1]:+7.2f}"
                      f"  yaw {a[2]:+7.2f}")
                break
        else:
            print("     (tidak ada paket 0x53/SUDUT -- lihat catatan di bawah)")

    baud_ok = semua[0][1]
    punya_sudut = any(0x53 in t for _, _, _, t, _ in semua)
    print("\n" + "-" * 64)
    if baud_ok != 230400:
        print(f"Modul berbicara pada {baud_ok}, firmware mendengarkan di 230400.")
        print("Itu sebabnya senyap -- bukan kabel, bukan modul rusak.")
        print()
        print("Dua jalan, pilih salah satu:")
        print(f"  A. Ubah firmware ikut modul  -- config.h:629")
        print(f"       #define IMU_BAUD  {baud_ok}")
        print("     Paling cepat, dan tidak menyentuh modul sama sekali.")
        print("  B. Ubah modul ikut firmware -- perlu perintah konfigurasi WIT")
        print("     (buka kunci, setel baud, simpan). Kode lajunya berbeda-beda")
        print("     antar model, jadi JANGAN kirim membabi buta: setel lewat")
        print("     aplikasi/alat resmi modulnya, lalu jalankan skrip ini lagi")
        print("     untuk MEMBUKTIKAN ia benar-benar pindah.")
        print()
        print("  Kalau ragu, ambil A. Ia satu baris, mudah dibalik, dan tidak")
        print("  bisa membuat modul jadi tak bisa dihubungi.")
    else:
        print("Modul SUDAH di 230400, sama dengan firmware.")
        print("Jadi masalahnya bukan baud. Yang tersisa: jalur ke Teensy.")
        print("  - TX modul harus ke pin 7 (RX2), RX modul ke pin 8 (TX2).")
        print("  - Kalau lewat Type-C ke Pi jalan tapi ke Teensy tidak, berarti")
        print("    frame-nya keluar lewat USB, bukan lewat pin UART. Sebagian")
        print("    modul memang mengeluarkan lewat SATU saja, bukan keduanya.")
    if not punya_sudut:
        print()
        print("CATATAN: ada frame sah tapi TIDAK ADA paket 0x53 (SUDUT).")
        print("Hanya 0x53 yang mengisi yawDeg(), jadi hasData() akan tetap")
        print("false walau modulnya ramai. Nyalakan keluaran 'angle' di")
        print("pengaturan modul.")
    return 0


def uji_sendiri():
    """Uji parser tanpa perangkat: frame buatan yang sudah diketahui isinya."""
    ok = gagal = 0

    def cek(nama, dapat, harap):
        nonlocal ok, gagal
        if dapat == harap:
            ok += 1
            print(f"  OK   {nama}")
        else:
            gagal += 1
            print(f"  GAGAL {nama}: dapat {dapat!r}, harap {harap!r}")

    def buat(tipe, nilai):
        b = bytearray([WIT_HEAD, tipe])
        for v in nilai:
            n = int(round(v / 180.0 * 32768.0)) & 0xFFFF
            b += bytes([n & 0xFF, (n >> 8) & 0xFF])
        b += bytes([0, 0])                       # padding sampai 10 byte
        b = b[:WIT_LEN - 1]
        b.append(sum(b) & 0xFF)
        return bytes(b)

    print("uji parser sidik_imu (tanpa perangkat)\n")
    f = buat(0x53, [10.0, -20.0, 90.0])
    cek("panjang frame 11 byte", len(f), 11)
    cek("frame buatan dianggap sah", frame_sah(f), True)
    r, p, y = sudut(f)
    cek("roll ~10", round(r, 1), 10.0)
    cek("pitch ~-20", round(p, 1), -20.0)
    cek("yaw ~90", round(y, 1), 90.0)

    rusak = bytearray(f); rusak[-1] ^= 0xFF
    cek("checksum rusak ditolak", frame_sah(bytes(rusak)), False)
    salah = bytearray(f); salah[0] = 0x54
    cek("header salah ditolak", frame_sah(bytes(salah)), False)
    cek("frame non-0x53 tidak menghasilkan sudut", sudut(buat(0x51, [1, 2, 3])), None)

    # Sinkronisasi ulang: frame sah yang didahului sampah harus tetap ketemu.
    cek("frame di tengah sampah tetap ketemu",
        len(cari_frame(b"\x00\xff\x55\x12" + f + b"\x99")), 1)
    cek("dua frame beruntun", len(cari_frame(f + f)), 2)
    cek("sampah murni -> nol frame", len(cari_frame(bytes(200))), 0)
    # Byte 0x55 yang kebetulan muncul di tengah data tidak boleh menipu.
    cek("0x55 palsu tidak dihitung",
        len(cari_frame(b"\x55" * 10 + f)), 1)

    print(f"\n=== {ok} lulus, {gagal} gagal ===")
    return 1 if gagal else 0


if __name__ == "__main__":
    sys.exit(main())
