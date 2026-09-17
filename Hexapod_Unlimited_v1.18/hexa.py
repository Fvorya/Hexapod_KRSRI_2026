"""Kirim perintah ke hexapod dan baca balasannya sampai robot diam.

Jendela baca TETAP memotong keluaran panjang seperti 'd' dan 'l'. Yang benar
adalah membaca sampai tidak ada byte baru selama beberapa ratus milidetik --
robot ini mencetak dalam satu semburan, jadi jeda sunyi menandai akhir jawaban.

    python hexa.py d l v
    python hexa.py --tunggu 20 "m6 2"
"""
import sys
import time

import serial

PORT = "COM4"
BAUD = 115200


def buka():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, 0.05
    s.dtr = s.rts = True
    s.open()
    # Buffer terima bawaan Windows (4 KB) kekecilan untuk semburan seperti 'm4'
    # atau 'd': robot mencetak 2 KB sekaligus dan awalannya hilang sebelum
    # sempat dibaca. Yang tampak dari sini: perintah seolah tidak menjawab.
    try:
        s.set_buffer_size(rx_size=262144, tx_size=8192)
    except Exception:
        pass
    time.sleep(0.9)          # Teensy CDC perlu sesaat sesudah host membuka port
    s.reset_input_buffer()
    return s


def kirim(s, cmd, sunyi=0.5, batas=8.0, awal=3.0):
    """Kirim satu baris, kembalikan semua yang dicetak sampai sunyi `sunyi` detik.

    `sunyi` hanya berlaku SESUDAH byte pertama tiba. Versi lama menghitungnya
    sejak detik nol, sehingga perintah yang butuh lebih dari 0,5 detik untuk
    MULAI menjawab -- 'l' dan 'v' saat LiDAR sedang sibuk -- pulang sebagai
    string KOSONG. Itu bukan gangguan kosmetik: 6 Sep 2026 sebuah loop koreksi
    membaca balasan kosong itu sebagai "tidak ada halangan yang dilaporkan"
    lalu terus menyuruh robot bergerak sampai menabrak dinding. Sekarang
    balasan kosong hanya mungkin kalau robot benar-benar bisu `awal` detik.
    """
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    s.flush()
    buf, t0, terakhir = b"", time.time(), time.time()
    while time.time() - t0 < batas:
        b = s.read(4096)
        if b:
            buf += b
            terakhir = time.time()
        elif not buf:
            if time.time() - t0 > awal:
                break
        elif time.time() - terakhir > sunyi:
            break
    return buf.decode("utf-8", "replace").strip()


def main():
    args = sys.argv[1:]
    batas = 8.0
    if args and args[0] == "--tunggu":
        batas = float(args[1])
        args = args[2:]
    s = buka()
    try:
        for cmd in args:
            print("=" * 16, cmd, "=" * 16)
            print(kirim(s, cmd, batas=batas) or "(kosong)")
    finally:
        s.close()


if __name__ == "__main__":
    main()
