"""Geser menyamping lewat PIVOT, bukan jalan kepiting.

Jalan kepiting sudah ditolak di arena: ~6 cm sapuan kaki untuk 1 cm
perpindahan, dan sensor sisi buta terhadap tanggul 1-2 cm. Yang dipakai:
pivot ke arah tujuan (MUTLAK, dari tabel kompas), maju berpagar rem jarak,
lalu pivot kembali ke arah semula -- juga MUTLAK, sehingga galat kedua pivot
tidak menumpuk.

    python geser.py 3 2 10     # pivot ke BARAT, maju 10 cm, balik ke SELATAN
"""
import sys, time
from hexa import buka, kirim

ke, balik, cm = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
NAMA = ("UTARA", "TIMUR", "SELATAN", "BARAT")


def tunggu(s, tanda, batas=40.0):
    t0 = time.time()
    while time.time() - t0 < batas:
        b = s.read(4096)
        if b:
            sys.stdout.write(b.decode("utf-8", "replace")); sys.stdout.flush()
            if any(t.encode() in b for t in tanda):
                return True
    print("\n!! batas %g detik lewat" % batas)
    return False


s = buka()
try:
    print(">> pivot ke", NAMA[ke], "|", kirim(s, "o%d" % ke, sunyi=0.4))
    if not tunggu(s, ("Pivot SELESAI", "BERHENTI")):
        print(kirim(s, "s")); raise SystemExit(1)

    print("\n-------- l sebelum maju\n" + kirim(s, "l", sunyi=0.8, batas=12))

    for c in ("D0", "D%g" % cm):
        print(">>", c, "|", kirim(s, c, sunyi=0.4))
    # 'w' dibatasi waktu JUGA, bukan hanya rem: kalau rem tak kunjung tercapai
    # (kaki selip di puing), timer inilah yang menghentikan robot.
    print(">> w |", kirim(s, "w 0.6 0 6", sunyi=0.4))
    # Kalau tanda berhentinya tidak pernah datang, JANGAN diam saja: kirim 's'.
    # Batas waktu 'w' memang jaring terakhir di firmware, tapi mengandalkannya
    # berarti robot berjalan beberapa detik tanpa ada yang mengawasi dari sini.
    if not tunggu(s, ("berhenti", "BERHENTI", "Rem jarak"), batas=15):
        print(kirim(s, "s", sunyi=0.4))
    time.sleep(1.0)

    print("\n>> pivot balik ke", NAMA[balik], "|", kirim(s, "o%d" % balik, sunyi=0.4))
    tunggu(s, ("Pivot SELESAI", "BERHENTI"))

    print("\n===== keadaan =====")
    for c in ("D", "v", "l"):
        print("--------", c)
        print(kirim(s, c, sunyi=0.8, batas=12) or "(kosong)")
finally:
    s.close()
