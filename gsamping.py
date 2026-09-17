"""Geser menyamping bertahap sampai satu sisi mencapai kelegaan tertentu.

Tiga pelajaran dari arena 6 September 2026 tertanam di sini, ketiganya lahir
dari robot yang menabrak dinding berulang kali:

1. Odometri geser bohong, jadi penggarisnya SENSOR SISI, bukan 'D'.
2. `stop()` cuma menolkan vektor gerak; kaki menyelesaikan langkahnya dan badan
   TERUS berpindah sesudahnya. Membaca sensor 1,5 detik sesudah semburan
   memberi angka basi, dan loop koreksi yang memakainya memantulkan robot dari
   dinding ke dinding. Sekarang ditunggu dua siklus gait TANGGA.
3. GAGAL TERTUTUP. Sensor yang tidak terbaca berarti TIDAK TAHU, bukan
   "jalannya lapang". Versi lama menerjemahkan signal fail jadi 999 cm dan
   terus menyemburkan gerakan justru saat robot mendekati dinding.

Perpindahan per semburan TIDAK bisa diminta setepat itu: ia terkuantisasi satu
langkah penuh, dan dua perintah identik bisa berbeda tiga kali lipat. Untuk
offset kecil yang pasti, pakai 't<x> <y> <z>' (geser badan di atas kaki diam)
atau tangan operator.

    python gsamping.py kiri 25           # geser sampai sensor KIRI-DPN >= 25 cm
    python gsamping.py kanan 29 0.5 0.2 3
    python gsamping.py --uji             # periksa parsing, tanpa menyentuh robot
"""
import re
import sys
import time

from hexa import buka, kirim

POLA = {"kiri": r"0 KIRI-DPN\s*:\s*(\d+) cm",
        "kanan": r"4 KANAN-DPN\s*:\s*(\d+) cm"}


def baca_cm(teks, pola):
    """Ambil satu jarak dari keluaran 'l', atau None bila TIDAK DIKETAHUI.

    None berarti sensor tidak menjawab, menjawab 'jauh', atau balasannya kosong.
    Ketiganya harus diperlakukan sama: tidak tahu. Yang TIDAK boleh adalah
    menerjemahkannya jadi angka besar -- dinding gelap, permukaan menyerong,
    dan tanggul rendah semuanya menghasilkan signal fail dari benda yang nyata.
    """
    m = re.search(pola, teks)
    return int(m.group(1)) if m else None


def demo():
    sah = ("--- LIDAR ---\n"
           "  0 KIRI-DPN  : 39 cm   [mentah 387 mm, range valid]\n"
           "  4 KANAN-DPN : 19 cm   [mentah 246 mm, range valid]\n"
           "  sensor hidup: 6 dari 6")
    buta = ("--- LIDAR ---\n"
            "  0 KIRI-DPN  : jauh (di atas 130 cm)   [mentah 387 mm, signal fail]\n"
            "  4 KANAN-DPN : jauh (di atas 130 cm)   [mentah 246 mm, signal fail]\n"
            "  sensor hidup: 6 dari 6")
    assert baca_cm(sah, POLA["kiri"]) == 39
    assert baca_cm(sah, POLA["kanan"]) == 19
    # INI yang menabrakkan robot: signal fail dulu dibaca 999 alias "lapang".
    assert baca_cm(buta, POLA["kiri"]) is None
    assert baca_cm(buta, POLA["kanan"]) is None
    # Balasan kosong (hexa.py pulang terlalu cepat) juga tidak boleh jadi angka.
    assert baca_cm("", POLA["kanan"]) is None
    assert baca_cm("(kosong)", POLA["kanan"]) is None
    print("uji parsing LULUS: 'tidak tahu' tidak pernah jadi 'lapang'.")


def main(argv):
    sisi = argv[0].lower()                                    # sisi yang DILEGAKAN
    sasaran = float(argv[1])
    detik = float(argv[2]) if len(argv) > 2 else 0.5
    laju = float(argv[3]) if len(argv) > 3 else 0.6
    maks = int(argv[4]) if len(argv) > 4 else 4
    pola = POLA[sisi]
    # Melegakan sisi KIRI berarti menjauh darinya, yaitu geser ke KANAN (+).
    arah = +laju if sisi == "kiri" else -laju

    s = buka()
    try:
        print(kirim(s, "D0", sunyi=0.4))
        d = baca_cm(kirim(s, "l", sunyi=0.8, batas=12), pola)
        print("\n%s awal = %s cm, sasaran >= %g cm, semburan %g det laju %g"
              % (sisi.upper(), d, sasaran, detik, laju))
        for i in range(1, maks + 1):
            if d is None:
                print("BERHENTI: sisi %s tidak terbaca -- tanpa penggaris, "
                      "tidak ada semburan lagi." % sisi.upper())
                break
            if d >= sasaran:
                print("SASARAN TERCAPAI.")
                break
            j = kirim(s, "w 0 %g %g" % (arah, detik), sunyi=0.4)
            if "DITOLAK" in j or "PENJAGA BUTA" in j:
                print("semburan %d: %s" % (i, j))
                if "DITOLAK" in j:
                    break
            time.sleep(detik + 3.5)      # 2 siklus gait TANGGA (1,3 s) + jeda
            d = baca_cm(kirim(s, "l", sunyi=0.8, batas=12), pola)
            v = kirim(s, "v", sunyi=0.8, batas=12)
            r = re.search(r"roll (-?[\d.]+) der", v)
            print("semburan %d -> %s = %s cm, roll %s"
                  % (i, sisi.upper(), d, r.group(1) if r else "?"))
        # Berhenti TEGAS lalu tunggu: versi lama menutup port sambil kaki masih
        # melangkah, jadi angka terakhir yang tercetak bukan posisi robot lagi.
        kirim(s, "s", sunyi=0.4)
        time.sleep(3.0)
        print("\n===== keadaan akhir (sesudah 2 siklus gait) =====")
        for c in ("v", "l"):
            print("--------", c)
            print(kirim(s, c, sunyi=0.8, batas=12) or "(kosong)")
    finally:
        s.close()


if __name__ == "__main__":
    if "--uji" in sys.argv:
        demo()
    else:
        main(sys.argv[1:])
