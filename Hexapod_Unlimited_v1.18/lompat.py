"""Satu lompatan mapping: nolkan odometri, pasang rem, jalan, tunggu diam, lapor.

Dipisah dari hexa.py karena hexa.py mengirim perintah lalu PULANG; mapping
butuh menunggu robot benar-benar berhenti di rem jaraknya sebelum bertanya,
dan bertanya lewat sambungan yang SAMA supaya tidak ada jeda buka-port di
tengah lompatan.

    python lompat.py 10          # lompat 10 cm dengan 'P' (dinding kanan+kompas)
    python lompat.py 10 p        # pakai dinding kiri
    python lompat.py 0           # tidak melompat, cuma lapor keadaan
"""
import sys, time
from hexa import buka, kirim

cm = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
mode = sys.argv[2] if len(sys.argv) > 2 else "P"
batas = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
# Arah arena untuk DILURUSKAN sebelum melangkah (0=U 1=T 2=S 3=B), "-" = jangan.
# Dijadikan bawaan karena tiap ruas yang sudah diukur menunjukkan pola yang
# sama: dinding samping duduk di dalam pita wall.min, aturan "terlalu dekat"
# memicu putaran menjauh terus-menerus, dan kunci kompas KALAH. Simpang tumbuh
# ke 9-15 der dalam dua lompatan, dan tiap derajat memotong odometri
# (cos 15 der = 3%). Satu pivot di awal lompatan mengembalikannya ke <1 der.
luruskan = sys.argv[4] if len(sys.argv) > 4 else "2"
# "i1" = abaikan sensor depan selama lompatan ini (bidang miring / tangga).
# WAJIB dikirim SESUDAH mode navigasi mulai, bukan sebelum: navBerhenti()
# memulihkan sensor depan, dan tiap lompatan berakhir dengan berhenti, jadi
# 'i1' yang dikirim lebih dulu sudah hangus sebelum kaki bergerak. 6 Sep 2026
# robot lolos dari tangga hanya karena kebetulan -- pantulan 59 mm jatuh di
# bawah ambang "mustahil" 7 cm sehingga dilaporkan 'jauh', bukan halangan.
buta = "i1" in sys.argv[5:]

def tunggu(s, tanda, batas=40.0):
    t0 = time.time()
    while time.time() - t0 < batas:
        b = s.read(4096)
        if b:
            sys.stdout.write(b.decode("utf-8", "replace")); sys.stdout.flush()
            if any(t.encode() in b for t in tanda):
                return True
    return False


s = buka()
try:
    if luruskan != "-":
        print(">> o%s |" % luruskan, kirim(s, "o" + luruskan, sunyi=0.4))
        tunggu(s, ("Pivot SELESAI", "BERHENTI"))
        time.sleep(1.5)
    if cm > 0:
        for c in ("D0", "D%g" % cm, mode):
            print(">>", c, "|", kirim(s, c, sunyi=0.4))
        if buta:
            print(">> i1 |", kirim(s, "i1", sunyi=0.4))
        t0 = time.time()
        while time.time() - t0 < batas:
            b = s.read(4096)
            if b:
                sys.stdout.write(b.decode("utf-8", "replace")); sys.stdout.flush()
                if b"berhenti" in b or b"BERHENTI" in b:
                    break
        else:
            print("\n!! batas %g detik lewat -- HENTIKAN paksa" % batas)
            print(kirim(s, "s"))
        time.sleep(0.5)
    print("\n===== keadaan =====")
    for c in ("D", "v", "l"):
        print("--------", c)
        print(kirim(s, c, sunyi=0.8, batas=12) or "(kosong)")
finally:
    s.close()
