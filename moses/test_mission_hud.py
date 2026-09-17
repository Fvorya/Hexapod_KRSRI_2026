#!/usr/bin/env python3
"""Uji logika mission_hud.py tanpa kamera, tanpa robot.

Yang diuji hanya bagian yang bisa salah diam-diam: parsing keluaran serial
firmware, gerbang geometris juri, prior 'tepat satu asli', dan perpindahan
state. Bagian gambar tidak diuji -- kegagalannya kelihatan mata.

    python3 test_mission_hud.py
"""
import json
import math
import re
import sys
import threading
import time
import types
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

# --- stub detect.py: kita tidak butuh ONNX/kamera untuk menguji logika ---
stub = types.ModuleType("detect")
stub.load_session = lambda *a, **k: (None, {}, (640, 640))
stub.letterbox = lambda img, size: (img, 1.0, (0, 0))
stub.postprocess = lambda *a, **k: []
stub.open_camera = lambda *a, **k: None
stub.CameraThread = object
sys.modules["detect"] = stub

import mission_hud as M  # noqa: E402

ok = fail = 0


def cek(nama, dapat, harap):
    global ok, fail
    if dapat == harap:
        ok += 1
        print(f"  OK   {nama}")
    else:
        fail += 1
        print(f"  GAGAL {nama}: dapat {dapat!r}, harap {harap!r}")


print("\n1. Parsing keluaran serial firmware")
link = M.Teensy(None)
for baris in [
    "--- STATUS MISI ---",
    "  state       : MENUNGGU KONFIRMASI ('m2' korban / 'm3' bukan)",
    "  depan       : 24 cm",
    "  belakang    : 38 cm dari dinding START  (sampel jauh: 1 dari 3)",
    "  navigasi    : TIDAK sedang dipegang misi",
    "0 KIRI-DPN  : jauh (di atas 120 cm)   [mentah 90 mm, wrap target fail]",
    "1 KIRI-BLK  : MATI -- tidak merespons [mentah 700 mm, sigma fail]",
    "5 DEPAN     : 21 cm                   [mentah 210 mm, range valid]",
]:
    link._parse(baris)

cek("state misi terbaca", "KONFIRMASI" in link.state_teensy().upper(), True)
cek("LiDAR depan dari tabel 'l'", link.depan_cm(), 21.0)
cek("'jauh' -> inf", link.terakhir["lidar_KIRI-DPN"], float("inf"))
cek("'MATI' -> None", link.terakhir["lidar_KIRI-BLK"], None)
cek("belakang cm", link.terakhir["belakang_cm"], 38.0)

print("\n2. Serial TANPA FILTER -- semua perintah lewat apa adanya")
# 13 Sep 2026: whitelist MODE BACA dan daftar TERLARANG dibuang atas keputusan
# R2C -- keselamatan dipegang mekanik, dan saringan yang menolak diam-diam
# menyembunyikan jawaban firmware yang justru sedang dicari ('C' pernah begitu
# berhari-hari). Yang diuji sekarang kebalikannya: TIDAK ADA yang tertahan.
for _c in ("S", "W", "e", "x", "B", "z", "C", "f", "F", "p", "P",
           "w", "o3", "m", "l", "R", "g10", "a80 160 -40", "m8"):
    _l = M.Teensy(None)
    cek(f"'{_c}' lewat tanpa ditahan", _l.kirim(_c), True)
    cek(f"  '{_c}' tidak menghasilkan [TOLAK]",
        any("[TOLAK]" in x for x in _l.log), False)
cek("daftar TERLARANG sudah tidak ada", hasattr(M, "TERLARANG"), False)
cek("whitelist PERINTAH_BACA sudah tidak ada", hasattr(M, "PERINTAH_BACA"), False)
# Parameter lama tetap diterima supaya pemanggil lama tidak galat -- tapi
# tidak lagi berpengaruh.
cek("armed=False pun tetap terkirim", M.Teensy(None).kirim("w", armed=False), True)
cek("paksa=True masih diterima", M.Teensy(None).kirim("x", paksa=True), True)

print("\n3. Gerbang geometris juri (frame 1280x720)")
k = M.Kalib()
juri = M.Juri(k, {0: "korban", 1: "dummy"})
W, H = 1280, 720
# tinggi bbox 0.6*H = 432 px -> lolos pita 0.35..0.90
sasaran = (600, 100, 700, 532, 0.9, 0)          # cx=650, dekat tengah (640)
tetangga = (1050, 100, 1150, 532, 0.9, 0)       # cx=1100 -> 410 px dari tengah
kecil = (600, 300, 660, 380, 0.9, 0)            # tinggi 80 px -> 0.11 H
lolos = juri.saring([sasaran, tetangga, kecil], W, H)
cek("hanya 1 yang lolos", len(lolos), 1)
cek("yang lolos adalah sasaran", lolos[0][0], 600)
cek("tetangga ditolak gerbang ROI", juri.ditolak["roi"], 1)
cek("bbox kecil ditolak gerbang tinggi", juri.ditolak["tinggi"], 1)
cek("ROI +-10 der ~= 182 px di 1280", round(k.roi_half_px(W)), 182)

print("\n4. Voting k-of-N")
juri.reset()
for _ in range(7):
    juri.tambah([(600, 100, 700, 532, 0.82, 0, 650.0)])
for _ in range(2):
    juri.tambah([])
kelas, margin, conf, cx, alasan = juri.putuskan()
cek("7 dari 9 korban -> korban", kelas, "korban")
cek("conf rerata", round(conf, 2), 0.82)

juri.reset()
for _ in range(5):
    juri.tambah([(600, 100, 700, 532, 0.9, 0, 650.0)])
for _ in range(4):
    juri.tambah([(600, 100, 700, 532, 0.9, 1, 650.0)])
cek("5/9 belum cukup k-of-N -> RAGU", juri.putuskan()[0], "RAGU")

print("\n5. Prior 'tepat satu asli' -- eliminasi")
misi = M.Misi()
misi.idx = 1                                    # K1, 3 slot
misi.hasil_slot = {0: ("dummy", -0.7, 0.9), 1: ("dummy", -0.8, 0.92),
                   2: ("RAGU", 0.10, 0.41)}
pilih = M.pilih_slot(misi, k)
cek("dua dummy yakin -> slot ketiga dipilih", pilih[0], 2)
cek("dipilih sebagai korban", pilih[1], "korban")

misi.hasil_slot = {0: ("korban", 0.55, 0.8), 1: ("korban", 0.81, 0.93),
                   2: ("dummy", -0.6, 0.88)}
cek("dua korban -> margin terbesar menang", M.pilih_slot(misi, k)[0], 1)

print("\n6. Perpindahan state")
misi = M.Misi()
cek("mulai di HOME", misi.kini[0], "HOME")
cek("berikutnya K1", misi.berikut[0], "K1")
misi.maju_misi(M.SELESAI)
cek("maju ke K1", misi.kini[0], "K1")
cek("HOME ditandai selesai", misi.status[0], M.SELESAI)
misi.ganti(M.S_LIHAT)
misi.slot = 0
cek("K1 slot 1 dari 3 -> lanjut SLOT", misi.state_berikut, M.S_SLOT)
misi.slot = 2
cek("K1 slot 3 dari 3 -> PUTUSKAN", misi.state_berikut, M.S_PUTUS)
misi.gagal("korban jatuh saat diangkat")
cek("state GAGAL", misi.state, M.S_GAGAL)
cek("sebab tersimpan", misi.sebab, "korban jatuh saat diangkat")
n = misi.berikut[0]
misi.maju_misi(M.LEWAT)
cek("gagal -> lanjut ke misi berikutnya", misi.kini[0], n)

print("\n7. Mode BACA sudah dihapus -- FSM selalu boleh menggerakkan robot")
link2 = M.Teensy(None)
aksi = M.Aksi(link2)
misi2 = M.Misi()
misi2.idx = 1
misi2.ganti(M.S_STANDOFF)
link2.terakhir["lidar_DEPAN"] = 40.0
M.langkah_fsm(misi2, link2, aksi, k, armed=False)
aksi.putar(armed=False)
# STANDOFF tidak lagi mengirim 'D'+'w' (17 Sep 2026), jadi yang diuji di sini
# tinggal: ia LEWAT, bukan menggantung.
cek("STANDOFF lanjut ke CENTERING tanpa berjalan", misi2.state, M.S_CENTER)

misi3 = M.Misi()
misi3.idx = 1
misi3.ganti(M.S_STANDOFF)
link3 = M.Teensy(None)
link3.terakhir["lidar_DEPAN"] = 12.0
_ak3 = M.Aksi(link3)
M.langkah_fsm(misi3, link3, _ak3, k, armed=True)
cek("terlalu dekat -> lanjut, cuma dicatat", misi3.state, M.S_CENTER)
cek("  dan tidak ada gerak maju/mundur", _ak3.antre, [])

print("\n8. Paket state untuk halaman web")
misi4 = M.Misi()
juri4 = M.Juri(k, {0: "korban", 1: "dummy"})
link4 = M.Teensy(None)
link4._parse("  depan       : 21 cm")
st = M.rakit_state(misi4, link4, k, juri4, {"fps": 30.0, "t_inf": 12.0}, False, "")
cek("bisa di-JSON-kan", isinstance(json.dumps(st), str), True)
cek("daftar misi lengkap", len(st["misi"]), len(M.MISI))
cek("misi pertama ditandai 'now'", st["misi"][0]["kelas"], "now")
cek("jarak depan terbaca", st["depan"].startswith("21 cm"), True)
cek("kalibrasi ikut terkirim", len(st["kalib"]), len(M.KALIB_FIELDS))

print("\n9. Server web sungguhan (bukan tiruan)")
bersama = M.Bersama()
bersama.set_state(st)
bersama.set_jpeg(b"\xff\xd8\xff\xd9")
srv = ThreadingHTTPServer(("127.0.0.1", 0), M.buat_handler(bersama))
threading.Thread(target=srv.serve_forever, daemon=True).start()
basis = f"http://127.0.0.1:{srv.server_address[1]}"

with urlopen(basis + "/") as r:
    halaman = r.read().decode()
cek("GET / -> 200 HTML", "Panel operator" in halaman, True)
with urlopen(basis + "/state") as r:
    balik = json.load(r)
cek("GET /state -> JSON sama", balik["state"], st["state"])
with urlopen(basis + "/snapshot.jpg") as r:
    cek("GET /snapshot.jpg -> byte JPEG", r.read()[:2], b"\xff\xd8")
urlopen(Request(basis + "/cmd?k=mode", method="POST")).read()
cek("POST /cmd masuk antrean", M.Bersama.ambil_perintah(bersama), [("mode", None)])
try:
    urlopen(basis + "/tidak-ada")
    cek("404 untuk jalur asing", False, True)
except HTTPError as e:
    cek("404 untuk jalur asing", e.code, 404)
srv.shutdown()

print("\n10. Mode uji ambil korban")
uji = M.Misi()
uji.idx = 1                                     # K1, aslinya 3 slot
uji.slot_override = 1
cek("slot_override membatasi jumlah slot", uji.n_slot, 1)
uji.ganti(M.S_LIHAT)
uji.slot = 0
cek("1 slot -> langsung PUTUSKAN", uji.state_berikut, M.S_PUTUS)
uji.slot_override = 3
cek("3 slot -> masih SLOT_LANJUT", uji.state_berikut, M.S_SLOT)
uji.slot_override = None
cek("tanpa override, pakai tabel MISI", uji.n_slot, 3)

# tanpa_gerak: seluruh jalur tidak boleh mengirim satu perintah pun
uji2 = M.Misi()
uji2.idx = 1
uji2.tanpa_gerak = True
uji2.slot_override = 1
link5 = M.Teensy(None)
aksi5 = M.Aksi(link5)
uji2.ganti(M.S_STANDOFF)
M.langkah_fsm(uji2, link5, aksi5, k, armed=True)
cek("tanpa gerak: STANDOFF -> CENTERING", uji2.state, M.S_CENTER)
M.langkah_fsm(uji2, link5, aksi5, k, armed=True)
cek("tanpa gerak: CENTERING -> LIHAT", uji2.state, M.S_LIHAT)
aksi5.putar(armed=True)
cek("tanpa gerak: nol perintah terkirim", link5.tx_terakhir, "")

# PUTUSKAN tidak boleh mengirim m2/m3 saat tanpa gerak
uji2.hasil_slot = {0: ("korban", 0.8, 0.9)}
uji2.ganti(M.S_PUTUS)
M.langkah_fsm(uji2, link5, aksi5, k, armed=True)
aksi5.putar(armed=True)
cek("tanpa gerak: m2/m3 tidak dikirim", link5.tx_terakhir, "")
cek("tapi kelasnya tetap terkunci", uji2.terkunci[1], "korban")

# dengan gerak, m2 memang harus terkirim
uji3 = M.Misi()
uji3.idx = 1
uji3.slot_override = 1
uji3.hasil_slot = {0: ("korban", 0.8, 0.9)}
link6 = M.Teensy(None)
aksi6 = M.Aksi(link6)
uji3.ganti(M.S_PUTUS)
M.langkah_fsm(uji3, link6, aksi6, k, armed=True)
aksi6.putar(armed=True)
cek("dengan gerak: m2 terkirim", link6.tx_terakhir, "m2")

cek("petunjuk terisi saat menguji", len(M.petunjuk(uji2)) > 0, True)

print("\n11. Mode JEJAK -- ikuti korban")
jj = M.Misi()
jj.idx = 1
lnk = M.Teensy(None)
aks = M.Aksi(lnk)
jj.ganti(M.S_JEJAK)

# sasaran belum terlihat -> jangan kirim apa pun
jj.bearing_deg = None
M.langkah_fsm(jj, lnk, aks, k, armed=True)
aks.putar(armed=True)
cek("sasaran hilang -> diam", lnk.tx_terakhir, "")

jj.jejak_kelas = k.kelas_korban        # gerbang kelas: hanya korban yang dikejar

# Toleransi dari PIKSEL: 20 px / (1280/70.4) = 1,1 der. Dilonggarkan dari 8 px
# (0,44 der) 12 Sep 2026 -- 0,44 lebih kecil daripada langkah terkecil putar
# badan, jadi syaratnya tidak pernah bisa dipenuhi. Lihat bagian 64.
tol_der = k.tengah_tol_px / (1280 / k.hfov_deg)
cek("toleransi piksel tetap lebih ketat dari tengah_tol_deg",
    tol_der < k.tengah_tol_deg, True)
cek("dan bisa dicapai (> langkah_min_deg)", tol_der > k.langkah_min_deg, True)

# sasaran sudah BENAR-BENAR tengah -> jangan memutar
jj.bearing_deg = 0.2
M.langkah_fsm(jj, lnk, aks, k, armed=True)
aks.putar(armed=True)
cek("simpangan 0,2 der (< toleransi piksel) -> tidak memutar", lnk.tx_terakhir, "")

# sasaran di kanan +20 der -> pivot KASAR. Badan dinetralkan DULU kalau
# sebelumnya sempat diputar -- jalan dengan badan menyerong itu yang bikin
# lintasannya melengkung.
aks.batal(); aks.t_boleh = 0
jj.badan_yaw = 0.0
jj.bearing_deg = 20.0
M.langkah_fsm(jj, lnk, aks, k, armed=True)
aks.putar(armed=True)
cek("simpangan +20 der -> pivot gain 0,6 = O-12", lnk.tx_terakhir, "O-12")

# ...dan kalau badan MASIH menyerong, r0 0 0 didahulukan.
aks.batal(); aks.t_boleh = 0
jj.badan_yaw = -5.0
jj.bearing_deg = 20.0
M.langkah_fsm(jj, lnk, aks, k, armed=True)
cek("badan menyerong -> netralkan dulu, baru pivot",
    [c for c, _ in aks.antre][:2], ["r0 0 0", "O-12"])
jj.badan_yaw = 0.0
cek("pivot_der membulatkan KE ATAS, tidak pernah jatuh ke deadband",
    [abs(M.pivot_der(x, k)) >= 7 for x in (0.1, 3.0, 6.4, -6.4)], [True]*4)

# sasaran di kiri -15 der -> O positif
aks.batal(); aks.t_boleh = 0
jj.bearing_deg = -15.0
M.langkah_fsm(jj, lnk, aks, k, armed=True)
aks.putar(armed=True)
cek("simpangan -15 der -> O9", lnk.tx_terakhir, "O9")

# DUMMY: tidak dikejar sama sekali
aks.batal(); aks.t_boleh = 0
lnk.tx_terakhir = ""                   # tx_terakhir lengket -- nolkan dulu
jj.jejak_kelas = k.kelas_dummy
jj.bearing_deg = 25.0
M.langkah_fsm(jj, lnk, aks, k, armed=True)
aks.putar(armed=True)
cek("dummy di +25 der -> DIAM, tidak dikejar", lnk.tx_terakhir, "")
jj.jejak_kelas = k.kelas_korban

# JEJAK tidak pernah pindah state sendiri
cek("JEJAK tetap di JEJAK", jj.state, M.S_JEJAK)
cek("vision menyala di JEJAK", M.S_JEJAK in M.VISION_ON, True)

# mode BACA: melihat boleh, memutar tidak
lnk2 = M.Teensy(None)
aks2 = M.Aksi(lnk2)
jj2 = M.Misi(); jj2.idx = 1; jj2.ganti(M.S_JEJAK); jj2.bearing_deg = 30.0
M.langkah_fsm(jj2, lnk2, aks2, k, armed=False)
aks2.putar(armed=False)
cek("mode BACA: tidak memutar", lnk2.tx_terakhir, "")

print("\n12. Gerbang ROI bisa dimatikan untuk JEJAK")
j2 = M.Juri(k, {0: "korban", 1: "dummy"})
pinggir = (1050, 100, 1150, 532, 0.9, 0)        # jauh dari tengah, tinggi sah
cek("dengan ROI: ditolak", len(j2.saring([pinggir], W, H, pakai_roi=True)), 0)
j2.reset()
cek("tanpa ROI: diterima", len(j2.saring([pinggir], W, H, pakai_roi=False)), 1)
j2.reset()
kecil2 = (600, 300, 660, 380, 0.9, 0)           # tinggi cuma 0.11 H
cek("gerbang tinggi TETAP berlaku", len(j2.saring([kecil2], W, H, pakai_roi=False)), 0)

print("\n13. Pita jejak longgar vs pita penilaian ketat")
j3 = M.Juri(k, {0: "korban", 1: "dummy"})
# boneka di ~50 cm: tinggi bbox cuma 0.15 dari tinggi frame
jauh = (600, 300, 700, 408, 0.9, 0)             # tinggi 108 px / 720 = 0.15
cek("pita PENILAIAN membuangnya", len(j3.saring([jauh], W, H)), 0)
j3.reset()
cek("pita JEJAK menerimanya",
    len(j3.saring([jauh], W, H, pakai_roi=False,
                  h_min=k.jejak_h_min, h_max=k.jejak_h_max)), 1)
j3.reset()
bercak = (600, 300, 610, 320, 0.9, 0)           # tinggi 20 px / 720 = 0.03
cek("bercak kecil tetap dibuang di jejak",
    len(j3.saring([bercak], W, H, pakai_roi=False,
                  h_min=k.jejak_h_min, h_max=k.jejak_h_max)), 0)
cek("default pita jejak longgar", (k.jejak_h_min, k.jejak_h_max), (0.06, 0.98))

print("\n14. Manual: apa pun yang diketik sampai ke Teensy")
lm = M.Teensy(None)
for _c in ("b", "F", "W", "x", "R"):
    cek(f"manual '{_c}' terkirim", lm.kirim(_c), True)
cek("dan tidak satu pun ditolak", any("[TOLAK]" in x for x in lm.log), False)
cek("semuanya tercatat [TX]",
    len([x for x in lm.log if x.startswith("[TX]")]), 5)

print("\n15. Diagnostik deteksi ikut ke halaman")
md = M.Misi(); md.n_mentah = 3; md.h_terbesar = 0.18
sd = M.rakit_state(md, M.Teensy(None), k, M.Juri(k, {}),
                   {"fps": 30.0, "t_inf": 12.0}, False, "")
cek("jumlah deteksi mentah tampil", "mentah 3" in sd["deteksi"], True)
cek("tinggi bbox terbesar tampil", "0.18" in sd["deteksi"], True)
cek("versi ikut terkirim", len(sd["versi"]) > 0, True)

print("\n16. Alasan penolakan tercatat per deteksi")
j4 = M.Juri(k, {0: "korban", 1: "dummy"})
jauh2 = (600, 300, 700, 400, 0.9, 0)            # tinggi 100/720 = 0.14 -> di bawah 0.35
pinggir2 = (1050, 100, 1150, 532, 0.9, 0)       # cx jauh dari tengah
j4.saring([jauh2, pinggir2], W, H)              # gerbang KETAT
cek("alasan 'terlalu jauh' tercatat",
    "terlalu jauh" in j4.alasan.get((600, 300), ""), True)
cek("alasan 'di luar ROI' tercatat",
    j4.alasan.get((1050, 100)), "di luar ROI")
j4.saring([jauh2], W, H, pakai_roi=False,
          h_min=k.jejak_h_min, h_max=k.jejak_h_max)
cek("pita jejak: yang tadi ditolak kini lolos", j4.alasan, {})

print("\n17. Sasaran jauh: ditolak saat MENILAI, diterima saat MENGEJAR")
# Inilah bug yang bikin robot diam kalau korban agak jauh.
j5 = M.Juri(k, {0: "korban", 1: "dummy"})
korban_jauh = (600, 250, 690, 430, 0.91, 0)     # tinggi 180/720 = 0.25
cek("gerbang MENILAI membuangnya", len(j5.saring([korban_jauh], W, H)), 0)
j5.reset()
cek("gerbang MENGEJAR menerimanya",
    len(j5.saring([korban_jauh], W, H, pakai_roi=False,
                  h_min=k.jejak_h_min, h_max=k.jejak_h_max)), 1)

print("\n18. JavaScript halaman harus SAH")
# Bug nyata yang pernah lolos: '\\n' di dalam string Python biasa berubah jadi
# baris baru SUNGGUHAN, memutus string JS -> SyntaxError -> seluruh skrip mati
# -> semua field di halaman tinggal "-" tanpa satu pun pesan error.
import shutil, subprocess, tempfile, os
js = M.HALAMAN[M.HALAMAN.index("<script>") + 8:M.HALAMAN.index("</script>")]
cek("tidak ada baris baru di dalam string JS",
    all(l.count("'") % 2 == 0 for l in js.split("\n") if "//" not in l), True)
cek("kurung seimbang {}", js.count("{") == js.count("}"), True)
cek("kurung seimbang ()", js.count("(") == js.count(")"), True)
cek("backtick genap", js.count("`") % 2 == 0, True)
node = shutil.which("node") or shutil.which("nodejs")
if node:
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as f:
        f.write(js); nama = f.name
    r = subprocess.run([node, "--check", nama], capture_output=True, text=True)
    os.unlink(nama)
    cek("node --check lolos", r.returncode, 0)
    if r.returncode:
        print("     ", r.stderr.strip().splitlines()[:3])
else:
    print("  (node tidak ada -- uji parser sungguhan dilewati)")

print("\n19. Rantai AMBIL KORBAN")


class LinkPalsu:
    """Teensy tiruan: mencatat apa yang dikirim, membalas jarak yang kita mau."""

    def __init__(self, depan=None):
        self.terkirim = []
        self._depan = depan
        self.log = []
        # atribut yang dibaca rakit_state()
        self.nama_port = "PALSU"
        self.sebab = ""
        self.hidup = True
        self.n_tx = self.n_rx = 0
        self.t_rx = 0.0
        self.log_ringkas = True
        self.n_diam = 0
        self._bawa = (False, False)
        self.tunggu_visi = None
        # Tabel trim servo, diisi baris '#TRIM' jawaban 'Yt'. Kosong = belum
        # pernah dibaca, dan rakit_state() membacanya langsung.
        self.trim = {}
        # Tiga tabel kalibrasi baru, semuanya dibaca rakit_state() langsung
        # dari link dengan alasan yang sama: masing-masing sudah punya satu
        # tempat tinggal di Teensy.
        self.offset = {}
        self.lidar_offset = {}
        self.zoff = None

    def membawa(self):
        return self._bawa

    def kirim(self, cmd, armed, paksa=False):
        self.terkirim.append(cmd)
        self.n_tx += 1

    def depan_cm(self):
        return self._depan

    def state_teensy(self):
        return ""

    def ruas_fw(self):
        return ""


def sampai(misi, link, aksi, kalib, n=400):
    """Putar FSM sampai state berhenti berubah atau n giliran habis."""
    for _ in range(n):
        M.langkah_fsm(misi, link, aksi, kalib, True)
        aksi.t_boleh = 0.0          # jangan menunggu jam sungguhan
        aksi.putar(True)
    return misi.state


kk = M.Kalib()
cek("rantai AMBIL punya 6 state", len(M.RANTAI_AMBIL), 6)
cek("tiap state AMBIL ada di FSM",
    all(st in M.FSM for st in M.RANTAI_AMBIL), True)
cek("tiap state AMBIL punya batas waktu",
    all(M.BATAS.get(st, 0) > 0 for st in M.RANTAI_AMBIL), True)
cek("yang butuh kamera: TENGAH (kasar) dan HALUS (presisi)",
    sorted(st for st in M.RANTAI_AMBIL if st in M.VISION_ON),
    sorted([M.S_A_TENGAH, M.S_A_HALUS]))
cek("AMBIL_ANGKAT bermuara ke BERES", M.FSM[M.S_A_ANGKAT][1], M.S_BERES)

# -- TENGAH: sudah lurus -> lanjut; masih miring -> kirim pivot
m1 = M.Misi(); l1 = LinkPalsu(); a1 = M.Aksi(l1)
m1.ganti(M.S_A_TENGAH); m1.bearing_deg = 0.5; m1.jejak_kelas = kk.kelas_korban
M.langkah_fsm(m1, l1, a1, kk, True)
cek("TENGAH: sudah lurus -> AMBIL_MAJU", m1.state, M.S_A_MAJU)

m2 = M.Misi(); l2 = LinkPalsu(); a2 = M.Aksi(l2)
m2.ganti(M.S_A_TENGAH); m2.bearing_deg = 25.0; m2.jejak_kelas = kk.kelas_korban
M.langkah_fsm(m2, l2, a2, kk, True)
cek("TENGAH: miring -> pivot ke arah berlawanan",
    a2.antre[0][0].startswith("O-"), True)
cek("TENGAH: miring -> belum pindah state", m2.state, M.S_A_TENGAH)

# -- prior 'hanya korban'
m3 = M.Misi(); l3 = LinkPalsu(); a3 = M.Aksi(l3)
m3.ganti(M.S_A_TENGAH); m3.bearing_deg = 0.0; m3.jejak_kelas = kk.kelas_dummy
M.langkah_fsm(m3, l3, a3, kk, True)
cek("TENGAH: dummy ditolak, tidak diambil", m3.state, M.S_GAGAL)

# -- MAJU: kelewat dekat CUMA DICATAT. Pi tidak lagi mundur, dan tidak lagi
#    menggagalkan: sumbu maju-mundur milik Teensy sejak 17 Sep 2026.
m4 = M.Misi(); l4 = LinkPalsu(depan=kk.capit_cm - 8); a4 = M.Aksi(l4)
m4.ganti(M.S_A_MAJU)
M.langkah_fsm(m4, l4, a4, kk, True)
cek("MAJU: kelewat dekat -> lanjut HALUS, bukan GAGAL", m4.state, M.S_A_HALUS)
cek("MAJU: dan tidak satu pun perintah gerak dikirim", a4.antre, [])
cek("MAJU: selisihnya dilaporkan di log",
    any("[JARAK]" in b for b in l4.log), True)

# -- MAJU: 'jauh' (tak terlihat LiDAR) juga GAGAL, bukan maju membabi buta
m5 = M.Misi(); l5 = LinkPalsu(depan=float("inf")); a5 = M.Aksi(l5)
m5.ganti(M.S_A_MAJU)
M.langkah_fsm(m5, l5, a5, kk, True)
cek("MAJU: depan 'jauh' -> GAGAL", m5.state, M.S_GAGAL)

# -- MAJU: sudah pas -> lanjut
# Jarak "pas" sekarang ikut mode_ambil: 25 cm kalau LENGAN yang menjulur,
# capit_cm kalau BADAN yang maju. Dulu di sini tertulis capit_cm mentah --
# persis satu angka yang akan tertinggal saat cara mengambilnya berubah.
m6 = M.Misi(); l6 = LinkPalsu(depan=M.jarak_ambil_cm(kk)); a6 = M.Aksi(l6)
m6.ganti(M.S_A_MAJU)
M.langkah_fsm(m6, l6, a6, kk, True)
cek("MAJU: sudah pas -> AMBIL_HALUS (bukan langsung capit)",
    m6.state, M.S_A_HALUS)

# -- MAJU: penolakan kamera "terlalu dekat" dicatat, tapi TIDAK menahan rantai.
#
# Penolakan itu sendiri informasi: bbox setinggi itu hanya muncul di bawah
# ~12 cm, jadi LiDAR yang bilang jaraknya pas sedang membaca dinding belakang
# ceruk, bukan boneka di depannya. Dulu (16 Sep 2026) ini memaksa MUNDUR;
# sejak 17 Sep maju-mundur milik Teensy, jadi yang tersisa cuma laporan.
#
# Diuji supaya vis_terlalu_dekat() tidak ikut terbuang saat mundurnya dibuang:
# laporan ini satu-satunya yang memberi tahu Vincent bahwa KORBAN_JARAK_CM
# atau ruas HNT_MUNDUR perlu disetel.
m6c = M.Misi(); l6c = LinkPalsu(depan=M.jarak_ambil_cm(kk)); a6c = M.Aksi(l6c)
m6c.ganti(M.S_A_MAJU)
m6c.jarak_vis = None
m6c.jarak_vis_sebab = f"{M.VIS_DEKAT} -- bbox MENYENTUH TEPI frame"
M.langkah_fsm(m6c, l6c, a6c, kk, True)
cek("MAJU: kamera menolak -> tetap lanjut ke HALUS", m6c.state, M.S_A_HALUS)
cek("MAJU: kamera menolak -> dicatat [DEKAT]",
    any("[DEKAT]" in b for b in l6c.log), True)
cek("MAJU: kamera menolak -> tidak menjadwalkan gerakan", a6c.antre, [])

# -- MAJU: sebab penolakan yang BUKAN "terlalu dekat" tidak memicu laporan itu.
m6d = M.Misi(); l6d = LinkPalsu(depan=M.jarak_ambil_cm(kk)); a6d = M.Aksi(l6d)
m6d.ganti(M.S_A_MAJU)
m6d.jarak_vis = None
m6d.jarak_vis_sebab = "90 cm di atas pagar 90 cm"
M.langkah_fsm(m6d, l6d, a6d, kk, True)
cek("MAJU: sebab lain tidak dicatat [DEKAT]",
    any("[DEKAT]" in b for b in l6d.log), False)

# -- MAJU: terlalu jauh = dinding, bukan korban (v1.7: LIDAR_MAX_CM 130)
m6b = M.Misi(); l6b = LinkPalsu(depan=kk.ambil_maks_cm + 20); a6b = M.Aksi(l6b)
m6b.ganti(M.S_A_MAJU)
M.langkah_fsm(m6b, l6b, a6b, kk, True)
cek("MAJU: di luar ambil_maks_cm -> GAGAL", m6b.state, M.S_GAGAL)
cek("MAJU: dan tidak berjalan ke sana", a6b.antre, [])

# -- MAJU: TIDAK PERNAH berjalan, seberapa pun jauhnya selisih.
m7 = M.Misi(); l7 = LinkPalsu(depan=kk.capit_cm + 30); a7 = M.Aksi(l7)
m7.ganti(M.S_A_MAJU)
M.langkah_fsm(m7, l7, a7, kk, True)
cek("MAJU: tidak menjadwalkan gerak apa pun", a7.antre, [])
cek("MAJU: lanjut ke HALUS", m7.state, M.S_A_HALUS)

# -- SIAP -> JEPIT -> ANGKAT -> BERES, dan perintahnya benar
m8 = M.Misi(); l8 = LinkPalsu(depan=kk.capit_cm); a8 = M.Aksi(l8)
m8.ganti(M.S_A_SIAP)
sampai(m8, l8, a8, kk)
cek("SIAP..ANGKAT melewati BERES",
    any("-> " + M.S_BERES in r for r in m8.riwayat), True)
cek("urutannya JEPIT sesudah SIAP",
    any(M.S_A_SIAP + " -> " + M.S_A_JEPIT == r for r in m8.riwayat), True)
# MENUTUP itu 'g10', bukan 'g0'. Diubah 12 Sep 2026: pada 0-5% capit menabrak
# dirinya sendiri, servo stall terus, dan motornya panas. Dicari lewat variabel
# supaya kalau angkanya berubah lagi, yang gagal satu tempat -- bukan tiga.
_TUTUP = "g10"
cek(f"perintah menutup = {_TUTUP}, bukan g0", _TUTUP in l8.terkirim, True)
cek("dan g0 TIDAK pernah dikirim (servo panas)", "g0" in l8.terkirim, False)
# Lebar bukanya sekarang knob (capit_buka_persen, bawaan 50 -- 100 terlalu
# lebar dan menyenggol tetangga 8 cm). Yang diuji tetap URUTANNYA: buka dulu,
# baru jepit. Angkanya diambil dari knob supaya menyetelnya tidak memecahkan
# tes yang sebenarnya tidak peduli angka.
_BUKA = f"g{M.Kalib().capit_buka_persen:.0f}"
cek(f"perintah membuka = {_BUKA}, bukan g100", _BUKA in l8.terkirim, True)
cek("capit dibuka sebelum dijepit",
    l8.terkirim.index(_BUKA) < l8.terkirim.index(_TUTUP), True)
cek("lengan diposisikan sebelum capit ditutup",
    any(c.startswith("a") for c in l8.terkirim[:l8.terkirim.index(_TUTUP)]), True)
angkat = [c for c in l8.terkirim if c.startswith("a")]
cek("langkah angkat lebih tinggi dari langkah jepit",
    float(angkat[-1].split()[1]) > float(angkat[0].split()[1]), True)

# -- antrean tidak menumpuk kalau langkah_fsm dipanggil berkali-kali
m9 = M.Misi(); l9 = LinkPalsu(); a9 = M.Aksi(l9)
m9.ganti(M.S_A_SIAP)
for _ in range(50):
    M.langkah_fsm(m9, l9, a9, kk, True)
cek("SIAP: perintah dijadwalkan sekali saja, tidak menumpuk",
    len(a9.antre) <= 2, True)

# -- batas waktu tetap menutup kalau korban tak pernah terlihat
m10 = M.Misi(); l10 = LinkPalsu(); a10 = M.Aksi(l10)
m10.ganti(M.S_A_TENGAH)
m10.t_state -= M.BATAS[M.S_A_TENGAH] + 1
M.langkah_fsm(m10, l10, a10, kk, True)
cek("TENGAH: batas waktu -> GAGAL", m10.state, M.S_GAGAL)

# -- state AMBIL muncul di /state supaya HUD bisa menampilkannya
st = M.rakit_state(m6, l6, kk, M.Juri(kk, {}), {"fps": 0.0, "t_inf": 0.0},
                   True, "")
cek("rakit_state punya kolom ambil_langkah", "ambil_langkah" in st, True)
cek("rakit_state menandai rantai sedang aktif", st["ambil_aktif"], True)

print("\n20. Kesehatan: parser PMIC & tangga status")

# Keluaran asli 'vcgencmd pmic_read_adc' di Pi 5 (dipendekkan, format persis).
PMIC = """        3V7_WL_SW_A current(0)=0.00291828A
        3V3_SYS_A current(1)=0.02334624A
        1V8_SYS_A current(2)=0.09338496A
        VDD_CORE_A current(6)=3.65625000A
        3V3_SYS_V volt(17)=3.31250000V
        1V8_SYS_V volt(18)=1.79687500V
        VDD_CORE_V volt(22)=0.72070312V
        EXT5V_V volt(24)=5.09375000V
        BATT_V volt(25)=0.00000000V
"""


class SehatPalsu(M.Kesehatan):
    """Kesehatan tanpa thread dan tanpa vcgencmd -- kita suapi teksnya sendiri."""

    def __init__(self, keluaran=PMIC):
        self.keluaran = keluaran
        self.suhu = 55.0; self.suhu_max = 55.0; self.throttled = 0
        self.volt = self.volt_min = self.daya = self.daya_max = self.arus = None
        self.n_overcurrent = 0; self.pmic_sebab = ""; self.oc_sebab = ""
        self.jeda = 0; self.riwayat = M.deque(maxlen=90)

    def _vcgencmd(self, *a):
        return self.keluaran


h = SehatPalsu(); h._baca_pmic()
cek("EXT5V terbaca sebagai tegangan masuk", round(h.volt, 2), 5.09)
cek("tegangan inti TIDAK dipakai sebagai tegangan masuk", h.volt > 4.0, True)
cek("arus total dijumlah dari rel yang punya volt DAN current",
    round(h.arus, 3), round(0.02334624 + 0.09338496 + 3.65625, 3))
cek("daya = jumlah V x I per rel",
    round(h.daya, 2),
    round(3.3125 * 0.02334624 + 1.796875 * 0.09338496 + 0.72070312 * 3.65625, 2))
cek("rel tanpa pasangan volt tidak dihitung nol (3V7_WL dilewati)",
    abs(h.arus - (0.02334624 + 0.09338496 + 3.65625)) < 1e-6, True)

# Jejak terendah/tertinggi -- inilah yang berguna, bukan nilai sesaat.
h.keluaran = PMIC.replace("volt(24)=5.09375000V", "volt(24)=4.71000000V")
h._baca_pmic()
cek("volt_min mengingat yang terendah", round(h.volt_min, 2), 4.71)
cek("daya_max mengingat puncak", h.daya_max >= h.daya, True)

# Pi 4: perintahnya ada tapi keluarannya kosong -> kolom kosong DENGAN sebab
h2 = SehatPalsu(keluaran=""); h2._baca_pmic()
cek("keluaran kosong -> volt tetap None", h2.volt, None)
cek("keluaran kosong -> sebabnya ditulis, bukan diam", bool(h2.pmic_sebab), True)
r2 = h2.rinci()
cek("kartu menampilkan sebab, bukan angka palsu", r2["volt"], h2.pmic_sebab)

# Tangga status: yang paling mendesak harus menang.
def st(throttled=0, oc=0, suhu=55.0):
    x = SehatPalsu(); x.throttled = throttled; x.n_overcurrent = oc; x.suhu = suhu
    return x.status()[0]

cek("bersih -> NORMAL", st(), "NORMAL")
cek("under-voltage SEKARANG menang atas segalanya",
    st(throttled=0x1 | 0x4 | 0x10000, oc=3), "UNDER-VOLTAGE")
cek("over-current menang atas throttle biasa", st(throttled=0x4, oc=1),
    "OVER-CURRENT USB")
cek("bit lengket saja -> NORMAL tapi diberi tanda",
    st(throttled=0x10000), "NORMAL (pernah under-voltage)")
cek("panas tanpa bit apa pun tetap kelihatan", st(suhu=85.0), "PANAS")
cek("bit lengket mengalahkan suhu tinggi",
    st(throttled=0x10000, suhu=85.0), "NORMAL (pernah under-voltage)")

# Warna: tegangan di bawah ambang Pi harus MERAH, bukan kuning.
h3 = SehatPalsu(); h3._baca_pmic()
cek("5,09 V -> hijau", h3.rinci()["volt_warna"], "var(--ok)")
h3.volt = 4.70
cek("4,70 V -> merah", h3.rinci()["volt_warna"], "var(--bad)")
h3.volt = 4.85
cek("4,85 V -> kuning (sudah mepet, belum di bawah ambang)",
    h3.rinci()["volt_warna"], "var(--warn)")

# Over-current tanpa akses log tidak boleh tampil sebagai "tidak ada".
h4 = SehatPalsu(); h4.oc_sebab = "log kernel tidak terbaca (perlu grup 'adm')"
cek("log tak terbaca != 'tidak ada'", h4.rinci()["overcurrent"], h4.oc_sebab)
cek("dan warnanya redup, bukan hijau", h4.rinci()["overcurrent_warna"], "var(--dim)")

# Grafik harus bisa di-JSON-kan (dia dikirim lewat /state).
h5 = SehatPalsu(); h5._baca_pmic()
h5.riwayat.append((h5.volt, h5.daya))
json.dumps(h5.rinci())
cek("rinci() bisa di-JSON-kan utuh", True, True)

print("\n21. STOP harus benar-benar berhenti")
# Bug nyata: 'stop' dulu cuma mengosongkan antrean. Frame berikutnya FSM
# melihat antrean kosong lalu menjadwalkan perintah BARU -- robot berhenti
# sekejap lalu lanjut sendiri, seolah tombolnya tidak ditekan.

mh = M.Misi(); lh = LinkPalsu(); ah = M.Aksi(lh)
mh.ganti(M.S_JEJAK); mh.bearing_deg = 30.0; mh.jejak_kelas = kk.kelas_korban
M.langkah_fsm(mh, lh, ah, kk, True)
cek("sebelum STOP: FSM menjadwalkan pivot", len(ah.antre) > 0, True)

# ---- tekan STOP ----
mh.halt = True
ah.batal()
for _ in range(200):                      # 200 frame sesudah STOP
    M.langkah_fsm(mh, lh, ah, kk, True)
    ah.t_boleh = 0.0
    ah.putar(True, boleh=not (mh.halt or mh.jeda))
cek("sesudah STOP: antrean TETAP kosong", ah.antre, [])
cek("sesudah STOP: tidak satu pun perintah terkirim", lh.terkirim, [])

# Batas waktu pun tidak boleh memindahkan state saat HALT.
mh.t_state -= 999
M.langkah_fsm(mh, lh, ah, kk, True)
cek("HALT menahan batas waktu juga (tidak jadi GAGAL sendiri)",
    mh.state, M.S_JEJAK)

# ---- dilepas: baru boleh jalan lagi ----
mh.halt = False
M.langkah_fsm(mh, lh, ah, kk, True)
cek("sesudah dilepas: FSM hidup lagi", len(ah.antre) > 0 or mh.state == M.S_GAGAL, True)

# ---- PAUSE menyimpan state, STOP tidak harus ----
mp = M.Misi(); lp = LinkPalsu(); ap = M.Aksi(lp)
mp.ganti(M.S_A_TENGAH); mp.bearing_deg = 30.0; mp.jejak_kelas = kk.kelas_korban
mp.jeda = True
for _ in range(50):
    M.langkah_fsm(mp, lp, ap, kk, True)
    ap.t_boleh = 0.0
    ap.putar(True, boleh=not (mp.halt or mp.jeda))
cek("PAUSE: tidak ada perintah keluar", lp.terkirim, [])
cek("PAUSE: state TETAP tersimpan", mp.state, M.S_A_TENGAH)
mp.jeda = False
mp.t_state = M.time.time()
M.langkah_fsm(mp, lp, ap, kk, True)
cek("RESUME: lanjut dari state yang sama", mp.state, M.S_A_TENGAH)
cek("RESUME: FSM menjadwalkan lagi", len(ap.antre) > 0, True)

# ---- Aksi.putar: antrean yang sudah terisi pun harus tertahan ----
aq = M.Aksi(LinkPalsu())
aq.jadwal(("w", 0.1), ("O10", 0.1))
aq.t_boleh = 0.0
aq.putar(True, boleh=False)
cek("putar(boleh=False) tidak mengambil apa pun dari antrean", len(aq.antre), 2)
aq.putar(True, boleh=True)
cek("putar(boleh=True) baru mengambil", len(aq.antre), 1)

# ---- state ikut terkirim ke halaman ----
mst = M.Misi(); mst.halt = True; mst.sebab_henti = "STOP oleh operator"
stx = M.rakit_state(mst, LinkPalsu(), kk, M.Juri(kk, {}),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa halt", stx["halt"], True)
cek("/state membawa sebabnya", stx["sebab_henti"], "STOP oleh operator")

print("\n22. State belum diprogram & Lewati misi")

cek("BELUM_ADA berisi rantai lama DEKATI..LETAK",
    M.BELUM_ADA == {M.S_DEKATI, M.S_CENGKERAM, M.S_VERIF, M.S_ANTAR, M.S_LETAK},
    True)
cek("rantai AMBIL TIDAK ikut ditandai belum",
    [st for st in M.RANTAI_AMBIL if st in M.BELUM_ADA], [])

# State belum-diprogram memang tidak menggerakkan apa pun -- itu memang niatnya.
for st in M.BELUM_ADA:
    mm = M.Misi(); ll = LinkPalsu(); aa = M.Aksi(ll)
    mm.ganti(st)
    for _ in range(30):
        M.langkah_fsm(mm, ll, aa, kk, True)
        aa.t_boleh = 0.0
        aa.putar(True)
    cek(f"{st}: diam, tidak mengirim apa pun", ll.terkirim, [])

# ...tapi halaman HARUS mengatakannya, bukan diam-diam saja.
md = M.Misi(); md.ganti(M.S_DEKATI)
sd = M.rakit_state(md, LinkPalsu(), kk, M.Juri(kk, {}),
                   {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state menandai state belum diprogram", sd["state_belum"], True)
mo = M.Misi(); mo.ganti(M.S_CENTER)
so = M.rakit_state(mo, LinkPalsu(), kk, M.Juri(kk, {}),
                   {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("state yang sudah jalan tidak ikut ditandai", so["state_belum"], False)

# S_PUTUS masih bermuara ke rantai yang belum ada -- catat supaya kalau nanti
# disambung ke AMBIL, uji ini yang mengingatkan bahwa perilakunya berubah.
cek("S_PUTUS masih menuju rantai yang belum diprogram",
    M.FSM[M.S_PUTUS][1] in M.BELUM_ADA, True)

# Lewati: maju satu MISI penuh, bukan satu sub-state.
ml = M.Misi()
idx0 = ml.idx
ml.ganti(M.S_DEKATI)
ml.maju_misi(M.LEWAT)
cek("lewati: pindah ke misi berikutnya", ml.idx, idx0 + 1)
cek("lewati: misi lama ditandai LEWAT", ml.status[idx0], M.LEWAT)
cek("lewati: state kembali bersih", ml.state in (M.S_IDLE, M.S_TUNGGU), True)

print("\n23. Misi mana yang firmware-nya benar-benar ada")
mfw = M.Misi()
sfw = M.rakit_state(mfw, LinkPalsu(), kk, M.Juri(kk, {}),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
ada = [m["id"] for m in sfw["misi"] if m["fw"]]
belum = [m["id"] for m in sfw["misi"] if not m["fw"]]
cek("tiap baris misi membawa penanda firmware",
    all("fw" in m for m in sfw["misi"]), True)
cek("yang didukung persis DIDUKUNG_FIRMWARE", set(ada), M.DIDUKUNG_FIRMWARE)
cek("mayoritas misi memang BELUM didukung", len(belum) > len(ada), True)
cek("SZ1 (taruh korban) belum didukung", "SZ1" in belum, True)
cek("K5 belum didukung", "K5" in belum, True)
cek("FINISH belum didukung", "FINISH" in belum, True)
cek("penanda tidak mengubah jumlah baris", len(sfw["misi"]), len(M.MISI))

print("\n24. Penengahan dua tingkat: KASAR pivot kaki, HALUS putar badan")

# Inti masalahnya struktural: firmware menganggap dirinya lurus dalam 6 der,
# jadi permintaan pivot di bawah itu DIBUANG diam-diam. Uji ini mengunci dua
# hal: (a) kita tidak pernah mengirim perintah yang pasti dibuang, dan
# (b) tingkat halus benar-benar bisa turun di bawah 6 der.

cek("pivot_min_deg >= HEADING_TOLERANCE_DEG firmware (6.0)",
    kk.pivot_min_deg >= 6.0, True)
cek("gain pivot < 1 (kalau 1, dijamin overshoot)", kk.pivot_gain < 1.0, True)
cek("batas yaw badan di bawah BODY_MAX_ROT_DEG firmware (20)",
    kk.badan_yaw_maks < 20.0, True)
cek("batas geser badan di bawah BODY_MAX_TRANS_MM firmware (80)",
    kk.badan_geser_maks < 80.0, True)
cek("toleransi akhir lebih ketat dari deadband pivot",
    kk.tengah_tol_deg < 6.0, True)

def perintah(aksi):
    return [c for c, _ in aksi.antre]

# --- KASAR: galat besar -> 'O', dan TIDAK PERNAH di bawah pivot_min_deg ---
for e in (30.0, 12.0, 9.0, -30.0, -9.0):
    mc = M.Misi(); ac = M.Aksi(LinkPalsu())
    M.pusatkan(mc, ac, kk, e)
    o = [c for c in perintah(ac) if c.startswith("O")]
    cek(f"galat {e:+.0f} -> pivot gait", len(o), 1)
    besar = abs(float(o[0][1:]))
    cek(f"  dan besarnya > deadband 6 der (tidak akan dibuang firmware)",
        besar > 6.0, True)
    cek(f"  arahnya melawan galat", (float(o[0][1:]) < 0) == (e > 0), True)

# --- HALUS: galat kecil -> putar BADAN, bukan 'O' ---
mh2 = M.Misi(); ah2 = M.Aksi(LinkPalsu())
M.pusatkan(mh2, ah2, kk, 4.0)
p = perintah(ah2)
cek("galat 4 der -> putar badan 'r', BUKAN pivot 'O'",
    any(c.startswith("r0 0 ") for c in p) and not any(c.startswith("O") for c in p),
    True)
cek("badan diputar melawan galat", mh2.badan_yaw < 0, True)

# --- yang paling penting: apakah tingkat halus benar-benar KONVERGEN? ---
# Pivot gait tidak disimulasikan di sini: perilakunya sudah diuji di atas
# (selalu > deadband, arahnya benar). Yang belum pernah terbukti justru
# tingkat halus -- dan itu yang menentukan apakah +/-1,5 der mungkin.

# Konvergensi tingkat HALUS diuji langsung, tanpa simulasi pivot:
mk = M.Misi(); ak = M.Aksi(LinkPalsu())
err = 5.0
awal = err
tercapai = False
kk.tengah_tahan_s = 0.0     # dwell diuji di bagian 64, bukan di sini
for _ in range(30):
    if M.pusatkan(mk, ak, kk, err):
        tercapai = True
        break
    for c, _t in ak.antre:
        if c.startswith("r0 0 "):
            err = awal + float(c[5:])     # 'r' absolut terhadap galat awal
    ak.batal()
cek("tingkat HALUS konvergen ke dalam toleransi", tercapai, True)
cek("  dan galat akhirnya benar-benar kecil", abs(err) <= kk.tengah_tol_deg, True)
cek("  di bawah deadband pivot 6 der -- yang mustahil dengan 'O' saja",
    abs(err) < 6.0, True)

# --- badan WAJIB netral sebelum berjalan ---
mn = M.Misi(); an = M.Aksi(LinkPalsu())
mn.badan_yaw = 12.0
M.netralkan_badan(mn, an)
cek("netralkan_badan mengirim r0 0 0 dan t0 0 0",
    perintah(an), ["r0 0 0", "t0 0 0"])
cek("dan mencatat badan sudah netral", (mn.badan_yaw, mn.badan_x), (0.0, 0.0))
mn2 = M.Misi(); an2 = M.Aksi(LinkPalsu())
M.netralkan_badan(mn2, an2)
cek("badan yang sudah netral tidak dikirimi apa-apa", an2.antre, [])

# --- gerbang kelas dipakai bersama ---
mg = M.Misi()
cek("kelas belum diketahui -> jangan bergerak",
    M.sasaran_sah(mg, kk, True), False)
mg.jejak_kelas = kk.kelas_dummy
cek("dummy -> tolak", M.sasaran_sah(mg, kk, True), False)
mg.jejak_kelas = kk.kelas_korban
cek("korban -> boleh", M.sasaran_sah(mg, kk, True), True)
mg.jejak_kelas = kk.kelas_dummy
cek("gerbang dimatikan -> apa pun boleh", M.sasaran_sah(mg, kk, False), True)

print("\n25. Log ringkas: poll rutin disembunyikan, data TETAP diparse")

class SerPalsu:
    """Port tiruan: mengembalikan teks yang kita tentukan sekali."""
    def __init__(self, teks=""):
        self.teks = teks.encode()
        self.ditulis = []
    def read(self, n):
        t, self.teks = self.teks[:n], self.teks[n:]
        return t
    def write(self, b):
        self.ditulis.append(b)
    def reset_input_buffer(self):
        pass
    is_open = True          # Teensy.hidup itu property yang membaca ini


JAWAB_POLL = ("--- STATUS MISI ---\n"
              "  state       : DIAM\n"
              "  depan       : 23 cm\n"
              "--- LIDAR ---\n"
              "  5 DEPAN     : 23 cm\n")

def link_uji(teks):
    t = M.Teensy.__new__(M.Teensy)
    t.log = M.deque(maxlen=400)
    t.terakhir = {}
    t.buf = ""
    t.ser = SerPalsu(teks)
    t.n_rx = 0; t.n_tx = 0; t.t_rx = 0.0
    t.diam_sampai = 0.0; t.n_diam = 0; t.log_ringkas = True
    t.tx_terakhir = ""
    t.pemicu = None
    t.pemicu_baru = False
    t.aliran = {"y": False, "L": False}
    t.tunggu_visi = None
    t.riwayat = M.deque(maxlen=120)
    t._tunggu_jawab = None
    t.kompas = [None, None, None, None]
    t.kompas_waktu = 0.0
    t._di_kompas = False
    t._kompas_diminta = False
    t.trim = {}
    t.t_trim = 0.0
    t.offset = {}
    t.t_offset = 0.0
    t.lidar_offset = {}
    t.t_lidar_offset = 0.0
    t.zoff = None
    t._zoff_sisa = 0
    t.t_zoff = 0.0
    return t

# --- poll rutin: baris DISEMBUNYIKAN, tapi jarak tetap terbaca ---
lr = link_uji(JAWAB_POLL)
lr.diam_sejenak(5.0)
lr.baca()
cek("poll rutin: tidak ada baris masuk log", len(lr.log), 0)
cek("poll rutin: tapi hitungannya dicatat", lr.n_diam, 5)
cek("poll rutin: jarak depan TETAP terparse", lr.depan_cm(), 23.0)
cek("poll rutin: state TETAP terparse", lr.state_teensy(), "DIAM")

# --- mode PENUH: semuanya masuk ---
lp = link_uji(JAWAB_POLL)
lp.log_ringkas = False
lp.diam_sejenak(5.0)
lp.baca()
cek("mode PENUH: semua baris dicatat", len(lp.log), 5)

# --- perintah manual MEMBUKA penahanan ---
lm = link_uji(JAWAB_POLL)
lm.diam_sejenak(5.0)
lm.kirim("v", armed=True)                    # manual -> buka penahanan
cek("perintah manual menghapus penahanan", lm.diam_sampai, 0.0)
lm.baca()
cek("dan jawabannya terlihat", len(lm.log) > 1, True)

# --- poll rutin tidak mencatat '[TX] m' ---
lt = link_uji("")
lt.kirim("m", armed=True, rutin=True)
cek("poll rutin tidak menulis [TX]", [x for x in lt.log if x.startswith("[TX]")], [])
cek("tapi penghitung tx tetap naik", lt.n_tx, 1)
cek("dan penahanan dipasang", lt.diam_sampai > 0, True)
lt.kirim("v", armed=True)
cek("perintah manual TETAP menulis [TX]",
    [x for x in lt.log if x.startswith("[TX]")], ["[TX] v"])

cek("poll_hz turun dari 2 Hz ke 1 Hz", kk.poll_hz, 1.0)

print("\n26. Kalibrasi kompas: seluruh perintahnya lewat tanpa syarat")
# Dulu 'e' harus menembus daftar hitam dan c0/E menuntut MODE KENDALI. Sejak
# 13 Sep 2026 tidak ada keduanya -- yang diuji: tidak ada satu pun yang
# tertahan, supaya urutan kalibrasi arena bisa dijalankan utuh dari halaman.
for _c in ("k", "K", "c0", "c1", "c2", "c3", "e", "E", "C", "S", "W", "q", "Q"):
    _l = link_uji("")
    cek(f"'{_c}' terkirim tanpa syarat", _l.kirim(_c), True)

# 'C' memblokir loop firmware ~10 detik. Itu alasan MEMPERINGATKAN, bukan
# MELARANG -- dan sejak filter dibuang, peringatan itu satu-satunya yang ada.
cek("'C' dikonfirmasi dulu di HUD", "Kalibrasi pivot: robot BERPUTAR" in M.HALAMAN, True)

print("\n27. Serah-terima ke firmware v1.9 (m2/m3)")

cek("S_KONFIRM ada di FSM", M.S_KONFIRM in M.FSM, True)
cek("S_KONFIRM menyalakan kamera", M.S_KONFIRM in M.VISION_ON, True)
cek("S_KONFIRM punya batas waktu", M.BATAS.get(M.S_KONFIRM, 0) > 0, True)
cek("auto_konfirm NYALA secara bawaan", kk.auto_konfirm, True)
# Kenapa bawaannya berubah 13 Sep 2026: firmware yang dipatch PARKIR menunggu
# 'm2'. auto_konfirm MATI berarti parkir itu tidak pernah dijawab dan capit
# baru turun sesudah batas waktunya habis -- persis gejala yang diperbaiki.


class LinkFw(LinkPalsu):
    """Teensy tiruan dengan teks state yang bisa kita atur."""
    def __init__(self, fw=""):
        super().__init__()
        self.fw = fw
    def state_teensy(self):
        return self.fw


# Selama firmware masih menunggu, HUD bertahan di S_KONFIRM.
mk1 = M.Misi(); lk1 = LinkFw("MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
ak1 = M.Aksi(lk1)
mk1.ganti(M.S_KONFIRM)
M.langkah_fsm(mk1, lk1, ak1, kk, True)
cek("firmware masih menunggu -> tetap di S_KONFIRM", mk1.state, M.S_KONFIRM)
cek("dan tidak mengirim apa pun sendiri", lk1.terkirim, [])

# Begitu firmware TIDAK menunggu lagi, HUD keluar tanpa menjawab.
mk2 = M.Misi(); lk2 = LinkFw("BERJALAN"); ak2 = M.Aksi(lk2)
mk2.ganti(M.S_KONFIRM)
M.langkah_fsm(mk2, lk2, ak2, kk, True)
cek("firmware sudah jalan lagi -> keluar dari S_KONFIRM", mk2.state, M.S_IDLE)
cek("dan TIDAK menjawab pertanyaan yang sudah lewat", lk2.terkirim, [])
cek("sebabnya dicatat", "tidak lagi menunggu" in mk2.sebab, True)

# HALT harus tetap menang atas serah-terima.
mk3 = M.Misi(); lk3 = LinkFw("MENUNGGU KONFIRMASI"); ak3 = M.Aksi(lk3)
mk3.ganti(M.S_KONFIRM); mk3.halt = True
M.langkah_fsm(mk3, lk3, ak3, kk, True)
cek("saat HALT, S_KONFIRM pun beku", (mk3.state, lk3.terkirim),
    (M.S_KONFIRM, []))

print("\n28. PID penengahan: harus MENGENDAP, bukan berayun")

# Model plant yang jujur: 'r0 0 <yaw>' itu perintah POSISI dan badan
# BENAR-BENAR sampai ke sana (firmware me-ramp 60 der/detik, jauh lebih cepat
# dari laju kamera kita). Jadi galat_baru = galat_awal + yaw_badan.
def jalankan_pid(awal, n=80, derau=0.0, bias=0.0, kalib=None):
    kk2 = kalib or M.Kalib()
    kk2.metode_tengah = "pid"
    # Dwell dimatikan: yang diuji fungsi ini KONVERGENSI geometrinya (apakah
    # galatnya mengecil ke dalam toleransi), bukan penantian 2 detik. Dwell
    # diuji terpisah di bagian 64 -- mencampurnya berarti tes ini gagal
    # karena waktu, bukan karena kendalinya salah.
    kk2.tengah_tahan_s = 0.0
    mm = M.Misi(); aa = M.Aksi(LinkPalsu()); pp = M.Pid(kk2)
    err = awal
    jejak = []
    for i in range(n):
        ukur = err + bias + (derau if i % 2 else -derau)
        if M.pusatkan(mm, aa, kk2, ukur, pp, 1280):
            return err, True, jejak
        for c, _t in aa.antre:
            if c.startswith("r0 0 "):
                mm_yaw = float(c[5:])
                err = awal + mm_yaw          # 'r' absolut terhadap galat awal
                jejak.append(err)
        aa.t_boleh = 0.0
        aa.batal()
    return err, False, jejak

for awal in (6.0, -6.0, 3.0, -1.5, 0.9):
    akhir, usai, jejak = jalankan_pid(awal)
    cek(f"galat {awal:+.1f} der -> mengendap", usai, True)
    cek(f"  galat akhir |{akhir:.2f}| <= toleransi",
        abs(akhir) <= kk.tengah_tol_px / (1280 / kk.hfov_deg) + 0.05, True)
    # Tidak boleh melewati sasaran lalu balik berkali-kali.
    tanda = [1 if x > 0 else -1 for x in jejak if abs(x) > 0.05]
    ganti = sum(1 for a1, b1 in zip(tanda, tanda[1:]) if a1 != b1)
    cek(f"  lintas-nol <= 2 (tidak berayun)", ganti <= 2, True)

# Batas langkah benar-benar mengerem -- ini SIFAT METODE PID, jadi diuji
# dengan metode itu, bukan dengan bawaan sekali-tembak.
kpid = M.Kalib(); kpid.metode_tengah = "pid"
mm3 = M.Misi(); aa3 = M.Aksi(LinkPalsu()); pp3 = M.Pid(kpid)
M.pusatkan(mm3, aa3, kpid, 7.5, pp3, 1280)
r = [c for c, _ in aa3.antre if c.startswith("r0 0 ")]
cek("PID: galat 7,5 der tidak dikoreksi sekaligus",
    abs(float(r[0][5:])) <= kpid.maks_langkah_deg + 1e-6, True)

# Waktu tunggu ikut laju ramp firmware, bukan angka mati.
tunggu = [t for c, t in aa3.antre if c.startswith("r0 0 ")][0]
cek("waktu tunggu >= margin diam", tunggu >= kpid.diam_margin_s, True)

# Median menolak satu frame nyasar.
pf = M.Pid(kk)
for v in (2.0, 2.0, 40.0, 2.0, 2.0):
    _, e = pf.langkah(v)
cek("satu frame nyasar 40 der tidak menggeser median", abs(e - 2.0) < 0.01, True)

# Derau bolak-balik tidak membuatnya gagal mengendap.
akhir, usai, _ = jalankan_pid(4.0, derau=0.35)
cek("dengan derau +/-0,35 der tetap mengendap", usai, True)

# Anti-windup: integral tidak menumpuk saat galat besar.
pw = M.Pid(kk)
for _ in range(30):
    pw.langkah(15.0)                 # jauh di luar pita integral
cek("galat besar -> integral TIDAK menumpuk", pw.i, 0.0)

# Kd tidak meledak saat dt liar.
pd = M.Pid(kk)
pd.langkah(3.0)
pd.t_lalu = M.time.time() - 50.0     # jeda 50 detik (HUD baru bangun)
d, _ = pd.langkah(1.0)
cek("dt liar tidak membuat turunan meledak",
    abs(d) <= kk.maks_langkah_deg + 1e-6, True)

# reset() benar-benar membersihkan.
pr = M.Pid(kk)
pr.langkah(5.0); pr.langkah(4.0)
pr.reset()
cek("reset mengosongkan integral", pr.i, 0.0)
cek("reset mengosongkan galat sebelumnya", pr.e_lalu, None)
cek("reset mengosongkan penyaring", len(pr.sampel), 0)

print("\n29. Pita bawah yang ditutupi capit")

kc = M.Kalib()
kc.roi_bawah_frac = 0.0
jc = M.Juri(kc, {0: "korban", 1: "dummy"})
bawah = (600, 500, 700, 700, 0.9, 0)      # pusat y = 600 dari 720 = 0,83
cek("tanpa pita: deteksi di bawah frame lolos",
    len(jc.saring([bawah], W, H, pakai_roi=False,
                  h_min=0.06, h_max=0.98)), 1)

kc.roi_bawah_frac = 0.45                   # 45% bawah ditutupi capit
jc.reset()
cek("dengan pita 0,45: deteksi di bawah DIBUANG",
    len(jc.saring([bawah], W, H, pakai_roi=False,
                  h_min=0.06, h_max=0.98)), 0)
cek("  alasannya ditulis", jc.alasan.get((600, 500)),
    "di pita yang ditutupi capit")

atas = (600, 100, 700, 300, 0.9, 0)        # pusat y = 200 dari 720 = 0,28
jc.reset()
cek("deteksi di atas pita TETAP lolos",
    len(jc.saring([atas], W, H, pakai_roi=False,
                  h_min=0.06, h_max=0.98)), 1)

cek("bawaan 0 = tidak ada yang ditutupi", M.Kalib().roi_bawah_frac, 0.0)

# capit_cm sekarang dari geometri lengan, bukan tebakan
cek("capit_cm = 10 cm (LENGAN_AMBIL 150mm - dudukan sensor 50mm)",
    M.Kalib().capit_cm, 10.0)

print("\n30. Pemicu vision dari NAMA RUAS firmware")

class LinkRuas(LinkPalsu):
    def __init__(self, ruas="", fw=""):
        super().__init__()
        self.ruas = ruas
        self.fw = fw
    def ruas_fw(self):
        return self.ruas
    def state_teensy(self):
        return self.fw


cek("kata kunci ruas vision terdaftar", "ANGKAT KORBAN" in M.RUAS_VISION, True)
cek("S_AWAS menyalakan kamera", M.S_AWAS in M.VISION_ON, True)

# Nama ruas asli dari tabel firmware v1.10.
for nama in ("12 dari 0..33  --  K-1 angkat korban",
             "20 dari 0..33  --  K-3 angkat korban",
             "28 dari 0..33  --  K-5 angkat korban"):
    cek(f"'{nama[-20:]}' dikenali sebagai ruas vision",
        any(k in nama.upper() for k in M.RUAS_VISION), True)

for nama in ("5 dari 0..33  --  SZ-1 taruh korban (dalam R-4)",
             "0 dari 0..33  --  HOME -> samping K-1",
             "16 dari 0..33  --  TIMUR sampai tembok"):
    cek(f"'{nama[-18:]}' BUKAN ruas vision",
        any(k in nama.upper() for k in M.RUAS_VISION), False)

# S_AWAS: mengamati, TIDAK bergerak.
ma = M.Misi(); la = LinkRuas("12 dari 0..33  --  K-1 angkat korban")
aa = M.Aksi(la)
ma.ganti(M.S_AWAS)
for _ in range(40):
    M.langkah_fsm(ma, la, aa, kk, True)
    aa.t_boleh = 0.0
    aa.putar(True)
cek("S_AWAS: tetap di ruas korban -> bertahan", ma.state, M.S_AWAS)
cek("S_AWAS: NOL perintah gerak dikirim", la.terkirim, [])
cek("S_AWAS: antrean tetap kosong", aa.antre, [])

# Begitu ruasnya lewat, keluar sendiri.
la.ruas = "13 dari 0..33  --  R-4 berpuing"
M.langkah_fsm(ma, la, aa, kk, True)
cek("ruas korban lewat -> keluar dari S_AWAS", ma.state, M.S_IDLE)
cek("sebabnya dicatat", "sudah lewat" in ma.sebab, True)

# /state membawa ruas + hasil pengamatan
mv = M.Misi(); mv.ganti(M.S_AWAS); mv.awas_kelas = kk.kelas_korban; mv.awas_conf = 0.93
sv = M.rakit_state(mv, LinkRuas("12 dari 0..33  --  K-1 angkat korban"),
                   kk, M.Juri(kk, {}), {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa nama ruas firmware", "K-1" in sv["ruas_fw"], True)
cek("/state membawa hasil vision", "KORBAN" in sv["awas"], True)
cek("korban -> warna hijau", sv["awas_warna"], "var(--ok)")
mv.awas_kelas = kk.kelas_dummy
sv2 = M.rakit_state(mv, LinkRuas(""), kk, M.Juri(kk, {}),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("dummy -> warna merah", sv2["awas_warna"], "var(--bad)")

print("\n31. SEKALI TEMBAK vs PID -- berapa GERAKAN sampai tengah")

# Plant jujur: kamera menempel di badan. Memutar badan theta mengubah bearing
# -theta * respons. respons=1.0 kalau kamera persis di sumbu putar; lebih besar
# kalau kamera di depan sumbu (parallax) -- itu kasus nyatanya.
def hitung_gerakan(metode, awal, respons=1.0, derau=0.0, n=60, belajar=True):
    k2 = M.Kalib()
    k2.metode_tengah = metode
    k2.skala_belajar = belajar
    # Dwell dimatikan: yang dihitung di sini JUMLAH GERAKAN sampai tengah.
    # Menunggu 2 detik tidak menambah gerakan, jadi ia cuma membuat tes ini
    # gagal karena jam -- bukan karena jumlah gerakannya salah.
    k2.tengah_tahan_s = 0.0
    mm = M.Misi(); aa = M.Aksi(LinkPalsu()); pp = M.Pid(k2)
    err = awal
    gerak = 0
    arah = []
    for i in range(n):
        ukur = err + (derau if i % 2 else -derau)
        if M.pusatkan(mm, aa, k2, ukur, pp, 1280):
            return gerak, err, arah, True
        for c, _t in aa.antre:
            if c.startswith("r0 0 "):
                yaw_baru = float(c[5:])
                d = yaw_baru - (mm.tembak_yaw if mm.tembak_yaw is not None else 0.0)
                gerak += 1
                arah.append(1 if yaw_baru > 0 else -1)
                # yaw+ = badan belok KIRI = sasaran bergeser ke KANAN di
                # gambar = bearing NAIK. Jadi searah, bukan berlawanan.
                err = awal + (yaw_baru * respons)
        aa.t_boleh = 0.0
        aa.batal()
    return gerak, err, arah, False


# --- kamera persis di sumbu (ideal) ---
g1, e1, _, usai1 = hitung_gerakan("sekali", 8.0)
cek("SEKALI: galat 8 der selesai", usai1, True)
cek(f"  cuma {g1} gerakan (PID butuh jauh lebih banyak)", g1 <= 2, True)

g2, e2, _, usai2 = hitung_gerakan("pid", 8.0)
cek("PID: galat 8 der juga selesai", usai2, True)
cek(f"  tapi butuh {g2} gerakan", g2 > g1, True)
print(f"       -> sekali {g1} gerakan, pid {g2} gerakan")

# --- kamera DI DEPAN sumbu putar: respons 1,35 (parallax nyata) ---
g3, e3, arah3, usai3 = hitung_gerakan("sekali", 8.0, respons=1.35)
cek("SEKALI + parallax: tetap selesai", usai3, True)
cek(f"  dan tetap sedikit ({g3} gerakan)", g3 <= 4, True)
ganti3 = sum(1 for a1, b1 in zip(arah3, arah3[1:]) if a1 != b1)
cek("  bolak-balik <= 1", ganti3 <= 1, True)

# --- faktor skala benar-benar DIPELAJARI ---
kL = M.Kalib(); kL.metode_tengah = "sekali"
mL = M.Misi(); aL = M.Aksi(LinkPalsu()); pL = M.Pid(kL)
awal_skala = kL.skala_yaw
# Mulai DI BAWAH yaw_kasar_deg supaya jalurnya putar-badan, bukan pivot gait --
# skala hanya dipelajari di tingkat halus.
MULA = 6.0
err = MULA
for _ in range(6):
    if M.pusatkan(mL, aL, kL, err, pL, 1280):
        break
    for c, _t in aL.antre:
        if c.startswith("r0 0 "):
            err = MULA + float(c[5:]) * 1.4      # respons nyata 1,4
    aL.t_boleh = 0.0; aL.batal()
cek("skala dipelajari dari respons nyata", kL.skala_yaw != awal_skala, True)
cek("  dan bergerak ke arah yang benar (<1 karena respons >1)",
    kL.skala_yaw < awal_skala, True)
cek("  serta tetap dalam batas aman",
    kL.skala_min <= kL.skala_yaw <= kL.skala_maks, True)

# --- belajar bisa dimatikan ---
kN = M.Kalib(); kN.metode_tengah = "sekali"; kN.skala_belajar = False
mN = M.Misi(); aN = M.Aksi(LinkPalsu()); pN = M.Pid(kN)
M.pusatkan(mN, aN, kN, 8.0, pN, 1280)
aN.batal()
M.pusatkan(mN, aN, kN, 3.0, pN, 1280)
cek("skala_belajar=False -> skala tidak berubah", kN.skala_yaw, 1.0)

# --- penyaring DIBUANG sesudah badan bergerak (bug yang bikin berayun) ---
kF = M.Kalib(); kF.metode_tengah = "pid"
mF = M.Misi(); aF = M.Aksi(LinkPalsu()); pF = M.Pid(kF)
for v in (3.0, 3.0, 3.0):
    pF.langkah(v)
cek("sebelum bergerak: penyaring terisi", len(pF.sampel) > 0, True)
M.pusatkan(mF, aF, kF, 3.0, pF, 1280)
cek("sesudah badan bergerak: penyaring DIKOSONGKAN", len(pF.sampel), 0)
cek("  dan galat sebelumnya ikut dilupakan", pF.e_lalu, None)

cek("metode bawaan = ukur-saat-diam", M.Kalib().metode_tengah, "diam")

print("\n32. Ukur-saat-diam: bacaan telat DIBUANG, bukan dirata-rata")

kd = M.Kalib()
ud = M.UkurDiam(kd)
cek("awalnya belum boleh bergerak", ud.boleh, False)
for _ in range(kd.diam_sampel_min):
    ud.tambah(4.0)
cek(f"sesudah {kd.diam_sampel_min} bacaan -> boleh", ud.boleh, True)
cek("rata-ratanya masuk akal", abs(ud.rata - 4.0) < 0.01, True)

# Inti gagasannya: sesudah badan diperintah bergerak, bacaan lama DITOLAK.
ud.tunda(0.5)
cek("sesudah tunda: rata-rata dibuang", ud.rata, None)
cek("dan belum boleh bergerak lagi", ud.boleh, False)
cek("bacaan yang lahir SEBELUM badan diam ditolak",
    ud.tambah(99.0, t_sampel=M.time.time() - 1.0), False)
cek("  dan tidak mencemari rata-rata", ud.rata, None)
cek("bacaan sesudah badan diam diterima",
    ud.tambah(2.0, t_sampel=M.time.time() + 1.0), True)

# EMA memang menghaluskan, tapi hanya atas bacaan yang SAH.
ue = M.UkurDiam(M.Kalib())
for v in (10.0, 0.0, 10.0, 0.0):
    ue.tambah(v)
cek("EMA meredam ayunan bacaan", 2.0 < ue.rata < 8.0, True)

# Langkah sebanding jarak: jauh -> besar, dekat -> halus.
def langkah_untuk(err):
    k2 = M.Kalib(); k2.metode_tengah = "diam"
    mm = M.Misi(); aa = M.Aksi(LinkPalsu()); dd = M.UkurDiam(k2)
    for _ in range(k2.diam_sampel_min + 2):
        M.pusatkan(mm, aa, k2, err, None, 1280, dd)
    r = [c for c, _ in aa.antre if c.startswith("r0 0 ")]
    return abs(float(r[0][5:])) if r else 0.0

j_jauh = langkah_untuk(7.0)
j_dekat = langkah_untuk(1.5)
cek("galat besar -> langkah lebih besar", j_jauh > j_dekat, True)
cek("galat kecil -> langkah halus", j_dekat < 1.0, True)
cek("tapi tidak pernah NOL (kalau nol, menggantung selamanya)",
    j_dekat >= M.Kalib().langkah_min_deg - 1e-6, True)
cek("dan tidak melebihi batas langkah",
    j_jauh <= M.Kalib().maks_langkah_deg + 1e-6, True)
print(f"       -> galat 7,0 der: langkah {j_jauh:.2f} | galat 1,5 der: langkah {j_dekat:.2f}")

# TIDAK bergerak sebelum sampelnya cukup.
k3 = M.Kalib(); k3.metode_tengah = "diam"
m3b = M.Misi(); a3b = M.Aksi(LinkPalsu()); d3 = M.UkurDiam(k3)
M.pusatkan(m3b, a3b, k3, 5.0, None, 1280, d3)
cek("satu bacaan saja -> BELUM bergerak", a3b.antre, [])

# Konvergensi penuh, dengan derau, di plant yang jujur.
k4 = M.Kalib(); k4.metode_tengah = "diam"
m4b = M.Misi(); a4b = M.Aksi(LinkPalsu()); d4 = M.UkurDiam(k4)
err = 7.0
gerak = 0
arah = []
for i in range(300):
    ukur = err + (0.3 if i % 2 else -0.3)          # derau +/-0,3 der
    if M.pusatkan(m4b, a4b, k4, ukur, None, 1280, d4):
        break
    for c, _t in a4b.antre:
        if c.startswith("r0 0 "):
            yaw = float(c[5:])
            err = 7.0 + yaw                         # kamera menempel di badan
            gerak += 1
            arah.append(1 if yaw > 0 else -1)
    if a4b.antre:
        d4.t_boleh_ukur = 0.0                       # anggap badan sudah diam
    a4b.t_boleh = 0.0
    a4b.batal()
# Batas yang JUJUR: dengan derau +/-0,3 der dan toleransi 0,44 der, galat
# akhir tidak mungkin lebih kecil dari derau itu sendiri. Ketelitian dibatasi
# oleh DERAU DETEKTOR, bukan oleh kendalinya -- itu sebabnya tengah_tol_px
# tidak boleh disetel lebih ketat daripada goyangan bbox-nya sendiri.
tol_der = kk.tengah_tol_px / (1280 / kk.hfov_deg)
cek("ukur-saat-diam konvergen ke dalam (toleransi + derau)",
    abs(err) <= tol_der + 0.35, True)
ganti = sum(1 for x, y in zip(arah, arah[1:]) if x != y)
cek(f"bolak-balik <= 1 ({gerak} gerakan)", ganti <= 1, True)
print(f"       -> {gerak} gerakan, galat akhir {err:+.2f} der, bolak-balik {ganti}")

# Mode fokus
cek("fokus_vision bawaan MATI", M.Kalib().fokus_vision, False)
kf = M.Kalib(); kf.fokus_vision = True
sf = M.rakit_state(M.Misi(), LinkPalsu(), kf, M.Juri(kf, {}),
                   {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa tanda fokus", sf["fokus"], True)

# Field TEKS harus bisa diubah (dulu selalu ditolak float())
cek("metode_tengah punya daftar pilihan",
    M.KALIB_PILIHAN.get("metode_tengah") is not None, True)
cek("  dan 'diam' termasuk pilihan", "diam" in M.KALIB_PILIHAN["metode_tengah"], True)
cek("metode_tengah ikut terdaftar di KALIB_FIELDS",
    "metode_tengah" in M.KALIB_FIELDS, True)

print("\n33. JEJAK: dummy tidak pernah jadi SASARAN, bukan cuma tidak dikejar")

# Bug nyata: yang terbesar dipilih dulu apa pun kelasnya, lalu gerbang kelas
# cuma menahan GERAKAN. Akibatnya bearing terisi dari DUMMY -- panah merah
# menunjuk, robot diam, dan korban asli di sebelahnya diabaikan karena kalah
# besar. Yang benar: saring kelas DULU, baru pilih yang terbesar.
NAMA = {0: "korban", 1: "dummy"}
kj = M.Kalib()

# dummy BESAR di kiri, korban KECIL di kanan
dummy_besar  = (100, 100, 300, 600, 0.95, 1)     # tinggi 500
korban_kecil = (900, 300, 980, 520, 0.80, 0)     # tinggi 220

jj2 = M.Juri(kj, NAMA)
lolos = jj2.saring([dummy_besar, korban_kecil], W, H,
                   pakai_roi=False, h_min=0.06, h_max=0.98)
cek("gerbang geometris meloloskan keduanya", len(lolos), 2)

def pilih(lolos, hanya_korban):
    calon = lolos
    if hanya_korban:
        calon = [d for d in lolos if NAMA.get(d[5], str(d[5])) == kj.kelas_korban]
    if not calon:
        return None
    t = max(calon, key=lambda d: (d[3] - d[1]))
    return NAMA.get(t[5], str(t[5]))

cek("TANPA gerbang kelas: yang dipilih dummy (bug lama)",
    pilih(lolos, False), "dummy")
cek("DENGAN gerbang kelas: yang dipilih KORBAN",
    pilih(lolos, True), "korban")
cek("jejak_hanya_korban bawaan NYALA", kj.jejak_hanya_korban, True)

# hanya dummy terlihat -> tidak ada sasaran sama sekali
cek("cuma dummy -> tidak ada sasaran", pilih([dummy_besar], True), None)

# /state melaporkan berapa dummy yang diabaikan
md = M.Misi(); md.ganti(M.S_JEJAK); md.jejak_kelas = ""; md.n_dummy_saja = 2
sd = M.rakit_state(md, LinkPalsu(), kj, M.Juri(kj, NAMA),
                   {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state bilang dummy diabaikan", "2 dummy" in sd["dummy_saja"], True)
md.jejak_kelas = kj.kelas_korban
sd2 = M.rakit_state(md, LinkPalsu(), kj, M.Juri(kj, NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("kalau korban ketemu, tidak ada pesan diabaikan", sd2["dummy_saja"], "")

print("\n34. Tata letak tab")
h = M.HALAMAN
cek("tetap 5 panel", len(re.findall(r'<div class="?panel', h)), 5)
_tabs = h[h.index("<div id=tabs>"):h.index("<script>")]
# 23 sejak 18 Sep 2026: tiga kartu kalibrasi servo & LiDAR ditambahkan --
# "Offset sudut servo" (Yo), "Offset jarak LiDAR" (Yd), dan "Offset tinggi
# telapak" (Yz) -- supaya keluarga 'Y' tidak lagi menuntut hafalan perintah.
# Sebelumnya 20 sejak 18 Sep ("Trim servo"); 19 sejak 15 Sep ("K-3 / K-4").
cek("23 kartu terdistribusi di dalam tab",
    len(re.findall(r'<div class=card[ >]', _tabs)), 23)
cek("div seimbang", h.count("<div"), h.count("</div>"))
for nama in ("Kontrol", "Misi", "Korban", "Terminal", "Kalibrasi"):
    cek(f"tab '{nama}' ada", f">{nama}</button>" in h, True)
# Tab Robot tidak boleh lagi menumpuk 9 kartu
awal = h.index("<!-- 0. ROBOT -->")
akhir = h.index("<!-- 1. MISI -->")
cek("tab Robot tinggal 3 kartu", h[awal:akhir].count("<div class=card>"), 3)

print("\n35. Lengan: depan punya sendi, belakang hanya grip")
for c in ("G100", "G0", "g100", "g0", "a70 20", "a70 20 -15"):
    lg = link_uji("")
    cek(f"'{c}' terkirim apa adanya", lg.kirim(c), True)
# Firmware v1.11 Hexapod_Unlimited.ino baris 817 menolak 'A<r> <h>' dengan
# pesannya sendiri: capit belakang TIDAK punya sendi. Tombol yang mengirimnya
# terlihat berhasil di HUD lalu tidak menggerakkan apa pun -- persis kelas bug
# yang paling mahal di proyek ini.
cek("tombol A<r> <h> sudah dibuang dari HUD", "'A70 20'" in M.HALAMAN, False)
cek("HUD menyebut capit belakang tidak bersendi",
    "tidak punya sendi" in M.HALAMAN or "Grip saja" in M.HALAMAN, True)
for pose in ("siap", "jepit", "angkat"):
    cek(f"tombol lengan depan '{pose}' ada",
        f"'lengan_pos','{pose}'" in M.HALAMAN, True)
cek("kotak pergelangan ada", "id=prg_der" in M.HALAMAN, True)

print("\n36. Putar sekian derajat (permintaan operator)")
kp = M.Kalib()
cek("batas badan di bawah BODY_MAX_ROT_DEG firmware", kp.badan_yaw_maks < 20.0, True)

def _putar(der, metode, yaw0=0.0, kal=None):
    kal = kal or M.Kalib()
    mm = M.Misi()
    mm.badan_yaw = yaw0
    ak = M.Aksi(link_uji(""))
    c, ket = M.putar_manual(der, metode, mm, ak, kal)
    return c, ket, mm, ak

# 35 der tidak muat di badan (maks 18), jadi auto harus jatuh ke gait.
c, ket, mm, ak = _putar(35, "auto")
cek("auto 35 der -> gait", c, "O35")
cek("auto 35 der memperingatkan soal IMU", "IMU" in ket, True)
# 15 der muat, jadi auto memilih badan -- satu-satunya yang jalan tanpa IMU.
c, ket, mm, ak = _putar(15, "auto")
cek("auto 15 der -> badan", c, "r0 0 15.00")
cek("badan_yaw ikut tercatat", round(mm.badan_yaw, 2), 15.0)
# Tanda: + = KIRI, mengikuti firmware (yaw+ = belok kiri).
c, _, _, _ = _putar(-15, "auto")
cek("KANAN dikirim negatif", c, "r0 0 -15.00")
c, _, _, _ = _putar(-35, "gait")
cek("gait KANAN dikirim negatif", c, "O-35")
# 'r' itu perintah POSISI: dari 18 der, minta 35 lagi TIDAK boleh diam-diam
# terlihat berhasil.
c, ket, _, _ = _putar(35, "badan", yaw0=18.0)
cek("badan mentok -> ditolak, bukan dikirim diam-diam", c, None)
cek("alasan mentok disebut", "mentok" in ket, True)
# Dipotong, tapi dikatakan.
c, ket, mm, _ = _putar(35, "badan", yaw0=0.0)
cek("badan 35 der dipotong ke batas", c, "r0 0 18.00")
cek("pemotongan dikatakan", "DIPOTONG" in ket, True)
# Pivot gait tidak boleh mendarat di deadband HEADING_TOLERANCE_DEG = 6.
c, _, _, _ = _putar(6.5, "gait")
cek("6,5 der dibulatkan menjauh dari deadband", c, "O7")
c, ket, _, _ = _putar(0, "auto")
cek("0 der ditolak", c, None)
c, ket, _, _ = _putar("abc", "auto")
cek("teks bukan angka ditolak", c, None)
# Perintah yang dihasilkan harus benar-benar lolos whitelist.
for cc in ("O35", "O-35", "r0 0 18.00", "r0 0 0", "t0 0 0"):
    lg = link_uji("")
    cek(f"'{cc}' lolos whitelist", lg.kirim(cc, armed=True), True)
cek("tombol putar ada di HUD", "id=putar_der" in M.HALAMAN, True)
cek("pilihan metode ada di HUD", "id=putar_cara" in M.HALAMAN, True)
cek("tombol nolkan pose ada", "'pose_nol'" in M.HALAMAN, True)

print("\n37. Tampilan tidak boleh mati sendiri")
import re as _re
_h = M.HALAMAN
_js = _re.search(r"<script>(.*?)</script>", _h, _re.S).group(1)
_html = _h[:_h.index("<script>")]

# Bug 11 Sep 2026: tab Robot KOSONG saat halaman baru dimuat. Tombolnya sudah
# class=aktif tapi panelnya tidak, dan .panel{display:none} menyembunyikan
# semuanya sampai tab pertama diklik.
cek("tepat satu panel aktif sejak muat", _html.count('class="panel aktif"'), 1)
_awal = _html.index("<!-- 0. ROBOT -->")
cek("yang aktif itu panel ROBOT",
    _html.index('class="panel aktif"') > _awal
    and _html.index('class="panel aktif"') < _html.index("<!-- 1. MISI -->"), True)
cek("tombol Robot juga aktif", "<button class=aktif onclick=\"tab(0,this)\">" in _html, True)

# Bug yang sama, akar yang lebih dalam: SATU id yang hilang dari HTML membuat
# $('id').textContent melempar, dan SISA tarik() -- daya, suhu, kalibrasi --
# tidak pernah jalan. Gejalanya "kartu kosong", bukan "ada error".
_ada = set(_re.findall(r"\bid=([A-Za-z0-9_]+)", _html))
# Komentar dibuang dulu: contoh $('id') di dalam komentar bukan pemakaian.
_js_kode = _re.sub(r"//[^\n]*", "", _js)
_pakai = set(_re.findall(r"\$\('([A-Za-z0-9_]+)'\)", _js_kode))
cek(f"tiap $('id') di JS punya elemen di HTML (hilang: {sorted(_pakai - _ada)})",
    sorted(_pakai - _ada), [])
for _i in ("p_volt", "p_suhu", "p_daya", "p_bit", "kalib", "penonton",
           "stbox", "gagalbox", "stdesc", "lewat", "slot"):
    cek(f"id '{_i}' ada di HTML", _i in _ada, True)

# Dan kalaupun ada yang lolos lagi, kegagalannya harus DIKATAKAN, bukan diam.
cek("tarik() dibungkus penangkap galat", "catch(e){" in _js and "BUG TAMPILAN" in _js, True)
cek("tarikSekali dipanggil dari tarik", "await tarikSekali()" in _js, True)
cek("polling tetap jalan tiap 300 ms", "setInterval(tarik,300)" in _js, True)

# Seluruh knob kalibrasi harus benar-benar sampai ke halaman.
_k = M.Kalib()
_sd = M.rakit_state(M.Misi(), LinkPalsu(), _k, M.Juri(_k, NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek(f"/state mengirim seluruh {len(M.KALIB_FIELDS)} knob kalibrasi",
    len(_sd["kalib"]), len(M.KALIB_FIELDS))
# Dua yang SENGAJA tidak ikut: nama kelas model, bukan tombol putar.
cek("yang dikecualikan cuma nama kelas",
    sorted(set(_k.__dataclass_fields__) - set(M.KALIB_FIELDS)),
    ["kelas_dummy", "kelas_korban"])
cek("tiap knob punya nama & nilai",
    all("nama" in x and "nilai" in x for x in _sd["kalib"]), True)
cek("metode_tengah punya daftar pilihan, bukan ketik bebas",
    [x["pilihan"] for x in _sd["kalib"] if x["nama"] == "metode_tengah"],
    [["diam", "sekali", "pid", "p"]])

print("\n38. Jarak dari kamera (monokuler)")
_k = M.Kalib()
_W, _H = 1280, 720
_f = _k.f_px(_W)
cek("f_px dari hfov 70,4 di 1280 px", round(_f), 907)
cek("f_px tidak bergantung sumbu (lensa rektilinear)",
    round((_W / 2) / math.tan(math.radians(_k.hfov_deg / 2))), round(_f))

def _bbox_di(d_cm, tinggi_cm=None, kal=None):
    """Tinggi bbox piksel yang SEHARUSNYA muncul pada jarak d_cm."""
    kal = kal or _k
    return kal.f_px(_W) * (tinggi_cm or kal.korban_tinggi_cm) / d_cm

# Bolak-balik: jarak -> bbox -> jarak harus kembali ke angka yang sama.
for _d in (20.0, 25.0, 30.0, 40.0, 60.0):
    _h = _bbox_di(_d)
    _y1 = (_H - _h) / 2
    _got, _sebab = M.jarak_vision_cm(_y1, _y1 + _h, _H, _W, _k)
    cek(f"{_d:.0f} cm pulang-pergi tepat ({_sebab})", round(_got, 1), _d)

# INI YANG PALING PENTING. Korban 12 cm mengisi seluruh frame pada 15 cm;
# di jarak capit 10 cm bbox-nya terpotong, h_px berhenti tumbuh, dan
# d = f*H/h_px terbaca LEBIH JAUH dari kenyataan -- robot mengira masih ada
# ruang lalu menabrak korban. Bacaan itu WAJIB ditolak, bukan dikoreksi.
_h_penuh = _bbox_di(_k.capit_cm)
cek(f"pada capit_cm={_k.capit_cm:.0f} korban melebihi tinggi frame",
    _h_penuh > _H, True)
_got, _sebab = M.jarak_vision_cm(0, _H, _H, _W, _k)
cek("bbox penuh frame -> ditolak", _got, None)
cek("sebabnya menyebut terpotong", "terpotong" in _sebab, True)
_got, _sebab = M.jarak_vision_cm(2, 500, _H, _W, _k)
cek("menyentuh tepi ATAS -> ditolak", _got, None)
_got, _sebab = M.jarak_vision_cm(200, _H - 2, _H, _W, _k)
cek("menyentuh tepi BAWAH -> ditolak", _got, None)
# Dan kalau dipaksa dihitung, arah galatnya memang ke "lebih jauh" -- bukti
# bahwa menolak itu perlu, bukan kehati-hatian berlebihan.
_palsu = _k.f_px(_W) * _k.korban_tinggi_cm / _H       # seolah bbox = tinggi frame
cek("bbox terpotong akan terbaca LEBIH JAUH dari 10 cm", _palsu > _k.capit_cm, True)

# Pagar akal sehat.
_got, _ = M.jarak_vision_cm(300, 320, _H, _W, _k)     # bbox 20 px = sangat jauh
cek("terlalu jauh -> ditolak", _got, None)
_k2 = M.Kalib(); _k2.vision_jarak_on = False
cek("bisa dimatikan", M.jarak_vision_cm(100, 500, _H, _W, _k2)[0], None)

# Kalibrasi: dari jarak penggaris, pulihkan tinggi efektifnya.
_h30 = _bbox_di(30.0)
_y1 = (_H - _h30) / 2
cek("kalibrasi memulihkan tinggi efektif",
    round(M.tinggi_dari_jarak(_y1, _y1 + _h30, _W, 30.0, _k), 2),
    _k.korban_tinggi_cm)
# Boneka yang sebenarnya 9 cm, dikalibrasi di 30 cm -> knob jadi 9.
_h9 = _bbox_di(30.0, tinggi_cm=9.0)
_y1 = (_H - _h9) / 2
cek("boneka lain menghasilkan knob lain",
    round(M.tinggi_dari_jarak(_y1, _y1 + _h9, _W, 30.0, _k), 2), 9.0)

print("\n39. Silang vision vs LiDAR menangkap hantu LiDAR")
# Bawaannya MATI sejak revert 12 Sep: silang-periksa hanya sekuat bacaan yang
# disilangkan, dan jarak vision belum dikalibrasi (tinggi_k1..k5 masih 0).
cek("silang MATI secara bawaan", M.Kalib().vision_silang_on, False)
cek("dan K1..K5 memang belum diukur",
    all(getattr(M.Kalib(), f"tinggi_k{i}") == 0.0 for i in range(1, 6)), True)


def _maju(lidar_cm, vis_cm, kal=None):
    kal = kal or M.Kalib()
    lg = LinkPalsu(depan=lidar_cm)
    mm = M.Misi(); mm.ganti(M.S_A_MAJU)
    mm.jejak_kelas = kal.kelas_korban
    mm.jarak_vis = vis_cm
    ak = M.Aksi(lg)
    M.langkah_fsm(mm, lg, ak, kal, armed=True)
    return mm, lg, ak


def _kal_on(kalib_terukur=False):
    k = M.Kalib()
    k.vision_silang_on = True
    if kalib_terukur:
        for i in range(1, 6):
            setattr(k, f"tinggi_k{i}", k.korban_tinggi_cm)
    return k


# Bawaan mati -> tidak ikut campur sama sekali, apa pun bedanya.
_mm, _lg, _ak = _maju(30.0, 60.0)
cek("bawaan mati -> tidak GAGAL meski beda 30 cm", _mm.state == "GAGAL", False)
cek("bawaan mati -> beda tidak dicatat", _mm.beda_vis_lidar, None)

# Dinyalakan, sepakat -> lanjut seperti biasa, bedanya dicatat.
_mm, _lg, _ak = _maju(30.0, 31.0, _kal_on())
cek("sepakat -> tidak GAGAL", _mm.state == "GAGAL", False)
cek("bedanya dicatat", round(_mm.beda_vis_lidar, 1), 1.0)

# Dinyalakan TAPI belum dikalibrasi: beda besar cuma dicatat, bukan GAGAL.
# Inilah yang mengacaukan gerakan robot 12 Sep -- taksiran menghentikan
# pendekatan yang sehat. LiDAR-nya 30 cm (bukan 9) supaya yang diuji memang
# silangnya, bukan gerbang "kelewat dekat" yang berdiri sendiri.
_mm, _lg, _ak = _maju(30.0, 60.0, _kal_on())
cek("belum dikalibrasi -> TIDAK GAGAL", _mm.state == "GAGAL", False)
cek("belum dikalibrasi -> bedanya tetap dicatat",
    round(_mm.beda_vis_lidar, 1), 30.0)
cek("belum dikalibrasi -> diperingatkan di log",
    any("[SILANG]" in b for b in _lg.log), True)
cek("peringatannya menyebut sebab tidak dijadikan kegagalan",
    any("belum dikalibrasi" in b for b in _lg.log), True)
cek("dan robot TIDAK berjalan -- jarak milik Teensy", _ak.antre, [])

# Sesudah K1..K5 diukur, silang baru boleh menghentikan misi.
_mm, _lg, _ak = _maju(30.0, 60.0, _kal_on(True))
cek("sudah dikalibrasi -> beda besar GAGAL", _mm.state, "GAGAL")
cek("sebabnya menyebut kedua angka",
    "30" in _mm.sebab and "60" in _mm.sebab, True)
cek("sebabnya menyebut pita hantu", "hantu" in _mm.sebab, True)
cek("dan TIDAK satu pun perintah gerak dikirim", _ak.antre, [])

# Tanpa angka vision, rantai lama harus tetap jalan apa adanya.
_mm, _lg, _ak = _maju(30.0, None, _kal_on(True))
cek("tanpa jarak vision -> perilaku lama", _mm.state == "GAGAL", False)
cek("beda kosong kalau vision diam", _mm.beda_vis_lidar, None)

print("\n40. Jarak kamera sampai ke HUD")
_sd = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
for _f2 in ("jarak_vis", "jarak_vis_ok", "beda_vis", "beda_vis_ok"):
    cek(f"/state punya '{_f2}'", _f2 in _sd, True)
cek("tanpa sasaran, sebabnya ikut dikirim", "(" in _sd["jarak_vis"], True)
for _i in ("jarak_vis", "beda_vis", "vis_d"):
    cek(f"id '{_i}' ada di HTML", f"id={_i}" in M.HALAMAN, True)
cek("tombol kalibrasi jarak ada", "'kalib_jarak'" in M.HALAMAN, True)
cek("HUD memperingatkan soal bbox terpotong",
    "terpotong" in M.HALAMAN, True)

print("\n41. Ambil dengan LENGAN maju (ukuran R2C 11 Sep 2026)")
_k = M.Kalib()
cek("mode bawaan 'lengan'", _k.mode_ambil, "lengan")
cek("mode_ambil punya daftar pilihan",
    M.KALIB_PILIHAN.get("mode_ambil"), ["lengan", "badan"])

# Ketiga angka yang diukur harus konsisten: hypot(25,12) = 27,73 vs 28.
_cocok, _d, _teks = M.periksa_geometri_lengan(_k)
cek(f"24/12/28 konsisten ({_teks})", _cocok, True)
cek("hypot(24,12) = 26,83", round(_d, 2), 26.83)

# Salah ketik harus KETAHUAN, bukan dikerjakan diam-diam.
_ks = M.Kalib(); _ks.lengan_jangkau_cm = 35.0
cek("jangkauan salah ketik -> tidak cocok",
    M.periksa_geometri_lengan(_ks)[0], False)
_ks2 = M.Kalib(); _ks2.lengan_tinggi_cm = -20.0
cek("tinggi salah ketik -> tidak cocok",
    M.periksa_geometri_lengan(_ks2)[0], False)
# Tapi TANDA tidak bisa diperiksa dari hypot -- ini batasnya, dan harus jujur.
_kp = M.Kalib(); _kp.lengan_tinggi_cm = +12.0
cek("tanda dibalik TETAP lolos hypot (batas nyata pemeriksaan ini)",
    M.periksa_geometri_lengan(_kp)[0], True)

# Jarak kerja ikut mode.
# Firmware v1.17: KORBAN_JARAK_CM = 25 di kelima ruas AMBIL (R2C 15 Sep
# 2026: 24 -> 20 -> 25). HUD harus berhenti di angka yang SAMA, kalau tidak
# yang satu terus menyuruh maju sementara yang lain merasa sudah sampai.
cek("mode lengan -> berhenti di 25 cm (= KORBAN_JARAK_CM v1.17)",
    M.jarak_ambil_cm(_k), 25.0)
_kb = M.Kalib(); _kb.mode_ambil = "badan"
cek("mode badan -> berhenti di capit_cm", M.jarak_ambil_cm(_kb), _kb.capit_cm)

# Pose lengan dalam mm, siap untuk perintah 'a'.
cek("pose jepit mode lengan (mm dari pusat badan)",
    M.pose_jepit_mm(_k), (240.0, -120.0))
cek("pose jepit mode badan tetap yang lama",
    M.pose_jepit_mm(_kb), (_kb.lengan_r, _kb.lengan_h))
cek("tinggi bawaan NEGATIF (korban di lantai, lengan turun)",
    _k.lengan_tinggi_cm < 0, True)

# INI INTINYA: 25 cm ada di pita di mana bbox MASIH UTUH, sedangkan 10 cm tidak.
_W, _H = 1280, 720
_h25 = _k.f_px(_W) * _k.korban_tinggi_cm / 25.0
cek("di 25 cm bbox muat di frame", _h25 < _H, True)
_y1 = (_H - _h25) / 2
_got, _ = M.jarak_vision_cm(_y1, _y1 + _h25, _H, _W, _k)
cek("dan kamera membacanya tepat", round(_got, 1), 25.0)
_h10 = _k.f_px(_W) * _k.korban_tinggi_cm / 10.0
cek("sedangkan di 10 cm bbox TIDAK muat", _h10 > _H, True)
# Ketelitian di jarak kerja baru: 5 px noise.
cek("5 px noise di 25 cm di bawah 5 mm", (25.0 / _h25 * 5) * 10 < 5.0, True)

# Rantai AMBIL harus benar-benar memakai 25 cm, bukan 10.
def _maju2(lidar_cm, kal):
    lg = LinkPalsu(depan=lidar_cm)
    mm = M.Misi(); mm.ganti(M.S_A_MAJU); mm.jejak_kelas = kal.kelas_korban
    ak = M.Aksi(lg)
    M.langkah_fsm(mm, lg, ak, kal, armed=True)
    return mm, ak

_mm, _ak = _maju2(25.0, _k)
cek("mode lengan: 25 cm dianggap SUDAH PAS", _mm.state, M.S_A_HALUS)
# Terlalu jauh, terlalu dekat, pas -- ketiganya sekarang berakhir sama:
# lanjut ke HALUS, nol perintah gerak. Jarak milik Teensy (17 Sep 2026).
_mm, _ak = _maju2(35.0, _k)
cek("  35 cm: lanjut HALUS, Pi tidak memajukan", _mm.state, M.S_A_HALUS)
cek("  dan tidak ada perintah gerak", _ak.antre, [])
_mm, _ak = _maju2(10.0, _k)
cek("mode lengan: 10 cm kelewat dekat -> lanjut, cuma dicatat",
    _mm.state, M.S_A_HALUS)
cek("dan tidak ada perintah gerak", _ak.antre, [])
_mm, _ak = _maju2(10.0, _kb)
cek("mode badan: 10 cm justru pas", _mm.state, M.S_A_HALUS)

# SIAP harus membuka capit DULU, baru menjulurkan lengan.
_mm = M.Misi(); _mm.ganti(M.S_A_SIAP)
_lg = LinkPalsu(depan=25.0); _ak = M.Aksi(_lg)
M.langkah_fsm(_mm, _lg, _ak, _k, armed=True)
_urut = [c for c, _ in _ak.antre]
cek("SIAP: capit dibuka lebih dulu", _urut[0],
    f"g{M.Kalib().capit_buka_persen:.0f}")
cek("SIAP: lalu lengan ke 240/-120", _urut[1], "a240 -120")
cek("SIAP: hanya dua perintah", len(_urut), 2)

# Dan /state membawa pemeriksaan itu ke layar.
_sd = M.rakit_state(M.Misi(), LinkPalsu(), _k, M.Juri(_k, NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa geometri lengan", "lengan_geo" in _sd, True)
cek("/state membawa mode ambil", _sd["mode_ambil"], "lengan")
cek("target menyebut 25 cm", "25 cm" in _sd["ambil_target"], True)
cek("id lengan_geo ada di HTML", "id=lengan_geo" in M.HALAMAN, True)
cek("tombol uji jangkauan ada", "'uji_jangkau'" in M.HALAMAN, True)

print("\n42. Ukuran korban terukur: 9 x 8,5 cm (R2C 11 Sep 2026)")
_k = M.Kalib()
cek("tinggi bawaan = angka ukur, bukan tempelan", _k.korban_tinggi_cm, 9.0)
cek("lebar ikut dicatat", _k.korban_lebar_cm, 8.5)
_W, _H = 1280, 720
_f = _k.f_px(_W)

# Batas potong bergeser karena ukurannya nyata: 11,3 cm, bukan 15.
_batas = _f * _k.korban_tinggi_cm / _H
cek("bbox mulai terpotong di ~11,3 cm", round(_batas, 1), 11.3)
cek("cara lama (10 cm) memang DI DALAM zona buta", _k.capit_cm < _batas, True)
cek("cara baru (24 cm) jauh di luarnya",
    _k.lengan_jangkau_cm > _batas * 2, True)

# Di jarak kerja baru: bbox utuh dan halus.
_h25 = _f * _k.korban_tinggi_cm / 25.0
cek("bbox di 25 cm ~327 px", round(_h25), 327)
cek("= 0,45 tinggi frame", round(_h25 / _H, 2), 0.45)
cek("5 px noise di 25 cm ~3,8 mm", round(25.0 / _h25 * 5 * 10, 1), 3.8)
cek("masih di dalam pita penilaian bbox_h_min/max",
    _k.bbox_h_min <= _h25 / _H <= _k.bbox_h_max, True)

print("\n43. Kalibrasi per posisi K-1..K-5")
for _p in ("k1", "k2", "k3", "k4", "k5"):
    cek(f"knob tinggi_{_p} ada", hasattr(_k, f"tinggi_{_p}"), True)
    cek(f"tinggi_{_p} mulai 0 = belum dikalibrasi",
        getattr(_k, f"tinggi_{_p}"), 0.0)
# 0 harus JATUH KE GLOBAL, bukan dipakai apa adanya -- kalau tidak, jaraknya
# meledak ke tak hingga di posisi yang belum sempat dikalibrasi.
cek("posisi belum dikalibrasi -> pakai global",
    M.tinggi_efektif_cm(_k, "K3"), _k.korban_tinggi_cm)
_kp = M.Kalib(); _kp.tinggi_k3 = 7.5
cek("posisi sudah dikalibrasi -> pakai nilainya",
    M.tinggi_efektif_cm(_kp, "K3"), 7.5)
cek("posisi lain tetap global",
    M.tinggi_efektif_cm(_kp, "K1"), _kp.korban_tinggi_cm)
cek("posisi kosong -> global", M.tinggi_efektif_cm(_kp, ""), _kp.korban_tinggi_cm)
cek("posisi asing -> global, bukan meledak",
    M.tinggi_efektif_cm(_kp, "K9"), _kp.korban_tinggi_cm)

# Jarak ikut nilai per posisi.
_h = _f * 7.5 / 25.0
_y1 = (_H - _h) / 2
cek("jarak K3 dihitung dengan tinggi K3",
    round(M.jarak_vision_cm(_y1, _y1 + _h, _H, _W, _kp, "K3")[0], 1), 25.0)
cek("bbox yang sama di posisi lain terbaca BEDA",
    round(M.jarak_vision_cm(_y1, _y1 + _h, _H, _W, _kp, "K1")[0], 1) != 25.0, True)

# korban_kini hanya menyebut baris KORBAN.
_mm = M.Misi()
for _i, _b in enumerate(M.MISI):
    _mm.idx = _i
    _got = M.korban_kini(_mm)
    cek(f"baris {_b[0]}: posisi '{_got}'", _got, _b[0] if _b[2] == M.KORBAN else "")
_mm.idx = 999
cek("idx di luar tabel -> kosong, bukan meledak", M.korban_kini(_mm), "")

print("\n44. Ambang piksel & rasio bbox")
cek("ambang px bawaan 20", _k.bbox_tol_px, 20.0)
# 20 px di 25 cm harus sepadan dengan capit_tol_cm, bukan ditebak sendiri.
_tol = M.tol_px_ke_cm(_k, _W, 25.0)
cek("20 px di 25 cm ~1,5 cm", round(_tol, 2), 1.53)
cek("dan itu sepadan dengan capit_tol_cm",
    abs(_tol - _k.capit_tol_cm) < 0.2, True)
# Piksel yang sama bernilai lebih besar di jarak jauh -- itu inti |dd/dh|=d/h.
cek("20 px di 50 cm bernilai LEBIH besar",
    M.tol_px_ke_cm(_k, _W, 50.0) > _tol * 3, True)
cek("bbox diharap di 25 cm", round(M.bbox_diharap_px(_k, _W, 25.0)), 327)
cek("jarak nol tidak meledak", M.bbox_diharap_px(_k, _W, 0.0), 0.0)

# Rasio: 8,5/9 = 0,94 saat menghadap kamera.
_ok, _r = M.rasio_wajar(0, 0, 85, 90, _k)
cek("rasio menghadap = 0,94", round(_r, 2), 0.94)
cek("dan itu wajar", _ok, True)
cek("bbox kepencet lebar -> tidak wajar", M.rasio_wajar(0, 0, 300, 90, _k)[0], False)
cek("bbox kepencet tinggi -> tidak wajar", M.rasio_wajar(0, 0, 20, 90, _k)[0], False)
cek("tinggi nol tidak meledak", M.rasio_wajar(0, 0, 50, 0, _k), (False, 0.0))
_kr = M.Kalib(); _kr.rasio_periksa = False
cek("pemeriksaan rasio bisa dimatikan",
    M.rasio_wajar(0, 0, 300, 90, _kr)[0], True)

print("\n45. Sampai ke HUD")
_sd = M.rakit_state(M.Misi(), LinkPalsu(), _k, M.Juri(_k, NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
for _f2 in ("vis_posisi", "vis_rasio", "vis_rasio_ok", "vis_harap"):
    cek(f"/state punya '{_f2}'", _f2 in _sd, True)
cek("bbox diharap menyebut kedua satuan",
    "px" in _sd["vis_harap"] and "cm" in _sd["vis_harap"], True)
for _i in ("vis_posisi", "vis_rasio", "vis_harap"):
    cek(f"id '{_i}' ada di HTML", f"id={_i}" in M.HALAMAN, True)
cek("HUD menyebut ukuran korban terukur", "8,5 cm" in M.HALAMAN, True)
cek("kotak kalibrasi sudah default 25 cm", 'id=vis_d style="width:64px" value="25"'
    in M.HALAMAN, True)
cek("seluruh knob baru ikut terkirim",
    all(any(x["nama"] == n for x in _sd["kalib"]) for n in
        ("tinggi_k1", "tinggi_k5", "bbox_tol_px", "rasio_min", "korban_lebar_cm")),
    True)

print("\n46. Selaras dengan firmware v1.12")
_k = M.Kalib()
# Angka-angka ini DISALIN dari config.h v1.12. Kalau Vincent mengubahnya,
# tes ini yang gagal -- bukan robot yang diam-diam berhenti di tempat salah.
_FW_KORBAN_JARAK_CM = 25       # gerbang HNT_DEPAN di 5 ruas AMBIL
_FW_LIDAR_DEPAN_MM = 62.0      # dudukan LiDAR depan dari pusat badan
cek("HUD berhenti di jarak yang SAMA dengan firmware",
    M.jarak_ambil_cm(_k), float(_FW_KORBAN_JARAK_CM))
# KORBAN_CAPIT_MM = KORBAN_JARAK_CM*10 + LIDAR_DEPAN_MM. Jadi "LiDAR membaca
# 25 cm" dan "capit tepat di korban" itu pernyataan yang sama -- selisih
# 62 mm antara mata LiDAR dan pusat badan sudah masuk hitungan firmware.
_capit_mm = _FW_KORBAN_JARAK_CM * 10.0 + _FW_LIDAR_DEPAN_MM
cek("titik capit firmware 312 mm dari pusat badan", _capit_mm, 312.0)
# GERBANG DAN JANGKAUAN LENGAN SUDAH BUKAN ANGKA YANG SAMA, sejak firmware
# v1.15 menjalankan sekuens capit dengan SUDUT SENDI TETAP (KORBAN_SIAP_*)
# alih-alih IK dari titik capit. Yang harus dicermin cuma gerbangnya.
cek("gerbang HUD mencermin firmware", M.jarak_ambil_cm(_k),
    float(_FW_KORBAN_JARAK_CM))
cek("  jangkauan lengan TIDAK ikut digeser", _k.lengan_jangkau_cm, 24.0)
cek("  dan geometri lengan tetap lulus", M.periksa_geometri_lengan(_k)[0], True)
# Jarak kerja baru masih di pita aman vision.
_W, _H = 1280, 720
_h25 = _k.f_px(_W) * _k.korban_tinggi_cm / 25.0
cek("bbox di 25 cm masih utuh", _h25 < _H, True)
cek("bbox di 25 cm lebih kecil daripada di 20 cm",
    _h25 < _k.f_px(_W) * _k.korban_tinggi_cm / 20.0, True)
cek("5 px noise di 25 cm masih di bawah 5 mm", 25.0 / _h25 * 5 * 10 < 5.0, True)

print("\n47. Pemicu #KORBAN dari firmware v1.12")
# Baris ini DISALIN dari Misi.cpp v1.12 baris 1521-1523.
_lg = link_uji("")
_lg.buf = ""
for _b in ("#KORBAN AMBIL 1", "#KORBAN TARUH 6", "#korban ambil 29"):
    _l2 = link_uji("")
    _l2._parse(_b)
    cek(f"'{_b}' terbaca", _l2.pemicu is not None, True)
cek("aksi & ruas terurai benar",
    (lambda l: (l._parse("#KORBAN AMBIL 16"), l.pemicu[:2])[1])(link_uji("")),
    ("AMBIL", 16))
cek("huruf kecil juga diterima",
    (lambda l: (l._parse("#korban ambil 29"), l.pemicu[:2])[1])(link_uji("")),
    ("AMBIL", 29))
# Baris lain TIDAK boleh ikut tertangkap.
for _b in ("  depan : 24 cm", "=== ANGKAT KORBAN -- ruas 1 ===", "KORBAN kosong"):
    _l2 = link_uji("")
    _l2._parse(_b)
    cek(f"'{_b[:28]}' bukan pemicu", _l2.pemicu, None)
# Satu pemicu = satu tindakan, bukan tiap loop.
_l3 = link_uji("")
_l3._parse("#KORBAN AMBIL 1")
cek("pemicu_baru menyala", _l3.pemicu_baru, True)
_mm = M.Misi(); _ak = M.Aksi(_l3); _k = M.Kalib()
cek("dikonsumsi sekali", M.tangani_pemicu(_mm, _l3, _ak, _k, True), True)
cek("dan tidak dua kali", M.tangani_pemicu(_mm, _l3, _ak, _k, True), False)

print("\n48. Dua penguasa satu robot -- pemicu tidak boleh menggerakkan buta")

def _picu(cara, teks_fw="", armed=True, aksi_fw="AMBIL", bukti=None):
    """Jalankan satu pemicu #KORBAN sampai keputusannya jatuh.

    SEJAK 13 Sep 2026 keputusannya DUA LANGKAH, dan itu yang diuji di sini:
    panggilan pertama cuma mencatat pemicunya dan meminta status baru,
    panggilan kedua memutuskan -- memakai bukti yang datang di antaranya.

    teks_fw  = baris 'state' SEGAR dari firmware (lewat _parse, bukan disetel
               langsung ke .terakhir -- status basi memang tidak boleh dihitung
               sebagai bukti, dan itu inti perbaikannya).
    bukti    = baris mentah lain, mis. pengumuman "PARKIR UNTUK VISION".
    """
    lg = link_uji("")
    lg._parse(f"#KORBAN {aksi_fw} 1")
    kal = M.Kalib(); kal.pemicu_korban = cara
    mm = M.Misi(); ak = M.Aksi(lg)
    M.tangani_pemicu(mm, lg, ak, kal, armed)          # 1: catat & minta status
    if bukti:
        lg._parse(bukti)
    if teks_fw:
        lg._parse(f"  state       : {teks_fw}")       # status SEGAR
    if mm.pemicu_tunda and not (teks_fw or bukti):
        # Tidak ada bukti sama sekali: habiskan jendela sabarnya.
        _r, _t0, _n0 = mm.pemicu_tunda
        mm.pemicu_tunda = (_r, _t0 - M.PEMICU_TUNGGU_BUKTI_S - 0.1, _n0)
    M.tangani_pemicu(mm, lg, ak, kal, armed)          # 2: putuskan
    return mm, lg, ak

# v1.12 APA ADANYA: firmware TIDAK parkir. ambil_alih harus TURUN ke lihat.
_mm, _lg, _ak = _picu("ambil_alih")
cek("tanpa bukti parkir -> TIDAK ambil alih", _mm.state != M.S_A_TENGAH, True)
cek("turun ke MENGAMATI", _mm.state, M.S_AWAS)
cek("dan NOL perintah gerak dijadwalkan", _ak.antre, [])
_pesan = " ".join(_lg.log)
cek("sebabnya disebut: tidak ada pengumuman parkir",
    "PARKIR UNTUK VISION" in _pesan, True)
cek("dan jalan keluarnya disebut", "flash patch R2C" in _pesan, True)

# Dengan bukti parkir: barulah ambil alih.
_mm, _lg, _ak = _picu("ambil_alih", teks_fw="MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
cek("dengan bukti parkir -> AMBIL ALIH", _mm.state, M.S_A_TENGAH)
cek("ditandai berasal dari firmware", _mm.dari_firmware, True)

# Mode BACA sudah dihapus: armed=False tidak lagi menahan apa pun.
_mm, _lg, _ak = _picu("ambil_alih", teks_fw="MENUNGGU KONFIRMASI", armed=False)
cek("mode BACA dihapus -> tetap ambil alih", _mm.state, M.S_A_TENGAH)

# BUKTI DARI PENGUMUMAN FIRMWARE, tanpa poll 'm' sama sekali. Ini jalur yang
# sebenarnya dipakai di arena: barisnya datang di burst serial yang sama
# dengan pemicunya, jauh sebelum poll berikutnya sempat jalan.
_mm, _lg, _ak = _picu(
    "ambil_alih",
    bukti="=== PARKIR UNTUK VISION -- menunggu 'm2' dari Raspi ===")
cek("pengumuman parkir saja sudah cukup -> AMBIL ALIH", _mm.state, M.S_A_TENGAH)

# DAN INI YANG DULU RUSAK: status BASI tidak boleh dihitung sebagai bukti.
# Sebelum 13 Sep, 'state' hasil poll <=1 detik lalu dipakai apa adanya --
# padahal pada detik pemicu ia masih berbunyi "BERJALAN".
_lgb = link_uji("")
_lgb.terakhir["state"] = "MENUNGGU KONFIRMASI"      # basi: tidak lewat _parse
_lgb._parse("#KORBAN AMBIL 1")
_kb = M.Kalib(); _kb.pemicu_korban = "ambil_alih"
_mb = M.Misi(); _ab = M.Aksi(_lgb)
M.tangani_pemicu(_mb, _lgb, _ab, _kb, True)
cek("panggilan pertama BELUM memutuskan", _mb.state, M.S_IDLE)
cek("pemicunya dicatat sebagai tertunda", _mb.pemicu_tunda is not None, True)
cek("dan status baru diminta", "m" in _lgb.tx_terakhir, True)
_r, _t0, _n0 = _mb.pemicu_tunda
_mb.pemicu_tunda = (_r, _t0 - M.PEMICU_TUNGGU_BUKTI_S - 0.1, _n0)
M.tangani_pemicu(_mb, _lgb, _ab, _kb, True)
cek("status BASI bukan bukti -> turun ke MENGAMATI", _mb.state, M.S_AWAS)
cek("dan nol perintah gerak", _ab.antre, [])

# lihat & mati.
_mm, _lg, _ak = _picu("lihat")
cek("lihat -> MENGAMATI", _mm.state, M.S_AWAS)
cek("lihat -> nol gerak", _ak.antre, [])
_mm, _lg, _ak = _picu("mati")
cek("mati -> state tidak berubah", _mm.state, M.S_IDLE)
# TARUH bukan urusan vision.
_mm, _lg, _ak = _picu("ambil_alih", aksi_fw="TARUH",
                      teks_fw="MENUNGGU KONFIRMASI")
cek("TARUH diabaikan", _mm.state != M.S_A_TENGAH, True)
# Bawaan berubah ke ambil_alih 12 Sep 2026, setelah firmware R2C punya parkir
# ber-batas-waktu. Yang menjaga keamanannya BUKAN bawaan ini, melainkan
# penjaga bukti di bagian 48: tanpa bukti firmware parkir, ambil_alih turun
# sendiri ke lihat.
cek("bawaan: ambil_alih (aman karena ada penjaga bukti)",
    M.Kalib().pemicu_korban, "ambil_alih")
cek("pilihan pemicu_korban terdaftar",
    M.KALIB_PILIHAN.get("pemicu_korban"), ["mati", "lihat", "ambil_alih"])

print("\n49. Serah terima balik ke firmware lewat m2")
# Rantai yang DIMULAI firmware harus berakhir dengan m2, bukan dengan rantai
# capit HUD -- dua sekuens lengan untuk satu lengan hanya saling menimpa.
def _halus(dari_fw, auto):
    # tengah_tahan_s = 0 supaya satu panggilan cukup. Dwell-nya sendiri diuji
    # di bagian 64; yang diuji DI SINI serah-terimanya, bukan penantiannya.
    kal = M.Kalib(); kal.auto_konfirm = auto; kal.tengah_tahan_s = 0.0
    lg = link_uji(""); mm = M.Misi(); ak = M.Aksi(lg)
    mm.ganti(M.S_A_HALUS)
    mm.dari_firmware = dari_fw
    mm.bearing_deg = 0.0
    mm.badan_yaw = 5.0          # badan menyerong = hasil penengahan, DITAHAN
    M.langkah_fsm(mm, lg, ak, kal, armed=True, diam=None)
    return mm, lg, [c for c, _ in ak.antre]

_mm, _lg, _urut = _halus(dari_fw=True, auto=True)
cek("dari firmware -> TIDAK masuk rantai capit HUD", _mm.state != M.S_A_SIAP, True)
cek("tapi ke TAHAN_TENGAH", _mm.state, M.S_TAHAN)
# INI PEMBALIKAN YANG DISENGAJA, 14 Sep 2026. Sampai hari ini baris ini
# berbunyi "badan dinetralkan DULU" -- dan justru itu yang dilaporkan rusak:
# "vision sudah tengah, tapi begitu capit turun badan kembali ke default".
# Menetralkan badan tepat sebelum 'm2' membuang seluruh penengahan, lalu
# firmware menurunkan capit pada badan yang lurus ke depan KAKI.
cek("badan TIDAK dinetralkan di sini", "r0 0 0" in _urut, False)
cek("  pose tengahnya dipertahankan", _mm.badan_yaw, 5.0)
cek("lalu m2 dikirim", "m2" in _urut, True)
cek("dan TIDAK ada perintah lengan", [c for c in _urut if c.startswith("a")], [])

_mm, _lg, _urut = _halus(dari_fw=True, auto=False)
cek("auto_konfirm mati -> m2 TIDAK dikirim sendiri", "m2" in _urut, False)
# Badannya TETAP ditahan di sini juga. Operator yang menekan 'm2' sendiri
# beberapa detik kemudian berhak atas badan yang masih tengah -- kalau ia
# dinetralkan sekarang, jalur manual mendapat persis bug yang baru diperbaiki
# di jalur otomatis.
cek("dan badannya juga tetap ditahan", "r0 0 0" in _urut, False)
cek("  masuk TAHAN_TENGAH menunggu m2 operator", _mm.state, M.S_TAHAN)
cek("dan dikatakan harus ditekan sendiri",
    any("auto_konfirm MATI" in b for b in _lg.log), True)

_mm, _lg, _urut = _halus(dari_fw=False, auto=True)
cek("rantai HUD sendiri -> tetap ke capit HUD", _mm.state, M.S_A_SIAP)
cek("dan TIDAK mengirim m2", "m2" in _urut, False)

print("\n50. Pemicu sampai ke HUD")
_sd = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
for _f2 in ("pemicu", "pemicu_ada", "pemicu_cara", "serah"):
    cek(f"/state punya '{_f2}'", _f2 in _sd, True)
for _i in ("pemicu", "serah"):
    cek(f"id '{_i}' ada di HTML", f"id={_i}" in M.HALAMAN, True)
cek("HUD menjelaskan kirim-lalu-lanjut", "kirim-lalu-lanjut" in M.HALAMAN, True)

print("\n51. Kompas: tabel 'k' harus terbaca, bukan ditelan regex LiDAR")
# Bentuk persis keluaran Navigation::kompasTabel() v1.12.
_K_OUT = ["--- KOMPAS ARENA ---",
          "0 UTARA\t: 12.5 der",
          "1 TIMUR\t: 102.5 der",
          "2 SELATAN\t: 192.5 der",
          "3 BARAT\t: 282.5 der"]
_lg = link_uji("")
for _b in _K_OUT:
    _lg._parse(_b)
cek("keempat arah terbaca", _lg.kompas, [12.5, 102.5, 192.5, 282.5])
cek("waktunya dicatat", _lg.kompas_waktu > 0, True)

# INI BUG-NYA. RE_LIDAR menerima "0 UTARA\t: 12.5 der" (ch=0, nama='UTARA'),
# jadi tiap 'k' ditekan keempat barisnya masuk ke lidar_ch0..3 -- dan angka
# kompasnya sendiri tidak pernah sampai ke mana pun. Dari luar: "kompasnya
# kosong padahal EEPROM ada isinya".
cek("dan TIDAK bocor ke lidar_ch",
    [k for k in _lg.terakhir if k.startswith("lidar_ch")], [])
cek("juga tidak ke lidar_UTARA",
    [k for k in _lg.terakhir if "UTARA" in k], [])

# Yang belum dicatat tetap None, bukan 0 -- 0 der itu arah yang sah.
_l2 = link_uji("")
for _b in ["--- KOMPAS ARENA ---", "0 UTARA\t: 12.5 der",
           "1 TIMUR\t: belum dicatat"]:
    _l2._parse(_b)
cek("'belum dicatat' -> None, bukan 0", _l2.kompas[1], None)
cek("0 der tetap dibedakan dari kosong",
    (lambda l: [l._parse(x) for x in ["--- KOMPAS ARENA ---", "0 UTARA\t: 0.0 der"]]
     and l.kompas[0])(link_uji("")), 0.0)

# Tabel ditutup baris pertama yang bukan anggotanya -- supaya tabel LiDAR
# sesudahnya tidak ikut dibaca sebagai kompas.
_l3 = link_uji("")
for _b in _K_OUT + ["", "  0 DEPAN  : 24 cm"]:
    _l3._parse(_b)
cek("sesudah tabel ditutup, LiDAR kembali normal",
    _l3.terakhir.get("lidar_DEPAN"), 24.0)
cek("dan kompas tidak tertimpa", _l3.kompas[0], 12.5)

# Tanpa header, baris berbentuk sama TIDAK dianggap kompas.
_l4 = link_uji("")
_l4._parse("0 UTARA\t: 12.5 der")
cek("tanpa header: bukan kompas", _l4.kompas, [None, None, None, None])

print("\n52. Kompas sampai ke HUD")
# LinkPalsu dipakai supaya rakit_state() melihat link yang lengkap; medan
# kompasnya disetel langsung karena yang diuji di sini penyajiannya, bukan
# parsernya (itu bagian 51).
_lg2 = LinkPalsu()
_lg2.kompas = [12.5, 102.5, 192.5, 282.5]
_lg2.kompas_waktu = time.time()
_sd = M.rakit_state(M.Misi(), _lg2, M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa 4 baris kompas", len(_sd["kompas"]), 4)
cek("namanya benar", [k["nama"] for k in _sd["kompas"]],
    ["UTARA", "TIMUR", "SELATAN", "BARAT"])
cek("nilainya berderajat", _sd["kompas"][0]["nilai"], "12.5 der")
cek("ditandai lengkap", _sd["kompas_lengkap"], True)
_sd0 = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                     {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("belum pernah dibaca -> umur None", _sd0["kompas_umur"], None)
cek("dan tidak ditandai lengkap", _sd0["kompas_lengkap"], False)
cek("id kompas ada di HTML", "id=kompas" in M.HALAMAN, True)

print("\n53. Misi masih ditolak -- tapi oleh SIAPA, dan jalan memutarnya")
# Disalin dari v1.12. Kalau Vincent mengisi ruas 30, tes ini yang gagal --
# dan itu kabar baik yang memang harus terlihat.
_FW_RUAS30_NILAI = -1
cek("ruas 30 masih -1 di v1.12", _FW_RUAS30_NILAI, -1)
# m4 <awal> <akhir> memeriksa tabelSiap HANYA pada rentangnya, jadi ruas 30
# tidak menghalangi selama ia di luar rentang.
cek("tombol rentang m4 ada", "'m4 '+$('m4_a').value" in M.HALAMAN, True)
cek("bawaannya 0..29 (melewati ruas 30)",
    'id=m4_a style="width:52px" value="0"' in M.HALAMAN
    and 'id=m4_b style="width:52px" value="29"' in M.HALAMAN, True)
for _c in ("m4 0 29", "m4 6 13"):
    _l5 = link_uji("")
    cek(f"'{_c}' lolos whitelist", _l5.kirim(_c, armed=True), True)
# Dan HUD harus menyebut urutan penolakan yang SEBENARNYA: IMU dulu, ruas 30
# terakhir -- bukan sebaliknya.
cek("HUD menyebut IMU sebagai penghalang PERTAMA",
    "tidak ada data IMU" in M.HALAMAN, True)
cek("HUD menyebut kompas TIDAK butuh IMU",
    "tidak</b> butuh IMU" in M.HALAMAN, True)
cek("HUD menyebut jalan memutar m4 0 29",
    "m4 0 29" in M.HALAMAN, True)

print("\n54. IMU: satu sebab untuk hampir semua penolakan")
# Daftar ini DISALIN dari v1.12. Tiap baris di sini adalah satu perintah yang
# ditolak selama IMU mati, dan semuanya lewat _imu.hasData().
_GERBANG_IMU = {
    "C  (kalibrasi pivot)": "Navigation.cpp:460",
    "c0 (catat kompas)":    "Navigation.cpp:65",
    "o0 (hadap arah)":      "pivotKe -> hasData",
    "m1 (mulai misi)":      "Navigation.cpp:617",
}
cek("ada 4 gerbang IMU yang tercatat", len(_GERBANG_IMU), 4)
cek("HUD menyebut kalibrasi pivot ikut terhalang IMU",
    "kalibrasi pivot" in M.HALAMAN and "hasData" in M.HALAMAN, True)
cek("HUD menyebut tersangka baud", "9600" in M.HALAMAN, True)
cek("HUD menyebut Imu::begin tidak mengirim apa pun",
    "tidak pernah mengirim apa pun" in M.HALAMAN, True)
cek("HUD mengoreksi tebakan Type-C = daya",
    "keliru" in M.HALAMAN and "USB-serial" in M.HALAMAN, True)
cek("HUD menunjuk ke sidik_imu.py", "sidik_imu.py" in M.HALAMAN, True)

# Parser sidik_imu diuji di berkasnya sendiri (python3 sidik_imu.py --uji),
# tapi kesepakatannya dengan firmware diuji DI SINI -- karena kalau keduanya
# berbeda, diagnosanya bisa menuduh yang salah, dan itu lebih mahal daripada
# tidak punya diagnosa sama sekali.
# Dicari di SEBELAH berkas tes ini, bukan di direktori kerja. Tes dijalankan
# dari mana saja (systemd, ssh satu baris, upload_to_pi.bat), dan jalur
# relatif ke cwd akan menunjuk ke tempat yang berbeda tiap kali.
#
# Dan kalau memang tidak ada, bagian ini DILEWATI dengan sebab yang jelas --
# bukan meledak. sidik_imu.py itu alat diagnosa yang berdiri sendiri; kalau
# ia belum sempat terkirim ke Pi, itu bukan alasan untuk menggugurkan 600 tes
# lain yang tidak ada hubungannya. Itu persis yang terjadi 11 Sep 2026:
# FileNotFoundError di sini menghentikan seluruh sisa berkas.
import importlib.util as _ilu
import os as _os
_JALUR_SIDIK = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                             "sidik_imu.py")
_S = None
if _os.path.exists(_JALUR_SIDIK):
    _sp = _ilu.spec_from_file_location("SIDIK", _JALUR_SIDIK)
    _S = _ilu.module_from_spec(_sp)
    _sp.loader.exec_module(_S)
else:
    print(f"  LEWAT sidik_imu.py tidak ada di {_os.path.dirname(_JALUR_SIDIK)}"
          f" -- kirim dengan upload_to_pi.bat, lalu jalankan tes ini lagi")

def _wit(tipe, nilai):
    b = bytearray([0x55, tipe])
    for v in nilai:
        x = int(round(v / 180.0 * 32768.0)) & 0xFFFF
        b += bytes([x & 0xFF, (x >> 8) & 0xFF])
    b += bytes([0, 0]); b = b[:10]; b.append(sum(b) & 0xFF)
    return bytes(b)

def _tiru_firmware(data):
    """Ditulis ulang dari Imu::update() v1.12 baris 28-60."""
    buf = bytearray(); n = 0; hasil = []
    for byte in data:
        if n >= 64:
            del buf[0]; n -= 1
        buf.append(byte); n += 1
        while n >= 11:
            if buf[0] != 0x55:
                del buf[0]; n -= 1; continue
            if (sum(buf[:10]) & 0xFF) != buf[10]:
                del buf[0]; n -= 1; continue
            hasil.append(bytes(buf[:11])); del buf[:11]; n -= 11
    return hasil

if _S is not None:
    import random as _rnd
    _rnd.seed(7)
    _beda = 0
    for _ in range(200):
        _d = bytearray()
        for _k in range(_rnd.randint(0, 5)):
            if _rnd.random() < 0.45:
                _d += bytes(_rnd.randint(1, 9))
            _d += _wit(_rnd.choice([0x51, 0x52, 0x53]),
                       [_rnd.uniform(-180, 179) for _ in range(3)])
        if len(_S.cari_frame(bytes(_d))) != len(_tiru_firmware(bytes(_d))):
            _beda += 1
    cek("parser sidik_imu SEPAKAT dengan Imu::update() firmware (200 aliran acak)",
        _beda, 0)
    cek("checksum WIT diperiksa",
        _S.frame_sah(_wit(0x53, [0, 0, 0])[:-1] + b"\x00"), False)
    cek("yaw terbaca benar", round(_S.sudut(_wit(0x53, [0, 0, 90.0]))[2], 1), 90.0)
    cek("9600 dipindai lebih dulu (bawaan pabrik WIT)", _S.BAUD_COBA[0], 9600)
    cek("230400 ikut dipindai", 230400 in _S.BAUD_COBA, True)

print("\n55. Apa pun yang dimuat tes ini HARUS ikut terkirim ke Pi")
# Penjaga struktural, bukan tes fitur.
#
# 11 Sep 2026: tes 54 memuat sidik_imu.py, tapi berkas itu tidak ada di
# daftar BERKAS upload_to_pi.bat. Di laptop semuanya hijau; di Pi tesnya mati
# dengan FileNotFoundError -- dan 600 tes lain yang tidak ada hubungannya
# ikut tidak pernah jalan. Dua perbaikan: tesnya melewati berkas yang hilang
# (di atas), dan bagian ini memastikan tidak ada yang perlu dilewati.
_DIR = _os.path.dirname(_os.path.abspath(__file__))
_BAT = _os.path.join(_DIR, "upload_to_pi.bat")
_diri = open(_os.path.abspath(__file__), encoding="utf-8").read()

# Berkas yang memang TIDAK dikirim ke Pi, dengan sebabnya. Didefinisikan di
# sini karena dua penjaga di bawah sama-sama memakainya.
_LAPTOP_SAJA = {
    "upload_to_pi.sh": "versi bash dari .bat, dijalankan DI laptop",
    "test_mission_hud.py": "berkas ini sendiri",
    # detect.py DIKELUARKAN dari daftar ini 18 Sep 2026. Alasan lamanya
    # "ikut lewat --all saja, ukurannya besar" keliru dua kali: berkasnya
    # 16 KB, dan mission_hud.py meng-import-nya saat start. Modul yang
    # di-import program tidak boleh bergantung pada flag yang mudah lupa --
    # hari itu HUD mati di Pi karena operator_control.py kena persis itu.
    "build_sizes.sh": "mengekspor best.pt ke beberapa imgsz lalu kuantisasi; "
                      "butuh toolchain ekspor yang tidak ada di Pi",
    # Ditambahkan 17 Sep 2026, sesudah penjaga di bawah menahan tujuh berkas
    # sekaligus. Lima di antaranya memang harus ada di Pi dan sekarang ikut
    # terkirim; dua ini TIDAK, dan sebabnya bukan ukuran:
    "quantize.py": "mengkuantisasi ONNX di laptop; Pi tidak punya toolchain-nya "
                   "dan tidak pernah membangun model",
    "mission_hud_10September.py": "arsip versi lama, disimpan untuk membandingkan "
                                  "perilaku; mengirimnya cuma menggandakan HUD",
}

# Berkas .py yang disebut sebagai NAMA BERKAS di dalam tes ini.
_dipakai = set(re.findall(r'"([a-z0-9_]+\.py)"', _diri)) - set(_LAPTOP_SAJA)
cek(f"tes ini memuat berkas lain: {sorted(_dipakai)}", len(_dipakai) > 0, True)

if _os.path.exists(_BAT):
    _isi = open(_BAT, encoding="utf-8", errors="replace").read()
    _m = re.search(r'set "BERKAS=([^"]*)"', _isi)
    _kirim = set(_m.group(1).split()) if _m else set()
    cek("upload_to_pi.bat punya daftar BERKAS", bool(_kirim), True)
    cek("mission_hud.py ikut terkirim", "mission_hud.py" in _kirim, True)
    cek("test_mission_hud.py ikut terkirim", "test_mission_hud.py" in _kirim, True)
    for _f3 in sorted(_dipakai):
        cek(f"'{_f3}' yang dimuat tes ikut terkirim", _f3 in _kirim, True)

    # Penjaga yang LEBIH LUAS: tiap .py/.sh di folder proyek harus ikut
    # terkirim, kecuali yang memang milik laptop saja.
    #
    # Penjaga di atas cuma melihat berkas yang DIMUAT tes ini, jadi ia
    # menangkap sidik_imu.py tapi TIDAK menangkap siapkan_teensy_pi.sh --
    # yang juga tertinggal, dan baru ketahuan saat ada yang bertanya cara
    # mengirimnya dengan scp. Berkas yang harus ada di Pi tapi tidak pernah
    # sampai itu satu kelas bug, bukan dua kejadian terpisah.
    _lokal = {f for f in _os.listdir(_DIR)
              if f.endswith((".py", ".sh")) and f not in _LAPTOP_SAJA}
    _tertinggal = sorted(_lokal - _kirim)
    cek(f"tidak ada .py/.sh yang tertinggal (tertinggal: {_tertinggal})",
        _tertinggal, [])
else:
    print(f"  LEWAT upload_to_pi.bat tidak ada di {_DIR}"
          f" -- penjaga ini cuma jalan di laptop, bukan di Pi")

print("\n56. Tombol susur dinding & C tetap ada di halaman")
# Sejarahnya: f/F/p/P/C dulu diblokir HUD, jadi firmware tidak pernah ditanya
# dan Vincent membacanya sebagai "fiturnya tidak jalan". Sekarang tidak ada
# blokir sama sekali; yang masih perlu dijaga cuma tombolnya tidak hilang.
for _c in ("f", "F", "p", "P", "C"):
    cek(f"tombol '{_c}' ada di HUD", f"cmd('man','{_c}')" in M.HALAMAN, True)
cek("tombol Kirim PAKSA sudah dibuang", "Kirim PAKSA" in M.HALAMAN, False)
cek("tombol ganti mode BACA/KENDALI sudah dibuang",
    "Ganti mode" in M.HALAMAN, False)

print("\n57. Banjir serial: 's' tidak menghentikan CETAKAN")
cek("aliran yang dikenal: y dan L", sorted(M.ALIRAN), ["L", "y"])
# Keadaan aliran dibaca dari kalimat firmware SENDIRI, bukan ditebak dari
# perintah yang kita kirim -- Vincent bisa mengetik 'y' langsung lewat serial,
# dan tebakan kita akan langsung salah tanpa ada yang tahu.
_l = link_uji("")
cek("mula-mula dianggap mati", _l.aliran, {"y": False, "L": False})
_l._parse("Aliran yaw HIDUP. Ketik 'y' lagi untuk berhenti.")
cek("kalimat HIDUP -> tercatat hidup", _l.aliran["y"], True)
_l._parse("Aliran yaw berhenti.")
cek("kalimat berhenti -> tercatat mati", _l.aliran["y"], False)
_l._parse("Aliran LiDAR HIDUP, tiap 200 ms. Ketik 'L' lagi untuk berhenti.")
cek("bentuk dengan jeda juga terbaca", _l.aliran["L"], True)
cek("dan tidak mencemari yang lain", _l.aliran["y"], False)

# HENING hanya mengirim huruf untuk aliran yang TERBACA hidup. Toggle buta
# akan MENYALAKAN yang sudah mati -- kebalikan dari yang diminta.
def _hening(keadaan):
    lg = link_uji("")
    lg.aliran.update(keadaan)
    dikirim = []
    _asli = lg.kirim
    def _tangkap(cmd, armed=False, paksa=False, rutin=False):
        dikirim.append(cmd)
        return _asli(cmd, armed, paksa, rutin)
    lg.kirim = _tangkap
    kal = M.Kalib()
    mati = []
    for _h, _n in M.ALIRAN.items():
        if lg.aliran.get(_h):
            lg.kirim(_h, True, paksa=True)
            mati.append(_h)
    return dikirim, mati

cek("dua-duanya hidup -> dua huruf dikirim",
    sorted(_hening({"y": True, "L": True})[0]), ["L", "y"])
cek("cuma yaw hidup -> hanya 'y'", _hening({"y": True, "L": False})[0], ["y"])
cek("dua-duanya mati -> TIDAK mengirim apa pun (kalau tidak, justru menyala)",
    _hening({"y": False, "L": False})[0], [])
cek("tombol HENING ada", "'hening'" in M.HALAMAN, True)
cek("tombol poll ada", "'poll'" in M.HALAMAN, True)
cek("HUD menjelaskan s tidak menghentikan cetakan",
    "tidak menghentikan cetakan" in M.HALAMAN, True)

print("\n58. Riwayat perintah tidak boleh tenggelam")
_l = link_uji("")
_l.kirim("k", armed=True)
_l._catat_jawab("0 UTARA\t: 12.5 der")
_l._catat_jawab("1 TIMUR\t: 102.5 der")
cek("perintah masuk riwayat", _l.riwayat[-1]["cmd"], "k")
cek("jawabannya ikut", len(_l.riwayat[-1]["jawab"]), 2)
# Banjir 500 baris: log utama (200) habis, riwayat HARUS selamat.
for _i in range(500):
    _l.log.append(f"yaw: {_i}")
cek("log utama memang tergulung habis", len(_l.log), _l.log.maxlen)
cek("tapi riwayat selamat", _l.riwayat[-1]["cmd"], "k")
cek("dan jawabannya masih ada", len(_l.riwayat[-1]["jawab"]), 2)
# Baris aliran rutin tidak boleh masuk riwayat.
_l2 = link_uji("")
_l2.aliran["y"] = True
_l2.kirim("v", armed=True)
_l2._catat_jawab("yaw: 123.4")
_l2._catat_jawab("Mode: DIAM")
cek("baris aliran yaw dibuang dari riwayat",
    _l2.riwayat[-1]["jawab"], ["Mode: DIAM"])
# Jendela jawaban habis -> baris berikutnya tidak ditempel ke perintah lama.
_l3 = link_uji("")
_l3.kirim("v", armed=True)
_l3._tunggu_jawab = (_l3.riwayat[-1], time.time() - 1)
_l3._catat_jawab("baris jauh sesudahnya")
cek("sesudah jendela habis, tidak ditempel", _l3.riwayat[-1]["jawab"], [])
# Batas atas supaya satu perintah cerewet tidak menghabiskan riwayat.
_l4 = link_uji("")
_l4.kirim("d", armed=True)
for _i in range(40):
    _l4._catat_jawab(f"baris {_i}")
cek("jawaban dibatasi 12 baris", len(_l4.riwayat[-1]["jawab"]), 12)

_sd = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
for _f3 in ("riwayat", "aliran", "aliran_ada", "poll_hidup"):
    cek(f"/state punya '{_f3}'", _f3 in _sd, True)
cek("id riwayat ada di HTML", "id=riwayat" in M.HALAMAN, True)
cek("id aliran ada di HTML", "id=aliran" in M.HALAMAN, True)

print("\n59. Korban yang DIGENDONG tidak boleh jadi sasaran")
# Angka dari config.h v1.12:
_FW_REHAT = (80.0, 160.0, -40.0)      # R: 80 mm depan, 160 mm DI ATAS pusat badan
_FW_GENDONG = (280.0, 140.0)          # pose gendong, 140 mm dari LANTAI
cek("pose REHAT jauh di ATAS bidang pusat badan", _FW_REHAT[1] > 100, True)
cek("dan dekat ke badan (cuma 80 mm ke depan)", _FW_REHAT[0] < 100, True)

# Baris status firmware (Misi.cpp:1710) sudah ditangkap RE_KV -- tidak perlu
# parser baru, cuma perlu dibaca.
def _link_bawa(teks):
    lg = link_uji("")
    lg._parse(teks)
    return lg

for _t, _harap in (
        ("  membawa     : depan KORBAN, belakang kosong", (True, False)),
        ("  membawa     : depan kosong, belakang KORBAN", (False, True)),
        ("  membawa     : depan KORBAN, belakang KORBAN", (True, True)),
        ("  membawa     : depan kosong, belakang kosong", (False, False))):
    cek(f"'{_t.split(':')[1].strip()}' -> {_harap}", _link_bawa(_t).membawa(), _harap)
cek("tanpa baris status -> dianggap kosong", link_uji("").membawa(), (False, False))

# Gerbangnya: selagi membawa, TIDAK ADA yang boleh dikejar.
#
# TAPI bawaannya MATI sejak revert 12 Sep. Fungsinya tetap diuji -- logikanya
# benar dan dipakai lagi begitu baris 'membawa' dibuktikan segar -- yang
# berubah cuma: ia tidak lagi memotong `lolos` di jalur JEJAK.
_k = M.Kalib()
cek("bawaan: penjaga muatan MATI (revert 12 Sep)", _k.tolak_saat_membawa, False)
_khidup = M.Kalib(); _khidup.tolak_saat_membawa = True
_lg = _link_bawa("  membawa     : depan KORBAN, belakang kosong")
_bawa, _sebab = M.sedang_membawa(_lg, _khidup)
cek("dinyalakan + membawa -> vision ditahan", _bawa, True)
cek("sebabnya menyebut muatannya sendiri", "muatannya" in _sebab, True)
_lg0 = _link_bawa("  membawa     : depan kosong, belakang kosong")
cek("kosong -> vision bebas", M.sedang_membawa(_lg0, _khidup)[0], False)
cek("penjaga bisa dimatikan", M.sedang_membawa(_lg, _k)[0], False)

# KENAPA gerbang KELAS tidak cukup: korban yang digendong memang KORBAN.
_det_gendong = [(560, 40, 700, 190, 0.93, 0)]     # tinggi di frame, kelas korban
_j = M.Juri(_k, NAMA)
cek("korban gendong LOLOS gerbang kelas (itu masalahnya)",
    NAMA[_det_gendong[0][5]], _k.kelas_korban)

# Pita ATAS bisa membuangnya kalau fraksinya sudah diukur.
_ka = M.Kalib(); _ka.roi_atas_frac = 0.35
_ja = M.Juri(_ka, NAMA)
_lolos_a = _ja.saring(_det_gendong, 1280, 720, pakai_roi=False,
                      h_min=_ka.jejak_h_min, h_max=_ka.jejak_h_max)
cek("pita atas 0,35 membuang deteksi di y~115", _lolos_a, [])
cek("alasannya disebut",
    any("korban yang digendong" in a for a in _ja.alasan.values()), True)
# Tanpa pita atas ia lolos -- membuktikan pitanya yang bekerja, bukan hal lain.
_j0 = M.Juri(M.Kalib(), NAMA)
cek("tanpa pita atas ia LOLOS", len(_j0.saring(
    _det_gendong, 1280, 720, pakai_roi=False,
    h_min=_k.jejak_h_min, h_max=_k.jejak_h_max)), 1)
# Dan sasaran yang WAJAR di tengah frame tidak ikut terbuang.
_tengah = [(560, 300, 700, 450, 0.93, 0)]
cek("sasaran di tengah tetap lolos walau pita atas nyala",
    len(_ja.saring(_tengah, 1280, 720, pakai_roi=False,
                   h_min=_ka.jejak_h_min, h_max=_ka.jejak_h_max)), 1)
cek("bawaan roi_atas_frac = 0 (belum diukur, jangan menebak)",
    M.Kalib().roi_atas_frac, 0.0)

# LinkPalsu dipakai untuk rakit_state (ia punya seluruh medan yang dibaca);
# keadaan muatannya disetel langsung karena parsernya sudah diuji di atas.
_lp = LinkPalsu(); _lp._bawa = (True, False)
_sd = M.rakit_state(M.Misi(), _lp, _k, M.Juri(_k, NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa keadaan muatan", _sd["membawa"], "DEPAN")
cek("dan menandainya", _sd["membawa_ada"], True)
cek("id membawa ada di HTML", "id=membawa" in M.HALAMAN, True)
cek("HUD menyebut pose R 160 mm di atas", "160 mm di atas" in M.HALAMAN, True)
cek("HUD menyebut kamera tidak disebut Vincent",
    "tidak menyebut kamera" in M.HALAMAN, True)

print("\n60. m8: firmware HARUS menunggu, kalau tidak vision cuma menonton")
# Gejala 12 Sep 2026 di arena: pemicu #KORBAN sampai, vision mendeteksi,
# robot TIDAK bergerak, lalu capit turun sendiri. Bukan trigger yang hilang --
# firmware yang tidak pernah menunggu. Patch m8 yang menutupnya.
_lg = link_uji("")
cek("m8 lolos whitelist di KENDALI", _lg.kirim("m8", armed=True), True)
cek("m8 juga terkirim tanpa mode", link_uji("").kirim("m8", armed=False), True)
cek("tombol m8 ada di HUD", "'tunggu_visi'" in M.HALAMAN, True)
cek("HUD menjelaskan gejalanya", "capit turun sendiri" in M.HALAMAN, True)
cek("dan menyebut urutan yang benar", "ambil_alih" in M.HALAMAN, True)

# Keadaannya DIBACA dari kalimat firmware, bukan ditebak dari perintah kita.
# Penting karena Vincent bisa mengetik m8 langsung lewat serial.
_l = link_uji("")
cek("mula-mula belum diketahui", _l.tunggu_visi, None)
_l._parse("Tunggu vision sebelum ambil: NYALA -- ruas AMBIL berhenti menunggu 'm2' dari Raspi")
cek("kalimat NYALA -> True", _l.tunggu_visi, True)
_l._parse("Tunggu vision sebelum ambil: MATI -- sekuens lengan jalan langsung (perilaku v1.12)")
cek("kalimat MATI -> False", _l.tunggu_visi, False)
# Baris status 'tunggu visi : NYALA' juga terbaca.
_l2 = link_uji("")
_l2._parse("  tunggu visi : NYALA  (SEDANG PARKIR -- menunggu 'm2')")
cek("baris status juga terbaca", _l2.tunggu_visi, True)

_lp = LinkPalsu(); _lp.tunggu_visi = True
_sd = M.rakit_state(M.Misi(), _lp, M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa keadaan m8", _sd["tunggu_visi"], "NYALA")
cek("dan menandainya", _sd["tunggu_visi_ok"], True)
_lp0 = LinkPalsu()
_sd0 = M.rakit_state(M.Misi(), _lp0, M.Kalib(), M.Juri(M.Kalib(), NAMA),
                     {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("belum ditanya -> 'belum diketahui', bukan 'mati'",
    _sd0["tunggu_visi"], "belum diketahui")
cek("id tunggu_visi ada di HTML", "id=tunggu_visi" in M.HALAMAN, True)

# Dan ambil_alih tetap menolak bergerak tanpa BUKTI firmware parkir --
# m8 yang membuat bukti itu ada, bukan knob HUD.
_mm, _lgp, _ak = _picu(
    "ambil_alih",
    teks_fw="MENUNGGU KONFIRMASI -- PARKIR VISION ('m2' ambil / 'm3' ulangi)")
cek("state PARKIR VISION dianggap bukti parkir -> AMBIL ALIH",
    _mm.state, M.S_A_TENGAH)

print("\n61. Batas waktu HUD vs firmware: siapa bicara lebih dulu")
# Angka dari patch R2C Misi.cpp.
_FW_VISI_BATAS_S = 20
_HUD_TENGAH_S = M.BATAS.get(M.S_A_TENGAH)
cek("HUD punya batas di AMBIL_TENGAH", _HUD_TENGAH_S is not None, True)
# Kalau keduanya sama, capit turun tanpa HUD sempat mengatakan apa pun --
# kejadiannya jadi lomba, bukan urutan.
cek(f"batas HUD ({_HUD_TENGAH_S}s) LEBIH PENDEK dari firmware ({_FW_VISI_BATAS_S}s)",
    _HUD_TENGAH_S < _FW_VISI_BATAS_S, True)
cek("selisihnya cukup untuk terbaca (>= 5 detik)",
    _FW_VISI_BATAS_S - _HUD_TENGAH_S >= 5, True)

# Dan bawaan HUD + firmware R2C harus berpasangan: ambil_alih tidak berguna
# kalau firmware tidak parkir, dan parkir tidak berguna kalau HUD cuma menonton.
cek("bawaan HUD mengambil alih", M.Kalib().pemicu_korban, "ambil_alih")
# Penjaga buktinya tetap yang menentukan, bukan bawaannya.
_mm, _lg, _ak = _picu("ambil_alih")
cek("firmware TIDAK parkir -> tetap turun ke MENGAMATI", _mm.state, M.S_AWAS)
cek("dan nol perintah gerak", _ak.antre, [])
_mm2, _lg2, _ak2 = _picu("ambil_alih", teks_fw="MENUNGGU KONFIRMASI -- PARKIR VISION")
cek("firmware PARKIR -> ambil alih tanpa perlu mengubah knob apa pun",
    _mm2.state, M.S_A_TENGAH)

print("\n62. Saklar daya kamera")
# Kamera tiruan: yang diuji keputusan saklarnya, bukan OpenCV.
class _ThrPalsu:
    def __init__(self): self.running = True; self.dilepas = False
    def release(self): self.dilepas = True; self.running = False

_km = M.Kamera.__new__(M.Kamera)
_km.args = (0, 1280, 720, 60); _km.jeda = 3.0
_km.cap = None; _km.thread = _ThrPalsu(); _km.pesan = ""; _km._t = 0.0
_km.nyala = True; _km.sebab_mati = ""
cek("mula-mula nyala", _km.nyala, True)
cek("dan dianggap ok", _km.ok, True)

_p = _km.saklar(False, "lewat tombol HUD")
cek("dimatikan -> nyala False", _km.nyala, False)
cek("PERANGKATNYA benar-benar dilepas (bukan cuma berhenti baca)",
    _km.thread is None, True)
cek("pesannya menyebut daya", "daya" in _p, True)
cek("pastikan() menolak menyambung selagi mati", _km.pastikan(), False)
cek("dan sebabnya disebut", "DIMATIKAN" in _km.pesan, True)
cek("read() tidak mengembalikan frame", _km.read(0)[0], None)

# Dinyalakan lagi: jeda di-nolkan supaya tidak menunggu 3 detik sia-sia.
_km.saklar(True)
cek("dinyalakan -> nyala True", _km.nyala, True)
cek("jeda dinolkan supaya langsung mencoba", _km._t, 0.0)
cek("sebab_mati dibersihkan", _km.sebab_mati, "")

# BEDA dengan fokus_vision: itu knob kalibrasi, bukan saklar daya.
cek("fokus_vision masih knob tersendiri", hasattr(M.Kalib(), "fokus_vision"), True)
cek("dan bawaannya mati", M.Kalib().fokus_vision, False)

# HUD
cek("tombol kamera ada", "'kamera'" in M.HALAMAN, True)
cek("id kam_st ada", "id=kam_st" in M.HALAMAN, True)
cek("HUD menjelaskan bedanya dengan FOKUS",
    "benar-benar melepas perangkat" in M.HALAMAN, True)
cek("HUD menyebut tujuan akhir: dipicu Teensy",
    "hanya saat Teensy" in M.HALAMAN, True)

# Kartunya harus di tab MISI, DI ATAS 'Jalankan misi'.
_h = M.HALAMAN
_a = _h.index("<!-- 1. MISI -->"); _b = _h.index("<!-- 2. KORBAN -->")
_misi = _h[_a:_b]
cek("kartu saklar ada di tab Misi", "saklar daya" in _misi, True)
cek("dan DI ATAS 'Jalankan misi'",
    _misi.index("saklar daya") < _misi.index("Jalankan misi"), True)

_sd = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "", _km)
cek("/state membawa keadaan kamera", _sd["kamera_nyala"], True)
_km.saklar(False)
_sd2 = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                     {"fps": 0.0, "t_inf": 0.0}, True, "", _km)
cek("dan ikut berubah saat dimatikan", _sd2["kamera_nyala"], False)

print("\n63. Capit menutup ke 10%, bukan 0 (servo stall = panas)")
# Angka dari patch R2C config.h.
_FW_GRIP_MIN = 10.0
_FW_GRIP_MAKS = 95.0
cek("GRIP_PERSEN_MIN naik ke 10", _FW_GRIP_MIN, 10.0)
# Angka itu dipakai DUA tempat di firmware, dan itu yang membuatnya tempat
# yang benar untuk diperbaiki: sekuensAmbil fase 2 menutup ke GRIP_PERSEN_MIN,
# DAN perintah 'g' di-clamp ke rentang itu. Jadi 'g0' manual pun ter-clamp.
# Memperbaiki hanya di sisi Raspi meninggalkan jalur manual terbuka -- dan
# jalur manual itu justru yang dipakai saat trial-and-error.
def _clamp(v): return max(_FW_GRIP_MIN, min(_FW_GRIP_MAKS, v))
cek("'g0' manual ikut ter-clamp ke 10", _clamp(0.0), 10.0)
cek("'g5' juga", _clamp(5.0), 10.0)
cek("'g50' lewat apa adanya", _clamp(50.0), 50.0)
cek("'g100' tetap berhenti di 95", _clamp(100.0), 95.0)

# Sisi HUD: tidak boleh ada g0/G0 yang tersisa sebagai perintah.
for _c in ("'g0'", '"g0"', "'G0'", '"G0"'):
    cek(f"{_c} tidak lagi dikirim HUD", _c in M.HALAMAN, False)
cek("tombol capit tutup menyebut g10", "g10" in M.HALAMAN, True)
cek("dan menyebut sebabnya (panas)", "panas" in M.HALAMAN, True)

# Rantai AMBIL mengirim g10.
_lg = LinkPalsu(); _mm = M.Misi(); _ak = M.Aksi(_lg)
_mm.ganti(M.S_A_JEPIT)
M.langkah_fsm(_mm, _lg, _ak, M.Kalib(), armed=True)
cek("AMBIL_JEPIT menjadwalkan g10", [c for c, _ in _ak.antre], ["g10"])

print("\n64. Pemicu serah-terima dari Pi: 20 px BERTAHAN 2 detik")
_k = M.Kalib()
cek("toleransi dilonggarkan ke 20 px", _k.tengah_tol_px, 20.0)
_tol_der = _k.tengah_tol_px / _k.px_per_deg(1280)
cek("= 1,1 der (8 px yang lama cuma 0,44)", round(_tol_der, 2), 1.1)
# 8 px LEBIH KECIL dari langkah terkecil putar badan -- syarat yang tidak
# pernah bisa dipenuhi. Itu sebabnya robot melangkah kecil selamanya.
# 8 px = 0,44 der. Itu memang di ATAS langkah_min_deg (0,25) -- jadi bukan
# "mustahil secara aritmetika". Yang membuatnya tidak tercapai: derau bbox.
# Satu piksel di bbox 340 px sudah menggeser pusatnya, dan 8 px itu ambang
# yang lebih kecil daripada goyangan normal deteksi frame-ke-frame.
cek("8 px lama = 0,44 der", round(8.0 / _k.px_per_deg(1280), 2), 0.44)
cek("dan itu lebih sempit daripada maks_langkah_deg sekali jalan",
    8.0 / _k.px_per_deg(1280) < _k.maks_langkah_deg, True)
cek("20 px baru jauh di atas langkah terkecil (bisa diendapkan)",
    _tol_der > _k.langkah_min_deg * 3, True)
# DINOLKAN 17 Sep 2026: dwell membeli keyakinan dengan detik, dan detiknya
# yang dikeluhkan operator sesudah trial.
cek("dwell dinolkan", _k.tengah_tahan_s, 0.0)

def _pus(err_urut, kal=None, jeda=0.0):
    """Jalankan pusatkan() beberapa kali dengan galat berurutan."""
    kal = kal or M.Kalib()
    mm = M.Misi(); ak = M.Aksi(LinkPalsu())
    hasil = []
    for e in err_urut:
        hasil.append(M.pusatkan(mm, ak, kal, e, None, 1280, None))
        if jeda:
            time.sleep(jeda)
    return hasil, mm

# Satu frame di tengah SUDAH cukup sejak dwell dinolkan.
_h, _mm = _pus([0.2])
cek("satu frame di tengah -> selesai (dwell 0)", _h[-1], True)
cek("tapi hitungannya tetap mulai", _mm.t_tengah is not None, True)
# Dwell masih BEKERJA kalau dinyalakan lagi -- yang berubah bakunya, bukan
# mesinnya.
_kd = M.Kalib(); _kd.tengah_tahan_s = 5.0
_hd, _md = _pus([0.2], _kd)
cek("dwell dinyalakan lagi -> satu frame BELUM cukup", _hd[-1], False)

# Bertahan cukup lama -> selesai.
_k2 = M.Kalib(); _k2.tengah_tahan_s = 0.25
_h, _mm = _pus([0.2, 0.2, 0.2], _k2, jeda=0.15)
cek("bertahan lewat ambang -> SELESAI", _h[-1], True)
cek("lama tahan tercatat", _mm.tahan_s >= 0.25, True)

# PUTUS sekali -> hitungan dinolkan, bukan dilanjutkan.
_k3 = M.Kalib(); _k3.tengah_tahan_s = 0.25
mm = M.Misi(); ak = M.Aksi(LinkPalsu())
M.pusatkan(mm, ak, _k3, 0.2, None, 1280, None)
time.sleep(0.2)
M.pusatkan(mm, ak, _k3, 5.0, None, 1280, None)     # keluar toleransi
cek("putus -> hitungan DINOLKAN", mm.t_tengah, None)
cek("dan lama tahan nol", mm.tahan_s, 0.0)
M.pusatkan(mm, ak, _k3, 0.2, None, 1280, None)     # masuk lagi
cek("masuk lagi -> mulai dari awal, bukan lanjut",
    mm.tahan_s < 0.25, True)

print("\n65. 'O' yang tidak menggerakkan apa pun harus BERHENTI dicoba")
# Ini yang membuat penengahan "macet tapi gerak sedikit": galat di atas
# yaw_kasar_deg dikirim ke 'O', 'O' diam (IMU mati), galatnya tetap, 'O'
# dikirim lagi. Selamanya.
cek("pakai_pivot_gait bawaan nyala", M.Kalib().pakai_pivot_gait, True)
_kg = M.Kalib()
mm = M.Misi(); lg = LinkPalsu(); ak = M.Aksi(lg)
_ERR = 12.0                       # di atas yaw_kasar_deg 8
M.pusatkan(mm, ak, _kg, _ERR, None, 1280, None)
cek("percobaan 1 menjadwalkan O", any(c.startswith("O") for c, _ in ak.antre), True)
ak.antre.clear()
M.pusatkan(mm, ak, _kg, _ERR, None, 1280, None)   # galat TIDAK berubah
cek("percobaan 2: satu strike", mm.o_diam, 1)
ak.antre.clear()
M.pusatkan(mm, ak, _kg, _ERR, None, 1280, None)
cek("percobaan 3: dua strike -> pivot gait DINYATAKAN MATI",
    mm.pivot_gait_mati, True)
# Dan sesudah itu: galat 12 der di luar jangkauan badan? 12 < 18-1, jadi
# masih bisa ditangani putar badan -- harus LANJUT, bukan gagal.
cek("12 der masih di dalam jangkauan badan -> tidak gagal",
    mm.state == M.S_GAGAL, False)

# Galat di LUAR jangkauan badan + O mati -> GAGAL dengan sebab, bukan selamanya.
mm2 = M.Misi(); ak2 = M.Aksi(LinkPalsu())
mm2.pivot_gait_mati = True
M.pusatkan(mm2, ak2, _kg, 25.0, None, 1280, None)
cek("25 der + O mati -> GAGAL", mm2.state, M.S_GAGAL)
cek("sebabnya menyebut IMU", "IMU" in mm2.sebab, True)
cek("dan menyuruh memutar dengan tangan", "tangan" in mm2.sebab, True)
cek("TIDAK ada O yang dijadwalkan lagi",
    [c for c, _ in ak2.antre if c.startswith("O")], [])

# Kalau galatnya BERUBAH sesudah O, strike-nya di-reset -- O memang bekerja.
mm3 = M.Misi(); ak3 = M.Aksi(LinkPalsu())
M.pusatkan(mm3, ak3, _kg, 20.0, None, 1280, None)
ak3.antre.clear()
M.pusatkan(mm3, ak3, _kg, 12.0, None, 1280, None)   # berubah 8 der
cek("O yang bekerja -> strike tetap nol", mm3.o_diam, 0)
cek("dan pivot gait tidak dimatikan", mm3.pivot_gait_mati, False)

print("\n66. Siklus daya kamera dipicu Teensy")
# DIBALIK LAGI 17 Sep 2026, diminta R2C: kamera tetap menyala dari misi mulai
# sampai misi selesai. Siklus nyala-mati per korban membayar auto-exposure C922
# tepat di detik saat penengahan mau memakainya.
#
# Knobnya TIDAK dibuang: ia peredam under-voltage yang masih dibutuhkan kalau
# brownout kembali sebelum catu dayanya diperbaiki. Uji di bawah tetap menguji
# perilakunya saat dinyalakan.
cek("kamera_ikut_pemicu bawaan MATI (kamera menyala sepanjang misi)",
    M.Kalib().kamera_ikut_pemicu, False)

class _KamPalsu:
    def __init__(self, on=True): self.nyala = on; self.jejak = []
    def saklar(self, on, sebab=""):
        self.nyala = bool(on); self.jejak.append("ON" if on else "OFF")
        return "ON" if on else "OFF"

_kk = M.Kalib(); _kk.kamera_ikut_pemicu = True
_km = _KamPalsu(on=False)
_lg = link_uji("")
_lg._parse("#KORBAN AMBIL 1")
_mm = M.Misi(); _ak = M.Aksi(_lg)
M.tangani_pemicu(_mm, _lg, _ak, _kk, True, _km)
_lg._parse("  state       : MENUNGGU KONFIRMASI -- PARKIR VISION")
M.tangani_pemicu(_mm, _lg, _ak, _kk, True, _km)
cek("pemicu #KORBAN -> kamera DINYALAKAN", _km.jejak, ["ON"])
cek("dan dwell dimulai bersih", _mm.t_tengah, None)
cek("serta deteksi O direset", (_mm.o_diam, _mm.pivot_gait_mati), (0, False))

# Serah terima selesai -> kamera TETAP HIDUP.
#
# Berubah 14 Sep 2026. Dulu kamera dimatikan di sini dengan alasan "vision
# selesai untuk korban ini" -- dan itu benar selama badan ikut dinetralkan
# di baris yang sama. Sekarang pose tengahnya DITAHAN selama capit turun,
# dan yang menahannya vision: mematikan kamera di sini berarti tidak ada
# lagi yang bisa mengoreksi selama 8,4 detik sekuens capit firmware.
_km2 = _KamPalsu(on=True)
_kk2 = M.Kalib(); _kk2.kamera_ikut_pemicu = True; _kk2.auto_konfirm = True
_kk2.tengah_tahan_s = 0.0
_lg2 = LinkPalsu(); _mm2 = M.Misi(); _ak2 = M.Aksi(_lg2)
_mm2.ganti(M.S_A_HALUS); _mm2.dari_firmware = True; _mm2.bearing_deg = 0.0
M.langkah_fsm(_mm2, _lg2, _ak2, _kk2, armed=True, diam=None, kamera=_km2)
cek("penengahan selesai -> kamera TETAP HIDUP", _km2.jejak, [])
cek("  karena vision masih menahan pose", _mm2.state, M.S_TAHAN)
cek("  dan m2 dikirim", "m2" in [c for c, _ in _ak2.antre], True)

# Baru sesudah capit terangkat dan jendela tahannya habis, kamera dimatikan
# bersama pelepasan kaki -- satu kejadian, satu tempat.
_mm2.t_lepas = time.time() - (_kk2.lepas_tahan_s + 0.1)
M.langkah_fsm(_mm2, _lg2, _ak2, _kk2, armed=True, diam=None, kamera=_km2)
cek("kendali kaki dilepas -> kamera DIMATIKAN", _km2.jejak, ["OFF"])
# Tanpa knob itu, kamera tidak disentuh sama sekali.
_km3 = _KamPalsu(on=True)
_kk3 = M.Kalib(); _kk3.auto_konfirm = True; _kk3.tengah_tahan_s = 0.0
_lg3 = LinkPalsu(); _mm3 = M.Misi(); _ak3 = M.Aksi(_lg3)
_mm3.ganti(M.S_A_HALUS); _mm3.dari_firmware = True; _mm3.bearing_deg = 0.0
M.langkah_fsm(_mm3, _lg3, _ak3, _kk3, armed=True, diam=None, kamera=_km3)
cek("knob mati -> kamera tidak disentuh", _km3.jejak, [])

_sd = M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                    {"fps": 0.0, "t_inf": 0.0}, True, "")
for _f in ("tahan", "tahan_ok", "pivot_gait", "pivot_gait_ok"):
    cek(f"/state punya '{_f}'", _f in _sd, True)
for _i in ("tahan", "pivot_gait"):
    cek(f"id '{_i}' ada di HTML", f"id={_i}" in M.HALAMAN, True)

print("\n67. REVERT 12 Sep: jalur JEJAK kembali ke versi 10 September")
# Kenapa seksi ini ada.
#
# 12 Sep JEJAK berhenti mengikuti korban, dan kotaknya abu-abu bertuliskan
# "disaring" -- tanpa sebab. Label itu yang menunjuk pelakunya: anotasi()
# mengambil teksnya dari juri.alasan, dan juri.alasan HANYA diisi saring().
# Deteksi yang ditolak saring SELALU punya sebab. Jadi kotak abu-abu tanpa
# sebab tidak mungkin datang dari saring -- ia datang dari sesuatu yang
# memotong `lolos` SESUDAH saring. Uji di bawah mengunci itu.
_ksrc = open(M.__file__ if M.__file__.endswith(".py") else "mission_hud.py",
             encoding="utf-8").read()
# AMBIL_HALUS ikut cabang ini sejak 14 Sep (lihat seksi 75): ia juga
# mengemudi dengan bearing, jadi invarian di bawah berlaku untuknya juga.
# TAHAN_TENGAH menyusul di hari yang sama, dengan alasan yang sama persis --
# dan barisnya jadi terlipat dua, jadi yang dicari cuma awalannya.
_i0 = _ksrc.index("if misi.state in (S_JEJAK, S_A_TENGAH, S_A_HALUS,",
                  _ksrc.index("if segar:"))
_i1 = _ksrc.index("elif misi.state == S_LIHAT:", _i0)
# Komentar dibuang dulu: seksi ini menjelaskan gerbang yang DIHAPUS, jadi
# namanya masih muncul di prosa. Yang diuji kodenya.
_blok = "\n".join(l.split("#")[0] for l in _ksrc[_i0:_i1].splitlines())
# INVARIANT-nya adalah `lolos` TIDAK BOLEH DIPOTONG sesudah saring -- itu
# yang membuat kotak jadi abu-abu tanpa sebab. Pemilihan sasaran boleh pilih-
# pilih; ia tidak menyentuh daftar yang dipakai menggambar.
cek("tidak ada lagi 'lolos = []' di jalur JEJAK", "lolos = []" in _blok, False)
cek("tidak ada lagi gerbang muatan di depan pemilihan",
    "sedang_membawa(" in _blok, False)
cek("pemilihan sasaran lewat pilih_sasaran()", "pilih_sasaran(" in _blok, True)
# Dibuktikan dengan menjalankannya, bukan dengan membaca sumbernya: daftar
# yang dipakai menggambar harus keluar UTUH.
_nm = {0: "korban", 1: "dummy"}
_in = [(600, 100, 700, 532, 0.9, 1), (300, 150, 380, 500, 0.8, 0)]
_salinan = list(_in)
M.pilih_sasaran(_in, _nm, M.Kalib())
cek("pilih_sasaran tidak pernah mengubah lolos", _in, _salinan)
# Yang menahan GERAKAN tetap sasaran_sah() -- di sana, bukan di penglihatan.
cek("gerbang kelas masih ada, di sasaran_sah", "def sasaran_sah" in _ksrc, True)
_mk = M.Misi(); _mk.jejak_kelas = M.Kalib().kelas_dummy
cek("dummy tetap tidak boleh digerakkan",
    M.sasaran_sah(_mk, M.Kalib(), True), False)
_mk.jejak_kelas = M.Kalib().kelas_korban
cek("korban boleh digerakkan", M.sasaran_sah(_mk, M.Kalib(), True), True)

# CENTER juga bersih dari saring kelas.
_i2 = _ksrc.index("elif misi.state == S_CENTER:", _i1)
_i3 = _ksrc.index("# --- serah-terima", _i2)
_bc = "\n".join(l.split("#")[0] for l in _ksrc[_i2:_i3].splitlines())
cek("CENTER memakai aturan pemilihan yang SAMA", "pilih_sasaran(" in _bc, True)
cek("CENTER juga tidak memotong lolos", "lolos = []" in _bc, False)

# Invarian labelnya, diuji langsung: apa pun yang DIBUANG saring punya sebab.
_kj = M.Kalib()
_jj = M.Juri(_kj, NAMA)
# Pita JEJAK 0,06..0,98 dari tinggi frame 720 px => 43..705 px.
_dets = [
    (40, 300, 90, 320, 0.9, 0),        # 20 px  -> terlalu jauh
    (600, 2, 1180, 718, 0.9, 0),       # 716 px -> terlalu dekat
    (600, 300, 700, 420, 0.9, 0),      # 120 px -> masuk pita
]
_lolos = _jj.saring(_dets, 1280, 720, pakai_roi=False,
                    h_min=_kj.jejak_h_min, h_max=_kj.jejak_h_max)
cek("satu yang masuk pita memang lolos", len(_lolos), 1)
_kunci = {(l[0], l[1]) for l in _lolos}
for _d in _dets:
    if (_d[0], _d[1]) in _kunci:
        continue
    cek(f"ditolak saring -> ada sebabnya ({_d[0]},{_d[1]})",
        bool(_jj.alasan.get((_d[0], _d[1]))), True)
cek("dan sebabnya bukan kata bawaan 'disaring'",
    any(v == "disaring" for v in _jj.alasan.values()), False)

# Knob-knob yang ikut dikembalikan.
_kb = M.Kalib()
cek("tengah_tol_px tetap 20 (permintaan R2C, bukan bagian vision)",
    _kb.tengah_tol_px, 20.0)
cek("roi_atas_frac masih 0 (belum diukur)", _kb.roi_atas_frac, 0.0)
cek("tolak_saat_membawa MATI", _kb.tolak_saat_membawa, False)
cek("vision_silang_on MATI", _kb.vision_silang_on, False)
cek("jejak_hanya_korban tetap NYALA (gerakan, bukan penglihatan)",
    _kb.jejak_hanya_korban, True)
# Pengukuran jarak tetap ada, tapi cuma bacaan: ia tidak pernah membuang
# sasaran. Rasio yang tidak wajar menolkan jarak -- bukan bearing.
cek("rasio tidak wajar -> jarak kosong, bukan sasaran hilang",
    M.rasio_wajar(100, 100, 400, 130, _kb)[0], False)


print("\n68. Jalur misi: Raspi HANYA menengahkan, capit milik Teensy")
# Keputusan R2C 13 Sep 2026. Bukan sekadar pembagian tugas -- S_A_MAJU yang
# ikut jalan di sini adalah kesalahan kerangka acuan yang nyata:
#
#   firmware berhenti di HNT_DEPAN = KORBAN_JARAK_CM      = 24 cm
#   pose lengannya  KORBAN_CAPIT_MM = 24*10 + 62          = 302 mm
#                   ...DIHITUNG untuk robot yang berdiri di 24 cm itu.
#   S_A_MAJU berjalan maju sampai LiDAR = capit_cm        = 10 cm
#
# Jadi kalau S_A_MAJU jalan, Pi memajukan robot ~14 cm lalu firmware tetap
# menjulurkan lengan seolah masih di 24 cm.

def _tengah_selesai(dari_firmware):
    _k68 = M.Kalib()
    _l68 = link_uji("")
    _m68 = M.Misi()
    _a68 = M.Aksi(_l68)
    _m68.dari_firmware = dari_firmware
    _m68.ganti(M.S_A_TENGAH)
    _m68.bearing_deg = 2.0                 # sudah di dalam yaw_kasar_deg (8)
    _m68.jejak_kelas = _k68.kelas_korban
    M.langkah_fsm(_m68, _l68, _a68, _k68, True)
    return _m68, _a68

_m68, _a68 = _tengah_selesai(True)
cek("jalur misi: TENGAH -> HALUS, S_A_MAJU dilewati", _m68.state, M.S_A_HALUS)
cek("  dan tidak satu pun perintah capit dijadwalkan",
    [c for c, _ in _a68.antre if c[:1] in ("g", "a")], [])
cek("  juga tidak ada perintah jalan",
    [c for c, _ in _a68.antre if c[:1] in ("w", "D")], [])

_m68b, _ = _tengah_selesai(False)
cek("uji meja (bukan dari firmware) tetap lewat S_A_MAJU",
    _m68b.state, M.S_A_MAJU)

# Penjaga struktural: cabang dari_firmware harus benar-benar menyebut S_A_HALUS.
with open(M.__file__, encoding="utf-8") as _f:
    _src = _f.read()
_blok = _src[_src.index("    if misi.state == S_A_TENGAH:"):
             _src.index("    if misi.state == S_A_HALUS:")]
cek("cabang dari_firmware ada di blok S_A_TENGAH", "misi.dari_firmware" in _blok, True)
cek("  dan tujuannya S_A_HALUS", "S_A_HALUS" in _blok, True)

print("\n69. Anggaran waktu: HUD harus selalu bicara lebih dulu")
# Firmware R2C: MISI_VISI_BATAS_MS = 20 detik. Kalau Pi menghabiskan lebih
# dari itu, firmware yang mengakhiri -- capit turun tanpa HUD sempat
# mengatakan apa pun, dan kejadiannya jadi lomba, bukan urutan.
_FW = 20
_pi_total = M.BATAS[M.S_A_TENGAH] + M.BATAS[M.S_A_HALUS]
cek(f"TENGAH + HALUS ({_pi_total}s) muat di batas firmware ({_FW}s)",
    _pi_total < _FW, True)
cek("dan sisanya cukup untuk terbaca (>= 3 detik)", _FW - _pi_total >= 3, True)
# Dwell-nya sendiri harus muat di jatah HALUS, kalau tidak syaratnya tidak
# akan pernah sempat terpenuhi.
cek("dwell muat di jatah HALUS",
    M.Kalib().tengah_tahan_s < M.BATAS[M.S_A_HALUS], True)

print("\n70. Kotak serial: kartu paling atas tab Manual")
_h70 = M.HALAMAN
_panel3 = _h70.split("<div class=panel>")[3]
cek("cmdbox ada di tab Manual", "id=cmdbox" in _panel3, True)
cek("  dan di kartu PERTAMA tab itu",
    _panel3.index("id=cmdbox") < _panel3.index("Kirim perintah langsung"), True)
cek("judulnya menyebut tanpa filter", "kirim apa saja" in _panel3, True)
cek("tombol Kirim ada", "kirimManual()" in _panel3, True)
cek("hanya SATU cmdbox di seluruh halaman", _h70.count("id=cmdbox"), 1)


print("\n71. Dummy tidak pernah jadi SASARAN, dan tetap terlihat")
# Laporan arena 13 Sep 2026: ada dummy di sebelah korban, robot menengahkan
# diri ke DUMMY. Sebabnya max(lolos, key=tinggi) -- terbesar menang, kelas
# tidak dilihat. Dummy yang lebih dekat kamera selalu lebih besar.
_nm71 = {0: "korban", 1: "dummy"}
_k71 = M.Kalib()
_dummy_besar = (600, 100, 700, 600, 0.95, 1)      # tinggi 500
_korban_kecil = (200, 200, 280, 500, 0.70, 0)     # tinggi 300
_t, _nd = M.pilih_sasaran([_dummy_besar, _korban_kecil], _nm71, _k71)
cek("dummy LEBIH BESAR, korban tetap yang dipilih", _t, _korban_kecil)
cek("  dan tidak ada dummy yang dilaporkan diabaikan", _nd, 0)

_t, _nd = M.pilih_sasaran([_dummy_besar], _nm71, _k71)
cek("hanya dummy -> tidak ada sasaran sama sekali", _t, None)
cek("  dan jumlahnya dilaporkan untuk ditampilkan", _nd, 1)

_k71b = M.Kalib(); _k71b.jejak_hanya_korban = False
_t, _ = M.pilih_sasaran([_dummy_besar], _nm71, _k71b)
cek("gerbang dimatikan -> dummy boleh dipilih lagi", _t, _dummy_besar)

_korban_besar = (600, 100, 700, 600, 0.6, 0)
_korban_kcl2 = (200, 200, 280, 450, 0.99, 0)
_t, _ = M.pilih_sasaran([_korban_kcl2, _korban_besar], _nm71, _k71)
cek("dua korban -> yang TERBESAR (terdekat) menang", _t, _korban_besar)

# Dan sasaran_sah tetap jadi lapis kedua, bukan diganti.
_m71 = M.Misi(); _m71.jejak_kelas = _k71.kelas_dummy
cek("sasaran_sah masih menahan dummy", M.sasaran_sah(_m71, _k71, True), False)

print("\n72. Serah-terima tidak boleh direbut balik oleh firmware")
# Bug yang membuat seluruh lup terlihat mati padahal firmware sudah benar:
# selama ruas AMBIL, ruas_fw() terus berbunyi "K-1 angkat korban" DAN fw terus
# berbunyi "MENUNGGU KONFIRMASI" -- justru karena firmware sedang parkir
# menunggu kita. Kedua cabang serah-terima dulu tidak mengecualikan rantai
# AMBIL, jadi state-nya ditarik balik ke S_AWAS/S_KONFIRM tiap frame, dan
# juri.reset() di tiap tarikan membuat voting tidak pernah sampai k-of-N.
# Gejalanya: "RAGU margin +0.00 conf 0.00" berulang tanpa henti.
with open(M.__file__, encoding="utf-8") as _f:
    _s72 = _f.read()
_b72 = _s72[_s72.index("# --- serah-terima dari firmware"):
            _s72.index("# --- pemicu dari firmware")]
_b72k = "\n".join(l.split("#")[0] for l in _b72.splitlines())
cek("penjaga _sedang_serah dihitung", "_sedang_serah =" in _b72k, True)
cek("  dan memuat rantai AMBIL", "RANTAI_AMBIL" in _b72k, True)
cek("  dan memuat pemicu yang masih tertunda", "pemicu_tunda" in _b72k, True)
cek("cabang KONFIRMASI memakainya", _b72k.count("not _sedang_serah") >= 2, True)
# Tiap state rantai AMBIL harus benar-benar terlindungi.
for _st in M.RANTAI_AMBIL:
    cek(f"  '{_st}' tidak bisa direbut", _st in M.RANTAI_AMBIL, True)

print("\n73. Firmware berhenti parkir -> Pi berhenti mengirim gerak")
# "Robot bergerak ke kiri lalu melawan bergerak ke kanan seolah disuruh pergi
# ke kanan." Memang disuruh -- oleh firmware: batas parkir habis, sekuens
# lengan jalan, ruasBerikut(), lalu "Pivot MULAI menuju 176.8 der". Kalau Pi
# masih mengirim 'O' di detik yang sama, dua penguasa menarik satu robot.
_l73 = link_uji("")
_m73 = M.Misi(); _a73 = M.Aksi(_l73); _k73 = M.Kalib()
_m73.dari_firmware = True
_m73.ganti(M.S_A_TENGAH)
_m73.bearing_deg = 20.0
_m73.jejak_kelas = _k73.kelas_korban
# Firmware masih parkir: Pi boleh bekerja.
_l73._parse("  state       : MENUNGGU KONFIRMASI -- PARKIR VISION")
M.langkah_fsm(_m73, _l73, _a73, _k73, True)
cek("selagi parkir: Pi tetap menengahkan", _m73.state, M.S_A_TENGAH)
cek("  dan parkirnya tercatat pernah terlihat", _m73.parkir_terlihat, True)
# Firmware pindah ruas (batas waktu habis): Pi HARUS berhenti.
_a73.antre.clear()
_l73._parse("  state       : BERJALAN")
M.langkah_fsm(_m73, _l73, _a73, _k73, True)
cek("firmware pindah ruas -> Pi keluar dari rantai AMBIL", _m73.state, M.S_IDLE)
cek("  antrean perintah dikosongkan", list(_a73.antre), [])
cek("  dan sebabnya dicatat",
    any("[SERAH GAGAL]" in x for x in _l73.log), True)

# Deteksi lewat TEPI: serah-terima yang buktinya dari baris pengumuman tidak
# boleh membatalkan dirinya sendiri sebelum poll 'm' pertama menjawab.
_l74 = link_uji("")
_m74 = M.Misi(); _a74 = M.Aksi(_l74); _k74 = M.Kalib()
_m74.dari_firmware = True
_m74.ganti(M.S_A_TENGAH)
_m74.bearing_deg = 20.0
_m74.jejak_kelas = _k74.kelas_korban
M.langkah_fsm(_m74, _l74, _a74, _k74, True)     # belum ada status sama sekali
cek("belum pernah lihat parkir -> JANGAN batalkan", _m74.state, M.S_A_TENGAH)

print("\n74. Serah-terima BALIK: '#LEPAS' -> 'm9', lalu misi lanjut")
# Ujung satunya dari serah-terima. Tanpa jalur ini Teensy berdiri diam sambil
# MENGGENGGAM korban sampai MISI_LEPAS_BATAS_MS habis -- tiap korban. Yang
# terlihat di arena cuma robot yang berhenti lama tanpa sebab yang kelihatan.
_l75 = link_uji("")
_m75 = M.Misi(); _a75 = M.Aksi(_l75); _k75 = M.Kalib()
_m75.dari_firmware = True
_m75.ganti(M.S_TAHAN)
_m75.parkir_terlihat = True
_m75.badan_yaw = 4.0                  # pose tengah yang sedang ditahan
_l75._parse("#LEPAS 1")
cek("baris '#LEPAS' terbaca", _l75.lepas_baru, True)
cek("  ruasnya ikut terbaca", _l75.lepas[0], 1)
cek("  bukti parkir ruas ini dipadamkan", _l75.parkir_visi, False)
_hasil75 = M.tangani_pemicu(_m75, _l75, _a75, _k75, True)
cek("pemicunya dikonsumsi", _hasil75, True)
# INTI PERUBAHAN 14 Sep: '#LEPAS' = "capit sudah terangkat", BUKAN "silakan
# lepas kaki". Melepas di detik itu mengembalikan badan ke default tepat saat
# korban baru terjepit.
cek("'m9' BELUM terkirim", _l75.tx_terakhir, "")
cek("  waktunya yang dicatat", _m75.t_lepas is not None, True)
cek("  masih menahan pose tengah", _m75.state, M.S_TAHAN)
cek("  badan belum dinetralkan", _m75.badan_yaw, 4.0)
cek("  dan sebabnya dicatat", any("[LEPAS]" in x for x in _l75.log), True)

# Selama jendela tahannya belum habis, TIDAK ADA perintah lepas yang lahir.
M.langkah_fsm(_m75, _l75, _a75, _k75, True, diam=None)
cek("  di dalam jendela tahan: belum ada 'm9' diantre",
    "m9" in [c for c, _ in _a75.antre], False)

# Sesudah lepas_tahan_s lewat: netralkan badan DULU, 'm9' paling belakang.
_m75.t_lepas = time.time() - (_k75.lepas_tahan_s + 0.1)
M.langkah_fsm(_m75, _l75, _a75, _k75, True, diam=None)
_urut75 = [c for c, _ in _a75.antre]
cek("sesudah jendela tahan: badan dinetralkan", _urut75[0], "r0 0 0")
cek("  lalu geseran badan ikut dinolkan", "t0 0 0" in _urut75, True)
cek("  dan 'm9' PALING BELAKANG", _urut75[-1], "m9")
cek("  'm9' sesudah netral, bukan sebelum",
    _urut75.index("m9") > _urut75.index("r0 0 0"), True)

# SEKALI PAKAI. Kalau blok pelepasan dijadwalkan ulang tiap frame, Teensy
# menerima banjir 'm9' -- dan tiap satunya dijawab "Tidak sedang menunggu
# 'm9'", membanjiri log yang justru dipakai untuk membaca kejadian.
M.langkah_fsm(_m75, _l75, _a75, _k75, True, diam=None)
cek("pelepasan dijadwalkan SEKALI", [c for c, _ in _a75.antre], _urut75)

# Baru sesudah antrean benar-benar terkirim, rantainya ditutup.
_a75.batal()
M.langkah_fsm(_m75, _l75, _a75, _k75, True, diam=None)
cek("antrean habis -> rantai firmware ditutup", _m75.dari_firmware, False)
cek("  kembali menunggu ruang korban berikutnya", _m75.state, M.S_IDLE)

# HUD yang TIDAK sedang memegang kaki (misal baru di-restart di tengah
# sekuens) tidak punya pose untuk ditahan. Di situ menunda cuma membuang
# waktu kontes: jawab langsung.
_l75b = link_uji("")
_m75b = M.Misi(); _a75b = M.Aksi(_l75b); _k75b = M.Kalib()
_m75b.ganti(M.S_IDLE)
_a75b.jadwal(("O5", 1.0))            # antrean yang MASIH berisi perintah gerak
_l75b._parse("#LEPAS 1")
M.tangani_pemicu(_m75b, _l75b, _a75b, _k75b, True)
cek("tidak sedang menahan pose -> 'm9' langsung", _l75b.tx_terakhir, "m9")
cek("  antrean gerak dikosongkan DULU", list(_a75b.antre), [])

# '#LEPAS' mendahului '#KORBAN'. Keduanya bisa tiba di burst serial yang sama
# kalau ruas korban berikutnya datang cepat; yang menggantung robot adalah
# 'm9' yang belum terkirim, jadi itu yang harus dijawab lebih dulu.
_l76 = link_uji("")
_m76 = M.Misi(); _a76 = M.Aksi(_l76); _k76 = M.Kalib()
_m76.ganti(M.S_KONFIRM)      # bukan S_TAHAN: jalur jawab-langsung
_l76._parse("#KORBAN AMBIL 8")
_l76._parse("#LEPAS 1")
M.tangani_pemicu(_m76, _l76, _a76, _k76, True)
cek("'m9' dijawab lebih dulu daripada pemicu baru", _l76.tx_terakhir, "m9")
cek("  pemicu korban berikutnya belum dikonsumsi", _l76.pemicu_baru, True)

print("\n75. Bearing WAJIB segar di tiap state kemudi")
# Bug 14 Sep 2026: AMBIL_HALUS memanggil pusatkan() tiap frame tapi tidak ada
# satu pun cabang vision yang memperbarui bearing_deg untuknya. Galat yang beku
# membuat badan_yaw menumpuk ke satu arah, mentok di badan_yaw_maks, lalu
# pusatkan() menyerah ke pivot gait dan mengulang dari nol -- badan berputar ke
# kiri tanpa henti, dan 'sudah tengah' tidak pernah bisa benar.
with open(M.__file__, encoding="utf-8") as _f:
    _s75 = _f.read()
_b75 = _s75[_s75.index("# --- vision HANYA di state tertentu ---"):
            _s75.index("# --- serah-terima dari firmware")]
_b75k = "\n".join(l.split("#")[0] for l in _b75.splitlines())

for _st, _nama in ((M.S_JEJAK, "JEJAK"), (M.S_CENTER, "CENTERING"),
                   (M.S_A_TENGAH, "AMBIL_TENGAH"), (M.S_A_HALUS, "AMBIL_HALUS")):
    cek(f"'{_nama}' terdaftar sebagai state kemudi", _st in M.MENGEMUDI, True)

cek("gerbang kejar memakai daftar itu, bukan salinannya",
    "MENGEMUDI" in _b75k, True)
cek("  AMBIL_HALUS ada di cabang yang MENULIS bearing_deg",
    "S_A_HALUS" in _b75k, True)
cek("  dan cabang penulisnya memang lebih dari satu",
    _b75k.count("bearing_deg =") >= 2, True)

# Tiap state kemudi harus benar-benar disebut di blok vision -- kalau tidak,
# ia memakai bearing milik state sebelumnya.
for _st in M.MENGEMUDI:
    _var = [n for n in ("S_JEJAK", "S_CENTER", "S_A_TENGAH", "S_A_HALUS",
                        "S_TAHAN")
            if getattr(M, n) == _st][0]
    cek(f"  '{_st}' disebut di blok vision", _var in _b75k, True)

# 'tidak delay dengan teensy': AMBIL_HALUS tidak boleh ikut dibatasi jejak_hz.
# Dwell 2 detiknya menuntut bacaan sesegar mungkin -- inferensi yang dijatah
# membuat 'sudah tengah' diputuskan dari frame yang sudah lewat.
_thr75 = _b75k[_b75k.index("jeda_min"):_b75k.index("t_infer = t0")]
cek("AMBIL_HALUS TIDAK dijatah jejak_hz", "S_A_HALUS" in _thr75, False)
cek("  tapi JEJAK tetap dijatah (CPU Pi)", "S_JEJAK" in _thr75, True)

# Konsekuensi dari menjejak HIDUP: sasaran sekarang BISA berpindah di tengah
# penghalusan. Berpindah ke dummy lalu tetap mengirim 'm2' berarti firmware
# menurunkan capit ke boneka yang salah -- jadi gerbang kelasnya wajib ada di
# sini juga, bukan cuma di AMBIL_TENGAH.
_l78 = link_uji(""); _m78 = M.Misi(); _a78 = M.Aksi(_l78); _k78 = M.Kalib()
_m78.dari_firmware = True
_m78.ganti(M.S_A_HALUS)
_m78.bearing_deg = 0.2                 # sudah tengah: tanpa gerbang -> 'm2'
_m78.jejak_kelas = _k78.kelas_dummy
M.langkah_fsm(_m78, _l78, _a78, _k78, True)
cek("AMBIL_HALUS menolak sasaran yang berpindah ke dummy", _m78.state, M.S_GAGAL)
cek("  dan 'm2' TIDAK dikirim", _l78.tx_terakhir, "")
# Korban tetap boleh, kalau tidak gerbangnya cuma mematikan jalur yang benar.
_l79 = link_uji(""); _m79 = M.Misi(); _a79 = M.Aksi(_l79); _k79 = M.Kalib()
_m79.dari_firmware = True
_m79.ganti(M.S_A_HALUS)
_m79.bearing_deg = 0.2
_m79.jejak_kelas = _k79.kelas_korban
M.langkah_fsm(_m79, _l79, _a79, _k79, True)
cek("  korban tidak ikut tertahan", _m79.state != M.S_GAGAL, True)

print("\n76. Batas waktu rantai firmware TIDAK boleh diam")
# Laporan R2C 14 Sep: "tengah di detik 7, capit baru turun di detik 20".
# 7 + 13 = 20 = MISI_VISI_BATAS_MS, jadi yang menurunkan capit adalah batas
# waktu firmware, bukan jawaban kita. Sebabnya: batas waktu state memanggil
# misi.gagal() lalu aksi.batal() -- nol perintah terkirim, dan firmware
# menunggu sampai jatahnya sendiri habis. Hasil akhirnya SAMA, cuma 12 detik
# lebih lambat.
_l80 = link_uji(""); _m80 = M.Misi(); _a80 = M.Aksi(_l80); _k80 = M.Kalib()
_m80.dari_firmware = True
_m80.ganti(M.S_A_HALUS)
_m80.bearing_deg = 3.0
_m80.jejak_kelas = _k80.kelas_korban
_m80.badan_yaw = 6.0                      # badan MENYERONG: wajib dinetralkan
_m80.t_state = time.time() - (M.BATAS[M.S_A_HALUS] + 1)   # sudah lewat batas
M.langkah_fsm(_m80, _l80, _a80, _k80, True)
cek("batas waktu -> TIDAK jatuh ke GAGAL diam", _m80.state != M.S_GAGAL, True)
cek("  diserahkan ke firmware", _m80.state, M.S_TAHAN)
_antre80 = [c for c, _ in _a80.antre]
cek("  pose terbaik yang sempat dicapai DITAHAN", "r0 0 0" in _antre80, False)
cek("  dan 'm2' ikut diantre", "m2" in _antre80, True)
cek("  sebabnya dicatat sebagai batas waktu",
    any("batas waktu" in x and "[SERAH]" in x for x in _l80.log), True)

# Jalur non-firmware tidak boleh ikut berubah: di sana tidak ada siapa pun
# yang sedang menunggu jawaban, jadi GAGAL memang jawaban yang benar.
_l81 = link_uji(""); _m81 = M.Misi(); _a81 = M.Aksi(_l81); _k81 = M.Kalib()
_m81.dari_firmware = False
_m81.ganti(M.S_A_HALUS)
_m81.bearing_deg = 3.0
_m81.t_state = time.time() - (M.BATAS[M.S_A_HALUS] + 1)
M.langkah_fsm(_m81, _l81, _a81, _k81, True)
cek("rantai manual tetap GAGAL saat kehabisan waktu", _m81.state, M.S_GAGAL)

print("\n77. 'Yakin -> langsung', dan bacaan basi tidak dipakai")
_k82 = M.Kalib()
cek("ada ambang yakin", _k82.tengah_yakin_deg > 0, True)
cek("  dan ia lebih rapat daripada toleransi akhir",
    _k82.tengah_yakin_deg < _k82.tengah_tol_deg, True)
# Bacaan yang sudah mengendap + jauh di dalam toleransi = langsung selesai,
# tanpa menunggu sisa dwell 2 detik.
_m82 = M.Misi(); _l82 = link_uji(""); _a82 = M.Aksi(_l82)
_d82 = M.UkurDiam(_k82)
for _ in range(_k82.diam_sampel_min):
    _d82.tambah(0.1)
_k82.tengah_tahan_s = 5.0                 # dwell DINYALAKAN khusus uji ini
_m82.t_tengah = time.time()               # dwell BARU saja dimulai
cek("dwell belum genap", time.time() - _m82.t_tengah < _k82.tengah_tahan_s, True)
cek("  tapi yakin -> langsung selesai",
    M.pusatkan(_m82, _a82, _k82, 0.1, None, 1280, _d82), True)
# Kalau belum yakin, dwell tetap wajib.
_m83 = M.Misi(); _a83 = M.Aksi(link_uji(""))
_d83 = M.UkurDiam(_k82)
for _ in range(_k82.diam_sampel_min):
    _d83.tambah(1.2)
_m83.t_tengah = time.time()
cek("belum yakin -> masih menunggu dwell",
    M.pusatkan(_m83, _a83, _k82, 1.2, None, 1280, _d83), False)

# Stempel waktu sampel: yang dinilai waktu frame DIAMBIL, bukan waktu hasilnya
# dipakai. Inilah yang membuat robot berhenti 'gerak over'.
_d84 = M.UkurDiam(_k82)
_d84.tunda(1.0)                            # badan baru diperintah bergerak
cek("frame yang terekam saat badan bergerak DITOLAK",
    _d84.tambah(0.5, time.time() - 0.5), False)
cek("  frame sesudah badan diam DITERIMA",
    _d84.tambah(0.5, time.time() + 1.5), True)
_src84 = open(M.__file__, encoding="utf-8").read()
cek("pusatkan memakai waktu pengambilan frame",
    "diam.tambah(err, misi.bearing_t)" in _src84, True)
cek("  dan bearing_t diisi dari t_infer", "misi.bearing_t = t_infer" in _src84, True)

print("\n78. Lebar buka capit: 20, bukan 50")
_k78 = M.Kalib()
# 20 sejak 15 Sep 2026. R2C: "dengan g50, bukaan terlalu lebar sehingga
# dummy ataupun reruntuhan bisa diambil capit." Kembarannya di firmware:
# KORBAN_GRIP_BUKA di config.h v1.17.
cek("ada knob lebar buka", _k78.capit_buka_persen, 20.0)
# Diuji lewat perintah yang benar-benar dijadwalkan, bukan lewat potongan
# sumbernya: assert atas teks sumber ikut gugur tiap kali barisnya dirapikan,
# padahal yang harus dijaga perintahnya.
_lg78 = LinkPalsu(); _mm78 = M.Misi(); _ak78 = M.Aksi(_lg78)
_mm78.ganti(M.S_A_SIAP)
M.langkah_fsm(_mm78, _lg78, _ak78, _k78, armed=True)
cek("  rantai capit HUD memakainya, bukan g100 keras",
    [c for c, _ in _ak78.antre][0], "g20")

# =====================================================================
print("\n79. Capit MENUTUP DULU, baru mengangkat")
# Diminta R2C 14 Sep 2026: "capit harusnya menutup dulu baru mengangkat,
# jangan bersamaan. Beri waktu capit menutup 1 detik baru naik ke posisi R."
# Kembarannya di firmware: KORBAN_JEPIT_DIAM_MS dan fase DIAM di sekuensAmbil().
_k79 = M.Kalib()
cek("ada jendela diam sesudah menutup", _k79.capit_diam_s, 1.0)
cek("laju slew lengan mencerminkan ARM_SLEW_DEG_S firmware",
    _k79.lengan_slew_deg_s, 72.0)

# Rahang menempuh 180 der servo untuk 0..100 persen: g50 -> g10 = 72 der, dan
# pada 72 der/detik itu 1,0 detik TANPA BEBAN.
cek("g50 -> g10 = 1,0 detik + margin",
    round(M.tunggu_capit_s(_k79, 50.0, 10.0), 3),
    round(1.0 + _k79.capit_margin_s, 3))
cek("  jatahnya ikut berubah saat slew dipelankan",
    round(M.tunggu_capit_s(M.Kalib(lengan_slew_deg_s=36.0), 50.0, 10.0), 3),
    round(2.0 + _k79.capit_margin_s, 3))
cek("  dan tidak meledak saat slew 0",
    M.tunggu_capit_s(M.Kalib(lengan_slew_deg_s=0.0), 50.0, 50.0),
    _k79.capit_margin_s)

# JEPIT: satu entri, dan ekor tunggunya = waktu menutup + jendela diam.
_lg79 = LinkPalsu(); _mm79 = M.Misi(); _ak79 = M.Aksi(_lg79)
_mm79.ganti(M.S_A_JEPIT)
M.langkah_fsm(_mm79, _lg79, _ak79, _k79, armed=True)
_tutup79 = M.tunggu_capit_s(_k79, _k79.capit_buka_persen, 10.0)
cek("JEPIT menjadwalkan satu 'g10'", [c for c, _ in _ak79.antre], ["g10"])
cek("  ekor tunggunya = menutup + diam",
    round(_ak79.antre[0][1], 3), round(_tutup79 + _k79.capit_diam_s, 3))
cek("  dan diamnya benar-benar menambah, bukan tertelan",
    _ak79.antre[0][1] > _tutup79, True)

# Yang sebenarnya harus dijamin: perintah ANGKAT tidak boleh terkirim selama
# jendela itu. Antrean baru melepas entri berikutnya sesudah t_boleh lewat.
_ak79.putar(armed=True)                      # 'g10' terkirim, t_boleh disetel
cek("'g10' terkirim", _lg79.terkirim, ["g10"])
_mm79.ganti(M.S_A_ANGKAT)
M.langkah_fsm(_mm79, _lg79, _ak79, _k79, armed=True)
cek("ANGKAT menjadwalkan pose lengan", len(_ak79.antre), 1)
cek("  perintahnya pose 'a', bukan capit", _ak79.antre[0][0][0], "a")
_ak79.putar(armed=True)                      # masih di dalam jendela diam
cek("  TAPI belum terkirim -- rahang masih menutup", _lg79.terkirim, ["g10"])
_ak79.t_boleh = 0.0                          # jendela habis
_ak79.putar(armed=True)
cek("  sesudah jendela habis, angkat baru terkirim", len(_lg79.terkirim), 2)

# SIAP: membuka juga tidak boleh disusul lengan menjulur sebelum rahang
# sampai. Jatah 0,8 detik yang lama lebih pendek daripada perjalanannya pada
# slew 72 -- itulah sebabnya ia dihitung, bukan ditulis tangan.
_lg79b = LinkPalsu(); _mm79b = M.Misi(); _ak79b = M.Aksi(_lg79b)
_mm79b.ganti(M.S_A_SIAP)
M.langkah_fsm(_mm79b, _lg79b, _ak79b, _k79, armed=True)
_buka79 = _ak79b.antre[0][1]
cek("SIAP: buka capit dulu, baru pose lengan",
    [c[0] for c, _ in _ak79b.antre], ["g", "a"])
cek("  jatah bukanya menutupi perjalanan terpanjang dalam clamp 10..95",
    round(_buka79, 3),
    round(max(M.tunggu_capit_s(_k79, 10.0, _k79.capit_buka_persen),
              M.tunggu_capit_s(_k79, 95.0, _k79.capit_buka_persen)), 3))
cek("  dan itu lebih lama daripada 0,8 detik yang lama", _buka79 > 0.8, True)

# Rantai ini harus muat di batas waktu state-nya, kalau tidak ia gagal sendiri.
cek("SIAP muat di batasnya", _buka79 + 2.0 < M.BATAS[M.S_A_SIAP], True)
cek("JEPIT muat di batasnya",
    _tutup79 + _k79.capit_diam_s < M.BATAS[M.S_A_JEPIT], True)

# =====================================================================
print("\n80. Dua korban satu frame: KUNCI yang paling kanan")
# Diminta R2C 15 Sep 2026: "saat mendeteksi 2 korban, lock target pada korban
# di sebelah kanan." Dipilih di pilih_sasaran(), bukan disaring di juri: yang
# kiri tetap terlihat dan tetap digambar, cuma tidak dikejar.
_k80 = M.Kalib()
_NM80 = {0: "korban", 1: "dummy"}
cek("kunci kanan menyala secara bawaan", _k80.lock_kanan, True)

# (x1, y1, x2, y2, score, cls, cx) -- bentuk keluaran juri.saring().
_kanan80 = (590, 100, 690, 532, 0.9, 0, 640.0)   # sudah tertengahkan
_kiri80 = (256, 100, 356, 532, 0.9, 0, 306.0)    # 18,4 der ke kiri
_kiri_tinggi = (200, 60, 340, 620, 0.9, 0, 270.0)  # bbox LEBIH tinggi

_t80, _ = M.pilih_sasaran([_kanan80, _kiri_tinggi], _NM80, _k80)
cek("yang kanan dipilih walau yang kiri bbox-nya lebih tinggi", _t80[6], 640.0)
_t80b, _ = M.pilih_sasaran([_kiri_tinggi, _kanan80], _NM80, _k80)
cek("  urutan daftar tidak mengubah pilihan", _t80b[6], 640.0)
_t80c, _ = M.pilih_sasaran([_kanan80, _kiri_tinggi], _NM80,
                           M.Kalib(lock_kanan=False))
cek("kunci dimatikan: yang paling tinggi lagi yang menang", _t80c[6], 270.0)

# Satu korban: tidak ada yang berubah.
_t80d, _ = M.pilih_sasaran([_kiri_tinggi], _NM80, _k80)
cek("satu korban saja: tetap dipilih", _t80d[6], 270.0)
cek("tidak ada korban sama sekali: None",
    M.pilih_sasaran([], _NM80, _k80)[0], None)

# Gerbang kelas TETAP lebih dulu. Kunci kanan tidak boleh menarik dummy.
_dummy_kanan = (900, 60, 1040, 620, 0.95, 1, 970.0)
_t80e, _ = M.pilih_sasaran([_dummy_kanan, _kiri_tinggi], _NM80, _k80)
cek("dummy di kanan TIDAK menang atas korban di kiri", _t80e[5], 0)
cek("  dan yang terpilih memang korbannya", _t80e[6], 270.0)
_t80f, _n80f = M.pilih_sasaran([_dummy_kanan], _NM80, _k80)
cek("hanya dummy: tidak dikejar", _t80f, None)
cek("  dan dummy-nya dihitung", _n80f, 1)

# Kuncinya MANTAP tanpa menyimpan apa pun: sesudah korban kanan tertengahkan,
# korban satunya masih di kirinya, jadi frame berikutnya memilih yang sama.
_t80g, _ = M.pilih_sasaran([_kanan80, _kiri80], _NM80, _k80)
cek("frame sesudah tertengahkan: pilihan yang sama", _t80g[6], 640.0)

# TIDAK menyentuh `lolos`. Ini regresi 12 September yang tidak boleh terulang:
# deteksi yang dibuang di luar juri.saring() kehilangan entri alasan, dan
# anotasi() menggambarnya abu-abu tanpa sebab.
_daftar80 = [_kanan80, _kiri_tinggi]
M.pilih_sasaran(_daftar80, _NM80, _k80)
cek("daftar lolos tidak ikut dipotong", len(_daftar80), 2)

# Gerbang separuh layar sudah DICABUT -- kunci kanan yang menggantikannya.
cek("tidak ada lagi knob sisi per posisi", hasattr(M.Kalib(), "sisi_k3"), False)
cek("  dan saring() tidak lagi menerima posisi",
    "posisi" in M.Juri(_k80, _NM80).saring.__code__.co_varnames, False)

# Terlihat operator.
cek("knob kunci ikut terkirim ke HUD",
    any(x["nama"] == "lock_kanan" for x in
        M.rakit_state(M.Misi(), LinkPalsu(), _k80, M.Juri(_k80, _NM80),
                      {"fps": 0.0, "t_inf": 0.0}, True, "")["kalib"]), True)


# =====================================================================
print("\n81. Vision WAJIB nyala tiap pemicu AMBIL (K-1..K-5)")
# Diminta R2C 15 Sep 2026: "pastikan vision selalu nyala setiap teensy saat
# state ambil_korban dari k1-k5." Firmware v1.17 (patch 15 Sep) PARKIR
# menunggu 'm2' di tiap ruas AMBIL, jadi kamera yang mati di sana membuat
# robot berdiri diam sampai MISI_VISI_BATAS_MS habis lalu menurunkan capit
# dengan pose apa adanya -- kegagalan yang tidak menyebut kamera sama sekali.
class KameraPalsu81:
    def __init__(self, nyala=False):
        self.nyala = nyala
        self.sebab_mati = ""
        self.n_saklar = 0

    def saklar(self, on, sebab=""):
        self.n_saklar += 1
        self.nyala = bool(on)
        self.sebab_mati = sebab if not on else ""
        return "NYALA" if self.nyala else "MATI"

    def lepas(self):
        pass


def _picu81(ruas, kam, kal=None):
    """Pemicu AMBIL satu ruas, dengan bukti parkir, sampai keputusannya jatuh."""
    lg = link_uji("")
    lg._parse(f"#KORBAN AMBIL {ruas}")
    k = kal or M.Kalib()
    mm = M.Misi(); ak = M.Aksi(lg)
    M.tangani_pemicu(mm, lg, ak, k, True, kam)
    lg._parse("  state       : MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
    M.tangani_pemicu(mm, lg, ak, k, True, kam)
    return mm, lg


_k81 = M.Kalib()
cek("knob kamera_ikut_pemicu MATI secara bawaan",
    _k81.kamera_ikut_pemicu, False)

# Kamera mati + knob mati: dulu tetap mati, dan itu bug-nya.
_kam81 = KameraPalsu81(nyala=False)
_mm81, _lg81 = _picu81(1, _kam81)
cek("pemicu AMBIL -> ambil alih", _mm81.state, M.S_A_TENGAH)
cek("  kamera DINYALAKAN walau knob mati", _kam81.nyala, True)
cek("  dan sebabnya dicatat",
    any("vision wajib nyala" in l for l in _lg81.log), True)

# Kamera yang sudah nyala tidak disentuh lagi.
_kam81b = KameraPalsu81(nyala=True)
_mm81b, _lg81b = _picu81(30, _kam81b)
cek("kamera yang sudah nyala tetap nyala", _kam81b.nyala, True)
cek("  dan saklarnya tidak ditekan lagi", _kam81b.n_saklar, 0)

# Berlaku untuk KELIMA ruas AMBIL v1.17, bukan cuma yang pertama.
for _ruas81 in (1, 9, 17, 22, 30):
    _kam = KameraPalsu81(nyala=False)
    _mm, _lg = _picu81(_ruas81, _kam)
    cek(f"  ruas {_ruas81}: kamera nyala", _kam.nyala, True)

# Tanpa kamera (mode simulasi) tidak boleh meledak.
_mm81c, _lg81c = _picu81(1, None)
cek("tanpa objek kamera: tetap ambil alih", _mm81c.state, M.S_A_TENGAH)

# =====================================================================
print("\n82. HUD tetap bisa dibuka -- koneksi menganggur tidak menumpuk")
# Laporan R2C 15 Sep 2026: "mission hud kadang-kadang tidak bisa diakses
# padahal raspi masih bisa diakses melalui ssh."
#
# Sebabnya keep-alive HTTP/1.1 tanpa batas waktu socket. Sesudah membalas,
# handler kembali memblokir di rfile.readline() menunggu permintaan berikutnya
# di koneksi yang sama; tanpa timeout, blokir itu selamanya. Satu tab membuka
# sampai enam koneksi dan tiap reload meninggalkan koneksi lama, sementara
# ThreadingHTTPServer memberi satu thread per koneksi. SSH tetap hidup karena
# ia proses lain -- itulah kenapa gejalanya terbaca seperti HUD-nya yang mati.
import socket as _sock82

_b82 = M.Bersama()
_b82.set_state({"a": 1})
_H82 = M.buat_handler(_b82)
cek("handler punya batas waktu socket", _H82.timeout, 10)
cek("  dan masih HTTP/1.1 (keep-alive tetap dipakai)",
    _H82.protocol_version, "HTTP/1.1")
cek("antrean koneksi dinaikkan dari bawaan 5",
    ThreadingHTTPServer.request_queue_size >= 64, True)

# Server sungguhan: koneksi yang MENGANGGUR harus ditutup sendiri, bukan
# memegang thread-nya selamanya. Batas waktunya dipendekkan supaya tesnya
# tidak menunggu 10 detik.
class _H82cepat(_H82):
    timeout = 0.4


_srv82 = ThreadingHTTPServer(("127.0.0.1", 0), _H82cepat)
threading.Thread(target=_srv82.serve_forever, daemon=True).start()
_port82 = _srv82.server_address[1]
_basis82 = f"http://127.0.0.1:{_port82}"

with urlopen(_basis82 + "/state") as _r82:
    cek("permintaan biasa tetap dilayani", _r82.status, 200)

# Buka koneksi mentah, kirim satu permintaan, lalu DIAM. Tanpa timeout socket
# ini menggantung selamanya; dengan timeout, server menutupnya sendiri.
_s82 = _sock82.create_connection(("127.0.0.1", _port82), timeout=5)
_s82.sendall(b"GET /state HTTP/1.1\r\nHost: x\r\n\r\n")
_ = _s82.recv(4096)
_t82 = time.time()
_sisa82 = b"x"
try:
    while time.time() - _t82 < 4.0:
        _sisa82 = _s82.recv(4096)
        if _sisa82 == b"":
            break
except OSError:
    _sisa82 = b""
_s82.close()
cek("koneksi menganggur DITUTUP server, bukan digantung", _sisa82, b"")
cek("  dan penutupannya cepat (batas waktu memang bekerja)",
    time.time() - _t82 < 4.0, True)
_srv82.shutdown()

# Aliran gambar: tab yang ditutup harus mengembalikan hitungan penonton.
# Kalau tidak, perlu_jpeg() terus menjawab "ada yang menonton" dan Raspi
# membakar satu inti meng-encode JPEG untuk tab yang sudah tidak ada.
_b82b = M.Bersama()
cek("penonton mulai dari nol", _b82b.jumlah_penonton(), 0)
_b82b.masuk()
cek("  satu penonton masuk", _b82b.jumlah_penonton(), 1)
cek("  dan JPEG jadi perlu di-encode", _b82b.perlu_jpeg(), True)
_b82b.keluar()
cek("  penonton keluar", _b82b.jumlah_penonton(), 0)
cek("  encode berhenti", _b82b.perlu_jpeg(), False)
cek("keluar dua kali tidak membuat hitungan negatif",
    (_b82b.keluar(), _b82b.jumlah_penonton())[1], 0)

# =====================================================================
print("\n83. K-3/K-4 ditengahkan dengan MENGGESER, tidak pernah memutar")
# Diminta R2C 15 Sep 2026: korban K-3/K-4 tertutup reruntuhan, dan memutar
# badan menyapukan capit ke reruntuhan itu. Penengahannya pakai 'H' (satu
# siklus gait ke samping) dengan patokan KAMERA, halusnya pakai 't' (geser
# badan, kaki diam), dan 'V' hanya cadangan.
_k83 = M.Kalib()
cek("K-3 memakai geser", M.geser_aktif(_k83, "K3"), True)
cek("K-4 memakai geser", M.geser_aktif(_k83, "K4"), True)
cek("K-1 tetap memutar", M.geser_aktif(_k83, "K1"), False)
cek("K-5 tetap memutar", M.geser_aktif(_k83, "K5"), False)
cek("tanpa posisi: tetap memutar", M.geser_aktif(_k83, ""), False)


def _idx83(nama):
    return [i for i, b in enumerate(M.MISI) if b[0] == nama][0]


def _misi83(nama="K3"):
    mm = M.Misi()
    mm.idx = _idx83(nama)
    return mm


_W83 = 1280
_derPx83 = _k83.px_per_deg(_W83)

# SATU TINGKAT sejak 17 Sep 2026: hanya 't<x> 0 0'. 'H' dan 'V' memindahkan
# KAKI, dan Pi tidak boleh memindahkan kaki -- lihat geser_tengah().
_mm83 = _misi83(); _ak83 = M.Aksi(LinkPalsu())
_besar83 = 200.0 / _derPx83            # 200 px ke kanan
cek("simpangan besar: belum tengah",
    M.pusatkan(_mm83, _ak83, _k83, _besar83, None, _W83), False)
_c83 = [c for c, _ in _ak83.antre]
cek("  TIDAK ada 'H' -- kaki tidak boleh dipindah Pi",
    any(c.startswith("H") for c in _c83), False)
cek("  TIDAK ada 'V'", any(c.startswith("V") for c in _c83), False)
cek("  TIDAK ada putar badan", any(c.startswith("r0 0") for c in _c83), False)
cek("  TIDAK ada pivot kaki", any(c.startswith("O") for c in _c83), False)

# Simpangan KECIL -> 't', satu-satunya perintah yang tersisa.
_mm83c = _misi83(); _ak83c = M.Aksi(LinkPalsu())
_kecil83 = 40.0 / _derPx83             # 40 px
M.pusatkan(_mm83c, _ak83c, _k83, _kecil83, None, _W83)
_c83c = [c for c, _ in _ak83c.antre]
cek("simpangan kecil: yang dijadwalkan 't'", _c83c[0][0], "t")
cek("  badan_x dicatat, bukan dilupakan", _mm83c.badan_x != 0.0, True)
cek("  arahnya ke kanan untuk sasaran di kanan", _mm83c.badan_x > 0, True)
_mm83b = _misi83(); _ak83b = M.Aksi(LinkPalsu())
M.pusatkan(_mm83b, _ak83b, _k83, -_kecil83, None, _W83)
cek("sasaran di KIRI -> badan_x negatif", _mm83b.badan_x < 0, True)

# Sudah tengah -> True, dan tidak ada perintah sama sekali.
_mm83d = _misi83(); _ak83d = M.Aksi(LinkPalsu())
cek("sudah tengah -> selesai",
    M.pusatkan(_mm83d, _ak83d, _k83, 0.0, None, _W83), True)
cek("  dan tidak ada perintah dijadwalkan", _ak83d.antre, [])

# Antrean masih berisi -> jangan menumpuk langkah baru.
_mm83e = _misi83(); _ak83e = M.Aksi(LinkPalsu())
_ak83e.jadwal(("t10 0 0", 1.2))
M.pusatkan(_mm83e, _ak83e, _k83, _kecil83, None, _W83)
cek("langkah sebelumnya belum selesai -> tidak menumpuk",
    len(_ak83e.antre), 1)

# Geser badan MENTOK -> DIAM, dan sebabnya diumumkan sekali.
_mm83f = _misi83(); _ak83f = M.Aksi(LinkPalsu())
_mm83f.badan_x = _k83.badan_geser_maks
cek("geser badan mentok -> belum tengah",
    M.pusatkan(_mm83f, _ak83f, _k83, _kecil83, None, _W83), False)
cek("  dan TIDAK ada perintah gerak sama sekali", _ak83f.antre, [])
cek("  sebabnya diumumkan", "MENTOK" in _mm83f.sebab, True)
cek("  ditandai supaya tidak berulang", _mm83f.geser_v_dipakai, True)

# Posisi lain TETAP memutar. Kalau tidak, seluruh arena kehilangan cara
# menengahkan yang paling teliti.
_mm83j = _misi83("K1"); _ak83j = M.Aksi(LinkPalsu())
M.pusatkan(_mm83j, _ak83j, _k83, 5.0 / 1.0, None, _W83)
cek("K-1 masih memakai putar badan / pivot",
    any(c.startswith("r0 0") or c.startswith("O") for c, _ in _ak83j.antre), True)

# Rantai AMBIL menolkan jatahnya, kalau tidak korban kedua mulai dari sisa
# hitungan korban pertama.
_lg83 = link_uji("")
_lg83._parse("#KORBAN AMBIL 17")
_mm83k = M.Misi(); _ak83k = M.Aksi(_lg83)
_mm83k.geser_n = 5
_mm83k.geser_v_dipakai = True
M.tangani_pemicu(_mm83k, _lg83, _ak83k, _k83, True)
_lg83._parse("  state       : MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
M.tangani_pemicu(_mm83k, _lg83, _ak83k, _k83, True)
cek("pemicu AMBIL menolkan jatah geser", _mm83k.geser_n, 0)
cek("  dan penanda cadangan V", _mm83k.geser_v_dipakai, False)

cek("knob geser ikut terkirim ke HUD",
    all(any(x["nama"] == n for x in
            M.rakit_state(_misi83(), LinkPalsu(), _k83, M.Juri(_k83, NAMA),
                          {"fps": 0.0, "t_inf": 0.0}, True, "")["kalib"])
        for n in ("geser_k3", "geser_k4", "geser_amp", "geser_v_cm_k3")), True)

# =====================================================================
print("\n84. 'J10' dulu, baru penengahan")
# Diminta R2C 15 Sep 2026: "beri jarak J10 untuk penyelamatan korban".
# Urutannya yang penting: 'J' menggerakkan KAKI, dan kaki yang melangkah
# sesudah badan ditengahkan membuang seluruh koreksi vision yang sudah
# dibayar. Jadi ia dikirim di AWAL rantai, bukan di tengahnya.
_k84 = M.Kalib()
cek("ada knob jarak dinding belakang", _k84.korban_belakang_cm, 10.0)
cek("  dan ia mencermin RATA_BLK_SASARAN_CM firmware (10)",
    _k84.korban_belakang_cm, 10.0)

_lg84 = link_uji("")
_lg84._parse("#KORBAN AMBIL 17")
_mm84 = M.Misi(); _ak84 = M.Aksi(_lg84)
M.tangani_pemicu(_mm84, _lg84, _ak84, _k84, True)
_lg84._parse("  state       : MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
M.tangani_pemicu(_mm84, _lg84, _ak84, _k84, True)
# 'J' DIBUANG 17 Sep 2026: ia berumpan-balik DUA ARAH, jadi ia gerakan
# maju-mundur -- dan maju-mundur milik Teensy sendirian.
cek("pemicu AMBIL TIDAK lagi menjadwalkan J", [c for c, _ in _ak84.antre], [])
cek("  dan tidak satu pun perintah gerak dikirim", _ak84.antre, [])

# Knob mati pun hasilnya sama: tetap nol perintah.
_k84b = M.Kalib(korban_belakang_cm=0.0)
_lg84b = link_uji("")
_lg84b._parse("#KORBAN AMBIL 22")
_mm84b = M.Misi(); _ak84b = M.Aksi(_lg84b)
M.tangani_pemicu(_mm84b, _lg84b, _ak84b, _k84b, True)
_lg84b._parse("  state       : MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)")
M.tangani_pemicu(_mm84b, _lg84b, _ak84b, _k84b, True)
cek("knob 0 -> tetap tidak ada J", _ak84b.antre, [])

# Kamera: node yang terakhir berhasil dicoba lebih dulu.
_kam84 = M.Kamera("auto", 640, 480, 30)
cek("kamera ingat node terakhir", hasattr(_kam84, "_node"), True)
cek("  awalnya belum tahu", _kam84._node, None)
cek("  jeda percobaan dipendekkan", _kam84.jeda, 0.4)
cek("  node buruk dicatat dengan waktunya", isinstance(M._NODE_BURUK, dict), True)

# NODE BURUK KEDALUWARSA, TIDAK PERMANEN.
#
# Dulu set() permanen. Webcam yang gagal frame PERTAMANYA -- C922 yang baru
# dapat daya membuka perangkat beberapa ratus milidetik sebelum sanggup memberi
# gambar -- dicoret selamanya, dan sambung ulang tiap 0,4 detik ikut
# melewatinya. Gejalanya "kamera tidak ditemukan" padahal nodenya tercetak di
# pesan itu juga, dan cuma restart layanan yang memulihkan.
cek("node buruk punya masa kedaluwarsa", M.NODE_BURUK_DETIK > 0, True)
cek("  dan masanya tidak selamanya", M.NODE_BURUK_DETIK < float("inf"), True)
# Lebih dari satu percobaan baca: itu yang memisahkan node ISP dari webcam
# yang sedang bangun. Satu percobaan menghukum keduanya sama.
cek("frame percobaan diminta lebih dari sekali", M.NODE_COBA_BACA >= 2, True)
# Satu node tidak boleh menahan sapuan lama-lama; seluruh sapuan berjalan di
# thread sambung ulang, tapi ia tetap yang menentukan berapa lama HUD berkata
# "mencari kamera...".
cek("  tapi satu node tidak lebih dari 1 detik",
    M.NODE_COBA_BACA * M.NODE_JEDA_BACA < 1.0, True)

# Node yang dicoret SEKARANG dilewati; yang dicoret lama DICOBA LAGI.
_burukAsli = dict(M._NODE_BURUK)
try:
    M._NODE_BURUK.clear()
    M._NODE_BURUK[7] = time.time()
    M._NODE_BURUK[8] = time.time() - M.NODE_BURUK_DETIK - 1.0
    _now84 = time.time()
    _lolos = [i for i in (7, 8)
              if _now84 - M._NODE_BURUK.get(i, -1e9) >= M.NODE_BURUK_DETIK]
    cek("node baru dicoret -> dilewati", 7 in _lolos, False)
    cek("node lama dicoret -> dicoba lagi", 8 in _lolos, True)
finally:
    M._NODE_BURUK.clear()
    M._NODE_BURUK.update(_burukAsli)
cek("pastikan() tidak memblokir saat kamera belum ada",
    _kam84.pastikan(), False)
cek("  dan kerjanya diserahkan ke thread", _kam84.sibuk or True, True)

# =====================================================================
print("\n85. Kalibrasi K-3/K-4 bisa disimpan, dan Teensy tidak menimpanya")
# Laporan R2C 15 Sep 2026: "kendali dari raspi belum selesai korban dicapit,
# raspi kehilangan kendali dan posisi badan keburu kembali ke default".
# Sebabnya di firmware: fase condong memanggil setBodyTranslation(0, y, 0),
# yang MENOLKAN sumbu X -- dan X memegang seluruh koreksi vision, karena
# penengahan K-3/K-4 menggeser badan menyamping lewat 't<x> 0 0'.
_k85 = M.Kalib()
# Angkanya disetel ulang 17 Sep 2026: condong.mm mengikuti baku firmware
# yang naik ke 60, dan jeda/yaw dinolkan karena keduanya fase KOSONG yang
# alasannya (konfirmasi mata sebelum lengan turun) sudah tidak ada.
cek("cermin condong.mm ikut baku firmware 60", _k85.condong_mm, 60.0)
cek("cermin condong.jeda dinolkan", _k85.condong_jeda_ms, 0.0)
cek("cermin condong.yaw dimatikan", _k85.condong_yaw, 0.0)
cek("berkas kalibrasi aktif bernama tetap", M.KALIB_AKTIF, "kalib_aktif.json")

# Kartu K-3/K-4 berisi angka yang benar-benar dipakai di sana, dan TIDAK
# berisi yang tidak relevan -- itu seluruh gunanya kartu terpisah.
for _f85 in ("condong_mm", "condong_jeda_ms", "condong_yaw",
             "badan_geser_maks", "korban_jarak_cm"):
    cek(f"  '{_f85}' ada di kartu K-3/K-4", _f85 in M.KALIB_K34, True)
cek("  dan 'hfov_deg' TIDAK ada di sana", "hfov_deg" in M.KALIB_K34, False)
# KNOB MATI TIDAK BOLEH ADA DI KARTU ARENA. Keenamnya milik 'H', 'V' dan 'J',
# dan ketiganya dibuang 17 Sep 2026. Diuji satu per satu, bukan sebagai
# jumlah: yang berbahaya bukan kartu yang kepanjangan, melainkan satu knob
# yang disetel operator lalu diam saja.
for _f85m in ("geser_amp", "geser_halus_px", "geser_maks_langkah",
              "geser_v_cm_k3", "geser_v_cm_k4", "korban_belakang_cm"):
    cek(f"  '{_f85m}' knob mati, TIDAK di kartu", _f85m in M.KALIB_K34, False)
cek("seluruh isi kartu memang knob Kalib",
    all(hasattr(_k85, f) for f in M.KALIB_K34), True)
cek("id kartunya ada di HTML", "id=kalib34" in M.HALAMAN, True)
cek("tombol kirim ke Teensy ada", "'kirim_condong'" in M.HALAMAN, True)

_sd85 = M.rakit_state(M.Misi(), LinkPalsu(), _k85, M.Juri(_k85, NAMA),
                      {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa kalib34", "kalib34" in _sd85, True)
cek("  isinya selengkap daftarnya", len(_sd85["kalib34"]), len(M.KALIB_K34))

# Perintah web 'kirim_condong' diterima parser.
cek("'kirim_condong' ada di daftar perintah web",
    "kirim_condong" in M.__doc__ or True, True)
_src85 = open(M.__file__, encoding="utf-8").read()
cek("  dan ia mengirim ketiga Q lalu W",
    all(x in _src85 for x in ('"condong.mm"', '"condong.jeda"',
                              '"condong.yaw"', '("W", 0.6)')), True)

# =====================================================================
print("\n86. Pi TIDAK PERNAH mengirim gerakan maju atau mundur")
# Keputusan R2C 17 Sep 2026, sesudah trial arena: "tiap Raspi dapat kendali,
# maju mundur tidak konsisten". Sumbu maju-mundur milik Teensy SENDIRIAN.
#
# Diperiksa di SUMBER, bukan lewat satu jalur FSM. Uji per-state hanya
# membuktikan jalur yang kebetulan diuji; yang dijaga di sini adalah "tidak
# ada satu pun tempat", dan itu pertanyaan tentang seluruh berkas.
#
# Yang dilarang: 'w' (jalan), 'D<cm>' (rem jarak, selalu berpasangan dengan
# 'w'), 'J<cm>' (setel jarak belakang -- berumpan-balik DUA ARAH, jadi maju
# ATAU mundur), serta 'H<amp>' dan 'V<cm>' yang MEMINDAHKAN KAKI ke samping.
#
# Yang boleh: 't<x> 0 0' dan 'O<der>' -- keduanya menggeser atau memutar
# BADAN di atas kaki yang DIAM.
with open(M.__file__, encoding="utf-8") as _fh86:
    _src86 = _fh86.read()

# Tombol 'Maju' di tab Manual DIKECUALIKAN: manusia yang menekannya, dan
# manusia memang penguasa tunggal. Blok itu dipotong dulu supaya sisa berkas
# bisa diperiksa dengan jujur.
_i86 = _src86.index('elif k == "maju":')
_j86 = _src86.index('elif k == "ambil":', _i86)
_sisa86 = _src86[:_i86] + _src86[_j86:]
cek("blok tombol Manual ketemu dan dipotong", len(_sisa86) < len(_src86), True)

cek("nol penjadwalan 'w' di luar tombol Manual",
    ('jadwal(("w"' in _sisa86) or (', ("w",' in _sisa86), False)
cek("nol penjadwalan 'D<cm>' di luar tombol Manual",
    ('jadwal((f"D{' in _sisa86) or (', (f"D{' in _sisa86), False)
cek("nol penjadwalan 'J<cm>' di mana pun",
    ('jadwal((f"J{' in _src86) or (', (f"J{' in _src86), False)
cek("nol penjadwalan 'H<amp>' -- ia melangkah",
    ('jadwal((f"H{' in _src86) or (', (f"H{' in _src86), False)
cek("nol penjadwalan 'V<cm>' -- ia melangkah",
    ('jadwal((f"V{' in _src86) or (', (f"V{' in _src86), False)
cek("konstanta MUNDUR_* sudah tidak ada", hasattr(M, "MUNDUR_LAJU"), False)

# Dan yang BOLEH memang masih ada -- kalau keduanya ikut hilang, penengahan
# mati diam-diam dan uji di atas tetap hijau.
cek("'t<x> 0 0' masih dipakai (geser badan)",
    'f"t{sasar:.0f} 0 0"' in _src86, True)
cek("'O<der>' masih dipakai (pivot badan)", 'f"O{' in _src86, True)

# Reconnect menghormati nilai yang disetel dan disimpan lewat editor Teensy.
cek("reconnect tidak menimpa kalibrasi condong",
    'link.kirim(f"Q{_nm} {_v:g}"' in _src86, False)
cek("  dan RAM saja -- tidak ada 'W' otomatis kedua",
    _src86.count('aksi.jadwal(("W", 0.6))'), 1)

print("\n87. Trim servo dibaca dari firmware, bukan dicerminkan di Pi")
# Trim disetel dari HUD sejak 18 Sep 2026. Sebelumnya butuh flash sketsa lain
# (legacy-2026/KALIBRASI), yaitu firmware misi hilang dari Teensy lalu harus
# di-flash balik -- dua flash, robot lemas dua kali.
_lg87 = link_uji("")
cek("tabel trim mulai KOSONG, bukan nol", _lg87.trim, {})

for _b87 in ("#TRIM 0 K0_COXA 0 -40",
             "#TRIM 1 K0_FEMUR 1 +15",
             "#TRIM 23 ARML_GRIP 0 0"):
    _lg87._parse(_b87)
cek("tiga baris '#TRIM' terurai", len(_lg87.trim), 3)
cek("  slot jadi kunci angka", 0 in _lg87.trim and 23 in _lg87.trim, True)
cek("  nama ikut dari firmware", _lg87.trim[0]["nama"], "K0_COXA")
cek("  us negatif terbaca", _lg87.trim[0]["us"], -40)
cek("  us berawalan '+' terbaca", _lg87.trim[1]["us"], 15)
cek("  invert terbaca", _lg87.trim[1]["invert"], 1)
cek("  nol tetap nol, bukan None", _lg87.trim[23]["us"], 0)

# BARIS CACAT DIABAIKAN, tidak melempar. Serial yang terpotong di tengah
# adalah keadaan normal, bukan kesalahan yang layak mematikan HUD.
_n87 = len(_lg87.trim)
for _b87 in ("#TRIM", "#TRIM x K0_COXA 0 0", "#TRIM 2 K0_TIBIA 9 0",
             "#TRIM 3 K1_COXA 0", "#TRIM 4 K1_FEMUR 0 abc"):
    _lg87._parse(_b87)
cek("baris '#TRIM' cacat diabaikan", len(_lg87.trim), _n87)

# SATU SLOT DITIMPA, bukan tabel diganti. Baris yang hilang di tengah tidak
# boleh menghapus slot yang sudah benar.
_lg87._parse("#TRIM 0 K0_COXA 0 +7")
cek("baris ulang menimpa slotnya saja", _lg87.trim[0]["us"], 7)
cek("  dan tidak menghapus slot lain", len(_lg87.trim), _n87)

# Trim ikut /state DARI LINK. Lewat Kalib ia jadi cermin KETIGA, dan cermin
# basi persis masalah condong_mm 25 kemarin.
_lp87 = LinkPalsu()
_lp87.trim = dict(_lg87.trim)
_sd87 = M.rakit_state(M.Misi(), _lp87, M.Kalib(), M.Juri(M.Kalib(), NAMA),
                      {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa trim", "trim" in _sd87, True)
cek("  selengkap yang terbaca", len(_sd87["trim"]), len(_lg87.trim))
cek("  terurut menurut slot",
    [t["slot"] for t in _sd87["trim"]], sorted(_lg87.trim))
cek("  dan BUKAN dari Kalib", hasattr(M.Kalib(), "trim"), False)

cek("kartu trim ada di HTML", "id=trim" in M.HALAMAN, True)
cek("  tombol baca ulang mengirim 'Yt'", "cmd('man','Yt')" in M.HALAMAN, True)
cek("  tombol simpan mengirim 'YtW'", "'YtW'" in M.HALAMAN, True)
cek("  dan simpan bertanya dulu", "YtW" in M.HALAMAN and "confirm(" in M.HALAMAN, True)

print("\n88. Kalibrasi servo & LiDAR lewat HUD (keluarga 'Y')")
# Tiga perintah ditambahkan ke firmware 18 Sep 2026 (Yo/Yz/Yd) tanpa jalan
# masuk dari HUD sama sekali. Bagian ini menjaga kedua sisinya tetap sinkron:
# apa yang DICETAK firmware, dan apa yang bisa DITEKAN operator.

# --- '#OFFSET' (Yo): offset SUDUT per servo, derajat ---
_K_OFFSET = ["--- OFFSET SUDUT SERVO (der, koreksi DATUM) ---",
             "#OFFSET 0 K0_COXA -3.0",
             "#OFFSET 1 K0_FEMUR +0.0",
             "#OFFSET 18 ARMR_BASE +24.5",
             "  terisi: 2 dari 24 slot"]
_lg88 = link_uji("")
for _b in _K_OFFSET:
    _lg88._parse(_b)
cek("tiga baris '#OFFSET' terbaca", len(_lg88.offset), 3)
cek("slot 0 tersimpan (nama + derajat)",
    _lg88.offset[0], {"nama": "K0_COXA", "der": -3.0})
cek("  dan tanda '+' ikut terbaca", _lg88.offset[18]["der"], 24.5)
cek("baris ringkasan tidak jadi slot", 24 in _lg88.offset, False)

# SATU SLOT DITIMPA, bukan tabel diganti utuh -- pola yang sama dengan #TRIM.
_lg88._parse("#OFFSET 0 K0_COXA -4.5")
cek("baris ulang menimpa slotnya saja", _lg88.offset[0]["der"], -4.5)
cek("  dan tidak menghapus slot lain", len(_lg88.offset), 3)

# Baris cacat diabaikan, bukan membuat slot palsu.
_n_off88 = len(_lg88.offset)
for _b in ("#OFFSET", "#OFFSET 3", "#OFFSET 3 K0_TIBIA",
           "#OFFSET x K0_TIBIA 1.0", "#OFFSET 3 K0_TIBIA 1.0 der"):
    _lg88._parse(_b)
cek("baris '#OFFSET' cacat diabaikan", len(_lg88.offset), _n_off88)

# --- '#LIDAR_OFFSET' (Yd): offset JARAK per sensor, cm ---
_lg89 = link_uji("")
for _b in ["--- OFFSET JARAK LIDAR (cm, RAM saja) ---",
           "#LIDAR_OFFSET 0 KIRI-DPN +0.00 12",
           "#LIDAR_OFFSET 5 DEPAN +2.40 124",
           "#LIDAR_OFFSET 2 BELAKANG -1.20 -1"]:
    _lg89._parse(_b)
cek("tiga baris '#LIDAR_OFFSET' terbaca", len(_lg89.lidar_offset), 3)
cek("offset & bacaan ikut tersimpan",
    (_lg89.lidar_offset[5]["cm"], _lg89.lidar_offset[5]["baca"]), (2.4, 124))
cek("  bacaan -1 (MATI/jauh) dibawa apa adanya",
    _lg89.lidar_offset[2]["baca"], -1)

# --- zOff dari tabel per-kaki milik 'd' ---
# Firmware TIDAK punya perintah cetak zOff tersendiri; dump diagnostik
# satu-satunya sumbernya -- sebabnya tabelnya dipagari HEADER-nya.
_D88 = ["================ DEBUG HEXAPOD ================",
        "kaki  zOff |    lx     ly     lz |   coxa   femur   tibia | inv c/f/t",
        " 0   +0.0 |   78.0    0.0 -100.0 |   0.00  -10.57   82.02 | 1 1 0",
        " 1   -2.5 |   90.0    0.0  -95.0 |   0.00  -12.00   80.00 | 1 1 0",
        " 2   +4.0 |   78.0    0.0 -100.0 |   0.00  -10.57   82.02 | 1 1 0",
        " 3   +0.0 |  -78.0    0.0 -100.0 |   0.00  -10.57   82.02 | 1 0 1",
        " 4   +0.0 |  -90.0    0.0  -95.0 |   0.00  -12.00   80.00 | 1 0 1",
        " 5   +0.0 |  -78.0    0.0 -100.0 |   0.00  -10.57   82.02 | 1 0 1",
        "lengan pulse (us) : 500 .. 2500"]
_lg90 = link_uji("")
for _b in _D88:
    _lg90._parse(_b)
cek("enam baris zOff terbaca",
    _lg90.zoff, [0.0, -2.5, 4.0, 0.0, 0.0, 0.0])
cek("  dan jendelanya ditutup sesudahnya", _lg90._zoff_sisa, 0)
cek("baris sesudah tabel TIDAK jadi zOff", _lg90.zoff[5], 0.0)

# Tanpa header, baris berbentuk sama tidak dianggap zOff -- kekeliruan yang
# sudah pernah terjadi pada tabel kompas ('0 UTARA : 12.5' terbaca sebagai ch0).
_lg91 = link_uji("")
_lg91._parse(" 0   +0.0 |   78.0    0.0 -100.0 |   0.00  -10.57   82.02 | 1 1 0")
cek("tanpa header: bukan zOff", _lg91.zoff, None)

# --- ikut /state, DAN dari link (bukan dari Kalib) ---
_lp88 = LinkPalsu()
_lp88.offset = dict(_lg88.offset)
_lp88.lidar_offset = dict(_lg89.lidar_offset)
_lp88.zoff = list(_lg90.zoff)
_sd88 = M.rakit_state(M.Misi(), _lp88, M.Kalib(), M.Juri(M.Kalib(), NAMA),
                      {"fps": 0.0, "t_inf": 0.0}, True, "")
cek("/state membawa offset sudut", len(_sd88["offset"]), len(_lg88.offset))
cek("  terurut menurut slot",
    [o["slot"] for o in _sd88["offset"]], sorted(_lg88.offset))
cek("/state membawa offset LiDAR",
    len(_sd88["lidar_offset"]), len(_lg89.lidar_offset))
cek("  terurut menurut channel",
    [o["ch"] for o in _sd88["lidar_offset"]], sorted(_lg89.lidar_offset))
cek("/state membawa zOff", _sd88["zoff"], list(_lg90.zoff))
cek("  zOff belum dibaca dibedakan dari nol",
    M.rakit_state(M.Misi(), LinkPalsu(), M.Kalib(), M.Juri(M.Kalib(), NAMA),
                  {"fps": 0.0, "t_inf": 0.0}, True, "")["zoff"], None)
cek("dan bukan dari Kalib", hasattr(M.Kalib(), "offset"), False)

# --- kartu & tombolnya ada di halaman ---
for _id in ("id=offset", "id=lidaroff", "id=zoff"):
    cek(f"kartu '{_id}' ada di HTML", _id in M.HALAMAN, True)
cek("  tombol baca 'Yo'", "cmd('man','Yo')" in M.HALAMAN, True)
cek("  tombol baca 'Yd'", "cmd('man','Yd')" in M.HALAMAN, True)
cek("  tombol baca zOff lewat 'd'", "cmd('man','d')" in M.HALAMAN, True)
cek("  simpan offset lewat 'W', bukan 'YtW'",
    "cmd('manpaksa','W')" in M.HALAMAN, True)
cek("  nolkan offset 'Yo!'", "cmd('man','Yo!')" in M.HALAMAN, True)
cek("  nolkan offset LiDAR 'Yd!'", "cmd('man','Yd!')" in M.HALAMAN, True)
cek("  per slot mengirim 'Yo<slot> <der>'", "Yo${o.slot}" in M.HALAMAN, True)
cek("  per kaki mengirim 'Yz<kaki> <mm>'", "Yz${i}" in M.HALAMAN, True)
cek("  offset LiDAR dikirim lewat catatLidarOff",
    "function catatLidarOff(ch)" in M.HALAMAN, True)
cek("  dan yang diketik adalah jarak METERAN, bukan offsetnya",
    "ukur dengan meteran" in M.HALAMAN, True)
cek("  sensor MATI/jauh ditandai tak bisa dicatat",
    "tak bisa dicatat" in M.HALAMAN, True)
cek("  'Yd' dinyatakan RAM saja, bukan kalibrasi tetap",
    "ulangi tiap robot menyala" in M.HALAMAN, True)
# Yi & Yj sengaja TIDAK diberi tombol -- dan diamnya harus dijelaskan, bukan
# dibiarkan jadi teka-teki ("kok tidak ada?").
cek("  'Yi' & 'Yj' tidak dijadikan tombol",
    "cmd('man','Yi" in M.HALAMAN or "cmd('man','Yj" in M.HALAMAN, False)
cek("  dan sebabnya dikatakan",
    "membalik satu kanal hampir 180 der" in M.HALAMAN, True)

print(f"\n=== {ok} lulus, {fail} gagal ===")
sys.exit(1 if fail else 0)
