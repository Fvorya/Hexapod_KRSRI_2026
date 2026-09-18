#!/usr/bin/env python3
"""Periksa tabel lintasan di Hexapod_Unlimited/Misi.cpp.

Dijalankan di PC, tanpa robot dan tanpa compiler C++ (mesin ini tidak punya
g++ host, cuma arm-none-eabi). Yang diperiksa adalah hal-hal yang KALAU LOLOS
baru ketahuan di tengah arena:

  1. Belok relatif menghasilkan mata angin yang sama dengan konstanta misi
     adik tingkat (Mission.cpp Ver1_8) untuk tiga ruas yang sudah pernah dia
     jalankan. Ini yang menjaga rantai beloknya tidak melenceng sejak awal.
  2. Ruas yang berjalan BUTA ke depan wajib dibatasi odometri -- kalau tidak,
     tak ada apa pun yang menghentikannya.
  3. Ambang HNT_DEPAN harus di atas FRONT_STOP_CM, kalau tidak navigasi
     keburu berbelok sebelum misi sempat berhenti.
  4. Pemicu HNT_BELAKANG harus di dalam jangkauan sensor.
  5. Tabelnya tidak melebihi RUAS_MAKS.
  6. Baris AKS_AMBIL ditempatkan oleh sesuatu: HNT_DEPAN (ruas ini yang
     mendekat) atau HNT_LANGSUNG dengan ruas pendekat tepat sebelumnya.
     Sekuensnya memainkan sudut sendi tetap, jadi korban harus sudah berada
     di dalam amplop jangkauan lengan (cek_korban.cpp).

Firmware memeriksa 2-5 lagi saat 'm1' (tabelSiap) dan sebagiannya saat
kompilasi (static_assert); yang di sini berjalan tanpa perlu mengunggah apa
pun, jadi salah tulis ketahuan sebelum robot dinyalakan.

    python cek_tabel_misi.py
"""
import pathlib
import re
import sys

AKAR = pathlib.Path(__file__).resolve().parent
MISI_CPP = AKAR / "Hexapod_Unlimited" / "Misi.cpp"
MISI_H = AKAR / "Hexapod_Unlimited" / "Misi.h"
CONFIG_H = AKAR / "Hexapod_Unlimited" / "config.h"

BELOK = {"BLK_LURUS": 0, "BLK_KANAN": 1, "BLK_BALIK": 2, "BLK_KIRI": 3}
NAMA_ARAH = ["UTARA", "TIMUR", "SELATAN", "BARAT"]

# Konstanta misi adik tingkat, Mission.cpp @ Ver1_8. Ruas yang sudah pernah
# dia jalankan harus keluar dengan mata angin yang sama persis.
#
# DICARI LEWAT NAMA, bukan indeks. Versi lama memakai nomor ruas, dan nomor
# ruas bergeser tiap kali satu baris disisipkan atau dibuang -- dua kali dalam
# satu hari pada 17 Sep 2026, waktu ruas "mundur ke tembok" masuk lalu keluar
# lagi. Jangkar yang menyala karena tabelnya disunting, bukan karena arahnya
# salah, adalah jangkar yang lama-lama diabaikan orang.
#
# Cocoknya POTONGAN nama, huruf kecil. Nama ruas boleh disunting selama
# potongan ini tetap ada di dalamnya.
# CATATAN: skrip ini membaca TABEL SUMBER, jadi ia selalu memeriksa versi yang
# TIDAK tercermin. 'arena.mirror' adalah saklar RUNTIME di Teensy dan tidak
# terbaca dari sini. Itu benar dan disengaja -- yang dijaga di sini isi tabel,
# dan cerminnya hasil turunan yang dihitung firmware. Jangkar mata angin di
# bawah karena itu juga hanya berlaku untuk mode normal.
JANGKAR = [
    ("home", 0, "MISI_ARAH_AWAL 0 (UTARA) -- berangkat dari HOME"),
    ("k-1 angkat", 3, "MISI_ARAH_KORBAN1 3 (BARAT) -- K-1 di samping lintasan"),
    ("r-1 jalan pecah", 0, "pivot balik ke lorong 0 (UTARA)"),
]


def angka_define(teks, nama):
    m = re.search(r"^#define\s+%s\s+\(?(-?\d+)" % re.escape(nama), teks, re.M)
    if not m:
        sys.exit("tidak menemukan #define %s" % nama)
    return int(m.group(1))


def nilai_cm(x, cfg):
    """Kolom `nilai` boleh berupa angka ATAU nama #define.

    Baris korban menulis KORBAN_JARAK_CM, bukan 24: gerbang jarak dan pose
    lengan dihitung dari angka yang sama, dan dua tempat yang harus cocok
    tidak boleh ditulis dua kali."""
    try:
        return int(x)
    except ValueError:
        return angka_define(cfg, x)


def baca_tabel(teks):
    awal = teks.index("const Ruas RUAS_BAKU[] = {")
    blok = teks[awal:teks.index("};", awal)]
    # Baris yang DIKOMENTARI bukan bagian tabel: firmware tidak melihatnya,
    # jadi pemeriksa ini pun tidak boleh. Tanpa ini, K-3/K-4 yang dimatikan
    # ikut terhitung dan SETIAP nomor ruas yang dilaporkan meleset sembilan --
    # nomor yang salah lebih buruk daripada tidak ada nomor sama sekali.
    blok = "\n".join(b for b in blok.splitlines()
                      if not b.lstrip().startswith("//"))
    pola = re.compile(
        r'\{\s*"([^"]+)"\s*,\s*(BLK_\w+)\s*,\s*(KMD_\w+)\s*,\s*(PRF_\w+)\s*,'
        r'\s*(true|false)\s*,\s*(HNT_\w+)\s*,\s*(-?\d+|[A-Z_][A-Z0-9_]*)\s*,\s*(AKS_\w+)'
        # aksiA lalu `putar` yang OPSIONAL: 33 baris lama berhenti di aksiA dan
        # mengandalkan inisialisasi agregat untuk menolkan putar.
        r'\s*,\s*\w+(?:\s*,\s*([-+]?[\d.]+)f?)?'
    )
    baris = pola.findall(blok)
    if not baris:
        sys.exit("tabel RUAS_BAKU[] tidak terbaca -- formatnya berubah?")
    return baris


def main():
    cpp = MISI_CPP.read_text(encoding="utf-8", errors="replace")
    hdr = MISI_H.read_text(encoding="utf-8", errors="replace")
    cfg = CONFIG_H.read_text(encoding="utf-8", errors="replace")

    berangkat = angka_define(hdr, "MISI_ARAH_BERANGKAT")
    ruas_maks = angka_define(hdr, "RUAS_MAKS")
    front_stop = angka_define(cfg, "FRONT_STOP_CM")
    lidar_maks = angka_define(cfg, "LIDAR_MAX_CM")

    baris = baca_tabel(cpp)
    salah = []
    # Peringatan: dicetak, tapi TIDAK membuat skrip ini keluar dengan 1.
    # Untuk hal yang benar-salahnya tidak bisa dibaca dari tabel sendirian.
    awas = []

    # Belok relatif -> mata angin mutlak. Rumus yang sama dengan
    # Navigation::arahGeser() dan Misi::hitungArah().
    arah, a = [], berangkat
    for _, belok, *_ in baris:
        a = (a + BELOK[belok]) % 4
        arah.append(a)

    print("idx  arah      belok   kemudi  profil     henti      cm   aksi       nama")
    # Serong menumpuk persis seperti belok menumpuk (Misi::hitungArah).
    serong, akum = [], 0.0
    for b in baris:
        akum += float(b[8]) if b[8] else 0.0
        serong.append(akum)

    for i, (nama, belok, kemudi, profil, buta, henti, cm, aksi, putar) in enumerate(baris):
        cm = nilai_cm(cm, cfg)
        arah_txt = NAMA_ARAH[arah[i]]
        if abs(serong[i]) > 0.5:
            arah_txt += "%+.0f" % serong[i]
        print("%3d  %-8s  %-6s  %-6s  %-9s  %-9s %4s  %-9s  %s" % (
            i, arah_txt, belok[4:].lower(), kemudi[4:].lower(),
            profil[4:], henti[4:], "?" if cm < 0 else cm, aksi[4:], nama))

        # Serong seperempat penuh ditulis lewat `belok`, bukan lewat `putar`.
        # Keduanya bekerja, tapi hanya `belok` yang ikut ke mata angin, jadi
        # tabel yang memakai putar=90 kehilangan nama arahnya tanpa untung.
        if putar and abs(float(putar)) >= 90.0:
            salah.append("ruas %d putar %s der -- pakai kolom belok untuk kelipatan 90"
                         % (i, putar))

        # HNT_PUNCAK dikecualikan: 'buta ke depan' setelan Navigation, dan
        # yang membaca sensor di PUNCAK adalah ruasSelesai() lewat
        # getDistance() langsung. Buta untuk menyetir, melihat untuk berhenti.
        if buta == "true" and henti not in ("HNT_ODO", "HNT_PUNCAK"):
            salah.append("ruas %d buta ke depan tapi henti=%s, bukan HNT_ODO "
                         "atau HNT_PUNCAK" % (i, henti))
        # PUNCAK memakai sensor DEPAN, jadi ambangnya tunduk pada batas yang
        # sama: di bawah FRONT_STOP_CM navigasi keburu berhenti sendiri.
        if henti == "HNT_PUNCAK" and profil not in ("PRF_TANJAK", "PRF_KAIL"):
            awas.append("ruas %d HNT_PUNCAK tapi profilnya %s -- PUNCAK menunggu "
                        "badan MIRING dulu, dan di ruas datar itu tidak pernah "
                        "terjadi" % (i, profil))
        # PUNCAK dengan cm 0 SAH: artinya "gyro sendirian", dinding depan
        # dimatikan. HNT_DEPAN tidak punya arti itu -- di sana 0 salah tulis.
        if ((henti == "HNT_DEPAN" or (henti == "HNT_PUNCAK" and cm > 0))
                and 0 <= cm <= front_stop):
            salah.append("ruas %d ambang depan %d <= FRONT_STOP_CM %d" % (i, cm, front_stop))
        # Cermin tabelSiap(). Sekuens AMBIL memainkan sudut sendi TETAP dan
        # tidak pernah bertanya di mana korbannya, jadi sesuatu harus
        # menempatkan robot lebih dulu: ruas ini sendiri (HNT_DEPAN) atau ruas
        # tepat sebelumnya (HNT_LANGSUNG). Yang kedua dipakai K-1 dan K-2,
        # yang didahului HNT_MUNDUR ke tembok belakang.
        if aksi == "AKS_AMBIL" and henti not in ("HNT_DEPAN", "HNT_LANGSUNG"):
            salah.append("ruas %d AKS_AMBIL tapi henti=%s, bukan HNT_DEPAN atau HNT_LANGSUNG"
                         % (i, henti))
        # HNT_SISI cuma menggeser menyamping dan baris TARUH tidak bergerak
        # sama sekali, jadi keduanya bukan pendekat.
        #
        # PERINGATAN, bukan kegagalan, sama seperti tabelSiap(): jaraknya boleh
        # diatur ruas yang lebih jauh ke belakang atau oleh operator sebelum
        # start, dan itu tidak terbaca dari tabel. Tetap dicetak karena
        # akibatnya diam -- capit menutup di udara tanpa satu pun pesan.
        if aksi == "AKS_AMBIL" and henti == "HNT_LANGSUNG":
            sebelum = baris[i - 1][5] if i > 0 else None
            if sebelum not in ("HNT_MUNDUR", "HNT_DEPAN"):
                awas.append("ruas %d AMBIL HNT_LANGSUNG, ruas sebelumnya (%s) "
                            "tidak mendekatkan robot -- jaraknya diatur dari "
                            "luar tabel" % (i, sebelum or "tidak ada"))
        if henti == "HNT_BELAKANG" and cm >= lidar_maks - 15:
            salah.append("ruas %d pemicu belakang %d di luar jangkauan sensor (%d)"
                         % (i, cm, lidar_maks))

    if len(baris) > ruas_maks:
        salah.append("tabel %d baris > RUAS_MAKS %d -- _cm[] meluap" % (len(baris), ruas_maks))

    print("\njumlah ruas : %d (RUAS_MAKS %d)" % (len(baris), ruas_maks))
    belum = [i for i, b in enumerate(baris)
             if b[5] != "HNT_LANGSUNG" and nilai_cm(b[6], cfg) < 0]
    print("belum diukur: %s" % (", ".join(map(str, belum)) if belum else "tidak ada"))
    print("              (bukan kesalahan -- 'm7 <idx> <cm>' saat mapping)")

    print("\njangkar terhadap misi adik tingkat (Mission.cpp @ Ver1_8):")
    for kunci, harus, ket in JANGKAR:
        cocok = [i for i, b in enumerate(baris) if kunci in b[0].lower()]
        if len(cocok) != 1:
            # Nol = ruasnya dibuang atau namanya disunting sampai kuncinya
            # hilang. Lebih dari satu = kuncinya terlalu pendek. Keduanya
            # membuat jangkar ini berhenti mengukur apa pun, jadi keduanya
            # kegagalan -- bukan dilewati diam-diam.
            print("  SALAH jangkar '%s' cocok %d ruas  %s" % (kunci, len(cocok), ket))
            salah.append("jangkar '%s' cocok %d ruas, harus tepat 1 (%s)"
                         % (kunci, len(cocok), ket))
            continue
        idx = cocok[0]
        ok = arah[idx] == harus
        print("  %s ruas %d -> %-8s  %s" % ("OK  " if ok else "SALAH", idx,
                                            NAMA_ARAH[arah[idx]], ket))
        if not ok:
            salah.append("ruas %d seharusnya %s (%s)" % (idx, NAMA_ARAH[harus], ket))

    if awas:
        print("\n%d PERINGATAN (bukan kegagalan):" % len(awas))
        for m in awas:
            print("  ! " + m)

    if salah:
        print("\n%d MASALAH:" % len(salah))
        for m in salah:
            print("  - " + m)
        return 1
    print("\nSemua pemeriksaan lolos.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
