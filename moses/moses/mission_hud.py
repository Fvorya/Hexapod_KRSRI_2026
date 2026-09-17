#!/usr/bin/env python3
"""
mission_hud.py -- HUD misi + state machine hexapod SAR (R2C / Unlimited 2026).

DIAKSES LEWAT WEB:  http://terra-core:5000/
Tidak butuh VNC, tidak butuh window desktop, tidak butuh Flask (pakai
http.server dari pustaka standar -- satu dependensi kurang di Pi).

Berdiri sendiri di samping detect.py. Semua bagian berat (sesi ONNX, letterbox,
postprocess, kamera 60 FPS) DI-IMPOR dari detect.py, tidak disalin -- supaya
angka benchmark di EKSPERIMEN.md tetap mengacu ke kode yang sama persis.

    ./run_hud.sh                                  # web di :5000, tanpa robot
    ./run_hud.sh --port /dev/ttyACM0              # + Teensy, tetap MODE BACA
    ./run_hud.sh --web-port 5001                  # kalau :5000 masih dipakai app lama
    ./run_hud.sh --window                         # tambah window lokal (VNC)

DUA MODE
    BACA   (default) -- hanya MEMBACA dari Teensy: 'm', 'l'. Tidak ada satu pun
                        perintah gerak yang dikirim.
    KENDALI          -- FSM boleh mengirim 'o', 'O', 'D<cm>', 'w', 's', 'm2/m3'.
                        Ditekan dari tombol di halaman web. Spanduk merah.

KENAPA MODEL 640 PX JADI DEFAULT
    Robot BERDIRI DIAM saat menilai boneka, jadi 60 FPS tidak ada gunanya.
    9 frame @ 17 FPS = 0,53 detik per slot; 5 ruang x ~3 slot = ~8 detik dari
    anggaran 300 detik. Tukar laju jadi ketelitian: F1 0,990 -> 0,999.

KALIBRASI
    TIDAK ADA AUTOSAVE dan TIDAK ADA AUTOLOAD. Nilai hidup di RAM.
    Tombol "Simpan kalibrasi" menulis HANYA saat diklik; --calib memuat.
"""
import argparse
import glob
import io
import json
import math
import os
import re
import socket
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ANTREAN KONEKSI. Bawaan socketserver 5, dan lima itu penuh dalam satu
# kedipan saat beberapa tab dibuka bersamaan: peramban membuka enam koneksi
# sekaligus per tab. Koneksi yang tidak kebagian antrean ditolak oleh kernel,
# dan di peramban itu terbaca sebagai "situs tidak bisa dihubungi" -- bukan
# sebagai error apa pun di log HUD.
#
# Disetel di tingkat modul, bukan di dalam main(): ia dibaca saat server
# di-bind, dan tes juga mem-bind server-nya sendiri.
ThreadingHTTPServer.request_queue_size = 64
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

# Dipakai ulang apa adanya dari detektor produksi. detect.py punya penjaga
# __main__ jadi impor ini tidak menjalankan apa pun.
from detect import load_session, letterbox, postprocess, open_camera, CameraThread

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


# =====================================================================
# 1. PETA MISI -- urutan lintasan arena
# ---------------------------------------------------------------------
# SATU-SATUNYA tempat urutan misi ditulis. Kalau pembacaan peta di halaman
# 21 guidebook ternyata beda, betulkan DI SINI saja.
#
# PERIKSA SEBELUM LOMBA: pasangan K-n <-> SZ-n. Guidebook menulis "5 misi
# penyelamatan dengan pasangan safety zone masing-masing", tapi panah di peta
# untuk K-4 / SZ-4 / K-5 / SZ-5 masih bisa dibaca dua cara. Salah pasang =
# penyelamatan TIDAK SAH, bukan sekadar rugi waktu.
# =====================================================================
# Penanda versi. Ditampilkan di pojok kanan atas halaman supaya kamu bisa
# memastikan yang sedang jalan memang berkas yang barusan di-upload -- bukan
# menebak dari ada-tidaknya sebuah tombol.
VERSI = "2026-09-06 · v1.7 + ambil korban"

KORBAN = "KORBAN"
RINTANG = "RINTANG"
SAFETY = "SAFETY"
LAIN = "LAIN"

MISI = [
    # id,       label,                                       jenis,   slot, pasangan
    ("HOME",    "Keluar HOME lalu pivot ke UTARA",           LAIN,    0,  ""),
    ("K1",      "K-1  ceruk 40x15, 2 dummy + 1 asli",        KORBAN,  3,  "SZ-1"),
    ("R1_R4",   "R1..R4  pecah / berpuing",                  RINTANG, 0,  ""),
    ("SZ1",     "SZ-1  letakkan korban 1",                   SAFETY,  0,  "K-1"),
    ("K2",      "K-2  area 15x28, 1 dummy + 1 asli",         KORBAN,  2,  "SZ-2"),
    ("R5",      "R5  jalan berlumpur (kelereng)",            RINTANG, 0,  ""),
    ("SZ2",     "SZ-2  letakkan korban 2 (tinggi 4 cm)",     SAFETY,  0,  "K-2"),
    ("R6",      "R6  jalan pecah 45x55",                     RINTANG, 0,  ""),
    ("K3",      "K-3  tertimpa papan 14x17, serong 45",      KORBAN,  2,  "SZ-3"),
    ("R7",      "R7  BERSIHKAN koral SZ-3 (wajib bersih)",   RINTANG, 0,  ""),
    ("SZ3",     "SZ-3  letakkan korban 3",                   SAFETY,  0,  "K-3"),
    ("K4",      "K-4  tertimpa papan 14x17, serong 45",      KORBAN,  2,  "SZ-4"),
    ("R8_R9",   "R8, R9  menuju tangga M2",                  RINTANG, 0,  ""),
    ("SZ4",     "SZ-4  letakkan korban 4",                   SAFETY,  0,  "K-4"),
    ("R10",     "R10  miring + puing + lumpur",              RINTANG, 0,  ""),
    ("K5",      "K-5  ambil korban 5",                       KORBAN,  2,  "SZ-5"),
    ("SZ5",     "SZ-5  lubang 12x7 di bidang miring",        SAFETY,  0,  "K-5"),
    ("R11",     "R11  jalan longsor, lebar kaki 30 cm",      RINTANG, 0,  ""),
    ("FINISH",  "FINISH  berhenti & diam 10 detik",          LAIN,    0,  ""),
]

BELUM, JALAN, SELESAI, GAGAL_, LEWAT = "-", ">", "v", "x", "~"

# MISI YANG FIRMWARE-nya BENAR-BENAR BISA MENJALANKAN (v1.8).
#
# Ini pembeda yang paling sering disalahpahami, dan salahku karena tidak
# menuliskannya sejak awal: tabel MISI di atas itu RENCANA LOMBA, bukan daftar
# kemampuan. HUD sendiri TIDAK PUNYA kode navigasi sama sekali -- berpindah
# ruang, menyeberang lantai pecah, menuruni bidang miring, semuanya milik
# Mission.cpp di Teensy.
#
# Akibatnya "Lewati misi ini" hanya menggeser PENANDA di daftar; robotnya tidak
# ke mana-mana, karena memang tidak ada yang menyuruhnya berjalan. Itu bukan
# tombol rusak, itu tombol pembukuan.
#
# Yang ADA di Mission.cpp v1.8, urut: PIVOT_AWAL -> KE_KORBAN1 ->
# PIVOT_KORBAN1 -> KONFIRM1 -> PIVOT_LANTAI -> LANTAI_PECAH -> TURUN ->
# MAJU_AKHIR -> PIVOT_AKHIR -> SELESAI. Sesudah itu firmware sendiri bilang
# "SELESAI (irisan pertama)".
DIDUKUNG_FIRMWARE = {"HOME", "K1", "R1_R4"}

# Kata kunci nama ruas yang berarti "di sinilah vision seharusnya bekerja".
# Firmware v1.10 BELUM punya pemicu vision -- tidak satu pun ruas memakai
# AKS_KONFIRM, jadi misi tidak pernah berhenti menunggu keputusan. Tapi
# status 'm' SELALU menyebut nama ruas yang sedang dijalankan, dan nama itu
# sudah cukup untuk tahu kapan kamera perlu melihat.
RUAS_VISION = ("ANGKAT KORBAN",)


# =====================================================================
# 2. SUB-FSM VISION -- dijalankan Pi selagi Teensy menunggu di KONFIRM
# ---------------------------------------------------------------------
# Teensy memegang BATANG misi (Navigation + Mission.cpp). Pi hanya memegang
# CABANG penilaian korban. Satu penulis vektor gerak, sama seperti doktrin
# navBerhenti() di firmware.
# =====================================================================
S_IDLE = "IDLE"
S_TUNGGU = "TUNGGU_TEENSY"
S_STANDOFF = "STANDOFF"
S_CENTER = "CENTERING"
S_LIHAT = "LIHAT"
S_SLOT = "SLOT_LANJUT"
S_PUTUS = "PUTUSKAN"
S_DEKATI = "DEKATI"
S_CENGKERAM = "CENGKERAM"
S_VERIF = "VERIFIKASI"
S_ANTAR = "ANTAR_SZ"
S_LETAK = "LETAKKAN"
S_BERES = "SELESAI_RUANG"
S_JEJAK = "JEJAK"
# Rantai AMBIL: satu tombol, lima langkah, tiap langkah kelihatan di HUD.
S_A_TENGAH = "AMBIL_TENGAH"
S_A_MAJU = "AMBIL_MAJU"
S_A_HALUS = "AMBIL_HALUS"
S_KONFIRM = "KONFIRM_FW"
# Pose tengah DITAHAN selagi sekuens capit firmware berjalan. Bukan state
# menonton: di sini vision masih satu-satunya yang memegang kaki, dan ia
# memegangnya justru supaya badan TIDAK kembali ke default saat capit turun.
S_TAHAN = "TAHAN_TENGAH"
S_AWAS = "AWAS_KORBAN"
S_A_SIAP = "AMBIL_SIAP"
S_A_JEPIT = "AMBIL_JEPIT"
S_A_ANGKAT = "AMBIL_ANGKAT"
S_GAGAL = "GAGAL"

# state -> (penjelasan singkat, state berikutnya kalau lancar)
FSM = {
    S_JEJAK:     ("ikuti korban, tengahkan terus-menerus",        S_STANDOFF),
    S_KONFIRM:   ("firmware menunggu m2/m3 - vision menilai",     S_IDLE),
    S_TAHAN:     ("tahan tengah sampai capit terangkat",         S_IDLE),
    S_AWAS:      ("ruas korban - vision MENGAMATI, tidak bergerak", S_IDLE),
    S_A_TENGAH:  ("AMBIL 1/6 - tengahkan KASAR (pivot kaki)",       S_A_MAJU),
    S_A_MAJU:    ("AMBIL 2/6 - maju ke jarak capit",             S_A_HALUS),
    S_A_HALUS:   ("AMBIL 3/6 - tengahkan HALUS (putar badan)",   S_A_SIAP),
    S_A_SIAP:    ("AMBIL 4/6 - buka capit & posisikan lengan",   S_A_JEPIT),
    S_A_JEPIT:   ("AMBIL 5/6 - tutup capit",                     S_A_ANGKAT),
    S_A_ANGKAT:  ("AMBIL 6/6 - angkat & periksa",                S_BERES),
    S_IDLE:      ("menunggu ruang korban berikutnya",            S_TUNGGU),
    S_TUNGGU:    ("Teensy menuju korban; vision MATI",           S_STANDOFF),
    S_STANDOFF:  ("atur jarak depan ke 20 cm (LiDAR)",           S_CENTER),
    S_CENTER:    ("tengahkan badan ke boneka (pivot)",           S_LIHAT),
    S_LIHAT:     ("9 frame, gerbang ROI + tinggi bbox",          S_SLOT),
    S_SLOT:      ("geser 8 cm ke slot berikut",                  S_STANDOFF),
    S_PUTUS:     ("pilih margin terbesar + prior 1 asli",        S_DEKATI),
    S_DEKATI:    ("class-lock; LiDAR + IR saja, vision MATI",    S_CENGKERAM),
    S_CENGKERAM: ("tutup capit (triple-gate)",                   S_VERIF),
    S_VERIF:     ("pastikan korban benar terangkat",             S_ANTAR),
    S_ANTAR:     ("bawa ke safety zone pasangannya",             S_LETAK),
    S_LETAK:     ("turunkan di dalam SZ, jangan diseret",        S_BERES),
    S_BERES:     ("ruang selesai",                               S_IDLE),
    S_GAGAL:     ("berhenti; tandai dicoba, JANGAN diulang",     S_IDLE),
}

# Vision hanya menyala di dua state ini. Di luar itu inferensi dilewati --
# kamera tetap streaming supaya tidak membayar init ulang ~1,5 detik.
VISION_ON = {S_CENTER, S_LIHAT, S_JEJAK, S_A_TENGAH, S_A_HALUS, S_KONFIRM,
             S_AWAS, S_TAHAN}

# Rantai ambil korban, urut. Dipakai HUD untuk tahu 'sedang mengambil'.
RANTAI_AMBIL = (S_A_TENGAH, S_A_MAJU, S_A_HALUS, S_A_SIAP, S_A_JEPIT, S_A_ANGKAT)

# State yang MENGEMUDI dengan bearing: tiap frame mereka memakai
# misi.bearing_deg untuk memilih arah gerak. Karena itu semuanya WAJIB
# mendapat bearing yang SEGAR tiap frame.
#
# Bug 14 Sep 2026, dan ia kelihatan persis seperti kesalahan penyetelan gain:
# AMBIL_HALUS tidak ada di daftar mana pun yang memperbarui bearing_deg --
# JEJAK/AMBIL_TENGAH punya cabangnya sendiri, CENTERING punya cabangnya
# sendiri, AMBIL_HALUS tidak punya. Jadi nilainya BEKU sejak AMBIL_TENGAH
# menyerahkan. pusatkan() menumpuk badan_yaw tiap frame dari galat yang sama
# dan sama besarnya, jadi badan berputar ke SATU arah tanpa henti; dan
# 'sudah tengah' tidak pernah bisa benar, karena galat yang beku tidak pernah
# mengecil berapa pun robot berputar. Dari luar: "centering tidak sebagus
# JEJAK, badan gerak ke kiri terus".
#
# Satu daftar dipakai bersama supaya state kemudi berikutnya tidak bisa lagi
# masuk ke satu daftar lalu terlupa di daftar satunya.
MENGEMUDI = (S_JEJAK, S_CENTER, S_A_TENGAH, S_A_HALUS, S_TAHAN)

# STATE YANG BELUM DIPROGRAM. langkah_fsm() sengaja diam di sini, bukan
# pura-pura jalan -- tapi diam itu dulu TIDAK KELIHATAN dari halaman: robot
# berhenti, tidak ada pesan, dan operator menunggu sesuatu yang memang tidak
# akan pernah terjadi.
#
# Rantai lama DEKATI..LETAK sudah digantikan rantai AMBIL (S_A_*) yang benar-
# benar jalan, tapi jalur otomatis dari S_PUTUS masih bermuara ke sini.
# Selama belum disambung, jalan keluarnya tombol "Lewati misi ini".
BELUM_ADA = {S_DEKATI, S_CENGKERAM, S_VERIF, S_ANTAR, S_LETAK}

# Batas waktu per state (detik). 0 = tanpa batas.
# Berapa lama pemicu #KORBAN boleh MENGGANTUNG sambil menunggu bukti parkir
# yang segar datang dari firmware. Bukan batas kerja -- batas kesabaran.
# 2 detik itu longgar: pengumuman parkir datang di burst serial yang sama
# dengan pemicunya, dan poll 'm' paksa di bawah membalas < 0,3 detik.
PEMICU_TUNGGU_BUKTI_S = 2.0

# MUNDUR_LAJU / MUNDUR_DETIK / MUNDUR_MAKS_N DIHAPUS 17 Sep 2026. Pi tidak
# lagi mengirim gerakan maju atau mundur sama sekali; sumbu itu milik Teensy
# sendirian. Lihat catatan di S_A_MAJU.

BATAS = {S_STANDOFF: 20, S_CENTER: 20, S_LIHAT: 8, S_SLOT: 25,
         S_DEKATI: 25, S_CENGKERAM: 15, S_VERIF: 10,
         S_A_TENGAH: 8, S_A_MAJU: 30, S_A_HALUS: 8, S_KONFIRM: 25, S_A_SIAP: 10, S_A_JEPIT: 8,
         S_A_ANGKAT: 10,
         # Sekuens capit firmware = 6 fase x KORBAN_FASE_MS (1850) + fase
         # DIAM 1000 ms = 12,1 detik, dihitung dari 'm2'. (Naik dari 8,4 detik
         # 14 Sep 2026: slew lengan dipelankan 120 -> 72 der/s dan satu fase
         # diam disisipkan sebelum angkat.) 25 detik masih memberi ruang untuk
         # auto_konfirm MATI -- operator yang menekan m2 sendiri -- tanpa
         # menggantung selamanya kalau '#LEPAS' tidak pernah datang.
         S_TAHAN: 25}


# =====================================================================
# 3. KALIBRASI -- RAM saja. Tanpa autoload, tanpa autosave.
# =====================================================================
@dataclass
class Kalib:
    hfov_deg: float = 70.4      # C922 horizontal; ukur ulang kalau ganti lensa
    roi_half_deg: float = 10.0  # gerbang ROI: +-10 der = +-3,5 cm di 20 cm
    cx_offset_px: float = 0.0   # piksel-tengah kamera vs garis tengah capit
    # Kamera di robot ini dipasang TERBALIK, jadi default-nya True. Karena
    # kalibrasi sengaja tidak punya autoload, nilai default di baris inilah
    # yang berlaku tiap kali HUD start -- bukan file mana pun.
    rotate180: bool = True
    standoff_cm: float = 20.0   # jarak kerja kamera
    standoff_tol_cm: float = 2.0
    yaw_tol_deg: float = 6.0    # = HEADING_TOLERANCE_DEG firmware
    slot_step_cm: float = 8.0   # spasi tanda silang K-1 menurut guidebook
    bbox_h_min: float = 0.35    # tinggi bbox / tinggi frame, pita di 20 cm
    bbox_h_max: float = 0.90
    # JEJAK dipakai dari jarak BERAPA PUN -- boneka 50 cm jauhnya hanya
    # mengisi ~0,15 tinggi frame, jadi pita 0,35..0,90 milik mode penilaian
    # akan membuang semuanya dan robot tidak pernah bergerak. Pita jejak
    # sengaja longgar: tugasnya cuma membuang bercak kecil dan benda raksasa.
    jejak_h_min: float = 0.06
    jejak_h_max: float = 0.98
    # Laju inferensi maksimum saat MENJEJAK, per detik. 0 = TANPA BATAS.
    #
    # Dikembalikan ke 0 atas permintaan: komputasi penuh dianggap lebih
    # penting daripada margin daya, dan tim elektrik sedang menggarap catu
    # dayanya. Tapi catatan ini tetap ditulis supaya tidak hilang: knob INI
    # yang dulu menghentikan Pi mati tiap kali masuk mode JEJAK. Inferensi
    # 640 px memakai 3 inti penuh dan lonjakan arusnya yang menjatuhkan
    # tegangan -- multimeter tidak bisa melihatnya karena dia merata-rata.
    #
    # Kalau Pi mati lagi di JEJAK, ini knob pertama yang diputar, dan bisa
    # diubah LANGSUNG dari tab Kalibrasi tanpa restart: isi 8 lalu coba lagi.
    jejak_hz: float = 0.0
    # --- rantai AMBIL ---
    # Jarak saat robot BERHENTI untuk mencapit -- bukan jarak capit menutup.
    #
    # Diturunkan dari geometri lengan v1.9, bukan tebakan lagi:
    #   LENGAN_SIAP_MM  = 120 mm  |  LENGAN_AMBIL_MM = 150 mm, dari PUSAT BADAN
    #   dudukan LiDAR depan       ~50 mm di depan pusat badan
    #   -> korban di 150 mm dari pusat badan = 100 mm dari sensor = 10 cm
    #
    # Konsekuensinya besar: 3 cm terakhir dikerjakan LENGAN yang menjulur
    # 120 -> 150 mm, BUKAN robot yang berjalan. Jadi tidak ada "maju terakhir
    # sambil capit turun" -- dan kekhawatiran vision tertutup capit selama
    # mendekat jadi tidak berlaku, karena tidak ada mendekat.
    #
    # TETAP HARUS DIUKUR: ARM_ORIGINS di config.h sendiri masih ditandai
    # "ANGKA 50 MASIH PERKIRAAN, ukur ulang saat lengan terpasang".
    # Catatan: 10 cm jatuh dekat pita hantu LiDAR depan yang Vincent
    # dokumentasikan (3,2 dan 9 cm). Periksa 'l' saat berhenti.
    capit_cm: float = 10.0
    capit_tol_cm: float = 1.5
    # --- JARAK DARI KAMERA (monokuler, dari tinggi bbox) ---
    #
    # d = f * H / h_px, dengan f = (lebar/2) / tan(hfov/2) = 907 px di 1280.
    # Bukan tebakan: di 25 cm satu piksel tinggi bbox bernilai 0,06 cm, jadi
    # 5 px noise = 3 mm. Itu LEBIH HALUS daripada kuantisasi LiDAR.
    #
    # TAPI ADA BATASNYA, dan batas itu jatuh persis di jarak capit.
    # Korban 12 cm mengisi SELURUH tinggi frame pada 15 cm, jadi pada
    # capit_cm = 10 cm kepalanya di atas frame dan kakinya di bawah.
    # Bbox yang terpotong punya h_px lebih KECIL dari yang sebenarnya, dan
    # d = f*H/h_px membuatnya terbaca LEBIH JAUH. Robot akan mengira masih
    # 15 cm padahal sudah 8, lalu terus maju dan menabrak korban. Itu sebabnya
    # pemotongan dideteksi eksplisit dan bacaannya DIBUANG, bukan dipakai.
    vision_jarak_on: bool = True
    # Tinggi EFEKTIF korban dalam cm. Titik berangkatnya sekarang ANGKA UKUR
    # (R2C, 11 Sep 2026: tinggi +-9 cm, lebar +-8,5 cm) -- bukan lagi tempelan.
    # Tapi yang dipakai rumus tetap tinggi EFEKTIF, yaitu yang membuat hasilnya
    # cocok dengan penggaris: bbox YOLO jarang mepet dan sering memotong kaki.
    # Tombol "Kalibrasi jarak vision" menyelesaikannya dari satu pengukuran,
    # sekaligus menyerap bbox longgar, distorsi lensa, dan selisih titik nol
    # kamera vs titik nol lengan.
    korban_tinggi_cm: float = 9.0
    # Lebar dipakai untuk PEMERIKSAAN, bukan untuk mengukur jarak.
    # Alasannya geometri: memutar korban pada sumbu tegak TIDAK mengubah
    # tingginya, tapi MENGUBAH lebar tampaknya. Jadi tinggi = besaran yang
    # kokoh untuk jarak, lebar = petunjuk orientasi. K-3 dan K-4 menyerong 45
    # der menurut guidebook, dan di situlah bedanya akan terlihat.
    korban_lebar_cm: float = 8.5
    # Tinggi efektif PER POSISI korban. 0 = pakai korban_tinggi_cm global.
    # Kenapa perlu, padahal tinggi tidak berubah saat korban diputar:
    # yang berubah bukan korbannya, melainkan cara ia TERLIHAT -- K-3/K-4
    # tertimpa papan 14x17 dan cuma 1 cm dari lantai, SZ-2/SZ-4 setinggi 4 cm,
    # jadi bbox-nya terpotong tanggul atau papan dengan cara yang berbeda-beda.
    # Satu angka global tidak bisa mewakili kelimanya.
    tinggi_k1: float = 0.0
    tinggi_k2: float = 0.0
    tinggi_k3: float = 0.0
    tinggi_k4: float = 0.0
    tinggi_k5: float = 0.0

    # DUA KORBAN SATU FRAME: kunci yang paling KANAN.
    #
    # K-3, K-4 dan K-5 berdiri cukup berdekatan sehingga dua korban asli masuk
    # satu frame sekaligus. Gerbang kelas tidak memisahkan keduanya -- keduanya
    # korban. Tanpa aturan ini pilih_sasaran() memilih bbox yang paling
    # TINGGI, dan yang paling tinggi bisa korban ruas BERIKUTNYA: robot
    # menengahkan diri ke korban yang belum gilirannya, lalu membawanya ke
    # safe zone yang salah.
    #
    # Yang mengurutkan tabel lintasan, bukan knob ini: kerjakan yang kanan
    # dulu, dan sesudah korban itu ditaruh di safe zone-nya, korban berikutnya
    # yang berdiri paling kanan pada ruas sesudahnya. Jadi tidak ada yang
    # perlu mengingat korban mana yang sudah diambil.
    #
    # Kuncinya MANTAP dengan sendirinya, tanpa state: sesudah korban kanan
    # tertengahkan, korban satunya tetap di kirinya, jadi yang kanan tetap
    # yang terpilih di frame berikutnya. Itu sebabnya "kunci" di sini tidak
    # butuh ingatan -- ingatan yang salah justru mengunci sasaran yang sudah
    # tidak terlihat.
    lock_kanan: bool = True

    # PENENGAHAN TANPA MEMUTAR, per posisi korban.
    #
    # Di K-3 dan K-4 korbannya tertutup reruntuhan, dan memutar badan
    # menyapukan capit ke reruntuhan itu sebelum sempat turun. Jadi di kedua
    # posisi itu penengahan dikerjakan dengan MENGGESER: 'H' (satu siklus gait
    # ke samping) untuk jarak kasar, lalu 't' (geser badan di atas kaki diam)
    # untuk sisa yang halus. Tidak satu pun dari keduanya memutar badan.
    #
    # Posisi lain tetap memakai putar-badan: di sana tidak ada reruntuhan, dan
    # putar jauh lebih teliti daripada langkah geser yang terkuantisasi.
    geser_k1: bool = False
    geser_k2: bool = False
    geser_k3: bool = True
    geser_k4: bool = True
    geser_k5: bool = False

    # Amplitudo 'H'. Menentukan panjang langkah geser, bukan lamanya -- 'H'
    # selalu satu siklus gait penuh.
    geser_amp: float = 0.20
    # Tunggu sesudah 'H': satu siklus gait DITAMBAH waktu kaki menetap.
    # Mengukur bearing sebelum kaki diam berarti mengoreksi pose yang sudah
    # tidak ada lagi.
    geser_tunggu_s: float = 1.2
    # Di bawah simpangan ini, 'H' terlalu kasar -- satu langkahnya melewati
    # sasaran. Sisanya dikerjakan 't', yang perpindahannya milimeter dan tidak
    # menggerakkan kaki sama sekali.
    geser_halus_px: float = 60.0
    # Berapa 'H' boleh dicoba sebelum menyerah ke V. Bukan batas waktu:
    # langkah yang tidak memperbaiki apa pun harus berhenti dihitung sebagai
    # kemajuan.
    geser_maks_langkah: int = 6
    # CADANGAN 'V' per posisi, cm. Tanda = sisi: positif dinding KANAN,
    # negatif dinding KIRI, sama seperti perintah V firmware.
    #
    # Angkanya dari R2C 15 Sep 2026: K-3 bertemu batas 35 cm dari tembok utara
    # lewat LiDAR kanan; K-4 jatuhnya adu keberuntungan sekitar 45 cm, dibantu
    # 'V-55'. Ini acuan DINDING, bukan acuan korban -- karena itu ia cadangan,
    # bukan cara utama.
    geser_v_cm_k3: float = 35.0
    geser_v_cm_k4: float = -55.0
    # Gerbang "sudah di jarak kerja?" dalam PIKSEL, bukan cm. Di piksel-lah
    # noise-nya hidup, jadi di situ pula ambangnya paling jujur disetel.
    # Padanannya: +-20 px pada 25 cm (bbox 327 px) = +-1,5 cm -- sengaja
    # dibuat sepadan dengan capit_tol_cm. HUD menampilkan kedua satuannya
    # supaya kamu bisa menyetel yang satu dari yang lain.
    bbox_tol_px: float = 20.0
    # Rasio lebar/tinggi bbox yang masih wajar. 8,5/9 = 0,94 saat menghadap
    # kamera. Jauh di luar pita ini berarti bbox-nya bukan satu korban utuh:
    # terpotong, bergabung dengan dummy sebelahnya, atau tertimpa papan.
    rasio_min: float = 0.55
    rasio_maks: float = 1.60
    rasio_periksa: bool = True
    # Bbox yang menyentuh tepi atas/bawah frame dianggap TERPOTONG.
    bbox_tepi_px: int = 4
    # Pagar akal sehat. Di luar ini bacaan vision dibuang tanpa dipakai.
    vision_jarak_min_cm: float = 12.0
    vision_jarak_maks_cm: float = 90.0
    # Beda vision vs LiDAR yang masih dianggap wajar. Lebih dari ini berarti
    # SALAH SATU bohong -- dan di dekat 10 cm tersangka utamanya pita hantu
    # LiDAR (3,2 dan 9 cm) yang Vincent dokumentasikan. Robot berhenti dan
    # bilang, bukan memilih sendiri sensor mana yang dipercaya.
    vision_lidar_beda_maks_cm: float = 6.0
    # MATI SAMPAI K1-K5 DIUKUR. Silang-periksa hanya sekuat bacaan yang
    # disilangkan, dan bacaan vision belum dikalibrasi sama sekali
    # (tinggi_k1..k5 masih 0). Menyalakannya sekarang berarti menghentikan
    # pendekatan yang sehat karena angka yang belum pernah dicocokkan --
    # itu justru yang dikeluhkan: "mengacaukan gerakan robot". Nyalakan
    # SESUDAH kalibrasi per posisi selesai.
    vision_silang_on: bool = False
    # --- CARA MENGAMBIL: LENGAN maju, atau BADAN maju ---
    #
    # "lengan" = robot berhenti di jangkauan lengan lalu MENJULURKAN lengan.
    # "badan"  = robot berjalan sampai korban di depan capit (cara lama).
    #
    # Ini bukan sekadar pilihan gaya. Jarak kerjanya berubah 10 cm -> 25 cm,
    # dan 25 cm itu pita TERBAIK untuk kedua sensor sekaligus:
    #   kamera : bbox utuh (korban 12 cm mengisi 0,60 tinggi frame), dan
    #            5 piksel noise cuma 2,9 mm. Di 10 cm bbox TERPOTONG dan
    #            kamera buta total.
    #   LiDAR  : jauh dari pita hantu 3,2 dan 9 cm yang Vincent dokumentasikan.
    # Jadi cara ini memindahkan titik kerja dari tempat kedua sensor paling
    # lemah ke tempat keduanya paling kuat.
    mode_ambil: str = "lengan"
    # Diukur R2C 11 Sep 2026 di robot sungguhan:
    #   jangkauan depan 25 cm, tinggi 12 cm, vektor 28 cm ke titik tengah
    #   lubang capit. Cek: hypot(25, 12) = 27,73 -- cocok dengan 28 dalam
    #   3 mm, jadi ketiga angka itu konsisten satu sama lain.
    # 25,0 itu yang KAMU ukur. Firmware v1.12 memakai KORBAN_JARAK_CM = 24
    # untuk gerbang HNT_DEPAN di kelima ruas AMBIL, jadi angka DI SINI
    # diselaraskan ke 24: kalau HUD dan firmware berhenti di jarak berbeda,
    # yang satu akan terus memerintahkan maju sementara yang lain menganggap
    # sudah sampai. Selisih 1 cm itu masih di dalam capit_tol_cm (1,5).
    #
    # Angkanya cocok bukan kebetulan: capit ada di
    # KORBAN_JARAK_CM*10 + LIDAR_DEPAN_MM = 240 + 62 = 302 mm dari pusat
    # badan, dan LiDAR depan duduk 62 mm di depan pusat badan -- jadi "LiDAR
    # membaca 24 cm" dan "capit tepat di korban" itu pernyataan yang sama.
    lengan_jangkau_cm: float = 24.0
    # GERBANG BERHENTI, cermin KORBAN_JARAK_CM firmware. 25 sejak 15 Sep 2026
    # (R2C, revisi kedua hari itu: 24 -> 20 -> 25 cm).
    #
    # DIPISAH dari lengan_jangkau_cm, yang dulu merangkap keduanya. Alasannya
    # berubah di firmware, bukan di sini: sejak v1.15 sekuens capit memakai
    # SUDUT SENDI TETAP (KORBAN_SIAP_*, KORBAN_JEPIT_*), bukan IK dari titik
    # capit. Jadi "di mana robot berhenti" dan "seberapa jauh lengan menjulur"
    # sudah bukan angka yang sama, dan menyatukannya berarti menggeser
    # pemeriksaan geometri lengan tiap kali gerbang jaraknya disetel.
    #
    # Yang WAJIB tetap sama: angka ini dan KORBAN_JARAK_CM. Kalau berbeda,
    # yang satu terus memerintahkan maju sementara yang lain sudah menganggap
    # sampai, dan robot berayun di depan korban sampai batas waktu.
    korban_jarak_cm: float = 25.0
    # JARAK DINDING BELAKANG saat menyelamatkan korban, cm. 0 = jangan disetel.
    #
    # Diminta R2C 15 Sep 2026: "beri jarak J10 untuk penyelamatan korban".
    # Dikirim sekali di awal rantai AMBIL, sebelum penengahan -- bukan di
    # tengahnya, karena 'J' menggerakkan KAKI dan langkah kaki membuang
    # seluruh penengahan yang sudah dibayar.
    #
    # Kembarannya di firmware: RATA_BLK_SASARAN_CM, yang menjaga angka yang
    # sama selama perintah 'V'. Kalau yang ini diubah, ubah di sana juga.
    korban_belakang_cm: float = 10.0

    # --- CERMIN PARAMETER CONDONG K-3/K-4 DI FIRMWARE ---
    #
    # Ketiganya ada di tabel kalibrasi Teensy (Calib.h: condong.mm,
    # condong.jeda, condong.yaw) dan tersimpan di EEPROM-nya. Angka DI SINI
    # cuma salinan yang bisa dikirim ulang lewat tombol HUD, supaya menyetelnya
    # di arena tidak menuntut mengetik 'Q' di konsol serial.
    #
    # Yang BERLAKU di robot selalu yang ada di EEPROM Teensy. Tombol "kirim ke
    # Teensy" yang menyamakan keduanya; membaca angka di sini saja tidak
    # membuktikan apa pun tentang isi EEPROM.
    # 60, bukan 25: firmware menaikkan baku `condong.mm` ke 60 pada 16 Sep
    # 2026 bersama BODY_MAX_TRANS_MM 40 -> 80. Cermin yang tertinggal di 25
    # akan MENURUNKAN angka firmware begitu operator menekan "Kirim + simpan".
    condong_mm: float = 60.0        # geser badan maju sebelum lengan turun
    # DINOLKAN 17 Sep 2026, diminta R2C sesudah trial.
    #
    # Dua fase kosong ini diminta 15 Sep sebagai jeda KONFIRMASI MATA sebelum
    # lengan turun -- operator perlu waktu melihat apakah capit bebas dari
    # reruntuhan. Deteksi korban sekarang dikerjakan kamera, dan kamera sudah
    # selesai menilai sebelum 'm2' terkirim. Alasannya habis; detiknya tidak.
    #
    # 2200 ms dibulatkan NAIK ke jatah LENGAN_JEDA_MS (2100), jadi ia memakan
    # DUA fase = 4200 ms. Ditambah fase pelurusan yaw di bawah, ruas condong
    # (K-3 dan K-4) membayar 6300 ms yang tidak menggerakkan capit sama sekali.
    condong_jeda_ms: float = 0.0     # jeda konfirmasi sebelum meluruskan
    # DIMATIKAN 17 Sep 2026, alasan yang sama: satu fase penuh (2100 ms) untuk
    # memutar badan ke heading ruas, sesudah Pi baru saja memutarnya ke KORBAN.
    # Dua penguasa untuk satu sudut, dan yang terakhir menulis menang.
    condong_yaw: float = 0.0        # 1 = luruskan ke heading ruas, 0 = jangan
    # TANDA ITU PENTING dan belum dipastikan. Firmware memakai tinggi dari
    # PUSAT BADAN: + di ATAS pusat badan, - di BAWAH. Korban tergeletak di
    # lantai (K-3/K-4 cuma 1 cm dari lantai menurut guidebook), jadi lengan
    # hampir pasti harus turun -- karena itu default-nya NEGATIF. Kalau
    # ternyata pusat badan justru di bawah titik capit, balik tandanya.
    # Yang dipakai pemeriksaan vektor cuma nilai mutlaknya, jadi tanda salah
    # TIDAK akan ketahuan dari situ. Lihat dulu lengannya bergerak ke mana.
    lengan_tinggi_cm: float = -12.0
    # Vektor yang kamu ukur (28 cm). Dibandingkan dengan hypot(jangkauan,
    # tinggi). CATATAN: sesudah jangkauan diselaraskan ke 24, hypot(24, 12)
    # = 26,8 -- jadi selisihnya ke 28 jadi 1,2 cm dan toleransinya dinaikkan
    # ke 1,5. Yang tersisa itu kemungkinan besar BEDA TITIK NOL: kamu mengukur
    # ke titik tengah lubang capit, firmware menghitung dari pusat badan.
    lengan_vektor_ukur_cm: float = 28.0
    lengan_vektor_tol_cm: float = 1.5
    lengan_r: float = 70.0      # jangkauan lengan, mm dari PUSAT BADAN
    lengan_h: float = 20.0      # tinggi lengan saat menjepit, mm
    lengan_angkat: float = 25.0  # tambahan tinggi saat mengangkat, mm
    # LEBAR BUKA CAPIT saat mendekat, persen (0 = menutup, 100 = buka penuh).
    #
    # 20, turun dari 50 lalu dari 100. Laporan R2C 15 Sep 2026: "dengan g50,
    # bukaan terlalu lebar sehingga dummy ataupun reruntuhan bisa diambil
    # capit." Korban lebarnya 8,5 cm dan itu masih muat di 20; yang hilang cuma
    # sapuan rahang yang tidak pernah dibutuhkan. Di ruang korban tetangga
    # terdekat 8 cm -- rahang yang terbuka lebih lebar dari itu menyenggolnya
    # sebelum sempat menutup.
    #
    # Angka yang SAMA dipakai sekuens firmware (KORBAN_GRIP_BUKA di config.h).
    # Kalau yang ini diubah, ubah di sana juga -- keduanya menggerakkan satu
    # capit.
    capit_buka_persen: float = 20.0

    # DIAM sesudah capit menutup, SEBELUM lengan mengangkat. Detik.
    #
    # Kembaran KORBAN_JEPIT_DIAM_MS di firmware (config.h), dipakai rantai
    # capit MANUAL HUD -- rantai misi tidak lewat sini, capitnya milik Teensy.
    # Diminta R2C 14 Sep 2026: "capit harusnya menutup dulu baru mengangkat,
    # jangan bersamaan. Beri waktu capit menutup 1 detik baru naik."
    #
    # Diukur: rahang g50 -> g10 = 72 der servo, dan pada slew lengan 72 der/s
    # itu 1000 ms TANPA BEBAN. Jatah 'g10' sendiri sudah menutupi angka itu;
    # detik ini yang menutupi selisih bebannya -- boneka di antara rahang
    # memperlambat servo, dan tidak ada sensor yang bisa melaporkannya.
    #
    # Kalau yang ini diubah, ubah KORBAN_JEPIT_DIAM_MS juga: keduanya
    # menggerakkan satu capit yang sama.
    capit_diam_s: float = 1.0

    # LAJU SLEW LENGAN firmware (ARM_SLEW_DEG_S di config.h), derajat/detik.
    # Rahang capit ikut laju ini -- ia digerakkan servo lengan yang sama.
    #
    # 72 sejak 14 Sep 2026, turun dari 120. Angka ini ada di sini supaya jatah
    # tunggu rantai capit DIHITUNG, bukan ditulis tangan: tunggu tetap yang
    # dulu cukup pada 120 der/detik diam-diam jadi terlalu pendek pada 72, dan
    # akibatnya persis yang dijaga rantai ini -- perintah berikutnya lahir
    # sementara rahang masih berjalan.
    #
    # Kalau ARM_SLEW_DEG_S diubah, ubah yang ini juga.
    lengan_slew_deg_s: float = 72.0

    # Margin di atas waktu perjalanan rahang yang dihitung. Detik.
    #
    # Waktu hitungnya berlaku TANPA BEBAN dan tanpa gesekan; margin ini
    # menutupi selisih yang tidak bisa diukur dari sini. Ia BUKAN pengganti
    # capit_diam_s: margin menutupi galat perjalanan, capit_diam_s menutupi
    # beban korban di antara rahang.
    capit_margin_s: float = 0.5
    ambil_hanya_korban: bool = True
    # JEJAK & CENTER juga menolak dummy. Sebelumnya cuma rantai AMBIL yang
    # menolak, jadi robot tetap mengejar dan menengahkan dummy -- dan baru
    # menolak di detik terakhir, sesudah semua ongkos geraknya terbayar.
    jejak_hanya_korban: bool = True
    # Laju poll status ke Teensy. Dulu 2x/detik, dan tiap 'm'+'l' menumpahkan
    # ~20 baris ke log -- 40 baris per detik, cukup untuk menenggelamkan pesan
    # yang benar-benar penting dalam dua detik.
    poll_hz: float = 1.0
    # MODE FOKUS: berhenti mengirim gambar ke halaman. Encode JPEG 1280x720
    # memakan inti yang seharusnya dipakai inferensi, dan menggambar kotak
    # deteksi di atas frame juga tidak gratis. Saat menyetel penengahan,
    # gambarnya justru yang paling tidak dibutuhkan -- angka simpangan jauh
    # lebih berguna daripada melihat kotaknya bergerak.
    # Angka dan log TETAP mengalir; yang berhenti cuma videonya.
    fokus_vision: bool = False
    # SERAH-TERIMA OTOMATIS ke firmware v1.9.
    # Firmware berhenti di ruas ber-AKS_KONFIRM dan mencetak
    # "MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)". Di situlah vision
    # seharusnya menjawab, menggantikan jari operator.
    # Bawaan NYALA sejak 13 Sep 2026. Dulu MATI, dengan alasan "robot
    # melanjutkan misi karena kamera bilang begitu" -- dan itu benar selama
    # penengahan otomatis belum jadi jalur resmi. Sekarang ia jalur resminya:
    # firmware PARKIR menunggu 'm2', jadi auto_konfirm MATI berarti parkir itu
    # tidak pernah dijawab dan capit baru turun sesudah batas waktu habis.
    # Knob-nya tetap ada di tab Kalibrasi untuk mematikannya saat menyetel.
    auto_konfirm: bool = True
    # --- PEMICU VISION DARI FIRMWARE (v1.12) ---
    #
    # v1.12 mencetak "#KORBAN AMBIL <ruas>" saat berhenti di depan korban.
    # Itu SATU ARAH dan Vincent menulisnya begitu dengan sengaja: "Kirim
    # saja, tidak menunggu jawaban: TAHAP 1 memang tidak memakai balasannya."
    # Jadi begitu baris itu keluar, Teensy LANGSUNG menjalankan sekuens
    # capitnya sendiri -- ia tidak sedang menunggu siapa pun.
    #
    # Itulah kenapa knob ini ada dan kenapa bawaannya "lihat":
    #   mati       -- abaikan pemicunya.
    #   lihat      -- nyalakan vision, NOL perintah gerak. Aman dengan v1.12
    #                 apa adanya: kamera menilai dan melapor selagi lengan
    #                 firmware bekerja, tanpa berebut kendali.
    #   ambil_alih -- ambil alih gerakan, tengahkan, lalu jawab 'm2'.
    #
    # "ambil_alih" HANYA benar-benar mengambil alih kalau firmware terbukti
    # sedang PARKIR (mencetak MENUNGGU KONFIRMASI). Tanpa bukti itu ia turun
    # sendiri ke "lihat" dan mengatakan sebabnya. Alasannya bukan kehati-
    # hatian umum: menggerakkan badan selagi sekuens lengan berjalan berarti
    # dua penguasa untuk satu robot, dan yang kalah adalah korban yang
    # tersenggol capit yang sedang turun.
    # BAWAANNYA "ambil_alih" sejak firmware R2C punya parkir ber-batas-waktu.
    #
    # Sebelumnya "lihat", dan itu benar saat firmware tidak pernah menunggu:
    # satu-satunya yang aman memang MENONTON. Tapi itu juga yang membuat
    # gejala 12 Sep terasa seperti kerusakan -- vision menyala 1-2 detik,
    # menilai, lalu mati, dan capit turun sendiri. Tiap bagiannya bekerja
    # sesuai rancangan, dan hasil gabungannya tidak berguna.
    #
    # Dengan firmware R2C ('m8' NYALA + batas 40 detik), "ambil_alih" aman:
    # kalau HUD tidak sempat menjawab, firmware mengerjakan cara lama sendiri.
    # Dan penjaga buktinya TETAP berlaku -- kalau firmware yang terpasang
    # BELUM punya parkir, "ambil_alih" turun sendiri ke "lihat" dan menyebut
    # sebabnya. Jadi bawaan ini tidak bisa membuat robot bergerak di saat
    # yang salah, apa pun firmware yang sedang terpasang.
    pemicu_korban: str = "ambil_alih"

    # BERAPA LAMA VISION MASIH MENAHAN TENGAH SESUDAH CAPIT TERANGKAT.
    #
    # Dihitung dari baris '#LEPAS <ruas>' -- pengumuman firmware sendiri
    # bahwa sekuens capitnya selesai (fase terakhir 'R', korban sudah
    # terangkat). Selama detik ini badan TIDAK dinetralkan dan kaki masih
    # milik Pi, jadi korban yang baru terjepit tidak tersentak oleh pose
    # yang berubah di detik yang sama ia terangkat.
    #
    # Ongkosnya harus muat di MISI_LEPAS_BATAS_MS firmware (5 detik):
    #   tahan 1,0 + netralkan badan 1,6 ('r0 0 0' lalu 't0 0 0') + 'm9' 0,3
    #   = 2,9 detik. Menaikkannya di atas ~3 detik berarti firmware menyerah
    #   menunggu 'm9' lalu melangkah sendiri -- dan langkah pertamanya itu
    #   berjalan di atas pose badan yang belum sempat dinetralkan.
    lepas_tahan_s: float = 1.0

    # --- PENENGAHAN DUA TINGKAT ---
    # Sebabnya struktural, bukan tuning: firmware menganggap dirinya "lurus"
    # dalam HEADING_TOLERANCE_DEG = 6 der. Permintaan pivot di bawah itu
    # DIABAIKAN diam-diam -- persis gejala "robot bergerak tapi tidak jadi".
    # Di jarak 20 cm, 6 der = 2,1 cm. Kalau capit butuh +/-1 cm, pivot gait
    # SELAMANYA tidak bisa cukup. Tidak ada nilai PID yang memperbaikinya.
    #
    # Jadi dipakai dua aktuator berbeda:
    #   KASAR : 'O<der>' pivot gait   -- jangkauan bebas, resolusi 6 der
    #   HALUS : 'r0 0 <yaw>' putar BADAN di atas kaki, +/-20 der, TANPA melangkah
    #           dan TANPA deadband. Ini pose offset, bukan mode jalan.
    yaw_kasar_deg: float = 8.0     # di atas ini pakai pivot gait
    # PIVOT GAIT ('O') BUTUH IMU, dan selama IMU mati ia tidak menggerakkan
    # apa pun -- nav.pivotRelatif() kembali DIAM-DIAM, tanpa pesan.
    #
    # Itu yang membuat penengahan "macet tapi gerak sedikit": galat di atas
    # yaw_kasar_deg dikirim ke 'O', 'O' tidak berbuat apa-apa, galatnya tetap
    # sama, lalu 'O' dikirim lagi. Selamanya. Yang bergerak sedikit itu cuma
    # 'r0 0 0' yang menetralkan badan sebelum tiap percobaan.
    #
    # HUD sekarang MENGUKUR sendiri apakah 'O' bekerja: kalau dua kali
    # berturut-turut galatnya tidak berubah sesudah 'O', ia berhenti memakai
    # 'O' dan mengatakannya. Sesudah itu penengahan hanya memakai putar badan
    # (+-18 der) -- dan kalau sasarannya di luar itu, ia GAGAL dengan sebab,
    # bukan mencoba selamanya.
    pakai_pivot_gait: bool = True
    pivot_gait_beda_min: float = 1.5   # der; perubahan di bawah ini = 'O' diam
    pivot_gain: float = 0.6        # P-gain 'O'. 1.0 = minta seluruh galat =
                                   # dijamin overshoot, karena ada waktu mati
                                   # 2,5 detik antara mengukur dan bergerak.
    pivot_min_deg: float = 6.5     # >= HEADING_TOLERANCE_DEG firmware. Jangan
                                   # pernah mengirim yang lebih kecil: itu
                                   # perintah yang pasti dibuang tanpa pesan.
    # --- PID penengahan halus (putar badan) ---
    # Kenapa P saja tidak cukup, dan kenapa PID pun bukan obat utamanya:
    #
    # 'r0 0 <yaw>' itu perintah POSISI, bukan kecepatan. Firmware me-ramp ke
    # sana pada BODY_SLEW_DEG_S = 60 der/detik -- koreksi 5 der selesai dalam
    # 83 MILIDETIK. Itu sumber "terlalu cepat/drastis"-nya: bukan gain yang
    # kebesaran saja, tapi plant yang memang menyentak.
    #
    # Lalu kamera kita mengukur jauh lebih lambat daripada itu. Jadi lup ini
    # punya WAKTU MATI: kita mengukur, mengirim, badan sudah selesai bergerak,
    # tapi frame yang kita pakai untuk keputusan berikutnya masih frame lama.
    # Lup posisi + waktu mati + gain tinggi = persis gejala "drifting, tidak
    # pernah benar-benar tengah".
    #
    # Yang paling menolong justru BUKAN Ki/Kd, melainkan tiga hal di bawahnya:
    # batas langkah, tunggu-sampai-diam, dan penyaring galat. Ki/Kd tetap
    # disediakan karena kamu memintanya, dengan catatan jujur:
    #   Kp : inti. Turun 0,8 -> 0,35 karena ada waktu mati.
    #   Ki : gunanya SATU -- menghapus galat tetap dari pemasangan kamera
    #        (cx_offset_px yang belum pas, kamera tidak persis di sumbu putar).
    #        Untuk itu saja dia berguna; di lup posisi dia TIDAK menambah
    #        ketepatan. Dibatasi dan hanya aktif di dekat sasaran (anti-windup),
    #        kalau tidak dia menumpuk selama gerakan besar lalu dilepas sekaligus.
    #   Kd : meredam, TAPI turunan dari sensor berisik dan lambat itu berbahaya.
    #        Dihitung dari galat yang SUDAH disaring, dan sengaja kecil.
    # --- METODE PENENGAHAN ---
    # "sekali"  : SEKALI TEMBAK (bawaan). Kamera menempel di badan, jadi
    #             memutar badan theta mengubah bearing PERSIS -theta. Itu
    #             hubungan geometris yang SUDAH DIKETAHUI, bukan sesuatu yang
    #             perlu dicari-cari dengan umpan balik. Jadi: ukur sekali,
    #             koreksi SELURUHNYA, ukur lagi untuk memastikan. Dua gerakan,
    #             bukan sembilan.
    # "pid"     : umpan balik bertahap. Benar, tapi lambat -- tiap langkah
    #             cuma mengoreksi sebagian, dan tiap langkah menunggu badan diam.
    # "p"       : proporsional polos, untuk membandingkan.
    # "diam"   : BAWAAN. Hanya memakai bacaan yang diambil saat badan BENAR-BENAR
    #            DIAM, dirata-rata (EMA), lalu melangkah sebanding dengan
    #            jaraknya -- jauh = langkah besar, dekat = langkah halus.
    #            Ini yang menjawab "late information is false information":
    #            bacaan yang lahir SEBELUM gerakan terakhir selesai DIBUANG,
    #            bukan dirata-rata. Frame yang datang telat itu bukan data
    #            berisik, itu data SALAH -- dia menggambarkan pose yang sudah
    #            tidak ada lagi.
    # "sekali" : sekali tembak. Bagus di simulasi, TIDAK di robot nyata.
    # "pid"    : umpan balik bertahap.
    # "p"      : proporsional polos.
    metode_tengah: str = "diam"
    # Bobot EMA bacaan saat diam. Kecil = lebih halus tapi butuh lebih banyak
    # frame sebelum dipercaya.
    ema_alpha: float = 0.45
    # Berapa bacaan-saat-diam minimal sebelum boleh bergerak. Ini pengganti
    # median: bukan menyaring sesudahnya, tapi menolak bertindak sebelum
    # datanya cukup.
    diam_sampel_min: int = 3
    # Langkah = galat x gain, dijepit antara dua batas ini. Batas bawah ada
    # supaya koreksi kecil tidak jadi nol dan menggantung selamanya.
    langkah_gain: float = 0.55
    langkah_min_deg: float = 0.25
    # Faktor koreksi untuk metode sekali-tembak. Idealnya 1,0 (badan berputar
    # theta -> bearing berubah -theta). Nyatanya tidak persis, karena kamera
    # TIDAK duduk di sumbu putar: memutar badan ikut menggeser kamera ke
    # samping, dan di jarak 20 cm pergeseran itu terbaca sebagai sudut
    # tambahan. Karena itu angkanya DIPELAJARI dari respons nyata.
    skala_yaw: float = 1.0
    skala_belajar: bool = True
    skala_min: float = 0.4
    skala_maks: float = 2.0
    kp: float = 0.35
    ki: float = 0.05
    kd: float = 0.10
    # Batas perubahan SETPOINT per langkah. Ini rem yang sebenarnya: laju ramp
    # 60 der/detik milik firmware tidak bisa kita ubah, tapi kita bisa tidak
    # pernah memintanya bergerak jauh sekaligus.
    maks_langkah_deg: float = 2.0
    # Berapa lama menunggu badan benar-benar diam sebelum mengukur lagi.
    # Dihitung dari BODY_SLEW_DEG_S firmware + margin untuk kaki menetap.
    slew_deg_s: float = 60.0
    diam_margin_s: float = 0.35
    # Berapa sampel bearing yang dimedian sebelum dipakai. Median, bukan
    # rata-rata: satu frame nyasar tidak menggeser median, tapi menggeser
    # rata-rata.
    saring_n: int = 5
    halus_gain: float = 0.8      # dipakai hanya kalau pid_aktif = False
    pid_aktif: bool = True
    # Toleransi akhir dalam PIKSEL -- satuan yang benar-benar kamu lihat.
    # Diubah ke derajat memakai hfov: 8 px di 1280 px / 70,4 der = 0,44 der.
    # TOLERANSI TENGAH, dan KENAPA 8 px itu salah.
    #
    # 8 px di 1280 = 0,44 derajat. Langkah terkecil putar badan 0,25 der, dan
    # noise pengukuran bbox sendiri lebih besar dari itu. Jadi syaratnya
    # menuntut ketenangan yang tidak pernah bisa dicapai: robot melangkah,
    # mengukur, masih di luar 0,44 der, melangkah lagi -- selamanya.
    # 20 px = 1,1 der. Cukup ketat untuk capit, dan BISA dicapai.
    tengah_tol_px: float = 20.0
    # TAHAN DULU SEBELUM DIPUTUSKAN. Usul R2C 12 Sep 2026, dan ia benar:
    # satu frame yang kebetulan di tengah bukan bukti robot sudah diam di
    # tengah -- bisa jadi ia sedang melewatinya. Syaratnya harus BERTAHAN.
    #
    # Ini juga yang menggantikan peran batas waktu 40 detik sebagai pemicu:
    # penengahan yang sehat memenuhi syarat ini dalam beberapa detik dan
    # langsung menyerahkan ke firmware. 40 detik itu tenggat, bukan jadwal --
    # kalau ia yang mengakhiri, artinya ada yang salah, bukan ada yang lambat.
    #
    # DINOLKAN 17 Sep 2026, diminta R2C sesudah trial: jeda sebelum capit
    # turun terlalu lama. Dwell ini membeli keyakinan dengan detik, dan
    # keyakinannya sudah dibayar di tempat lain -- `tengah_yakin_deg` di bawah
    # menerima bacaan yang sudah MENGENDAP (lahir sesudah gerakan terakhir
    # selesai), dan itu bukti yang sama, tanpa menunggu.
    #
    # Naikkan lagi ke 2,0 kalau 'm2' mulai terkirim saat badan masih berayun.
    tengah_tahan_s: float = 0.0
    # YAKIN -> LANGSUNG, tanpa menunggu sisa dwell.
    #
    # Dwell di atas ada untuk membuktikan robot DIAM di tengah, bukan sedang
    # MELEWATINYA. Tapi bukti itu bisa datang lebih cepat: kalau bacaan yang
    # sudah MENGENDAP (diam.boleh -- minimal diam_sampel_min sampel yang lahir
    # SESUDAH gerakan terakhir selesai) berada jauh di dalam toleransi, robot
    # sudah terbukti diam DAN di tengah. Menunggu sisa detiknya cuma membakar
    # waktu kontes.
    #
    # 0,6 der pada standoff 20 cm kira-kira 2 mm -- jauh lebih rapat daripada
    # yang sanggup dimanfaatkan capit. Setel 0 untuk mematikan jalur ini dan
    # kembali menuntut dwell penuh.
    tengah_yakin_deg: float = 0.6
    # PITA BAWAH YANG DITUTUPI CAPIT. Saat lengan diturunkan, dia menutupi
    # bagian bawah frame. Deteksi yang PUSATNYA di pita itu dibuang, bukan
    # dipercaya -- capit oranye di depan lensa itu justru mirip korban.
    # 0 = tidak ada yang ditutupi. 0,45 = 45% bawah frame diabaikan.
    # Ukur sekali: turunkan lengan, lihat stream, catat di ketinggian berapa
    # capit mulai terlihat.
    roi_bawah_frac: float = 0.0
    # PITA ATAS YANG DITUTUPI KORBAN YANG SEDANG DIGENDONG.
    #
    # Pose REHAT ('R', config.h:97) itu 80 mm ke DEPAN dan 160 mm DI ATAS
    # bidang pusat badan -- lengan terlipat ke atas badan. Komentar Vincent
    # menyebut gunanya: supaya tidak menghalangi LiDAR depan dan tidak
    # tersangkut saat menaiki tangga. Yang TIDAK disebut: kamera.
    #
    # Kalau korban digendong di pose itu, ia duduk tinggi dan dekat -- dan
    # dari kamera ia terlihat seperti korban, karena memang korban. Ukur
    # sekali: gendong korban, lihat stream, catat di ketinggian berapa ia
    # mulai terlihat, lalu isi fraksinya. 0 = tidak ada yang ditutupi.
    roi_atas_frac: float = 0.0
    # Jangan mengejar sasaran selagi capit SEDANG memegang korban.
    #
    # Ini penjaga yang tidak bergantung geometri sama sekali, dan itu
    # sengaja: korban yang digendong akan terdeteksi sebagai korban (ia
    # memang korban), jadi JEJAK/CENTER akan menengahkan badan ke barang
    # yang sudah ada di tangannya sendiri. Robot mengejar muatannya.
    # Firmware sudah mengumumkan keadaannya di baris status 'membawa',
    # jadi tidak perlu ditebak.
    #
    # MATI sejak revert 12 Sep. Alasannya bukan bahwa idenya salah -- idenya
    # masih benar -- tapi letaknya salah: ia memotong `lolos` SESUDAH saring,
    # jadi kotaknya jadi abu-abu "disaring" tanpa sebab dan JEJAK berhenti
    # mengikuti korban. Yang memicunya baris status 'membawa' dari firmware,
    # dan baris itu bertahan di link.terakhir sampai status berikutnya datang:
    # sekali tercetak, ia masih berbunyi "membawa" lama sesudah korban
    # diletakkan. Gerbang yang bergantung pada keadaan basi tidak boleh
    # memegang kendali penglihatan. Nyalakan hanya kalau nanti baris itu
    # sudah dibuktikan segar.
    tolak_saat_membawa: bool = False
    # SIKLUS DAYA KAMERA, dipicu Teensy. Usul R2C: kamera tidak perlu hidup
    # kalau Teensy tidak memintanya.
    #   pemicu #KORBAN tiba      -> kamera dinyalakan
    #   penengahan selesai, m2   -> kamera dimatikan lagi
    # Jadi kamera hidup hanya selama jendela yang benar-benar membutuhkannya,
    # beberapa detik per korban alih-alih sepanjang misi.
    #
    # BAWAANNYA DIBALIK JADI NYALA, 17 Sep 2026. Daya memang sudah jadi
    # masalah di arena: Pi 5 mati mendadak berulang kali, dan dmesg mencatat
    # "hwmon hwmon4: Undervoltage detected!" pada detik 6,9 dan 17,0 sesudah
    # boot -- jauh sebelum HUD, ONNX, atau misi mana pun jalan.
    #
    # Itu berarti saklar ini BUKAN obatnya: under-voltage terjadi walau kamera
    # tidak pernah menyala. Ia peredam -- memangkas konsumen terbesar kedua
    # (C922 plus inferensi) dari sepanjang misi jadi beberapa detik per
    # korban, sehingga puncak serentaknya lebih jarang tercapai.
    #
    # Obatnya catu daya: Pi 5 menuntut 5,1 V / 5 A, dan node
    # /sys/firmware/devicetree/base/chosen/power/max_current_supported TIDAK
    # ADA di robot ini -- firmware Pi tidak pernah menegosiasikan 5 A.
    #
    # DIBALIK LAGI JADI MATI, 17 Sep 2026, diminta R2C sesudah trial: kamera
    # harus tetap menyala dari misi mulai sampai misi selesai.
    #
    # Sebabnya siklusnya sendiri yang mahal, bukan cuma dayanya. Menyalakan
    # C922 berarti membuka perangkat, menunggu auto-exposure mengendap, dan
    # frame pertama sesudah itu masih gelap atau buram -- tepat di detik saat
    # penengahan korban mau memakainya. Sekali per korban, lima kali per misi.
    #
    # Yang hilang: peredam under-voltage. Baca paragraf di atas sebelum
    # membiarkannya mati -- Pi 5 di robot ini memang pernah mati mendadak, dan
    # obat sebenarnya ada di catu daya, bukan di sini. Kalau brownout kembali
    # sebelum catunya diperbaiki, nyalakan lagi lewat tab Kalibrasi.
    kamera_ikut_pemicu: bool = False
    badan_yaw_maks: float = 18.0   # < BODY_MAX_ROT_DEG (20) firmware
    # 35 dipilih waktu BODY_MAX_TRANS_MM firmware masih 40. Ia naik ke 80 pada
    # 16 Sep 2026, jadi pagar ini sekarang jauh lebih ketat daripada yang
    # firmware paksakan -- dan sejak 17 Sep ia satu-satunya cara Pi
    # menengahkan, karena 'H' dan 'V' dibuang. Naikkan kalau HUD sering
    # menulis "MENTOK".
    badan_geser_maks: float = 35.0 # < BODY_MAX_TRANS_MM (80) firmware
    tengah_tol_deg: float = 1.5    # toleransi AKHIR sesudah koreksi halus
    # Pagar jarak untuk rantai AMBIL. v1.7 menaikkan LIDAR_MAX_CM 70 -> 130,
    # jadi bacaan besar tidak lagi otomatis jadi "jauh" -- pagar ini yang
    # menggantikan peran batas itu untuk urusan mengambil korban.
    ambil_maks_cm: float = 60.0   # tolak kalau sasaran terakhir DUMMY
    conf_min: float = 0.60
    n_frame: int = 9
    k_of_n: int = 7
    kelas_korban: str = "korban"
    kelas_dummy: str = "dummy"

    def f_px(self, w):
        """Panjang fokus dalam piksel dari hfov. Sama untuk sumbu tegak & datar
        pada lensa rektilinear, jadi satu angka ini cukup untuk keduanya."""
        return (w / 2.0) / math.tan(math.radians(self.hfov_deg / 2.0))

    def px_per_deg(self, w):
        return w / self.hfov_deg

    def roi_half_px(self, w):
        return self.roi_half_deg * self.px_per_deg(w)


# KNOB YANG DIPAKAI K-3/K-4, untuk kartu kalibrasinya sendiri di HUD.
#
# Kartu terpisah karena kartu kalibrasi utama sudah puluhan baris, dan saat
# menyetel K-3/K-4 di arena yang dibutuhkan cuma sembilan angka ini. Mencari
# sembilan di antara puluhan, di layar ponsel, sambil robot menunggu, adalah
# cara kehilangan waktu latihan.
# Berkas kalibrasi yang dimuat OTOMATIS tiap HUD start. Nama tetap, bukan
# bertanggal: yang bertanggal untuk riwayat, yang ini untuk "setelan yang
# berlaku sekarang".
KALIB_AKTIF = "kalib_aktif.json"

# DIPANGKAS 17 Sep 2026, sesudah 'H' dan 'V' dibuang. Enam knob di sini sudah
# tidak dibaca siapa pun: geser_amp, geser_halus_px, geser_maks_langkah,
# geser_v_cm_k3, geser_v_cm_k4 (ketiganya milik tingkat KASAR yang melangkah)
# dan korban_belakang_cm (milik 'J'). Knob mati di kartu arena lebih buruk
# daripada knob yang tidak ada: operator menyetelnya sambil robot menunggu,
# lalu menyimpulkan yang salah waktu tidak terjadi apa-apa.
#
# Bidangnya tetap ada di Kalib -- berkas kalibrasi lama tetap termuat, dan
# korban_belakang_cm masih berguna sebagai CATATAN angka standoff yang
# sekarang ditulis ke ruas HNT_MUNDUR.
#
# badan_geser_maks MASUK: sejak 'H' dan 'V' hilang, ia satu-satunya yang
# menentukan sejauh apa Pi masih bisa menengahkan.
KALIB_K34 = [
    "condong_mm", "condong_jeda_ms", "condong_yaw",
    "badan_geser_maks", "geser_tunggu_s",
    "korban_jarak_cm",
]

KALIB_FIELDS = [f for f in Kalib.__dataclass_fields__
                if f not in ("kelas_korban", "kelas_dummy")]

# Field bertipe TEKS: dibuat daftar pilihan, bukan kotak ketik bebas.
# Salah ketik satu huruf di sini diam-diam mengubah metode kendali robot.
KALIB_PILIHAN = {"metode_tengah": ["diam", "sekali", "pid", "p"],
                 "mode_ambil": ["lengan", "badan"],
                 "pemicu_korban": ["mati", "lihat", "ambil_alih"]}


# =====================================================================
# 4. SAMBUNGAN KE TEENSY
# ---------------------------------------------------------------------
# TANPA FILTER, 13 September 2026, atas keputusan R2C.
#
# Sampai 12 Sep ada dua saringan di jalur kirim: whitelist MODE BACA dan
# daftar TERLARANG. Keduanya dibuang, dan alasannya bukan kemalasan:
#
#   1. Keselamatan dipegang MEKANIK. Itu tempat yang benar untuknya --
#      saringan perangkat lunak tidak pernah bisa menahan servo yang sudah
#      bergerak, dan robot ini sudah punya penahannya sendiri.
#   2. Saringan yang menolak DIAM-DIAM menyembunyikan jawaban firmware yang
#      justru sedang dicari. 'C' pernah begitu berhari-hari: HUD menolaknya
#      lebih dulu, jadi pesan "Gagal: Tidak ada data IMU." yang seharusnya
#      muncul tidak pernah terlihat, dan satu masalah menyamar jadi lima.
#   3. Setiap perintah baru di firmware Vincent berarti satu baris baru di
#      daftar ini. Daftar yang tertinggal satu versi terbaca sebagai
#      "fiturnya tidak jalan", bukan sebagai penolakan yang bisa dibaca.
#
# Konsekuensinya disebut terbuka, bukan disembunyikan: 'x' tetap mematikan
# seluruh PWM dan robot AMBRUK; 'S', 'W' dan 'e' tetap menulis EEPROM. Tidak
# ada lagi yang menahannya dari sini.
# =====================================================================

# Perintah yang menyalakan ALIRAN CETAK terus-menerus di firmware. 's' TIDAK
# menghentikannya -- 's' menghentikan GERAK, bukan cetakan. Satu-satunya yang
# mematikannya huruf yang sama lagi, dan itulah sebabnya log bisa terus
# membanjir walau robotnya sudah diam.
ALIRAN = {"y": "aliran yaw", "L": "aliran LiDAR"}


POLA_PORT = ("/dev/serial/by-id/*Teensy*", "/dev/serial/by-id/*teensy*",
             "/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")


def cari_port(rinci=False):
    """Tebak port Teensy. by-id lebih stabil daripada ttyACM0 yang bisa geser.

    rinci=True mengembalikan (port, catatan) supaya kegagalan bisa DIJELASKAN.
    Dulu fungsi ini mengembalikan None tanpa sepatah kata pun, dan itu jadi
    lubang diagnosis: HUD diam, log kosong, tidak ada yang bisa dibaca.
    """
    terlihat = []
    pilih = None
    for pola in POLA_PORT:
        hit = sorted(glob.glob(pola))
        terlihat += hit
        if hit and pilih is None:
            pilih = hit[0]
    if not rinci:
        return pilih
    catatan = ("terlihat: " + ", ".join(terlihat)) if terlihat else \
              ("tidak ada kandidat (" + " ".join(POLA_PORT) + ")")
    return pilih, catatan


def daftar_video():
    """Node /dev/videoN yang ada, node USB biasa (0..9) didahulukan.

    Di Raspberry Pi, video10 ke atas adalah node ISP/codec bawaan board --
    ada tapi bukan kamera. Mencobanya lebih dulu cuma membuang waktu.
    """
    n = []
    for p in glob.glob("/dev/video*"):
        ekor = p[len("/dev/video"):]
        if ekor.isdigit():
            n.append(int(ekor))
    return sorted(n, key=lambda i: (i >= 10, i))


# Node yang terbukti bukan kamera, beserta KAPAN ia dicoret. Sengaja tidak
# disimpan ke berkas: susunan /dev/video* bisa berubah setelah reboot atau
# ganti port USB, dan daftar basi yang membuang node yang benar lebih mahal
# daripada satu pemindaian ulang.
#
# DULU set() PERMANEN, dan itu bug yang mematikan kamera sepanjang sesi.
# Node ISP memang tidak akan pernah jadi kamera, tapi WEBCAM SUNGGUHAN bisa
# gagal frame pertamanya: C922 yang baru dapat daya membuka perangkatnya
# beberapa ratus milidetik sebelum sanggup memberi gambar. Sekali itu terjadi,
# /dev/video0 dicoret selamanya -- dan karena sambung ulang tiap 0,4 detik juga
# melewati daftar ini, tidak ada satu pun percobaan berikutnya yang menolong.
# Gejalanya "kamera tidak ditemukan" padahal nodenya jelas tercetak di pesan
# itu sendiri, dan cuma restart layanan yang memulihkannya.
_NODE_BURUK = {}

# Lama sebuah node dicoret sebelum dicoba lagi, detik. Bukan permanen (lihat
# di atas), bukan pula nol: tanpa jeda, node ISP dicoba ulang tiap 0,4 detik
# dan tiap percobaan membakar beberapa detik di ioctl V4L2.
NODE_BURUK_DETIK = 60.0

# Berapa kali frame percobaan diminta sebelum sebuah node dihukum. Node ISP
# gagal ketiganya sama saja; webcam yang sedang bangun lulus di percobaan
# kedua atau ketiga. Jeda 0,15 detik dipilih supaya satu node menghabiskan
# paling banyak ~0,45 detik -- seluruh sapuan tetap di bawah satu detik untuk
# node USB, dan semuanya berjalan di thread sambung ulang, bukan loop utama.
NODE_COBA_BACA = 3
NODE_JEDA_BACA = 0.15


def buka_kamera(sumber, w, h, fps, utama=None):
    """-> (cap, pesan). cap None kalau tidak ada yang bisa dipakai.

    `utama` = nomor node yang TERAKHIR BERHASIL. Dicoba paling dulu, dan itu
    yang memotong lama sambung ulang dari belasan detik jadi sekitar satu.
    Sebabnya: daftar_video() mengembalikan semua node, dan di Pi node ISP
    bawaan board (video19..22) mau DIBUKA tapi tidak pernah memberi frame --
    tiap satunya membakar beberapa detik di open_camera() lalu cap.read()
    sebelum menyerah. Webcam yang baru saja putus hampir selalu muncul lagi di
    nomor yang sama, jadi mencobanya duluan hampir selalu langsung kena.

    open_camera() milik detect.py menganggap "terbuka" sudah cukup. Di Pi itu
    tidak cukup: node ISP bawaan board (video19..22) mau dibuka tapi tidak
    pernah memberi frame. Jadi syarat lulusnya di sini adalah satu frame
    sungguhan.

    TIDAK melempar SystemExit dengan sengaja. Kamera yang belum tercolok
    bukan alasan mematikan HUD -- justru saat itulah kamu paling butuh
    halamannya untuk tahu apa yang salah.
    """
    kandidat = daftar_video() if str(sumber) == "auto" else [int(sumber)]
    # NODE YANG SUDAH TERBUKTI BUKAN KAMERA DIBUANG, dan ini yang paling
    # memotong waktu sambung ulang. Node ISP bawaan Pi (video19..22) MAU dibuka
    # tapi tidak pernah memberi frame, dan tiap satunya membakar beberapa detik
    # di open_camera() lalu cap.read() sebelum menyerah. Dicoba sekali per
    # sesi, sesudah itu dilewati -- ia tidak akan berubah jadi kamera.
    sekarang = time.time()
    kandidat = [i for i in kandidat
                if sekarang - _NODE_BURUK.get(i, -1e9) >= NODE_BURUK_DETIK]
    if utama is not None and utama in kandidat:
        kandidat = [utama] + [i for i in kandidat if i != utama]
    for i in kandidat:
        try:
            cap = open_camera(i, w, h, fps)
        except SystemExit:
            continue
        except Exception:                                # noqa: BLE001
            continue
        # BEBERAPA KALI, bukan sekali. Webcam yang baru dapat daya membuka
        # perangkatnya lebih dulu daripada sanggup memberi gambar; menghukumnya
        # di frame pertama membuang kamera yang sebenarnya sehat.
        ok = False
        for n in range(NODE_COBA_BACA):
            try:
                ok, _ = cap.read()
            except Exception:                            # noqa: BLE001
                ok = False
            if ok:
                break
            if n + 1 < NODE_COBA_BACA:
                time.sleep(NODE_JEDA_BACA)
        if ok:
            _NODE_BURUK.pop(i, None)     # terbukti kamera -- bersihkan catatan
            return cap, f"/dev/video{i}"
        # Terbuka tapi tidak memberi frame: kemungkinan besar node ISP bawaan
        # board. Dicoret SEMENTARA, bukan selamanya -- lihat _NODE_BURUK.
        # Kamera sungguhan yang sedang sibuk gagal di open_camera() (exception
        # di atas), bukan di sini, jadi ia tidak ikut tercoret.
        _NODE_BURUK[i] = time.time()
        cap.release()

    ada = daftar_video()
    daftar = ", ".join(f"/dev/video{i}" for i in ada) if ada else "TIDAK ADA"
    return None, (f"kamera tidak ditemukan. Node terlihat: {daftar}. "
                  "Periksa: lsusb | v4l2-ctl --list-devices | groups (harus ada "
                  "'video') | sudo fuser -v /dev/video*")


class Kamera:
    """Pembungkus CameraThread yang boleh KOSONG dan mencoba lagi sendiri.

    Tanpa ini, webcam yang belum tercolok membuat layanan systemd mati lalu
    dihidupkan ulang terus-menerus -- dan halaman webnya ikut hilang, persis
    saat kamu butuh membacanya.
    """

    def __init__(self, sumber, w, h, fps, jeda=0.4):
        self.args = (sumber, w, h, fps)
        self.jeda = jeda
        self.cap = None
        self.thread = None
        self.pesan = "mencari kamera..."
        self._t = 0.0
        # PEKERJA SAMBUNG ULANG. Tanpa ini, pastikan() memanggil buka_kamera()
        # -- yaitu open_camera() dan cap.read() -- LANGSUNG DI LOOP UTAMA.
        # Keduanya ioctl V4L2, dan pada bus USB yang macet (webcam
        # re-enumerasi sesudah tegangan turun) ioctl itu menggantung berdetik
        # -detik sampai selamanya.
        #
        # Yang membeku bukan cuma gambarnya: loop utama itu juga yang memompa
        # antrean aksi dan menulis ke serial. Jadi kamera yang macet =
        # "LOOP BEKU" di HUD DAN perintah serial berhenti terkirim, persis
        # laporan R2C 15 Sep 2026. Sambungan ulang dipindah ke thread sendiri
        # supaya kemacetan V4L2 tinggal di sana.
        self._kerja = None
        self._kunci = threading.Lock()
        # Node yang terakhir berhasil. Webcam yang putus karena tegangan turun
        # muncul lagi di nomor yang sama hampir setiap kali, jadi menyimpannya
        # mengubah sambung ulang dari "sapu seluruh /dev/video*" jadi "coba
        # yang itu dulu".
        self._node = None
        # SAKLAR DAYA. Mematikan kamera BENAR-BENAR melepas perangkatnya
        # (thread.release()), bukan cuma berhenti menggambar kotak -- itu
        # bedanya dengan fokus_vision, yang cuma berhenti mengirim video
        # sementara kamera & inferensi tetap jalan.
        #
        # Webcam C922 menarik daya terus-menerus selama terbuka, dan di robot
        # ini daya bukan urusan kecil: tegangan 4,9 V sudah cukup membuat USB
        # macet, dan bit throttle Pi melatch. Jadi kamera yang tidak sedang
        # dibutuhkan memang lebih baik mati.
        self.nyala = True
        self.sebab_mati = ""

    @property
    def ok(self):
        return self.thread is not None and self.thread.running

    @property
    def sibuk(self):
        k = self._kerja
        return k is not None and k.is_alive()

    def pastikan(self):
        """TIDAK PERNAH MEMBLOKIR. True kalau kamera siap dipakai saat ini.

        Seluruh kerja yang bisa menggantung -- membuka perangkat, membaca
        frame percobaan, melepas perangkat -- dijalankan thread sendiri.
        Pemanggil di loop utama cuma membaca hasilnya.
        """
        if not self.nyala:
            # Dimatikan operator. Kalau perangkatnya masih dipegang, LEPAS --
            # tanpa itu "mati" cuma berarti berhenti membaca, dan dayanya tetap
            # terpakai. Yang diminta penghematan daya, bukan layar kosong.
            if (self.thread is not None or self.cap is not None) and not self.sibuk:
                self._mulai_kerja(self._lepas_kerja)
            self.pesan = ("kamera DIMATIKAN operator"
                          + (f" -- {self.sebab_mati}" if self.sebab_mati else ""))
            return False
        if self.ok:
            return True
        if self.sibuk:
            return False                                 # pekerja masih jalan
        if self.thread is not None:                      # mati di tengah jalan
            self.pesan = "kamera terputus -- mencoba sambung ulang"
            self._mulai_kerja(self._lepas_kerja)
            return False
        if time.time() - self._t < self.jeda:
            return False
        self._t = time.time()
        self._mulai_kerja(self._sambung_kerja)
        return False

    def _mulai_kerja(self, fn):
        self._kerja = threading.Thread(target=fn, daemon=True)
        self._kerja.start()

    def _lepas_kerja(self):
        self.lepas()

    def _sambung_kerja(self):
        cap, pesan = buka_kamera(*self.args, utama=self._node)
        self.pesan = pesan
        if cap is None:
            return
        # "/dev/video3" -> 3. Gagal mengurai bukan alasan menolak kameranya;
        # yang hilang cuma jalan pintas percobaan berikutnya.
        try:
            self._node = int(pesan.rsplit("video", 1)[1])
        except (IndexError, ValueError):
            pass
        with self._kunci:
            self.cap = cap
            self.thread = CameraThread(cap)
        print(f"[KAMERA] {pesan} dipakai")

    def read(self, seq, batas=1.0):
        """Ambil frame baru, TAPI menyerah sesudah `batas` detik.

        CameraThread.read() milik detect.py menunggu SELAMANYA sampai ada
        frame baru. Itu benar untuk detektor yang tidak punya pekerjaan lain,
        tapi mematikan di sini: webcam USB yang macet (dan pada tegangan 4,9 V
        itu sering) membuat loop utama berhenti di dalamnya -- HUD tersaji
        tapi seluruh isinya kosong, tanpa satu pun petunjuk. Jadi kita
        mengintip field-nya sendiri dengan tenggat.
        """
        if not self.ok:
            return None, seq
        t0 = time.time()
        while time.time() - t0 < batas:
            if not self.thread.running:
                return None, seq
            with self.thread.lock:
                if self.thread.seq != seq and self.thread.frame is not None:
                    return self.thread.frame, self.thread.seq
            time.sleep(0.002)
        self.pesan = f"kamera tidak memberi frame baru selama {batas:.0f} detik"
        return None, seq

    def saklar(self, on, sebab=""):
        """Nyalakan/matikan kamera. Kembalikan teks keadaan barunya."""
        self.nyala = bool(on)
        self.sebab_mati = sebab if not on else ""
        if not self.nyala:
            self.lepas()
            self._t = 0.0          # supaya menyala lagi tanpa menunggu jeda
            return "kamera DIMATIKAN -- perangkatnya dilepas, daya hemat"
        return "kamera dinyalakan -- menyambung ulang"

    def lepas(self):
        try:
            if self.thread is not None:
                self.thread.release()
            elif self.cap is not None:
                self.cap.release()
        except Exception:                                # noqa: BLE001
            pass
        self.thread = None
        self.cap = None


class Kesehatan:
    """Daya, suhu dan status Pi. Dibaca pelan-pelan di thread sendiri.

    Kenapa ada: under-voltage tidak pernah muncul sebagai pesan error -- Pi
    cuma MATI. Halaman ini yang membuatnya kelihatan datang, bukan ditebak
    sesudahnya.

    BATAS YANG HARUS KAMU TAHU, supaya angkanya tidak dipercaya berlebihan:

      Tegangan dan arus di sini dibaca dari ADC PMIC, beberapa kali per detik.
      Brownout yang menjatuhkan Pi berlangsung 10 milidetik. ADC ini TIDAK
      BISA melihatnya, persis seperti multimeter yang merata-rata.

      Yang BISA melihatnya cuma bit `throttled` -- perangkat kerasnya sendiri
      yang melatch, dan bit "pernah terjadi" (0x10000) menempel sampai reboot.
      Jadi kalau tegangan terlihat adem tapi bit "pernah" menyala, yang benar
      bit itu, bukan angka voltnya.

    Karena itu 'volt_min' dan 'daya_max' dicatat: nilai sesaat menipu, jejak
    terendah/tertinggi sejak HUD hidup jauh lebih memberi tahu.
    """

    ARTI = {0: "under-voltage SEKARANG", 1: "frekuensi dibatasi",
            2: "sedang di-throttle", 3: "batas suhu lunak",
            16: "under-voltage pernah terjadi", 17: "pernah dibatasi",
            18: "pernah di-throttle", 19: "pernah kena batas suhu"}

    # Baris pmic_read_adc: " EXT5V_V volt(24)=5.09V" / " 3V3_SYS_A current(1)=0.02A"
    RE_PMIC = re.compile(r"^\s*(\S+?)_([AV])\s+(volt|current)\(\d+\)=([\d.]+)")

    def __init__(self, jeda=2.0):
        self.suhu = 0.0
        self.throttled = 0
        self.volt = None          # tegangan masuk 5 V (EXT5V), None = tak terbaca
        self.volt_min = None      # terendah sejak HUD hidup
        self.daya = None          # perkiraan konsumsi papan, watt
        self.daya_max = None
        self.arus = None          # total arus rel PMIC, ampere
        self.suhu_max = 0.0
        self.n_overcurrent = 0    # kejadian over-current USB di log kernel
        self.pmic_sebab = ""      # kenapa volt/daya kosong, kalau kosong
        self.oc_sebab = ""
        self.jeda = jeda
        self.riwayat = deque(maxlen=90)   # (volt, daya) untuk grafik kecil
        threading.Thread(target=self._loop, daemon=True).start()

    # ---------------- pembacaan ----------------
    @staticmethod
    def _vcgencmd(*arg):
        return subprocess.run(["vcgencmd", *arg], capture_output=True,
                              text=True, timeout=4).stdout

    def _baca_pmic(self):
        """Tegangan & arus per rel dari PMIC Pi 5.

        Pi 4 dan sebelumnya TIDAK punya perintah ini -- di situ kolom daya
        memang kosong, dan itu bukan kerusakan. Sebabnya ikut ditulis supaya
        tidak jadi tanda tanya.
        """
        try:
            out = self._vcgencmd("pmic_read_adc")
        except FileNotFoundError:
            self.pmic_sebab = "vcgencmd tidak ada"
            return
        except Exception as e:                           # noqa: BLE001
            self.pmic_sebab = f"{type(e).__name__}"
            return
        if not out.strip():
            self.pmic_sebab = "pmic_read_adc kosong (Pi 4 atau lebih lama?)"
            return

        volt, arus = {}, {}
        for baris in out.splitlines():
            m = self.RE_PMIC.match(baris)
            if not m:
                continue
            nama, _, jenis, nilai = m.groups()
            (volt if jenis == "volt" else arus)[nama] = float(nilai)

        if not volt and not arus:
            self.pmic_sebab = "format pmic_read_adc tidak dikenali"
            return
        self.pmic_sebab = ""

        # EXT5V = tegangan yang BENAR-BENAR masuk ke papan. Ini angka yang
        # dibandingkan dengan 4,8 V, bukan tegangan inti (core) yang memang
        # selalu ~0,7-0,9 V dan tidak memberi tahu apa-apa soal catu daya.
        self.volt = volt.get("EXT5V")
        if self.volt is not None:
            self.volt_min = (self.volt if self.volt_min is None
                             else min(self.volt_min, self.volt))

        # Daya = jumlah (V x I) tiap rel yang punya KEDUANYA. Rel yang cuma
        # punya salah satu dilewati, bukan dianggap nol -- menganggap nol
        # membuat total terlihat lebih adem daripada kenyataannya.
        total_w = total_a = 0.0
        ada = False
        for nama, i in arus.items():
            v = volt.get(nama)
            if v is None:
                continue
            total_w += v * i
            total_a += i
            ada = True
        if ada:
            self.daya, self.arus = total_w, total_a
            self.daya_max = (total_w if self.daya_max is None
                             else max(self.daya_max, total_w))

    def _baca_overcurrent(self):
        """Hitung kejadian over-current USB di log kernel.

        Pi TIDAK punya bit status over-current -- get_throttled tidak
        mengenalnya. Satu-satunya jejak adalah pesan kernel. Kalau log tidak
        bisa dibaca, kolomnya bilang begitu, bukan menampilkan 0 yang
        menyesatkan.
        """
        for cmd in (["journalctl", "-k", "--since", "-10min", "--no-pager"],
                    ["dmesg"]):
            try:
                out = subprocess.run(cmd, capture_output=True, text=True,
                                     timeout=5)
            except Exception:                            # noqa: BLE001
                continue
            if out.returncode != 0:
                continue
            self.n_overcurrent = sum(
                1 for b in out.stdout.splitlines()
                if "over-current" in b.lower() or "overcurrent" in b.lower())
            self.oc_sebab = ""
            return
        self.oc_sebab = "log kernel tidak terbaca (perlu grup 'adm')"

    def _loop(self):
        n = 0
        while True:
            try:
                with open("/sys/class/thermal/thermal_zone0/temp") as f:
                    self.suhu = int(f.read().strip()) / 1000.0
                    self.suhu_max = max(self.suhu_max, self.suhu)
            except Exception:                            # noqa: BLE001
                pass
            try:
                out = self._vcgencmd("get_throttled")
                self.throttled = int(out.strip().split("=")[1], 16)
            except Exception:                            # noqa: BLE001
                pass
            self._baca_pmic()
            if self.volt is not None or self.daya is not None:
                self.riwayat.append((self.volt or 0.0, self.daya or 0.0))
            # Log kernel jauh lebih mahal dibaca daripada vcgencmd, dan
            # over-current tidak datang tiap detik. Sekali per ~30 detik cukup.
            if n % 15 == 0:
                self._baca_overcurrent()
            n += 1
            time.sleep(self.jeda)

    # ---------------- penilaian ----------------
    @property
    def gawat(self):
        """Bit 0-3 = sedang terjadi SEKARANG. Itu yang mendahului mati."""
        return bool(self.throttled & 0xF)

    def status(self):
        """Satu kata untuk keadaan Pi, plus warnanya.

        Urutannya sengaja: yang paling mendesak menang. Under-voltage yang
        SEDANG terjadi mengalahkan apa pun, karena itu yang beberapa detik
        lagi mematikan Pi.
        """
        if self.throttled & 0x1:
            return "UNDER-VOLTAGE", "var(--bad)"
        if self.n_overcurrent:
            return "OVER-CURRENT USB", "var(--bad)"
        if self.throttled & 0x4:
            return "DI-THROTTLE", "var(--bad)"
        if self.throttled & 0x2:
            return "FREKUENSI DIBATASI", "var(--warn)"
        if self.throttled & 0x8:
            return "BATAS SUHU", "var(--warn)"
        if self.throttled & 0x10000:
            return "NORMAL (pernah under-voltage)", "var(--warn)"
        if self.throttled & 0xF0000:
            return "NORMAL (pernah di-throttle)", "var(--warn)"
        if self.suhu >= 80:
            return "PANAS", "var(--warn)"
        if not self.suhu:
            return "-", "var(--dim)"
        return "NORMAL", "var(--ok)"

    def teks(self):
        """Ringkasan untuk strip status yang selalu terlihat."""
        if not self.suhu and not self.throttled:
            return "-"
        t = f"{self.suhu:.0f}C"
        if self.volt is not None:
            t += f"  {self.volt:.2f}V"
        if self.daya is not None:
            t += f"  {self.daya:.1f}W"
        if self.throttled:
            aktif = [n for b, n in self.ARTI.items() if self.throttled & (1 << b)]
            t += "  " + ", ".join(aktif)
        return t

    def rinci(self):
        """Isi kartu Daya & suhu di tab Robot."""
        st, warna = self.status()

        def volt_teks():
            if self.volt is None:
                return self.pmic_sebab or "tidak terbaca", "var(--dim)"
            w = ("var(--bad)" if self.volt < 4.75 else
                 "var(--warn)" if self.volt < 4.90 else "var(--ok)")
            s = f"{self.volt:.2f} V"
            if self.volt_min is not None:
                s += f"   (terendah {self.volt_min:.2f} V)"
            return s, w

        v_teks, v_warna = volt_teks()
        return {
            "status": st,
            "status_warna": warna,
            "suhu": (f"{self.suhu:.1f} C   (tertinggi {self.suhu_max:.1f} C)"
                     if self.suhu else "-"),
            "suhu_warna": ("var(--bad)" if self.suhu >= 80 else
                           "var(--warn)" if self.suhu >= 70 else "var(--ok)"),
            "volt": v_teks,
            "volt_warna": v_warna,
            "daya": ((f"{self.daya:.2f} W" if self.daya is not None else
                      self.pmic_sebab or "tidak terbaca")
                     + (f"   (puncak {self.daya_max:.2f} W)"
                        if self.daya_max is not None else "")),
            "arus": (f"{self.arus:.2f} A" if self.arus is not None else "-"),
            "overcurrent": (self.oc_sebab if self.oc_sebab else
                            f"{self.n_overcurrent} kejadian di log kernel"
                            if self.n_overcurrent else "tidak ada"),
            "overcurrent_warna": ("var(--bad)" if self.n_overcurrent
                                  else "var(--dim)" if self.oc_sebab
                                  else "var(--ok)"),
            "bit": (", ".join(n for b, n in self.ARTI.items()
                              if self.throttled & (1 << b))
                    or "bersih (0x0)"),
            "bit_warna": ("var(--bad)" if self.throttled & 0xF else
                          "var(--warn)" if self.throttled else "var(--ok)"),
            "grafik": [[round(v, 3), round(d, 2)] for v, d in self.riwayat],
        }


def bingkai_kosong(pesan, lebar=640, tinggi=360):
    """Gambar pengganti saat kamera belum ada -- supaya stream tidak mati."""
    img = np.zeros((tinggi, lebar, 3), np.uint8)
    img[:] = (18, 20, 24)
    cv2.putText(img, "KAMERA TIDAK AKTIF", (24, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (90, 90, 255), 2, cv2.LINE_AA)
    baris, kata = [], ""
    for w in pesan.split():
        if len(kata) + len(w) > 52:
            baris.append(kata)
            kata = w
        else:
            kata = (kata + " " + w).strip()
    baris.append(kata)
    for i, b in enumerate(baris[:8]):
        cv2.putText(img, b, (24, 110 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return img


class Teensy:
    """Bicara ke firmware lewat USB CDC, dengan whitelist perintah.

    Kalau port tidak ada, kelas ini tetap hidup dalam mode SIMULASI: semua
    kirim dicatat, tidak ada yang lepas ke kabel. Jadi HUD bisa diuji di
    laptop tanpa robot -- dan saat autostart, HUD tetap menyala walau Teensy
    belum sempat ter-enumerasi; sambungannya dicoba ulang tiap 2 detik.
    """

    def __init__(self, port=None, baud=115200, auto=False):
        self.port_name = port
        self.baud = baud
        self.auto = auto
        self.ser = None
        self.buf = ""
        self.log = deque(maxlen=200)
        self.terakhir = {}
        self.tx_terakhir = ""
        # Pemicu terakhir dari firmware: (aksi, ruas, waktu). pemicu_baru
        # dikonsumsi FSM lalu dimatikan -- jadi satu pemicu memicu satu kali,
        # bukan tiap loop selama barisnya masih yang terakhir dibaca.
        self.pemicu = None
        self.pemicu_baru = False
        # Tombol PCB terakhir yang dipicu, dari baris '#TOMBOL' firmware.
        # Selama OLED belum menyala, HUD adalah satu-satunya tempat operator
        # bisa melihat tombolnya terbaca -- dan membedakan "tombol tidak
        # bekerja" dari "tombol bekerja tapi tidak terdengar".
        self.tombol = ""
        self.t_tombol = 0.0
        # TRIM SERVO, {slot: {"nama","invert","us"}}, diisi baris '#TRIM'
        # jawaban 'Yt'. Kosong berarti BELUM PERNAH DIBACA, bukan "semua nol" --
        # tab trim membedakan keduanya, karena menggambar nol yang tidak pernah
        # dibaca lalu menekan simpan akan menulis nol itu ke EEPROM.
        self.trim = {}
        self.t_trim = 0.0
        # UJUNG SATUNYA dari serah-terima. Firmware mencetak "#LEPAS <ruas>"
        # sesudah sekuens capit selesai, artinya: "aku mau melangkah lagi --
        # kamu masih memegang kaki?". Pi menjawab 'm9'. Tanpa jalur ini Teensy
        # akan menunggu sampai batas waktunya habis tiap korban.
        self.lepas = None
        self.lepas_baru = False
        # Kompas arena: 4 arah, None = belum dicatat. Diisi dari keluaran 'k'.
        # Keadaan aliran cetak firmware, DIBACA dari kalimatnya sendiri --
        # bukan ditebak dari perintah yang kita kirim. Toggle buta berbahaya:
        # mengirim 'y' untuk mematikan aliran yang ternyata sudah mati justru
        # MENYALAKANNYA.
        self.aliran = {"y": False, "L": False}
        # None = belum diketahui (firmware belum ditanya atau tidak punya m8).
        self.tunggu_visi = None
        # BUKTI PARKIR VISION, dan kenapa ia perlu ada sendiri.
        #
        # Firmware yang dipatch mencetak "=== PARKIR UNTUK VISION ===" PERSIS
        # sesudah "#KORBAN AMBIL <ruas>", di serial yang sama. Itu bukti yang
        # datang SEKETIKA. Sebelum ini HUD hanya melihat baris 'state' hasil
        # poll 'm' 1x/detik -- yaitu keadaan sampai satu detik yang LALU, yang
        # pada detik pemicu masih berbunyi "BERJALAN". Akibatnya HUD selalu
        # menyimpulkan "firmware tidak parkir", turun ke mode lihat, dan
        # firmware menunggu jawaban yang tidak akan pernah datang sampai batas
        # waktunya habis. Gejalanya identik dengan firmware yang belum dipatch,
        # jadi ia gampang disalah-baca sebagai "patch-nya tidak jalan".
        self.parkir_visi = False
        # Dinaikkan tiap satu baris 'state' dari 'm' selesai diparse. Dipakai
        # untuk membedakan status yang datang SESUDAH pemicu dari status basi
        # yang kebetulan masih tersimpan.
        self.n_status = 0
        # Riwayat perintah operator + jawabannya. Terpisah dari log utama
        # supaya tidak bisa tenggelam oleh aliran rutin.
        self.riwayat = deque(maxlen=120)
        self._tunggu_jawab = None
        self.kompas = [None, None, None, None]
        self.kompas_waktu = 0.0
        self._di_kompas = False
        self._kompas_diminta = False
        self.sebab = "belum dicoba"
        # Berapa byte yang benar-benar KEMBALI dari Teensy. Ini penunjuk yang
        # selama ini hilang: '[TX] m' di log cuma membuktikan kita MENGIRIM.
        # Kalau rx tetap 0 sementara tx jalan, berarti ada yang memakan
        # balasannya -- ModemManager/brltty klasik -- atau firmware diam.
        self.n_rx = 0
        self.diam_sampai = 0.0
        self.n_diam = 0
        self.log_ringkas = True
        self.n_tx = 0
        self.t_rx = 0.0
        self._t_coba = 0.0
        self._t_sebab = 0.0
        # SENGAJA tidak menyambung di sini. serial.Serial() bisa MENGGANTUNG
        # lama pada ttyACM yang sedang putus-nyambung (tegangan turun), dan
        # kalau itu terjadi di loop utama seluruh HUD ikut beku: halaman
        # tersaji tapi kosong, tanpa satu pun petunjuk kenapa. Penyambungan
        # dikerjakan thread tersendiri -- lihat penyambung() di main().
        if not (port or auto):
            self._catat("[LINK] tanpa --port -- mode SIMULASI")

    def diam_sejenak(self, detik=0.8):
        """Tandai: jawaban yang datang sebentar lagi itu POLL RUTIN, jangan dicatat.

        Datanya tetap diparse -- yang disembunyikan cuma barisnya di log.
        Perintah manual apa pun langsung membatalkan penahanan ini, jadi
        keluaran yang KAMU minta tidak pernah ikut hilang.
        """
        self.diam_sampai = time.time() + detik

    def _catat(self, pesan):
        """Peristiwa sambungan: masuk log HUD DAN stdout.

        Yang di deque hanya terlihat di halaman web. Saat HUD jalan sebagai
        layanan systemd, satu-satunya jendela yang tersisa adalah journal --
        jadi kejadian sambung/putus harus ikut ke sana, bukan cuma ke layar
        yang mungkin tidak sedang dibuka siapa pun.
        """
        self.log.append(pesan)
        print(pesan, flush=True)

    # -- sambungan ----------------------------------------------------
    def _sebab(self, teks):
        """Simpan sebab gagal, dan catat ke log paling sering 30 detik sekali.

        Tanpa pembatas ini log penuh oleh baris yang sama tiap 2 detik dan
        pesan penting lain ikut terdorong keluar.
        """
        self.sebab = teks
        if teks and time.time() - self._t_sebab > 30.0:
            self._t_sebab = time.time()
            self._catat(f"[LINK] {teks}")

    def sambung(self):
        """Coba buka port. Aman dipanggil berulang; menahan diri 2 detik."""
        if self.hidup or time.time() - self._t_coba < 2.0:
            return
        self._t_coba = time.time()

        if serial is None:
            # JANGAN mematikan auto selamanya. Dulu begitu, jadi satu pesan
            # yang terlanjur tergulung keluar dari log berarti tidak ada lagi
            # petunjuk apa pun sepanjang sisa sesi.
            self._sebab("pyserial tidak ada di venv -- "
                        ".venv/bin/pip install pyserial")
            return

        if self.port_name:
            port, catatan = self.port_name, f"diminta lewat --port {self.port_name}"
        elif self.auto:
            port, catatan = cari_port(rinci=True)
        else:
            port, catatan = None, "auto mati dan --port kosong"

        if not port:
            self._sebab(catatan)
            return
        try:
            # write_timeout WAJIB. Tanpa itu ser.write() memblokir SELAMANYA
            # saat penyangga keluaran port penuh -- Teensy tercabut di tengah
            # tulis, atau bus USB macet sesudah tegangan turun. Penulisan itu
            # terjadi di loop utama, jadi port yang macet membekukan seluruh
            # HUD, bukan cuma satu perintah.
            #
            # 0,5 detik: satu baris perintah paling panjang di firmware ini
            # belasan byte pada 115200 baud, yaitu di bawah satu milidetik.
            # Apa pun yang lebih lama dari setengah detik berarti port-nya
            # memang sudah tidak menerima, bukan sedang sibuk.
            self.ser = serial.Serial(port, self.baud, timeout=0, write_timeout=0.5)
            self.sebab = ""
            self._catat(f"[LINK] tersambung {port} @ {self.baud}")
            # Minta tabel kompas sekali. 'k' cuma MEMBACA -- tidak butuh IMU,
            # tidak menggerakkan apa pun, dan boleh di mode BACA. Tanpa ini
            # kartu kompas kosong sampai ada yang menekan tombolnya, padahal
            # angkanya sudah ada di RAM firmware sejak boot (kompasMuat()).
            self._kompas_diminta = False
        except PermissionError as e:
            self.ser = None
            self._sebab(f"{port}: izin ditolak ({e}) -- user belum di grup dialout?")
        except Exception as e:                           # noqa: BLE001
            self.ser = None
            self._sebab(f"{port}: {e}")

    @property
    def hidup(self):
        return self.ser is not None and self.ser.is_open

    @property
    def nama_port(self):
        return self.ser.port if self.hidup else "SIMULASI"

    # -- kirim / terima ----------------------------------------------
    def kirim(self, cmd, armed=True, paksa=False, rutin=False):
        """Satu-satunya pintu keluar ke Teensy. Tidak menolak apa pun.

        Sejak 13 Sep 2026 tidak ada whitelist dan tidak ada daftar terlarang
        (bagian 4). Fungsi ini masih satu-satunya pintu keluar -- itu yang
        membuat penghitung tx, log, riwayat perintah dan penahan poll tetap
        melihat SELURUH lalu lintas -- ia cuma tidak lagi menyaringnya.
        """
        # Tidak ada gerbang lagi -- lihat catatan di bagian 4. Parameter
        # `armed` dan `paksa` masih diterima supaya seluruh pemanggil lama
        # (dan tesnya) tetap jalan, tapi keduanya sudah tidak berpengaruh.
        self.tx_terakhir = cmd
        self.n_tx += 1
        if rutin:
            # Poll otomatis: jangan catat '[TX] m' / '[TX] l', dan tahan
            # jawabannya. Dua perintah per detik yang masing-masing
            # menumpahkan belasan baris = pesan penting tenggelam dalam
            # hitungan detik.
            self.diam_sejenak()
        else:
            # Perintah yang KAMU minta: buka penahanan supaya jawabannya
            # pasti terlihat, walau kebetulan datang di tengah jendela poll.
            self.diam_sampai = 0.0
            self.log.append(f"[TX] {cmd}")
            # RIWAYAT PERINTAH -- daftar terpisah yang tidak bisa tenggelam.
            #
            # Log utama memuat 200 baris. Satu aliran yaw pada 100 ms
            # menghabiskan seluruhnya dalam 20 detik, jadi perintah yang
            # diketik Vincent tiga menit lalu -- dan jawabannya -- sudah
            # tidak ada lagi saat ia ingin membandingkan hasil percobaan.
            # Itu bukan log yang penuh, itu riwayat yang hilang.
            self.riwayat.append({"t": time.time(), "cmd": cmd, "jawab": []})
            # Baris yang datang sesudah ini dianggap jawabannya, sampai
            # jendelanya habis atau perintah berikutnya dikirim.
            self._tunggu_jawab = (self.riwayat[-1], time.time() + 2.5)
        if self.hidup:
            try:
                self.ser.write((cmd + "\n").encode())
            except Exception as e:                       # noqa: BLE001
                # Termasuk SerialTimeoutException dari write_timeout. Port yang
                # tidak menerima dalam setengah detik dianggap putus dan
                # disambung ulang, bukan ditunggu.
                self.log.append(f"[TX ERROR] {e}")
                self._putus()
                return False
        return True

    def _putus(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:                                # noqa: BLE001
            pass
        self.ser = None
        self._catat("[LINK] terputus -- akan dicoba sambung ulang")

    def baca(self):
        """Tarik byte yang datang, pecah per baris, parse yang dikenali."""
        if not self.hidup:
            return          # penyambungan diurus thread, bukan di sini
        try:
            data = self.ser.read(4096).decode("utf-8", "replace")
        except Exception as e:                           # noqa: BLE001
            self.log.append(f"[RX ERROR] {e}")
            self._putus()
            return
        if not data:
            return
        self.n_rx += len(data)
        self.t_rx = time.time()
        self.buf += data
        while "\n" in self.buf:
            baris, self.buf = self.buf.split("\n", 1)
            baris = baris.rstrip("\r")
            if baris:
                # Diparse SELALU. Yang ditahan cuma barisnya di log -- data
                # jarak dan state tetap masuk, jadi HUD tidak kehilangan apa pun.
                self._parse(baris)
                self._catat_jawab(baris)
                # Baris berawalan '#' adalah firmware yang berbicara DULUAN:
                # '#KORBAN', '#LEPAS', '#TOMBOL'. Ia tidak pernah jawaban atas
                # poll, jadi jendela diam tidak boleh menelannya. Tanpa
                # pengecualian ini baris tombol hilang ~80% -- poll sekali
                # sedetik menahan log 0,8 detik tiap kali, dan tombol ditekan
                # tanpa peduli jendela siapa yang sedang terbuka.
                if (self.log_ringkas and time.time() < self.diam_sampai
                        and not baris.startswith("#")):
                    self.n_diam += 1
                else:
                    self.log.append(baris)

    # Format yang di-parse mengikuti Mission::status() dan tabel 'l'.
    RE_KV = re.compile(r"^\s{2,}([a-zA-Z ]+?)\s*:\s*(.+)$")
    RE_LIDAR = re.compile(r"^\s*([0-5])\s+([A-Z\-]+)\s*:\s*(.+?)(?:\s{2,}\[|$)")
    RE_CM = re.compile(r"(-?\d+(?:[.,]\d+)?)\s*cm")

    # PEMICU VISION dari firmware v1.12. Teensy mencetak satu baris ini
    # begitu ia berhenti di depan korban:
    #     #KORBAN AMBIL 1
    #     #KORBAN TARUH 6
    # Ini yang pertama kalinya firmware berbicara DULUAN ke Pi. Sebelumnya
    # Pi cuma bisa menebak dari nama ruas -- yang berarti menebak dari teks
    # yang formatnya bisa berubah kapan saja.
    RE_PEMICU = re.compile(r"^#KORBAN\s+(AMBIL|TARUH)\s+(\d+)", re.I)

    # UJUNG BALIK. Dicetak firmware sesudah sekuens capit selesai:
    #     #LEPAS 1
    # Artinya Teensy sudah selesai dengan lengannya dan mau melangkah lagi,
    # tapi ia TIDAK akan melangkah sebelum Pi bilang kakinya sudah dilepas.
    RE_LEPAS = re.compile(r"^#LEPAS\s+(\d+)", re.I)

    # TOMBOL PCB. Dicetak Tampilan::lapor() tiap tombol dipicu:
    #     #TOMBOL D2 TEKAN siapkan: I lalu R lalu b
    #     #TOMBOL D5 TAHAN reset ruas & poin ke 0
    # Pin, jenis picuan, lalu fungsinya dalam kata-kata. Fungsi ikut dikirim
    # firmware, bukan dipetakan lagi di sini: peta kedua di Pi akan diam-diam
    # berbohong begitu tombolnya dipindah di Teensy.
    RE_TOMBOL = re.compile(r"^#TOMBOL\s+(D\d)\s+(TEKAN|TAHAN)\s+(.*)$", re.I)

    # TRIM SERVO. Dicetak Hexapod::cetakTrim() sebagai jawaban 'Yt':
    #     #TRIM 0 K0_COXA 0 -40
    #     slot nama invert us
    #
    # Nomor slot yang berarti, bukan namanya: nama ikut dikirim firmware supaya
    # HUD tidak perlu menyimpan peta kedua yang diam-diam berbohong begitu
    # urutan slot di EEMap.h berubah. Alasan yang sama dengan #TOMBOL.
    RE_TRIM = re.compile(
        r"^#TRIM\s+(\d+)\s+(\S+)\s+([01])\s+([+-]?\d+)\s*$", re.I)

    def _catat_jawab(self, baris):
        """Tempelkan baris ini ke perintah terakhir, kalau ia jawabannya.

        Jendela 2,5 detik, dan baris aliran rutin DIBUANG. Tanpa pembuangan
        itu riwayat ikut penuh oleh hal yang sama yang menenggelamkan log --
        dan perbaikannya jadi sia-sia.
        """
        if not getattr(self, "_tunggu_jawab", None):
            return
        entri, batas = self._tunggu_jawab
        if time.time() > batas:
            self._tunggu_jawab = None
            return
        t = baris.strip()
        if not t or t.startswith(("[TX]", "[RX")):
            return
        # Aliran rutin bukan jawaban atas apa pun.
        if self.aliran.get("y") and t.lower().startswith("yaw"):
            return
        if len(entri["jawab"]) < 12:
            entri["jawab"].append(t)

    ARAH = ("UTARA", "TIMUR", "SELATAN", "BARAT")
    RE_KOMPAS = re.compile(
        r"^\s*([0-3])\s+(UTARA|TIMUR|SELATAN|BARAT)\s*:\s*(.+)$", re.I)

    def _parse(self, baris):
        m = self.RE_PEMICU.match(baris.strip())
        if m:
            self.pemicu = (m.group(1).upper(), int(m.group(2)), time.time())
            self.pemicu_baru = True
            return

        m = self.RE_TOMBOL.match(baris.strip())
        if m:
            self.tombol = f"{m.group(1)} {m.group(2).upper()} · {m.group(3).strip()}"
            self.t_tombol = time.time()
            return

        m = self.RE_TRIM.match(baris.strip())
        if m:
            # DITIMPA PER SLOT, bukan tabel diganti utuh. 'Yt' mencetak 24
            # baris berurutan, dan baris yang hilang di tengah (buffer serial
            # penuh) tidak boleh menghapus slot yang sudah benar.
            self.trim[int(m.group(1))] = {
                "nama":   m.group(2),
                "invert": int(m.group(3)),
                "us":     int(m.group(4)),
            }
            self.t_trim = time.time()
            return

        m = self.RE_LEPAS.match(baris.strip())
        if m:
            self.lepas = (int(m.group(1)), time.time())
            self.lepas_baru = True
            # Parkir vision ruas ini SUDAH selesai. Membiarkannya menyala
            # membuat pemicu korban BERIKUTNYA melihat bukti parkir yang basi
            # lalu mengambil alih sebelum firmware benar-benar parkir lagi.
            self.parkir_visi = False
            return

        # KEADAAN ALIRAN CETAK. Firmware mengumumkannya sendiri:
        #   "Aliran yaw HIDUP. Ketik 'y' lagi untuk berhenti."
        #   "Aliran yaw berhenti."
        # Dibaca dari situ, bukan ditebak dari perintah yang kita kirim --
        # karena Vincent (atau siapa pun) bisa mengetiknya langsung lewat
        # serial, dan tebakan kita akan langsung salah tanpa ada yang tahu.
        _u = baris.upper()
        # PENGUMUMAN PARKIR. Datang seketika, satu-dua baris sesudah pemicu --
        # bukan lewat poll. Ini yang menutup lomba antara pemicu dan status.
        if "PARKIR UNTUK VISION" in _u:
            self.parkir_visi = True
        # Keadaan 'tunggu vision' (m8) -- dari kalimat firmware, bukan tebakan.
        if ("TUNGGU VISION SEBELUM AMBIL" in _u
                or ("TUNGGU VISI" in _u and ":" in baris)):
            self.tunggu_visi = "NYALA" in _u
        if "ALIRAN YAW" in _u:
            self.aliran["y"] = "HIDUP" in _u
        elif "ALIRAN LIDAR" in _u:
            self.aliran["L"] = "HIDUP" in _u

        # TABEL KOMPAS -- HARUS diperiksa SEBELUM RE_LIDAR.
        #
        # Ini bug yang sudah lama diam. Baris kompas berbentuk
        #     "0 UTARA\t: 123.4 der"
        # dan RE_LIDAR (`^\s*([0-5])\s+([A-Z\-]+)\s*:...`) MENERIMANYA:
        # ch=0, nama='UTARA'. Jadi tiap kali 'k' ditekan, keempat barisnya
        # masuk ke lidar_ch0..3 -- dan yang lebih merugikan, angkanya tidak
        # pernah sampai ke mana pun yang menampilkannya. Dari luar gejalanya
        # "kompasnya kosong padahal EEPROM ada isinya".
        #
        # Dipagari header supaya tidak menebak dari bentuk baris saja: tabel
        # apa pun yang kebetulan berbentuk "<angka> <NAMA> : <isi>" tidak
        # akan salah dibaca sebagai kompas.
        if "KOMPAS ARENA" in baris.upper():
            self._di_kompas = True
            return
        if getattr(self, "_di_kompas", False):
            m = self.RE_KOMPAS.match(baris)
            if m:
                i = int(m.group(1))
                isi = m.group(3).strip()
                mm = self.RE_CM.search(isi.replace("der", "cm"))
                self.kompas[i] = float(mm.group(1).replace(",", ".")) if mm else None
                self.kompas_waktu = time.time()
                return
            # Baris pertama yang bukan anggota tabel menutup tabelnya.
            self._di_kompas = False

        m = self.RE_LIDAR.match(baris)
        if m:
            ch, nama, isi = int(m.group(1)), m.group(2), m.group(3).strip()
            self.terakhir[f"lidar_{nama}"] = self._cm(isi)
            self.terakhir[f"lidar_ch{ch}"] = self._cm(isi)
            return
        m = self.RE_KV.match(baris)
        if m:
            kunci = m.group(1).strip().lower()
            nilai = m.group(2).strip()
            self.terakhir[kunci] = nilai
            if kunci in ("depan", "belakang"):
                self.terakhir[kunci + "_cm"] = self._cm(nilai)
            if kunci == "state":
                # Status SEGAR. Ia yang berhak menimpa keadaan parkir --
                # termasuk MEMATIKANNYA begitu firmware sudah tidak di
                # KONFIRM lagi, supaya bukti tidak menempel sesudah lewat.
                #
                # getattr, bukan akses langsung: Teensy juga dibangun lewat
                # jalur yang melewati __init__ (link_uji di berkas tes), dan
                # parser tidak boleh mati karena medan yang belum ada.
                self.n_status = getattr(self, "n_status", 0) + 1
                self.parkir_visi = "KONFIRMASI" in nilai.upper()

    @staticmethod
    def _cm(teks):
        """Tiga keadaan LiDAR firmware -> None (MATI), inf (jauh), atau cm.

        Urutannya penting dan bukan selera. Firmware mencetak kalimat yang
        MENGANDUNG kata lain di ekornya:
            "38 cm dari dinding START  (sampel jauh: 1 dari 3)" -> 38, bukan jauh
            "jauh -- dinding START di luar 70 cm atau hantu"    -> jauh, bukan 70
        Jadi yang menentukan kata PERTAMA, bukan kata yang ada di mana pun.
        """
        t = teks.strip().lower()
        if t.startswith("mati"):
            return None
        if t.startswith("jauh"):
            return float("inf")
        m = Teensy.RE_CM.search(teks)
        return float(m.group(1).replace(",", ".")) if m else None

    def depan_cm(self):
        return self.terakhir.get("lidar_DEPAN", self.terakhir.get("depan_cm"))

    def ruas_fw(self):
        """Baris 'ruas' dari status misi firmware v1.9+.

        Bentuknya: "12 dari 0..33  --  K-1 angkat korban". Namanya itu yang
        dipakai HUD untuk tahu kapan vision perlu menyala -- firmware belum
        punya pemicu khusus untuk itu, tapi dia SUDAH menyebutkan sedang di
        ruas apa, dan itu sudah cukup.
        """
        return self.terakhir.get("ruas", "")

    def state_teensy(self):
        return self.terakhir.get("state", "-")

    def membawa(self):
        """(depan, belakang) -- True kalau capit itu SEDANG memegang korban.

        Dibaca dari baris status firmware v1.12 (Misi.cpp:1710):

            "  membawa     : depan KORBAN, belakang kosong"

        RE_KV sudah menyimpannya sebagai kunci 'membawa', jadi tidak perlu
        parser baru -- cuma perlu dibaca. Yang penting DI SINI: keadaan ini
        datang dari FIRMWARE, bukan dari catatan HUD sendiri. HUD bisa
        di-restart, halaman bisa dimuat ulang, dan misi bisa dijalankan dari
        Teensy tanpa HUD tahu apa-apa; firmware yang selalu tahu isi
        capitnya.
        """
        t = self.terakhir.get("membawa", "")
        if not t:
            return (False, False)
        bagi = t.upper().split("BELAKANG", 1)
        depan = "KORBAN" in bagi[0]
        belakang = "KORBAN" in bagi[1] if len(bagi) > 1 else False
        return (depan, belakang)

    def sebab_teensy(self):
        return self.terakhir.get("sebab", "")


# =====================================================================
# 5. JURI VISION -- satu kandidat, satu keputusan
# =====================================================================
class Juri:
    """Mengumpulkan N frame di satu pose, lalu memutuskan sekali.

    Gerbangnya tiga lapis, semuanya geometris, bukan selera:
      1. ROI tengah  -- di 20 cm, boneka slot sebelah (8 cm) muncul di 21,8
                        derajat off-axis, JELAS di dalam frame. Tanpa gerbang
                        ini kamera menilai tetangga, bukan sasaran.
      2. Tinggi bbox -- jarak dikunci 20 cm, jadi tinggi bbox jatuh di pita
                        sempit. Ini yang membuang boneka ruang sebelah yang
                        terlihat melewati dinding setinggi 10 cm.
      3. k-of-N      -- satu frame tidak pernah cukup.
    """

    def __init__(self, kalib: Kalib, names):
        self.k = kalib
        self.names = names
        self.reset()

    def reset(self):
        self.sampel = []
        self.ditolak = {"roi": 0, "tinggi": 0, "conf": 0}
        self.alasan = {}          # (x1,y1) -> kenapa deteksi ini dibuang

    def saring(self, dets, w, h, pakai_roi=True, h_min=None, h_max=None):
        """Buang deteksi yang tidak mungkin jadi sasaran di pose ini.

        pakai_roi=False dipakai mode JEJAK: di situ sasaran memang SEDANG
        di pinggir -- justru itu yang mau dikejar. Gerbang tinggi bbox tetap
        dipakai, karena dia yang membuang boneka ruang sebelah.

        DUA KORBAN SATU FRAME TIDAK DIURUS DI SINI. Membuang salah satunya di
        saring() berarti membuangnya dari penglihatan; yang benar memilih di
        antara keduanya, dan itu pekerjaan pilih_sasaran(). Lihat lock_kanan.
        """
        cx0 = w / 2 + self.k.cx_offset_px
        batas = self.k.roi_half_px(w)
        lo = self.k.bbox_h_min if h_min is None else h_min
        hi = self.k.bbox_h_max if h_max is None else h_max
        self.alasan = {}
        lolos = []
        for x1, y1, x2, y2, score, cls in dets:
            cx = (x1 + x2) / 2
            if pakai_roi and abs(cx - cx0) > batas:
                self.ditolak["roi"] += 1
                self.alasan[(x1, y1)] = "di luar ROI"
                continue
            if self.k.roi_bawah_frac > 0:
                cy = (y1 + y2) / 2
                if cy > h * (1.0 - self.k.roi_bawah_frac):
                    self.ditolak["roi"] += 1
                    self.alasan[(x1, y1)] = "di pita yang ditutupi capit"
                    continue
            if self.k.roi_atas_frac > 0:
                cy = (y1 + y2) / 2
                if cy < h * self.k.roi_atas_frac:
                    self.ditolak["roi"] += 1
                    self.alasan[(x1, y1)] = "di pita atas -- korban yang digendong"
                    continue
            frak = (y2 - y1) / h
            if not (lo <= frak <= hi):
                self.ditolak["tinggi"] += 1
                self.alasan[(x1, y1)] = (f"terlalu jauh {frak:.2f}<{lo:.2f}"
                                         if frak < lo else
                                         f"terlalu dekat {frak:.2f}>{hi:.2f}")
                continue
            lolos.append((x1, y1, x2, y2, score, cls, cx))
        return lolos

    def tambah(self, lolos):
        """Satu frame -> satu suara (kandidat terbesar di ROI)."""
        if not lolos:
            self.sampel.append((None, 0.0, 0.0))
            return
        best = max(lolos, key=lambda d: (d[3] - d[1]) * d[4])
        nama = self.names.get(best[5], str(best[5]))
        self.sampel.append((nama, best[4], best[6]))

    @property
    def cukup(self):
        return len(self.sampel) >= self.k.n_frame

    def putuskan(self):
        """-> (kelas, margin, conf_rerata, bearing_px, alasan)"""
        n_korban = [s for s in self.sampel if s[0] == self.k.kelas_korban]
        n_dummy = [s for s in self.sampel if s[0] == self.k.kelas_dummy]
        p_k = sum(s[1] for s in n_korban) / max(1, len(self.sampel))
        p_d = sum(s[1] for s in n_dummy) / max(1, len(self.sampel))
        margin = p_k - p_d
        cx = np.mean([s[2] for s in self.sampel if s[0]]) if (n_korban or n_dummy) else 0.0

        if len(n_korban) >= self.k.k_of_n:
            conf = sum(s[1] for s in n_korban) / len(n_korban)
            if conf >= self.k.conf_min:
                return self.k.kelas_korban, margin, conf, cx, f"{len(n_korban)}/{len(self.sampel)} frame"
            return "RAGU", margin, conf, cx, f"conf {conf:.2f} < {self.k.conf_min}"
        if len(n_dummy) >= self.k.k_of_n:
            conf = sum(s[1] for s in n_dummy) / len(n_dummy)
            return self.k.kelas_dummy, margin, conf, cx, f"{len(n_dummy)}/{len(self.sampel)} frame"
        return "RAGU", margin, max(p_k, p_d), cx, "tidak ada yang mencapai k-of-N"


# =====================================================================
# 6. STATE MISI (pembukuan Pi)
# =====================================================================
class Misi:
    def __init__(self):
        self.idx = 0
        self.status = [BELUM] * len(MISI)
        self.status[0] = JALAN
        self.state = S_IDLE
        # Penanda "sudah dilaporkan" rantai AMBIL. Dinolkan tiap korban baru
        # di ganti(), supaya laporan korban sebelumnya tidak membungkam
        # laporan korban berikutnya.
        self._jarak_ditinjau = False
        self._dekat_diberitahu = False
        self.t_state = time.time()
        self.sebab = ""
        self.slot = 0
        self.hasil_slot = {}
        self.terkunci = None
        self.bearing_deg = None
        # KAPAN FRAME-nya DIAMBIL, bukan kapan hasilnya selesai dihitung.
        #
        # UkurDiam membuang bacaan yang "lahir sebelum badan diam", dan
        # penilaian itu cuma benar kalau yang dibandingkan waktu PENGAMBILAN.
        # Dengan time.time() saat hasilnya dipakai, frame yang terekam selagi
        # badan MASIH BERPUTAR tetap lolos asalkan inferensinya baru selesai
        # sesudah badan berhenti -- lalu koreksi berikutnya dihitung dari pose
        # yang sudah tidak ada, dan robot melewati tengah. Itu "gerak over
        # sebelum benarkan posisi".
        self.bearing_t = None
        # --- mode uji (bench test) ---
        # slot_override : batasi jumlah slot saat menguji, tanpa mengubah tabel MISI
        # tanpa_gerak   : nilai vision saja, TIDAK satu pun perintah gerak dikirim
        self.slot_override = None
        self.tanpa_gerak = False
        self.jejak_kelas = ""       # kelas sasaran yang sedang diikuti
        self.jejak_conf = 0.0
        self.n_mentah = 0           # deteksi sebelum disaring
        self.h_terbesar = 0.0       # tinggi bbox terbesar / tinggi frame
        self.riwayat = deque(maxlen=12)
        # Penanda "urutan perintah untuk state ini sudah dijadwalkan sekali".
        # Dipakai rantai AMBIL supaya tidak menumpuk antrean tiap frame.
        # Selalu dibersihkan di ganti(), jadi tiap masuk state baru mulai bersih.
        self._urut = None
        self._geser_dijadwal = False
        # --- dua kunci pengaman, sengaja TERPISAH ---
        # halt : STOP KERAS. Semua otomasi mati dan TIDAK bisa hidup sendiri.
        #        Harus dilepas operator. Ini yang dulu tidak ada: 'stop' cuma
        #        mengosongkan antrean, lalu frame berikutnya FSM melihat
        #        antrean kosong dan menjadwalkan perintah baru -- jadi robot
        #        berhenti sekejap lalu lanjut sendiri, persis seperti tidak
        #        ditekan apa-apa.
        # jeda : PAUSE. Membekukan di tempat TAPI menyimpan state, supaya bisa
        #        dilanjutkan. Bedanya dengan halt cuma itu: niat operator.
        # Pose badan yang SEDANG diperintahkan (bukan yang sebenarnya).
        # Disimpan karena 'r' dan 't' itu perintah ABSOLUT, bukan tambahan --
        # untuk menambah 2 der kita harus mengirim total barunya, bukan '2'.
        self._lengan_diberitahu = False
        self.n_dummy_saja = 0
        self._dummy_diberitahu = False
        # Jarak dari kamera. Selalu berpasangan dengan SEBABNYA kalau
        # kosong -- 'tidak ada angka' dan 'ada angka tapi tak boleh
        # dipakai' itu dua keadaan yang sangat berbeda di sini.
        self.jarak_vis = None
        self.jarak_vis_sebab = "belum ada sasaran"
        self.vis_bbox = None       # (y1, y2, h_frame, w_frame, x1, x2)
        self.vis_rasio = 0.0
        self.vis_posisi = ""
        # True kalau rantai AMBIL yang sedang jalan DIMULAI oleh pemicu
        # firmware. Menentukan siapa yang mengambil korbannya di ujung:
        # sekuens firmware (serah terima balik lewat 'm2'), atau rantai
        # capit HUD sendiri (uji manual).
        self.dari_firmware = False
        # Pemicu #KORBAN yang sudah dibaca tapi BELUM diputuskan, sambil
        # menunggu bukti parkir yang segar: (ruas, saat, n_status saat itu).
        self.pemicu_tunda = None
        # Parkir firmware pernah benar-benar TERLIHAT selama serah-terima ini.
        # Dipakai untuk mengenali TEPI turunnya -- lihat langkah_fsm().
        self.parkir_terlihat = False
        self._bawa_diberitahu = False
        self._silang_diberitahu = False
        # Dwell penengahan: kapan syarat tengah mulai terpenuhi (None = belum).
        self.t_tengah = None
        self.tahan_s = 0.0
        # Kapan '#LEPAS' datang -- yaitu kapan capit selesai terangkat.
        # None selagi sekuens capit firmware masih berjalan. Dari sinilah
        # lepas_tahan_s dihitung.
        self.t_lepas = None
        # Sebab terakhir kenapa TAHAN_TENGAH tidak mengoreksi. Disimpan
        # supaya log mencatatnya sekali per sebab, bukan tiap frame.
        self._tahan_sebab = ""
        # Deteksi 'O' yang tidak bekerja: galat saat 'O' terakhir dikirim,
        # dan berapa kali berturut-turut galatnya tidak berubah sesudahnya.
        self.o_err_lalu = None
        self.o_diam = 0
        self.pivot_gait_mati = False
        self.beda_vis_lidar = None
        self.tembak_err = None
        self.tembak_yaw = None
        self.awas_kelas = ""
        self.awas_conf = 0.0
        self.badan_yaw = 0.0
        self.badan_x = 0.0
        # Jatah langkah 'H' dan penanda cadangan 'V', dipakai geser_tengah().
        self.geser_n = 0
        self.geser_v_dipakai = False
        self.halt = False
        self.jeda = False
        self.sebab_henti = ""

    @property
    def kini(self):
        return MISI[self.idx]

    @property
    def n_slot(self):
        """Jumlah slot yang dipakai FSM: hasil uji kalau sedang menguji,
        kalau tidak ya jumlah sebenarnya dari tabel MISI."""
        return self.slot_override if self.slot_override else self.kini[3]

    @property
    def berikut(self):
        return MISI[self.idx + 1] if self.idx + 1 < len(MISI) else None

    def maju_misi(self, tandai=SELESAI):
        self.status[self.idx] = tandai
        self.riwayat.append(f"{self.kini[0]} -> {tandai}")
        if self.idx + 1 < len(MISI):
            self.idx += 1
            self.status[self.idx] = JALAN
        self.slot = 0
        self.hasil_slot = {}
        self.terkunci = None
        self.sebab = ""
        self.ganti(S_IDLE)

    def ganti(self, s, sebab=""):
        if s != self.state:
            self.riwayat.append(f"{self.state} -> {s}")
            self._urut = None
        # Rantai AMBIL dimulai ulang: penanda laporan dikosongkan.
        if s == S_A_TENGAH:
            self._jarak_ditinjau = False
            self._dekat_diberitahu = False
        self.state = s
        self.t_state = time.time()
        if sebab:
            self.sebab = sebab

    @property
    def lewat(self):
        return time.time() - self.t_state

    @property
    def state_berikut(self):
        if self.state == S_LIHAT:
            return S_SLOT if self.slot + 1 < self.n_slot else S_PUTUS
        return FSM[self.state][1]

    def gagal(self, sebab):
        self.sebab = sebab
        self.ganti(S_GAGAL)


# =====================================================================
# 7. AKSI -- satu-satunya tempat perintah gerak lahir
# =====================================================================
class Aksi:
    """Antrean perintah bertenggat. Satu perintah per giliran, tidak memblokir."""

    def __init__(self, link: Teensy):
        self.link = link
        self.antre = []
        self.t_boleh = 0.0

    def jadwal(self, *pasangan):
        """jadwal(('o3', 3.0), ('w', 0.2)) -- (perintah, tunggu detik sesudahnya)"""
        self.antre.extend(pasangan)

    def kosong(self):
        return not self.antre

    def batal(self):
        self.antre.clear()

    def putar(self, armed, boleh=True):
        # 'boleh' dimatikan saat STOP/PAUSE. Mengosongkan antrean saja TIDAK
        # cukup -- perintah yang sudah terlanjur diambil tetap harus tertahan.
        if not boleh or not self.antre or time.time() < self.t_boleh:
            return
        cmd, tunggu = self.antre.pop(0)
        self.link.kirim(cmd, armed)
        self.t_boleh = time.time() + tunggu


class UkurDiam:
    """Kumpulkan bearing HANYA saat badan diam, lalu rata-ratakan.

    Kenapa bukan median/rata-rata biasa atas semua frame: karena masalahnya
    BUKAN derau. Frame yang diambil sebelum gerakan terakhir selesai
    menggambarkan pose yang sudah tidak ada lagi -- merata-ratakannya dengan
    frame yang benar justru MENCEMARI hasilnya. Yang benar membuangnya.

    'boleh' baru True sesudah cukup banyak bacaan sah terkumpul.
    """

    def __init__(self, kalib):
        self.k = kalib
        self.reset()

    def reset(self):
        self.rata = None
        self.n = 0
        self.t_boleh_ukur = 0.0      # sebelum ini, bacaan dianggap basi

    def tunda(self, detik):
        """Badan baru saja diperintah bergerak: buang semua bacaan sampai diam."""
        self.t_boleh_ukur = time.time() + detik
        self.rata = None
        self.n = 0

    def tambah(self, err, t_sampel=None):
        """Masukkan satu bacaan. Mengembalikan True kalau diterima."""
        t = time.time() if t_sampel is None else t_sampel
        if t < self.t_boleh_ukur:
            return False                       # lahir sebelum badan diam
        a = min(max(self.k.ema_alpha, 0.01), 1.0)
        self.rata = err if self.rata is None else self.rata * (1 - a) + err * a
        self.n += 1
        return True

    @property
    def boleh(self):
        return self.rata is not None and self.n >= max(1, self.k.diam_sampel_min)


class Pid:
    """PID untuk penengahan halus lewat putar-badan.

    Tiga hal yang membuatnya beda dari PID buku, dan ketiganya karena plant-nya
    lup POSISI dengan waktu mati, bukan lup kecepatan:

    1. Keluarannya PERUBAHAN setpoint, bukan setpoint. Badan sudah menahan
       posisinya sendiri; yang kita atur cuma seberapa jauh menggesernya.
    2. Perubahan itu DIBATASI (maks_langkah_deg). Ini rem yang sebenarnya --
       laju ramp 60 der/detik milik firmware tidak bisa diubah dari sini, tapi
       kita bisa tidak pernah memintanya melompat jauh.
    3. Integral hanya menumpuk DI DEKAT sasaran. Di luar itu dia cuma windup:
       menumpuk selama gerakan besar lalu dilepas sekaligus jadi sentakan --
       persis gejala yang mau kita hilangkan.
    """

    def __init__(self, kalib):
        self.k = kalib
        self.reset()

    def reset(self):
        self.i = 0.0
        self.e_lalu = None
        self.t_lalu = None
        self.sampel = deque(maxlen=max(1, int(self.k.saring_n)))

    def saring(self, err):
        """Median beberapa sampel terakhir. Satu frame nyasar tidak menggeser
        median; rata-rata akan tergeser."""
        self.sampel.append(err)
        s = sorted(self.sampel)
        return s[len(s) // 2]

    def langkah(self, err):
        """Kembalikan (delta_derajat, err_tersaring). delta sudah dibatasi."""
        e = self.saring(err)
        now = time.time()
        dt = (now - self.t_lalu) if self.t_lalu else 0.0
        self.t_lalu = now

        p = self.k.kp * e

        # Integral hanya di dekat sasaran, dan dijepit. Dua-duanya anti-windup.
        pita_i = max(3.0, self.k.maks_langkah_deg * 1.5)
        if abs(e) <= pita_i and dt > 0:
            self.i += e * dt
            batas_i = self.k.maks_langkah_deg / max(self.k.ki, 1e-6)
            self.i = max(-batas_i, min(batas_i, self.i))
        else:
            self.i = 0.0
        integ = self.k.ki * self.i

        # Turunan dari galat yang SUDAH disaring, dan hanya kalau dt masuk akal.
        # dt liar (frame tersendat, HUD baru bangun) membuat turunan meledak.
        deriv = 0.0
        if self.e_lalu is not None and 0.02 < dt < 2.0:
            deriv = self.k.kd * (e - self.e_lalu) / dt
        self.e_lalu = e

        keluar = p + integ + deriv
        batas = self.k.maks_langkah_deg
        return max(-batas, min(batas, keluar)), e


def pivot_der(minta, kalib):
    """Bulatkan permintaan pivot ke DERAJAT BULAT yang pasti dikerjakan firmware.

    Dua jebakan sekaligus, dan keduanya diam:
      1. 'O' dikirim sebagai bilangan bulat. f"{6.5:.0f}" menghasilkan "6" --
         PERSIS di HEADING_TOLERANCE_DEG, jadi firmware menganggap dirinya
         sudah lurus dan membuangnya.
      2. Membulatkan ke bawah selalu berbahaya di sini: hasilnya perintah yang
         terlihat terkirim di log tapi tidak menggerakkan apa pun.
    Jadi besarannya SELALU dibulatkan KE ATAS, menjauh dari nol.
    """
    besar = max(abs(minta), kalib.pivot_min_deg)
    return int(math.copysign(math.ceil(besar), minta))


def putar_manual(der, metode, misi: Misi, aksi: Aksi, kalib: Kalib):
    """Putar sekian derajat atas permintaan operator. Kembalikan (perintah, catatan).

    Firmware punya DUA jalan memutar dan keduanya TIDAK setara:

      'O<der>'      pivot gait -- melangkah, sudut berapa pun, tapi seluruhnya
                    digerakkan umpan balik IMU. Selagi IMU mati,
                    nav.pivotRelatif() kembali DIAM-DIAM: tidak ada satu baris
                    pun di log serial, dan gejalanya sama persis dengan kabel
                    putus. Inilah sebabnya tombol 90 der di atas tidak pernah
                    berbuat apa-apa sejak 6 September.
      'r0 0 <yaw>'  putar BADAN di atas kaki yang tetap menapak. TIDAK butuh
                    IMU -- ini satu-satunya rotasi yang terbukti jalan sekarang
                    -- tapi firmware membatasi BODY_MAX_ROT_DEG = 20 der.

    Dan 'r' itu perintah POSISI, bukan kecepatan: 'r0 0 18' dua kali TIDAK
    menghasilkan 36 der, yang kedua tidak menggerakkan apa pun. Jadi sudut
    yang diminta ditambahkan ke pose yang sedang berlaku, lalu dipotong --
    dan kalau potongannya membuat gerakannya nol, itu dikatakan, bukan
    dikirim diam-diam sebagai perintah yang terlihat berhasil di log.

    Sumbu tanda mengikuti firmware: der > 0 = KIRI, der < 0 = KANAN.
    """
    try:
        der = float(der)
    except (TypeError, ValueError):
        return None, "derajat tidak sah"
    if der == 0.0:
        return None, "0 derajat -- tidak ada yang dikerjakan"
    if metode not in ("auto", "gait", "badan"):
        metode = "auto"
    muat = abs(misi.badan_yaw + der) <= kalib.badan_yaw_maks + 1e-6
    if metode == "auto":
        # Selagi IMU belum hidup, 'badan' satu-satunya yang benar-benar
        # memutar robot. Dipilih setiap kali sudutnya masih muat.
        metode = "badan" if muat else "gait"

    if metode == "gait":
        d = pivot_der(der, kalib)
        aksi.jadwal((f"O{d:d}", 2.5))
        return f"O{d:d}", ("pivot gait -- TIDAK akan bergerak kalau IMU mati "
                           "(cek tombol 'Aliran yaw')")

    sasar = max(-kalib.badan_yaw_maks, min(kalib.badan_yaw_maks,
                                           misi.badan_yaw + der))
    geser = sasar - misi.badan_yaw
    if abs(geser) < 0.05:
        return None, (f"pose badan sudah mentok di {misi.badan_yaw:+.1f} der "
                      f"(batas {kalib.badan_yaw_maks:.0f}). Nolkan pose dulu, "
                      f"atau pakai metode gait.")
    tunggu = abs(geser) / max(kalib.slew_deg_s, 1e-6) + kalib.diam_margin_s
    misi.badan_yaw = sasar
    aksi.jadwal((f"r0 0 {sasar:.2f}", tunggu))
    catatan = f"badan {sasar:+.1f} der (bergeser {geser:+.1f})"
    if abs(geser - der) > 0.05:
        catatan += (f" -- DIPOTONG dari {der:+.1f}, badan tidak bisa lebih dari "
                    f"{kalib.badan_yaw_maks:.0f} der")
    return f"r0 0 {sasar:.2f}", catatan


def jarak_ambil_cm(kalib: Kalib):
    """Jarak berhenti yang benar, menurut cara mengambil yang dipilih.

    Satu fungsi, dipakai semua tempat. Sebelumnya capit_cm tersebar di rantai
    AMBIL, dan mengganti cara mengambil berarti mengubah beberapa tempat
    sekaligus -- persis cara sebuah angka jadi tertinggal di satu tempat.
    """
    if kalib.mode_ambil == "lengan":
        return kalib.korban_jarak_cm
    return kalib.capit_cm


def periksa_geometri_lengan(kalib: Kalib):
    """Periksa jangkauan/tinggi/vektor lengan saling konsisten.

    hypot(jangkauan, tinggi) harus sama dengan vektor yang diukur. Kalau
    tidak, salah satu dari ketiga angka itu salah ketik -- dan salah ketik di
    sini membuat lengan menjulur ke tempat yang bukan tempat korban, tanpa
    ada satu pun pesan error karena firmware dengan senang hati mengerjakan
    koordinat yang sah tapi keliru.

    Kembalikan (cocok: bool, hitung_cm: float, teks).

    CATATAN JUJUR: ini hanya memeriksa BESARNYA. Tanda lengan_tinggi_cm --
    lengan naik atau turun -- tidak bisa diperiksa dari sini sama sekali,
    karena hypot membuang tandanya. Itu harus dilihat dengan mata.
    """
    d = math.hypot(kalib.lengan_jangkau_cm, kalib.lengan_tinggi_cm)
    beda = d - kalib.lengan_vektor_ukur_cm
    cocok = abs(beda) <= kalib.lengan_vektor_tol_cm
    teks = (f"hypot({kalib.lengan_jangkau_cm:.0f}, "
            f"{abs(kalib.lengan_tinggi_cm):.0f}) = {d:.2f} cm "
            f"vs ukur {kalib.lengan_vektor_ukur_cm:.0f} cm "
            f"({beda:+.2f})")
    return cocok, d, teks


def tunggu_capit_s(kalib: Kalib, dari_persen: float, ke_persen: float) -> float:
    """Berapa detik rahang butuh berpindah lebar, plus capit_margin_s.

    Rahang menempuh seluruh 180 derajat servo untuk 0..100 persen, jadi
    selisih persen bisa diubah jadi waktu asal laju slew-nya diketahui.

    Tidak ada sensor yang bisa melaporkan rahang sudah sampai. Menunggu adalah
    satu-satunya cara "menunggu sampai", persis seperti di firmware -- dan
    itulah kenapa jatahnya harus ikut berubah saat lengan_slew_deg_s disetel.
    """
    der = abs(ke_persen - dari_persen) / 100.0 * 180.0
    return der / max(kalib.lengan_slew_deg_s, 1e-6) + kalib.capit_margin_s


def pose_jepit_mm(kalib: Kalib):
    """Pose lengan saat menjepit, dalam mm, siap dipakai perintah 'a'.

    Mode "lengan": dari yang KAMU ukur di robot -- jangkauan 25 cm, tinggi
    12 cm. Firmware memakai mm dari PUSAT BADAN, jadi dikalikan 10.

    Mode "badan": lengan_r/lengan_h yang lama, karena di sana robot yang
    berjalan mendekat dan lengan cuma perlu menjangkau sedikit.
    """
    if kalib.mode_ambil == "lengan":
        return kalib.lengan_jangkau_cm * 10.0, kalib.lengan_tinggi_cm * 10.0
    return kalib.lengan_r, kalib.lengan_h


def korban_kini(misi):
    """Nama posisi korban yang sedang dikerjakan ("K1".."K5"), atau "".

    Dipakai untuk memilih tinggi efektif per posisi. Kalau robot sedang tidak
    di ruang korban mana pun, kembalikan "" dan pemanggilnya jatuh ke nilai
    global -- bukan ke nol, yang akan membuat jaraknya nol.
    """
    try:
        baris = MISI[misi.idx]
    except (IndexError, TypeError):
        return ""
    return baris[0] if baris[2] == KORBAN else ""


def tinggi_efektif_cm(kalib: Kalib, posisi=""):
    """Tinggi efektif yang berlaku: per posisi kalau ada, global kalau tidak.

    0 berarti "belum dikalibrasi di posisi ini", BUKAN "tingginya nol" --
    jadi 0 jatuh ke global, tidak dipakai apa adanya. Kalau tidak begitu,
    posisi yang belum sempat dikalibrasi akan membuat jaraknya meledak ke
    tak hingga tanpa ada yang menyadarinya.
    """
    n = getattr(kalib, f"tinggi_{posisi.lower()}", 0.0) if posisi else 0.0
    return n if n and n > 0 else kalib.korban_tinggi_cm


def geser_aktif(kalib: Kalib, posisi=""):
    """True kalau posisi ini ditengahkan dengan MENGGESER, bukan memutar."""
    if not posisi:
        return False
    return bool(getattr(kalib, f"geser_{posisi.lower()}", False))


def geser_v_cm(kalib: Kalib, posisi=""):
    """Sasaran 'V' cadangan untuk posisi ini, cm. 0 = tidak ada cadangan."""
    if not posisi:
        return 0.0
    return float(getattr(kalib, f"geser_v_cm_{posisi.lower()}", 0.0) or 0.0)


def rasio_wajar(x1, y1, x2, y2, kalib: Kalib):
    """Periksa bentuk bbox, bukan ukurannya. Kembalikan (wajar, rasio).

    Memutar korban pada sumbu tegak tidak mengubah TINGGI-nya, cuma lebarnya,
    jadi rasio lebar/tinggi itu petunjuk orientasi yang murah. Yang dicari di
    sini bukan sudutnya, melainkan bbox yang jelas BUKAN satu korban utuh:
    terpotong tepi frame, bergabung dengan dummy sebelahnya, atau separuh
    tertutup papan 14x17 di K-3/K-4.
    """
    h = float(y2) - float(y1)
    w = float(x2) - float(x1)
    if h <= 1.0:
        return False, 0.0
    r = w / h
    if not kalib.rasio_periksa:
        return True, r
    return kalib.rasio_min <= r <= kalib.rasio_maks, r


def bbox_diharap_px(kalib: Kalib, lebar_frame, jarak_cm, posisi=""):
    """Tinggi bbox yang SEHARUSNYA muncul pada jarak itu. Kebalikan rumus."""
    if jarak_cm <= 0:
        return 0.0
    return kalib.f_px(lebar_frame) * tinggi_efektif_cm(kalib, posisi) / jarak_cm


def tol_px_ke_cm(kalib: Kalib, lebar_frame, jarak_cm, posisi=""):
    """Ambang piksel itu setara berapa cm di jarak ini.

    |dd/dh| = d/h, jadi piksel yang sama bernilai jauh lebih besar di jarak
    jauh. Ditampilkan di HUD supaya bbox_tol_px dan capit_tol_cm bisa disetel
    sepadan, bukan masing-masing ditebak.
    """
    h = bbox_diharap_px(kalib, lebar_frame, jarak_cm, posisi)
    if h <= 0:
        return 0.0
    return jarak_cm / h * kalib.bbox_tol_px


# Awalan sebab penolakan yang berarti "korban KELEWAT DEKAT untuk diukur",
# bukan "kamera tidak tahu". Dua keadaan itu menuntut tindakan berlawanan:
# yang pertama harus MUNDUR, yang kedua boleh menunggu. Dipakai sebagai
# awalan supaya pemanggil tidak perlu mencocokkan kalimat penuh yang berubah
# tiap kali teksnya disunting.
VIS_DEKAT = "TERLALU DEKAT"


def vis_terlalu_dekat(misi):
    """True kalau kamera MENOLAK mengukur karena korban kelewat dekat.

    Penolakan itu sendiri informasi. bbox setinggi itu hanya muncul di bawah
    ~12 cm, jadi 'tidak ada angka' di sini berarti 'dekat', bukan 'tak tahu'.
    """
    return (misi.jarak_vis is None
            and VIS_DEKAT in (misi.jarak_vis_sebab or ""))


def jarak_vision_cm(y1, y2, tinggi_frame, lebar_frame, kalib: Kalib,
                    posisi=""):
    """Jarak ke korban dari TINGGI bbox. Kembalikan (cm | None, sebab).

        d = f * H / h_px       f = (lebar/2) / tan(hfov/2)

    Kembalian None SELALU disertai sebabnya, dan itu disengaja: fungsi ini
    lebih sering menolak daripada menjawab, dan penolakan yang diam akan
    terbaca di HUD sebagai "kamera tidak melihat apa-apa" -- padahal kamera
    melihat dengan jelas, cuma bacaannya tidak boleh dipakai.

    KENAPA PEMOTONGAN DIPERIKSA LEBIH DULU DARI APA PUN.
    Korban setinggi 12 cm mengisi seluruh tinggi frame pada jarak 15 cm.
    Di bawah itu bbox-nya terpotong tepi frame, h_px berhenti tumbuh, dan
    d = f*H/h_px terbaca LEBIH JAUH daripada kenyataan. Arah galatnya yang
    berbahaya: robot mengira masih ada ruang, lalu maju, lalu menabrak
    korban yang mau diselamatkannya. Bacaan seperti itu tidak boleh
    "dikoreksi" -- informasinya memang sudah hilang dari frame.
    """
    if not kalib.vision_jarak_on:
        return None, "jarak vision dimatikan"
    h_px = float(y2) - float(y1)
    if h_px <= 1.0:
        return None, "bbox terlalu tipis"
    m = kalib.bbox_tepi_px
    if y1 <= m or y2 >= tinggi_frame - m:
        return None, (f"{VIS_DEKAT} -- bbox MENYENTUH TEPI frame, terpotong, "
                      f"jarak tak terbaca")
    d = kalib.f_px(lebar_frame) * tinggi_efektif_cm(kalib, posisi) / h_px
    if d < kalib.vision_jarak_min_cm:
        return None, (f"{VIS_DEKAT} -- {d:.0f} cm di bawah pagar "
                      f"{kalib.vision_jarak_min_cm:.0f} cm")
    if d > kalib.vision_jarak_maks_cm:
        return None, f"{d:.0f} cm di atas pagar {kalib.vision_jarak_maks_cm:.0f} cm"
    return d, ""


def tinggi_dari_jarak(y1, y2, lebar_frame, jarak_cm, kalib: Kalib):
    """Kebalikannya: dari jarak yang KAMU ukur dengan penggaris, hitung tinggi
    efektif korban. Inilah cara mengkalibrasinya -- bukan dengan mengukur
    tinggi fisik boneka.

    Bedanya bukan sepele. Yang dibutuhkan rumus bukan tinggi boneka, melainkan
    tinggi yang DILIHAT MODEL: bbox YOLO jarang mepet, sering memotong kaki,
    dan lensanya tidak sempurna. Satu pengukuran penggaris menyerap ketiganya
    sekaligus, dan hasilnya cocok dengan kenyataan -- sedangkan tinggi fisik
    yang "benar" justru membuat rumusnya meleset.
    """
    h_px = float(y2) - float(y1)
    if h_px <= 1.0:
        raise ValueError("bbox terlalu tipis untuk dikalibrasi")
    if jarak_cm <= 0:
        raise ValueError("jarak harus lebih dari 0")
    return jarak_cm * h_px / kalib.f_px(lebar_frame)


def sedang_membawa(link, kalib: Kalib):
    """(True, sebab) kalau vision TIDAK BOLEH mengejar apa pun sekarang.

    Korban yang sedang digendong TERDETEKSI SEBAGAI KORBAN -- ia memang
    korban. Jadi tanpa penjaga ini JEJAK/CENTER akan menengahkan badan ke
    barang yang sudah ada di tangannya sendiri, lalu rantai AMBIL akan maju
    ke arahnya. Robot mengejar muatannya, dan tiap langkahnya terlihat benar
    dari dalam.

    Gerbang kelas tidak menolong di sini: kelasnya MEMANG korban. Gerbang
    ROI atas menolong kalau fraksinya sudah diukur, tapi ia bergantung pose
    dan pemasangan kamera. Yang ini tidak bergantung apa pun kecuali satu
    kalimat yang firmware sudah cetak sendiri.
    """
    if not kalib.tolak_saat_membawa:
        return False, ""
    depan, belakang = link.membawa()
    if not (depan or belakang):
        return False, ""
    isi = ", ".join(n for n, ada in (("depan", depan), ("belakang", belakang))
                    if ada)
    return True, (f"capit {isi} SEDANG memegang korban -- vision tidak mengejar "
                  f"apa pun supaya robot tidak menengahkan diri ke muatannya "
                  f"sendiri. Taruh dulu di safe zone.")


def pilih_sasaran(lolos, names, kalib):
    """Pilih SATU deteksi untuk dikejar. Korban dulu, selalu.

    Mengembalikan (deteksi_atau_None, jumlah_dummy_yang_diabaikan).

    KENAPA DI SINI, BUKAN DI sasaran_sah(). 13 Sep 2026, laporan arena: ada
    dummy di sebelah korban dan robot menengahkan diri ke DUMMY. Sebabnya
    pemilihan sasaran memakai `max(lolos, key=tinggi)` -- yang TERBESAR, tanpa
    peduli kelasnya. Dummy yang kebetulan lebih dekat kamera menang, lalu
    sasaran_sah() menahan gerakan. Hasilnya robot mematung menghadap dummy
    sementara korban berdiri di sebelahnya, tidak pernah dipilih.

    sasaran_sah() menjawab "boleh bergerak?". Itu pertanyaan yang berbeda dari
    "yang mana?", dan selama keduanya dijawab satu fungsi, jawaban yang benar
    untuk pertanyaan kedua tidak pernah ada.

    DI ANTARA SESAMA KORBAN, YANG PALING KANAN (lock_kanan, bawaan NYALA).
    Permintaan R2C 15 Sep 2026: di K-3, K-4 dan K-5 ada DUA korban asli dalam
    satu frame, dan yang harus dikerjakan lebih dulu selalu yang kanan.
    "Paling tinggi" memilih yang paling DEKAT kamera, dan itu bisa korban ruas
    berikutnya -- robot menengahkan diri ke korban yang belum gilirannya.

    Kuncinya tidak menyimpan apa pun. Sesudah korban kanan tertengahkan,
    korban satunya tetap berada di kirinya, jadi pilihan yang sama lahir lagi
    tiap frame. Kunci yang MENGINGAT id sasaran justru lebih buruk di sini:
    deteksi kedip satu frame sudah cukup untuk mengunci sesuatu yang sudah
    tidak ada, dan tidak ada sensor yang bisa membantahnya.

    INI BUKAN PENGULANGAN REGRESI 12 SEPTEMBER. Yang dulu merusak JEJAK adalah
    `lolos = []` -- membuang deteksi dari daftar yang dipakai MENGGAMBAR, jadi
    kotaknya jadi abu-abu tanpa sebab. Fungsi ini tidak menyentuh `lolos` sama
    sekali: anotasi() tetap menggambar semuanya, dummy tetap merah dan tetap
    terlihat. Yang berubah cuma MANA yang dijadikan sasaran.
    """
    if not lolos:
        return None, 0
    korban = [d for d in lolos if names.get(d[5], str(d[5])) == kalib.kelas_korban]
    if korban:
        if kalib.lock_kanan:
            # cx dihitung dari bbox-nya sendiri, bukan diambil dari kolom ke-7
            # milik juri.saring(). Fungsi ini sudah membaca d[1], d[3] dan
            # d[5] langsung; ikut bergantung pada panjang tuple berarti ia
            # pecah untuk pemanggil yang memberi deteksi mentah.
            return max(korban, key=lambda d: (d[0] + d[2]) / 2), 0
        return max(korban, key=lambda d: (d[3] - d[1])), 0
    if kalib.jejak_hanya_korban:
        return None, len(lolos)
    return max(lolos, key=lambda d: (d[3] - d[1])), 0


def sasaran_sah(misi: Misi, kalib: Kalib, hanya_korban):
    """True kalau sasaran yang terlihat boleh dikejar.

    Dipisah jadi fungsi sendiri supaya JEJAK, CENTER dan AMBIL memakai aturan
    yang SAMA PERSIS. Dulu hanya rantai AMBIL yang menolak dummy, jadi robot
    tetap berjalan dan menengahkan diri ke dummy lebih dulu, lalu baru menolak
    di detik terakhir -- sesudah seluruh ongkos geraknya terbayar.
    """
    if not hanya_korban:
        return True
    if not misi.jejak_kelas:
        return False                      # belum tahu kelasnya: jangan bergerak
    return misi.jejak_kelas == kalib.kelas_korban


def pusatkan(misi: Misi, aksi: Aksi, kalib: Kalib, err, pid=None, lebar=1280,
             diam=None):
    """Satu langkah penengahan bertingkat. Mengembalikan True kalau SUDAH tengah.

    DI POSISI YANG MENUNTUTNYA (K-3, K-4) penengahan diserahkan ke
    geser_tengah(), yang tidak memutar badan sama sekali. Penyerahannya di
    SINI, bukan di tiap pemanggil: JEJAK, CENTER, AMBIL dan TAHAN semuanya
    lewat fungsi ini, dan aturan "jangan memutar di depan reruntuhan" berlaku
    untuk keempatnya. Menaruhnya di pemanggil berarti empat tempat untuk lupa.

    KASAR (|err| > yaw_kasar_deg): pivot gait 'O'.
        Gain < 1 karena ada waktu mati ~2,5 detik antara mengukur dan selesai
        bergerak; meminta seluruh galat dijamin melewati sasaran.
        Permintaan dilantai ke pivot_min_deg -- di bawah HEADING_TOLERANCE_DEG
        firmware menganggap dirinya SUDAH lurus dan membuang perintahnya tanpa
        satu pun pesan. Itulah "bergerak tapi tidak jadi".

    HALUS (sisanya): putar BADAN di atas kaki dengan 'r0 0 <yaw>'.
        Tidak melangkah, tidak ada deadband, resolusi pecahan derajat.
        Ini yang membuat +/-1,5 der mungkin -- pivot gait tidak akan pernah bisa.
    """
    # PALING ATAS, sebelum satu pun perintah putar bisa lahir. K-3 dan K-4
    # tidak boleh diputar sama sekali -- capitnya menyapu reruntuhan.
    if geser_aktif(kalib, korban_kini(misi)):
        return geser_tengah(misi, aksi, kalib, err, lebar, diam)

    # Toleransi dihitung dari PIKSEL, bukan derajat: itu satuan yang benar-benar
    # kamu lihat di layar, dan tidak ikut berubah kalau resolusi kamera diganti.
    tol = min(kalib.tengah_tol_deg,
              kalib.tengah_tol_px / max(kalib.px_per_deg(lebar), 1e-6))

    # TAHAN DULU. Satu frame di dalam toleransi bukan bukti robot DIAM di
    # tengah -- bisa jadi ia sedang melewatinya. Jadi syaratnya harus
    # bertahan `tengah_tahan_s` detik tanpa putus, dan putus sekali saja
    # menolkan hitungannya.
    #
    # Ini yang diminta R2C: pemicunya datang dari Pi saat ia YAKIN, bukan dari
    # jam yang habis. Penengahan yang sehat memenuhi ini dalam beberapa detik.
    if abs(err) <= tol:
        if misi.t_tengah is None:
            misi.t_tengah = time.time()
        misi.tahan_s = time.time() - misi.t_tengah
        # YAKIN -> LANGSUNG. Bacaan yang sudah mengendap (lahir sesudah
        # gerakan terakhir selesai, dan sudah cukup banyak) yang berada JAUH di
        # dalam toleransi sudah membuktikan dua hal sekaligus: robot diam, dan
        # ia di tengah. Itu persis yang hendak dibuktikan dwell, jadi menunggu
        # sisa detiknya tidak menambah bukti apa pun.
        if (kalib.tengah_yakin_deg > 0 and diam is not None and diam.boleh
                and diam.rata is not None
                and abs(diam.rata) <= kalib.tengah_yakin_deg):
            return True
        if misi.tahan_s >= kalib.tengah_tahan_s:
            return True
        return False            # di tengah, tapi belum cukup lama
    misi.t_tengah = None
    misi.tahan_s = 0.0
    if not aksi.kosong():
        return False

    if abs(err) > kalib.yaw_kasar_deg:
        # APAKAH 'O' YANG TADI BEKERJA? Diukur, bukan diasumsikan.
        #
        # Kalau galatnya praktis tidak berubah sesudah 'O' dikirim, 'O' tidak
        # menggerakkan apa pun -- dan di robot ini sebabnya sudah diketahui:
        # nav.pivotRelatif() kembali diam-diam tanpa data IMU. Mengirimnya
        # lagi tidak akan lebih berhasil. Dua kali cukup untuk memastikan itu
        # bukan kebetulan satu frame.
        if misi.o_err_lalu is not None:
            if abs(err - misi.o_err_lalu) < kalib.pivot_gait_beda_min:
                misi.o_diam += 1
            else:
                misi.o_diam = 0
        if misi.o_diam >= 2 and not misi.pivot_gait_mati:
            misi.pivot_gait_mati = True

        if kalib.pakai_pivot_gait and not misi.pivot_gait_mati:
            minta = pivot_der(err * kalib.pivot_gain, kalib)
            if abs(misi.badan_yaw) > 0.1:
                aksi.jadwal(("r0 0 0", 0.8))
                misi.badan_yaw = 0.0
            misi.o_err_lalu = err
            aksi.jadwal((f"O{-minta:d}", 2.5), ("l", 0.3))
            return False

        # 'O' tidak tersedia. Yang tersisa putar badan, dan ia terbatas
        # +-badan_yaw_maks. Kalau sasarannya di luar itu, katakan -- jangan
        # mencoba selamanya. Operator bisa memutar robotnya dengan tangan;
        # HUD tidak bisa, dan berpura-pura bisa cuma membuang jam kontes.
        if abs(err) > kalib.badan_yaw_maks - 1.0:
            misi.gagal(
                f"sasaran {err:+.1f} der, di luar jangkauan putar badan "
                f"(+/-{kalib.badan_yaw_maks:.0f}). Pivot gait 'O' tidak "
                f"menggerakkan apa pun -- hampir pasti IMU mati, jadi tidak ada "
                f"cara melangkah memutar. Putar robot dengan tangan sampai "
                f"korban kira-kira di depan, lalu ulangi.")
            return False
        # Masih di dalam jangkauan badan: lanjut ke tingkat halus di bawah.

    # --- halus: putar badan ---
    if kalib.metode_tengah == "diam" and diam is not None:
        # Bacaan yang lahir sebelum gerakan terakhir selesai DIBUANG di
        # tambah(); yang lolos dirata-rata. Selama belum cukup, JANGAN
        # bergerak -- diam sebentar jauh lebih murah daripada bergerak ke
        # arah yang salah lalu harus dikoreksi balik.
        # Dinilai memakai waktu frame DIAMBIL, bukan waktu hasilnya dipakai.
        # Selisih keduanya = waktu inferensi + antrean kamera, dan pada Pi itu
        # ratusan milidetik -- cukup untuk meloloskan frame yang terekam
        # selagi badan masih berputar, lalu mengoreksi dari pose yang sudah
        # tidak ada lagi.
        if not diam.tambah(err, misi.bearing_t):
            return False
        if not diam.boleh:
            return False
        e = diam.rata
        if abs(e) <= tol:
            return True
        # Langkah sebanding jaraknya: jauh = besar, dekat = halus. Persis
        # yang kamu minta. Batas bawah supaya koreksi kecil tidak jadi nol
        # lalu menggantung selamanya di ambang toleransi.
        langkah = e * kalib.langkah_gain
        if abs(langkah) < kalib.langkah_min_deg:
            langkah = math.copysign(kalib.langkah_min_deg, langkah)
        langkah = max(-kalib.maks_langkah_deg,
                      min(kalib.maks_langkah_deg, langkah))
        yaw = misi.badan_yaw - langkah
    elif kalib.metode_tengah == "sekali":
        # SEKALI TEMBAK. Hubungan badan<->bearing itu geometri yang sudah
        # diketahui, bukan misteri yang harus diraba dengan umpan balik.
        # Koreksi SELURUH galat sekaligus, lalu ukur ulang untuk memastikan.
        #
        # Sebelum menggerakkan badan, PELAJARI dulu dari gerakan sebelumnya:
        # kalau tadi kita menggeser badan sekian dan bearing ternyata berubah
        # lebih/kurang dari itu, faktornya dibetulkan. Itulah yang menangani
        # parallax kamera yang tidak duduk di sumbu putar -- tanpa perlu tahu
        # jarak kamera ke sumbu.
        if (kalib.skala_belajar and misi.tembak_yaw is not None
                and misi.tembak_err is not None):
            d_yaw = misi.badan_yaw - misi.tembak_yaw      # yang kita perintahkan
            d_err = err - misi.tembak_err                 # yang benar-benar terjadi
            if abs(d_yaw) > 0.3 and abs(d_err) > 0.05:
                # Tandanya: yaw+ = badan berbelok KIRI, dan itu menggeser
                # sasaran makin ke KANAN di gambar -- jadi bearing ikut NAIK.
                # Artinya d_err dan d_yaw searah, dan respons idealnya +1.
                # Lebih dari 1 = badan bereaksi berlebihan (kamera di depan
                # sumbu putar), jadi perintah berikutnya harus DIKURANGI.
                nyata = d_err / d_yaw
                if 0.05 < nyata < 5.0:
                    baru = kalib.skala_yaw * 0.6 + (1.0 / nyata) * 0.4
                    kalib.skala_yaw = max(kalib.skala_min,
                                          min(kalib.skala_maks, baru))
        misi.tembak_err = err
        misi.tembak_yaw = misi.badan_yaw
        yaw = misi.badan_yaw - err * kalib.skala_yaw
    elif pid is not None and kalib.pid_aktif and kalib.metode_tengah == "pid":
        delta, e_saring = pid.langkah(err)
        # Diukur lagi SESUDAH disaring: satu frame nyasar tidak boleh
        # menghentikan penengahan lebih awal.
        if abs(e_saring) <= tol:
            return True
        yaw = misi.badan_yaw - delta
    else:
        yaw = misi.badan_yaw - err * kalib.halus_gain
    yaw = max(-kalib.badan_yaw_maks, min(kalib.badan_yaw_maks, yaw))
    if abs(yaw - misi.badan_yaw) < 0.05:
        # Sudah mentok di batas badan dan galatnya tidak mengecil lagi.
        # Serahkan ke pivot gait sekali, lalu mulai lagi dari badan netral.
        aksi.jadwal(("r0 0 0", 0.8),
                    (f"O{-pivot_der(err, kalib):d}", 2.5))
        misi.badan_yaw = 0.0
        return False
    # Tunggu sampai badan BENAR-BENAR diam sebelum frame berikutnya dipakai.
    # Tanpa ini kita mengukur badan yang masih bergerak, lalu mengoreksi
    # berdasar angka yang sudah basi -- itu yang membuatnya berayun terus.
    geser = abs(yaw - misi.badan_yaw)
    tunggu = geser / max(kalib.slew_deg_s, 1e-6) + kalib.diam_margin_s
    misi.badan_yaw = yaw
    aksi.jadwal((f"r0 0 {yaw:.2f}", tunggu))
    # BUANG sampel lama. Ini bug yang membuatnya berayun: median 5 sampel
    # menahan bacaan dari pose LAMA, jadi sesudah badan bergerak kita masih
    # mengoreksi berdasar posisi sebelum bergerak -- lalu kelewatan, lalu
    # balik lagi. Persis "kelebihan kanan lalu kiri".
    if pid is not None:
        pid.sampel.clear()
        pid.e_lalu = None
    if diam is not None:
        diam.tunda(tunggu)
    return False


def geser_tengah(misi: Misi, aksi: Aksi, kalib: Kalib, err, lebar=1280,
                 diam=None):
    """Satu langkah penengahan TANPA MEMUTAR. True kalau sudah tengah.

    Dipakai di K-3 dan K-4, tempat memutar badan menyapukan capit ke
    reruntuhan yang menutupi korban. Bentuknya sama persis dengan pusatkan():
    satu langkah per panggilan, tidak memblokir, dan True hanya saat sasaran
    benar-benar sudah di tengah.

    SATU TINGKAT sejak 17 Sep 2026, diminta R2C. Dulu ada dua: 't' menggeser
    BADAN di atas kaki yang diam, dan 'H' melangkah satu SIKLUS GAIT ke
    samping saat simpangannya lebih besar daripada jangkauan badan. 'V'
    (ratakan ke dinding) jadi cadangan terakhirnya.

    'H' dan 'V' MEMINDAHKAN KAKI. Keputusan R2C sesudah trial: satu-satunya
    perintah gerak yang boleh datang dari Pi adalah 't<x> 0 0' dan 'O<der>' --
    yang menggeser atau memutar badan di atas kaki yang DIAM. Segala yang
    melangkah milik Teensy.

    HARGANYA JANGKAUAN. Penengahan ini sekarang terbatas pada
    `badan_geser_maks` mm ke tiap sisi (juga di-clamp firmware ke
    BODY_MAX_TRANS_MM). Korban yang lebih jauh dari itu TIDAK bisa ditengahkan
    dari sini; fungsi ini melapor sekali lalu berhenti mencoba, dan batas waktu
    state-nya yang menyerahkan ke firmware. Kalau ini sering terjadi, yang
    disetel letak berhenti ruas di tabel misi -- bukan fungsi ini.
    """
    tol = kalib.tengah_tol_px / kalib.px_per_deg(lebar)
    if abs(err) <= tol:
        return True
    if not aksi.kosong():
        return False                      # langkah sebelumnya belum selesai

    d_cm = misi.jarak_vis or jarak_ambil_cm(kalib)
    # Simpangan sudut -> simpangan menyamping di jarak korban. Badan digeser ke
    # ARAH sasaran: sasaran di kanan (err+) -> badan ke kanan.
    mm = 10.0 * d_cm * math.tan(math.radians(err))
    sasar = misi.badan_x + mm
    if abs(sasar) > kalib.badan_geser_maks:
        # MENTOK. Tidak ada lagi tingkat kasar yang bisa menolong, jadi ini
        # diumumkan sekali lalu dibiarkan. Diam lebih baik daripada menggeser
        # badan ke batas dan berpura-pura sudah tengah.
        if not misi.geser_v_dipakai:
            misi.geser_v_dipakai = True   # dipakai lagi sebagai "sudah lapor"
            misi.sebab = (f"geser badan MENTOK di {kalib.badan_geser_maks:.0f} mm, "
                          f"korban masih {err:+.1f} der dari tengah. 'H' dan 'V' "
                          f"sudah dibuang -- Pi tidak boleh memindahkan kaki. "
                          f"Setel letak berhenti ruas di tabel misi.")
        return False
    misi.badan_x = sasar
    aksi.jadwal((f"t{sasar:.0f} 0 0", kalib.geser_tunggu_s))
    if diam is not None:
        diam.tunda(kalib.geser_tunggu_s)
    return False


def netralkan_badan(misi: Misi, aksi: Aksi, di_depan=False):
    """Kembalikan pose badan ke nol. WAJIB sebelum berjalan.

    Badan yang masih menyerong 15 der lalu disuruh 'w' akan berjalan miring:
    gait mengarah ke depan KAKI, sedangkan kamera dan capit mengarah ke depan
    BADAN. Keduanya tidak sama lagi begitu badan diputar.
    """
    if abs(misi.badan_yaw) < 0.05 and abs(misi.badan_x) < 0.05:
        return
    p = [("r0 0 0", 0.8), ("t0 0 0", 0.8)]
    if di_depan:
        aksi.antre[:0] = p
    else:
        aksi.jadwal(*p)
    misi.badan_yaw = misi.badan_x = 0.0


def serahkan_ke_firmware(misi: Misi, link, aksi: Aksi, kalib: Kalib, kamera,
                         err, sebab):
    """Jawab 'm2' SAMBIL MENAHAN pose tengah, lalu masuk TAHAN_TENGAH.

    Dipakai DUA kali: penengahan yang SELESAI, dan penengahan yang KEHABISAN
    WAKTU. Ditulis sekali supaya keduanya tidak bisa menyimpang.

    DI SINI DULU ADA netralkan_badan(), DAN ITULAH BUG-nya. Laporan R2C
    14 Sep 2026: "vision sudah tengah, tapi begitu capit turun posisi badan
    kembali ke default dan capit tidak tengah lagi." Penengahan halus
    dikerjakan dengan MEMUTAR BADAN ('r0 0 <yaw>'); mengembalikan yaw itu ke
    nol tepat sebelum 'm2' membuang seluruh koreksi yang barusan dibayar --
    lalu firmware menurunkan capit pada badan yang lurus ke depan KAKI, bukan
    ke korban. Tiap bagiannya bekerja sesuai rancangan, dan hasil gabungannya
    persis kebalikan dari yang dikerjakan 8 detik sebelumnya.

    Badannya TIDAK netral di sini. Yang menetralkannya S_TAHAN, sesudah capit
    terangkat -- dan di sana netralkan_badan() tetap WAJIB, karena langkah
    pertama ruas berikutnya berjalan ke arah KAKI, bukan ke arah badan.
    """
    tahan = misi.tahan_s
    if kalib.auto_konfirm:
        aksi.jadwal(("m2", 0.5))
        link.log.append(
            f"[SERAH] {sebab}. Galat {abs(err):.1f} der, bertahan {tahan:.1f} "
            f"detik. Pose tengah DITAHAN (badan yaw {misi.badan_yaw:+.1f} der, "
            f"TIDAK dinetralkan), 'm2' dikirim -- Teensy menurunkan capit di "
            f"badan yang sudah tengah. Kaki baru dilepas "
            f"{kalib.lepas_tahan_s:.1f} detik sesudah capit terangkat.")
    else:
        link.log.append(
            f"[SERAH] {sebab}. Galat {abs(err):.1f} der. Pose tengah DITAHAN "
            f"(badan yaw {misi.badan_yaw:+.1f} der). auto_konfirm MATI -- "
            f"tekan 'm2' (tab Manual) supaya Teensy melanjutkan.")
    # KAMERA TETAP HIDUP, dan itu berubah dari sebelumnya. Selama capit turun,
    # vision-lah yang menahan badan tetap tengah; mematikannya di sini berarti
    # tidak ada lagi yang bisa mengoreksi selama 12 detik sekuens capit.
    # Ia dimatikan di ujung S_TAHAN, sesudah kendali benar-benar dilepas.
    misi.t_lepas = None
    misi._tahan_sebab = ""
    misi.ganti(S_TAHAN, "tahan tengah sampai capit firmware selesai")


def tahan_tengah(misi: Misi, link, aksi: Aksi, kalib: Kalib, pid, lebar, diam):
    """Koreksi tengah HALUS SAJA, selagi lengan firmware bekerja.

    Bedanya dengan memanggil pusatkan() apa adanya: di sini KAKI TIDAK BOLEH
    BERGERAK SAMA SEKALI. Lengan sedang turun di antara boneka yang berjarak
    8 cm, dan pusatkan() menjadwalkan pivot gait 'O' di dua tempat -- begitu
    galat melewati yaw_kasar_deg, dan begitu putar-badan mentok di batasnya.
    Satu langkah 'O' di detik itu menyeret seluruh robot dan menyenggol
    korban yang sedang dijepit. Kedua pintu itu ditutup di sini, dan yang
    tersisa cuma 'r0 0 <yaw>': putar badan di atas kaki yang diam.

    Kalau koreksi tidak boleh dikerjakan, robot MENAHAN pose yang ada --
    bukan kembali ke netral. Pose tengah yang sedikit meleset selalu lebih
    baik daripada pose default yang sudah pasti meleset.
    """
    def diam_saja(sebab):
        if sebab and misi._tahan_sebab != sebab:
            misi._tahan_sebab = sebab
            link.log.append(f"[TAHAN] pose tengah ditahan apa adanya "
                            f"(badan yaw {misi.badan_yaw:+.1f} der): {sebab}")

    # Korban sudah di dalam capit -> kelasnya MEMANG korban, dan mengejarnya
    # berarti menengahkan badan ke muatan sendiri. Penjaga yang sama dipakai
    # JEJAK dan CENTER; sesudah capit menutup, inilah yang mengunci koreksi.
    bawa, sebab_bawa = sedang_membawa(link, kalib)
    if bawa:
        return diam_saja(sebab_bawa)
    err = misi.bearing_deg
    if err is None:
        return diam_saja("")           # tidak terlihat: diam, tanpa ribut
    # DUMMY DIABAIKAN, sama seperti di seluruh rantai AMBIL. Gerbangnya
    # sasaran_sah(), bukan penglihatan: kotaknya tetap digambar di HUD.
    if not sasaran_sah(misi, kalib, kalib.ambil_hanya_korban):
        return diam_saja(f"sasaran terbaca "
                         f"{misi.jejak_kelas or 'tidak jelas'}, bukan "
                         f"{kalib.kelas_korban}")
    if abs(err) > kalib.yaw_kasar_deg:
        return diam_saja(
            f"galat {err:+.1f} der lebih besar dari yaw_kasar_deg "
            f"{kalib.yaw_kasar_deg:.1f} -- memperbaikinya butuh pivot KAKI, "
            f"dan kaki tidak boleh melangkah selagi capit di bawah")
    if abs(misi.badan_yaw) >= kalib.badan_yaw_maks - 0.5:
        return diam_saja(
            f"putar badan sudah mentok di +/-{kalib.badan_yaw_maks:.0f} der -- "
            f"sisanya cuma bisa dengan pivot kaki, yang dilarang di sini")
    misi._tahan_sebab = ""
    pusatkan(misi, aksi, kalib, err, pid, lebar, diam)


def lepaskan_kaki(misi: Misi, link, aksi: Aksi, kalib: Kalib, kamera, sebab):
    """Netralkan badan lalu jadwalkan 'm9'. Urutannya yang penting.

    'm9' berarti "Pi sudah tidak memegang kaki". Mengirimnya selagi antrean
    kita sendiri masih berisi perintah pose adalah kebohongan yang baru
    ketahuan saat robot menyentak di langkah pertama ruas berikutnya -- dan
    saat itu sebabnya sudah jauh di belakang. Jadi: kosongkan antrean, isi
    dengan netralisasi, dan taruh 'm9' PALING BELAKANG supaya ia benar-benar
    terkirim sesudah badan kembali ke nol.

    Netralisasinya sendiri tidak bisa dilewati: gait mengarah ke depan KAKI,
    sedangkan badan yang barusan ditahan menyerong beberapa derajat.
    """
    aksi.batal()
    netralkan_badan(misi, aksi)
    aksi.jadwal(("m9", 0.3))
    if kalib.kamera_ikut_pemicu and kamera is not None:
        link.log.append("[KAMERA] " + kamera.saklar(
            False, "kendali kaki dilepas ke firmware"))
    link.log.append(
        f"[LEPAS] {sebab}. Badan dinetralkan lalu 'm9' dikirim -- kendali "
        f"kaki kembali ke Teensy, misi dilanjutkan.")


def tangani_pemicu(misi: Misi, link, aksi: Aksi, kalib: Kalib, armed,
                   kamera=None):
    """Tanggapi "#KORBAN AMBIL <ruas>" dari firmware v1.12.

    Kembalikan True kalau pemicunya dikonsumsi.

    SATU KEPUTUSAN YANG MENENTUKAN SEGALANYA DI SINI: apakah firmware sedang
    MENUNGGU, atau sedang BEKERJA?

    Di v1.12 apa adanya, jawabannya "bekerja". Vincent menulis pemicunya
    sebagai kirim-lalu-lanjut ("Kirim saja, tidak menunggu jawaban"), dan
    sebaris di bawahnya Teensy masuk MISI_LENGAN lalu mulai menurunkan capit.
    Kalau Pi ikut menggerakkan badan di detik itu, ada DUA penguasa untuk
    satu robot -- dan yang kalah adalah korban yang tersenggol capit yang
    sedang turun.

    Firmware baru benar-benar menunggu kalau ruasnya ber-AKS_KONFIRM; saat
    itu ia mencetak "MENUNGGU KONFIRMASI" dan diam sampai dijawab 'm2'.
    Mekanisme itu SUDAH ADA dan sudah lengkap (Misi::jawab -> ruasBerikut);
    yang belum ada cuma barisnya di tabel RUAS.

    Jadi yang dicari di sini BUKTI, bukan niat yang ditulis di knob.
    """
    # SERAH-TERIMA BALIK, dan ia dijawab PALING DULU.
    #
    # Firmware sudah selesai dengan capitnya dan sekarang berdiri diam sambil
    # menggenggam korban, menunggu izin melangkah lagi. Tiap frame yang lewat
    # tanpa 'm9' adalah waktu kontes yang hangus, jadi ini didahulukan di atas
    # segala pemicu lain.
    if getattr(link, "lepas_baru", False):
        link.lepas_baru = False
        ruas = link.lepas[0] if getattr(link, "lepas", None) else "?"
        # '#LEPAS' = capit SELESAI TERANGKAT, bukan "silakan lepas sekarang
        # juga". Dulu baris ini langsung mengirim 'm9', jadi pose tengah
        # dibuang di detik yang sama korban terangkat. Sekarang waktunya yang
        # dicatat; S_TAHAN yang menahan lepas_tahan_s detik lagi, menetralkan
        # badan, baru menjawab.
        if misi.state == S_TAHAN:
            misi.t_lepas = time.time()
            link.log.append(
                f"[LEPAS] firmware selesai mengangkat korban di ruas {ruas}. "
                f"Vision MASIH menahan badan tetap tengah "
                f"{kalib.lepas_tahan_s:.1f} detik lagi sebelum kaki dilepas.")
            return True
        # Bukan dari S_TAHAN: HUD tidak sedang memegang kaki sama sekali
        # (misal ia baru di-restart di tengah sekuens). Tidak ada pose yang
        # perlu ditahan dan tidak ada yang perlu dinetralkan -- jawab saja,
        # karena firmware menunggu dan tiap detiknya waktu kontes.
        aksi.batal()
        misi.dari_firmware = False
        misi.parkir_terlihat = False
        misi.t_tengah = None
        misi.tahan_s = 0.0
        misi.t_lepas = None
        link.kirim("m9", armed)
        link.log.append(
            f"[LEPAS] firmware selesai mengambil korban di ruas {ruas}, tapi "
            f"HUD tidak sedang menahan pose (state {misi.state}). 'm9' "
            f"dikirim langsung -- kendali kaki kembali ke Teensy.")
        if misi.state != S_GAGAL:
            misi.ganti(S_IDLE, f"kendali kaki dilepas ke firmware, ruas {ruas}")
        return True

    # getattr, bukan akses langsung: Teensy yang dibuat lewat jalur lain
    # (uji, atau versi lama yang ter-pickle) bisa tidak punya medan ini, dan
    # FSM tidak boleh mati karena pemicu yang memang tidak pernah ada.
    if getattr(link, "pemicu_baru", False):
        link.pemicu_baru = False
        aksi_fw, ruas, _ = link.pemicu
        cara = kalib.pemicu_korban
        if cara == "mati":
            link.log.append(f"[PEMICU] #KORBAN {aksi_fw} ruas {ruas} -- diabaikan "
                            f"(pemicu_korban = mati)")
            return True
        if aksi_fw != "AMBIL":
            link.log.append(f"[PEMICU] #KORBAN TARUH ruas {ruas} -- vision tidak "
                            f"dipakai untuk menaruh, diabaikan")
            return True
        if cara == "lihat":
            misi.ganti(S_AWAS, f"pemicu firmware ruas {ruas}")
            link.log.append(f"[PEMICU] #KORBAN AMBIL ruas {ruas} -- vision MENILAI "
                            f"(tanpa gerak). Hasilnya di kartu Penilaian slot.")
            return True

        # ambil_alih: JANGAN putuskan sekarang. Pada detik ini satu-satunya
        # keadaan firmware yang kita punya adalah hasil poll 'm' terakhir --
        # umurnya sampai satu detik, dan ia dicetak SEBELUM firmware berhenti.
        # Memutuskan di sini berarti hampir selalu menyimpulkan "tidak parkir"
        # untuk firmware yang detik itu juga sedang parkir. Jadi: catat, minta
        # status baru, dan putuskan di loop berikutnya saat buktinya segar.
        misi.pemicu_tunda = (ruas, time.time(), getattr(link, "n_status", 0))
        link.kirim("m", armed, rutin=True)
        return True

    if not misi.pemicu_tunda:
        return False

    ruas, sejak, n0 = misi.pemicu_tunda
    fw = link.state_teensy().upper()
    # DUA bukti, dan yang pertama tidak butuh poll sama sekali: firmware yang
    # dipatch mengumumkan parkirnya sendiri di baris berikutnya. Yang kedua
    # jaring pengaman kalau barisnya hilang tertelan banjir log -- dan ia HARUS
    # datang sesudah pemicu (n_status naik), kalau tidak ia cuma status basi
    # yang sama yang dulu membuat semuanya salah.
    parkir = (getattr(link, "parkir_visi", False)
              or (getattr(link, "n_status", 0) > n0 and "KONFIRMASI" in fw))
    if not parkir:
        if time.time() - sejak < PEMICU_TUNGGU_BUKTI_S:
            return False                       # masih menunggu; belum diputuskan
        misi.pemicu_tunda = None
        misi.ganti(S_AWAS, f"pemicu firmware ruas {ruas}")
        link.log.append(
            f"[PEMICU] #KORBAN AMBIL ruas {ruas} -- diminta AMBIL ALIH, tapi "
            f"{PEMICU_TUNGGU_BUKTI_S:.0f} detik berlalu tanpa firmware "
            f"mengumumkan parkir ('PARKIR UNTUK VISION') dan tanpa status baru "
            f"yang menyebut KONFIRMASI. Firmware yang terpasang hampir pasti "
            f"v1.12 ASLI: ia mengirim pemicu lalu LANGSUNG menurunkan capit, "
            f"jadi menggerakkan badan sekarang = dua penguasa satu robot. "
            f"Turun ke LIHAT: vision menilai, NOL perintah gerak. "
            f"Perbaikannya: flash patch R2C, lalu pastikan 'm8' NYALA.")
        return True

    misi.pemicu_tunda = None
    misi.parkir_terlihat = False
    aksi.batal()
    misi.dari_firmware = True
    # KAMERA DINYALAKAN, TANPA SYARAT. Knob kamera_ikut_pemicu mengurus
    # MEMATIKAN kamera sesudah kendali dilepas; ia tidak boleh ikut menentukan
    # penyalaan. Firmware sekarang PARKIR menunggu 'm2' di tiap ruas AMBIL
    # (K-1..K-5), jadi kamera yang mati di sini berarti robot berdiri diam
    # sampai MISI_VISI_BATAS_MS habis lalu menurunkan capit dengan pose apa
    # adanya -- kegagalan yang tidak menyebut kamera sama sekali.
    #
    # Operator bisa mematikan kamera kapan saja ('fokus_vision', saklar HUD);
    # penyalaan di sini yang membuat matinya tidak pernah menjalar ke ruas
    # korban berikutnya.
    if kamera is not None and not kamera.nyala:
        link.log.append("[KAMERA] " + kamera.saklar(True)
                        + " -- pemicu AMBIL dari firmware, vision wajib nyala")
    # Hitungan dwell & deteksi 'O' dimulai bersih untuk korban ini.
    misi.t_tengah = None
    misi.tahan_s = 0.0
    misi.geser_n = 0
    misi.geser_v_dipakai = False
    misi.o_err_lalu = None
    # 'J' DIBUANG, 17 Sep 2026, diminta R2C sesudah trial arena.
    #
    # 'J<cm>' menyetel jarak ke dinding BELAKANG, dan ia berumpan-balik DUA
    # ARAH: maju kalau bacaannya terlalu kecil, mundur kalau terlalu besar.
    # Itu gerakan MAJU-MUNDUR, dan maju-mundur sekarang milik Teensy
    # sendirian -- lihat catatan di kepala S_A_MAJU.
    #
    # Ia juga memakan 4 detik dari jatah aksi, tepat sesudah pemicu datang dan
    # tepat sebelum penengahan. Empat detik itu bagian dari jeda yang
    # dikeluhkan operator sebagai "capit lama sekali turun".
    #
    # Standoff per korban tetap bisa disetel -- di TABEL MISI, lewat ruas
    # HNT_MUNDUR, bukan dari Pi. 'korban_belakang_cm' tinggal dipakai kartu
    # kalibrasi K-3/K-4 sebagai catatan angka.
    misi.o_diam = 0
    misi.pivot_gait_mati = False
    misi.ganti(S_A_TENGAH, f"ambil alih dari firmware, ruas {ruas}")
    link.log.append(
        f"[PEMICU] #KORBAN AMBIL ruas {ruas} -- AMBIL ALIH. Firmware parkir di "
        f"KONFIRM; Pi menengahkan, lalu menjawab 'm2' supaya Teensy melanjutkan "
        f"sekuens capit dan misinya."
        + ("" if kalib.auto_konfirm else
           " CATATAN: auto_konfirm MATI, jadi 'm2' harus kamu tekan sendiri."))
    return True


def langkah_fsm(misi: Misi, link: Teensy, aksi: Aksi, kalib: Kalib, armed,
                pid=None, lebar=1280, diam=None, kamera=None):
    """Satu langkah sub-FSM. Dipanggil tiap frame, tidak pernah memblokir."""
    # GERBANG PALING ATAS, sebelum apa pun -- termasuk sebelum batas waktu.
    # Kalau di bawah pemeriksaan batas waktu, robot yang di-STOP masih bisa
    # "gagal karena timeout" dan pindah state sendiri saat berhenti.
    if misi.halt or misi.jeda:
        return
    batas = BATAS.get(misi.state, 0)
    if batas and misi.lewat > batas and misi.state not in (S_GAGAL, S_IDLE, S_TUNGGU):
        # RANTAI FIRMWARE: JANGAN DIAM SAJA.
        #
        # Firmware sedang parkir menunggu 'm2'. Kalau kita cuma gagal tanpa
        # bersuara, ia menunggu sampai MISI_VISI_BATAS_MS (20 detik) habis lalu
        # menurunkan capit dengan pose apa adanya. Jadi diamnya kita TIDAK
        # menyelamatkan apa pun -- ia cuma menunda hasil yang sama selama 12
        # detik. Gejalanya persis yang dilaporkan R2C 14 Sep: "tengah di detik
        # 7, capit baru turun di detik 20".
        #
        # Menjawab sekarang memakai pose terbaik yang sempat dicapai tidak
        # pernah lebih buruk daripada membiarkannya habis, dan mengembalikan
        # 12 detik waktu kontes per korban.
        if misi.dari_firmware and misi.state in RANTAI_AMBIL:
            aksi.batal()
            serahkan_ke_firmware(
                misi, link, aksi, kalib, kamera,
                misi.bearing_deg if misi.bearing_deg is not None else 0.0,
                f"batas waktu {batas}s di {misi.state} -- diserahkan apa adanya")
            return
        # TAHAN_TENGAH YANG KEHABISAN WAKTU TIDAK BOLEH SEKADAR GAGAL.
        #
        # Di state ini Pi MEMEGANG KAKI. misi.gagal() cuma berhenti dan
        # bersuara -- badan tetap menyerong, 'm9' tidak pernah dikirim, dan
        # firmware melangkah sendiri sesudah MISI_LEPAS_BATAS_MS habis, di
        # atas pose yang tidak pernah dinetralkan. Jadi habisnya waktu di
        # sini artinya "lepas sekarang", bukan "menyerah".
        if misi.state == S_TAHAN:
            if misi.t_lepas is None:
                misi.t_lepas = time.time()
                link.log.append(
                    f"[TAHAN] '#LEPAS' tidak datang dalam {batas} detik. "
                    f"Sekuens capit firmware mestinya 12,1 detik sesudah 'm2' "
                    f"-- periksa apakah 'm2' benar terkirim. Kendali kaki "
                    f"DILEPAS sekarang supaya firmware tidak melangkah di "
                    f"atas pose yang masih ditahan.")
            # SENGAJA TIDAK return, dan sengaja tidak gagal: pelepasannya
            # dikerjakan blok S_TAHAN di bawah, dan blok itu perlu dipanggil
            # TIAP FRAME sampai antrean ('r0 0 0', 't0 0 0', 'm9') habis.
            # Return di sini membuat batas waktu memakan state-nya sendiri:
            # t_lepas terpasang, lalu tidak ada satu pun frame yang sampai ke
            # blok yang seharusnya mengirim 'm9'.
        else:
            misi.gagal(f"batas waktu {batas}s di state {misi.state}")
            aksi.batal()
            return

    if misi.state == S_IDLE:
        if misi.kini[2] == KORBAN:
            misi.ganti(S_TUNGGU)
        return

    if misi.state == S_TUNGGU:
        if "KONFIRMASI" in link.state_teensy().upper():
            misi.ganti(S_STANDOFF)
        return

    if misi.state == S_AWAS:
        # MENGAMATI saja. Tidak satu pun perintah gerak dikirim: firmware
        # sedang menjalankan ruasnya sendiri, dan menyelanya di tengah jalan
        # akan merusak lintasan yang sudah diukur. Vision di sini gunanya
        # melapor -- dan bersiap, supaya begitu Vincent memasang AKS_KONFIRM
        # jawabannya sudah ada tanpa menunggu 9 frame lagi.
        if not any(kata in link.ruas_fw().upper() for kata in RUAS_VISION):
            misi.ganti(S_IDLE, "ruas korban sudah lewat")
        return

    if misi.state == S_KONFIRM:
        # Firmware v1.9 berhenti di ruas ber-AKS_KONFIRM dan menunggu.
        # Penilaiannya dikerjakan blok vision di loop utama (state ini ada di
        # VISION_ON); di sini kita cuma menunggu Juri selesai lalu menjawab.
        # Kalau firmware SUDAH tidak menunggu lagi -- operator menekan m2/m3
        # sendiri, atau misi dibatalkan -- keluar diam-diam, jangan mengirim
        # jawaban untuk pertanyaan yang sudah lewat.
        if "KONFIRMASI" not in link.state_teensy().upper():
            misi.ganti(S_IDLE, "firmware tidak lagi menunggu konfirmasi")
        return

    if misi.state == S_TAHAN:
        # TIGA FASE, dan urutannya yang menjawab keluhan 14 Sep:
        #   1. capit masih bekerja  -> tahan tengah, koreksi halus saja
        #   2. capit sudah terangkat -> tahan lepas_tahan_s detik LAGI
        #   3. baru netralkan badan, kirim 'm9', lepas kendali
        #
        # Fase 1 tidak punya syarat henti sendiri: yang mengakhirinya '#LEPAS'
        # dari firmware (atau batas waktu di atas). Menebaknya dari jam --
        # "12,1 detik sesudah m2" -- akan meleset tiap kali loop firmware
        # tersendat, dan melesetnya ke arah yang salah: melepas kaki selagi
        # capit masih di bawah.
        if misi.t_lepas is None:
            tahan_tengah(misi, link, aksi, kalib, pid, lebar, diam)
            return

        sisa = kalib.lepas_tahan_s - (time.time() - misi.t_lepas)
        if sisa > 0:
            # Korban SUDAH terjepit di sini, jadi sedang_membawa() di dalam
            # tahan_tengah() praktis selalu mengunci koreksi. Itu memang yang
            # diinginkan: detik ini gunanya MENAHAN pose, bukan mengejar apa
            # pun. Yang penting badan belum kembali ke default.
            tahan_tengah(misi, link, aksi, kalib, pid, lebar, diam)
            return

        if misi._urut != S_TAHAN:
            misi._urut = S_TAHAN
            lepaskan_kaki(misi, link, aksi, kalib, kamera,
                          f"capit terangkat dan pose tengah ditahan "
                          f"{kalib.lepas_tahan_s:.1f} detik sesudahnya")
            return
        # Tunggu antrean ('r0 0 0', 't0 0 0', 'm9') benar-benar terkirim
        # sebelum pindah state. Pindah lebih awal membuat state berikutnya
        # menjadwalkan geraknya sendiri di atas antrean yang belum habis.
        if aksi.kosong():
            misi.dari_firmware = False
            misi.parkir_terlihat = False
            misi.t_tengah = None
            misi.tahan_s = 0.0
            misi.t_lepas = None
            misi._tahan_sebab = ""
            misi.ganti(S_IDLE, "kendali kaki dilepas ke firmware")
        return

    if misi.state == S_JEJAK:
        # Mengikuti sasaran terus-menerus. TIDAK pernah pindah state sendiri --
        # kapan maju dan kapan mencengkeram adalah keputusan operator, bukan
        # tebakan FSM. Batas waktu juga sengaja tidak dipasang.
        err = misi.bearing_deg
        if err is None:
            return
        if not sasaran_sah(misi, kalib, kalib.jejak_hanya_korban):
            # Dummy terlihat: DIAM. Tidak melangkah, tidak menengahkan diri,
            # tidak membuang tenaga. Sebelumnya robot mengejarnya sampai
            # tengah dulu baru menolak -- ongkosnya sudah terbayar duluan.
            return
        pusatkan(misi, aksi, kalib, err, pid, lebar, diam)
        return

    # ---- rantai AMBIL KORBAN: tengahkan -> maju -> capit --------------
    # Lima state kecil, bukan satu blok besar, supaya kalau berhenti kamu tahu
    # PERSIS di langkah mana dan kenapa. Semuanya lewat 'aksi' yang non-blokir,
    # jadi kamera dan halaman web tetap hidup selama urutan ini berjalan.

    # FIRMWARE SUDAH TIDAK PARKIR LAGI -> BERHENTI SEKETIKA.
    #
    # Gejala 13 Sep 2026: "robot bergerak ke kiri lalu melawan bergerak ke
    # kanan seolah disuruh pergi ke kanan". Memang disuruh -- oleh firmware.
    # Kalau batas parkir habis (atau operator menekan m2/m0), firmware
    # menjalankan sekuens lengannya lalu ruasBerikut(), dan ruas berikutnya
    # dimulai dengan PIVOT ke arah barunya: "Pivot MULAI menuju 176.8 der".
    # Kalau Pi masih mengirim 'O' untuk menengahkan di detik yang sama, dua
    # penguasa menarik satu robot ke arah berlawanan -- dan tidak ada satu pun
    # baris serial yang menyebutkan bahwa itu sedang terjadi.
    #
    # Deteksinya lewat TEPI, bukan lewat nilai: parkir harus pernah TERLIHAT
    # dulu, baru hilangnya berarti sesuatu. Tanpa itu, serah-terima yang
    # buktinya datang dari baris pengumuman (bukan dari poll 'm') akan
    # dibatalkan sendiri sebelum poll pertama sempat menjawab.
    if misi.dari_firmware and misi.state in RANTAI_AMBIL:
        if getattr(link, "parkir_visi", False):
            misi.parkir_terlihat = True
        elif misi.parkir_terlihat:
            misi.parkir_terlihat = False
            aksi.batal()
            link.log.append(
                "[SERAH GAGAL] firmware BERHENTI parkir sebelum penengahan "
                "selesai -- batas waktu 'm8' habis, atau ada yang mengirim "
                "m2/m0. Firmware sekarang menjalankan ruas berikutnya sendiri, "
                "jadi Pi BERHENTI mengirim perintah gerak: dua penguasa satu "
                "robot cuma menarik ke arah berlawanan. Kalau ini berulang, "
                "penengahannya yang terlalu lambat -- periksa baris 'pivot "
                "gait' dan 'tengah bertahan'.")
            misi.ganti(S_IDLE, "firmware tidak lagi menunggu vision")
            return

    if misi.state == S_A_TENGAH:
        err = misi.bearing_deg
        if err is None:
            return                        # belum terlihat; batas waktu yang menutup
        if (kalib.ambil_hanya_korban and misi.jejak_kelas
                and misi.jejak_kelas != kalib.kelas_korban):
            misi.gagal(f"sasaran terbaca '{misi.jejak_kelas}', bukan "
                       f"'{kalib.kelas_korban}' -- matikan ambil_hanya_korban "
                       f"kalau memang mau dicoba")
            aksi.batal()
            return
        # KASAR saja. Yang halus dikerjakan di S_A_HALUS.
        if abs(err) <= kalib.yaw_kasar_deg:
            if aksi.kosong():
                netralkan_badan(misi, aksi)   # tahap berikutnya mulai dari nol
                if pid is not None:
                    pid.reset()
                if diam is not None:
                    diam.reset()
                misi.tembak_err = misi.tembak_yaw = None
                if misi.dari_firmware:
                    # JALUR MISI: LANGSUNG ke penghalusan, S_A_MAJU DILEWATI.
                    #
                    # Ini bukan penghematan langkah, ini memperbaiki kesalahan
                    # kerangka acuan. Firmware sudah menghentikan robot pada
                    # HNT_DEPAN = KORBAN_JARAK_CM (24 cm), dan pose lengannya
                    # KORBAN_CAPIT_MM = 24*10 + LIDAR_DEPAN_MM = 302 mm
                    # DIHITUNG UNTUK ROBOT YANG BERDIRI DI 24 CM ITU.
                    #
                    # S_A_MAJU akan berjalan maju sampai LiDAR = capit_cm
                    # (10 cm). Kalau ia jalan di sini, Pi memajukan robot ~14 cm
                    # lalu firmware tetap menjulurkan lengan seolah masih di
                    # 24 cm -- capit lewat jauh di belakang korban, dan tidak
                    # ada satu pun pesan yang menyebutkan kenapa.
                    #
                    # Pembagian tugasnya juga jadi bersih, sesuai keputusan R2C
                    # 13 Sep: Raspi HANYA menengahkan. Turun-naiknya capit --
                    # dan seluruh jaraknya -- milik Teensy sendirian.
                    misi.ganti(S_A_HALUS, "jalur misi: centering saja, "
                                          "jarak & capit milik firmware")
                else:
                    misi.ganti(S_A_MAJU)
            return
        if aksi.kosong():
            aksi.jadwal((f"O{-pivot_der(err * kalib.pivot_gain, kalib):d}", 2.5),
                        ("l", 0.3))
        return

    if misi.state == S_A_HALUS:
        # Penengahan PRESISI, dikerjakan sesudah robot berhenti di jarak capit
        # dan TIDAK akan berjalan lagi. Di sinilah putar-badan dipakai: dia
        # bisa pecahan derajat, sedangkan pivot kaki mentok di 6 der = 2,1 cm.
        err = misi.bearing_deg
        if err is None:
            return
        # GERBANG KELAS, sama seperti AMBIL_TENGAH. Wajib sejak state ini
        # benar-benar menjejak hidup (14 Sep): selama bearing-nya beku, sasaran
        # tidak bisa berpindah, jadi pemeriksaan AMBIL_TENGAH sudah cukup.
        # Sekarang bisa -- dan berpindah ke dummy di detik terakhir berarti
        # 'm2' terkirim untuk boneka yang salah.
        if (kalib.ambil_hanya_korban and misi.jejak_kelas
                and misi.jejak_kelas != kalib.kelas_korban):
            misi.gagal(f"sasaran berpindah ke '{misi.jejak_kelas}' saat "
                       f"penghalusan -- 'm2' TIDAK dikirim")
            aksi.batal()
            return
        if pusatkan(misi, aksi, kalib, err, pid, lebar, diam) and aksi.kosong():
            # TINJAU JARAK SEKALI, sesudah badan diputar. Memutar badan
            # MENGGESER capit terhadap korban, jadi jarak yang diukur di
            # S_A_MAJU tadi sudah tidak berlaku lagi -- dan sebelum ini tidak
            # ada yang pernah memeriksanya. Gejalanya di arena: robot maju,
            # mundur sedikit, lalu hanya belok, dan jaraknya tidak pernah
            # dikoreksi kembali.
            #
            # SEKALI saja. Tanpa batas, badan dan jarak saling mengejar dan
            # rantai tidak pernah sampai ke capit.
            if not misi._jarak_ditinjau:
                misi._jarak_ditinjau = True
                d2 = link.depan_cm()
                if d2 is not None and d2 != float("inf"):
                    beda = d2 - jarak_ambil_cm(kalib)
                    if abs(beda) > kalib.capit_tol_cm:
                        link.log.append(
                            f"[TINJAU] sesudah putar badan jarak {d2:.0f} cm, "
                            f"meleset {beda:+.0f} cm. LAPORAN SAJA -- jarak "
                            f"milik Teensy, Pi tidak membetulkannya.")
            if misi.dari_firmware:
                # SERAH TERIMA BALIK. Rantai ini dimulai oleh firmware yang
                # sedang parkir di KONFIRM, jadi yang mengambil korban itu
                # SEKUENS FIRMWARE -- bukan rantai capit HUD. Dua sekuens
                # lengan untuk satu lengan hanya akan saling menimpa.
                serahkan_ke_firmware(misi, link, aksi, kalib, kamera, err,
                                     "penengahan selesai")
                return
            link.log.append(f"[HALUS] tengah dalam {abs(err):.1f} der "
                            f"(badan yaw {misi.badan_yaw:+.1f}) -- lanjut capit")
            misi.ganti(S_A_SIAP)
        return

    if misi.state == S_A_MAJU:
        if not aksi.kosong():
            return                        # tunggu langkah sebelumnya selesai
        d = link.depan_cm()
        if d is None:
            return                        # 'l' belum menjawab
        if d == float("inf"):
            misi.gagal("depan 'jauh' -- korban tidak terlihat LiDAR, "
                       "jangan maju membabi buta")
            return
        if d > kalib.ambil_maks_cm:
            # v1.7 menaikkan LIDAR_MAX_CM 70 -> 130, jadi dinding sejauh 1 meter
            # kini terbaca sebagai ANGKA, bukan lagi "jauh". Tanpa pagar ini,
            # rantai AMBIL akan dengan patuh berjalan menyeberangi ruangan 6 cm
            # sekali jalan menuju dinding yang disangkanya korban.
            misi.gagal(f"depan {d:.0f} cm, lebih jauh dari batas ambil "
                       f"{kalib.ambil_maks_cm:.0f} cm -- itu kemungkinan besar "
                       f"dinding, bukan korban")
            return
        # SILANG VISION vs LiDAR. Ini pemeriksaan yang paling berharga di
        # seluruh rantai, dan alasannya spesifik: LiDAR depan punya PITA HANTU
        # di 3,2 dan 9 cm yang Vincent dokumentasikan, dan target capit kita
        # 10 cm -- tepat di sebelahnya. Kalau LiDAR sedang berhalusinasi, satu-
        # satunya cara tahu adalah sensor KEDUA yang salahnya tidak berkorelasi.
        #
        # Kamera memberi itu, dan di pita ini dia justru teliti: pada 25 cm,
        # 5 piksel noise tinggi bbox = 3 mm.
        #
        # Kalau keduanya berselisih jauh, SALAH SATU bohong dan HUD tidak punya
        # cara memilih. Jadi robot berhenti dan bilang, bukan menebak sensor
        # mana yang benar lalu menutup capit di udara -- atau di muka korban.
        #
        # Tapi ia hanya boleh MENGHENTIKAN misi kalau bacaan vision-nya memang
        # sudah dikalibrasi. Selama tinggi_k1..k5 masih 0, angka kamera itu
        # taksiran dari tinggi nominal 9 cm, dan menghentikan pendekatan yang
        # sehat karena taksiran adalah persis kekacauan yang dilaporkan.
        # Belum dikalibrasi -> cukup dicatat di log, misi lanjut.
        misi.beda_vis_lidar = None
        if (kalib.vision_silang_on and misi.jarak_vis is not None
                and sasaran_sah(misi, kalib, kalib.ambil_hanya_korban)):
            beda = misi.jarak_vis - d
            misi.beda_vis_lidar = beda
            sudah_kalib = any(getattr(kalib, f"tinggi_k{i}", 0.0) > 0.0
                              for i in range(1, 6))
            if abs(beda) > kalib.vision_lidar_beda_maks_cm and not sudah_kalib:
                if not getattr(misi, "_silang_diberitahu", False):
                    misi._silang_diberitahu = True
                    link.log.append(
                        f"[SILANG] LiDAR {d:.0f} cm vs kamera "
                        f"{misi.jarak_vis:.0f} cm (beda {beda:+.0f}). TIDAK "
                        f"dijadikan kegagalan: jarak vision belum dikalibrasi "
                        f"(tinggi_k1..k5 masih 0). Misi lanjut pakai LiDAR.")
            elif abs(beda) > kalib.vision_lidar_beda_maks_cm:
                misi.gagal(
                    f"LiDAR {d:.0f} cm tapi kamera {misi.jarak_vis:.0f} cm "
                    f"(beda {beda:+.0f} cm, batas "
                    f"{kalib.vision_lidar_beda_maks_cm:.0f}). Salah satu salah "
                    f"-- di dekat 10 cm tersangkanya pita hantu LiDAR. "
                    f"Periksa 'l', atau kalibrasi ulang jarak vision.")
                return
        sasar = jarak_ambil_cm(kalib)
        delta = d - sasar
        # PAGAR BBOX TERPOTONG. Kamera menolak mengukur di bawah ~12 cm karena
        # bbox-nya menyentuh tepi frame. Di sanalah d = f*H/h berhenti tumbuh
        # dan terbaca LEBIH JAUH daripada kenyataan -- dan LiDAR depan bisa
        # ikut setuju "masih jauh", karena di ceruk 40x15 ia memandang dinding
        # belakang, bukan boneka 15 cm di depannya. Dua sensor sepakat,
        # dua-duanya salah, lalu siku mengayun 110 der menembus korban.
        #
        # Sejak 17 Sep 2026 penolakan itu cuma DICATAT. Pi tidak lagi mundur
        # -- lihat catatan di bawah.
        dekat_paksa = vis_terlalu_dekat(misi)
        if dekat_paksa and not getattr(misi, "_dekat_diberitahu", False):
            misi._dekat_diberitahu = True
            link.log.append(
                f"[DEKAT] kamera menolak mengukur ({misi.jarak_vis_sebab}). "
                f"LiDAR bilang {d:.0f} cm, tapi bbox sebesar itu hanya muncul "
                f"kalau korban jauh lebih dekat. LAPORAN SAJA -- jarak milik "
                f"Teensy.")

        # MAJU-MUNDUR DIBUANG SELURUHNYA, 17 Sep 2026, diminta R2C sesudah
        # trial arena: "tiap Raspi dapat kendali, maju mundur tidak konsisten".
        #
        # Sebabnya struktural, bukan tuning. Firmware menghentikan robot pada
        # HNT_DEPAN = KORBAN_JARAK_CM, dan pose lengannya DIHITUNG untuk robot
        # yang berdiri di jarak itu. Setiap sentimeter yang Pi tambahkan atau
        # kurangi sesudahnya menggeser korban terhadap amplop lengan yang
        # firmware kira tidak berubah. Dua penguasa untuk satu sumbu.
        #
        # Jadi sumbu maju-mundur sekarang milik Teensy SENDIRIAN. Pi hanya
        # menengahkan: 't<x> 0 0' (geser badan) dan 'O<der>' (pivot). Tidak
        # ada 'w', tidak ada 'D', tidak ada 'J'.
        #
        # Selisih jaraknya tetap diukur dan dicatat -- itu yang memberi tahu
        # Vincent ke mana KORBAN_JARAK_CM atau ruas HNT_MUNDUR harus disetel.
        # Ia tidak pernah lagi menghentikan pengambilan.
        if abs(delta) > kalib.capit_tol_cm:
            link.log.append(
                f"[JARAK] LiDAR {d:.0f} cm, sasaran capit Pi {sasar:.0f} cm "
                f"(meleset {delta:+.0f} cm). TIDAK dikoreksi -- maju-mundur "
                f"milik Teensy. Setel KORBAN_JARAK_CM atau ruas HNT_MUNDUR.")
        misi.ganti(S_A_HALUS)
        return

    if misi.state == S_A_SIAP:
        if misi._urut != S_A_SIAP:
            r, h = pose_jepit_mm(kalib)
            # Capit DIBUKA lebih dulu, baru lengan menjulur. Urutan terbalik
            # akan menyodok korban dengan capit tertutup dan menggesernya --
            # lalu seluruh penengahan yang barusan dikerjakan jadi sia-sia.
            #
            # Jatah bukanya DIHITUNG. Angka tetap yang lama (0,8 detik) cukup
            # saat slew lengan 120 der/detik, tapi tidak lagi pada 72: rahang
            # g10 -> g50 menempuh 72 der, yaitu 1000 ms. Pada 0,8 detik lengan
            # mulai menjulur sementara rahang masih setengah menutup, yaitu
            # urutan terbalik yang baru saja dihindari di atas.
            #
            # Lebar rahang saat masuk state ini TIDAK diketahui -- tidak ada
            # sensor, dan operator bisa meninggalkannya di mana saja dalam
            # 10..95 (rentang clamp 'g' firmware). Yang dipakai perjalanan
            # TERPANJANG dari kedua ujung rentang itu.
            buka = kalib.capit_buka_persen
            t_buka = max(tunggu_capit_s(kalib, 10.0, buka),
                         tunggu_capit_s(kalib, 95.0, buka))
            aksi.jadwal((f"g{buka:.0f}", t_buka),
                        (f"a{r:.0f} {h:.0f}", 2.0))
            misi._urut = S_A_SIAP
        elif aksi.kosong():
            misi.ganti(S_A_JEPIT)
        return

    if misi.state == S_A_JEPIT:
        if misi._urut != S_A_JEPIT:
            # g10, bukan g0. Pada 0-5% capit menabrak dirinya sendiri:
            # servo terus mendorong ke posisi yang sudah tertahan mekanis,
            # arus stall mengalir terus, dan motornya panas (laporan R2C,
            # 12 Sep 2026). Firmware R2C juga menaikkan GRIP_PERSEN_MIN ke 10
            # supaya 'g0' yang diketik manual ikut ter-clamp -- perbaikan di
            # sini saja akan meninggalkan jalur manual terbuka.
            #
            # TUNGGUNYA waktu-menutup + capit_diam_s, DAN ITU SATU TUNGGU,
            # BUKAN DUA. Antrean aksi tidak punya perintah "diam" -- tiap
            # entri mengirim sesuatu -- jadi jendela diamnya dinyatakan
            # sebagai ekor tunggu milik 'g10' itu sendiri. Efeknya identik
            # dengan fase DIAM firmware (KORBAN_JEPIT_DIAM_MS): tidak ada satu
            # pun perintah lahir di detik itu.
            #
            # Bagian pertama menutupi rahang menutup TANPA BEBAN (g50 -> g10 =
            # 72 der, 1000 ms pada slew 72 der/detik, plus capit_margin_s);
            # capit_diam_s menutupi selisih bebannya. Angkat baru dijadwalkan
            # sesudah keduanya habis -- itulah "menutup dulu, baru mengangkat".
            aksi.jadwal(("g10", tunggu_capit_s(kalib, kalib.capit_buka_persen, 10.0)
                                + kalib.capit_diam_s))
            misi._urut = S_A_JEPIT
        elif aksi.kosong():
            misi.ganti(S_A_ANGKAT)
        return

    if misi.state == S_A_ANGKAT:
        if misi._urut != S_A_ANGKAT:
            r, h = pose_jepit_mm(kalib)
            aksi.jadwal((f"a{r:.0f} {h + kalib.lengan_angkat:.0f}", 2.0))
            misi._urut = S_A_ANGKAT
        elif aksi.kosong():
            # Tidak ada sensor cengkeraman. HUD TIDAK bisa tahu korbannya
            # benar-benar terpegang -- operator yang melihat dan memutuskan.
            link.log.append("[AMBIL] selesai -- PERIKSA MATA: korban terpegang?")
            misi.ganti(S_BERES)
        return

    if misi.state == S_STANDOFF:
        if misi.tanpa_gerak:
            misi.ganti(S_CENTER)          # pose apa adanya, operator yang menaruh
            return
        d = link.depan_cm()
        if d is None:
            return
        if d == float("inf"):
            misi.gagal("depan 'jauh' -- boneka tidak terlihat LiDAR di pose ini")
            return
        delta = d - kalib.standoff_cm
        # MAJU-MUNDUR MILIK TEENSY, 17 Sep 2026. State ini dulu menutup
        # jaraknya sendiri dengan 'D' + 'w'. Sekarang ia cuma melapor, lalu
        # lanjut ke penengahan -- satu-satunya sumbu yang masih milik Pi.
        # Operator yang memutuskan jaraknya, lewat tabel misi atau tombol
        # 'Maju' di tab Manual.
        if abs(delta) > kalib.standoff_tol_cm:
            link.log.append(
                f"[JARAK] depan {d:.0f} cm, standoff {kalib.standoff_cm:.0f} cm "
                f"(meleset {delta:+.0f} cm). TIDAK dikoreksi -- maju-mundur "
                f"milik Teensy.")
        misi.ganti(S_CENTER)
        return

    if misi.state == S_CENTER:
        if misi.tanpa_gerak:
            misi.ganti(S_LIHAT)
            return
        err = misi.bearing_deg
        if err is None:
            return
        if not sasaran_sah(misi, kalib, kalib.jejak_hanya_korban):
            return
        if pusatkan(misi, aksi, kalib, err, pid, lebar, diam):
            misi.ganti(S_LIHAT)
        return

    if misi.state == S_SLOT:
        if misi.tanpa_gerak:
            return          # operator menggeser robot sendiri, lalu tekan lanjut
        # GESER SLOT DIMATIKAN, 17 Sep 2026. Caranya dulu: hadap TIMUR ('o1'),
        # berjalan MAJU 8 cm ('D' + 'w'), hadap BARAT lagi ('o3'). Berjalan
        # maju sesudah memutar badan tetaplah 'w' -- dan 'w' sekarang milik
        # Teensy sendirian.
        #
        # Tanpa geser, pemindaian slot cuma memakai pose yang ada. Operator
        # yang memindahkan robot ke slot berikutnya kalau memang perlu.
        if not getattr(misi, "_geser_dijadwal", False):
            link.log.append(
                f"[SLOT] geser {kalib.slot_step_cm:.0f} cm DILEWATI -- ia "
                f"berjalan maju, dan maju-mundur milik Teensy. Pindahkan robot "
                f"dengan tangan atau lewat tabel misi, lalu tekan lanjut.")
            misi._geser_dijadwal = True
            misi.slot += 1
        elif aksi.kosong():
            misi._geser_dijadwal = False
            misi.ganti(S_STANDOFF)
        return

    if misi.state == S_PUTUS:
        pilih = pilih_slot(misi, kalib)
        if pilih is None:
            misi.gagal("tidak ada slot yang bisa dipilih")
            return
        misi.terkunci = pilih
        if not misi.tanpa_gerak:
            aksi.jadwal(("m2" if pilih[1] == kalib.kelas_korban else "m3", 0.5))
        misi.ganti(S_DEKATI)
        return

    if misi.state in BELUM_ADA:
        # Belum diprogram. Sengaja berhenti, bukan pura-pura jalan -- tapi
        # halaman SEKARANG mengatakannya (kolom state jadi kuning + "BELUM
        # DIPROGRAM") supaya tidak ada yang menunggu sia-sia.
        return

    if misi.state == S_BERES:
        misi.maju_misi(SELESAI)
        return


def pilih_slot(misi: Misi, kalib: Kalib):
    """Prior guidebook: TEPAT SATU boneka asli per ruang.

    Jadi jangan berhenti di slot pertama yang bilang 'korban'. Nilai semua,
    ambil margin terbesar. Kalau dua slot yakin dummy, slot ketiga adalah
    korban walau conf-nya sendiri sedang-sedang saja -- eliminasi gratis, dan
    justru paling menolong di K-3/K-4 yang tertimpa papan.
    """
    if not misi.hasil_slot:
        return None
    kandidat = [(i, k, m, c) for i, (k, m, c) in misi.hasil_slot.items()]
    korban = [x for x in kandidat if x[1] == kalib.kelas_korban]
    if korban:
        pilih = max(korban, key=lambda x: x[2])
        return (pilih[0], pilih[1], pilih[2])
    pilih = max(kandidat, key=lambda x: x[2])
    return (pilih[0], kalib.kelas_korban, pilih[2])


# =====================================================================
# 8. GAMBAR DI ATAS FRAME (untuk stream video)
# ---------------------------------------------------------------------
# Panel teks TIDAK lagi digambar di frame -- itu pekerjaan halaman web.
# Yang tinggal di frame hanya yang harus menempel pada piksel: kotak
# deteksi dan gerbang ROI.
# =====================================================================
# Warna dalam BGR (urutan OpenCV, bukan RGB). Dipilih supaya terbaca di
# stream JPEG yang sudah diperkecil 0,5x dan dikompres -- warna pucat hilang
# duluan saat dikompres, jadi keduanya sengaja jenuh dan berlawanan.
HIJAU = (80, 255, 80)        # KORBAN  -- hijau terang
MERAH = (60, 80, 255)        # DUMMY   -- merah-oranye
KUNING = (60, 220, 255)      # kelas tak dikenal
ABU = (120, 120, 120)        # ditolak gerbang
FONT = cv2.FONT_HERSHEY_SIMPLEX


def warna_kelas(nama, kalib):
    if nama == kalib.kelas_korban:
        return HIJAU
    if nama == kalib.kelas_dummy:
        return MERAH
    return KUNING


def _label(frame, x, y, teks, warna, skala=0.62):
    """Teks hitam di atas kotak berwarna penuh -- terbaca di latar apa pun."""
    (tw, th), _ = cv2.getTextSize(teks, FONT, skala, 2)
    y = max(th + 10, y)
    cv2.rectangle(frame, (x, y - th - 10), (x + tw + 12, y), warna, -1)
    cv2.putText(frame, teks, (x + 6, y - 6), FONT, skala, (0, 0, 0), 2, cv2.LINE_AA)


def anotasi(frame, dets, lolos, kalib, names, jejak=False, alasan=None):
    h, w = frame.shape[:2]
    cx0 = int(w / 2 + kalib.cx_offset_px)
    batas = int(kalib.roi_half_px(w))

    # Garis tengah selalu; kotak gerbang ROI hanya saat gerbang itu dipakai.
    cv2.line(frame, (cx0, 0), (cx0, h), (90, 90, 90), 1)
    if not jejak:
        cv2.rectangle(frame, (cx0 - batas, 0), (cx0 + batas, h - 1), (110, 110, 110), 1)

    kunci_lolos = {(l[0], l[1]): l for l in lolos}
    for x1, y1, x2, y2, score, cls in dets:
        nama = names.get(cls, str(cls))
        diterima = (x1, y1) in kunci_lolos
        warna = warna_kelas(nama, kalib) if diterima else ABU
        cv2.rectangle(frame, (x1, y1), (x2, y2), warna, 3 if diterima else 1)
        if diterima:
            teks = f"{nama.upper()} {score:.2f}"
        else:
            sebab = (alasan or {}).get((x1, y1), "disaring")
            teks = f"{nama} {score:.2f} -- {sebab}"
        _label(frame, x1, y1, teks, warna, 0.62 if diterima else 0.45)

    # Saat menjejak: panah dari sumbu kamera ke sasaran, supaya arah koreksi
    # terlihat tanpa harus membaca angka.
    if jejak and lolos:
        t = max(lolos, key=lambda d: (d[3] - d[1]))
        tx, ty = int(t[6]), int((t[1] + t[3]) / 2)
        cv2.arrowedLine(frame, (cx0, ty), (tx, ty),
                        warna_kelas(names.get(t[5], ""), kalib), 3, tipLength=0.2)

    # Legenda kecil, supaya tidak perlu menghafal warnanya.
    _label(frame, 8, 30, "KORBAN", HIJAU, 0.5)
    _label(frame, 120, 30, "DUMMY", MERAH, 0.5)
    _label(frame, 228, 30, "ditolak", ABU, 0.5)
    return frame


# =====================================================================
# 9. HALAMAN WEB
# =====================================================================
HALAMAN = r"""<!doctype html>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>R2C Mission HUD</title>
<style>
:root{--bg:#0d1013;--panel:#151a1f;--line:#242c33;--txt:#dfe6ea;--dim:#8a969e;
      --ok:#57e08a;--warn:#ffc857;--bad:#ff5f56;--acc:#4fc3f7}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);
     font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{padding:7px 14px;font-weight:700;letter-spacing:.5px;border-bottom:1px solid var(--line);
       display:flex;justify-content:space-between;align-items:center}
#mode{padding:7px 14px;font-weight:700;text-align:center;letter-spacing:1px}
.baca{background:#14361f;color:var(--ok)}.kendali{background:#5b1111;color:#fff}
#galat{display:none;padding:8px 14px;background:#3a0d0d;color:#ffb4b0;
       border-bottom:1px solid var(--bad);white-space:pre-wrap;font-size:11px}
/* Strip status: SELALU terlihat, tidak pernah sembunyi di balik tab. */
#strip{display:flex;flex-wrap:wrap;gap:14px;padding:7px 14px;background:#11161b;
       border-bottom:1px solid var(--line);font-size:12px}
#strip b{font-weight:700}
.wrap{display:grid;grid-template-columns:minmax(320px,560px) 1fr;gap:10px;padding:10px;align-items:start}
@media(max-width:920px){.wrap{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px;margin-bottom:10px}
.card h2{margin:0 0 8px;font-size:11px;letter-spacing:1.5px;color:var(--acc);text-transform:uppercase}
img#cam{width:100%;border-radius:8px;border:1px solid var(--line);display:block;background:#000}
table{width:100%;border-collapse:collapse}td{padding:2px 4px;vertical-align:top}
.mid{color:var(--dim)}.big{font-size:19px;font-weight:700}
ul{margin:0;padding:0}li{list-style:none}
.m{display:flex;gap:6px;padding:2px 4px;border-radius:4px;font-size:12px}
.m.now{background:#243040;color:var(--warn);font-weight:700}
.m.done{color:var(--ok)}.m.fail{color:var(--bad)}.m.skip{color:var(--dim)}.m.todo{color:#6d7a83}
.m .id{width:62px;flex:none}
button{background:#1e262d;color:var(--txt);border:1px solid var(--line);border-radius:6px;
       padding:7px 10px;font:inherit;cursor:pointer;margin:2px 2px 0 0}
button:hover{border-color:var(--acc)}
button.danger{border-color:#733}
/* Tab: satu baris tombol, isi bergantian. Tidak ada yang perlu di-scroll jauh. */
#tabs{display:flex;flex-wrap:wrap;gap:4px;margin-bottom:8px}
#tabs button{margin:0;border-radius:6px 6px 0 0;border-bottom-color:transparent}
#tabs button.aktif{background:#243040;color:var(--warn);border-color:var(--acc)}
.panel{display:none}.panel.aktif{display:block}
#log{height:190px;overflow:auto;font-size:11px;color:#7f9c86;white-space:pre-wrap}
.gagal{border:1px solid var(--bad);background:#2a0f0f}
input{background:#0e1317;color:var(--txt);border:1px solid var(--line);
      border-radius:4px;padding:3px 5px;width:76px;font:inherit}
input.lebar{width:170px}
[data-tip]{position:relative}
[data-tip]:hover::after{content:attr(data-tip);position:absolute;left:0;top:calc(100% + 6px);
  z-index:50;width:300px;padding:8px 10px;background:#0a0e12;color:var(--txt);
  border:1px solid var(--acc);border-radius:6px;font-size:11px;line-height:1.5;
  white-space:normal;box-shadow:0 6px 18px #000a}
</style>
<header><span>R2C HEXAPOD &mdash; MISSION HUD <span class=mid id=host></span></span>
        <span class=mid id=versi></span></header>
<div id=mode class=kendali>&nbsp;</div>
<div id=galat></div>
<div id=strip>
  <span>state <b id=state>-</b></span>
  <span class=mid>&rarr; <span id=next></span></span>
  <span>serial <b id=port>-</b></span>
  <span>kamera <b id=kamera>-</b></span>
  <span>teensy <b id=teensy>-</b></span>
  <span>depan <b id=depan>-</b></span>
  <span>vision <b id=vis>-</b></span>
  <span class=mid id=fps></span>
  <span>serial <b id=rx>-</b></span>
  <span>daya <b id=sehat>-</b></span>
  <span>penonton <b id=penonton>-</b></span>
  <span id=umur></span>
</div>
<div id=henti hidden style="background:#511;border:2px solid #d55;border-radius:6px;
     padding:10px 14px;margin:8px 0;font-weight:700;font-size:15px">
  <span id=henti_teks></span>
  <button style="margin-left:14px" onclick="cmd('lepas_stop')">Lepas STOP</button>
  <button onclick="cmd('resume')">Lanjutkan (resume)</button>
</div>

<div class=wrap>
  <div>
    <img id=cam src="/stream.mjpg" alt="video kamera">
    <div class=card style="margin-top:10px"><h2>Log serial
      <button style="float:right;font-size:11px" data-tip="RINGKAS: jawaban poll otomatis (m dan l tiap detik) tidak dicatat -- datanya TETAP diparse, cuma barisnya disembunyikan. PENUH: semuanya dicatat, termasuk ~20 baris tiap detik dari tabel LiDAR dan STATUS MISI. Perintah yang KAMU tekan selalu terlihat di kedua mode." onclick="cmd('log_ringkas')">Ringkas / Penuh</button></h2>
      <div class=kecil id=tombol_info></div>
      <div class=kecil id=log_info></div><div id=log></div></div>
    <div class=card style="margin-top:10px"><h2>Riwayat perintah
      <button style="float:right;font-size:11px" data-tip="Kosongkan riwayat. Log serial di atas tidak tersentuh." onclick="cmd('riwayat_hapus')">Hapus</button></h2>
      <p class=kecil>Perintah yang KAMU kirim, beserta jawabannya &mdash; disimpan
      <b>terpisah</b> dari log serial. Log utama memuat 200 baris; satu aliran yaw
      pada 100 ms menghabiskannya dalam 20 detik, jadi percobaan tiga menit lalu
      sudah hilang persis saat kamu ingin membandingkannya. Daftar ini tidak bisa
      tenggelam: baris aliran rutin tidak pernah masuk ke sini.</p>
      <div id=riwayat class=kecil></div></div>
    <div class=card style="margin-top:10px"><h2>Hentikan banjir serial</h2>
      <p class=kecil><b><code>s</code> tidak menghentikan cetakan.</b> Ia
      menghentikan <b>gerak</b>. Aliran cetak dinyalakan <code>y</code> (yaw) dan
      <code>L</code> (LiDAR), dan satu-satunya yang mematikannya huruf yang sama
      lagi &mdash; itulah kenapa log bisa terus membanjir walau robotnya sudah
      diam.</p>
      <div id=aliran class=kecil></div>
      <button data-tip="Matikan SEMUA aliran cetak firmware yang sedang hidup. Bukan toggle buta: hanya yang keadaannya terbaca HIDUP yang dikirimi huruf pematiannya -- mengirim 'y' ke aliran yang sudah mati justru MENYALAKANNYA." onclick="cmd('hening')">HENING &mdash; matikan semua aliran</button>
      <button data-tip="Hidup/matikan poll HUD sendiri (m + l tiap detik). Kalau dimatikan, jarak dan status BERHENTI diperbarui -- kartu-kartunya membeku di angka terakhir. Pakai saat menyetel lewat serial, lalu nyalakan lagi." onclick="cmd('poll')">Poll HUD: <span id=poll_st>?</span></button></div>
  </div>

  <div>
    <div id=tabs>
      <button class=aktif onclick="tab(0,this)">Robot</button>
      <button onclick="tab(1,this)">Misi</button>
      <button onclick="tab(2,this)">Korban</button>
      <button onclick="tab(3,this)">Manual</button>
      <button onclick="tab(4,this)">Kalibrasi</button>
    </div>

    <!-- 0. ROBOT -->
    <div class="panel aktif">
      <div class=card><h2>Kendali robot</h2>
        <button data-tip="Firmware: b. Menghitung pose berdiri DULU baru menyalakan PWM. WAJIB sebelum perintah gerak apa pun -- firmware boot dengan servo lemas." onclick="cmd('berdiri')">Berdiri &amp; nyalakan servo (b)</button>
        <button data-tip="Firmware: m1. Pivot ke UTARA, catat titik nol jarak dari dinding START, lalu susur dinding KANAN terkunci kompas sampai berhenti di samping K-1." onclick="cmd('mulai')">Mulai misi (m1)</button>
        <button class=danger data-tip="STOP KERAS. Mengirim s, mengosongkan antrean, DAN mengunci seluruh otomasi -- FSM tidak akan menjadwalkan apa pun lagi sampai kamu tekan 'Lepas STOP'. Servo tetap hidup (yang mematikan servo cuma LEMAS). Tombol gerak lain ikut ditolak selama terkunci." onclick="cmd('stop')">STOP semua (s)</button>
        <button data-tip="Melepas kunci STOP. Sesudah ini FSM dan tombol gerak boleh dipakai lagi. Tidak melanjutkan apa pun sendiri -- kamu yang memilih mau mulai dari mana." onclick="cmd('lepas_stop')">Lepas STOP</button>
        <button class=danger data-tip="Firmware: x. Semua PWM mati, robot AMBRUK. Satu-satunya tombol yang menembus whitelist." onclick="if(confirm('LEMAS: semua servo mati, robot AMBRUK. Yakin?'))cmd('lemas')">LEMAS darurat (x)</button>
        <hr>
        <button class=danger data-tip="Menjalankan ulang mission_hud.py dari awal tanpa SSH. Kamera dan serial dibuka ulang, kalibrasi RAM kembali ke default. Halaman memuat sendiri sesudah 6 detik." onclick="restartApp()">Restart program</button>
      </div>
      <div class=card><h2>Kendali misi</h2>
        <p class=kecil><b>PAUSE</b> membekukan di tempat dan <b>menyimpan state</b>,
        jadi bisa dilanjutkan. <b>STOP</b> mengunci semuanya. <b>ABORT</b> membatalkan
        misi di kedua sisi &mdash; HUD dan firmware.</p>
        <button data-tip="Bekukan di tempat: kirim s, kosongkan antrean, FSM berhenti menjadwalkan. State DISIMPAN, jam batas waktunya ikut ditahan. Robot tidak bergerak sampai Resume." onclick="cmd('pause')">Pause</button>
        <button data-tip="Lanjutkan dari state yang dibekukan. Jam batas waktu state di-nolkan supaya jeda panjang tidak langsung terbaca sebagai timeout." onclick="cmd('resume')">Resume</button>
        <button class=danger data-tip="Batalkan misi SELURUHNYA: kirim m0 (batalkan Mission di firmware) lalu s, tandai misi ini GAGAL, kosongkan hasil slot, kembali ke IDLE, dan KUNCI otomasi. m0 wajib -- tanpa itu Mission firmware masih hidup dan navUpdate() menyalakan navigasi lagi sendiri." onclick="if(confirm('ABORT: misi ditandai GAGAL dan otomasi dikunci. Yakin?'))cmd('abort')">Abort mission</button>
        <table>
          <tr><td class=mid width=80>kunci</td><td id=kunci_st>-</td></tr>
        </table>
      </div>
      <div class=card><h2>Daya &amp; suhu Pi</h2>
        <table>
          <tr><td class=mid width=95>status</td><td id=p_status>-</td></tr>
          <tr><td class=mid>tegangan 5V</td><td id=p_volt>-</td></tr>
          <tr><td class=mid>konsumsi</td><td id=p_daya>-</td></tr>
          <tr><td class=mid>arus total</td><td id=p_arus>-</td></tr>
          <tr><td class=mid>suhu</td><td id=p_suhu>-</td></tr>
          <tr><td class=mid>over-current</td><td id=p_oc>-</td></tr>
          <tr><td class=mid>bit throttle</td><td id=p_bit>-</td></tr>
        </table>
        <canvas id=p_graf width=520 height=64
                style="width:100%;height:64px;margin-top:8px;border-radius:4px"></canvas>
        <div class=kecil id=p_graf_ket>tegangan (garis) &amp; konsumsi (isian), ~3 menit terakhir</div>
        <p class=kecil><b>Jangan terlalu percaya angka voltnya.</b> ADC PMIC dibaca
        beberapa kali per detik; brownout yang menjatuhkan Pi berlangsung 10 milidetik
        dan <b>tidak akan pernah</b> muncul di situ &mdash; sama seperti multimeter yang
        merata-rata. Yang benar-benar menangkapnya cuma baris <b>bit throttle</b>:
        perangkat kerasnya sendiri yang melatch, dan bit "pernah terjadi" menempel
        sampai reboot. Kalau volt terlihat adem tapi "pernah under-voltage" menyala,
        percaya bitnya.</p>
      </div>
    </div>

    <!-- 1. MISI -->
    <div class=panel>
      <div class=card><h2>Kamera &amp; vision &mdash; saklar daya</h2>
        <p class=kecil>Mematikan di sini <b>benar-benar melepas perangkat
        kameranya</b>, bukan cuma berhenti menggambar kotak. Itu bedanya dengan
        <b>mode FOKUS</b>, yang berhenti mengirim video tapi kamera dan
        inferensi tetap jalan.</p>
        <p class=kecil>Webcam C922 menarik daya terus-menerus selama terbuka,
        dan di robot ini daya bukan urusan kecil: tegangan 4,9 V sudah cukup
        membuat USB macet dan bit throttle Pi melatch. Jadi kamera yang tidak
        sedang dibutuhkan memang lebih baik mati.</p>
        <button data-tip="Nyalakan/matikan kamera. MATI = perangkatnya dilepas (thread.release()), stream kosong, vision tidak jalan, daya hemat. NYALA = menyambung ulang dalam beberapa detik." onclick="cmd('kamera')">Kamera: <span id=kam_st>?</span></button>
        <button data-tip="MODE FOKUS: berhenti mengirim video dan berhenti menggambar kotak, tapi kamera & inferensi TETAP jalan. Untuk menyetel penengahan tanpa ongkos encode JPEG -- bukan untuk menghemat daya." onclick="cmd('fokus')">Mode FOKUS video</button>
        <p class=kecil style="color:var(--warn)">Yang dituju akhirnya: kamera
        menyala <b>hanya saat Teensy memintanya</b> (pemicu <code>#KORBAN</code>)
        lalu mati sendiri sesudah korban diambil. Untuk sekarang masih manual,
        karena monitoring masih dibutuhkan &mdash; saklar ini dulu.</p>
      </div>
      <div class=card id=stbox><h2>Keadaan sekarang</h2>
        <p class=kecil id=stdesc>-</p>
        <table>
          <tr><td class=mid width=90>ruang</td><td id=lewat>-</td></tr>
          <tr><td class=mid>slot</td><td id=slot>-</td></tr>
        </table>
        <div id=gagalbox></div>
      </div>
      <div class=card><h2>Jalankan misi</h2>
        <p class=kecil>Misi <b>sudah bisa dijalankan dari sini</b> sejak dulu &mdash;
        tombol "Mulai misi (m1)" di kartu Robot. Tidak perlu colok laptop ke Teensy.</p>
        <p class=kecil style="color:var(--warn)"><b>Tiga syarat, dan firmware
        menolak kalau salah satu belum:</b><br>
        1. servo menyala (<code>b</code>) &nbsp; 2. kompas arena lengkap
        (<code>k</code> harus menunjukkan keempat arah) &nbsp; 3. pivot terkalibrasi
        (<code>K</code>).<br>
        Ketiganya butuh <b>IMU hidup</b>. Selama IMU mati, <code>m1</code> akan
        ditolak dengan pesan yang jelas &mdash; bukan diam.</p>
        <p class=kecil style="color:var(--bad)"><b>Urutan penolakannya, dari
        v1.12 yang sudah dibaca baris per baris:</b><br>
        1. <code>Navigation.cpp:617</code> &mdash; <code>!_imu.hasData()</code>
        &rarr; <b>"Gagal: tidak ada data IMU."</b> Ini yang <b>pertama</b>
        menghentikanmu selama IMU belum hidup, bukan ruas 30.<br>
        2. kompas lengkap &mdash; ini <b>tidak</b> butuh IMU, cuma membaca
        <code>_headArah[]</code> yang sudah dimuat dari EEPROM saat boot.
        Lihat kartu Kalibrasi kompas: kalau keempatnya hijau, syarat ini
        <b>sudah lolos</b>.<br>
        3. pivot terkalibrasi &mdash; juga dari EEPROM, juga tidak butuh IMU
        untuk diperiksa.<br>
        4. <code>tabelSiap()</code> &mdash; ruas 30. Yang terakhir, dan yang
        paling mudah dilewati (<code>m4 0 29</code>).</p>
        <button data-tip="Cetak status misi: state, ruas ke berapa dari 0..33, nama ruasnya, dan sisa syarat. Aman." onclick="cmd('man','m')">Status misi (m)</button>
        <button data-tip="Mulai misi dari ruas 0. Firmware memeriksa servo, mux, LiDAR, IMU, kompas dan kalibrasi pivot lebih dulu, lalu mencetak persis mana yang gagal." onclick="if(confirm('Mulai misi dari ruas 0. Robot AKAN berjalan. Lanjut?'))cmd('mulai')">Mulai misi (m1)</button>
        <button class=danger data-tip="Batalkan misi di firmware. Robot berhenti, navigasi mati." onclick="cmd('man','m0')">Batalkan misi (m0)</button>
        <div class=mid style="margin-top:8px">Jalan SEBAGIAN &mdash; melewati ruas yang belum diukur</div>
        <p class=kecil><code>m4 &lt;awal&gt; &lt;akhir&gt;</code> menjalankan misi
        pada RENTANG ruas, dan <code>tabelSiap()</code> cuma memeriksa rentang
        itu. Jadi ruas 30 yang belum diukur <b>tidak lagi menghalangi</b> selama
        ia di luar rentang &mdash; <code>m4 0 29</code> menempuh HOME sampai K-5,
        yaitu hampir seluruh misi.</p>
        <div style="margin-top:6px">
          <input id=m4_a style="width:52px" value="0">
          <input id=m4_b style="width:52px" value="29">
          <button class=danger data-tip="m4 <awal> <akhir>: jalankan rentang ruas itu saja. Robot AKAN berjalan." onclick="if(confirm('Jalankan misi ruas '+$('m4_a').value+'..'+$('m4_b').value+'. Robot AKAN berjalan. Lanjut?'))cmd('man','m4 '+$('m4_a').value+' '+$('m4_b').value)">Jalankan rentang (m4)</button>
        </div>
        <table>
          <tr><td class=mid width=90>ruas firmware</td><td id=ruas_fw>-</td></tr>
          <tr><td class=mid>vision</td><td id=awas>-</td></tr>
        </table>
      </div>
      <div class=card><h2>Panjang ruas &mdash; kenapa misi menolak berangkat</h2>
        <p class=kecil><b>"Gagal: masih ada ruas yang panjangnya BELUM DIUKUR"</b>
        itu <b>bukan bug</b>, itu penjaga <code>tabelSiap()</code>. Panjang tiap ruas
        harus diukur di arena sungguhan; yang belum diisi ditandai <code>-1</code> di
        tabel, dan firmware menolak berangkat daripada berjalan menebak.</p>
        <p class=kecil style="color:var(--warn)">Di <b>v1.12</b> masih tetap
        <b>SATU</b> yang kosong: <b>ruas 30 &mdash; "R-11 longsor (lebar jalan 30)"</b>
        (<code>Misi.cpp:559</code>, nilainya masih <code>-1</code>). Isi lewat m7,
        <b>atau</b> pakai <code>m4 0 29</code> di kartu Jalankan misi untuk
        melewatinya sama sekali.</p>
        <button data-tip="Cetak SELURUH tabel lintasan 0..32. Kolom cm bertanda '?' = belum diukur. Aman, hanya membaca." onclick="cmd('man','m4')">Lihat tabel (m4)</button>
        <div style="margin-top:8px">
          <input id=ruas_i style="width:56px" placeholder="ruas" value="30">
          <input id=ruas_cm style="width:66px" placeholder="cm">
          <button data-tip="Kirim m7 <ruas> <cm>: setel panjang ruas itu. Berlaku di RAM firmware, TIDAK menulis EEPROM -- jadi hilang kalau Teensy mati. Catat angkanya." onclick="setRuas()">Setel panjang (m7)</button>
        </div>
        <p class=kecil style="margin-top:6px"><b>Belum tahu angkanya?</b> MODE UKUR:
        robot berjalan di ruas itu <b>tanpa syarat henti</b> sambil mencetak berapa cm
        yang sudah ditempuh. Kamu yang menghentikannya dengan <b>STOP</b> di ujung ruas,
        lalu catat angkanya dan masukkan lewat m7 di atas.</p>
        <button class=danger data-tip="m6 <ruas>: MODE UKUR. Robot BERJALAN dan TIDAK akan berhenti sendiri -- kamu yang menyetop. Siapkan tangan di STOP." onclick="if(confirm('MODE UKUR: robot berjalan dan TIDAK berhenti sendiri. Siap?'))cmd('man','m6 '+($('ruas_i').value||'30'))">Mode ukur ruas ini (m6)</button>
      </div>
      <div class=card><h2>Urutan misi</h2>
        <p class=kecil>Ini <b>rencana lomba</b>, bukan daftar kemampuan. HUD
        <b>tidak punya kode navigasi</b> &mdash; berpindah ruang, menyeberang lantai
        pecah, menuruni bidang miring, semuanya milik <code>Mission.cpp</code> di
        Teensy. Baris bertanda <span style="color:var(--warn)">firmware belum ada</span>
        cuma bisa dilewati sebagai catatan; robotnya harus kamu pindahkan sendiri.</p>
        <ul id=misi></ul></div>
    </div>

    <!-- 2. KORBAN -->
    <div class=panel>
      <div class=card><h2>Jejak korban</h2>
        <button data-tip="Kamera menyala terus. Boneka boleh digeser ke mana saja; robot memutar badan mengikutinya sampai simpangan di bawah toleransi, lalu berhenti menunggu perintahmu. Gerbang ROI dimatikan dan pita tinggi dilonggarkan, jadi jarak bebas." onclick="cmd('jejak')">Mulai JEJAK</button>
        <button data-tip="Sekali tekan: baca jarak LiDAR depan, hitung selisih ke standoff_cm, kirim D<selisih> lalu w. Rem jarak firmware yang menghentikannya." onclick="cmd('maju')">Maju ke korban</button>
        <button data-tip="Firmware: g100. Membuka penjepit lengan DEPAN penuh." onclick="cmd('capit_buka')">Capit BUKA</button>
        <button data-tip="Firmware: g10. Menutup penjepit lengan DEPAN sampai 10%, BUKAN 0. Pada 0-5% capit menabrak dirinya sendiri dan servo stall terus -- motornya panas. Firmware R2C juga men-clamp g0 ke 10." onclick="cmd('capit_tutup')">Capit TUTUP (g10)</button>
        <button data-tip="Firmware: n. Mematikan servo KEDUA lengan. Kaki tidak terpengaruh." onclick="cmd('lengan_off')">Lengan OFF</button>
        <div class=mid>Sendi lengan DEPAN &mdash; bahu &amp; siku dipecahkan firmware dari jangkauan/tinggi, pergelangan sudut langsung. Angkanya ikut kalibrasi <code>lengan_r</code>/<code>lengan_h</code>/<code>lengan_angkat</code>.</div>
        <button data-tip="a<r> <h> dengan tinggi = lengan_h + lengan_angkat. Lengan terangkat bebas, capit belum turun -- pose aman untuk mendekat." onclick="cmd('lengan_pos','siap')">Lengan SIAP</button>
        <button data-tip="a<r> <h> pada lengan_h. Ini tinggi menjepit: capit turun setinggi korban." onclick="cmd('lengan_pos','jepit')">Lengan TURUN (jepit)</button>
        <button data-tip="a<r> <h> dengan tinggi = lengan_h + 2x lengan_angkat. Mengangkat korban lepas dari lantai." onclick="cmd('lengan_pos','angkat')">Lengan ANGKAT</button>
        <input id=prg_der style="width:56px" placeholder="prg&deg;" value="0">
        <button data-tip="Argumen ketiga 'a': sudut pergelangan -90..90 der. Jangkauan &amp; tinggi ikut dikirim ulang, karena tanpa itu firmware membiarkan pergelangan di sudut lama." onclick="cmd('pergelangan',$('prg_der').value)">Setel pergelangan</button>
        <div class=mid>Capit BELAKANG &mdash; huruf BESAR. <b>Grip saja, tidak punya sendi</b>: firmware menolak <code>A&lt;r&gt; &lt;h&gt;</code> dengan pesan sendiri, letaknya ditentukan letak BADAN. Tabel misi juga tidak memakainya sama sekali (10 ruas korban semuanya ARM_DEPAN), jadi ini murni manual.</div>
        <button data-tip="Firmware: G100. Membuka penjepit lengan BELAKANG penuh." onclick="cmd('man','G100')">Capit BLK buka (G100)</button>
        <button data-tip="Firmware: G10. Menutup penjepit lengan BELAKANG sampai 10%, bukan 0 -- alasan yang sama dengan capit depan." onclick="cmd('man','G10')">Capit BLK tutup (G10)</button>
        <table>
          <tr><td class=mid width=80>sasaran</td><td id=sasaran>-</td></tr>
          <tr><td class=mid>simpangan</td><td id=bearing>-</td></tr>
          <tr><td class=mid>deteksi</td><td id=deteksi>-</td></tr>
          <tr><td class=mid>diabaikan</td><td id=dummy_saja>-</td></tr>
        </table>
      </div>
      <div class=card><h2>Ambil korban (otomatis)</h2>
        <p class=kecil>Lima langkah berurutan, tiap langkah punya batas waktu sendiri.
        Kalau berhenti, kolom <b>sebab</b> di tab Misi menyebut langkah mana yang gagal.
        Robot BERGERAK MAJU di langkah 2 &mdash; siapkan tangan di tombol LEMAS.</p>
        <button data-tip="1) tengahkan badan ke korban dengan kamera, 2) maju bertahap maksimal 6 cm sekali jalan sampai jarak LiDAR = capit_cm, 3) buka capit dan turunkan lengan, 4) tutup capit, 5) angkat. Butuh mode KENDALI." onclick="if(confirm('AMBIL OTOMATIS: robot akan berputar lalu MAJU ke korban. Lanjut?'))cmd('ambil')">Mulai AMBIL otomatis</button>
        <button class=danger data-tip="Kosongkan antrean perintah, kirim s (stop, servo tetap hidup), kembali ke IDLE." onclick="cmd('ambil_batal')">Batalkan AMBIL</button>
        <table>
          <tr><td class=mid width=80>langkah</td><td id=ambil_langkah>-</td></tr>
          <tr><td class=mid>jarak LiDAR</td><td id=ambil_depan>-</td></tr>
          <tr><td class=mid>jarak kamera</td><td id=jarak_vis>-</td></tr>
          <tr><td class=mid>beda</td><td id=beda_vis>-</td></tr>
          <tr><td class=mid>posisi kalib</td><td id=vis_posisi>-</td></tr>
          <tr><td class=mid>rasio bbox</td><td id=vis_rasio>-</td></tr>
          <tr><td class=mid>bbox diharap</td><td id=vis_harap>-</td></tr>
          <tr><td class=mid>capit membawa</td><td id=membawa>-</td></tr>
          <tr><td class=mid>tunggu vision</td><td id=tunggu_visi>-</td></tr>
          <tr><td class=mid>tengah bertahan</td><td id=tahan>-</td></tr>
          <tr><td class=mid>pivot gait</td><td id=pivot_gait>-</td></tr>
          <tr><td class=mid>target capit</td><td id=ambil_target>-</td></tr>
          <tr><td class=mid>pose badan</td><td id=ambil_badan>-</td></tr>
          <tr><td class=mid>geometri lengan</td><td id=lengan_geo>-</td></tr>
          <tr><td class=mid>pemicu firmware</td><td id=pemicu>-</td></tr>
          <tr><td class=mid>yang mengambil</td><td id=serah>-</td></tr>
        </table>
        <div class=mid style="margin-top:8px">Pemicu dari firmware &mdash; knob
          <code>pemicu_korban</code></div>
        <p class=kecil>Firmware <b>v1.12</b> mencetak <code>#KORBAN AMBIL
        &lt;ruas&gt;</code> begitu berhenti di depan korban. HUD membaca baris
        itu. Tapi Vincent menulisnya <b>kirim-lalu-lanjut</b> &mdash; sebaris di
        bawahnya Teensy langsung menjalankan sekuens capitnya sendiri, jadi ia
        <b>tidak sedang menunggu siapa pun</b>.</p>
        <p class=kecil><b>lihat</b> (bawaan): vision menilai, <b>nol</b> perintah
        gerak. Aman dengan v1.12 apa adanya.<br>
        <b>ambil_alih</b>: Pi menengahkan lalu menjawab <code>m2</code>. Ini
        hanya benar-benar jalan kalau firmware <b>terbukti parkir</b> (mencetak
        MENUNGGU KONFIRMASI); tanpa bukti itu HUD turun sendiri ke
        <b>lihat</b> dan menyebut sebabnya &mdash; karena menggerakkan badan
        selagi sekuens lengan berjalan berarti dua penguasa untuk satu robot,
        dan yang kalah korban yang tersenggol capit yang sedang turun.<br>
        <b>mati</b>: abaikan pemicunya.</p>
        <div class=mid style="margin-top:8px">Cara mengambil &mdash; knob
          <code>mode_ambil</code> di tab Kalibrasi</div>
        <p class=kecil><b>lengan</b> (bawaan): robot berhenti di
        <b>25 cm</b> lalu MENJULURKAN lengan &mdash; tidak melangkah lagi.
        <b>badan</b>: robot berjalan sampai korban di depan capit (cara lama,
        10 cm). Pindah ke "lengan" memindahkan titik kerja dari tempat kedua
        sensor paling lemah ke tempat keduanya paling kuat: di 25 cm bbox masih
        utuh (kamera teliti ~3 mm) dan LiDAR jauh dari pita hantunya.</p>
        <button data-tip="Firmware m8 (butuh v1.12+patch): balik mode tunggu-vision. NYALA = ruas AMBIL berhenti menunggu m2 dari Pi, jadi vision punya jendela untuk meluruskan badan sebelum capit turun. MATI = capit turun segera sesudah #KORBAN (perilaku v1.12 asli)." onclick="cmd('tunggu_visi')">Balik tunggu-vision (m8)</button>
        <p class=kecil style="color:var(--warn)"><b>Inilah yang hilang saat diuji
        12 Sep.</b> Pemicu <code>#KORBAN</code> sampai, vision mendeteksi, tapi
        firmware <b>tidak menunggu</b> &mdash; jadi capit turun sendiri dan
        satu-satunya yang boleh dikerjakan HUD adalah MENONTON. Urutan yang
        benar: <code>m8</code> (nyalakan) &rarr; <code>pemicu_korban =
        ambil_alih</code> di tab Kalibrasi &rarr; jalankan misi.</p>
        <button data-tip="Kirim pose jepit ke lengan DEPAN sekarang, tanpa menjalankan rantai AMBIL. Perhatikan log: kalau firmware menjawab 'Di luar jangkauan lengan', berarti 25 cm melebihi IK lengan dan sudutnya TIDAK dikirim -- itu jawaban yang kita cari." onclick="cmd('uji_jangkau')">Uji jangkauan lengan (tanpa capit)</button>
      </div>
      <div class=card><h2>Uji berurutan</h2>
        <button data-tip="Menilai satu boneka di depan kamera: 9 frame, gerbang ROI + pita tinggi bbox, lalu putuskan korban/dummy. NOL perintah gerak dikirim, m2/m3 juga tidak." onclick="cmd('ujiv')">Vision saja (tanpa gerak)</button>
        <button data-tip="SLOT = satu posisi berhenti di depan SATU boneka. '1 slot' = nilai boneka di depan saja, tanpa bergeser. 'GERAK' = robot menutup jarak ke 20 cm dengan LiDAR lalu memutar badan menengahkan boneka, baru menilai." onclick="cmd('uji1')">1 slot + gerak</button>
        <button data-tip="Meniru K-1 sungguhan: 2 dummy + 1 korban berjajar 8 cm. Nilai slot 1, geser 8 cm, nilai slot 2, geser lagi, nilai slot 3, lalu pilih margin terbesar. Gesernya lewat heading arena (o1/o3) jadi kompas harus terkalibrasi." onclick="cmd('uji3')">3 slot + gerak</button>
        <div class=mid style="margin-top:6px" id=hint></div>
      </div>
      <div class=card><h2>Penilaian slot</h2>
        <div id=slots></div>
        <div style="margin-top:8px">
          <button data-tip="Paksa masuk state LIHAT sekarang: ambil 9 frame di pose apa adanya lalu putuskan." onclick="cmd('lihat')">Nilai slot ini</button>
          <button data-tip="Lompat ke SUB-STATE berikutnya di dalam misi yang sama. Saat GAGAL, tombol ini menandai misi LEWAT lalu maju. Kalau tujuannya BELUM DIPROGRAM, log akan bilang begitu -- di situ pakai 'Lewati misi ini'." onclick="cmd('next')">State berikutnya</button>
          <button data-tip="Lompati SELURUH misi yang sedang jalan, tandai LEWAT, lanjut ke misi berikutnya di tabel. Ini jalan keluar dari rantai DEKATI/CENGKERAM/VERIF/ANTAR/LETAK yang belum diprogram -- di sana 'State berikutnya' cuma berpindah dari satu state diam ke state diam berikutnya." onclick="cmd('lewati')">Lewati misi ini</button>
          <button class=danger data-tip="Menandai misi yang sedang jalan sebagai gagal dan menghentikan antrean perintah." onclick="cmd('gagal')">Tandai GAGAL</button>
          <button data-tip="Mengosongkan kumpulan frame yang sudah dinilai dan hitungan tolakan. Tidak menyentuh robot." onclick="cmd('reset')">Reset suara</button>
          <span class=mid id=slotbtn></span>
        </div>
      </div>
    </div>

    <!-- 3. MANUAL -->
    <div class=panel>
      <div class=card><h2>Serial &mdash; kirim apa saja</h2>
        <div style="display:flex;gap:6px">
          <input class=lebar id=cmdbox style="flex:1" autocomplete=off
                 placeholder="perintah firmware, mis.  m  |  b60  |  O-15  |  a80 160 -40  |  R"
                 onkeydown="if(event.key==='Enter')kirimManual()">
          <button data-tip="Kirim apa adanya ke Teensy. TIDAK ADA FILTER sejak 13 Sep 2026: whitelist MODE BACA dan daftar TERLARANG sudah dibuang. Jawaban firmware muncul di log serial di kiri." onclick="kirimManual()">Kirim</button>
        </div>
        <div class=mid style="margin-top:6px">Tanpa filter &mdash; apa pun yang
          kamu ketik sampai ke Teensy apa adanya, dan jawabannya kembali utuh.
          <b style="color:var(--bad)">x</b> melemaskan servo (robot ambruk),
          <b style="color:var(--bad)">S W e</b> menulis EEPROM.</div>
      </div>
      <div class=card><h2>Kirim perintah langsung</h2>
        <button data-tip="Berjalan maju sampai ditekan Stop. Firmware: w. Butuh servo menyala (b)." onclick="cmd('man','w')">Maju (w)</button>
        <button data-tip="Firmware: s. Berhenti, servo tetap hidup." onclick="cmd('man','s')">Stop (s)</button>
        <button data-tip="Putar 15 derajat ke KIRI di tempat. Firmware: O15." onclick="cmd('man','O15')">&#8634; 15&deg;</button>
        <button data-tip="Putar 15 derajat ke KANAN. Firmware: O-15." onclick="cmd('man','O-15')">&#8635; 15&deg;</button>
        <button data-tip="Putar 90 derajat ke KIRI. Firmware: O90." onclick="cmd('man','O90')">&#8634; 90&deg;</button>
        <button data-tip="Putar 90 derajat ke KANAN. Firmware: O-90." onclick="cmd('man','O-90')">&#8635; 90&deg;</button>
        <div class=mid>Putar sekian derajat &mdash; isi sendiri angkanya</div>
        <div style="margin-bottom:6px">
          <input id=putar_der style="width:60px" value="35"
                 onkeydown="if(event.key==='Enter')putar(1)">
          <span class=kecil>derajat</span>
          <select id=putar_cara style="margin:0 4px">
            <option value="auto">auto</option>
            <option value="badan">badan (r) &mdash; tanpa IMU, maks 18&deg;</option>
            <option value="gait">gait (O) &mdash; butuh IMU</option>
          </select>
          <button data-tip="Putar KIRI sebanyak derajat di kotak. Tanda + = kiri, mengikuti firmware (yaw+ = belok KIRI)." onclick="putar(1)">&#8634; KIRI</button>
          <button data-tip="Putar KANAN sebanyak derajat di kotak. Dikirim sebagai sudut negatif." onclick="putar(-1)">&#8635; KANAN</button>
          <button data-tip="r0 0 0 + t0 0 0. Kembalikan badan ke netral. WAJIB sebelum meminta putaran badan lagi ke arah yang sama -- 'r' itu perintah POSISI, bukan langkah." onclick="cmd('pose_nol')">Nolkan pose</button>
        </div>
        <p class=kecil><b>auto</b> memilih <b>badan</b> selama sudutnya masih muat,
        karena selagi IMU mati itulah satu-satunya rotasi yang benar-benar jalan.
        <b>badan</b> memutar badan di atas kaki yang tetap menapak &mdash; tidak
        butuh IMU, tapi firmware membatasi <code>BODY_MAX_ROT_DEG = 20&deg;</code>
        dan ini perintah <b>POSISI</b>: minta 18&deg; dua kali tidak menghasilkan
        36&deg;, yang kedua tidak menggerakkan apa pun (HUD akan bilang begitu,
        bukan diam saja). <b>gait</b> melangkah, sudut berapa pun, tapi
        <code>pivotRelatif()</code> kembali <b>diam-diam</b> tanpa data IMU &mdash;
        tidak ada pesan error, gejalanya sama persis dengan kabel putus.</p>
        <br>
        <button data-tip="Hadap UTARA menurut kompas arena. Firmware: o0. Butuh kompas sudah dicatat (c0..c3 lalu e)." onclick="cmd('man','o0')">Hadap U</button>
        <button data-tip="Hadap TIMUR. Firmware: o1." onclick="cmd('man','o1')">T</button>
        <button data-tip="Hadap SELATAN. Firmware: o2." onclick="cmd('man','o2')">S</button>
        <button data-tip="Hadap BARAT. Firmware: o3." onclick="cmd('man','o3')">B</button>
        <div class=mid>Susur dinding &mdash; navigasi otonom firmware</div>
        <p class=kecil>Keempatnya dulu <b>diblokir whitelist HUD</b>, jadi dari web
        jawabannya cuma penolakan &mdash; itu sebabnya "fiturnya tidak jalan".
        Alasan lamanya ("robot langsung berjalan") tidak konsisten: <code>w</code>
        juga membuat robot berjalan dan selalu boleh. Sekarang keempatnya
        diperlakukan seperti <code>w</code>: butuh MODE KENDALI dan dikonfirmasi.
        <b>Robot AKAN berjalan sampai kamu tekan STOP.</b></p>
        <p class=kecil><code>f</code>/<code>F</code> murni sensor sisi.
        <code>p</code>/<code>P</code> menambah kunci kompas arena &mdash; keduanya
        butuh <b>IMU hidup</b> dan kompas lengkap, jadi selama IMU mati pakai
        <code>f</code>/<code>F</code>.</p>
        <button class=danger data-tip="Firmware: f. Susur dinding KIRI dengan sensor sisi. Robot berjalan sampai ditekan Stop." onclick="if(confirm('Susur dinding KIRI. Robot AKAN berjalan sampai kamu tekan STOP. Lanjut?'))cmd('man','f')">Susur KIRI (f)</button>
        <button class=danger data-tip="Firmware: F. Susur dinding KANAN dengan sensor sisi. Robot berjalan sampai ditekan Stop." onclick="if(confirm('Susur dinding KANAN. Robot AKAN berjalan sampai kamu tekan STOP. Lanjut?'))cmd('man','F')">Susur KANAN (F)</button>
        <button class=danger data-tip="Firmware: p. Susur dinding KIRI + terkunci heading arena. BUTUH IMU hidup dan kompas lengkap." onclick="if(confirm('Susur KIRI + kunci arena. Butuh IMU. Robot AKAN berjalan. Lanjut?'))cmd('man','p')">KIRI + arena (p)</button>
        <button class=danger data-tip="Firmware: P. Susur dinding KANAN + terkunci heading arena. BUTUH IMU hidup dan kompas lengkap." onclick="if(confirm('Susur KANAN + kunci arena. Butuh IMU. Robot AKAN berjalan. Lanjut?'))cmd('man','P')">KANAN + arena (P)</button>
        <button data-tip="Firmware: C. Kalibrasi arah putar, ~10 detik dan MEMBLOKIR loop firmware selama itu (log berhenti sebentar -- itu normal). Butuh IMU hidup dan servo menyala." onclick="if(confirm('Kalibrasi pivot: robot BERPUTAR ~10 detik dan log firmware berhenti selama itu. Lanjut?'))cmd('man','C')">Kalibrasi pivot (C)</button>
        <br>
        <button data-tip="Status navigasi + jarak sekitar. Aman, hanya membaca. Firmware: v." onclick="cmd('man','v')">Status nav (v)</button>
        <button data-tip="Tabel jarak keenam LiDAR. Aman. Firmware: l." onclick="cmd('man','l')">LiDAR (l)</button>
        <button data-tip="Dump diagnostik: status PWM, ServoMap, sudut & pulse per kaki. Aman. Firmware: d." onclick="cmd('man','d')">Diagnostik (d)</button>
        <p class=kecil style="color:var(--warn)"><b>Awas huruf O vs angka 0.</b>
        Perintah belok itu huruf <b>O</b> besar. Kalau kamu mengetik <b>angka nol</b>
        (<code>015</code>), firmware membaca perintah <code>0</code> = "nolkan pose badan",
        mencetak <code>Pose badan dinolkan.</code> lalu tidak bergerak sama sekali &mdash;
        gejalanya <b>persis sama</b> dengan IMU mati. Periksa log serialnya dulu.</p>
        <div class=mid>Diagnosa IMU &mdash; SATU sebab untuk hampir semua penolakan</div>
        <p class=kecil style="color:var(--bad)">Ini yang menghentikan
        <code>C</code> (kalibrasi pivot), <code>c0..c3</code> (catat kompas),
        <code>O</code>, <code>o0..o3</code>, <b>dan</b> <code>m1</code>/<code>m4</code>
        &mdash; semuanya memanggil <code>_imu.hasData()</code> lebih dulu.
        Menyetel yang lain tidak akan menolong sampai ini hidup.</p>
        <p class=kecil><b>Tersangka utama: baud.</b> <code>Imu::begin()</code> cuma
        MEMBUKA port &mdash; ia tidak pernah mengirim apa pun ke modul, tidak ada
        perintah pindah baud. Jadi semuanya bergantung pada modul yang SUDAH ada di
        baud yang sama. Komentar Vincent sendiri di <code>Imu.cpp:19</code> berbunyi
        <i>"Pastikan IMU_BAUD di config.h sudah diubah ke 230400"</i> &mdash; kata
        <b>diubah</b> itu petunjuknya: angka aslinya bukan 230400, dan modul WIT
        keluar pabrik pada <b>9600</b>. Modul di 9600 + Teensy di 230400 = senyap
        total tanpa satu pun pesan error.</p>
        <p class=kecil style="color:var(--warn)"><b>Cara membuktikannya hari ini,
        tanpa alat tambahan:</b> colok <b>Type-C</b> modul ke Pi. Aku pernah bilang
        itu mungkin colokan daya &mdash; kemungkinan besar <b>keliru</b>; di Yahboom
        10-axis itu biasanya antarmuka USB-serial (CH340/CP2102). Lalu di Pi:
        <code>python3 ~/M/sidik_imu.py</code>. Ia memindai 9 baud dan menghitung
        frame WIT yang sah. Ketemu = modul hidup, tinggal angkanya. Tidak ketemu di
        baud mana pun = berhenti menyetel angka, masalahnya kabel atau daya.</p>
        <div class=mid>Yang bisa dicek dari sini</div>
        <button data-tip="Tabel lintasan lengkap. Firmware BARU (lebih baru dari v1.8) punya tabel ruas 0..31 -- ini yang mencetaknya. Aman, hanya membaca." onclick="cmd('man','m4')">Tabel lintasan (m4)</button>
        <button data-tip="Daftar SELURUH perintah yang dikenali firmware yang SEDANG terpasang. Ini cara paling cepat tahu apakah sudah ada perintah geser samping / mundur. Aman." onclick="cmd('man','h')">Daftar perintah (h)</button>
        <button data-tip="Nyala/mati aliran cetak yaw. Aman, hanya mencetak. UJI PALING MENENTUKAN: kalau tidak ada satu pun baris yaw muncul, IMU mati -- dan SEMUA pivot (O, o0..o3, mode arena, m1) tidak akan pernah jalan. Tekan lagi untuk mematikan." onclick="cmd('man','y')">Aliran yaw (y) &mdash; uji IMU</button>
        <button class=danger data-tip="Pivot ke UTARA. BUKAN sekadar diagnosa -- robot AKAN berputar kalau IMU hidup. Gunanya: saat IMU MATI perintah ini MENCETAK 'Gagal: Tidak ada data IMU.', sedangkan O tidak mencetak apa pun sama sekali." onclick="if(confirm('o0 akan MEMUTAR robot kalau IMU hidup. Lanjut?'))cmd('man','o0')">o0 &mdash; paksa pesan error IMU</button>
        <div class=mid style="margin-top:6px">Kotak kirim serialnya ada di
          kartu paling atas tab ini. Arahkan kursor ke tombol untuk penjelasannya.</div>
      </div>
      <div class=card><h2>Firmware v1.7 / v1.8 saja</h2>
        <p class=kecil>Tombol di bawah <b>tidak ada</b> di v1.61 &mdash; kalau Teensy
        masih pakai firmware lama, jawabannya cuma "perintah tidak dikenal".
        Semuanya boleh ditukar <b>sambil robot berjalan</b>.</p>
        <div class=mid>Profil medan &mdash; mengubah tinggi &amp; panjang langkah, di-ramp halus</div>
        <button data-tip="Cetak profil medan yang SEDANG berlaku: tinggi langkah, panjang langkah, waktu siklus, tinggi badan, radius kaki. Aman, hanya membaca. Firmware: T." onclick="cmd('man','T')">Profil? (T)</button>
        <button data-tip="Profil DATAR. Ini profil bawaan yang juga dipakai tombol Berdiri. Firmware: T0." onclick="cmd('man','T0')">T0 datar</button>
        <button data-tip="Profil TANGGA: langkah lebih tinggi untuk menyeberangi lantai pecah. Firmware: T1." onclick="cmd('man','T1')">T1 tangga</button>
        <button data-tip="Profil MERUNDUK: badan lebih rendah untuk turunan/bidang miring. Firmware: T2." onclick="cmd('man','T2')">T2 merunduk</button>
        <button data-tip="Profil SEMPIT: radius kaki dikecilkan untuk lorong sempit. Firmware: T3." onclick="cmd('man','T3')">T3 sempit</button>
        <div class=mid>Kemudi dinding &mdash; PD vs fuzzy, dan sumber turunannya</div>
        <button data-tip="Cetak mode kemudi yang sedang dipakai: PD atau SAMAR (fuzzy), turunan dari selisih waktu atau dari SUDUT. Aman, hanya membaca. Firmware: N." onclick="cmd('man','N')">Kemudi? (N)</button>
        <button data-tip="PD + turunan dari selisih waktu. Bawaan di v1.7. Di v1.8 BUKAN lagi bawaan -- pakai ini untuk kembali ke perilaku lama kalau sepasang sensor sisi bermasalah. Firmware: N0." onclick="cmd('man','N0')">N0 PD/waktu</button>
        <button data-tip="Fuzzy (Sugeno orde-0, 9 aturan) + turunan dari selisih waktu. Firmware: N1." onclick="cmd('man','N1')">N1 fuzzy/waktu</button>
        <button data-tip="PD + turunan dihitung dari SUDUT badan terhadap dinding (sepasang sensor sisi yang sama). INI BAWAAN v1.8: goyang perintah kemudi turun dari 4,4/detik ke 0,16/detik karena pembaginya dasar 11 cm, bukan selang sampel 25 ms. Firmware: N2." onclick="cmd('man','N2')">N2 PD/sudut</button>
        <button data-tip="Fuzzy + turunan dari SUDUT. Ini kombinasi terbaru yang mau diuji Vincent. Firmware: N3." onclick="cmd('man','N3')">N3 fuzzy/sudut</button>
        <div class=mid>Kemudi lateral, sudut dinding, sensor depan</div>
        <button data-tip="Cetak keadaan kemudi lateral sekarang. Aman. Firmware: Z." onclick="cmd('man','Z')">Lateral? (Z)</button>
        <button data-tip="MENENGAH lorong: pakai selisih kiri-kanan, robot mencari tengah. Firmware: Z1." onclick="cmd('man','Z1')">Z1 menengah</button>
        <button data-tip="IKUT DINDING satu sisi saja, seperti v1.61. Firmware: Z0." onclick="cmd('man','Z0')">Z0 ikut dinding</button>
        <button data-tip="Tabel sudut badan terhadap dinding kiri dan kanan. Aman, hanya membaca. Firmware: Y." onclick="cmd('man','Y')">Sudut dinding (Y)</button>
        <button data-tip="Catat bias pemasangan sensor. Robot HARUS sedang sejajar lorong saat ditekan, kalau tidak biasnya justru salah. RAM saja, hilang tiap Teensy mati. Firmware: Y0." onclick="if(confirm('Robot sudah SEJAJAR lorong? Kalau belum, biasnya jadi salah.'))cmd('man','Y0')">Y0 catat bias</button>
        <button data-tip="Cetak keadaan sensor depan. Aman. Firmware: i." onclick="cmd('man','i')">Sensor depan? (i)</button>
        <div class=mid>v1.8 &mdash; ruas terakhir di bawah turunan</div>
        <button data-tip="Cetak status misi, termasuk baris 'ruas akhir' yang baru di v1.8. Aman. Firmware: m." onclick="cmd('man','m')">Status misi (m)</button>
        <button data-tip="Ambang sensor DEPAN untuk ruas terakhir di bawah turunan, bawaan 40 cm. Ketik di kotak bebas: m5 &lt;cm&gt;, misal m5 40. Harus lebih besar dari FRONT_STOP_CM (20)." onclick="cmd('man','m5 40')">m5 40 (ambang ruas akhir)</button>
        <button class=danger data-tip="ABAIKAN sensor depan -- robot jadi buta ke depan. Hanya untuk turunan, di mana lantai yang menjauh terbaca sebagai halangan. JANGAN lupa i0 sesudahnya. Firmware: i1." onclick="cmd('man','i1')">i1 abaikan depan</button>
        <button data-tip="Pakai lagi sensor depan. Firmware: i0." onclick="cmd('man','i0')">i0 pakai depan</button>
      </div>
    </div>

    <!-- 4. KALIBRASI -->
    <div class=panel>
      <div class=card><h2>Kalibrasi vision</h2>
        <p class=kecil>Semua knob-nya ada di tab <b>Kalibrasi</b> dan berlaku
        <b>seketika</b> tanpa restart. Yang belum ada cuma urutannya &mdash; ini dia.</p>
        <p class=kecil style="color:var(--warn)"><b>Revert 12 Sep.</b> Pemilihan
        sasaran di mode JEJAK/CENTER dikembalikan ke <b>versi 10 September</b>:
        yang TERBESAR di antara yang lolos saring, titik. Tiga gerbang yang
        ditambahkan 12 Sep dicabut dari jalur itu, karena mereka membuang sasaran
        <b>sesudah</b> saring &mdash; dan kotak yang dibuang sesudah saring tidak
        punya sebab untuk ditulis, jadi labelnya jatuh ke kata bawaan
        &ldquo;<b>disaring</b>&rdquo;. Kotak abu-abu tanpa sebab itu
        <b>tanda tangan</b> gerbang muatan, bukan tanda tangan saring.<br>
        Yang sekarang <b>MATI</b> secara bawaan:
        <code>tolak_saat_membawa</code> (baris status <code>membawa</code> dari
        firmware bertahan di memori sampai status berikutnya &mdash; gerbang yang
        bergantung keadaan basi tidak boleh memegang kendali penglihatan) dan
        <code>vision_silang_on</code> (silang-periksa hanya sekuat bacaan yang
        disilangkan; <code>tinggi_k1..k5</code> masih 0, jadi angka kameranya
        taksiran). <code>roi_atas_frac</code> tetap 0 sampai diukur.<br>
        Gerbang <b>kelas</b> tidak hilang &mdash; ia kembali ke tempat asalnya,
        <code>sasaran_sah()</code>, yang menahan <b>gerakan</b>, bukan
        penglihatan. Dummy masih tidak akan pernah dikejar; bedanya sekarang
        kotaknya tetap terlihat apa adanya. Pengukuran jarak tetap jalan, tapi
        murni <b>bacaan</b>: ia tidak pernah membuang sasaran.</p>
        <p class=kecil><b>1. Garis tengah (<code>cx_offset_px</code>).</b> Taruh korban
        tepat di depan capit, ukur pakai penggaris. Kalau kotaknya tidak di tengah
        layar, geser <code>cx_offset_px</code> sampai simpangan terbaca <b>0</b>.
        Ini yang paling menentukan: salah di sini membuat capit selalu meleset ke
        sisi yang sama, dan PID tidak akan pernah memperbaikinya.<br>
        <b>2. Sudut pandang (<code>hfov_deg</code>).</b> Taruh dua benda di tepi kiri
        dan kanan frame, ukur jarak nyatanya. Kalau derajat yang dihitung HUD tidak
        cocok, setel <code>hfov_deg</code>. Ini mengubah arti SEMUA angka derajat.<br>
        <b>3. Pita tinggi bbox (<code>bbox_h_min/max</code>).</b> Taruh korban di jarak
        kerja, catat "deteksi" di kartu Jejak, lalu longgarkan pitanya secukupnya
        &mdash; jangan lebih.<br>
        <b>4. Pita capit (<code>roi_bawah_frac</code>).</b> Turunkan lengan, lihat
        stream, catat di ketinggian berapa capit mulai terlihat.<br>
        <b>4b. Pita ATAS (<code>roi_atas_frac</code>).</b> Pose REHAT
        (<code>R</code>) melipat lengan <b>160 mm di atas</b> bidang pusat badan,
        cuma 80 mm ke depan. Komentar Vincent menyebut gunanya &mdash; tidak
        menghalangi LiDAR depan, tidak tersangkut di tangga &mdash; tapi
        <b>tidak menyebut kamera</b>. Gendong korban, lihat stream, catat di
        ketinggian berapa ia mulai terlihat, isi fraksinya.<br>
        <b>5. Toleransi (<code>tengah_tol_px</code>) &amp; tahan
        (<code>tengah_tahan_s</code>).</b> Sekarang <b>20 px</b> dan <b>2 detik</b>.
        8 px yang lama = 0,44&deg; &mdash; lebih kecil daripada noise bbox dan
        lebih kecil daripada langkah terkecil putar badan (0,25&deg;), jadi
        syaratnya menuntut ketenangan yang tidak pernah bisa dicapai: robot
        melangkah, mengukur, masih di luar, melangkah lagi, selamanya. 20 px =
        1,1&deg;: cukup ketat untuk capit, dan bisa dicapai.<br>
        Dan syaratnya harus <b>bertahan 2 detik tanpa putus</b> &mdash; satu frame
        yang kebetulan di tengah bukan bukti robot DIAM di tengah, bisa jadi ia
        sedang melewatinya. Putus sekali menolkan hitungannya. Inilah pemicu
        serah-terima ke firmware: datang dari Pi saat ia yakin, bukan dari jam
        yang habis.</p>
        <hr>
        <div class=mid>6. Jarak dari kamera (<code>korban_tinggi_cm</code>)</div>
        <p class=kecil>Kamera bisa mengukur jarak sendiri:
        <code>d = f &middot; H / h_px</code>. Di 25 cm, 5 piksel noise tinggi bbox
        cuma bernilai <b>3 mm</b> &mdash; lebih halus daripada LiDAR. Gunanya bukan
        menggantikan LiDAR, melainkan <b>memeriksa silang</b>-nya: LiDAR depan punya
        pita hantu di 3,2 dan 9 cm, dan target capit kita 10 cm, tepat di sebelahnya.
        Dua sensor yang salahnya tidak berkorelasi adalah satu-satunya cara tahu.</p>
        <p class=kecil style="color:var(--warn)"><b>Ada batas atasnya, dan batasnya
        jatuh sebelum jarak capit.</b> Korban 12 cm mengisi seluruh tinggi frame di
        <b>15 cm</b>; lebih dekat dari itu kepalanya di atas frame dan kakinya di
        bawah. Bbox terpotong &rarr; <code>h_px</code> berhenti tumbuh &rarr; jaraknya
        terbaca <b>lebih jauh</b> dari kenyataan &mdash; arah galat yang membuat robot
        mengira masih ada ruang lalu menabrak. HUD mendeteksi pemotongan dan
        <b>membuang</b> bacaannya; ia tidak menebak.</p>
        <p class=kecil><b>Cara kalibrasi &mdash; jangan ukur tinggi bonekanya.</b>
        Yang dibutuhkan rumus itu tinggi yang <b>dilihat model</b>: bbox YOLO jarang
        mepet dan sering memotong kaki. Jadi: taruh korban di jarak yang kamu ukur
        sendiri dengan penggaris (30&ndash;40 cm paling enak, bbox utuh dan besar),
        pastikan kotaknya terlihat, isi jaraknya, tekan tombol. Satu pengukuran
        menyerap bbox longgar, distorsi lensa dan kaki yang terpotong sekaligus.</p>
        <p class=kecil><b>Ukuran korban sudah diukur</b> (R2C, 11 Sep 2026):
        tinggi <b>&plusmn;9 cm</b>, lebar <b>&plusmn;8,5 cm</b>. Dari situ, pada
        jarak kerja <b>25 cm</b> bbox-nya <b>327 px</b> = 0,45 tinggi frame &mdash;
        utuh, dan 5 piksel noise cuma <b>3,8 mm</b>. Bbox baru mulai terpotong di
        bawah <b>11,3 cm</b>, jadi cara lama (10 cm) memang tepat di dalam zona
        buta itu.</p>
        <p class=kecil><b>Lebar dipakai untuk memeriksa, bukan mengukur.</b>
        Memutar korban pada sumbu tegak tidak mengubah tingginya, cuma lebarnya
        &mdash; jadi tinggi itu besaran yang kokoh untuk jarak, dan rasio
        lebar/tinggi (0,94 saat menghadap) jadi petunjuk murah bahwa bbox-nya
        bukan satu korban utuh: terpotong, bergabung dengan dummy, atau tertimpa
        papan 14&times;17 di K-3/K-4. Di luar pita
        <code>rasio_min..rasio_maks</code>, jaraknya dibuang.</p>
        <p class=kecil><b>Per posisi K-1..K-5.</b> Tombol di bawah menulis ke
        <code>tinggi_k1..tinggi_k5</code> kalau robot sedang di ruang korban, dan
        ke <code>korban_tinggi_cm</code> kalau tidak. Jadi kalibrasi tiap posisi
        tinggal: berdiri di ruangnya, ukur jaraknya, tekan. Yang masih
        <b>0</b> otomatis memakai nilai global &mdash; 0 berarti "belum
        dikalibrasi", bukan "tingginya nol".</p>
        <p class=kecil>Ambang <code>bbox_tol_px</code> (sekarang 20 px) adalah
        toleransi dalam <b>piksel</b>, tempat noise-nya sungguh-sungguh hidup.
        Padanan cm-nya ikut jarak (<code>|dd/dh| = d/h</code>) dan ditampilkan di
        baris <b>bbox diharap</b>, jadi ia dan <code>capit_tol_cm</code> bisa
        disetel sepadan &mdash; bukan masing-masing ditebak. Di 25 cm, 20 px
        &asymp; 1,5 cm.</p>
        <input id=vis_d style="width:64px" value="25"> <span class=kecil>cm (jarak sungguhan)</span>
        <button data-tip="Baca bbox sasaran yang TERLIHAT SEKARANG, lalu hitung korban_tinggi_cm dari jarak yang kamu isi. Butuh sasaran korban terdeteksi dan bbox-nya TIDAK menyentuh tepi frame." onclick="cmd('kalib_jarak',$('vis_d').value)">Kalibrasi jarak vision</button>
        <hr>
        <button data-tip="Ambil 9 frame di pose apa adanya lalu putuskan korban/dummy. NOL perintah gerak. Ini cara memeriksa hasil setelan tanpa menggerakkan robot." onclick="cmd('ujiv')">Uji vision (tanpa gerak)</button>
        <button data-tip="MODE FOKUS: berhenti mengirim video dan berhenti menggambar kotak di atas frame. Encode JPEG 1280x720 memakan inti yang seharusnya untuk inferensi. Angka simpangan, log dan seluruh kendali TETAP jalan -- yang berhenti cuma gambarnya. Pakai ini saat menyetel penengahan." onclick="cmd('fokus')">Mode FOKUS video mati/nyala</button>
        <button data-tip="Simpan seluruh nilai kalibrasi ke kalib_<waktu>.json di folder ~/M. Satu-satunya penulisan berkas di seluruh program." onclick="simpanKalib()">Simpan kalibrasi ke file</button>
      </div>
      <div class=card><h2>Kalibrasi kompas arena</h2>
        <p class=kecil><b>Semua ini SUDAH ada di firmware</b> &mdash; tidak perlu minta
        Vincent menambah apa pun. Kompas arena itu empat sudut yaw IMU yang dicatat
        saat robot benar-benar menghadap tiap dinding. Perintah <code>o0</code>..<code>o3</code>,
        mode arena, dan <code>m1</code> semuanya bergantung padanya.</p>
        <p class=kecil style="color:var(--bad)"><b>WAJIB IMU HIDUP.</b>
        <code>kompasCatat()</code> membaca <code>_imu.yawDeg()</code> dan langsung
        keluar dengan <i>"Navigation: Tidak ada data sudut IMU."</i> kalau IMU mati.
        Selama IMU-mu belum beres, <b>tidak ada satu pun</b> dari tombol catat di bawah
        yang akan bekerja. Perbaiki IMU dulu.</p>
        <div class=mid>1 &mdash; lihat dulu apa yang tersimpan (aman, tanpa IMU pun jalan)</div>
        <table><tbody id=kompas></tbody></table>
        <div class=kecil id=kompas_ket>-</div>
        <button data-tip="Cetak empat arah yang tercatat di RAM sekarang. Aman, hanya membaca, tidak butuh IMU. Yang belum dicatat tertulis 'belum dicatat'. Firmware: k." onclick="cmd('man','k')">Lihat kompas (k)</button>
        <button data-tip="Muat ulang empat arah dari EEPROM 1792 ke RAM. Berguna kalau kompas di RAM sudah kacau tapi yang tersimpan masih benar. Tidak butuh IMU. Firmware: E." onclick="cmd('man','E')">Muat dari EEPROM (E)</button>
        <div class=mid>2 &mdash; catat: hadapkan robot ke dinding itu DULU, baru tekan</div>
        <button data-tip="Catat yaw IMU saat ini sebagai UTARA. Robot harus BENAR-BENAR sedang menghadap dinding utara arena. Firmware: c0." onclick="cmd('man','c0')">c0 UTARA</button>
        <button data-tip="Catat arah TIMUR. Firmware: c1." onclick="cmd('man','c1')">c1 TIMUR</button>
        <button data-tip="Catat arah SELATAN. Wajib ada -- pivot akhir v1.8 gagal tanpa SELATAN. Firmware: c2." onclick="cmd('man','c2')">c2 SELATAN</button>
        <button data-tip="Catat arah BARAT. Wajib ada -- korban 1 dan pivot akhir memakainya. Firmware: c3." onclick="cmd('man','c3')">c3 BARAT</button>
        <div class=mid>3 &mdash; simpan permanen</div>
        <button class=danger data-tip="Tulis keempat arah ke EEPROM 1792. MENEMBUS whitelist: 'e' sengaja diblokir karena menulis EEPROM, dan tulisan EEPROM yang tidak disengaja itulah yang dulu bikin kalibrasi berubah tanpa ada yang menyentuhnya. Periksa 'k' dulu sebelum menekan ini." onclick="if(confirm('SIMPAN kompas ke EEPROM 1792.\n\nSudah cek dengan k bahwa KEEMPAT arah benar?\nYang tersimpan sekarang akan DITIMPA.'))cmd('manpaksa','e')">Simpan ke EEPROM (e)</button>
        <table>
          <tr><td class=mid width=80>catatan</td><td class=kecil>Keempat arah TIDAK harus berjarak 90 der. Firmware sengaja memakai sudut yang TERCATAT, bukan mengasumsikan keempatnya tegak lurus sempurna di IMU &mdash; jadi catat apa adanya, jangan &quot;dirapikan&quot;.</td></tr>
        </table>
      </div>
      <div class=card><h2>Kalibrasi <span class=mid>(RAM, tanpa autosave)</span></h2>
        <div id=kalib></div>
      </div>

      <div class=card>
        <h2>K-3 / K-4 &mdash; condong &amp; geser</h2>
        <div class=mid>Sembilan angka yang menentukan penyelamatan di bawah
        reruntuhan. Tiga yang pertama tinggal di EEPROM Teensy; sisanya milik
        Raspi.</div>
        <div id=kalib34></div>
        <button data-tip="Kirim condong.mm, condong.jeda dan condong.yaw ke Teensy lalu 'W' -- tersimpan di EEPROM, jadi misi serial berikutnya tidak perlu flash ulang." onclick="cmd('kirim_condong')">Kirim + simpan ke Teensy</button>
        <button data-tip="Tulis SELURUH kalibrasi Raspi ke kalib_aktif.json. Berkas itu dimuat otomatis tiap HUD start." onclick="simpanKalib()">Simpan kalibrasi Raspi</button>
        <button onclick="simpanKalib()">Simpan kalibrasi ke file</button>
      </div>

      <div class=card>
        <h2>Trim servo <span class=mid>(EEPROM 1024)</span></h2>
        <div class=mid>Koreksi netral tiap servo, mikrodetik. Sebelum ini cuma
        bisa disetel dengan mem-flash sketsa KALIBRASI &mdash; firmware misi
        hilang dari Teensy, lalu harus di-flash balik.
        <b>Angka yang diketik masuk RAM saja</b>; tekan Simpan supaya bertahan
        sesudah reset.</div>
        <button data-tip="Minta firmware mencetak 24 baris '#TRIM'. Lakukan ini DULU sebelum menyetel -- tabel yang belum pernah dibaca tidak tahu angka yang sedang berlaku. Firmware: Yt." onclick="cmd('man','Yt')">Baca ulang (Yt)</button>
        <div id=trim class=kecil>belum dibaca &mdash; tekan Baca ulang</div>
        <button class=danger data-tip="Tulis trim ke EEPROM 1024, alamat yang sama yang dibaca firmware tiap boot dan yang dipakai sketsa legacy. Firmware membaca balik hasilnya dan melapor kalau gagal. Firmware: YtW." onclick="if(confirm('SIMPAN trim ke EEPROM 1024.\n\nYang tersimpan sekarang akan DITIMPA.\nSudah tekan Baca ulang dan angkanya benar?'))cmd('manpaksa','YtW')">Simpan ke EEPROM (YtW)</button>
        <button class=danger data-tip="Nolkan seluruh trim di RAM. Tidak menyentuh EEPROM sampai Simpan ditekan. Firmware: Yt!." onclick="if(confirm('NOLKAN seluruh trim di RAM.\n\nEEPROM belum berubah sampai Simpan ditekan.'))cmd('man','Yt!')">Nolkan semua (Yt!)</button>
        <table>
          <tr><td class=mid width=80>urutan</td><td class=kecil>Setel pada pose yang DIPAKAI, berbeban &mdash; bukan di 90 der tanpa beban. Trim ditambahkan sesudah konversi derajat ke pulse, jadi galatnya nol tepat di 90 der dan tumbuh sebanding jaraknya dari situ. Pose berdiri jauh dari 90 (femur &minus;10,6 der, tibia &minus;8,0 der).</td></tr>
          <tr><td class=mid>bukan trim</td><td class=kecil>Kalau satu sendi menuntut lebih dari &plusmn;200 us, yang salah bukan trim: horn terpasang di gigi yang salah, atau invert terbalik. Firmware menjepitnya di 200.</td></tr>
        </table>
      </div>
    </div>
  </div>
</div>
<script>
const $=i=>document.getElementById(i);
// Grafik kecil daya. Sengaja canvas mentah, bukan pustaka: satu file, nol
// unduhan, dan halaman ini harus tetap hidup walau Pi tidak punya internet.
function gambarDaya(r){
  var c=$('p_graf'); if(!c||!r) return;
  var x=c.getContext('2d'), W=c.width, H=c.height;
  x.clearRect(0,0,W,H);
  if(r.length<2){ return; }
  var vs=r.map(a=>a[0]).filter(v=>v>0), ds=r.map(a=>a[1]);
  var dmax=Math.max.apply(null,ds)||1;
  // Sumbu tegangan DIPAKU 4,6-5,3 V, tidak auto-skala. Auto-skala membuat
  // riak 10 mV terlihat seperti tebing dan bikin panik tanpa sebab.
  var VLO=4.6, VHI=5.3;
  var y=v=>H-((Math.min(VHI,Math.max(VLO,v))-VLO)/(VHI-VLO))*(H-4)-2;
  // garis 4,8 V: ambang under-voltage Pi
  x.strokeStyle='#a33'; x.setLineDash([3,3]); x.lineWidth=1;
  x.beginPath(); x.moveTo(0,y(4.8)); x.lineTo(W,y(4.8)); x.stroke();
  x.setLineDash([]);
  // konsumsi sebagai isian di belakang
  x.fillStyle='rgba(90,140,220,0.25)';
  x.beginPath(); x.moveTo(0,H);
  r.forEach(function(a,i){ x.lineTo(i/(r.length-1)*W, H-(a[1]/dmax)*(H-6)-2); });
  x.lineTo(W,H); x.closePath(); x.fill();
  // tegangan sebagai garis di depan
  if(vs.length>1){
    x.strokeStyle='#6c6'; x.lineWidth=1.5; x.beginPath();
    var mulai=true;
    r.forEach(function(a,i){
      if(a[0]<=0) return;
      var px=i/(r.length-1)*W, py=y(a[0]);
      if(mulai){ x.moveTo(px,py); mulai=false; } else { x.lineTo(px,py); }
    });
    x.stroke();
  }
  $('p_graf_ket').textContent='tegangan 4,6-5,3 V (garis hijau, putus-putus = ambang 4,8 V)'
    + ' \u00b7 konsumsi 0-'+dmax.toFixed(1)+' W (isian biru) \u00b7 '
    + r.length+' sampel';
}
$('host').textContent=location.host;
function tab(i,el){document.querySelectorAll('.panel').forEach((p,n)=>p.classList.toggle('aktif',n===i));
  document.querySelectorAll('#tabs button').forEach(b=>b.classList.remove('aktif'));el.classList.add('aktif');}
async function cmd(k,v){await fetch('/cmd?k='+k+(v!==undefined?'&v='+encodeURIComponent(v):''),{method:'POST'});tarik();}
function setRuas(){
  var i=$('ruas_i').value.trim(), c=$('ruas_cm').value.trim();
  if(i===''||c===''){ alert('Isi nomor ruas DAN panjangnya dalam cm.'); return; }
  cmd('man','m7 '+i+' '+c);
}
function kirimManual(){const b=$('cmdbox');if(!b.value.trim())return;cmd('man',b.value.trim());b.value='';}
function putar(arah){
  var d=parseFloat($('putar_der').value);
  if(!isFinite(d)||d===0){alert('Isi dulu berapa derajat.');return;}
  cmd('putar',(arah*Math.abs(d))+'|'+$('putar_cara').value);
}
function kirimPaksa(){const b=$('cmdbox');if(!b||!b.value.trim())return;
  if(!confirm('Kirim MENEMBUS whitelist: '+b.value+'\n\nPerintah ini sengaja diblokir. Yakin?'))return;
  cmd('manpaksa',b.value.trim());b.value='';}
function restartApp(){if(!confirm('Restart mission_hud.py?\n\nHalaman kosong ~6 detik lalu muat sendiri.'))return;
  cmd('restart');setTimeout(()=>location.reload(),6000);}
async function simpanKalib(){const r=await fetch('/cmd?k=simpan_kalib',{method:'POST'});alert(await r.text());}
// Geser satu trim. Angkanya dibaca dari kotaknya sendiri, bukan disimpan
// terpisah di JS: kotak itu yang diperbarui poll berikutnya dari jawaban
// firmware, jadi ia satu-satunya yang tidak bisa menyimpang dari Teensy.
function trimGeser(slot,d){
  const el=$('tr_'+slot); if(!el) return;
  const v=(parseInt(el.value||'0',10)||0)+d;
  el.value=v; cmd('man','Yt'+slot+' '+v);
}
function esc(s){return String(s).replace(/[<>&]/g,c=>({'<':'&lt;','>':'&gt;','&':'&amp;'}[c]));}
// Satu elemen yang hilang dari HTML pernah MEMATIKAN SELURUH HUD: $('id')
// mengembalikan null, '.textContent=' melempar, dan sisa tarik() -- daya,
// suhu, kalibrasi, semuanya -- tidak pernah dijalankan. Dari luar gejalanya
// "kartu kosong", bukan "ada error", jadi tidak ada yang mencarinya di
// console. Sekarang kegagalannya DIKATAKAN di bilah galat, dan pembaruan
// berikutnya tetap jalan.
async function tarik(){
  try{ await tarikSekali(); }
  catch(e){
    var g=document.getElementById('galat');
    if(g){ g.style.display='block';
           g.textContent='BUG TAMPILAN: '+e.message+' -- sebagian kartu tidak diperbarui. '
                       + 'Muat ulang halaman; kalau tetap, laporkan baris ini.'; }
    if(window.console) console.error(e);
  }
}
async function tarikSekali(){
  let d; try{d=await(await fetch('/state')).json()}catch(e){return}
  if(!d||!d.misi){$('galat').style.display='block';
    $('galat').textContent='Loop utama belum mengisi data. Cek: journalctl -u r2c-hud -n 50';return}
  $('galat').style.display=d.galat?'block':'none'; if(d.galat)$('galat').textContent=d.galat;
  $('versi').textContent='v '+d.versi;
  $('mode').className='kendali';
  $('mode').textContent='KENDALI - SEMUA PERINTAH LANGSUNG KE TEENSY, TANPA FILTER';
  // State yang belum diprogram diberi warna kuning DAN label. Tanpa label,
  // robot yang diam di sini tidak bisa dibedakan dari robot yang sedang
  // menunggu sensor -- dan operator menunggu sesuatu yang tidak akan datang.
  $('state').textContent = d.state + (d.state_belum? '  (BELUM DIPROGRAM)' : '');
  $('state').style.color = d.state=='GAGAL' ? 'var(--bad)'
                         : d.state_belum    ? 'var(--warn)' : 'var(--ok)';
  $('next').style.color = d.next_belum ? 'var(--warn)' : '';
  $('next').textContent=d.next;
  $('stdesc').textContent=d.state_desc; $('lewat').textContent=d.lewat; $('slot').textContent=d.slot;
  $('vis').textContent=d.vision?'NYALA':'mati';
  $('vis').style.color=d.vision?'var(--ok)':'var(--dim)';
  $('depan').textContent=d.depan; $('teensy').textContent=d.teensy;
  $('port').textContent=d.port+(d.port_ok?'':' TIDAK TERSAMBUNG');
  $('port').style.color=d.port_ok?'var(--ok)':'var(--bad)';
  $('kamera').textContent=d.kamera_ok?d.kamera:'TIDAK AKTIF';
  $('kamera').style.color=d.kamera_ok?'var(--ok)':'var(--bad)';
  $('fps').textContent='fps '+d.fps+' · penonton '+d.penonton;
  var kunci = d.halt?'TERKUNCI (STOP)' : d.jeda?'DIJEDA (pause)' : 'bebas';
  $('kunci_st').textContent = kunci + (d.sebab_henti? '  -- '+d.sebab_henti : '');
  $('kunci_st').style.color = d.halt?'var(--bad)' : d.jeda?'var(--warn)' : 'var(--ok)';
  $('kunci_st').style.fontWeight = (d.halt||d.jeda)?'700':'400';
  $('henti').hidden = !(d.halt || d.jeda);
  $('henti_teks').textContent = d.halt
    ? ('OTOMASI TERKUNCI \u2014 ' + (d.sebab_henti||'STOP') +
       '. Tombol gerak ditolak sampai STOP dilepas.')
    : ('DIJEDA di state ' + d.state + '. State disimpan, tekan Lanjutkan untuk resume.');
  // Mode fokus: video sengaja dimatikan supaya inti Pi dipakai inferensi.
  var cam=$('cam');
  if(d.fokus){
    if(cam.src.indexOf('stream')>=0){ cam.removeAttribute('src'); }
    cam.style.background='#222'; cam.alt='MODE FOKUS - video dimatikan, vision tetap jalan';
  } else if(!cam.getAttribute('src')){
    cam.src='/stream.mjpg';
  }
  $('ruas_fw').textContent=d.ruas_fw;
  $('awas').textContent=d.awas; $('awas').style.color=d.awas_warna;
  $('awas').style.fontWeight=d.awas_warna=='var(--dim)'?'400':'700';
  $('tombol_info').textContent = 'tombol PCB: ' + d.tombol;
  $('tombol_info').style.color = d.tombol_baru ? 'var(--ok)' : 'var(--dim)';
  $('tombol_info').style.fontWeight = d.tombol_baru ? '700' : '400';
  $('log_info').textContent = d.log_ringkas
    ? ('mode RINGKAS \u00b7 ' + d.n_diam + ' baris poll rutin disembunyikan')
    : 'mode PENUH \u00b7 semua baris dicatat';
  $('log_info').style.color = d.log_ringkas ? 'var(--dim)' : 'var(--warn)';
  $('sehat').textContent=d.sehat;
  $('sehat').style.color=d.sehat_ok?'var(--ok)':'var(--bad)';
  $('sehat').style.fontWeight=d.sehat_ok?'400':'700';
  if(d.daya_rinci){
    var p=d.daya_rinci;
    $('p_status').textContent=p.status;
    $('p_status').style.color=p.status_warna;
    $('p_status').style.fontWeight='700';
    $('p_volt').textContent=p.volt;   $('p_volt').style.color=p.volt_warna;
    $('p_daya').textContent=p.daya;
    $('p_arus').textContent=p.arus;
    $('p_suhu').textContent=p.suhu;   $('p_suhu').style.color=p.suhu_warna;
    $('p_oc').textContent=p.overcurrent; $('p_oc').style.color=p.overcurrent_warna;
    $('p_bit').textContent=p.bit;     $('p_bit').style.color=p.bit_warna;
    gambarDaya(p.grafik);
  }
  $('rx').textContent=d.rx;
  $('rx').style.color=d.rx_ok?'var(--ok)':'var(--bad)';
  const beku=(d.umur===undefined?0:d.umur)>3;
  $('umur').textContent=beku?('LOOP BEKU '+d.umur+' detik'):('data '+d.umur+'s');
  $('umur').style.color=beku?'var(--bad)':'var(--dim)';
  $('umur').style.fontWeight=beku?'700':'400';
  $('penonton').textContent=d.penonton;
  $('dummy_saja').textContent=d.dummy_saja||'-';
  $('dummy_saja').style.color=d.dummy_saja?'var(--bad)':'var(--dim)';
  $('sasaran').textContent=d.sasaran; $('sasaran').style.color=d.sasaran_warna;
  $('sasaran').style.fontWeight='700';
  $('bearing').textContent=d.bearing;
  $('bearing').style.color=d.bearing_ok?'var(--ok)':'var(--warn)';
  $('ambil_langkah').textContent=d.ambil_langkah;
  $('ambil_langkah').style.color=d.ambil_aktif?'var(--warn)':'var(--dim)';
  $('ambil_langkah').style.fontWeight=d.ambil_aktif?'700':'400';
  $('ambil_depan').textContent=d.ambil_depan;
  $('jarak_vis').textContent=d.jarak_vis;
  $('jarak_vis').style.color=d.jarak_vis_ok?'var(--ok)':'var(--dim)';
  $('beda_vis').textContent=d.beda_vis;
  $('beda_vis').style.color=d.beda_vis_ok?'var(--dim)':'var(--bad)';
  $('beda_vis').style.fontWeight=d.beda_vis_ok?'400':'700';
  $('vis_posisi').textContent=d.vis_posisi;
  $('vis_rasio').textContent=d.vis_rasio;
  $('vis_rasio').style.color=d.vis_rasio_ok?'var(--dim)':'var(--bad)';
  $('vis_harap').textContent=d.vis_harap;
  $('membawa').textContent=d.membawa;
  $('membawa').style.color=d.membawa_ada?'var(--warn)':'var(--dim)';
  $('membawa').style.fontWeight=d.membawa_ada?'700':'400';
  $('tunggu_visi').textContent=d.tunggu_visi;
  $('tunggu_visi').style.color=d.tunggu_visi_ok?'var(--ok)':'var(--warn)';
  $('tahan').textContent=d.tahan;
  $('tahan').style.color=d.tahan_ok?'var(--ok)':'var(--dim)';
  $('tahan').style.fontWeight=d.tahan_ok?'700':'400';
  $('pivot_gait').textContent=d.pivot_gait;
  $('pivot_gait').style.color=d.pivot_gait_ok?'var(--dim)':'var(--bad)';
  $('ambil_target').textContent=d.ambil_target;
  $('ambil_badan').textContent=d.ambil_badan;
  $('ambil_badan').style.color=/netral/.test(d.ambil_badan)?'var(--dim)':'var(--warn)';
  $('lengan_geo').textContent=d.lengan_geo;
  $('lengan_geo').style.color=d.lengan_geo_ok?'var(--ok)':'var(--bad)';
  $('lengan_geo').style.fontWeight=d.lengan_geo_ok?'400':'700';
  $('kompas').innerHTML=d.kompas.map(function(k){
    return '<tr><td class=mid width=80>'+k.nama+'</td><td style="color:'
      +(k.ada?'var(--ok)':'var(--warn)')+'">'+k.nilai+'</td></tr>';}).join('');
  $('kompas_ket').textContent = d.kompas_umur===null
    ? 'belum pernah dibaca dari firmware -- tekan "Lihat kompas (k)"'
    : (d.kompas_lengkap
        ? 'keempat arah lengkap \u00b7 dibaca '+d.kompas_umur+' detik lalu'
        : 'BELUM LENGKAP -- m1 akan ditolak \u00b7 dibaca '+d.kompas_umur+' detik lalu');
  $('kompas_ket').style.color=d.kompas_umur!==null&&!d.kompas_lengkap
    ?'var(--bad)':'var(--dim)';
  $('pemicu').textContent=d.pemicu+'   ('+d.pemicu_cara+')';
  $('pemicu').style.color=d.pemicu_ada?'var(--ok)':'var(--dim)';
  $('serah').textContent=d.serah;
  $('serah').style.color=/AMBIL ALIH/.test(d.serah)?'var(--warn)':'var(--dim)';
  $('deteksi').textContent=d.deteksi;
  $('hint').textContent=d.hint;
  // Misi yang firmware-nya BELUM ada diberi tanda. Tanpa ini, daftar 19 baris
  // terbaca seolah robot bisa menjalankan semuanya -- padahal HUD tidak punya
  // kode navigasi sama sekali, dan Mission.cpp v1.8 baru sampai korban 1.
  $('misi').innerHTML=d.misi.map(m=>`<li class="m ${m.kelas}"><span class=id>${m.id}</span>`
    + `<span>${esc(m.label)}`
    + (m.fw ? '' : ` <span style="color:var(--warn)">&middot; firmware belum ada</span>`)
    + `</span></li>`).join('');
  $('stbox').className='card'+(d.state=='GAGAL'?' gagal':'');
  $('gagalbox').innerHTML=d.state=='GAGAL'
    ? `<hr><b style="color:var(--bad)">MISI INI GAGAL</b><br>${esc(d.sebab)}<br>
       <span style="color:var(--warn)">LANJUT KE: ${esc(d.next_misi)}</span><br>
       <button onclick="cmd('next')">Lanjut &amp; tandai LEWAT</button>
       <button onclick="cmd('ulangi')">Ulangi ruang ini</button>` : '';
  $('slots').innerHTML=d.slots.map(s=>`<div>slot ${s.i}: <b style="color:${s.warna}">${s.kelas}</b>
       <span class=mid>margin ${s.margin} conf ${s.conf}</span></div>`).join('')
       +`<div class=mid style="margin-top:6px">suara ${d.suara} &middot; tolak ROI ${d.tolak_roi} / tinggi ${d.tolak_tinggi}</div>`
       +(d.terkunci?`<div style="color:var(--warn)">TERKUNCI: ${esc(d.terkunci)}</div>`:'');
  $('slotbtn').innerHTML=d.n_slot?Array.from({length:d.n_slot},(_,i)=>
       `<button onclick="cmd('slot',${i})">slot ${i+1}</button>`).join(''):'';
  // Kotak kalibrasi dibangun SEKALI saja.
  //
  // Bug sebelumnya: innerHTML ditimpa ulang tiap 300 ms bersama seluruh
  // tarik(). Elemen <input> yang sedang kamu ketik DIHAPUS sebelum onchange
  // sempat jalan, jadi nilainya tidak pernah terkirim -- dan dari luar
  // terlihat seperti kalibrasinya tidak bisa diubah sama sekali.
  //
  // Sekarang barisnya dibuat sekali, lalu tiap poll cuma MEMPERBARUI NILAI --
  // dan itu pun dilewati kalau kotaknya sedang kamu pegang.
  if(!$('kalib').dataset.siap){
    $('kalib').innerHTML=d.kalib.map(k=>{
      var id='kb_'+k.nama;
      if(k.pilihan){
        return `<div>${k.nama} <select id="${id}"
          onchange="cmd('kalib',this.value+'|'+'${k.nama}')">`
          + k.pilihan.map(o=>`<option value="${o}">${o}</option>`).join('')
          + `</select></div>`;
      }
      return `<div>${k.nama} <input id="${id}"
        onchange="cmd('kalib',this.value+'|'+'${k.nama}')"></div>`;
    }).join('');
    $('kalib').dataset.siap='1';
  }
  if(!$('kalib34').dataset.siap){
    $('kalib34').innerHTML=d.kalib34.map(k=>`<div>${k.nama}
      <input id="k34_${k.nama}"
        onchange="cmd('kalib',this.value+'|'+'${k.nama}')"></div>`).join('');
    $('kalib34').dataset.siap='1';
  }
  d.kalib34.forEach(function(k){
    var el=$('k34_'+k.nama);
    if(!el || el===document.activeElement) return;
    if(el.value!=String(k.nilai)) el.value=k.nilai;
  });
  // TRIM. Baris dibangun ulang hanya kalau JUMLAHNYA berubah -- membangun
  // ulang tiap poll akan mencabut fokus dari kotak yang sedang diketik.
  // Tabel kosong berarti BELUM PERNAH DIBACA, bukan "semua nol": menggambar
  // nol yang tidak pernah dibaca lalu menekan Simpan akan menulis nol itu ke
  // EEPROM, dan kalibrasi yang sudah ada hilang tanpa ada yang menyentuhnya.
  if($('trim').dataset.n!=String(d.trim.length)){
    if(!d.trim.length){
      $('trim').innerHTML='belum dibaca &mdash; tekan Baca ulang';
    }else{
      $('trim').innerHTML=d.trim.map(t=>`<div>${t.slot} ${t.nama}
        <button onclick="trimGeser(${t.slot},-5)">&minus;5</button>
        <input id="tr_${t.slot}" size=5
          onchange="cmd('man','Yt${t.slot} '+this.value)">
        <button onclick="trimGeser(${t.slot},5)">+5</button></div>`).join('');
    }
    $('trim').dataset.n=String(d.trim.length);
  }
  d.trim.forEach(function(t){
    var el=$('tr_'+t.slot);
    if(!el || el===document.activeElement) return;
    if(el.value!=String(t.us)) el.value=t.us;
  });
  d.kalib.forEach(function(k){
    var el=$('kb_'+k.nama);
    if(!el || el===document.activeElement) return;   // jangan ganggu yang diketik
    if(el.value!=String(k.nilai)) el.value=k.nilai;
  });
  $('riwayat').innerHTML = d.riwayat.length ? d.riwayat.slice().reverse().map(function(r){
    var j = r.jawab.length
      ? '<div style="color:var(--dim);margin:0 0 4px 14px;white-space:pre-wrap">'
        + r.jawab.map(esc).join('\n') + '</div>'
      : '<div style="color:var(--dim);margin:0 0 4px 14px">(tidak ada jawaban)</div>';
    return '<div><span style="color:var(--dim)">'+r.t+'</span> '
      + '<b style="color:var(--warn)">'+esc(r.cmd)+'</b></div>'+j;}).join('')
    : '<span style="color:var(--dim)">belum ada perintah</span>';
  $('aliran').innerHTML = d.aliran.map(function(a){
    return '<div>'+a.nama+" ('"+a.huruf+"') : <b style=\"color:"
      +(a.hidup?'var(--bad)':'var(--dim)')+'">'
      +(a.hidup?'HIDUP - membanjiri log':'mati')+'</b></div>';}).join('');
  $('kam_st').textContent=d.kamera_nyala?'NYALA':'MATI';
  $('kam_st').style.color=d.kamera_nyala?'var(--ok)':'var(--warn)';
  $('poll_st').textContent=d.poll_hidup?'hidup':'MATI';
  $('poll_st').style.color=d.poll_hidup?'var(--ok)':'var(--warn)';
  $('log').textContent=d.log.join('\n'); $('log').scrollTop=1e6;
}
tarik(); setInterval(tarik,300);
</script>
"""


class Bersama:
    """Kotak data yang dibagi antara loop utama dan thread HTTP."""

    def __init__(self):
        self.kunci = threading.Lock()
        self.jpeg = None
        self.state = {}
        self.antre_perintah = deque()
        self.detak = 0.0           # kapan loop utama terakhir menyelesaikan iterasi
        self.penonton = 0          # berapa tab yang sedang membuka stream
        self._sekali = False       # permintaan /snapshot.jpg satu kali

    # -- siapa yang sedang menonton -----------------------------------
    # Enkode JPEG memakan satu inti yang seharusnya untuk inferensi. Saat
    # lomba berjalan tidak ada yang menatap layar, jadi frame yang di-encode
    # untuk penonton nol adalah pemborosan murni.
    def masuk(self):
        with self.kunci:
            self.penonton += 1

    def keluar(self):
        with self.kunci:
            self.penonton = max(0, self.penonton - 1)

    def minta_sekali(self):
        with self.kunci:
            self._sekali = True

    def perlu_jpeg(self):
        with self.kunci:
            perlu = self.penonton > 0 or self._sekali
            self._sekali = False
            return perlu

    def jumlah_penonton(self):
        with self.kunci:
            return self.penonton

    def set_jpeg(self, buf):
        with self.kunci:
            self.jpeg = buf

    def get_jpeg(self):
        with self.kunci:
            return self.jpeg

    def set_state(self, d):
        with self.kunci:
            self.state = d
            self.detak = time.time()

    def get_state(self):
        """State terakhir + umurnya. Umur dihitung SAAT DIMINTA, jadi halaman
        bisa membedakan 'data ini baru' dari 'loop utama sudah membeku'."""
        with self.kunci:
            d = dict(self.state)
            d["umur"] = round(time.time() - self.detak, 1) if self.detak else -1.0
            return d

    def perintah(self, k, v):
        with self.kunci:
            self.antre_perintah.append((k, v))

    def ambil_perintah(self):
        with self.kunci:
            out = list(self.antre_perintah)
            self.antre_perintah.clear()
        return out


def buat_handler(bersama: Bersama):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        # BATAS WAKTU SOCKET, dan tanpa ini HUD lama-lama tidak bisa dibuka
        # sama sekali walaupun Raspi masih bisa di-SSH.
        #
        # HTTP/1.1 memakai keep-alive: sesudah membalas, handler kembali
        # memblokir di rfile.readline() menunggu permintaan berikutnya di
        # koneksi yang sama. Tanpa timeout, blokir itu SELAMANYA. Satu tab
        # peramban membuka sampai enam koneksi, dan tiap reload meninggalkan
        # koneksi lama yang tidak pernah ditutup peramban. ThreadingHTTPServer
        # memberi satu thread per koneksi, jadi thread yang tertahan menumpuk
        # tanpa batas sampai proses berhenti menerima koneksi baru.
        #
        # SSH tetap hidup karena ia proses lain -- itulah kenapa gejalanya
        # terbaca seperti "HUD-nya yang mati", bukan Raspi-nya.
        #
        # 10 detik: halaman menarik /state tiap 300 ms, jadi koneksi yang
        # masih dipakai tidak pernah mendekati angka ini. handle_one_request()
        # menangkap socket.timeout sendiri lalu menutup koneksinya.
        timeout = 10

        def log_message(self, *a):
            pass                                   # jangan banjiri konsol

        def _kirim(self, kode, tipe, isi):
            self.send_response(kode)
            self.send_header("Content-Type", tipe)
            self.send_header("Content-Length", str(len(isi)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(isi)

        def do_GET(self):
            jalur = urlparse(self.path).path
            if jalur in ("/", "/index.html"):
                self._kirim(200, "text/html; charset=utf-8", HALAMAN.encode())
            elif jalur == "/state":
                self._kirim(200, "application/json",
                            json.dumps(bersama.get_state()).encode())
            elif jalur == "/stream.mjpg":
                self.stream()
            elif jalur == "/snapshot.jpg":
                bersama.minta_sekali()
                batas = time.time() + 1.0
                while bersama.get_jpeg() is None and time.time() < batas:
                    time.sleep(0.02)
                self._kirim(200, "image/jpeg", bersama.get_jpeg() or b"")
            else:
                self._kirim(404, "text/plain", b"tidak ada")

        def do_POST(self):
            q = parse_qs(urlparse(self.path).query)
            k = q.get("k", [""])[0]
            v = q.get("v", [None])[0]
            bersama.perintah(k, v)
            pesan = b"ok"
            if k == "simpan_kalib":
                time.sleep(0.35)                  # beri loop utama waktu menulis
                pesan = bersama.get_state().get("kalib_pesan", "ok").encode()
            self._kirim(200, "text/plain; charset=utf-8", pesan)

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=bingkai")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            terakhir = None
            bersama.masuk()
            try:
                t_tulis = time.time()
                while True:
                    buf = bersama.get_jpeg()
                    if buf is None or buf is terakhir:
                        # TIDAK ADA FRAME BARU, dan itu keadaan biasa: kamera
                        # dimatikan operator, atau mode fokus sedang menahan
                        # encode. Tanpa penyelidik di bawah, loop ini tidur
                        # selamanya tanpa pernah menyentuh socket -- jadi tab
                        # yang sudah ditutup tidak pernah ketahuan, thread-nya
                        # tidak pernah selesai, dan bersama.keluar() tidak
                        # pernah dipanggil. Akibatnya perlu_jpeg() terus
                        # menjawab "ada yang menonton" dan Raspi membakar satu
                        # inti meng-encode JPEG untuk tab yang sudah tidak ada.
                        if time.time() - t_tulis > 2.0:
                            self.wfile.write(b"--bingkai\r\n"
                                             b"Content-Type: image/jpeg\r\n"
                                             b"Content-Length: 0\r\n\r\n\r\n")
                            t_tulis = time.time()
                        time.sleep(0.02)
                        continue
                    terakhir = buf
                    self.wfile.write(b"--bingkai\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(buf)).encode()
                                     + b"\r\n\r\n" + buf + b"\r\n")
                    t_tulis = time.time()
            except OSError:
                # BrokenPipe, ConnectionReset, DAN socket.timeout -- ketiganya
                # OSError, dan ketiganya berarti hal yang sama di sini: klien
                # sudah tidak mengambil frame. Menangkap hanya dua yang
                # pertama meninggalkan timeout sebagai traceback di konsol.
                pass
            finally:
                bersama.keluar()
    return H


# =====================================================================
# 10. MAIN
# =====================================================================
def petunjuk(misi):
    """Satu baris: apa yang harus DILAKUKAN ORANG di state ini saat menguji."""
    if not (misi.slot_override or misi.tanpa_gerak):
        return "Taruh robot di depan boneka, lalu pilih salah satu uji di atas."
    if misi.tanpa_gerak:
        if misi.state == S_LIHAT:
            return "Menilai... robot TIDAK digerakkan."
        if misi.state == S_SLOT:
            return ("Geser robot dengan tangan ke slot berikutnya, "
                    "lalu tekan 'State berikutnya'.")
        if misi.state in (S_DEKATI, S_CENGKERAM):
            return "Keputusan terkunci. m2/m3 TIDAK dikirim (mode tanpa gerak)."
    else:
        if misi.state == S_STANDOFF:
            return "Robot menutup jarak ke 20 cm memakai LiDAR depan."
        if misi.state == S_CENTER:
            return "Robot memutar badan menengahkan boneka."
        if misi.state == S_SLOT:
            return "Robot bergeser 8 cm ke slot berikutnya."
        if misi.state == S_DEKATI:
            return ("Berhenti di sini: lengan belum terpasang, "
                    "belum ada perintah cengkeram di firmware.")
    if misi.state == S_GAGAL:
        return "Baca sebabnya, lalu 'Ulangi ruang ini' atau mulai uji lagi."
    return ""


def rakit_state(misi, link, kalib, juri, stats, armed, pesan_kalib, kamera=None,
                bersama=None, sehat=None):
    kelas_misi = []
    for i, (mid, label, jenis, slot, pasangan) in enumerate(MISI):
        t = misi.status[i]
        kelas = ("now" if i == misi.idx else
                 "done" if t == SELESAI else
                 "fail" if t == GAGAL_ else
                 "skip" if t == LEWAT else "todo")
        kelas_misi.append({"id": mid, "label": label, "kelas": kelas,
                           "fw": mid in DIDUKUNG_FIRMWARE})

    d = link.depan_cm()
    depan = "-" if d is None else ("jauh" if d == float("inf") else f"{d:.0f} cm")
    batas = BATAS.get(misi.state, 0)
    nxt = misi.berikut

    slots = []
    for i in range(max(1, misi.n_slot)):
        r = misi.hasil_slot.get(i)
        if r is None:
            slots.append({"i": i + 1, "kelas": "belum", "margin": "-", "conf": "-",
                          "warna": "var(--dim)"})
        else:
            kelas, margin, conf = r
            warna = ("var(--ok)" if kelas == kalib.kelas_korban else
                     "var(--bad)" if kelas == kalib.kelas_dummy else
                     "var(--warn)")            # RAGU -> kuning, bukan merah
            slots.append({"i": i + 1, "kelas": kelas.upper(), "margin": f"{margin:+.2f}",
                          "conf": f"{conf:.2f}", "warna": warna})

    return {
        "armed": armed,
        "misi": kelas_misi,
        "state": misi.state,
        "state_desc": FSM[misi.state][0],
        "state_belum": misi.state in BELUM_ADA,
        "next_belum": misi.state_berikut in BELUM_ADA,
        "next": misi.state_berikut,
        "next_misi": f"{nxt[0]}  {nxt[1]}" if nxt else "tidak ada",
        "lewat": f"{misi.lewat:.1f}s" + (f" / {batas}s" if batas else ""),
        "slot": f"{misi.slot + 1} dari {misi.n_slot}" if misi.n_slot else "-",
        "n_slot": misi.n_slot,
        "vision": misi.state in VISION_ON,
        "depan": f"{depan}   (target {kalib.standoff_cm:.0f} cm)",
        "ambil_langkah": (FSM[misi.state][0] if misi.state in RANTAI_AMBIL
                          else ("GAGAL saat ambil: " + misi.sebab)
                          if misi.state == S_GAGAL and misi.sebab
                          else "belum jalan"),
        "ambil_aktif": misi.state in RANTAI_AMBIL,
        "ambil_depan": depan,
        "jarak_vis": (f"{misi.jarak_vis:.1f} cm" if misi.jarak_vis is not None
                      else f"- ({misi.jarak_vis_sebab})"),
        "jarak_vis_ok": misi.jarak_vis is not None,
        "beda_vis": ("-" if misi.beda_vis_lidar is None
                     else f"{misi.beda_vis_lidar:+.1f} cm"),
        "beda_vis_ok": (misi.beda_vis_lidar is None
                        or abs(misi.beda_vis_lidar)
                        <= kalib.vision_lidar_beda_maks_cm),
        "ambil_badan": (f"yaw {misi.badan_yaw:+.1f} der  "
                        f"geser {misi.badan_x:+.0f} mm"
                        + ("   (netral)" if abs(misi.badan_yaw) < 0.05
                           and abs(misi.badan_x) < 0.05 else "   AKTIF")),
        "ambil_target": (
            f"{jarak_ambil_cm(kalib):.0f} cm (+/- {kalib.capit_tol_cm:.1f})"
            + (f"   [LENGAN maju: a{kalib.lengan_jangkau_cm * 10:.0f} "
               f"{kalib.lengan_tinggi_cm * 10:.0f}]"
               if kalib.mode_ambil == "lengan"
               else f"   [BADAN maju: lengan r={kalib.lengan_r:.0f} "
                    f"h={kalib.lengan_h:.0f} mm]")),
        "tahan": (f"{misi.tahan_s:.1f} / {kalib.tengah_tahan_s:.1f} s"
                  if misi.t_tengah is not None
                  else f"- (butuh {kalib.tengah_tahan_s:.1f} s bertahan)"),
        "tahan_ok": misi.t_tengah is not None,
        "pivot_gait": ("MATI -- 'O' tidak menggerakkan apa pun (IMU?)"
                       if misi.pivot_gait_mati else "dipakai"),
        "pivot_gait_ok": not misi.pivot_gait_mati,
        "kamera_nyala": bool(getattr(kamera, "nyala", True)) if kamera else False,
        "tunggu_visi": ("belum diketahui"
                        if getattr(link, "tunggu_visi", None) is None
                        else ("NYALA" if link.tunggu_visi else "mati")),
        "tunggu_visi_ok": bool(getattr(link, "tunggu_visi", None)),
        "membawa": (lambda b: ("depan + belakang" if b[0] and b[1]
                              else "DEPAN" if b[0]
                              else "BELAKANG" if b[1] else "kosong"))(
            link.membawa() if hasattr(link, "membawa") else (False, False)),
        "membawa_ada": any(link.membawa()) if hasattr(link, "membawa") else False,
        "riwayat": [
            {"t": time.strftime("%H:%M:%S", time.localtime(r["t"])),
             "cmd": r["cmd"], "jawab": r["jawab"]}
            for r in list(getattr(link, "riwayat", []))[-25:]],
        "aliran": [{"huruf": h, "nama": n,
                    "hidup": bool(getattr(link, "aliran", {}).get(h))}
                   for h, n in ALIRAN.items()],
        "aliran_ada": any(getattr(link, "aliran", {}).values()),
        "poll_hidup": kalib.poll_hz > 0,
        "kompas": [
            {"nama": n,
             "nilai": (f"{v:.1f} der" if v is not None else "belum dicatat"),
             "ada": v is not None}
            for n, v in zip(Teensy.ARAH, getattr(link, "kompas", [None] * 4))],
        "kompas_lengkap": all(v is not None
                              for v in getattr(link, "kompas", [None] * 4)),
        "kompas_umur": (int(time.time() - link.kompas_waktu)
                        if getattr(link, "kompas_waktu", 0) else None),
        "pemicu": (
            f"#KORBAN {link.pemicu[0]} ruas {link.pemicu[1]}"
            if getattr(link, "pemicu", None) else "belum ada"),
        "pemicu_ada": bool(getattr(link, "pemicu", None)),
        "pemicu_cara": kalib.pemicu_korban,
        "serah": ("AMBIL ALIH dari firmware" if misi.dari_firmware
                  else "rantai HUD sendiri"),
        "vis_posisi": misi.vis_posisi or "(global)",
        "vis_rasio": (f"{misi.vis_rasio:.2f}" if misi.vis_rasio else "-"),
        "vis_rasio_ok": (not misi.vis_rasio
                         or kalib.rasio_min <= misi.vis_rasio <= kalib.rasio_maks),
        "vis_harap": (
            f"{bbox_diharap_px(kalib, 1280, jarak_ambil_cm(kalib), misi.vis_posisi):.0f} px"
            f" +/-{kalib.bbox_tol_px:.0f}"
            f"  (= +/-{tol_px_ke_cm(kalib, 1280, jarak_ambil_cm(kalib), misi.vis_posisi):.2f} cm)"),
        "lengan_geo": periksa_geometri_lengan(kalib)[2],
        "lengan_geo_ok": periksa_geometri_lengan(kalib)[0],
        "mode_ambil": kalib.mode_ambil,
        "teensy": link.state_teensy()[:46],
        "port": link.nama_port + (f"  ({link.sebab})" if link.sebab
                                  and not link.hidup else ""),
        "port_ok": link.hidup,
        "rx": (f"tx {link.n_tx} · rx {link.n_rx} B"
               + (f" ({time.time() - link.t_rx:.0f}s lalu)" if link.t_rx
                  else "  << TIDAK ADA BALASAN")),
        "rx_ok": link.n_rx > 0,
        "kamera_ok": bool(kamera and kamera.ok),
        "kamera": (kamera.pesan if kamera else "-"),
        "penonton": bersama.jumlah_penonton() if bersama else 0,
        "fps": f"{stats['fps']:.1f} / {stats['t_inf']:.0f} ms",
        "sebab": misi.sebab,
        "versi": VERSI,
        "sehat": sehat.teks() if sehat else "-",
        "sehat_ok": (not sehat.gawat) if sehat else True,
        "daya_rinci": sehat.rinci() if sehat else None,
        "fokus": kalib.fokus_vision,
        "log_ringkas": link.log_ringkas,
        "n_diam": link.n_diam,
        # getattr, bukan atribut langsung: pembangun payload ini dipanggil
        # juga oleh test_mission_hud.py dengan Link tiruan yang cuma punya
        # medan yang diujinya. Idiom yang sama sudah dipakai di sekitar
        # sini untuk parkir_visi dan n_status, dan alasannya sama.
        "tombol": getattr(link, "tombol", "") or "-",
        # 5 detik: cukup lama untuk tertangkap mata yang sedang melihat robot,
        # bukan layar; cukup pendek untuk tidak menyala saat tombol berikutnya
        # ditekan.
        "tombol_baru": bool(getattr(link, "tombol", ""))
                       and (time.time() - getattr(link, "t_tombol", 0.0)) < 5.0,
        "ruas_fw": link.ruas_fw() or "-",
        "awas": (f"{misi.awas_kelas.upper()} conf {misi.awas_conf:.2f}"
                 if misi.awas_kelas else
                 "mengamati..." if misi.state == S_AWAS else "-"),
        "awas_warna": ("var(--ok)" if misi.awas_kelas == kalib.kelas_korban else
                       "var(--bad)" if misi.awas_kelas == kalib.kelas_dummy else
                       "var(--warn)" if misi.state == S_AWAS else "var(--dim)"),
        "halt": misi.halt,
        "jeda": misi.jeda,
        "sebab_henti": misi.sebab_henti,
        "galat": "",
        "hint": petunjuk(misi),
        "dummy_saja": (f"{misi.n_dummy_saja} dummy terlihat, DIABAIKAN"
                       if (not misi.jejak_kelas and misi.n_dummy_saja
                           and misi.state in VISION_ON) else ""),
        "bearing": (f"{misi.bearing_deg:+.1f} der  "
                    f"({math.tan(math.radians(misi.bearing_deg)) * kalib.standoff_cm:+.1f} cm)"
                    if misi.bearing_deg is not None else "sasaran tidak terlihat"),
        "bearing_ok": (misi.bearing_deg is not None
                       and abs(misi.bearing_deg) <= kalib.yaw_tol_deg),
        "sasaran": (f"{misi.jejak_kelas.upper()} {misi.jejak_conf:.2f}"
                    if misi.jejak_kelas else "-"),
        "sasaran_warna": ("var(--ok)" if misi.jejak_kelas == kalib.kelas_korban
                          else "var(--bad)" if misi.jejak_kelas == kalib.kelas_dummy
                          else "var(--dim)"),
        "deteksi": (f"mentah {misi.n_mentah} · bbox tertinggi "
                    f"{misi.h_terbesar:.2f} (pita jejak "
                    f"{kalib.jejak_h_min:.2f}..{kalib.jejak_h_max:.2f})"),
        "uji": bool(misi.slot_override or misi.tanpa_gerak),
        "slots": slots,
        "suara": f"{len(juri.sampel)}/{kalib.n_frame}",
        "tolak_roi": juri.ditolak["roi"],
        "tolak_tinggi": juri.ditolak["tinggi"],
        "terkunci": (f"slot {misi.terkunci[0] + 1} = {misi.terkunci[1]}"
                     if misi.terkunci else ""),
        "kalib": [{"nama": f, "nilai": getattr(kalib, f),
                   "pilihan": KALIB_PILIHAN.get(f)} for f in KALIB_FIELDS],
        "kalib_pesan": pesan_kalib,
        "kalib34": [{"nama": f, "nilai": getattr(kalib, f)} for f in KALIB_K34],
        # TRIM DARI LINK, bukan dari Kalib. Ia sudah punya dua tempat tinggal
        # (EEPROM 1024 dan gTrim di RAM Teensy); menyalinnya ke Kalib membuat
        # yang KETIGA, dan cermin basi persis masalah condong_mm 25 kemarin.
        # Kosong = belum pernah dibaca; tabnya menampilkan "tekan Baca ulang".
        "trim": [{"slot": s, **v} for s, v in sorted(link.trim.items())],
        "log": list(link.log)[-40:],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="best_int8.onnx",
                   help="default 640 px INT8 (F1 0.999). Robot diam saat menilai, "
                        "jadi laju frame bukan penghambat.")
    p.add_argument("--source", default="auto",
                   help="'auto' memindai /dev/video*, atau nomor node mis. 1")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=60)
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--threads", type=int, default=3)
    p.add_argument("--port", default=None,
                   help="port Teensy; kosongkan untuk deteksi otomatis by-id")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--no-serial", action="store_true", help="paksa mode SIMULASI")
    p.add_argument("--web-port", type=int, default=5000)
    p.add_argument("--stream-scale", type=float, default=0.5,
                   help="perkecil frame sebelum JPEG; hemat CPU Pi")
    p.add_argument("--stream-quality", type=int, default=70)
    p.add_argument("--calib", default=None, help="muat kalibrasi dari JSON tertentu")
    p.add_argument("--tanpa-simpanan", action="store_true",
                   help=f"jangan memuat {KALIB_AKTIF} -- mulai dari default")
    p.add_argument("--window", action="store_true", help="tambah window lokal (VNC/HDMI)")
    p.add_argument("--auto", action="store_true",
                   help="TIDAK BERLAKU LAGI -- mode BACA sudah dihapus, HUD "
                        "selalu di KENDALI. Diterima supaya perintah lama tidak galat.")
    args = p.parse_args()

    kalib = Kalib()
    # BERKAS TETAP dimuat otomatis, dan itu perubahan dari perilaku lama.
    # Sebabnya: kalibrasi K-3/K-4 punya sembilan angka yang cuma bisa disetel
    # di arena, dan kehilangan semuanya tiap layanan restart lebih mahal
    # daripada risiko memuat angka basi. Risiko itu ditutup dengan MENCETAK
    # apa yang dimuat, jadi angka basi tidak pernah senyap.
    sumber_kalib = args.calib
    if not sumber_kalib and not args.tanpa_simpanan and os.path.exists(KALIB_AKTIF):
        sumber_kalib = KALIB_AKTIF
    if sumber_kalib:
        try:
            with open(sumber_kalib) as f:
                dimuat = json.load(f)
            n = 0
            for k, v in dimuat.items():
                if hasattr(kalib, k):
                    setattr(kalib, k, v)
                    n += 1
            print(f"[KALIB] dimuat dari {os.path.abspath(sumber_kalib)} "
                  f"({n} nilai)")
            for f_ in KALIB_K34:
                print(f"[KALIB]   {f_} = {getattr(kalib, f_)}")
        except (OSError, ValueError) as e:
            # Berkas rusak BUKAN alasan mematikan HUD. Yang hilang setelannya;
            # yang dibutuhkan halamannya, justru untuk mengetiknya lagi.
            print(f"[KALIB] GAGAL membaca {sumber_kalib}: {e}")
            print("[KALIB] lanjut dengan nilai default di RAM")
    else:
        print("[KALIB] nilai default di RAM -- tidak ada file yang dibaca")

    # URUTAN INI DISENGAJA: halaman web dinyalakan PALING AWAL, sebelum model
    # dimuat dan sebelum kamera dicoba. Dulu sebaliknya, dan akibatnya kamera
    # yang menggantung atau model yang lambat membuat halaman belum ada sama
    # sekali -- persis saat kamu paling butuh membacanya untuk tahu kenapa.
    link = Teensy(None if args.no_serial else args.port,
                  args.baud, auto=not args.no_serial)
    misi, juri, aksi = Misi(), Juri(kalib, {}), Aksi(link)
    pid = Pid(kalib)
    diam = UkurDiam(kalib)
    kamera = Kamera(args.source, args.width, args.height, args.fps)
    bersama = Bersama()

    try:
        srv = ThreadingHTTPServer(("0.0.0.0", args.web_port), buat_handler(bersama))
    except OSError as e:
        print(f"\n[WEB] GAGAL memakai port {args.web_port}: {e}")
        print("[WEB] Kemungkinan aplikasi lamamu masih memegang port itu.")
        print("[WEB] Pilih salah satu:")
        print("[WEB]   sudo systemctl stop <service-lama>   lalu jalankan lagi")
        print("[WEB]   atau  ./run_hud.sh --web-port 5001")
        sys.exit(2)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[WEB] siap -> http://{socket.gethostname()}:{args.web_port}/")

    def penyambung():
        """Membuka port Teensy di latar belakang, selamanya, tiap 2 detik."""
        while True:
            if not link.hidup and (link.auto or link.port_name):
                try:
                    link.sambung()
                except Exception as e:                   # noqa: BLE001
                    link.log.append(f"[LINK] gagal: {e}")
            time.sleep(2.0)

    threading.Thread(target=penyambung, daemon=True).start()

    # MODE BACA DIHAPUS 13 September 2026. Tidak ada lagi dua keadaan --
    # HUD selalu boleh menggerakkan robot sejak detik pertama. Variabelnya
    # dipertahankan (selalu True) supaya seluruh pemanggil kirim()/langkah_fsm()
    # tidak perlu disentuh; ia sekarang konstanta, bukan saklar.
    armed = True
    pesan_kalib = ""
    stats = {"fps": 0.0, "t_inf": 0.0}

    # State pertama terbit SEKARANG, selagi model masih dimuat. Halaman langsung
    # bisa dibuka dan menampilkan "MEMUAT MODEL", bukan layar kosong.
    misi.ganti("MEMUAT")
    FSM["MEMUAT"] = ("memuat model & membuka kamera", S_IDLE)
    bersama.set_state(rakit_state(misi, link, kalib, juri, stats,
                                  armed, pesan_kalib, kamera, bersama))

    sess, names, imgsz = load_session(args.model, args.threads)
    inp = sess.get_inputs()[0].name
    juri.names = names
    print(f"[MODEL] {args.model}  {imgsz[0]}x{imgsz[1]}  kelas={names}")

    # Pemanasan: inferensi pertama selalu paling lambat. Lakukan sekarang,
    # bukan saat robot sudah berdiri di depan korban.
    kosong = np.zeros((1, 3, imgsz[1], imgsz[0]), dtype=np.float32)
    for _ in range(3):
        sess.run(None, {inp: kosong})
    print("[MODEL] pemanasan selesai")

    if not kamera.pastikan():
        print(f"[KAMERA] {kamera.pesan}")
        print("[KAMERA] HUD tetap jalan; kamera dicoba lagi tiap 3 detik.")

    misi.ganti(S_IDLE)
    galat = ""
    t_fps, n_fps, seq, t_poll, t_kirim = time.time(), 0, -1, 0.0, 0.0
    t_infer, dets_lama, lolos_lama = 0.0, [], []
    sehat = Kesehatan()
    bersama.set_state(rakit_state(misi, link, kalib, juri, stats,
                                  armed, pesan_kalib, kamera, bersama))

    try:
        while True:
            try:
                frame = None
                if kamera.pastikan():
                    frame, seq = kamera.read(seq)
                ada_kamera = frame is not None
                if not ada_kamera:
                    frame = bingkai_kosong(kamera.pesan)
                    time.sleep(0.1)
                elif kalib.rotate180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                frame = frame.copy()
                h, w = frame.shape[:2]

                # --- serial: baca selalu, poll status 2x/detik (perintah AMAN) ---
                link.baca()
                if kalib.poll_hz > 0 and time.time() - t_poll > 1.0 / kalib.poll_hz:
                    t_poll = time.time()
                    link.kirim("m", armed, rutin=True)
                    link.kirim("l", armed, rutin=True)
                # Tabel kompas diminta SEKALI tiap sambungan, bukan tiap poll.
                # Angkanya tidak berubah sendiri -- ia cuma berubah kalau ada
                # yang mengetik c0..c3 atau E, dan keduanya lewat tombol yang
                # jawabannya kita baca juga. Memasukkannya ke poll berarti
                # empat baris tambahan tiap detik demi angka yang diam.
                if link.hidup and not link._kompas_diminta:
                    link._kompas_diminta = True
                    link.kirim("k", armed, rutin=True)
                    # JEDA CONDONG DINOLKAN TIAP SAMBUNGAN, 17 Sep 2026.
                    #
                    # `condong.jeda` dan `condong.yaw` hidup di EEPROM Teensy,
                    # jadi menyetel cerminnya di Pi saja tidak mengubah apa
                    # pun sampai ada yang menekan "Kirim + simpan". Sesudah
                    # trial arena jeda itu harus hilang TANPA menunggu tombol,
                    # dan tanpa perlu flash ulang firmware.
                    #
                    # RAM SAJA -- tidak ada 'W' di sini. Menulis EEPROM tiap
                    # kali kabel USB dicolok akan memakai jatah tulisnya untuk
                    # angka yang sudah benar, dan menghapus setelan arena yang
                    # mungkin sengaja ditinggalkan operator. Dikirim ulang tiap
                    # sambungan, jadi reset Teensy pun tidak mengembalikannya.
                    for _nm, _v in (("condong.jeda", kalib.condong_jeda_ms),
                                    ("condong.yaw", kalib.condong_yaw)):
                        link.kirim(f"Q{_nm} {_v:g}", armed, paksa=True)
                    link.log.append(
                        f"[CONDONG] condong.jeda {kalib.condong_jeda_ms:g} ms, "
                        f"condong.yaw {kalib.condong_yaw:g} dikirim (RAM). "
                        f"Fase kosong sebelum capit menutup dihapus.")

                # --- vision HANYA di state tertentu ---
                dets, lolos = [], []
                segar = False
                if ada_kamera and misi.state in VISION_ON:
                    # Saat MENJEJAK, inferensi dibatasi jejak_hz. Frame di
                    # antaranya memakai ulang hasil terakhir untuk digambar --
                    # kotaknya tidak berkedip, tapi CPU-nya menganggur.
                    jeda_min = (1.0 / kalib.jejak_hz
                                if misi.state in (S_JEJAK, S_A_TENGAH)
                                and kalib.jejak_hz > 0
                                else 0.0)
                    if (time.time() - t_infer) < jeda_min:
                        dets, lolos = dets_lama, lolos_lama      # pakai ulang
                    else:
                        t_infer = t0 = time.time()
                        lb, r, pad = letterbox(frame, imgsz)
                        blob = np.ascontiguousarray(
                            lb[:, :, ::-1].transpose(2, 0, 1)[None]
                        ).astype(np.float32) / 255.0
                        out = sess.run(None, {inp: blob})[0]
                        dets = postprocess(out, r, pad, args.conf, args.iou, (h, w))
                        stats["t_inf"] = (time.time() - t0) * 1000
                        dets_lama = dets
                        segar = True
                    misi.n_mentah = len(dets)
                    misi.h_terbesar = (max((y2 - y1) / h for _, y1, _, y2, _, _ in dets)
                                       if dets else 0.0)
                    if segar:
                        # Gerbang dipilih menurut TUGAS state-nya, dan dihitung
                        # SEKALI saja. Dulu gerbang ketat selalu dijalankan lebih dulu
                        # untuk semua state -- akibatnya CENTERING, yang tugasnya justru
                        # menarik sasaran yang masih jauh dan melenceng ke tengah,
                        # membuang sasaran itu sebelum sempat dikejar. Gejalanya persis
                        # "kalau dekat mau, kalau agak jauh diam saja".
                        if misi.state in MENGEMUDI:
                            # Mengejar: jangan pakai ROI (sasaran memang di pinggir) dan
                            # pita tinggi longgar (jaraknya belum diatur).
                            #
                            # AMBIL_HALUS ikut di sini, bukan di gerbang ketat:
                            # tugasnya masih MENENGAHKAN, bukan menilai. Gerbang
                            # ketat dibuat untuk penilaian pada jarak yang sudah
                            # dikunci, dan ia membuang sasaran yang bbox-nya
                            # menyentuh tepi ROI -- yang persis terjadi selagi
                            # badan sedang berputar.
                            lolos = juri.saring(dets, w, h, pakai_roi=False,
                                                h_min=kalib.jejak_h_min,
                                                h_max=kalib.jejak_h_max)
                            lolos_lama = lolos
                        else:
                            # Menilai: gerbang ketat, karena di sinilah jarak sudah
                            # dikunci 20 cm dan tetangga 8 cm harus dibuang.
                            lolos = juri.saring(dets, w, h)
                        lolos_lama = lolos

                        # AMBIL_HALUS ikut cabang PENUH ini (bukan cabang
                        # CENTERING di bawah) supaya kartu vision tetap
                        # menampilkan jarak & bbox di detik-detik terakhir
                        # sebelum kendali diserahkan ke firmware. Yang WAJIB
                        # dari cabang ini cuma satu: bearing_deg ditulis ulang
                        # tiap frame. Lihat MENGEMUDI.
                        if misi.state in (S_JEJAK, S_A_TENGAH, S_A_HALUS,
                                          S_TAHAN):
                            # PEMILIHAN SASARAN = VERSI 10 SEPTEMBER. Sengaja.
                            #
                            # Hari ini tiga gerbang baru ditaruh DI SINI, di
                            # depan pemilihan sasaran: gerbang muatan
                            # (sedang_membawa -> lolos = []), saring kelas
                            # sebelum memilih, dan pita ROI atas. Gabungannya
                            # membuat JEJAK berhenti mengikuti korban, dan
                            # gejalanya persis yang dilaporkan R2C: kotaknya
                            # abu-abu bertuliskan "disaring" saja tanpa sebab.
                            #
                            # Sebabnya bisa dibaca dari kodenya: label kotak
                            # diambil dari juri.alasan, yang HANYA diisi oleh
                            # juri.saring(). Kalau sebuah deteksi lolos saring
                            # tapi dibuang sesudahnya -- lolos = [] -- ia tidak
                            # punya entri alasan, jadi anotasi() memakai teks
                            # bawaannya: "disaring". Kotak abu-abu tanpa sebab
                            # itu tanda tangan gerbang muatan, bukan tanda
                            # tangan saring.
                            #
                            # Jadi pemilihan sasaran dikembalikan apa adanya:
                            # yang TERBESAR di antara yang lolos saring, titik.
                            # Gerbang kelas tetap ada, tapi di tempat asalnya --
                            # sasaran_sah(), yang menahan GERAKAN, bukan
                            # penglihatan. Pengukuran jarak di bawah murni
                            # bacaan: ia tidak pernah membuang sasaran.
                            misi._bawa_diberitahu = False
                            t, _n_dummy = pilih_sasaran(lolos, names, kalib)
                            if t is not None:
                                misi.n_dummy_saja = 0
                                misi.bearing_deg = ((t[6] - (w / 2 + kalib.cx_offset_px))
                                                    / kalib.px_per_deg(w))
                                misi.bearing_t = t_infer
                                misi.jejak_kelas = names.get(t[5], str(t[5]))
                                misi.jejak_conf = t[4]
                                misi.vis_bbox = (t[1], t[3], h, w, t[0], t[2])
                                pos = korban_kini(misi)
                                misi.vis_posisi = pos
                                wajar, misi.vis_rasio = rasio_wajar(
                                    t[0], t[1], t[2], t[3], kalib)
                                if not wajar:
                                    # Bentuknya salah -> bbox-nya bukan satu
                                    # korban utuh, dan tingginya tidak boleh
                                    # dipakai mengukur apa pun.
                                    misi.jarak_vis = None
                                    misi.jarak_vis_sebab = (
                                        f"rasio bbox {misi.vis_rasio:.2f} di luar "
                                        f"{kalib.rasio_min:.2f}..{kalib.rasio_maks:.2f}"
                                        f" -- bukan satu korban utuh")
                                else:
                                    misi.jarak_vis, misi.jarak_vis_sebab = \
                                        jarak_vision_cm(t[1], t[3], h, w, kalib, pos)
                            else:
                                misi.bearing_deg = None
                                misi.jejak_kelas = ""
                                misi.jejak_conf = 0.0
                                # Dummy yang terlihat TAPI diabaikan. Angkanya
                                # tampil di kartu vision ("N dummy terlihat,
                                # DIABAIKAN") supaya "robot diam" bisa
                                # dibedakan dari "robot tidak melihat apa-apa".
                                misi.n_dummy_saja = _n_dummy
                                if _n_dummy and not misi._dummy_diberitahu:
                                    misi._dummy_diberitahu = True
                                    link.log.append(
                                        f"[SASARAN] {_n_dummy} deteksi lolos saring "
                                        f"tapi semuanya '{kalib.kelas_dummy}' -- "
                                        f"TIDAK dikejar. Menunggu "
                                        f"'{kalib.kelas_korban}' terlihat.")
                                if not _n_dummy:
                                    misi._dummy_diberitahu = False
                                misi.vis_bbox = None
                                misi.vis_rasio = 0.0
                                misi.jarak_vis = None
                                misi.jarak_vis_sebab = "tidak ada sasaran"
                        elif misi.state == S_LIHAT:
                            juri.tambah(lolos)
                            if juri.cukup:
                                kelas, margin, conf, cx, alasan = juri.putuskan()
                                misi.hasil_slot[misi.slot] = (kelas, margin, conf)
                                link.log.append(f"[VISION] slot {misi.slot + 1}: {kelas} "
                                                f"margin {margin:+.2f} ({alasan})")
                                juri.reset()
                                misi.ganti(S_SLOT if misi.slot + 1 < misi.n_slot else S_PUTUS)
                        elif misi.state == S_AWAS:
                            juri.tambah(lolos)
                            if juri.cukup:
                                kelas, margin, conf, cx, alasan = juri.putuskan()
                                juri.reset()
                                misi.awas_kelas = kelas
                                misi.awas_conf = conf
                                link.log.append(
                                    f"[AWAS] {link.ruas_fw()[:40]} -> vision "
                                    f"melihat {kelas} margin {margin:+.2f} "
                                    f"conf {conf:.2f} ({alasan})")
                        elif misi.state == S_KONFIRM:
                            # Gerbang KETAT dipakai di sini (bukan gerbang
                            # mengejar): robot sudah berhenti di pose kerja,
                            # dan keputusan ini menentukan misi lanjut atau
                            # mengulang ruas. Salah di sini mahal.
                            juri.tambah(lolos)
                            if juri.cukup:
                                kelas, margin, conf, cx, alasan = juri.putuskan()
                                korban = (kelas == kalib.kelas_korban)
                                juri.reset()
                                link.log.append(
                                    f"[KONFIRM] vision memutuskan: {kelas} "
                                    f"margin {margin:+.2f} conf {conf:.2f} ({alasan})")
                                if kalib.auto_konfirm:
                                    link.kirim("m2" if korban else "m3", armed)
                                    link.log.append(
                                        f"[KONFIRM] dijawab OTOMATIS: "
                                        f"{'m2 LANJUT' if korban else 'm3 ULANGI RUAS'}")
                                    misi.ganti(S_IDLE, f"dijawab {kelas}")
                                else:
                                    link.log.append(
                                        "[KONFIRM] auto_konfirm MATI -- tidak "
                                        "dijawab. Tekan m2/m3 sendiri, atau "
                                        "nyalakan auto_konfirm di tab Kalibrasi.")
                        elif misi.state == S_CENTER:
                            # Aturan pemilihan SAMA PERSIS dengan JEJAK dan
                            # AMBIL_TENGAH: korban dulu, dummy tidak pernah jadi
                            # sasaran. Aturan yang berbeda antar state itu yang
                            # dulu melahirkan "kenapa di JEJAK mau, di CENTER
                            # tidak" -- dan gejala seperti itu tidak pernah
                            # menunjuk sebabnya sendiri.
                            t, _n_dummy = pilih_sasaran(lolos, names, kalib)
                            if t is not None:
                                misi.bearing_deg = ((t[6] - (w / 2 + kalib.cx_offset_px))
                                                    / kalib.px_per_deg(w))
                                misi.bearing_t = t_infer
                                misi.jejak_kelas = names.get(t[5], str(t[5]))
                                misi.jejak_conf = t[4]
                                misi.n_dummy_saja = 0
                            else:
                                misi.bearing_deg = None
                                misi.jejak_kelas = ""
                                misi.n_dummy_saja = _n_dummy

                # --- serah-terima dari firmware v1.9 ---
                # Firmware tidak tahu apa-apa soal kamera. Yang dia lakukan
                # cuma berhenti dan mencetak kalimatnya; HUD yang menyambung.
                fw = link.state_teensy().upper()
                # JANGAN REBUT SERAH-TERIMA YANG SEDANG BERJALAN.
                #
                # Bug 13 Sep 2026, dan ia yang membuat seluruh lup terlihat
                # tidak jalan padahal firmware sudah benar. Kedua cabang di
                # bawah dulu hanya mengecualikan (S_AWAS, S_KONFIRM, S_GAGAL)
                # -- rantai AMBIL TIDAK ada di daftar itu. Padahal selama ruas
                # AMBIL berjalan, KEDUANYA terus bernilai benar:
                #
                #   * ruas_fw() terus berbunyi "K-1 angkat korban", dan
                #   * fw terus berbunyi "MENUNGGU KONFIRMASI -- PARKIR VISION"
                #     justru KARENA firmware sedang parkir menunggu kita.
                #
                # Jadi tepat sesudah tangani_pemicu() memindahkan state ke
                # AMBIL_TENGAH, frame berikutnya menariknya kembali ke S_AWAS
                # atau S_KONFIRM -- keduanya state MENONTON, nol perintah
                # gerak. Lalu frame berikutnya lagi, terus-menerus. Dan tiap
                # tarikan memanggil juri.reset(), jadi votingnya tidak pernah
                # sampai k-of-N: yang tercetak selalu
                # "RAGU margin +0.00 conf 0.00 (tidak ada yang mencapai k-of-N)".
                #
                # Yang merebut adalah bukti parkir yang sama yang baru saja
                # kita tunggu. Itu yang membuatnya sulit dilihat dari luar.
                _sedang_serah = (misi.state in RANTAI_AMBIL or misi.state == S_TAHAN
                                 or bool(misi.pemicu_tunda))
                if ("KONFIRMASI" in fw and misi.state not in (S_KONFIRM, S_GAGAL)
                        and not _sedang_serah
                        and not misi.halt and not misi.jeda):
                    juri.reset()
                    misi.ganti(S_KONFIRM, "firmware minta keputusan korban/bukan")
                    link.log.append("[KONFIRM] firmware berhenti menunggu m2/m3 "
                                    "-- vision dinyalakan")
                elif (any(k in link.ruas_fw().upper() for k in RUAS_VISION)
                      and misi.state not in (S_AWAS, S_KONFIRM, S_GAGAL)
                      and not _sedang_serah
                      and not misi.halt and not misi.jeda):
                    juri.reset()
                    misi.awas_kelas, misi.awas_conf = "", 0.0
                    misi.ganti(S_AWAS, "ruas korban -- vision mengamati")
                    link.log.append(
                        f"[AWAS] masuk ruas korban: {link.ruas_fw()[:50]}. "
                        f"Kamera NYALA, tapi firmware TIDAK menunggu jawaban "
                        f"(belum ada ruas ber-AKS_KONFIRM) -- ini pengamatan.")
                elif "SEKUENS LENGAN" in fw and not misi._lengan_diberitahu:
                    # Jendela ini SANGAT pendek: stub kosong di firmware cuma
                    # menunggu 1,5 detik lalu ruasBerikut() jalan sendiri.
                    # Jadi ini peringatan, bukan tempat menyisipkan vision.
                    misi._lengan_diberitahu = True
                    link.log.append(
                        "[LENGAN] firmware di ruas ANGKAT/TARUH. Capit belum "
                        "terpasang: berhenti kosong ~1,5 detik lalu LANJUT "
                        "SENDIRI. Tidak ada jendela untuk vision di sini.")
                elif "SEKUENS LENGAN" not in fw:
                    misi._lengan_diberitahu = False

                # --- pemicu dari firmware, SEBELUM FSM ---
                # Urutannya penting: pemicu bisa memindahkan state, dan FSM
                # harus melihat state yang BARU di loop yang sama. Kalau
                # dibalik, tiap pemicu tertunda satu frame -- kecil, tapi di
                # sini satu frame itu satu perintah gerak yang terlanjur
                # terkirim ke state yang sudah tidak berlaku.
                tangani_pemicu(misi, link, aksi, kalib, armed, kamera)

                # --- FSM & antrean perintah ---
                langkah_fsm(misi, link, aksi, kalib, armed, pid, w, diam,
                            kamera)
                aksi.putar(armed, boleh=not (misi.halt or misi.jeda))

                # --- perintah dari halaman web ---
                # Saat HALT, perintah gerak ditolak. Kalau tidak, "stop semua
                # aktivitas" cuma berlaku untuk FSM sementara tombol manual
                # tetap bisa menjalankan robot -- itu bukan stop, itu jeda
                # setengah hati. Yang TETAP boleh: melepas stop, melemaskan,
                # dan segala yang tidak menggerakkan apa pun.
                BOLEH_SAAT_HALT = {
                    "stop", "lepas_stop", "lemas", "abort", "pause", "mode",
                    "reset", "kalib", "simpan_kalib", "kirim_condong",
                    "restart", "slot", "gagal",
                }
                for k, v in bersama.ambil_perintah():
                    if misi.halt and k not in BOLEH_SAAT_HALT:
                        link.log.append(f"[HALT] '{k}' ditolak -- tekan "
                                        f"'Lepas STOP' dulu")
                        continue
                    if k == "mode":
                        # Sisa dari mode BACA yang sudah dihapus. Tombolnya
                        # tidak ada lagi, tapi halaman lama yang masih terbuka
                        # di tab browser bisa saja mengirimnya.
                        link.log.append("[MODE] mode BACA sudah dihapus -- "
                                        "HUD selalu di KENDALI.")
                    elif k == "lihat":
                        juri.reset()
                        misi.ganti(S_LIHAT)
                    elif k == "next":
                        if misi.state == S_GAGAL:
                            misi.maju_misi(LEWAT)
                        else:
                            tujuan = misi.state_berikut
                            misi.ganti(tujuan)
                            if tujuan in BELUM_ADA:
                                link.log.append(
                                    f"[NEXT] {tujuan} BELUM DIPROGRAM -- robot "
                                    f"tidak akan bergerak di sini. Pakai "
                                    f"'Lewati misi ini' untuk maju ke misi "
                                    f"berikutnya.")
                    elif k == "lewati":
                        # Lompati SELURUH misi yang sedang jalan, bukan cuma
                        # satu sub-state. Ini jalan keluar dari rantai
                        # DEKATI..LETAK yang belum diprogram: menekan 'State
                        # berikutnya' di sana cuma berpindah dari satu state
                        # diam ke state diam berikutnya.
                        lama = misi.kini[0]
                        aksi.batal()
                        juri.reset()
                        misi.bearing_deg = None
                        misi._urut = None
                        misi._geser_dijadwal = False
                        misi.maju_misi(LEWAT)
                        link.log.append(f"[LEWATI] {lama} ditandai LEWAT -> "
                                        f"{misi.kini[0]}  {misi.kini[1]}")
                        if misi.kini[0] not in DIDUKUNG_FIRMWARE:
                            link.log.append(
                                "[LEWATI] CATATAN: ini cuma menggeser penanda "
                                "di daftar. Robot TIDAK akan jalan ke sana -- "
                                "firmware v1.8 belum punya navigasi untuk misi "
                                "ini. Pindahkan robot dengan tangan.")
                    elif k == "ulangi":
                        misi.ganti(S_STANDOFF)
                    elif k == "gagal":
                        misi.gagal("ditandai gagal oleh operator")
                        aksi.batal()
                    elif k == "slot":
                        misi.slot = int(v or 0)
                        juri.reset()
                    elif k == "reset":
                        juri.reset()
                    elif k == "man" and v:
                        link.kirim(v.strip(), armed)
                    elif k == "manpaksa" and v:
                        link.kirim(v.strip(), armed, paksa=True)
                    elif k == "restart":
                        # Menjalankan ulang proses ini dari awal, tanpa SSH.
                        # os.execv dipakai supaya PID-nya tetap -- systemd tidak
                        # menganggapnya crash, dan jalur ini juga bekerja saat HUD
                        # dijalankan manual dari terminal.
                        link.log.append("[SISTEM] restart diminta dari halaman")
                        print("[SISTEM] restart diminta dari halaman", flush=True)
                        try:
                            srv.shutdown()
                        except Exception:                    # noqa: BLE001
                            pass
                        kamera.lepas()
                        if link.hidup:
                            try:
                                link.ser.close()
                            except Exception:                # noqa: BLE001
                                pass
                        time.sleep(0.3)
                        os.execv(sys.executable, [sys.executable] + sys.argv)
                    elif k == "jejak":
                        misi.slot_override = 1
                        misi.tanpa_gerak = False
                        misi.bearing_deg = None
                        juri.reset()
                        aksi.batal()
                        pid.reset(); diam.reset()
                        misi.tembak_err = misi.tembak_yaw = None
                        misi.ganti(S_JEJAK)
                        link.log.append("[UJI] JEJAK -- geser korban ke mana saja, "
                                        "robot menengahkan sendiri")
                    elif k == "maju":
                        # SATU-SATUNYA maju-mundur yang tersisa di seluruh
                        # HUD, dan ia butuh TANGAN operator tiap kali: satu
                        # tembakan, bukan lingkar tertutup, dari tab Manual.
                        #
                        # Tidak dibuang bersama yang otomatis (17 Sep 2026)
                        # karena bedanya siapa yang memutuskan. Yang dilarang
                        # R2C adalah Pi yang memilih sendiri kapan maju --
                        # dua penguasa untuk satu sumbu. Tombol ini manusia
                        # yang menekan, dan manusia memang penguasa tunggal.
                        #
                        # JANGAN dipakai selagi misi berjalan: di sana Teensy
                        # sedang memegang sumbu ini.
                        d = link.depan_cm()
                        if misi.state in RANTAI_AMBIL or misi.state == S_TAHAN:
                            link.log.append(
                                f"[MAJU] DITOLAK -- rantai AMBIL sedang jalan "
                                f"(state {misi.state}). Maju-mundur milik "
                                f"Teensy selama misi.")
                        elif d is None:
                            link.log.append("[MAJU] jarak depan belum terbaca ('l')")
                        elif d == float("inf"):
                            link.log.append("[MAJU] depan 'jauh' -- korban tak terlihat LiDAR")
                        else:
                            delta = d - kalib.standoff_cm
                            if delta <= kalib.standoff_tol_cm:
                                link.log.append(f"[MAJU] sudah {d:.0f} cm, tidak perlu maju")
                            else:
                                aksi.jadwal((f"D{delta:.0f}", 0.3), ("w", 0.5), ("l", 0.4))
                                link.log.append(f"[MAJU] {d:.0f} -> {kalib.standoff_cm:.0f} cm "
                                                f"(maju {delta:.0f} cm)")
                    elif k == "ambil":
                        # Rantai otomatis: tengahkan -> maju -> buka -> jepit ->
                        # angkat. Sengaja TIDAK dijalankan sendiri sesudah JEJAK;
                        # operator yang menekan, karena mulai dari sini robot
                        # bergerak ke arah korban dan sulit dibatalkan pelan-pelan.
                        if not armed:
                            link.log.append("[AMBIL] mode masih BACA -- "
                                            "nyalakan KENDALI dulu")
                        else:
                            misi.slot_override = 1
                            misi.tanpa_gerak = False
                            misi.sebab = ""
                            juri.reset()
                            aksi.batal()
                            pid.reset(); diam.reset()
                            misi.tembak_err = misi.tembak_yaw = None
                            # Ditekan operator, bukan dipicu firmware: yang
                            # mengambil korbannya rantai capit HUD sendiri.
                            misi.dari_firmware = False
                            misi.ganti(S_A_TENGAH)
                            link.log.append(
                                f"[AMBIL] mulai ({kalib.mode_ambil}) -- "
                                f"berhenti di {jarak_ambil_cm(kalib):.0f} cm, "
                                f"lengan r={kalib.lengan_r:.0f} h={kalib.lengan_h:.0f} mm. "
                                f"ANGKAT TANGAN DI TOMBOL LEMAS.")
                            cocok, _, teks = periksa_geometri_lengan(kalib)
                            if kalib.mode_ambil == "lengan" and not cocok:
                                link.log.append(
                                    f"[AMBIL] ! GEOMETRI LENGAN TIDAK COCOK: {teks}. "
                                    f"Salah satu dari jangkauan/tinggi/vektor salah "
                                    f"-- lengan akan menjulur ke tempat yang bukan "
                                    f"tempat korban, tanpa pesan error dari firmware.")
                    elif k == "ambil_batal":
                        aksi.batal()
                        misi.dari_firmware = False
                        link.kirim("s", armed)
                        misi.ganti(S_IDLE, "ambil dibatalkan operator")
                        link.log.append("[AMBIL] dibatalkan, robot dihentikan")
                    elif k == "capit_buka":
                        link.kirim("g100", armed)
                    elif k == "capit_tutup":
                        link.kirim("g10", armed)
                    elif k == "lengan_contoh":
                        link.kirim("a70 20", armed)
                    elif k == "lengan_off":
                        link.kirim("n", armed)
                    elif k == "lengan_pos" and v:
                        # Lengan DEPAN saja. 'A' ditolak firmware dengan sebab
                        # yang jelas: capit belakang tidak punya sendi sama
                        # sekali, letaknya ditentukan letak BADAN.
                        pose = {
                            "siap":   (kalib.lengan_r, kalib.lengan_h
                                       + kalib.lengan_angkat),
                            "jepit":  (kalib.lengan_r, kalib.lengan_h),
                            "angkat": (kalib.lengan_r, kalib.lengan_h
                                       + kalib.lengan_angkat * 2),
                        }.get(v)
                        if pose is None:
                            link.log.append(f"[LENGAN] pose '{v}' tidak dikenal")
                        else:
                            # Hanya dicatat kalau BENAR-BENAR terkirim. Baris
                            # log yang bilang lengan bergerak padahal perintahnya
                            # ditolak mode BACA itu kebohongan kecil yang mahal.
                            if link.kirim(f"a{pose[0]:.0f} {pose[1]:.0f}", armed):
                                link.log.append(
                                    f"[LENGAN] {v}: jangkauan {pose[0]:.0f} mm, "
                                    f"tinggi {pose[1]:.0f} mm (dari PUSAT BADAN)")
                    elif k == "pergelangan" and v:
                        try:
                            sudut = max(-90.0, min(90.0, float(v)))
                        except ValueError:
                            link.log.append("[LENGAN] sudut pergelangan tidak sah")
                        else:
                            # Argumen ketiga 'a' hanya disentuh kalau diminta --
                            # jadi jangkauan/tinggi harus ikut dikirim ulang,
                            # kalau tidak lengan tersentak ke pose lain.
                            if link.kirim(f"a{kalib.lengan_r:.0f} "
                                          f"{kalib.lengan_h:.0f} {sudut:.0f}",
                                          armed):
                                link.log.append(
                                    f"[LENGAN] pergelangan {sudut:+.0f} der")
                    elif k == "putar" and v:
                        bag = v.split("|")
                        cmd_txt, catatan = putar_manual(
                            bag[0], bag[1] if len(bag) > 1 else "auto",
                            misi, aksi, kalib)
                        if cmd_txt is None:
                            link.log.append(f"[PUTAR] ditolak -- {catatan}")
                        else:
                            link.log.append(f"[PUTAR] {cmd_txt}  {catatan}")
                    elif k == "kalib_jarak" and v:
                        # Menyelesaikan korban_tinggi_cm dari SATU pengukuran
                        # penggaris. Menolak dengan sebab yang spesifik, karena
                        # "gagal" saja akan membuat orang menekan tombolnya
                        # berulang-ulang tanpa tahu apa yang salah.
                        try:
                            d_ukur = float(v)
                        except ValueError:
                            link.log.append("[KALIB] jarak tidak sah")
                        else:
                            bb = misi.vis_bbox
                            if d_ukur <= 0:
                                link.log.append("[KALIB] jarak harus lebih dari 0")
                            elif bb is None:
                                link.log.append(
                                    "[KALIB] tidak ada sasaran KORBAN terlihat. "
                                    "Tekan 'Mulai JEJAK' dulu supaya sasaran dipilih.")
                            elif (bb[0] <= kalib.bbox_tepi_px
                                  or bb[1] >= bb[2] - kalib.bbox_tepi_px):
                                link.log.append(
                                    "[KALIB] bbox menyentuh tepi frame -- terpotong. "
                                    "Mundurkan korban sampai seluruh badannya masuk "
                                    "frame, lalu ukur ulang jaraknya.")
                            else:
                                try:
                                    tinggi = tinggi_dari_jarak(
                                        bb[0], bb[1], bb[3], d_ukur, kalib)
                                except ValueError as e:
                                    link.log.append(f"[KALIB] {e}")
                                else:
                                    # Ditulis ke posisi yang sedang dikerjakan
                                    # kalau ada, ke global kalau tidak. Jadi
                                    # kalibrasi K-1..K-5 tinggal: berdiri di
                                    # ruangnya, tekan tombolnya.
                                    pos = misi.vis_posisi or korban_kini(misi)
                                    medan = (f"tinggi_{pos.lower()}"
                                             if pos and hasattr(
                                                 kalib, f"tinggi_{pos.lower()}")
                                             else "korban_tinggi_cm")
                                    lama_t = getattr(kalib, medan)
                                    setattr(kalib, medan, round(tinggi, 2))
                                    h_px = bb[1] - bb[0]
                                    tol_cm = d_ukur / h_px * kalib.bbox_tol_px
                                    link.log.append(
                                        f"[KALIB] {medan} {lama_t:.2f} -> "
                                        f"{tinggi:.2f} cm (bbox {h_px:.0f} px "
                                        f"di {d_ukur:.0f} cm"
                                        + (f", posisi {pos}" if pos else ", global")
                                        + f"). Ambang {kalib.bbox_tol_px:.0f} px "
                                        f"= +/-{tol_cm:.2f} cm di jarak ini. "
                                        f"RAM saja -- simpan ke file kalau mau.")
                    elif k == "kamera":
                        pesan = kamera.saklar(not kamera.nyala, "lewat tombol HUD")
                        link.log.append(f"[KAMERA] {pesan}")
                    elif k == "tunggu_visi":
                        # m8 di firmware yang dipatch: ruas AMBIL parkir di
                        # KONFIRM dulu, jadi Pi punya jendela untuk meluruskan
                        # badan sebelum capit turun.
                        #
                        # Ini yang hilang saat diuji 12 Sep: pemicunya sampai,
                        # vision mendeteksi, tapi firmware tidak menunggu --
                        # jadi capit turun sendiri dan satu-satunya yang boleh
                        # dikerjakan HUD adalah MENONTON.
                        link.kirim("m8", armed)
                        link.log.append(
                            "[M8] balik mode 'tunggu vision' di firmware. Lihat "
                            "jawabannya di Riwayat perintah -- firmware yang "
                            "menyebut nyala atau mati, bukan HUD yang menebak. "
                            "Butuh firmware v1.12+patch; versi tanpa m8 akan "
                            "menjawab 'perintah tidak dikenal'.")
                    elif k == "uji_jangkau":
                        r, h = pose_jepit_mm(kalib)
                        cocok, dhit, teks = periksa_geometri_lengan(kalib)
                        if not cocok:
                            link.log.append(f"[UJI] ! geometri tidak cocok: {teks}")
                        if link.kirim(f"a{r:.0f} {h:.0f}", armed):
                            link.log.append(
                                f"[UJI] lengan -> jangkauan {r:.0f} mm, tinggi "
                                f"{h:.0f} mm. Kalau firmware menjawab 'Di luar "
                                f"jangkauan lengan', 25 cm melebihi IK-nya dan "
                                f"sudutnya TIDAK dikirim -- lengannya tidak "
                                f"bergerak, bukan bergerak salah.")
                    elif k == "hening":
                        # MATIKAN SEMUA ALIRAN CETAK. Bukan toggle buta:
                        # hanya yang keadaannya TERBACA hidup yang dikirimi
                        # huruf pematiannya. Mengirim 'y' ke aliran yang
                        # sudah mati justru MENYALAKANNYA -- kebalikan dari
                        # yang diminta, dan itu jenis tombol yang membuat
                        # orang berhenti memercayai tombol.
                        mati = []
                        for huruf, nama in ALIRAN.items():
                            if link.aliran.get(huruf):
                                link.kirim(huruf, armed, paksa=True)
                                mati.append(f"{nama} ('{huruf}')")
                        if mati:
                            link.log.append("[HENING] dimatikan: " + ", ".join(mati))
                        else:
                            link.log.append(
                                "[HENING] tidak ada aliran firmware yang hidup. "
                                "Kalau log masih ramai, itu poll HUD sendiri "
                                "(m+l tiap detik) -- pakai tombol Poll mati.")
                    elif k == "poll":
                        kalib.poll_hz = 0.0 if kalib.poll_hz > 0 else 1.0
                        link.log.append(
                            "[POLL] MATI -- jarak & status berhenti diperbarui"
                            if kalib.poll_hz == 0 else "[POLL] hidup 1x/detik")
                    elif k == "riwayat_hapus":
                        link.riwayat.clear()
                        link.log.append("[RIWAYAT] dikosongkan")
                    elif k == "pose_nol":
                        aksi.batal()
                        link.kirim("r0 0 0", armed, paksa=True)
                        link.kirim("t0 0 0", armed, paksa=True)
                        misi.badan_yaw = misi.badan_x = 0.0
                        link.log.append("[POSE] badan dinolkan (r0 0 0 + t0 0 0)")
                    elif k in ("uji1", "uji3", "ujiv"):
                        # Lompat langsung ke urutan ambil korban, melewati seluruh
                        # navigasi. Robot dianggap SUDAH ditaruh di depan boneka --
                        # itu yang mau diuji, bukan cara sampai ke sana.
                        for i, mm in enumerate(MISI):
                            if mm[2] == KORBAN:
                                misi.idx = i
                                break
                        misi.status = [BELUM] * len(MISI)
                        misi.status[misi.idx] = JALAN
                        misi.slot = 0
                        misi.hasil_slot = {}
                        misi.terkunci = None
                        misi.sebab = ""
                        misi.bearing_deg = None
                        misi._geser_dijadwal = False
                        misi.tanpa_gerak = (k == "ujiv")
                        misi.slot_override = 3 if k == "uji3" else 1
                        juri.reset()
                        aksi.batal()
                        misi.ganti(S_STANDOFF)
                        link.log.append(
                            f"[UJI] {misi.kini[0]}, {misi.slot_override} slot"
                            + (" -- TANPA GERAK, vision saja" if misi.tanpa_gerak
                               else " -- robot AKAN bergerak"))
                    elif k == "berdiri":
                        # 'w' saja tidak menggerakkan apa pun: firmware boot LEMAS
                        # dan hanya 'b' yang menyalakan PWM.
                        link.kirim("b", armed)
                    elif k == "mulai":
                        link.kirim("m1", armed)
                    elif k == "stop":
                        # STOP KERAS. Bukan cuma mengosongkan antrean -- itu
                        # yang dulu bikin robot jalan lagi sendiri sedetik
                        # kemudian. Sekarang mengunci FSM sampai dilepas.
                        misi.halt = True
                        misi.jeda = False
                        aksi.batal()
                        link.kirim("s", armed, paksa=True)
                        # Pose badan dipulangkan: badan yang ditinggal
                        # menyerong 15 der akan membuat perintah jalan
                        # berikutnya melenceng, dan sebabnya sulit ditebak.
                        link.kirim("r0 0 0", armed, paksa=True)
                        link.kirim("t0 0 0", armed, paksa=True)
                        misi.badan_yaw = misi.badan_x = 0.0
                        pid.reset(); diam.reset()
                        misi.sebab_henti = "STOP oleh operator"
                        misi.ganti(S_IDLE, misi.sebab_henti)
                        misi.bearing_deg = None
                        link.log.append("[STOP] semua otomasi DIKUNCI. "
                                        "Tekan 'Lepas STOP' untuk memakai lagi.")
                    elif k == "fokus":
                        kalib.fokus_vision = not kalib.fokus_vision
                        link.log.append(
                            f"[FOKUS] video {'MATI' if kalib.fokus_vision else 'nyala'}"
                            f" -- inferensi dan kendali tidak terpengaruh")
                    elif k == "log_ringkas":
                        link.log_ringkas = not link.log_ringkas
                        link.diam_sampai = 0.0
                        link.log.append(
                            f"[LOG] mode {'RINGKAS' if link.log_ringkas else 'PENUH'}"
                            + (f" -- {link.n_diam} baris rutin sudah disembunyikan"
                               if link.log_ringkas else ""))
                    elif k == "lepas_stop":
                        misi.halt = False
                        misi.jeda = False
                        misi.sebab_henti = ""
                        aksi.batal()
                        link.log.append("[STOP] dilepas -- FSM boleh jalan lagi")
                    elif k == "pause":
                        # Beda dengan STOP: state DISIMPAN, jadi bisa dilanjut.
                        misi.jeda = True
                        aksi.batal()
                        link.kirim("s", armed, paksa=True)
                        link.log.append(f"[PAUSE] dibekukan di state {misi.state}")
                    elif k == "resume":
                        if misi.halt:
                            link.log.append("[RESUME] masih HALT -- "
                                            "lepas STOP dulu")
                        else:
                            misi.jeda = False
                            # Jam state di-nolkan: kalau tidak, misi yang
                            # dijeda 2 menit langsung kena batas waktu begitu
                            # dilanjutkan, dan itu terbaca sebagai gagal palsu.
                            misi.t_state = time.time()
                            link.log.append(f"[RESUME] lanjut dari {misi.state}")
                    elif k == "abort":
                        # Batalkan misi SELURUHNYA, di kedua sisi. 'm0' penting:
                        # tanpa itu Mission di firmware masih hidup dan
                        # navUpdate() menyalakan navigasi lagi sendiri.
                        aksi.batal()
                        link.kirim("m0", armed, paksa=True)
                        link.kirim("s", armed, paksa=True)
                        link.kirim("r0 0 0", armed, paksa=True)
                        link.kirim("t0 0 0", armed, paksa=True)
                        misi.badan_yaw = misi.badan_x = 0.0
                        pid.reset(); diam.reset()
                        misi.halt = True
                        misi.jeda = False
                        misi.status[misi.idx] = GAGAL_
                        misi.sebab_henti = "misi DIBATALKAN operator"
                        misi.slot = 0
                        misi.hasil_slot = {}
                        misi.terkunci = None
                        misi.bearing_deg = None
                        misi._urut = None
                        misi._geser_dijadwal = False
                        juri.reset()
                        misi.ganti(S_IDLE, misi.sebab_henti)
                        link.log.append("[ABORT] misi dibatalkan (m0 + s), "
                                        "otomasi DIKUNCI")
                    elif k == "lemas":
                        # Satu-satunya pemakai paksa=True selain pengaman di
                        # atas. Robot AMBRUK -- dan otomasi ikut dikunci, kalau
                        # tidak FSM akan mengirim perintah gerak ke robot yang
                        # servonya baru saja dimatikan.
                        link.kirim("x", armed, paksa=True)
                        aksi.batal()
                        misi.halt = True
                        misi.jeda = False
                        misi.sebab_henti = "dilemaskan operator"
                        misi.gagal(misi.sebab_henti)
                        link.log.append(
                            "[LEMAS] 'x' dikirim. Firmware harus membalas "
                            "'Servo NONAKTIF (PWM mati, servo bebas).' -- "
                            "kalau baris itu TIDAK muncul, perintahnya tidak sampai.")
                    elif k == "kalib" and v and "|" in v:
                        nilai, nama = v.split("|", 1)
                        if nama in KALIB_FIELDS:
                            lama = getattr(kalib, nama)
                            try:
                                if isinstance(lama, bool):
                                    baru = nilai.strip().lower() in ("1", "true", "ya")
                                elif isinstance(lama, str):
                                    # Field teks TIDAK bisa lewat float().
                                    # Dulu semuanya jatuh ke float() dan field
                                    # teks selalu ditolak "nilai tidak sah".
                                    baru = nilai.strip()
                                    sah = KALIB_PILIHAN.get(nama)
                                    if sah and baru not in sah:
                                        raise ValueError(f"pilihannya {sah}")
                                else:
                                    baru = type(lama)(float(nilai))
                                setattr(kalib, nama, baru)
                                link.log.append(f"[KALIB] {nama}: {lama} -> {baru}")
                            except ValueError:
                                link.log.append(f"[KALIB] nilai tidak sah: {nilai}")
                    elif k == "kirim_condong":
                        # 'Q<nama> <nilai>' menyetel satu parameter, 'W'
                        # menulis seluruh tabel ke EEPROM. Urutannya wajib
                        # begitu: tanpa 'W' angkanya hilang saat Teensy reset,
                        # dan itu persis keadaan yang hendak dihindari --
                        # menyetel di arena lalu kehilangannya.
                        for nm, nilai in (("condong.mm", kalib.condong_mm),
                                          ("condong.jeda", kalib.condong_jeda_ms),
                                          ("condong.yaw", kalib.condong_yaw)):
                            aksi.jadwal((f"Q{nm} {nilai:g}", 0.3))
                        aksi.jadwal(("W", 0.6))
                        pesan_kalib = ("condong.mm/jeda/yaw dikirim + 'W' -- "
                                       "tersimpan di EEPROM Teensy")
                        link.log.append(f"[KALIB] {pesan_kalib}")
                    elif k == "simpan_kalib":
                        # SATU-SATUNYA penulisan berkas di seluruh program.
                        isi = {f: getattr(kalib, f) for f in KALIB_FIELDS}
                        # DUA BERKAS. Yang bertanggal adalah riwayat -- ia tidak
                        # pernah ditimpa, jadi setelan yang ternyata lebih baik
                        # kemarin masih bisa diambil lagi. Yang TETAP adalah
                        # yang dimuat otomatis tiap HUD start.
                        #
                        # Autoload ini membalik keputusan lama "tanpa autoload,
                        # tanpa autosave". Sebabnya berubah: sejak kalibrasi
                        # K-3/K-4 punya sembilan angka yang disetel di arena,
                        # kehilangan semuanya tiap layanan restart lebih mahal
                        # daripada risiko memuat angka basi. Yang dimuat SELALU
                        # dicetak saat start, jadi angka basi tidak pernah
                        # senyap. '--tanpa-simpanan' mematikannya.
                        nama = f"kalib_{time.strftime('%Y%m%d_%H%M%S')}.json"
                        with open(nama, "w") as fh:
                            json.dump(isi, fh, indent=2)
                        with open(KALIB_AKTIF, "w") as fh:
                            json.dump(isi, fh, indent=2)
                        pesan_kalib = (f"disimpan ke {os.path.abspath(nama)} DAN "
                                       f"{KALIB_AKTIF} (yang ini dimuat otomatis)")
                        link.log.append(f"[KALIB] {pesan_kalib}")

                # --- keluaran ---
                n_fps += 1
                if time.time() - t_fps >= 0.5:
                    stats["fps"] = n_fps / (time.time() - t_fps)
                    t_fps, n_fps = time.time(), 0

                if ada_kamera and not kalib.fokus_vision:
                    anotasi(frame, dets, lolos, kalib, names,
                            jejak=(misi.state in (S_JEJAK, S_CENTER, S_A_TENGAH)),
                            alasan=juri.alasan)
                st = rakit_state(misi, link, kalib, juri, stats,
                                 armed, pesan_kalib, kamera, bersama, sehat)
                st["galat"] = galat
                bersama.set_state(st)

                # JPEG dibatasi ~15 Hz DAN hanya saat ada yang menonton. Enkode
                # 1280x720 tiap frame memakan inti yang seharusnya untuk inferensi;
                # selama 300 detik lomba tidak ada yang menatap layar, jadi di situ
                # ongkosnya turun ke nol tanpa kamu perlu mematikan apa pun.
                if (not kalib.fokus_vision and bersama.perlu_jpeg()
                        and time.time() - t_kirim > 1 / 15):
                    t_kirim = time.time()
                    kecil = cv2.resize(frame, None, fx=args.stream_scale,
                                       fy=args.stream_scale) if args.stream_scale != 1 else frame
                    ok, buf = cv2.imencode(".jpg", kecil,
                                           [cv2.IMWRITE_JPEG_QUALITY, args.stream_quality])
                    if ok:
                        bersama.set_jpeg(buf.tobytes())

                if args.window:
                    cv2.imshow("R2C HEXAPOD - MISSION HUD", frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
            except Exception:
                # Satu galat tidak boleh membunuh loop diam-diam. Dulu kalau
                # ini terjadi, halaman tetap tersaji tapi SELURUH isinya kosong
                # tanpa satu pun petunjuk kenapa. Sekarang jejaknya muncul
                # sebagai pita merah di atas halaman DAN di journalctl.
                galat = traceback.format_exc()
                print(galat, flush=True)
                link.log.append('[GALAT] ' + galat.strip().splitlines()[-1])
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        # Sengaja TIDAK mengirim apa pun saat keluar. main_deploy.py yang lama
        # mengirim 'S' di sini -- di firmware ini itu MENULIS EEPROM.
        srv.shutdown()
        kamera.lepas()
        cv2.destroyAllWindows()
        print("\n[HUD] selesai. Tidak ada perintah yang dikirim saat keluar.")


if __name__ == "__main__":
    main()
