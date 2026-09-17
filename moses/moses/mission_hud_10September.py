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
S_AWAS = "AWAS_KORBAN"
S_A_SIAP = "AMBIL_SIAP"
S_A_JEPIT = "AMBIL_JEPIT"
S_A_ANGKAT = "AMBIL_ANGKAT"
S_GAGAL = "GAGAL"

# state -> (penjelasan singkat, state berikutnya kalau lancar)
FSM = {
    S_JEJAK:     ("ikuti korban, tengahkan terus-menerus",        S_STANDOFF),
    S_KONFIRM:   ("firmware menunggu m2/m3 - vision menilai",     S_IDLE),
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
             S_AWAS}

# Rantai ambil korban, urut. Dipakai HUD untuk tahu 'sedang mengambil'.
RANTAI_AMBIL = (S_A_TENGAH, S_A_MAJU, S_A_HALUS, S_A_SIAP, S_A_JEPIT, S_A_ANGKAT)

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
BATAS = {S_STANDOFF: 20, S_CENTER: 20, S_LIHAT: 8, S_SLOT: 25,
         S_DEKATI: 25, S_CENGKERAM: 15, S_VERIF: 10,
         S_A_TENGAH: 30, S_A_MAJU: 30, S_A_HALUS: 25, S_KONFIRM: 25, S_A_SIAP: 10, S_A_JEPIT: 8,
         S_A_ANGKAT: 10}


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
    lengan_r: float = 70.0      # jangkauan lengan, mm dari PUSAT BADAN
    lengan_h: float = 20.0      # tinggi lengan saat menjepit, mm
    lengan_angkat: float = 25.0  # tambahan tinggi saat mengangkat, mm
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
    # Default MATI: menjawab otomatis berarti robot melanjutkan misi karena
    # kamera bilang begitu, dan itu keputusan yang harus dinyalakan sadar.
    auto_konfirm: bool = False

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
    tengah_tol_px: float = 8.0
    # PITA BAWAH YANG DITUTUPI CAPIT. Saat lengan diturunkan, dia menutupi
    # bagian bawah frame. Deteksi yang PUSATNYA di pita itu dibuang, bukan
    # dipercaya -- capit oranye di depan lensa itu justru mirip korban.
    # 0 = tidak ada yang ditutupi. 0,45 = 45% bawah frame diabaikan.
    # Ukur sekali: turunkan lengan, lihat stream, catat di ketinggian berapa
    # capit mulai terlihat.
    roi_bawah_frac: float = 0.0
    badan_yaw_maks: float = 18.0   # < BODY_MAX_ROT_DEG (20) firmware
    badan_geser_maks: float = 35.0 # < BODY_MAX_TRANS_MM (40) firmware
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

    def px_per_deg(self, w):
        return w / self.hfov_deg

    def roi_half_px(self, w):
        return self.roi_half_deg * self.px_per_deg(w)


KALIB_FIELDS = [f for f in Kalib.__dataclass_fields__
                if f not in ("kelas_korban", "kelas_dummy")]

# Field bertipe TEKS: dibuat daftar pilihan, bukan kotak ketik bebas.
# Salah ketik satu huruf di sini diam-diam mengubah metode kendali robot.
KALIB_PILIHAN = {"metode_tengah": ["diam", "sekali", "pid", "p"]}


# =====================================================================
# 4. SAMBUNGAN KE TEENSY
# ---------------------------------------------------------------------
# Firmware Vincent memakai huruf besar-kecil sebagai perintah BERBEDA.
# Whitelist di bawah bukan kehati-hatian berlebihan: main_deploy.py yang lama
# mengirim 'S','F','L','R' -- di firmware ini 'S' MENULIS EEPROM dan 'F'
# membuat robot BERJALAN. Yang menjaga adalah whitelist, bukan niat baik.
# =====================================================================
PERINTAH_BACA = {"m", "v", "l", "d", "D", "k", "K", "q", "h", "M"}

TERLARANG = {
    "S": "SIMPAN kalibrasi pivot ke EEPROM 2048",
    "W": "SIMPAN blob parameter ke EEPROM 0",
    "e": "SIMPAN kompas arena ke EEPROM 1792",
    "C": "kalibrasi pivot -- MEMBLOKIR loop firmware sampai selesai",
    "F": "jalan ikut dinding KANAN (robot langsung berjalan)",
    "f": "jalan ikut dinding KIRI (robot langsung berjalan)",
    "p": "ikut dinding kiri + kunci arena",
    "P": "ikut dinding kanan + kunci arena",
    "B": "demo sapuan 6 sumbu, 18 detik",
    "z": "goyang badan terus-menerus (mode pajangan)",
    "x": "LEMAS -- semua PWM mati, robot ambruk",
}


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


def buka_kamera(sumber, w, h, fps):
    """-> (cap, pesan). cap None kalau tidak ada yang bisa dipakai.

    open_camera() milik detect.py menganggap "terbuka" sudah cukup. Di Pi itu
    tidak cukup: node ISP bawaan board (video19..22) mau dibuka tapi tidak
    pernah memberi frame. Jadi syarat lulusnya di sini adalah satu frame
    sungguhan.

    TIDAK melempar SystemExit dengan sengaja. Kamera yang belum tercolok
    bukan alasan mematikan HUD -- justru saat itulah kamu paling butuh
    halamannya untuk tahu apa yang salah.
    """
    kandidat = daftar_video() if str(sumber) == "auto" else [int(sumber)]
    for i in kandidat:
        try:
            cap = open_camera(i, w, h, fps)
        except SystemExit:
            continue
        except Exception:                                # noqa: BLE001
            continue
        try:
            ok, _ = cap.read()
        except Exception:                                # noqa: BLE001
            ok = False
        if ok:
            return cap, f"/dev/video{i}"
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

    def __init__(self, sumber, w, h, fps, jeda=3.0):
        self.args = (sumber, w, h, fps)
        self.jeda = jeda
        self.cap = None
        self.thread = None
        self.pesan = "mencari kamera..."
        self._t = 0.0

    @property
    def ok(self):
        return self.thread is not None and self.thread.running

    def pastikan(self):
        """Sambungkan kalau belum; menahan diri `jeda` detik antar percobaan."""
        if self.ok:
            return True
        if self.thread is not None:                      # mati di tengah jalan
            self.lepas()
            self.pesan = "kamera terputus -- mencoba sambung ulang"
        if time.time() - self._t < self.jeda:
            return False
        self._t = time.time()
        cap, pesan = buka_kamera(*self.args)
        self.pesan = pesan
        if cap is None:
            return False
        self.cap = cap
        self.thread = CameraThread(cap)
        print(f"[KAMERA] {pesan} dipakai")
        return True

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
            self.ser = serial.Serial(port, self.baud, timeout=0)
            self.sebab = ""
            self._catat(f"[LINK] tersambung {port} @ {self.baud}")
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
    def kirim(self, cmd, armed, paksa=False, rutin=False):
        """Satu-satunya pintu keluar. Menolak perintah berbahaya, selalu.

        `paksa` HANYA untuk aksi darurat yang ditekan operator secara sadar
        dari halaman (tombol LEMAS). FSM tidak pernah memakainya -- kalau
        dibiarkan bisa dipakai otomatis, whitelist ini kehilangan gunanya.
        """
        huruf = cmd[0] if cmd else ""
        if huruf in TERLARANG and not paksa:
            self.log.append(f"[TOLAK] '{cmd}' = {TERLARANG[huruf]}")
            return False
        if paksa:
            self.log.append(f"[PAKSA] '{cmd}' -- ditekan operator")
        if cmd not in PERINTAH_BACA and not armed and not paksa:
            self.log.append(f"[TOLAK] '{cmd}' butuh MODE KENDALI")
            return False
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
        if self.hidup:
            try:
                self.ser.write((cmd + "\n").encode())
            except Exception as e:                       # noqa: BLE001
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
                if self.log_ringkas and time.time() < self.diam_sampai:
                    self.n_diam += 1
                else:
                    self.log.append(baris)

    # Format yang di-parse mengikuti Mission::status() dan tabel 'l'.
    RE_KV = re.compile(r"^\s{2,}([a-zA-Z ]+?)\s*:\s*(.+)$")
    RE_LIDAR = re.compile(r"^\s*([0-5])\s+([A-Z\-]+)\s*:\s*(.+?)(?:\s{2,}\[|$)")
    RE_CM = re.compile(r"(-?\d+(?:[.,]\d+)?)\s*cm")

    def _parse(self, baris):
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
        self.t_state = time.time()
        self.sebab = ""
        self.slot = 0
        self.hasil_slot = {}
        self.terkunci = None
        self.bearing_deg = None
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
        self.tembak_err = None
        self.tembak_yaw = None
        self.awas_kelas = ""
        self.awas_conf = 0.0
        self.badan_yaw = 0.0
        self.badan_x = 0.0
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
    # Toleransi dihitung dari PIKSEL, bukan derajat: itu satuan yang benar-benar
    # kamu lihat di layar, dan tidak ikut berubah kalau resolusi kamera diganti.
    tol = min(kalib.tengah_tol_deg,
              kalib.tengah_tol_px / max(kalib.px_per_deg(lebar), 1e-6))
    if abs(err) <= tol:
        return True
    if not aksi.kosong():
        return False

    if abs(err) > kalib.yaw_kasar_deg:
        # Anggaran yaw badan dipulangkan dulu supaya tingkat halus punya ruang
        # penuh lagi sesudah pivot. Kalau tidak, badan bisa mentok di 18 der
        # dan koreksi halus berikutnya tidak punya sisa sama sekali.
        minta = pivot_der(err * kalib.pivot_gain, kalib)
        if abs(misi.badan_yaw) > 0.1:
            aksi.jadwal(("r0 0 0", 0.8))
            misi.badan_yaw = 0.0
        aksi.jadwal((f"O{-minta:d}", 2.5), ("l", 0.3))
        return False

    # --- halus: putar badan ---
    if kalib.metode_tengah == "diam" and diam is not None:
        # Bacaan yang lahir sebelum gerakan terakhir selesai DIBUANG di
        # tambah(); yang lolos dirata-rata. Selama belum cukup, JANGAN
        # bergerak -- diam sebentar jauh lebih murah daripada bergerak ke
        # arah yang salah lalu harus dikoreksi balik.
        if not diam.tambah(err):
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


def langkah_fsm(misi: Misi, link: Teensy, aksi: Aksi, kalib: Kalib, armed,
                pid=None, lebar=1280, diam=None):
    """Satu langkah sub-FSM. Dipanggil tiap frame, tidak pernah memblokir."""
    # GERBANG PALING ATAS, sebelum apa pun -- termasuk sebelum batas waktu.
    # Kalau di bawah pemeriksaan batas waktu, robot yang di-STOP masih bisa
    # "gagal karena timeout" dan pindah state sendiri saat berhenti.
    if misi.halt or misi.jeda:
        return
    batas = BATAS.get(misi.state, 0)
    if batas and misi.lewat > batas and misi.state not in (S_GAGAL, S_IDLE, S_TUNGGU):
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
        # KASAR saja. Menghaluskan di sini percuma: sesudah ini robot BERJALAN
        # maju, dan berjalan menghapus penengahan sehalus apa pun. Yang halus
        # dikerjakan di S_A_HALUS, sesudah robot berhenti di jarak capit.
        if abs(err) <= kalib.yaw_kasar_deg:
            if aksi.kosong():
                netralkan_badan(misi, aksi)   # jalan HARUS dengan badan netral
                if pid is not None:
                    pid.reset()               # tahap halus mulai dari nol
                if diam is not None:
                    diam.reset()
                misi.tembak_err = misi.tembak_yaw = None
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
        if pusatkan(misi, aksi, kalib, err, pid, lebar, diam) and aksi.kosong():
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
        delta = d - kalib.capit_cm
        if abs(delta) <= kalib.capit_tol_cm:
            misi.ganti(S_A_HALUS)
            return
        if delta < 0:
            # Firmware tidak punya mundur. Ini batas nyata, bukan bug.
            misi.gagal(f"kelewat dekat ({d:.0f} cm, target {kalib.capit_cm:.0f}) "
                       f"-- mundurkan robot dengan tangan")
            return
        # Sisakan sedikit supaya tidak menabrak: maju paling banyak 6 cm sekali
        # jalan, lalu ukur lagi. Odometri gait meleset beberapa cm per langkah.
        maju = min(delta, 6.0)
        aksi.jadwal((f"D{maju:.0f}", 0.3), ("w", 1.0), ("l", 0.5))
        return

    if misi.state == S_A_SIAP:
        if misi._urut != S_A_SIAP:
            aksi.jadwal(("g100", 0.8),
                        (f"a{kalib.lengan_r:.0f} {kalib.lengan_h:.0f}", 1.5))
            misi._urut = S_A_SIAP
        elif aksi.kosong():
            misi.ganti(S_A_JEPIT)
        return

    if misi.state == S_A_JEPIT:
        if misi._urut != S_A_JEPIT:
            aksi.jadwal(("g0", 1.5))
            misi._urut = S_A_JEPIT
        elif aksi.kosong():
            misi.ganti(S_A_ANGKAT)
        return

    if misi.state == S_A_ANGKAT:
        if misi._urut != S_A_ANGKAT:
            aksi.jadwal((f"a{kalib.lengan_r:.0f} "
                         f"{kalib.lengan_h + kalib.lengan_angkat:.0f}", 2.0))
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
        if abs(delta) <= kalib.standoff_tol_cm:
            misi.ganti(S_CENTER)
            return
        if delta < 0:
            # Tidak ada perintah mundur di firmware. Batas nyata, bukan bug.
            misi.gagal(f"terlalu dekat ({d:.0f} cm) -- firmware belum punya mundur")
            return
        if aksi.kosong():
            aksi.jadwal((f"D{delta:.0f}", 0.3), ("w", 1.0), ("l", 0.4))
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
        # Geser 8 cm memakai heading ARENA (o0..o3), bukan pivot relatif --
        # arah absolut tidak menumpuk galat seperti O+90 lalu O-90 berkali-kali.
        if not getattr(misi, "_geser_dijadwal", False):
            aksi.jadwal(("o1", 3.0),
                        (f"D{kalib.slot_step_cm:.0f}", 0.3), ("w", 2.5),
                        ("o3", 3.0), ("l", 0.5))
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
<div id=mode class=baca>&nbsp;</div>
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
      <div class=kecil id=log_info></div><div id=log></div></div>
  </div>

  <div>
    <div id=tabs>
      <button class=aktif onclick="tab(0,this)">Robot</button>
      <button onclick="tab(1,this)">Uji korban</button>
      <button onclick="tab(2,this)">Manual</button>
      <button onclick="tab(3,this)">Misi</button>
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
        <button data-tip="BACA: HUD hanya membaca (m, l), tidak satu pun perintah gerak dikirim. KENDALI: FSM dan tombol boleh menggerakkan robot." onclick="cmd('mode')">Ganti mode BACA / KENDALI</button>
        <button class=danger data-tip="Menjalankan ulang mission_hud.py dari awal tanpa SSH. Kamera dan serial dibuka ulang, kalibrasi RAM kembali ke default. Halaman memuat sendiri sesudah 6 detik." onclick="restartApp()">Restart program</button>
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
        <button data-tip="Cetak status misi: state, ruas ke berapa dari 0..33, nama ruasnya, dan sisa syarat. Aman." onclick="cmd('man','m')">Status misi (m)</button>
        <button data-tip="Mulai misi dari ruas 0. Firmware memeriksa servo, mux, LiDAR, IMU, kompas dan kalibrasi pivot lebih dulu, lalu mencetak persis mana yang gagal." onclick="if(confirm('Mulai misi dari ruas 0. Robot AKAN berjalan. Lanjut?'))cmd('mulai')">Mulai misi (m1)</button>
        <button class=danger data-tip="Batalkan misi di firmware. Robot berhenti, navigasi mati." onclick="cmd('man','m0')">Batalkan misi (m0)</button>
        <table>
          <tr><td class=mid width=90>ruas firmware</td><td id=ruas_fw>-</td></tr>
          <tr><td class=mid>vision</td><td id=awas>-</td></tr>
        </table>
      </div>
      <div class=card><h2>Kalibrasi vision</h2>
        <p class=kecil>Semua knob-nya ada di tab <b>Kalibrasi</b> dan berlaku
        <b>seketika</b> tanpa restart. Yang belum ada cuma urutannya &mdash; ini dia.</p>
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
        <b>5. Toleransi (<code>tengah_tol_px</code>).</b> Mulai 8 px. Kalau robot
        tidak pernah mengendap, longgarkan &mdash; bukan naikkan gain.</p>
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
      <div class=card><h2>Jejak korban</h2>
        <button data-tip="Kamera menyala terus. Boneka boleh digeser ke mana saja; robot memutar badan mengikutinya sampai simpangan di bawah toleransi, lalu berhenti menunggu perintahmu. Gerbang ROI dimatikan dan pita tinggi dilonggarkan, jadi jarak bebas." onclick="cmd('jejak')">Mulai JEJAK</button>
        <button data-tip="Sekali tekan: baca jarak LiDAR depan, hitung selisih ke standoff_cm, kirim D<selisih> lalu w. Rem jarak firmware yang menghentikannya." onclick="cmd('maju')">Maju ke korban</button>
        <button data-tip="Firmware: g100. Membuka penjepit lengan DEPAN penuh." onclick="cmd('capit_buka')">Capit BUKA</button>
        <button data-tip="Firmware: g0. Menutup penjepit lengan DEPAN penuh." onclick="cmd('capit_tutup')">Capit TUTUP</button>
        <button data-tip="Firmware: n. Mematikan servo KEDUA lengan. Kaki tidak terpengaruh." onclick="cmd('lengan_off')">Lengan OFF</button>
        <table>
          <tr><td class=mid width=80>sasaran</td><td id=sasaran>-</td></tr>
          <tr><td class=mid>simpangan</td><td id=bearing>-</td></tr>
          <tr><td class=mid>deteksi</td><td id=deteksi>-</td></tr>
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
          <tr><td class=mid>jarak depan</td><td id=ambil_depan>-</td></tr>
          <tr><td class=mid>target capit</td><td id=ambil_target>-</td></tr>
          <tr><td class=mid>pose badan</td><td id=ambil_badan>-</td></tr>
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

    <!-- 1. UJI KORBAN -->
    <div class=panel>
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
      <div class=card id=stbox><h2>State machine</h2>
        <div class=big id=state2>-</div>
        <div class=mid id=stdesc></div>
        <table>
          <tr><td class=mid width=80>berikut</td><td id=next2></td></tr>
          <tr><td class=mid>di state</td><td id=lewat></td></tr>
          <tr><td class=mid>slot</td><td id=slot></td></tr>
          <tr><td class=mid>penonton</td><td id=penonton></td></tr>
        </table>
        <div id=gagalbox></div>
      </div>
    </div>

    <!-- 2. MANUAL -->
    <div class=panel>
      <div class=card><h2>Kirim perintah langsung</h2>
        <button data-tip="Berjalan maju sampai ditekan Stop. Firmware: w. Butuh servo menyala (b)." onclick="cmd('man','w')">Maju (w)</button>
        <button data-tip="Firmware: s. Berhenti, servo tetap hidup." onclick="cmd('man','s')">Stop (s)</button>
        <button data-tip="Putar 15 derajat ke KIRI di tempat. Firmware: O15." onclick="cmd('man','O15')">&#8634; 15&deg;</button>
        <button data-tip="Putar 15 derajat ke KANAN. Firmware: O-15." onclick="cmd('man','O-15')">&#8635; 15&deg;</button>
        <button data-tip="Putar 90 derajat ke KIRI. Firmware: O90." onclick="cmd('man','O90')">&#8634; 90&deg;</button>
        <button data-tip="Putar 90 derajat ke KANAN. Firmware: O-90." onclick="cmd('man','O-90')">&#8635; 90&deg;</button>
        <br>
        <button data-tip="Hadap UTARA menurut kompas arena. Firmware: o0. Butuh kompas sudah dicatat (c0..c3 lalu e)." onclick="cmd('man','o0')">Hadap U</button>
        <button data-tip="Hadap TIMUR. Firmware: o1." onclick="cmd('man','o1')">T</button>
        <button data-tip="Hadap SELATAN. Firmware: o2." onclick="cmd('man','o2')">S</button>
        <button data-tip="Hadap BARAT. Firmware: o3." onclick="cmd('man','o3')">B</button>
        <br>
        <button data-tip="Status navigasi + jarak sekitar. Aman, hanya membaca. Firmware: v." onclick="cmd('man','v')">Status nav (v)</button>
        <button data-tip="Tabel jarak keenam LiDAR. Aman. Firmware: l." onclick="cmd('man','l')">LiDAR (l)</button>
        <button data-tip="Dump diagnostik: status PWM, ServoMap, sudut & pulse per kaki. Aman. Firmware: d." onclick="cmd('man','d')">Diagnostik (d)</button>
        <p class=kecil style="color:var(--warn)"><b>Awas huruf O vs angka 0.</b>
        Perintah belok itu huruf <b>O</b> besar. Kalau kamu mengetik <b>angka nol</b>
        (<code>015</code>), firmware membaca perintah <code>0</code> = "nolkan pose badan",
        mencetak <code>Pose badan dinolkan.</code> lalu tidak bergerak sama sekali &mdash;
        gejalanya <b>persis sama</b> dengan IMU mati. Periksa log serialnya dulu.</p>
        <div class=mid>Diagnosa IMU &mdash; cek ini kalau MAJU bisa tapi BELOK tidak</div>
        <button data-tip="Tabel lintasan lengkap. Firmware BARU (lebih baru dari v1.8) punya tabel ruas 0..31 -- ini yang mencetaknya. Aman, hanya membaca." onclick="cmd('man','m4')">Tabel lintasan (m4)</button>
        <button data-tip="Daftar SELURUH perintah yang dikenali firmware yang SEDANG terpasang. Ini cara paling cepat tahu apakah sudah ada perintah geser samping / mundur. Aman." onclick="cmd('man','h')">Daftar perintah (h)</button>
        <button data-tip="Nyala/mati aliran cetak yaw. Aman, hanya mencetak. UJI PALING MENENTUKAN: kalau tidak ada satu pun baris yaw muncul, IMU mati -- dan SEMUA pivot (O, o0..o3, mode arena, m1) tidak akan pernah jalan. Tekan lagi untuk mematikan." onclick="cmd('man','y')">Aliran yaw (y) &mdash; uji IMU</button>
        <button class=danger data-tip="Pivot ke UTARA. BUKAN sekadar diagnosa -- robot AKAN berputar kalau IMU hidup. Gunanya: saat IMU MATI perintah ini MENCETAK 'Gagal: Tidak ada data IMU.', sedangkan O tidak mencetak apa pun sama sekali." onclick="if(confirm('o0 akan MEMUTAR robot kalau IMU hidup. Lanjut?'))cmd('man','o0')">o0 &mdash; paksa pesan error IMU</button>
        <div style="margin-top:8px">
          <input class=lebar id=cmdbox placeholder="perintah, mis. b60"
                 onkeydown="if(event.key==='Enter')kirimManual()">
          <button data-tip="Kirim lewat whitelist yang sama. Perintah berbahaya tetap ditolak." onclick="kirimManual()">Kirim</button>
          <button class=danger data-tip="Kirim MENEMBUS whitelist. Untuk perintah yang sengaja diblokir (f F p P C W S e B z). Baca dulu artinya di README." onclick="kirimPaksa()">Kirim PAKSA</button>
        </div>
        <div class=mid style="margin-top:6px">Semua tombol di sini butuh MODE KENDALI.
          Arahkan kursor untuk penjelasannya.</div>
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

    <!-- 3. MISI -->
    <div class=panel>
      <div class=card><h2>Urutan misi</h2>
        <p class=kecil>Ini <b>rencana lomba</b>, bukan daftar kemampuan. HUD
        <b>tidak punya kode navigasi</b> &mdash; berpindah ruang, menyeberang lantai
        pecah, menuruni bidang miring, semuanya milik <code>Mission.cpp</code> di
        Teensy. Baris bertanda <span style="color:var(--warn)">firmware belum ada</span>
        cuma bisa dilewati sebagai catatan; robotnya harus kamu pindahkan sendiri.</p>
        <ul id=misi></ul></div>
    </div>

    <!-- 4. KALIBRASI -->
    <div class=panel>
      <div class=card><h2>Kalibrasi <span class=mid>(RAM, tanpa autosave)</span></h2>
        <div id=kalib></div>
        <button onclick="simpanKalib()">Simpan kalibrasi ke file</button>
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
function kirimManual(){const b=$('cmdbox');if(!b.value.trim())return;cmd('man',b.value.trim());b.value='';}
function kirimPaksa(){const b=$('cmdbox');if(!b.value.trim())return;
  if(!confirm('Kirim MENEMBUS whitelist: '+b.value+'\n\nPerintah ini sengaja diblokir. Yakin?'))return;
  cmd('manpaksa',b.value.trim());b.value='';}
function restartApp(){if(!confirm('Restart mission_hud.py?\n\nHalaman kosong ~6 detik lalu muat sendiri.'))return;
  cmd('restart');setTimeout(()=>location.reload(),6000);}
async function simpanKalib(){const r=await fetch('/cmd?k=simpan_kalib',{method:'POST'});alert(await r.text());}
function esc(s){return String(s).replace(/[<>&]/g,c=>({'<':'&lt;','>':'&gt;','&':'&amp;'}[c]));}
async function tarik(){
  let d; try{d=await(await fetch('/state')).json()}catch(e){return}
  if(!d||!d.misi){$('galat').style.display='block';
    $('galat').textContent='Loop utama belum mengisi data. Cek: journalctl -u r2c-hud -n 50';return}
  $('galat').style.display=d.galat?'block':'none'; if(d.galat)$('galat').textContent=d.galat;
  $('versi').textContent='v '+d.versi;
  $('mode').className=d.armed?'kendali':'baca';
  $('mode').textContent=d.armed?'MODE KENDALI - ROBOT BISA BERGERAK':'MODE BACA - robot tidak digerakkan';
  // State yang belum diprogram diberi warna kuning DAN label. Tanpa label,
  // robot yang diam di sini tidak bisa dibedakan dari robot yang sedang
  // menunggu sensor -- dan operator menunggu sesuatu yang tidak akan datang.
  $('state').textContent = d.state + (d.state_belum? '  (BELUM DIPROGRAM)' : '');
  $('state').style.color = d.state=='GAGAL' ? 'var(--bad)'
                         : d.state_belum    ? 'var(--warn)' : 'var(--ok)';
  $('state2').textContent=$('state').textContent;
  $('state2').style.color=$('state').style.color;
  $('next').style.color = d.next_belum ? 'var(--warn)' : '';
  $('next').textContent=d.next; $('next2').textContent=d.next;
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
  $('sasaran').textContent=d.sasaran; $('sasaran').style.color=d.sasaran_warna;
  $('sasaran').style.fontWeight='700';
  $('bearing').textContent=d.bearing;
  $('bearing').style.color=d.bearing_ok?'var(--ok)':'var(--warn)';
  $('ambil_langkah').textContent=d.ambil_langkah;
  $('ambil_langkah').style.color=d.ambil_aktif?'var(--warn)':'var(--dim)';
  $('ambil_langkah').style.fontWeight=d.ambil_aktif?'700':'400';
  $('ambil_depan').textContent=d.ambil_depan;
  $('ambil_target').textContent=d.ambil_target;
  $('ambil_badan').textContent=d.ambil_badan;
  $('ambil_badan').style.color=/netral/.test(d.ambil_badan)?'var(--dim)':'var(--warn)';
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
  d.kalib.forEach(function(k){
    var el=$('kb_'+k.nama);
    if(!el || el===document.activeElement) return;   // jangan ganggu yang diketik
    if(el.value!=String(k.nilai)) el.value=k.nilai;
  });
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
                while True:
                    buf = bersama.get_jpeg()
                    if buf is None or buf is terakhir:
                        time.sleep(0.02)
                        continue
                    terakhir = buf
                    self.wfile.write(b"--bingkai\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(buf)).encode()
                                     + b"\r\n\r\n" + buf + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                              # tab ditutup -- wajar
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
        "ambil_badan": (f"yaw {misi.badan_yaw:+.1f} der  "
                        f"geser {misi.badan_x:+.0f} mm"
                        + ("   (netral)" if abs(misi.badan_yaw) < 0.05
                           and abs(misi.badan_x) < 0.05 else "   AKTIF")),
        "ambil_target": (f"{kalib.capit_cm:.0f} cm "
                         f"(+/- {kalib.capit_tol_cm:.1f})   "
                         f"lengan r={kalib.lengan_r:.0f} h={kalib.lengan_h:.0f} mm"),
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
    p.add_argument("--calib", default=None, help="muat kalibrasi dari JSON (TIDAK otomatis)")
    p.add_argument("--window", action="store_true", help="tambah window lokal (VNC/HDMI)")
    p.add_argument("--auto", action="store_true", help="mulai di MODE KENDALI")
    args = p.parse_args()

    kalib = Kalib()
    if args.calib:
        with open(args.calib) as f:
            for k, v in json.load(f).items():
                if hasattr(kalib, k):
                    setattr(kalib, k, v)
        print(f"[KALIB] dimuat dari {args.calib}")
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

    armed = args.auto
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
                        if misi.state in (S_JEJAK, S_CENTER, S_A_TENGAH):
                            # Mengejar: jangan pakai ROI (sasaran memang di pinggir) dan
                            # pita tinggi longgar (jaraknya belum diatur).
                            lolos = juri.saring(dets, w, h, pakai_roi=False,
                                                h_min=kalib.jejak_h_min,
                                                h_max=kalib.jejak_h_max)
                            lolos_lama = lolos
                        else:
                            # Menilai: gerbang ketat, karena di sinilah jarak sudah
                            # dikunci 20 cm dan tetangga 8 cm harus dibuang.
                            lolos = juri.saring(dets, w, h)
                        lolos_lama = lolos

                        if misi.state in (S_JEJAK, S_A_TENGAH):
                            if lolos:
                                t = max(lolos, key=lambda d: (d[3] - d[1]))
                                misi.bearing_deg = ((t[6] - (w / 2 + kalib.cx_offset_px))
                                                    / kalib.px_per_deg(w))
                                misi.jejak_kelas = names.get(t[5], str(t[5]))
                                misi.jejak_conf = t[4]
                            else:
                                misi.bearing_deg = None
                                misi.jejak_kelas = ""
                                misi.jejak_conf = 0.0
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
                            if lolos:
                                t = max(lolos, key=lambda d: (d[3] - d[1]))
                                misi.bearing_deg = ((t[6] - (w / 2 + kalib.cx_offset_px))
                                                    / kalib.px_per_deg(w))
                                misi.jejak_kelas = names.get(t[5], str(t[5]))
                                misi.jejak_conf = t[4]
                            else:
                                misi.bearing_deg = None

                # --- serah-terima dari firmware v1.9 ---
                # Firmware tidak tahu apa-apa soal kamera. Yang dia lakukan
                # cuma berhenti dan mencetak kalimatnya; HUD yang menyambung.
                fw = link.state_teensy().upper()
                if ("KONFIRMASI" in fw and misi.state not in (S_KONFIRM, S_GAGAL)
                        and not misi.halt and not misi.jeda):
                    juri.reset()
                    misi.ganti(S_KONFIRM, "firmware minta keputusan korban/bukan")
                    link.log.append("[KONFIRM] firmware berhenti menunggu m2/m3 "
                                    "-- vision dinyalakan")
                elif (any(k in link.ruas_fw().upper() for k in RUAS_VISION)
                      and misi.state not in (S_AWAS, S_KONFIRM, S_GAGAL)
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

                # --- FSM & antrean perintah ---
                langkah_fsm(misi, link, aksi, kalib, armed, pid, w, diam)
                aksi.putar(armed, boleh=not (misi.halt or misi.jeda))

                # --- perintah dari halaman web ---
                # Saat HALT, perintah gerak ditolak. Kalau tidak, "stop semua
                # aktivitas" cuma berlaku untuk FSM sementara tombol manual
                # tetap bisa menjalankan robot -- itu bukan stop, itu jeda
                # setengah hati. Yang TETAP boleh: melepas stop, melemaskan,
                # dan segala yang tidak menggerakkan apa pun.
                BOLEH_SAAT_HALT = {
                    "stop", "lepas_stop", "lemas", "abort", "pause", "mode",
                    "reset", "kalib", "simpan_kalib", "restart", "slot", "gagal",
                }
                for k, v in bersama.ambil_perintah():
                    if misi.halt and k not in BOLEH_SAAT_HALT:
                        link.log.append(f"[HALT] '{k}' ditolak -- tekan "
                                        f"'Lepas STOP' dulu")
                        continue
                    if k == "mode":
                        armed = not armed
                        aksi.batal()
                        link.log.append(f"[MODE] {'KENDALI' if armed else 'BACA'}")
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
                        # Satu tembakan, bukan lingkar tertutup: operator yang
                        # memutuskan kapan mengulang. Lebih mudah dihentikan.
                        d = link.depan_cm()
                        if d is None:
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
                            misi.ganti(S_A_TENGAH)
                            link.log.append(
                                f"[AMBIL] mulai -- target capit {kalib.capit_cm:.0f} cm, "
                                f"lengan r={kalib.lengan_r:.0f} h={kalib.lengan_h:.0f} mm. "
                                f"ANGKAT TANGAN DI TOMBOL LEMAS.")
                    elif k == "ambil_batal":
                        aksi.batal()
                        link.kirim("s", armed)
                        misi.ganti(S_IDLE, "ambil dibatalkan operator")
                        link.log.append("[AMBIL] dibatalkan, robot dihentikan")
                    elif k == "capit_buka":
                        link.kirim("g100", armed)
                    elif k == "capit_tutup":
                        link.kirim("g0", armed)
                    elif k == "lengan_contoh":
                        link.kirim("a70 20", armed)
                    elif k == "lengan_off":
                        link.kirim("n", armed)
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
                    elif k == "simpan_kalib":
                        # SATU-SATUNYA penulisan berkas di seluruh program.
                        nama = f"kalib_{time.strftime('%Y%m%d_%H%M%S')}.json"
                        with open(nama, "w") as fh:
                            json.dump({f: getattr(kalib, f) for f in KALIB_FIELDS},
                                      fh, indent=2)
                        pesan_kalib = f"disimpan ke {os.path.abspath(nama)} " \
                                      f"(muat lagi dengan --calib)"
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
