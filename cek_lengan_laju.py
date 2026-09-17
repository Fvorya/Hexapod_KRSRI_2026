#!/usr/bin/env python3
"""Check the arm's acceleration-limited slew: does it converge, and does it
still fit inside the fixed victim-sequence delay?

This mirrors the integer arithmetic of HexaArm::commit() rather than calling
it. Two things can go wrong there and neither needs a robot to find:

  1. The brake drives the step toward zero near the target, but tahapSampai()
     demands _kini == _target exactly. A joint that stops one microsecond
     short never hands over to the next stage -- the arm freezes mid-pose.
  2. Acceleration limiting adds v/a per stage. LENGAN_JEDA_MS is a FIXED
     delay, so a move that grows past it gets cut off mid-travel.

Run after touching ARM_ACCEL_DEG_S2, ARM_SLEW_DEG_S, LENGAN_JEDA_MS or the
arm pulse range:

    python cek_lengan_laju.py
"""
import math
import re
import sys
from pathlib import Path

CONFIG = Path(__file__).parent / "Hexapod_Unlimited" / "config.h"


def ambil(nama, teks):
    # Tanda minus WAJIB ikut: KORBAN_SIAP_BAHU itu -50.0f, dan pola tanpa
    # minus diam-diam gagal mencocokkan lalu keluar dengan "tidak ketemu".
    # Akhir baris juga tidak dipatok -- banyak define punya komentar ekor.
    m = re.search(r"^#define\s+%s\s+(-?[0-9.]+)f?\b" % nama, teks, re.M)
    if not m:
        sys.exit("tidak ketemu di config.h: %s" % nama)
    return float(m.group(1))


teks = CONFIG.read_text(encoding="utf-8", errors="replace")
SLEW = ambil("ARM_SLEW_DEG_S", teks)
ACCEL = ambil("ARM_ACCEL_DEG_S2", teks)
JEDA = ambil("LENGAN_JEDA_MS", teks)

# arm.pulse.min/max live in Calib.cpp PARAM_DEFS, not config.h. Both defaults
# are 500/2500 and the firmware maps 0..180 deg across the whole range.
US_PER_DER = (2500.0 - 500.0) / 180.0
DT_MS = 20  # commit() gates on dt < 20, and SERVO_PWM_FREQ 50 matches it


def jalankan(derajat, batas_putaran=2000):
    """One joint, integer arithmetic copied from commit(). Returns
    (ms, laju_puncak_der_s) or raises if it never lands."""
    target = int(round(derajat * US_PER_DER))
    kini = 0
    laju = 0.0
    puncak = 0.0
    dts = DT_MS / 1000.0
    for putaran in range(batas_putaran):
        if kini == target:
            return putaran * DT_MS, puncak
        d = target - kini
        sisa_der = abs(d) / US_PER_DER
        laju = min(laju + ACCEL * dts, SLEW)
        laju = min(laju, math.sqrt(2.0 * ACCEL * sisa_der))
        puncak = max(puncak, laju)
        langkah = max(int(laju * US_PER_DER * dts), 1)
        d = max(-langkah, min(langkah, d))
        kini += d
    raise AssertionError("%g der tidak pernah sampai sasaran" % derajat)


gagal = 0
print("ARM_SLEW_DEG_S %g, ARM_ACCEL_DEG_S2 %g, LENGAN_JEDA_MS %g\n"
      % (SLEW, ACCEL, JEDA))
print("  derajat     waktu   laju puncak")
for derajat in (1, 5, 10, 30, 37.5, 50, 90, 135, 180):
    ms, puncak = jalankan(derajat)
    print("  %7.1f  %6d ms  %7.1f der/detik" % (derajat, ms, puncak))
    # Gerakan pendek WAJIB melandai sendiri: itu seluruh gunanya batas ini.
    if derajat <= 30 and puncak >= SLEW:
        print("    ! gerakan pendek masih mencapai laju penuh")
        gagal += 1

# --- PERPINDAHAN POSE NYATA DI sekuensAmbil() ---------------------------
#
# faseBaru() membagi waktu dengan LENGAN_JEDA_MS dan TIDAK menanyakan apakah
# lengan sudah sampai. Fase yang kehabisan jatah ditimpa perintah fase
# berikutnya di tengah gerak, dan posenya tidak pernah tercapai. Jadi tiap
# perpindahan harus muat di jatahnya, dengan kelegaan.
SLEW_REHAT  = ambil("LENGAN_SLEW_REHAT_DEG_S", teks)
SLEW_ANGKAT = ambil("LENGAN_SLEW_ANGKAT_DEG_S", teks)

POSE = {}
POSE["siap"]  = (ambil("KORBAN_SIAP_BAHU", teks),  ambil("KORBAN_SIAP_SIKU", teks),
                 ambil("KORBAN_SIAP_PRG", teks))
POSE["jepit"] = (ambil("KORBAN_JEPIT_BAHU", teks), ambil("KORBAN_JEPIT_SIKU", teks),
                 ambil("KORBAN_JEPIT_PRG", teks))
POSE["rehat"] = (ambil("REHAT_BAHU", teks),        ambil("REHAT_SIKU", teks),
                 ambil("REHAT_PERGELANGAN", teks))
POSE["angkat"] = (ambil("KORBAN_ANGKAT_BAHU", teks), ambil("KORBAN_ANGKAT_SIKU", teks),
                  ambil("KORBAN_ANGKAT_PRG", teks))
POSE["lepas"]  = (ambil("KORBAN_LEPAS_BAHU", teks),  ambil("KORBAN_LEPAS_SIKU", teks),
                  ambil("KORBAN_LEPAS_PRG", teks))


def pindah(dari, ke, laju, jatah):
    """Dua tahap: bahu sendiri, lalu siku+pergelangan bersama (TAHAP_MASK).
    Tiap tahap selesai saat sendi TERJAUHnya selesai."""
    global SLEW
    a, b = POSE[dari], POSE[ke]
    simpan, SLEW = SLEW, laju
    try:
        t1 = jalankan(abs(b[0] - a[0]))[0] if abs(b[0] - a[0]) > 0 else 0
        d2 = max(abs(b[1] - a[1]), abs(b[2] - a[2]))
        t2 = jalankan(d2)[0] if d2 > 0 else 0
    finally:
        SLEW = simpan
    return t1 + t2, jatah * JEDA


print("\nperpindahan pose sekuensAmbil (jatah = N x LENGAN_JEDA_MS)")
for dari, ke, laju, jatah, label in (
        ("siap",  "jepit", SLEW,       1, "fase 1  turun ke jepit"),
        ("jepit", "angkat", SLEW_ANGKAT, 1, "fase 4  ANGKAT dari jepit"),
        ("angkat", "rehat", SLEW_REHAT, 2, "fase 5  lipat ke rehat")):
    ms, budget = pindah(dari, ke, laju, jatah)
    sisa = budget - ms
    print("  %-26s %5d ms / %5d ms   sisa %5d ms (%3.0f%%)"
          % (label, ms, budget, sisa, 100.0 * sisa / budget))
    if sisa <= 0:
        print("    ! LEWAT JATAH -- pose ini tidak akan pernah tercapai")
        gagal += 1
    elif sisa < 0.10 * budget:
        print("    ! sisa di bawah 10% -- tambah satu fase kosong, atau")
        print("      naikkan LENGAN_SLEW_REHAT_DEG_S")
        gagal += 1

print("\nperpindahan pose sekuensTaruh")
for dari, ke, laju, jatah, label in (
        ("rehat",  "siap",   SLEW_REHAT,  2, "fase 0  rehat ke siap"),
        ("siap",   "lepas",  SLEW_ANGKAT, 1, "fase 2  turun ke lepas"),
        ("lepas",  "angkat", SLEW,        1, "fase 4  naik, capit kosong"),
        ("angkat", "rehat",  SLEW,        1, "fase 5  lipat ke rehat")):
    ms, budget = pindah(dari, ke, laju, jatah)
    sisa = budget - ms
    print("  %-26s %5d ms / %5d ms   sisa %5d ms (%3.0f%%)"
          % (label, ms, budget, sisa, 100.0 * sisa / budget))
    if sisa <= 0:
        print("    ! LEWAT JATAH -- pose ini tidak akan pernah tercapai")
        gagal += 1
    elif sisa < 0.10 * budget:
        print("    ! sisa di bawah 10% -- tambah satu fase kosong")
        gagal += 1

print("\n%s" % ("ADA MASALAH di atas" if gagal else "Semua pemeriksaan lolos."))
sys.exit(1 if gagal else 0)
