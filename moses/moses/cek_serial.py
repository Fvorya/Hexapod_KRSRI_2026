#!/usr/bin/env python3
"""
cek_serial.py -- kenapa HUD tidak menemukan Teensy? Jalankan DI RASPI.

    cd ~/M && .venv/bin/python cek_serial.py
    cd ~/M && .venv/bin/python cek_serial.py /dev/ttyACM0     # paksa satu port

Dia memakai fungsi yang SAMA dengan mission_hud.py (cari_port), jadi yang
diuji benar-benar jalur yang dipakai HUD -- bukan tiruan yang kebetulan jalan.

Enam pemeriksaan, berurutan dari yang paling sering jadi penyebab:
  1. interpreter mana yang dipakai  (venv atau python sistem?)
  2. pyserial ada dan bisa diimpor?
  3. node port apa saja yang terlihat kernel
  4. izin berkasnya, dan grup user ini
  5. ada proses lain yang memegang portnya?
  6. buka sungguhan, kirim 'm', tunggu jawaban firmware
"""
import glob
import os
import pwd
import subprocess
import sys
import time

GARIS = "-" * 68


def judul(n, teks):
    print(f"\n[{n}] {teks}\n{GARIS}")


def jalankan(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except Exception as e:                               # noqa: BLE001
        return f"(gagal menjalankan: {e})"


judul(1, "Interpreter")
print("python   :", sys.executable)
print("versi    :", sys.version.split()[0])
print("cwd      :", os.getcwd())
if ".venv" not in sys.executable:
    print("\n  !! Kamu TIDAK memakai .venv. Jalankan:")
    print("     cd ~/M && .venv/bin/python cek_serial.py")

judul(2, "pyserial")
try:
    import serial
    from serial.tools import list_ports
    print("pyserial :", serial.__version__, "->", serial.__file__)
    ports = list(list_ports.comports())
    print(f"comports : {len(ports)} perangkat")
    for p in ports:
        print(f"   {p.device:24} {p.description}  [{p.hwid}]")
except ImportError as e:
    serial = None
    list_ports = None
    print("!! pyserial TIDAK bisa diimpor:", e)
    print("   Perbaiki: .venv/bin/pip install pyserial")

judul(3, "Node port yang terlihat kernel")
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from mission_hud import POLA_PORT, cari_port
    pakai_hud = True
except Exception as e:                                   # noqa: BLE001
    print("(tidak bisa mengimpor mission_hud:", e, "-- pakai pola bawaan)")
    POLA_PORT = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
    cari_port = None
    pakai_hud = False

for pola in POLA_PORT:
    hit = sorted(glob.glob(pola))
    print(f"{pola:34} -> {hit if hit else 'kosong'}")

if pakai_hud:
    pilih, catatan = cari_port(rinci=True)
    print(f"\ncari_port() milik HUD memilih : {pilih}")
    print(f"catatannya                     : {catatan}")

judul(4, "Izin & grup")
user = pwd.getpwuid(os.getuid()).pw_name
print("user     :", user, f"(uid {os.getuid()})")
print("grup     :", jalankan("id -nG"))
if "dialout" not in jalankan("id -nG").split():
    print("\n  !! user ini TIDAK di grup 'dialout'.")
    print("     sudo usermod -aG dialout", user, "  lalu REBOOT")
print()
print(jalankan("ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null") or "(tidak ada node)")
byid = jalankan("ls -l /dev/serial/by-id/ 2>/dev/null")
print(byid if byid else "(/dev/serial/by-id/ tidak ada -- wajar bila tak ada perangkat serial USB)")

judul(5, "Ada yang memegang / mencuri portnya?")
# PENTING: fuser tanpa sudo TIDAK melihat proses milik root. Dua pencuri
# klasik pada /dev/ttyACM* justru berjalan sebagai root:
#   ModemManager -- menyangka Teensy sebagai modem, mengirim AT dan MEMAKAN
#                   balasannya, jadi HUD mengirim tapi tidak pernah menerima.
#   brltty       -- menyangka perangkat serial USB sebagai layar braille dan
#                   merebutnya sampai tidak bisa dipakai sama sekali.
sibuk = jalankan("sudo -n fuser -v /dev/ttyACM* 2>&1")
if not sibuk or "password" in sibuk.lower():
    sibuk = jalankan("fuser -v /dev/ttyACM* 2>&1") + "   <- tanpa sudo, proses root TIDAK terlihat"
print(sibuk or "(tidak ada)")

print("\nPencuri klasik:")
for svc in ("ModemManager", "brltty", "brltty-udev"):
    st = jalankan(f"systemctl is-active {svc}") or "tidak ada"
    tanda = "  <<< MATIKAN INI" if st == "active" else ""
    print(f"  {svc:14} : {st}{tanda}")
proses = jalankan("ps -eo pid,user,comm | grep -iE 'modemmanager|brltty' | grep -v grep")
print("  proses         :", proses if proses else "tidak ada")
print("\n  Kalau salah satu 'active':")
print("     sudo systemctl disable --now ModemManager")
print("     sudo systemctl disable --now brltty brltty-udev")
print("     sudo udevadm control --reload ; cabut-colok Teensy")

print("\nlayanan HUD:", jalankan("systemctl is-active r2c-hud") or "(tidak ada)")
print("  Kalau 'active', dia memegang portnya. Hentikan dulu sebelum uji buka:")
print("     sudo systemctl stop r2c-hud")

judul(6, "Buka sungguhan & minta status ke firmware")
target = sys.argv[1] if len(sys.argv) > 1 else (cari_port() if cari_port else None)
if not target:
    target = (sorted(glob.glob("/dev/ttyACM*")) or [None])[0]

if serial is None:
    print("dilewati -- pyserial tidak ada.")
elif not target:
    print("dilewati -- tidak ada port untuk dicoba.")
else:
    print("mencoba :", target)
    try:
        with serial.Serial(target, 115200, timeout=0.2) as ser:
            print("TERBUKA. Mengirim 'm' (status misi, perintah yang hanya MEMBACA)...")
            ser.reset_input_buffer()
            ser.write(b"m\n")
            time.sleep(1.5)
            data = ser.read(8192).decode("utf-8", "replace")
            if data.strip():
                print("\n--- jawaban firmware ---")
                print(data.strip()[:1500])
                print("--- selesai ---")
                print("\nKESIMPULAN: port SEHAT dan firmware MENJAWAB.")
                print("Kalau HUD tetap 'SIMULASI', berarti HUD tidak sempat")
                print("membukanya -- pastikan layanannya sudah di-restart dengan")
                print("mission_hud.py versi terbaru.")
            else:
                print("\nTerbuka tapi TIDAK ADA jawaban.")
                print("Kemungkinan, urut dari yang paling sering:")
                print("  - firmware lama tanpa perintah 'm' (flash v1.61)")
                print("  - Teensy tidak dapat daya VIN (sesudah pad VUSB-VIN")
                print("    dipotong, colok USB saja TIDAK menyalakan papannya)")
                print("  - baud beda dari 115200")
    except PermissionError as e:
        print("IZIN DITOLAK:", e)
        print("  sudo usermod -aG dialout", user, "  lalu REBOOT")
    except Exception as e:                               # noqa: BLE001
        print("GAGAL:", type(e).__name__, e)
        print("  Kalau 'Device or resource busy', ada proses lain memegangnya")
        print("  (lihat bagian 5) -- sudo systemctl stop r2c-hud")

print("\n" + GARIS)
print("Kirim SELURUH keluaran ini kalau masih buntu.")
