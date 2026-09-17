#!/usr/bin/env python3
"""flash_teensy.py -- compile & flash Teensy 4.1 LANGSUNG DARI RASPBERRY PI.

    python3 flash_teensy.py                 # menu, pilih versi
    python3 flash_teensy.py --versi v1.12   # langsung ke versi itu
    python3 flash_teensy.py --terbaru       # versi tertinggi, tanpa tanya
    python3 flash_teensy.py --coba          # siapkan & periksa, TIDAK flash
    python3 flash_teensy.py --hanya-compile # compile saja, tidak flash

KENAPA INI ADA
--------------
Sebelumnya: laptop -> IDE Arduino -> kabel USB ke Teensy. Artinya laptop harus
ada di sebelah robot. Sekarang: laptop -> scp ke Pi -> Pi yang flash. Pi sudah
menempel di robot dan sudah tersambung ke Teensy lewat USB, jadi kabelnya
memang sudah ada di sana.

YANG PALING MENENTUKAN DI SINI: ADAPTASI KE FILE VINCENT
--------------------------------------------------------
Tiap versi berbeda isinya, dan nama foldernya TIDAK konsisten. Ini daftar
yang benar-benar ada di disk per 11 Sep 2026:

    Hexapod_KRSRI_2026-ver1_4       <- "ver", garis bawah
    Hexapod_KRSRI_2026-ver1_6
    Hexapod_KRSRI_2026-ver1_7
    Hexapod_KRSRI_2026-Ver1_8       <- "Ver" huruf besar
    Hexapod_KRSRI_2026-v1.61        <- "v", titik
    Hexapod_Unlimited_v1.9          <- nama proyek ganti
    Hexapod_Unlimited_v1.10
    Hexapod_Unlimited_v1.11
    Hexapod_Unlimited_v1.12.zip     <- ZIP, belum diekstrak

Enam ejaan berbeda untuk hal yang sama. Jadi skrip ini TIDAK BOLEH menebak
pola nama. Yang dilakukan: cari semua yang mengandung .ino (folder maupun
zip), lalu urutkan menurut WAKTU BERKAS.

Kenapa waktu, bukan nomor versi? Karena nomornya sendiri berbohong. 'v1.61'
terurai jadi (1, 61), dan (1, 61) > (1, 12) -- padahal v1.61 itu versi KRSRI
lama, jauh sebelum Unlimited v1.12. Agaknya "1.6 revisi 1", tapi tidak ada
parser yang bisa tahu itu dari namanya.

Ini bukan kekhawatiran teoretis: 'dir /s /o-d' di upload_teensy.bat dulu
menyiapkan v1.61 padahal yang diminta v1.8. Nomor versi tetap DITAMPILKAN,
dan kalau urutan nomor dan urutan waktu tidak sepakat, itu dikatakan --
karena yang tahu mana yang benar itu kamu, bukan skrip ini.

YANG TIDAK DILAKUKAN, DAN KENAPA
--------------------------------
Tidak ada tombol "flash" di mission_hud. Godaannya besar, tapi HUD memegang
/dev/ttyACM0 terus-menerus dan flashing butuh port itu -- jadi HUD harus mati
dulu, dan sebuah tombol yang mematikan servernya sendiri lalu mengharap bisa
melapor hasilnya itu rancangan yang salah sejak awal. Skrip ini yang
mematikan dan menyalakan HUD, dari luar.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile
from pathlib import Path

FQBN = "teensy:avr:teensy41"
LAYANAN = "r2c-hud"

# Header yang IKUT core Teensy/Arduino atau libc -- bukan pustaka yang harus
# dipasang. Sisa dari daftar #include dianggap pihak ketiga dan diperiksa.
BAWAAN = {
    "Arduino.h", "EEPROM.h", "Wire.h", "SPI.h", "SD.h", "Servo.h",
    "SoftwareSerial.h", "HardwareSerial.h", "usb_serial.h", "elapsedMillis.h",
    "math.h", "stdio.h", "stdlib.h", "string.h", "stdint.h", "stddef.h",
    "stdarg.h", "stdbool.h", "float.h", "limits.h", "ctype.h", "errno.h",
    "inttypes.h", "time.h", "assert.h",
}

# Pustaka pihak ketiga yang kita tahu dipakai, dengan nama persis di indeks
# Arduino. Dipakai untuk menyarankan perintah pasangnya, bukan untuk
# membatasi: header lain yang muncul tetap dilaporkan.
PUSTAKA = {
    "VL53L1X.h": "VL53L1X",
    "Adafruit_PWMServoDriver.h": "Adafruit PWM Servo Driver Library",
    "Adafruit_Sensor.h": "Adafruit Unified Sensor",
    # OLED skor, dipakai Tampilan.cpp sejak 16 Sep 2026. SSD1306 menarik
    # Adafruit BusIO sendiri; GFX tidak, jadi keduanya disebut.
    "Adafruit_GFX.h": "Adafruit GFX Library",
    "Adafruit_SSD1306.h": "Adafruit SSD1306",
}


def lapor(*a):
    print(*a, flush=True)


def jalan(cmd, **kw):
    """Jalankan perintah, kembalikan (rc, keluaran). Tidak pernah melempar."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, **kw)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except FileNotFoundError:
        return 127, f"perintah tidak ada: {cmd[0]}"
    except Exception as e:                                  # noqa: BLE001
        return 1, str(e)


# =====================================================================
# 1. MENEMUKAN VERSI -- bagian yang harus tahan ejaan Vincent
# =====================================================================

def urai_versi(nama: str):
    """Ambil versi dari nama folder/zip sebagai TUPEL ANGKA.

    'Hexapod_KRSRI_2026-ver1_4'  -> (1, 4)
    'Hexapod_KRSRI_2026-Ver1_8'  -> (1, 8)
    'Hexapod_Unlimited_v1.9'     -> (1, 9)
    'Hexapod_Unlimited_v1.12'    -> (1, 12)

    Pemisahnya titik ATAU garis bawah, penandanya 'v' atau 'ver' dengan huruf
    besar-kecil apa pun. Yang tidak punya versi sama sekali kembali ().

    Tahun DIBUANG lebih dulu. Tanpa itu '2026' di 'Hexapod_KRSRI_2026-ver1_4'
    tertangkap sebagai versi 2026 dan folder tertua justru menang.

    ANGKA INI TIDAK DIPAKAI UNTUK MENENTUKAN MANA YANG TERBARU.
    Sebabnya ada di disk: 'v1.61' terurai jadi (1, 61), dan (1, 61) > (1, 12)
    -- padahal v1.61 itu versi KRSRI yang lama, jauh sebelum Unlimited v1.12.
    "1.61" di sana agaknya berarti 1.6 revisi 1, bukan satu-koma-enam-puluh-
    satu. Tidak ada parser yang bisa tahu itu dari namanya saja, dan menebak
    di sini persis yang dulu membuat upload_teensy.bat menyiapkan v1.61
    padahal yang diminta v1.8.

    Jadi versi dipakai untuk DITAMPILKAN, dan waktu berkas yang dipakai untuk
    MENGURUTKAN. Kalau keduanya tidak sepakat, itu dikatakan -- lihat
    cari_sketsa().
    """
    bersih = re.sub(r"20\d\d", "", nama)
    m = re.search(r"[vV](?:er)?\.?_?(\d+)(?:[._](\d+))?(?:[._](\d+))?", bersih)
    if not m:
        return ()
    return tuple(int(g) for g in m.groups() if g is not None)


def waktu_isi(jalur: Path) -> float:
    """Waktu ubah TERBARU dari isinya, bukan dari foldernya.

    mtime folder berubah setiap ada yang menyalin atau menyentuhnya, jadi ia
    bukan bukti kapan Vincent membuat versi itu. Yang lebih dekat ke
    kenyataan: berkas sumber termuda di dalamnya.
    """
    if jalur.is_file():
        return jalur.stat().st_mtime
    waktu = [jalur.stat().st_mtime]
    for f in jalur.rglob("*"):
        if f.is_file() and f.suffix.lower() in (".ino", ".h", ".hpp", ".c",
                                                ".cpp", ".md"):
            try:
                waktu.append(f.stat().st_mtime)
            except OSError:
                pass
    return max(waktu)


def versi_teks(v):
    return "v" + ".".join(str(x) for x in v) if v else "(tanpa nomor)"


def cari_sketsa(akar: Path):
    """Semua kandidat sketsa di bawah `akar`: folder ber-.ino, dan zip.

    Zip ikut karena v1.12 memang datang sebagai zip dan tidak pernah
    diekstrak. Melewatkannya berarti versi terbaru tidak kelihatan sama
    sekali -- persis keadaan yang membuat orang mengira belum ada update.

    Kembalikan daftar dict, terurut WAKTU MENAIK (terbaru di bawah, jadi
    nomor menu terbesar = yang paling baru diberikan Vincent).
    """
    hasil = []
    if not akar.is_dir():
        return hasil

    for anak in sorted(akar.iterdir()):
        if anak.is_dir():
            inos = sorted(anak.rglob("*.ino"))
            if inos:
                hasil.append({"nama": anak.name, "jalur": anak, "zip": False,
                              "ino": inos[0], "versi": urai_versi(anak.name)})
        elif anak.suffix.lower() == ".zip":
            try:
                with zipfile.ZipFile(anak) as z:
                    inos = [n for n in z.namelist() if n.lower().endswith(".ino")]
            except (zipfile.BadZipFile, OSError):
                continue
            if inos:
                hasil.append({"nama": anak.name, "jalur": anak, "zip": True,
                              "ino": inos[0], "versi": urai_versi(anak.stem)})

    for d in hasil:
        d["waktu"] = waktu_isi(d["jalur"])

    # DIURUTKAN MENURUT WAKTU, bukan menurut nomor versi. Lihat urai_versi()
    # untuk sebabnya: v1.61 itu versi lama tapi nomornya paling besar.
    #
    # Nomor versi jadi kunci KEDUA, untuk saat waktunya seri. Itu terjadi
    # kalau stempel waktunya hilang -- scp tanpa -p menulis ulang semuanya
    # dengan waktu sekarang, dan tiba-tiba seluruh versi seumur. Di keadaan
    # itu nomor versi jadi satu-satunya petunjuk yang tersisa, jadi ia
    # dipakai; bukan urutan abjad, yang tidak berarti apa-apa.
    hasil.sort(key=lambda d: (d["waktu"], d["versi"] or (0,)))

    # Dan kalau keduanya tidak sepakat, itu DIKATAKAN -- bukan dipendam.
    # Yang dipilih tetap urutan waktu, tapi operator berhak tahu bahwa
    # nomornya bercerita lain, karena dialah yang tahu mana yang benar.
    urut_versi = sorted(hasil, key=lambda d: (d["versi"] or (0,), d["waktu"]))
    if [d["nama"] for d in urut_versi] != [d["nama"] for d in hasil]:
        for d in hasil:
            d["bentrok"] = True

    return hasil


def siapkan_folder(pilih: dict, kerja: Path) -> Path:
    """Kembalikan folder sketsa yang SIAP dikompilasi arduino-cli.

    Dua hal yang harus diluruskan, dan keduanya nyata di file Vincent:

    1. Zip diekstrak dulu.
    2. Arduino MEWAJIBKAN nama folder sama dengan nama .ino. Kalau tidak
       sama, arduino-cli menolak dengan pesan yang tidak menyebut sebabnya.
       Jadi kalau berbeda, seluruh isinya disalin ke folder bernama benar --
       SALINAN, supaya file Vincent tidak pernah disentuh.
    """
    if pilih["zip"]:
        tujuan = kerja / "ekstrak"
        tujuan.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(pilih["jalur"]) as z:
            z.extractall(tujuan)
        inos = sorted(tujuan.rglob("*.ino"))
        if not inos:
            raise RuntimeError("zip tidak berisi .ino")
        ino = inos[0]
    else:
        ino = pilih["ino"]

    folder = ino.parent
    if folder.name == ino.stem:
        return folder

    lapor(f"  ! nama folder '{folder.name}' != nama sketsa '{ino.stem}'.")
    lapor(f"    Arduino mewajibkan keduanya sama, jadi dibuatkan SALINAN.")
    rapi = kerja / ino.stem
    if rapi.exists():
        shutil.rmtree(rapi)
    shutil.copytree(folder, rapi)
    return rapi


# =====================================================================
# 2. MEMERIKSA ISINYA -- pustaka apa yang dibutuhkan versi INI
# =====================================================================

def pustaka_dibutuhkan(folder: Path):
    """Header pihak ketiga yang di-#include versi ini.

    Dipindai ulang TIAP versi, bukan dari daftar tetap. Vincent menambah
    ArmInverse dan HexaArm di v1.12; kalau suatu saat ia menambah pustaka
    baru, yang gagal seharusnya pesan 'pustaka X belum ada' -- bukan ratusan
    baris error kompilasi yang harus dibaca satu-satu.
    """
    pola = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.M)
    lokal = {p.name for p in folder.rglob("*") if p.is_file()}
    butuh = set()
    for f in folder.rglob("*"):
        if f.suffix.lower() not in (".ino", ".h", ".hpp", ".c", ".cpp"):
            continue
        try:
            teks = f.read_text(errors="ignore")
        except OSError:
            continue
        for h in pola.findall(teks):
            nama = h.split("/")[-1]
            if nama in BAWAAN or nama in lokal:
                continue
            butuh.add(nama)
    return sorted(butuh)


def siapkan_path():
    """Masukkan tempat-tempat biner lokal ke PATH proses ini.

    KENAPA INI PERLU, dan kenapa bukan kerapian.
    siapkan_teensy_pi.sh memasang arduino-cli dan teensy_loader_cli ke
    ~/bin, lalu menambahkan baris export PATH ke ~/.bashrc. Itu cukup untuk
    ssh yang INTERAKTIF -- tapi

        ssh pi "python3 flash_teensy.py"

    itu shell NON-interaktif, dan .bashrc bawaan Debian/Raspberry Pi OS
    keluar di baris-baris pertamanya persis untuk shell non-interaktif:

        case $- in *i*) ;; *) return;; esac

    Jadi baris export-nya TIDAK PERNAH dijalankan, ~/bin tidak ada di PATH,
    dan skrip ini melaporkan "arduino-cli tidak ada" padahal berkasnya jelas
    ada di ~/bin. Gejalanya: pemasangan yang sukses, tapi tetap gagal jalan
    lewat ssh satu baris -- dan tidak ada satu pun pesan yang menunjuk ke
    sebabnya.

    Jadi jangan bergantung pada shell. Cari sendiri.
    """
    tambah = [Path.home() / "bin", Path.home() / ".local" / "bin",
              Path("/usr/local/bin"), Path("/opt/arduino-cli")]
    jalur = os.environ.get("PATH", "").split(os.pathsep)
    for d in tambah:
        if d.is_dir() and str(d) not in jalur:
            jalur.insert(0, str(d))
    os.environ["PATH"] = os.pathsep.join(jalur)


def periksa_alat(butuh_flash=True):
    """Pastikan alatnya ada SEBELUM apa pun dikerjakan.

    Kembalikan daftar keluhan; kosong berarti siap.

    Diperiksa di DEPAN, bukan saat dipakai. Versi sebelumnya baru tahu
    arduino-cli tidak ada sesudah mengekstrak zip, mencocokkan nama folder,
    dan memindai seluruh #include -- lalu berhenti dengan "COMPILE GAGAL",
    kalimat yang terdengar seperti kodenya yang salah. Padahal yang kurang
    cuma satu pemasangan, dan itu sudah bisa diketahui di detik pertama.
    """
    kurang = []
    if not shutil.which("arduino-cli"):
        kurang.append("arduino-cli")
    else:
        rc, out = jalan(["arduino-cli", "core", "list"])
        if rc != 0 or "teensy:avr" not in out.lower():
            kurang.append("core teensy:avr")
    if butuh_flash and not shutil.which("teensy_loader_cli"):
        kurang.append("teensy_loader_cli")
    return kurang


def pustaka_terpasang():
    rc, out = jalan(["arduino-cli", "lib", "list"])
    if rc != 0:
        return None
    return out.lower()


# =====================================================================
# 3. COMPILE & FLASH
# =====================================================================

def hentikan_hud():
    """HUD memegang /dev/ttyACM0. Harus lepas sebelum Teensy di-flash."""
    rc, _ = jalan(["systemctl", "is-active", "--quiet", LAYANAN])
    hidup = (rc == 0)
    if hidup:
        lapor(f"  menghentikan {LAYANAN} (ia memegang port serial)...")
        jalan(["sudo", "systemctl", "stop", LAYANAN])
    return hidup


def nyalakan_hud():
    lapor(f"  menyalakan {LAYANAN} lagi...")
    jalan(["sudo", "systemctl", "start", LAYANAN])
    rc, out = jalan(["systemctl", "is-active", LAYANAN])
    lapor(f"  {LAYANAN}: {out.strip() or 'tidak diketahui'}")


def cari_port():
    for pola in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ada = sorted(Path("/dev").glob(pola.split("/")[-1]))
        if ada:
            return str(ada[0])
    return None


def minta_bootloader(port):
    """Suruh Teensy masuk bootloader tanpa menyentuh tombolnya.

    Caranya: buka port USB serial-nya pada 134 baud. Core Teensy menangkap
    baud ganjil itu sebagai perintah reboot -- ini trik yang sama yang
    dipakai IDE Arduino sendiri.

    Dibuat BOLEH GAGAL. Kalau Teensy sedang tidak menjalankan sketsa yang
    membuka USB serial (misalnya baru saja gagal flash), port-nya tidak ada
    dan tidak ada yang bisa disuruh apa-apa. Di situ tombol fisik yang
    menjawab, dan itu keadaan normal -- bukan error yang perlu menghentikan
    apa pun.
    """
    if not port or not Path(port).exists():
        return False, "port serial tidak ada"
    try:
        import serial                                       # type: ignore
        s = serial.Serial(port, 134)
        s.close()
        return True, "reboot lunak dikirim (134 baud)"
    except Exception as e:                                  # noqa: BLE001
        return False, f"reboot lunak gagal: {e}"


def tunggu_port(lama=20.0, kecuali=None):
    """Tunggu node serial USB muncul. Kembalikan jalurnya, atau None.

    `kecuali` = daftar jalur yang sudah ada SEBELUM flash; yang dicari node
    yang BARU, supaya port milik perangkat lain tidak disalahartikan sebagai
    Teensy yang sudah kembali.
    """
    kecuali = set(kecuali or [])
    t0 = time.time()
    while time.time() - t0 < lama:
        ada = set(glob.glob("/dev/ttyACM*")) | set(glob.glob("/dev/ttyUSB*"))
        baru = sorted(ada - kecuali)
        if baru:
            return baru[0]
        if ada and not kecuali:
            return sorted(ada)[0]
        time.sleep(0.5)
    return None


def sapa_firmware(port, lama=6.0):
    """Buka port, tunggu sapaan firmware. Kembalikan (ok, teks).

    Bukan sekadar "node-nya ada". Node bisa muncul dari bootloader atau dari
    sketsa yang gagal di setup() dan tidak mencetak apa pun. Yang membuktikan
    sketsa BENAR-BENAR jalan cuma barisnya sendiri.

    Teensy mencetak "Memulai Hexapod Unlimited..." di setup(), tapi ia
    menunggu USB serial maksimal 3 detik sebelum itu -- dan kalau kita telat
    membuka port, sapaannya sudah lewat. Jadi 'm' dikirim sebagai cadangan:
    apa pun yang menjawab berarti firmware hidup dan mendengarkan.
    """
    try:
        import serial                                       # type: ignore
    except ImportError:
        return None, "pyserial tidak ada di python ini -- tidak bisa diperiksa"
    try:
        with serial.Serial(port, 115200, timeout=0.3) as sp:
            t0 = time.time()
            data = ""
            diminta = False
            while time.time() - t0 < lama:
                data += sp.read(4096).decode("utf-8", "replace")
                if "Memulai Hexapod" in data or "Hexapod" in data:
                    return True, "sapaan firmware terbaca"
                if not diminta and time.time() - t0 > 2.0:
                    sp.write(b"m\n")          # 'm' hanya MEMBACA status
                    diminta = True
                if diminta and data.strip():
                    return True, "firmware menjawab 'm'"
            return False, "port terbuka tapi firmware DIAM"
    except Exception as e:                                  # noqa: BLE001
        return False, f"tidak bisa dibuka: {e}"


def uji_sendiri():
    """Uji logika yang bisa diuji tanpa Teensy. `python3 flash_teensy.py --uji`.

    Yang diuji bukan pustaka pyserial, melainkan KEPUTUSAN skrip ini: kapan ia
    menyatakan firmware hidup, kapan diam, dan kapan ia jujur bilang tidak
    bisa memeriksa. Ketiganya pernah salah diartikan sebagai "flash sukses".
    """
    import types
    ok = gagal = 0

    def cek(nama, dapat, harap):
        nonlocal ok, gagal
        if dapat == harap:
            ok += 1
            lapor(f"  OK   {nama}")
        else:
            gagal += 1
            lapor(f"  GAGAL {nama}: dapat {dapat!r}, harap {harap!r}")

    def stub(skrip, meledak=None):
        mod = types.ModuleType("serial")
        if meledak:
            class S:
                def __init__(self, *a, **k):
                    raise OSError(meledak)
        else:
            class S:                                        # noqa: D401
                def __init__(self, port, baud, timeout=0):
                    self.t0 = time.time(); self.kirim = []
                def __enter__(self): return self
                def __exit__(self, *a): return False
                def read(self, n):
                    out = b""
                    for t, b in skrip:
                        if time.time() - self.t0 >= t and b not in self.kirim:
                            self.kirim.append(b); out += b
                    return out
                def write(self, b): self.kirim.append(b)
        mod.Serial = S
        sys.modules["serial"] = mod

    lapor("uji flash_teensy (tanpa Teensy)\n")
    lapor(" petunjuk cara pakai")
    _src = open(__file__ if "__file__" in dir() else "flash_teensy.py").read()
    cek("menyebut cara DI PI tanpa ssh", "SUDAH DI PI" in _src, True)
    cek("memperingatkan '-t' bukan perintah shell",
        "bukan perintah" in _src and "-t" in _src, True)
    cek("dan menyebut bentuk dari laptop", "DARI LAPTOP" in _src, True)
    cek("nomor versi kembar ditandai", "KEMBAR" in _src, True)

    lapor(" versi dari nama folder Vincent")
    for nama, harap in (("Hexapod_KRSRI_2026-ver1_4", (1, 4)),
                        ("Hexapod_KRSRI_2026-Ver1_8", (1, 8)),
                        ("Hexapod_Unlimited_v1.9", (1, 9)),
                        ("Hexapod_Unlimited_v1.12", (1, 12)),
                        ("tanpa-nomor", ())):
        cek(f"{nama} -> {harap}", urai_versi(nama), harap)
    cek("1.10 > 1.9 sebagai ANGKA, bukan teks",
        urai_versi("v1.10") > urai_versi("v1.9"), True)
    cek("tahun 2026 tidak tertangkap sebagai versi",
        urai_versi("Hexapod_KRSRI_2026-ver1_4"), (1, 4))

    lapor("\n sapaan firmware sesudah flash")
    stub([(0.2, b"\n\nMemulai Hexapod Unlimited...\n")])
    r = sapa_firmware("/dev/palsu", 3.0)
    cek("sapaan terbaca -> HIDUP", r[0], True)
    stub([])
    r = sapa_firmware("/dev/palsu", 1.5)
    cek("firmware diam -> BUKAN hidup", r[0], False)
    cek("dan sebabnya disebut", "DIAM" in r[1], True)
    stub([(2.3, b"  state       : DIAM\n")])
    cek("menjawab 'm' -> HIDUP", sapa_firmware("/dev/palsu", 5.0)[0], True)
    stub([], meledak="Permission denied")
    r = sapa_firmware("/dev/palsu", 1.0)
    cek("port ditolak -> BUKAN hidup", r[0], False)
    cek("izinnya disebut", "Permission" in r[1], True)
    sys.modules.pop("serial", None)
    cek("tanpa pyserial -> None, bukan False",
        sapa_firmware("/dev/palsu", 0.5)[0], None)

    lapor("\n menunggu port kembali")
    cek("tanpa node apa pun -> None", tunggu_port(0.6), None)

    lapor(f"\n=== {ok} lulus, {gagal} gagal ===")
    return 1 if gagal else 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--akar", default=os.path.expanduser("~/teensy"),
                   help="folder berisi versi-versi Vincent (bawaan ~/teensy)")
    p.add_argument("--versi", default=None,
                   help="pilih versi yang namanya MENGANDUNG teks ini, mis. v1.12")
    p.add_argument("--terbaru", action="store_true",
                   help="pakai yang berkasnya PALING BARU, tanpa bertanya")
    p.add_argument("--coba", action="store_true",
                   help="siapkan & periksa saja: TIDAK compile, TIDAK flash")
    p.add_argument("--hanya-compile", action="store_true",
                   help="compile saja, jangan flash")
    p.add_argument("--port", default=None, help="port serial Teensy")
    p.add_argument("--uji", action="store_true",
                   help="uji logika skrip ini, tanpa Teensy dan tanpa alat")
    args = p.parse_args()

    if args.uji:
        return uji_sendiri()

    akar = Path(args.akar).expanduser()
    siapkan_path()
    lapor(f"== flash_teensy == akar: {akar}")

    # ALAT DULU. Kalau belum lengkap, berhenti DI SINI -- sebelum mengekstrak
    # zip dan memindai apa pun. Gagal di depan dengan satu perintah perbaikan
    # jauh lebih berguna daripada gagal di tengah dengan kata "COMPILE GAGAL".
    kurang = periksa_alat(butuh_flash=not (args.coba or args.hanya_compile))
    if kurang:
        lapor("\n!! Alatnya belum lengkap: " + ", ".join(kurang))
        lapor("\n   Jalankan SEKALI di Pi ini:")
        lapor("     bash ~/M/siapkan_teensy_pi.sh")
        lapor("\n   Kalau berkasnya belum ada di ~/M, kirim dulu dari laptop")
        lapor("   dengan upload_to_pi.bat (siapkan_teensy_pi.sh sudah ikut).")
        if shutil.which("arduino-cli") is None:
            for d in (Path.home() / "bin", Path.home() / ".local" / "bin"):
                if (d / "arduino-cli").exists():
                    lapor(f"\n   CATATAN: arduino-cli ADA di {d} tapi tidak di PATH.")
                    lapor("   Itu biasa untuk 'ssh pi \"perintah\"': shell non-interaktif")
                    lapor("   tidak membaca ~/.bashrc. Skrip ini sudah mencoba")
                    lapor("   memasukkannya sendiri -- kalau masih gagal, periksa izin")
                    lapor(f"   berkasnya: ls -l {d}/arduino-cli")
        lapor("\n   Teensy TIDAK disentuh sama sekali.")
        return 3
    lapor("   alat: arduino-cli, core teensy:avr"
          + (", teensy_loader_cli" if shutil.which("teensy_loader_cli") else "")
          + " -- siap")

    daftar = cari_sketsa(akar)
    if not daftar:
        lapor(f"!! tidak ada sketsa (.ino atau .zip berisi .ino) di {akar}")
        lapor(f"   Kirim dulu dari laptop: kirim_teensy.bat")
        return 2

    import datetime
    lapor("\nVersi yang ada (URUT WAKTU, terbaru di bawah):")
    for i, d in enumerate(daftar, 1):
        tgl = datetime.datetime.fromtimestamp(d["waktu"]).strftime("%Y-%m-%d %H:%M")
        lapor(f"  {i:>2}. {d['nama']:<34} {versi_teks(d['versi']):>10}"
              f"  {tgl}" + ("  [zip]" if d["zip"] else ""))
    # NOMOR YANG KEMBAR. Sesudah ada patch di atas versi yang sama, dua
    # entri bisa sama-sama terurai jadi 'v1.12' -- folder aslinya dan zip
    # patch-nya. Di situ nomor versi berhenti jadi pembeda, dan yang
    # membedakan tinggal NAMA. Itu harus dikatakan, bukan dibiarkan terlihat
    # seperti dua baris yang kebetulan mirip.
    from collections import Counter
    _hitung = Counter(d["versi"] for d in daftar if d["versi"])
    _kembar = sorted({versi_teks(v) for v, n in _hitung.items() if n > 1})
    if _kembar:
        lapor(f"\n  ! Nomor versi KEMBAR: {', '.join(_kembar)}.")
        lapor("    Nomornya tidak bisa membedakan keduanya -- BACA NAMANYA.")
        lapor("    Yang dipilih '--terbaru' = baris paling bawah (berkas termuda).")
    if any(d.get("bentrok") for d in daftar):
        lapor("\n  ! Urutan NOMOR VERSI dan urutan WAKTU tidak sama.")
        lapor("    Yang dipakai di sini: WAKTU. Sebabnya v1.61 -- nomornya")
        lapor("    terbesar tapi umurnya paling tua, jadi nomor tidak bisa")
        lapor("    dipercaya sendirian. PERIKSA baris terakhir sebelum flash.")

    if args.versi:
        cocok = [d for d in daftar if args.versi.lower() in d["nama"].lower()]
        if not cocok:
            lapor(f"\n!! tidak ada yang cocok dengan '{args.versi}'")
            return 2
        if len(cocok) > 1:
            # '--versi v1.12' cocok dengan folder ASLI dan zip patch sekaligus.
            # Mengambil yang terakhir (termuda) adalah pilihan yang masuk akal,
            # tapi ia PILIHAN -- bukan kepastian. Jadi dikatakan apa saja yang
            # cocok, supaya kalau yang diambil bukan yang dimaksud, itu terlihat
            # SEBELUM Teensy disentuh, bukan sesudah.
            lapor(f"\n  ! '{args.versi}' cocok dengan {len(cocok)} entri:")
            for d in cocok:
                lapor(f"      {d['nama']}")
            lapor(f"    Diambil yang TERMUDA: {cocok[-1]['nama']}")
            lapor("    Sebut teks yang lebih khas kalau bukan itu yang kamu mau.")
        pilih = cocok[-1]
    elif args.terbaru:
        pilih = daftar[-1]
    else:
        try:
            n = input(f"\nNomor berapa? [{len(daftar)} = terbaru] ").strip()
        except EOFError:
            n = ""
        n = int(n) if n.isdigit() and 1 <= int(n) <= len(daftar) else len(daftar)
        pilih = daftar[n - 1]

    lapor(f"\n-> {pilih['nama']}  {versi_teks(pilih['versi'])}")

    with tempfile.TemporaryDirectory(prefix="flashteensy-") as tmp:
        kerja = Path(tmp)
        try:
            folder = siapkan_folder(pilih, kerja)
        except Exception as e:                              # noqa: BLE001
            lapor(f"!! gagal menyiapkan: {e}")
            return 2
        lapor(f"   sketsa: {folder}")

        butuh = pustaka_dibutuhkan(folder)
        lapor(f"\nPustaka pihak ketiga yang di-#include versi INI:")
        if not butuh:
            lapor("   (tidak ada -- semuanya bawaan core atau file lokal)")
        pasang = pustaka_terpasang()
        kurang = []
        for h in butuh:
            nama = PUSTAKA.get(h)
            if nama is None:
                # Header yang tidak dikenal DAN tidak ada di folder sketsa.
                # Dua kemungkinan dan keduanya perlu mata manusia: pustaka
                # baru yang Vincent pakai, atau berkas yang lupa ikut dikirim.
                tanda, ket = "?", "tidak dikenal -- pustaka baru, atau berkas kurang?"
            elif pasang is None:
                # Tidak seharusnya terjadi: periksa_alat() sudah memastikan
                # arduino-cli ada. Kalau tetap sampai sini, ia ada tapi
                # bermasalah -- dan itu harus dikatakan, bukan diberi '?'.
                tanda, ket = "!", f"{nama} -- arduino-cli ADA tapi gagal ditanya"
                kurang.append(nama)
            elif nama.lower() in pasang:
                tanda, ket = "v", nama
            else:
                tanda, ket = "!", f"BELUM ADA -- {nama}"
                kurang.append(nama)
            lapor(f"   [{tanda}] {h:<34} {ket}")
        tak_dikenal = [h for h in butuh if h not in PUSTAKA]
        if tak_dikenal:
            lapor(f"\n   ! {len(tak_dikenal)} header tidak dikenal. Kalau namanya")
            lapor(f"     terlihat seperti berkas Vincent sendiri (Calib.h, Hexapod.h,")
            lapor(f"     config.h...), berarti KIRIMANNYA TIDAK LENGKAP -- bukan")
            lapor(f"     pustaka yang kurang. Kirim ulang seluruh foldernya.")
        if kurang:
            lapor("\n   Pasang dulu:")
            for n in kurang:
                lapor(f'     arduino-cli lib install "{n}"')
            return 3

        if args.coba:
            lapor("\n--coba: berhenti di sini. Tidak compile, tidak flash.")
            cetak_cara_pakai()
            return 0

        hasil = kerja / "keluaran"
        hasil.mkdir(exist_ok=True)
        lapor(f"\nCompile ({FQBN}) -- ini beberapa menit di Pi...")
        rc, out = jalan(["arduino-cli", "compile", "--fqbn", FQBN,
                         "--output-dir", str(hasil), str(folder)])
        if rc != 0:
            lapor(out[-4000:])
            lapor("\n!! COMPILE GAGAL -- Teensy TIDAK disentuh sama sekali.")
            return 4
        lapor("   compile OK")

        hexes = sorted(hasil.glob("*.hex"))
        if not hexes:
            lapor("!! compile berhasil tapi tidak ada .hex -- ini aneh, berhenti.")
            return 4
        hexf = hexes[0]
        lapor(f"   {hexf.name}  {hexf.stat().st_size // 1024} KB")

        if args.hanya_compile:
            simpan = Path.home() / hexf.name
            shutil.copy2(hexf, simpan)
            lapor(f"\n--hanya-compile: .hex disimpan di {simpan}")
            return 0

        port = args.port or cari_port()
        lapor(f"\nFlash. Port serial: {port or '(tidak ketemu)'}")
        # Dicatat SEBELUM flash supaya sesudahnya kita bisa tahu node mana
        # yang BARU -- bukan menebak dari nama.
        port_sebelum = sorted(set(glob.glob("/dev/ttyACM*"))
                              | set(glob.glob("/dev/ttyUSB*")))
        hud_tadi = hentikan_hud()
        try:
            ok, ket = minta_bootloader(port)
            lapor(f"   {ket}")
            if not ok:
                lapor("   >> TEKAN TOMBOL PROGRAM di Teensy sekarang. <<")
            rc, out = jalan(["teensy_loader_cli", "--mcu=TEENSY41",
                             "-w", "-v", str(hexf)])
            lapor(out.strip()[-2000:])
            if rc != 0:
                lapor("\n!! FLASH GAGAL. Yang biasanya jadi sebab:")
                lapor("   - tombol program belum ditekan (kalau reboot lunak gagal)")
                lapor("   - Teensy tidak terlihat: cek 'lsusb' dan kabelnya")
                lapor("   - izin udev: taruh 00-teensy.rules di /etc/udev/rules.d/")
                return 5
            lapor("\n   FLASH SELESAI menulis.")

            # MENULIS BUKAN BERARTI JALAN. Ini kekurangan yang paling mahal
            # di versi sebelumnya: skrip mencetak "FLASH SELESAI", menyalakan
            # HUD lagi, lalu pulang -- sementara Teensy bisa tertinggal di
            # mode bootloader (HalfKay). Di keadaan itu ia muncul sebagai
            # perangkat HID, BUKAN serial, jadi /dev/ttyACM* tidak ada sama
            # sekali dan HUD jatuh ke SIMULASI. Dari luar: "flash sukses tapi
            # Teensy tidak nyambung ke mission HUD" -- dua pernyataan yang
            # terasa bertentangan padahal keduanya benar.
            lapor("\n   Menunggu Teensy kembali sebagai port serial...")
            balik = tunggu_port(20.0, kecuali=port_sebelum)
            if balik is None:
                balik = tunggu_port(3.0)        # mungkin namanya sama
            if balik is None:
                # COBA PULIHKAN SENDIRI. 'teensy_loader_cli -b' memerintahkan
                # HalfKay BOOT tanpa memprogram apa pun -- persis yang
                # dibutuhkan kalau sketsanya sudah ditulis tapi papannya
                # tertinggal di bootloader.
                #
                # Ini bukan sekadar kenyamanan: tanpa ini satu-satunya jalan
                # keluar adalah menekan tombol fisik, dan seluruh gunanya
                # flash-dari-Pi justru supaya tidak perlu ada orang di sebelah
                # robot. Tombol fisik sebagai satu-satunya pemulihan
                # meniadakan alasan alat ini ada.
                lapor("      belum kembali -- mencoba menyuruhnya BOOT"
                      " (teensy_loader_cli -b)...")
                rc_b, out_b = jalan(["teensy_loader_cli", "--mcu=TEENSY41", "-b"])
                if out_b.strip():
                    lapor("      " + out_b.strip().replace("\n", "\n      ")[:600])
                balik = tunggu_port(12.0, kecuali=port_sebelum) or tunggu_port(2.0)
                if balik:
                    lapor(f"      PULIH: port kembali di {balik}")
            if balik is None:
                lapor("\n   !! PORT SERIAL TIDAK KEMBALI dalam 20 detik.")
                lapor("      Sketsanya sudah ditulis, tapi Teensy tampaknya")
                lapor("      masih di mode BOOTLOADER. Di mode itu ia perangkat")
                lapor("      HID, bukan serial -- jadi /dev/ttyACM* tidak ada")
                lapor("      dan HUD akan jatuh ke SIMULASI.")
                lapor("      ('-b' di atas sudah dicoba dan tidak menolong.)")
                lapor("\n      Yang biasanya menyelesaikan, berurutan:")
                lapor("      1. Tekan tombol PROGRAM di Teensy sekali (reboot).")
                lapor("      2. Cabut-colok kabel USB Teensy ke Pi.")
                lapor("      3. Periksa: ls /dev/ttyACM*   dan   lsusb")
                lapor("         'Teensy Halfkay Bootloader' di lsusb = masih di")
                lapor("         bootloader; tidak ada sama sekali = daya/kabel.")
                lapor("      4. Kalau tetap tidak ada: .venv/bin/python ~/M/cek_serial.py")
                lapor("\n      DAYA juga tersangka yang sah di robot ini: servo")
                lapor("      menarik arus besar saat PWM menyala, dan Teensy yang")
                lapor("      brownout di awal boot bisa jatuh kembali ke bootloader.")
                lapor("      Periksa kartu 'Daya & suhu Pi' -- bit throttle melatch")
                lapor("      dan tetap menempel sampai reboot, jadi ia masih bisa")
                lapor("      dibaca sesudah kejadiannya lewat.")
                lapor("\n      CATATAN: 'dmesg | grep serial' TIDAK bisa menjawab ini.")
                lapor("      Teensy muncul sebagai cdc_acm/ttyACM, dan tidak satu")
                lapor("      pun barisnya mengandung kata 'serial'. Pakai:")
                lapor("        dmesg | grep -iE 'acm|teensy|usb 1-'")
            else:
                lapor(f"      port kembali: {balik}")
                ok2, ket2 = sapa_firmware(balik)
                if ok2 is None:
                    lapor(f"      {ket2}")
                elif ok2:
                    lapor(f"      FIRMWARE HIDUP -- {ket2}")
                else:
                    lapor(f"      !! {ket2}")
                    lapor("      Node-nya ada tapi firmware tidak bersuara. Bisa")
                    lapor("      berarti sketsa berhenti di setup(), atau yang")
                    lapor("      muncul itu perangkat lain. Periksa dengan")
                    lapor("      .venv/bin/python ~/M/cek_serial.py")
        finally:
            # Dijalankan WALAU flash gagal. Kalau tidak, HUD tertinggal mati
            # dan gejala berikutnya jadi 'site cannot be reached' -- salah
            # petunjuk yang sudah pernah memakan waktu sekali di proyek ini.
            if hud_tadi:
                nyalakan_hud()

    lapor("\nSesudah ini, periksa di HUD: tab Manual -> 'Daftar perintah (h)'")
    lapor("untuk memastikan firmware yang jalan memang versi yang baru.")
    return 0


def cetak_cara_pakai():
    """Dicetak saat --coba, supaya perintah berikutnya ada DI DEPAN MATA.

    Kenapa perlu: petunjuk di README ditulis sebagai
        ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"
    dan '-t' itu pilihan SSH, bukan perintah shell. Kalau kamu sudah berada
    DI Pi, menempelkan '-t' di depan menghasilkan '-bash: -t: command not
    found' -- dan itu benar-benar terjadi 12 Sep 2026. Kesalahan petunjuk,
    bukan kesalahan pemakainya. Jadi perintahnya dicetak di tempat ia
    dibutuhkan, dalam dua bentuk.
    """
    lapor("\n" + "-" * 62)
    lapor("Langkah berikutnya -- pilih sesuai kamu sedang di mana:")
    lapor("")
    lapor("  SUDAH DI PI (prompt 'bima@terra-core:~ $'):")
    lapor("     python3 ~/M/flash_teensy.py --hanya-compile --terbaru   # aman")
    lapor("     python3 ~/M/flash_teensy.py --terbaru                   # flash")
    lapor("     (TANPA ssh, TANPA -t. '-t' itu pilihan ssh, bukan perintah")
    lapor("      shell -- di Pi ia menjawab '-bash: -t: command not found'.)")
    lapor("")
    lapor("  DARI LAPTOP:")
    lapor('     ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"')
    lapor("     (-t di sini PERLU: supaya pesan 'tekan tombol PROGRAM' benar-")
    lapor("      benar muncul di layarmu, bukan tertahan di buffer.)")


if __name__ == "__main__":
    sys.exit(main())
