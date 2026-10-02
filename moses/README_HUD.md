# Mission HUD — hexapod SAR R2C

Panel misi berbasis web di Raspberry Pi 5: video kamera, deteksi korban,
log serial Teensy, kendali manual, dan editor tabel lintasan — satu halaman,
tanpa VNC.

**Buka di:** `http://terra-core:5000/`

Dilayani `http.server` pustaka standar Python (tanpa Flask). Bagian berat
(sesi ONNX, kamera) di-impor dari `detect.py`, tidak disalin, supaya angka di
`EKSPERIMEN.md` tetap mengacu ke kode yang sama.

Riwayat pengembangan yang dulu ada di berkas ini (crosscheck per versi
firmware, percobaan penengahan, bug lama) ada di riwayat git.

---

## 1. Berkas

| Berkas | Jalan di | Gunanya |
|---|---|---|
| `upload_to_pi.bat` | PC Windows | kirim berkas HUD ke Pi (`~/M`) |
| `kirim_teensy.bat` | PC Windows | kirim sumber firmware ke Pi (`~/teensy/`) |
| `upload_teensy.bat` | PC Windows | compile + upload firmware langsung dari laptop |
| `mission_hud.py` | Pi | aplikasinya |
| `operator_control.py` | Pi | di-impor `mission_hud.py` — wajib ikut terkirim |
| `flash_teensy.py` | Pi | compile + flash firmware dari Pi |
| `run_hud.sh` / `install_service.sh` | Pi | jalankan manual / pasang layanan `r2c-hud` |
| `cek_serial.py` | Pi | diagnosis kalau Teensy tidak terbaca |
| `hemat_daya.sh` | Pi | turunkan konsumsi daya |
| `test_mission_hud.py` | Pi atau PC | 1151 uji otomatis, tanpa robot dan tanpa kamera |
| `ARENA_GUIDEBOOK.md` | — | ringkasan arena, dimensi, penilaian dari guidebook |

`test_mission_hud.py` juga menjaga bahwa setiap modul yang di-impor HUD ada di
daftar `BERKAS` milik `upload_to_pi.bat` — HUD yang kehilangan satu impor mati
di Pi dengan `ModuleNotFoundError`.

---

## 2. Pasang dan jalankan

```bash
# PC Windows, dari folder moses
upload_to_pi.bat

# Pi, sekali saja
ssh bima@terra-core
cd ~/M
chmod +x run_hud.sh install_service.sh hemat_daya.sh
.venv/bin/pip install pyserial
./install_service.sh          # nyala sendiri tiap boot
sudo reboot
```

Sesudah upload berikutnya cukup `sudo systemctl restart r2c-hud`.

**Upload tidak butuh internet** — `scp` cuma butuh Pi dan laptop di jaringan
yang sama (hotspot HP tanpa paket data, router lokal, atau kabel Ethernet
langsung ke `terra-core.local`). Yang butuh internet hanya Tailscale.

Jalankan manual (hentikan layanannya dulu — port web, kamera, dan port serial
tidak bisa dipakai dua proses):

```bash
./run_hud.sh                         # web di :5000
./run_hud.sh --port /dev/ttyACM0     # + Teensy
./run_hud.sh --web-port 5001         # kalau :5000 dipakai
./run_hud.sh --calib kalib_X.json    # muat kalibrasi tersimpan
```

UART alih-alih USB: `TEENSY=/dev/ttyAMA0 ./install_service.sh`. Port UART harus
disebut eksplisit — node-nya selalu ada walau kabelnya kosong.

---

## 3. Serah-terima kendali dengan Teensy

Satu penguasa pada satu waktu:

```
Teensy  #KORBAN AMBIL <ruas> <arah> <cermin>  -> parkir, kendali kaki milik Raspi
Raspi   menengahkan badan ('r', 'O', 't'), menahan
Raspi   m2                                    -> kendali kaki kembali ke Teensy
Teensy  sekuens capit
Teensy  #LEPAS <ruas>                         -> masih memegang kaki?
Raspi   m9                                    -> tidak; misi lanjut
```

- Raspi **hanya boleh menjawab `m2`, `m3`, `m9`**, dan hanya saat firmware
  menunggunya. Byte lain masuk parser penuh firmware — `W` menulis kalibrasi,
  `m0` membatalkan misi.
- `<arah>` (0..3) dan `<cermin>` sudah hasil pencerminan di firmware; Raspi
  tidak menghitungnya ulang.
- Batas tunggu firmware: 20 detik untuk `m2`, 5 detik untuk `m9`. Habis waktu
  tidak menggagalkan apa pun — sekuens jalan dengan sudut tetap.
- **Tanpa Pi**, ketik `m8 0` di firmware sebelum misi supaya ruas AMBIL tidak
  menunggu sama sekali.

---

## 4. Editor tabel lintasan

Kartu Lintasan membaca tabel dari Teensy (`m5d` → baris `#TABR`) dan mengirim
balik per baris (`m5s`). `m5` membalas `#TABV <crc> <n>`; HUD menghitung CRC
yang sama dari draft-nya, jadi "cocok" berarti identik, bukan kira-kira. Sesudah
Teensy reset CRC kembali ke tabel flash dan HUD mengirim ulang sendiri.

- `m5+ <idx>` / `m5- <idx>` sisip / hapus baris; peta poin ikut bergeser.
- `m5r` memulangkan RAM ke tabel flash.
- Semua ditolak selama misi berjalan.
- `m5d` mencetak isi **mentah**; tabel yang **berlaku** (tercermin) dicetak
  `m4` tanpa argumen.

Daftar enum di halaman (`LTS_KOLOM` dan `ltsEkspor`) harus sama dengan
firmware. HUD yang lebih tua dari firmware menampilkan henti yang salah untuk
nilai yang tidak dikenalnya — jangan sunting baris itu.

---

## 5. Kalibrasi HUD

**Tidak ada autosave dan tidak ada autoload.** Default di `class Kalib` dalam
`mission_hud.py` berlaku tiap start. **Simpan kalibrasi** menulis
`kalib_<timestamp>.json`; muat lagi dengan `--calib`.

Beberapa yang paling sering disetel:

| Parameter | Default | Arti |
|---|---|---|
| `standoff_cm` | 20.0 | jarak kerja kamera |
| `yaw_tol_deg` | 6.0 | sama dengan `HEADING_TOLERANCE_DEG` firmware |
| `cx_offset_px` | 0.0 | piksel-tengah kamera vs garis tengah capit |
| `bbox_h_min/max` | 0.35 / 0.90 | pita tinggi bbox saat MENILAI |
| `capit_cm` | 10.0 | jarak berhenti untuk mencapit |
| `capit_buka_persen` | 20.0 | bukaan capit dari sisi Raspi |
| `condong_mm` | 60.0 | dikirim ke firmware sebagai `condong.mm` (tombol Kirim + simpan) |

`capit_buka_persen` punya kembaran di firmware (`KORBAN_GRIP_BUKA` di
`config.h`). Keduanya menggerakkan capit yang sama — samakan.

---

## 6. Flash Teensy

Dari Pi (Teensy tersambung ke Pi):

```bat
:: PC Windows
moses\kirim_teensy.bat
```
```bash
# lalu
ssh bima@terra-core "python3 ~/M/flash_teensy.py --coba --terbaru"   # periksa dulu
ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"
```

`flash_teensy.py` menghentikan `r2c-hud`, mem-flash, menunggu firmware menjawab
`m`, lalu menyalakan `r2c-hud` lagi.

Dari laptop (Teensy dicolok ke laptop): `moses\upload_teensy.bat`. **Daya robot
(VIN) harus menyala** — pad VUSB-VIN sudah dipotong, Teensy tidak mengambil
daya dari USB. Tutup Serial Monitor Arduino IDE dulu; ia memegang port.

---

## 7. Daya

Pi 5 pernah mencatat `Undervoltage detected` beberapa detik sesudah boot, dan
kamera sekarang menyala sepanjang misi. Pakai catu 5 V / 5 A yang sungguhan.
Strip status menyala merah saat under-voltage atau throttle **sedang** terjadi.

---

## 8. Kalau bermasalah

| Gejala | Sebab | Tindakan |
|---|---|---|
| Semua field `-`, video tetap hidup | JavaScript halaman mati | konsol browser (F12); `test_mission_hud.py` memarse JS dengan `node --check` |
| `serial rx 0 B`, `tx` naik | ada yang memakan balasan Teensy | `systemctl is-active ModemManager brltty`; `sudo fuser -v /dev/ttyACM0` |
| `serial` merah | sebabnya tertulis | `cek_serial.py` |
| `LOOP BEKU n detik` | loop utama tersendat | `journalctl -u r2c-hud -n 50` |
| HUD mati, `ModuleNotFoundError` | modul tidak ikut terkirim | tambahkan ke `BERKAS` di `upload_to_pi.bat` |
| `Permission denied` saat `./run_hud.sh` | `scp` tidak membawa bit executable | `chmod +x run_hud.sh` |
| Port 5000 dipakai | proses lain | `sudo ss -ltnp "sport = :5000"` atau `--web-port 5001` |
| Teensy tidak muncul di `lsusb` | VIN mati / kabel di laptop | nyalakan daya robot; pindahkan kabel |
| `W:onnxruntime … /sys/class/drm/card0` | onnxruntime mencari GPU | abaikan |

```bash
journalctl -u r2c-hud -f       # log HUD
sudo systemctl restart r2c-hud # sesudah upload
sudo systemctl stop r2c-hud    # sebelum run manual / cek_serial.py
```
