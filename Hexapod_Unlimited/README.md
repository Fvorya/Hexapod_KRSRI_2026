# Hexapod Unlimited

Firmware hexapod R2C untuk KRSRI / SAR UNLIMITED 2026, Teensy 4.1.

- **Angka dan alasan tiap angka** ada di `config.h` — hampir tiap konstanta
  membawa tabel ukurannya di komentar.
- **Tabel lintasan dan arti kolomnya** ada di `Misi.h`; lintasannya sendiri di
  `Misi.cpp` (`RUAS_BAKU[]`). Lihat juga [`MISI.md`](MISI.md).
- **Daftar perintah serial lengkap**: ketik `h`. Itulah sumber kebenarannya;
  ringkasan di bawah cuma yang paling sering dipakai.
- **Cara memverifikasi sebelum flash**: `CLAUDE.md` di akar repo (lima
  pemeriksaan, tanpa robot).

Riwayat pengembangan yang dulu ada di berkas ini ada di riwayat git.

---

## 1. Build dan flash

Dari laptop, Teensy dicolok ke laptop, **daya robot (VIN) menyala** (pad
VUSB-VIN dipotong — Teensy tidak mengambil daya dari USB):

```bat
moses\upload_teensy.bat
```

Dari Raspberry Pi: `moses\kirim_teensy.bat` lalu
`ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"` — lihat
`moses/README_HUD.md` bagian 6.

Robot selalu **boot lemas** (PWM mati). Tidak ada kaki yang bergerak sampai
`b`. Pengecualiannya `DEMO_BOOT` di `config.h` — biarkan 0.

---

## 2. Arsitektur

Satu fasad, `Hexapod`, membungkus rantai dari perintah gerak sampai pulsa:

```
Navigation / perintah serial
        |
Hexapod::walk(maju, geser, putar)        -1..1
        |
HexaGait            slew vektor, ramp profil medan, tripod -> legTargets[6]
        |
Hexapod::legSolve() + pose badan (di-ramp) + zOff per kaki (EEPROM 2048)
        |
LegInverseKinematics -> sudut coxa / femur / tibia
        |
angleToPulse()      invert, offset, trim per servo
        |
HexaServos          commit 18 pulsa tiap SERVO_COMMIT_MS (20 ms)
```

| Berkas | Tanggung jawab |
|---|---|
| `Hexapod_Unlimited.ino` | objek global, `setup()`, `loop()`, parser perintah serial |
| `Hexapod.*` | fasad: profil, pose badan, IK per kaki, pulsa, arm/disarm, `d` |
| `HexaGait.*` | gait tripod berbasis waktu, odometri maju dan geser |
| `LegInverseKinematics.*` | IK 3-DOF satu kaki |
| `HexaServos.*` | dua PCA9685, gerbang keselamatan PWM |
| `HexaArm.*` + `ArmInverse.*` | lengan depan 3 sendi + grip, grip belakang; slew trapesium |
| `Imu.*` | parser WIT (Yahboom 10-axis) |
| `LidarArray.*` | 6x VL53L1X lewat mux TCA9548A, round-robin non-blokir |
| `Navigation.*` | kompas arena, pivot, ikut-dinding, geser, mundur — non-blokir |
| `NavKoreksi.h` | keputusan koreksi-sambil-berhenti, murni (diuji `cek_koreksi`) |
| `Misi.*` | lapisan misi; lintasan adalah tabel data |
| `Skor.*` | pembukuan poin per ruas |
| `Tampilan.*` | OLED dan empat tombol |
| `Calib.*` | blob parameter `Q`/`W` di EEPROM 0 |
| `EEMap.h` | tata letak EEPROM + `static_assert` |
| `config.h` | konstanta perangkat keras, geometri, ambang |
| `types.h` | `Vec3`, helper sudut; murni |

**Urutan di `loop()` yang wajib dijaga:** `misi.update()` SEBELUM
`nav.navUpdate()` — keduanya membaca sampel LiDAR yang sama, dan yang lebih dulu
berhak memutuskan. Tidak ada `delay()`; satu-satunya jalur yang memblokir
adalah kalibrasi pivot `C`.

**Urutan di `setup()` yang wajib dijaga:** `Calib::load()` PALING AWAL. Tanpa
itu `gParam[]` nol semua — pulse min/max 0, servo lemas total, gait tidak
bergerak.

---

## 3. Bring-up dan kalibrasi

Serial 115200, mode *Newline* (atau `jalankan.bat` di akar repo).

1. **Periksa, jangan berdiri dulu.** Log boot: enam LiDAR OK, Calib valid,
   ServoMap termuat. `I` lalu `l`; gerakkan tangan di depan tiap sensor dan
   cocokkan nama arahnya. Arah ch0 dan ch2 belum pernah diverifikasi langsung.
2. **Berdiri.** Topang robot, `b`. `d` memeriksa rantai IK; kolom `rng` harus
   `ok` semua.
3. **Kompas arena.** Hadapkan robot ke **arah lorong pertama dari HOME** (bukan
   utara magnet), tunggu `y` tenang, `c0`. Ulangi `c1`..`c3` searah jarum jam.
   `e` menyimpan, `k` memeriksa. Seluruh kolom arah tabel bergantung padanya.
4. **Kalibrasi pivot.** `C` (memblokir, biarkan selesai), lalu **`S`**. Tanpa
   `S` hasilnya hilang saat reset. `K` memeriksa.
5. **Uji pivot.** `o1`, amati simpangan di aliran `y` — harus dalam 6 der.
6. **Bias sensor samping.** Robot sejajar lorong, `Y0`.
7. **Trim servo** bila perlu: `Yt<slot> <us>` (batas +-25), simpan `YtW`.

---

## 4. Perintah yang paling sering dipakai

| Perintah | Arti |
|---|---|
| `b` / `x` | berdiri (servo hidup) / LEMAS |
| `s`, `x`, Enter | hentikan apa pun |
| `l`, `I` | tabel LiDAR / pindai + pulihkan sensor mati |
| `y` | aliran yaw/roll/pitch |
| `T0`..`T5` | profil: datar, tangga, merunduk, sempit, kail, tanjak |
| `U<cm>` | naik tangga: profil TANJAK, heading terkunci kompas, rem jarak `<cm>`, buta depan, koreksi sambil berhenti, berhenti di puncak (gyro) |
| `V<cm>` / `J<cm>` | geser sampai dinding kanan `<cm>` / maju-mundur sampai belakang `<cm>` |
| `r` / `t` / `0` | rotasi badan / geser badan / nolkan pose badan |
| `R`, `g<0-100>`, `as<b> <s> <p>` | lengan REHAT / grip depan / tembak tiga sendi |
| `aa` / `at` | sekuens AMBIL / TARUH saja, untuk menyetel di meja |
| `q`, `Q<nama> <nilai>`, `W` | lihat / ubah / simpan parameter |
| `m` | status misi |
| `m1` | misi penuh dari ruas 0 |
| `m4` | tabel lintasan yang BERLAKU (sudah tercermin) |
| `m4 <a> [b]` | jalankan ruas a..b saja |
| `m6 <idx>` / `m7 <idx> <cm>` | ukur panjang ruas / setel panjangnya (RAM) |
| `m0` | batalkan misi |
| `m5`, `m5d`, `m5s`, `m5+`, `m5-`, `m5r` | editor tabel (dipakai HUD) |
| `m8 0` | jangan tunggu vision Raspi (wajib kalau Pi tidak ikut) |
| `m2` / `m3` / `m9` | jawaban Raspi: lanjut / ulang ruas / kaki dilepas |

Tombol fisik: D6 tekan = JALAN misi; tombol STOP tekan = rem, tahan = rem +
nolkan ruas dan poin; D4 tekan = catat satu arah kompas (minta konfirmasi),
tahan = kalibrasi pivot; D3 tekan = arena cermin on/off (dikunci selama misi
berjalan), tahan = siapkan robot (`I`, `b`, `R`).

---

## 5. Kebijakan kegagalan misi

Ruas yang gagal karena sebab lunak — pivot meleset, batas waktu ruas, heading
hilang, perataan ditolak — **dilewati** tanpa poin, dan misi lanjut. Yang
membatalkan misi cuma tiga: servo lemas, waktu kontes habis, dan LiDAR mati.
LiDAR yang putus dipindai ulang sekali (`LIDAR_ULANG_MAKS`); kalau pulih, ruas
itu **dilanjutkan** dari titik nolnya, bukan diulang.

---

## 6. Peta EEPROM

| Alamat | Isi | Ditulis oleh |
|---|---|---|
| 0 | CalibBlob — parameter, offset sudut | `W` |
| 1024 | ServoMap — trim & invert (menang atas Calib) | `YtW` |
| 1792 | kompas arena, 4 arah | `e` |
| 2048 | pivot, odometri, zOff per kaki | `S`, `Yz` |
| 2304 | enam profil medan | penyetelan profil |

Menaikkan `CALIB_VERSION` membuang blob di alamat 0 (parameter kembali ke
default kode); blok lain punya versinya sendiri dan tidak ikut terbuang.
`M` mencetak peta beserta kapasitas chip.
