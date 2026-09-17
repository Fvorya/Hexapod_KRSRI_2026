# Mission HUD — hexapod SAR R2C

Panel misi berbasis web untuk Raspberry Pi 5. Menampilkan seluruh rencana misi,
state machine yang sedang berjalan, keputusan vision, dan status sambungan ke
Teensy — semuanya di satu halaman, tanpa VNC.

**Buka di:** `http://terra-core:5000/`

Halamannya dilayani `http.server` dari pustaka standar Python. Tidak butuh
Flask, tidak butuh desktop, tidak butuh window.

---

## 1. Berkas & tempat jalannya

| Berkas | Jalan di mana | Gunanya |
|---|---|---|
| `upload_to_pi.bat` | **PC Windows (CMD)** | kirim berkas ke Pi |
| `upload_teensy.bat` | **PC Windows (CMD)** | kompilasi & upload firmware Teensy |
| `mission_hud.py` | Raspberry Pi | aplikasinya |
| `run_hud.sh` | Raspberry Pi | jalankan manual |
| `install_service.sh` | Raspberry Pi, sekali | pasang layanan systemd |
| `hemat_daya.sh` | Raspberry Pi | turunkan konsumsi daya |
| `cek_serial.py` | Raspberry Pi | diagnosis kalau Teensy tidak terbaca |
| `test_mission_hud.py` | Pi atau PC | 341 uji otomatis, bukan UI |
| `simpan_arsip.sh` | Pi atau PC | snapshot versi ke `arsip/` sebelum mengubah |
| `arsip/RIWAYAT.md` | dibaca saja | riwayat semua rilis r1–r9 + apa yang berubah |
| `ARENA_GUIDEBOOK.md` | dibaca saja | ringkasan arena, dimensi, penilaian & aturan dari guidebook |

`test_mission_hud.py` **tidak punya halaman** dan tidak menyentuh robot — dia
memverifikasi logika supaya kode yang rusak tidak sampai ke lapangan.
Uji 19 menjalankan seluruh rantai AMBIL tanpa robot: termasuk yang harus
**gagal** (kelewat dekat, korban tak terlihat, batas waktu habis).

`mission_hud.py` **mengimpor** `load_session`, `letterbox`, `postprocess`,
`open_camera`, `CameraThread` dari `detect.py` — tidak menyalinnya, supaya angka
benchmark di `EKSPERIMEN.md` tetap mengacu ke kode yang sama persis.

---

## 2. Jalan cepat

```bash
# di PC Windows (CMD), dari folder moses
upload_to_pi.bat

# di Pi, sekali saja
ssh bima@terra-core
cd ~/M
chmod +x run_hud.sh install_service.sh hemat_daya.sh
.venv/bin/pip install pyserial
./install_service.sh          # nyala sendiri tiap boot
sudo reboot
```

Sesudah itu: nyalakan robot, tunggu ~25 detik, buka `http://terra-core:5000/`.

**Pastikan versinya benar** — tertulis di pojok kanan atas halaman. Kalau
tombolnya berbeda dari dokumen ini, berkasnya belum ter-upload.

Kalau memakai UART (bukan USB): `TEENSY=/dev/ttyAMA0 ./install_service.sh`.
Port UART harus disebut eksplisit — node UART selalu ada walau tidak ada apa pun
di ujung kabel, jadi deteksi otomatis akan berbohong.

---

## 3. Tata letak halaman

**Strip status** selalu terlihat, tidak pernah sembunyi di balik tab:

```
state JEJAK → STANDOFF | serial /dev/ttyACM0 | kamera /dev/video0 | teensy DIAM
depan 23 cm | vision NYALA | serial tx 412 · rx 18734 B | daya 58C | data 0.2s
```

| Kolom | Merah berarti |
|---|---|
| `serial` (port) | tidak tersambung — sebabnya ikut ditulis |
| `serial tx/rx` | `rx 0 B` = kita mengirim tapi **tidak ada yang membalas** |
| `kamera` | tidak aktif |
| `daya` | under-voltage / throttle **sedang terjadi** |
| `LOOP BEKU n detik` | loop utama berhenti; data yang tampil sudah basi |

Di bawahnya: **video + log serial** di kiri, **lima tab** di kanan.

| Tab | Isi |
|---|---|
| **Robot** | Berdiri, Mulai misi, Stop, LEMAS, ganti mode, Restart — plus kartu **Jejak** dan kartu **Ambil korban (otomatis)** |
| **Uji korban** | tiga tombol uji, penilaian slot, state machine + kotak GAGAL |
| **Manual** | tombol perintah + kotak teks bebas + Kirim PAKSA |
| **Misi** | daftar 19 misi |
| **Kalibrasi** | semua knob + simpan |

Setiap tombol punya **tooltip**: arahkan kursor, penjelasannya muncul lengkap
dengan perintah firmware yang dikirim.

---

## 4. Mode kerja

### JEJAK — untuk memahami & menguji

Kamera menyala terus, boneka boleh digeser ke mana saja, robot memutar badan
mengikutinya sampai simpangan di bawah `yaw_tol_deg`, lalu **berhenti dan
menunggu perintahmu**. Tidak pernah pindah state sendiri, tidak ada batas waktu.

Tiga baris pemantau: `sasaran` (kelas + conf, berwarna), `simpangan`
(derajat + cm, hijau kalau sudah cukup tengah), `deteksi` (berapa deteksi mentah
dan tinggi bbox terbesar — ini yang memberitahu kenapa sesuatu ditolak).

Tombol pendamping: **Maju ke korban** (LiDAR menutup jarak ke `standoff_cm`),
**Capit BUKA/TUTUP** (`g100`/`g0`), **Lengan OFF** (`n`).

### AMBIL otomatis — centering → maju → capit

Tombol **Mulai AMBIL otomatis** menjalankan lima langkah berurutan. Butuh mode
**KENDALI**; kalau masih BACA, dia menolak dan bilang begitu.

| State | Yang dikerjakan | Batas |
|---|---|---|
| `AMBIL_TENGAH` | tengahkan badan dengan kamera sampai simpangan ≤ `yaw_tol_deg` | 30 s |
| `AMBIL_MAJU` | maju bertahap sampai LiDAR depan = `capit_cm` | 30 s |
| `AMBIL_SIAP` | `g100` buka capit, `a<lengan_r> <lengan_h>` posisikan lengan | 10 s |
| `AMBIL_JEPIT` | `g0` tutup capit | 8 s |
| `AMBIL_ANGKAT` | `a<lengan_r> <lengan_h + lengan_angkat>`, lalu `BERES` | 10 s |

Dipecah lima state, bukan satu blok, supaya kalau berhenti kolom **sebab** di
tab Misi menyebut **langkah mana** yang gagal — bukan cuma "gagal".

Yang sengaja dibuat begini:

- **Maju maksimal 6 cm sekali jalan**, lalu ukur ulang. Odometri gait meleset
  beberapa cm tiap langkah; satu lompatan besar ke arah korban berisiko
  menabraknya.
- **Kelewat dekat = gagal bersih, bukan dipaksakan.** Firmware belum punya
  perintah mundur. Robot harus dimundurkan dengan tangan.
- **`ambil_hanya_korban` (default True)** menolak menjalankan rantai kalau kelas
  yang terbaca `dummy`. Matikan di tab Kalibrasi kalau memang mau mencobanya.
- **Tidak dijalankan sendiri sesudah JEJAK.** Mulai dari sini robot bergerak
  maju ke korban dan sulit dibatalkan pelan-pelan — jadi operator yang menekan.
- Tombol **Batalkan AMBIL** mengosongkan antrean, mengirim `s`, kembali ke
  `IDLE`. Servo tetap hidup (yang mematikan servo cuma LEMAS).

⚠️ **`capit_cm = 12.0` itu tebakan aman, bukan hasil ukur.** Lengan hanya
menjangkau ±44 mm dari bahu yang duduk 50 mm di depan pusat badan, jadi angka
ini hampir pasti perlu diubah setelah diukur di robot sungguhan. Ukur dulu
dengan **Capit BUKA** + **Lengan** manual sebelum menjalankan rantai penuh.

⚠️ **Tidak ada sensor cengkeraman.** HUD tidak bisa tahu korbannya benar-benar
terpegang; langkah terakhir hanya menulis "PERIKSA MATA" di log. Jangan
mengandalkan `BERES` sebagai bukti keberhasilan.

### Uji berurutan

| Tombol | Yang terjadi | Butuh |
|---|---|---|
| Vision saja (tanpa gerak) | 9 frame, gerbang ketat, putuskan. **Nol perintah gerak** | kamera |
| 1 slot + gerak | standoff 20 cm → centering → nilai → `m2`/`m3` | Teensy + servo |
| 3 slot + gerak | + geser 8 cm antar slot lewat heading arena | + kompas terkalibrasi |

### Manual

Semua perintah firmware lewat tombol atau kotak teks. Dua tombol kirim:
**Kirim** (lewat whitelist) dan **Kirim PAKSA** (menembus daftar hitam, dengan
konfirmasi, tercatat `[PAKSA]` di log).

---

## 5. Apa itu "slot"

Slot bukan konsep vision, tapi konsep **arena**, khusus K-1.

Ruang K-1 lebarnya 40 cm, isinya 2 dummy + 1 korban asli di tiga tanda silang
berjarak 8 cm, posisinya **diundi** setelah robot ditaruh di Home.

Di jarak 20 cm bidang pandang kamera cuma 28 cm — tidak muat 40 cm — tapi boneka
tetangga yang 8 cm ke samping **tetap masuk frame** di 21,8° dari sumbu. Kalau
kamera disuruh menilai seluruh pemandangan, dia menilai tiga boneka sekaligus.

Solusinya: jangan suruh kamera memisahkan mereka — **suruh kaki robot yang
memisahkan**. Berhenti di depan satu boneka, gerbang ROI membuang tetangganya,
nilai; geser 8 cm; ulangi. Satu posisi berhenti = satu slot.

Guidebook menjamin **tepat satu** asli per ruang, jadi setelah semua slot dinilai
yang dipilih adalah margin terbesar — dan kalau dua slot yakin dummy, slot ketiga
pasti korban walau conf-nya sedang saja.

---

## 6. Gerbang vision — dipilih per state

Ini pernah jadi bug: gerbang ketat dipakai untuk semua state, sehingga CENTERING
membuang sasaran yang justru sedang dikejarnya. Sekarang:

| State | Tugasnya | Gerbang |
|---|---|---|
| JEJAK, CENTERING | **mengejar** — sasaran jauh & melenceng | tanpa ROI, pita `jejak_h_min..max` (0,06–0,98) |
| LIHAT | **menilai** — jarak terkunci 20 cm | ROI ±10°, pita `bbox_h_min..max` (0,35–0,90) |

Deteksi yang ditolak digambar **abu** dengan alasannya tertulis:
`korban 0.91 -- terlalu jauh 0.25<0.35`, `dummy 0.88 -- di luar ROI`.

Warna kotak: **hijau = KORBAN, merah = DUMMY**, label huruf hitam di atas blok
warna penuh. Saat menjejak ada panah dari sumbu kamera ke sasaran.

**Model default 640 px** (F1 0,999), bukan 320 px — robot berdiri diam saat
menilai, jadi laju frame bukan penghambat.

---

## 7. Kalibrasi

**Tidak ada autosave dan tidak ada autoload.** Nilai default di dalam
`mission_hud.py` itulah yang berlaku tiap start.

| Parameter | Default | Arti |
|---|---|---|
| `rotate180` | **True** | kamera di robot ini terpasang terbalik |
| `hfov_deg` | 70.4 | ukur ulang kalau ganti lensa |
| `roi_half_deg` | 10.0 | ±10° = ±3,5 cm di 20 cm |
| `cx_offset_px` | 0.0 | piksel-tengah kamera vs garis tengah capit |
| `standoff_cm` | 20.0 | jarak kerja kamera |
| `yaw_tol_deg` | 6.0 | = `HEADING_TOLERANCE_DEG` firmware |
| `slot_step_cm` | 8.0 | spasi tanda silang K-1 |
| `bbox_h_min/max` | 0.35 / 0.90 | pita MENILAI |
| `jejak_h_min/max` | 0.06 / 0.98 | pita MENGEJAR |
| **`jejak_hz`** | **8.0** | batas inferensi/detik saat JEJAK — knob hemat daya |
| `conf_min`, `n_frame`, `k_of_n` | 0.60, 9, 7 | ambang voting |
| **`capit_cm`** | **12.0** | jarak berhenti untuk mencapit — **tebakan, wajib diukur** |
| `capit_tol_cm` | 1.5 | toleransi jarak capit |
| `lengan_r` | 70.0 mm | jangkauan lengan, diukur dari **pusat badan** |
| `lengan_h` | 20.0 mm | tinggi lengan saat menjepit |
| `lengan_angkat` | 25.0 mm | tambahan tinggi saat mengangkat |
| `ambil_hanya_korban` | True | tolak rantai AMBIL kalau kelasnya `dummy` |
| `ambil_maks_cm` | 60.0 | di luar ini dianggap dinding, bukan korban (perlu sejak v1.7) |

Tombol **Simpan kalibrasi** menulis `kalib_<timestamp>.json` — satu-satunya
penulisan berkas di seluruh program. Muat lagi: `./run_hud.sh --calib file.json`.

---

## 8. Sambungan ke Teensy

Firmware memakai huruf besar-kecil sebagai perintah **berbeda**, jadi HUD memakai
**whitelist**.

**Boleh di mode BACA:** `m v l d D k K q h M` — semuanya hanya mencetak.
**Boleh di mode KENDALI:** `o0`–`o3`, `O<derajat>`, `D<cm>`, `w`, `s`, `b`,
`g<0-100>`, `n`, `a<r> <h>`, `m1`–`m3`.

**Selalu ditolak** (kecuali lewat Kirim PAKSA / tombol LEMAS):

| | |
|---|---|
| `S` `W` `e` | menulis EEPROM |
| `C` | kalibrasi pivot — memblokir loop firmware |
| `F` `f` `p` `P` | robot langsung berjalan |
| `B` `z` | demo/pajangan |
| `x` | LEMAS — servo mati, robot ambruk |

`main_deploy.py` yang lama mengirim `'S'`, `'F'`, `'L'`, `'R'` — itu penjelasan
paling mungkin untuk "kalibrasi berubah padahal tidak disentuh". HUD juga
**tidak mengirim apa pun saat keluar**.

### Firmware v1.8 — hasil crosscheck langsung ke sumbernya

Dibanding v1.7, hanya **4 berkas** yang berubah: `Hexapod_Unlimited.ino` (7 baris),
`Mission.cpp/h`, `Navigation.cpp/h`, `config.h`. `Calib.cpp`, `HexaArm.cpp`,
`ArmInverse.cpp`, `LidarArray.cpp` dan sisanya **byte-identik**.

**HUD tetap aman.** `Mission::status()` dan `Navigation::navStatus()` formatnya
tidak berubah, `LidarArray.cpp` identik → parser `RE_KV` dan tabel `l` tetap
cocok. Dua state baru muncul sebagai teks biasa di kolom `teensy`.

**Yang berubah dan berdampak nyata:**

1. **Bawaan kemudi sekarang `N2`, bukan `N0`.** `_wallSudut` kini `true` sejak
   boot. Alasannya diukur: goyang perintah kemudi turun **4,4/detik → 0,16/detik**,
   karena pembaginya dasar 11 cm alih-alih selang sampel 25 ms. Tooltip N0/N2 di
   tab Manual sudah diperbaiki mengikuti ini.
2. **`LIDAR_MIN_CM` sisi 10 → 4 cm.** Ini perbaikan bug tanda yang terbalik:
   di 10–11 cm bacaan dipetakan ke `LIDAR_JAUH`, yang bagi ikut-dinding berarti
   "dinding hilang" → robot **membelok KE ARAH** dinding yang sudah menempel.
   Itu penjelasan "sering serong ke kanan" saat ikut dinding kanan.
3. **`WALL_BIAS_*_CM = 2.0` jadi bawaan.** Hasil `Y0` (2,00 cm di kedua sisi,
   sama dua kali pengukuran) dibakukan, jadi turunan-dari-sudut sudah benar
   sejak menyala tanpa mengetik apa pun. `Y0` tetap menimpanya.
4. **Penjaga "terkurung" (`NAV_DEKAT_BATAS_MS`) DIBUANG.** Diganti
   `MISI_RUAS_BATAS_MS` 90 detik — **hanya untuk ruas misi**. ⚠️ Konsekuensi
   yang menyentuh kita: di mode manual `F`/`P` (lewat Kirim PAKSA), sensor yang
   macet membuat robot menyusuri dinding hantu **tanpa batas waktu**. Vincent
   menulisnya terbuka sebagai harga yang dibayar. Jangan tinggalkan `F`/`P`
   manual tanpa tangan di tombol LEMAS.
5. **Perintah baru `m5 <cm>`** — ambang sensor depan untuk ruas terakhir,
   bawaan 40 cm, wajib > `FRONT_STOP_CM` (dijaga `static_assert`).
6. **Dua state baru:** `MISI_MAJU_AKHIR` (maju di bawah turunan sampai depan ≤
   `m5`) dan `MISI_PIVOT_AKHIR` (pivot 45° dari SELATAN ke BARAT, satu pivot
   langsung, memakai `headingAntara()` dari kompas TERCATAT — bukan asumsi
   keempat mata angin berjarak 90° sempurna di IMU).

### Firmware v1.7 — crosscheck sebelumnya (masih berlaku)

Diperiksa berkas per berkas terhadap v1.61. Yang **tidak berubah sama sekali**:
`LidarArray.cpp` (byte-identik), `HexaArm.cpp`, `ArmInverse.cpp`, `HexaServos`,
`LegInverseKinematics`, `Imu`, `EEMap.h`, `types.h`, `Calib.h`.

**HUD aman di v1.7.** Ini bukan tebakan:

- `Hexapod_Unlimited.ino` hanya **menambah** case; dari seluruh diff cuma dua
  baris teks bantuan yang berubah. Semua perintah HUD (`m l v d`, `o0`–`o3`,
  `O`, `D`, `w`, `s`, `b`, `g`, `a`, `n`, `m1`–`m3`) utuh.
- `LidarArray.cpp` identik → tabel `l` yang di-parse HUD formatnya sama persis.
- `Mission::status()` tetap `  kunci : nilai` dengan indentasi yang sama →
  parser `RE_KV` HUD tetap cocok. Tiga state baru (`PIVOT -- memutar balik ke
  UTARA`, `LANTAI PECAH`, `TURUNAN`) muncul sebagai teks di kolom `teensy`
  tanpa perlu HUD diubah.
- Lengan & capit tidak tersentuh → rantai AMBIL berlaku sama di v1.61 dan v1.7.

**Tiga hal yang HARUS kamu lakukan sesudah flash v1.7:**

1. **`Qwall.setpoint 17`, `Qwall.min 13`, lalu `W`.** `CALIB_VERSION` tetap **9**
   dan urutan 25 parameternya tidak berubah, jadi blob EEPROM lama **tetap
   dipakai** — nilai default baru (19→17 dan 15→13) **tidak akan berlaku
   sendiri**. Ini perubahan hasil ukur: lebar arena sungguhan **45 cm**, bukan
   60 cm yang dipakai sim semula.
2. **Kirim `N1` atau `N3` kalau mau fuzzy.** Bawaannya `_wallSamar = false`,
   artinya menyalakan v1.7 saja **masih PD**, sama seperti v1.61. Mode kemudi
   sengaja disimpan di RAM, bukan di blob Calib, jadi hilang tiap Teensy mati.
3. **Periksa `l` sekali.** `LIDAR_MAX_CM` naik **70 → 130** dan
   `LIDAR_ROI_SEMPIT` kini **1**. Efeknya di sisi kita: dinding sejauh 1 meter
   sekarang terbaca sebagai **angka**, bukan lagi `jauh`.

Karena itu HUD dapat `ambil_maks_cm = 60.0`. Tanpa pagar itu, rantai AMBIL akan
dengan patuh berjalan menyeberangi ruangan 6 cm sekali jalan menuju dinding yang
disangkanya korban — dulu tidak mungkin, karena bacaan sejauh itu selalu jatuh
ke `jauh` dan langsung GAGAL.

**Koreksi soal "fuzzy dengan 3 LiDAR".** Bukan kanan-depan + depan + belakang.
Fuzzy (Sugeno orde-0, 9 aturan) hanya menggantikan **rumus PD** untuk error
dinding di **satu sisi**, dari sepasang sensor sisi yang sama (`KANAN-DPN` +
`KANAN-BLK`, atau `KIRI-DPN` + `KIRI-BLK`). Sensor `DEPAN` tetap pada tugas
lamanya: berhenti/belok di `FRONT_STOP_CM`. Yang benar-benar baru adalah
**sumber turunannya**: `N2`/`N3` menghitung `d(error)/dt` dari **sudut badan**
`atan2(selisih pasangan, WALL_BASE_CM)`, bukan dari selisih waktu.

Tombolnya sudah ada semua di tab **Manual**, kartu "Firmware v1.7 saja".

⚠️ `README.md` milik Vincent **tidak ikut diperbarui** — berkasnya byte-identik
dengan v1.61, jadi jangan cari penjelasan v1.7 di sana; yang di atas ini dibaca
langsung dari sumbernya.

### Firmware yang harus terpasang

`vincent\Hexapod_KRSRI_2026-v1.61\...\Hexapod_Unlimited\Hexapod_Unlimited.ino`

Upload dari Windows: `upload_teensy.bat` (`--cek` untuk kompilasi saja).
Wajib pustaka **"VL53L1X by Pololu"** — `LidarArray.cpp` ada di dalam folder
sketch, jadi tanpa pustaka itu **seluruh** sketch gagal dikompilasi.

⚠️ **Sesudah pad VUSB–VIN dipotong, Teensy tidak hidup dari USB.** Daya robot
(VIN) harus menyala supaya papannya muncul dan bisa di-upload. Aman dengan servo
terpasang: v1.61 boot LEMAS dan `DEMO_BOOT` sudah 0.

Versi ini menaikkan `CALIB_VERSION` 8→9, jadi blob EEPROM lama **dibuang**.
Sesudah flash: `q` (catat), `Q<nama> <nilai>`, `W`, lalu periksa `K`, `k`, dan
`l` (verifikasi arah ch0 & ch2).

---

## 9. Daya — bagian yang paling sering jadi biang

Gejala: Pi **mati mendadak** saat masuk JEJAK, padahal multimeter menunjukkan
5,03 V.

Sebabnya: sebelum JEJAK vision mati dan Pi menarik ~1 A. Begitu JEJAK menyala,
inferensi 640 px berjalan di tiga inti dan arus melonjak ke 4–5 A dalam hitungan
milidetik. **Multimeter merata-ratakan** — lekukan 10 ms tidak akan muncul di
layarnya. Pi 5 mati di sekitar 4,6–4,7 V transien.

### Alat ukur yang benar

Baris **daya** di strip status, dibaca tiap 3 detik dari `vcgencmd get_throttled`:

```
daya  58C                                     hijau  = sehat
daya  62C  under-voltage SEKARANG             merah  = sedang terjadi
```

Bit ini biasanya menyala beberapa detik sebelum Pi mati — jadi kamu bisa
**melihatnya datang**.

### Menurunkan konsumsi

```bash
./hemat_daya.sh --status     # lihat keadaan, tidak mengubah apa pun
./hemat_daya.sh              # terapkan (minta konfirmasi, ada cadangan)
sudo reboot
./hemat_daya.sh --balik      # kembalikan semula
```

Yang diubahnya:

1. **overclock dimatikan** — `arm_freq` / `over_voltage` / `force_turbo`
   dinonaktifkan di `config.txt`. Overclock dipasang demi 60 FPS yang tidak lagi
   dibutuhkan.
2. **`arm_freq_max=1800`** — arus puncak turun jauh lebih cepat daripada
   kecepatannya, karena daya naik kira-kira kuadrat terhadap tegangan inti.
3. **`--threads 2 --width 640 --height 480 --fps 30`** pada layanan HUD — dua
   inti jauh lebih landai dari tiga, dan dekode MJPG jadi setengahnya.
4. **boot ke konsol** (`multi-user.target`) — kompositor Wayland dan wayvnc
   tidak dijalankan.

### Gerak menyamping & mundur: mesinnya SUDAH ADA, cuma tidak dibuka

Ini temuan yang paling berguna dari seluruh pembacaan firmware.

```cpp
void Hexapod::walk(float forward, float strafe, float turn) {
    _gait.setMoveVector(strafe, forward, turn);   // masing-masing -1..1
}
```

Gait-nya **omnidirectional penuh**. Tapi seluruh pemanggilnya:

```
Hexapod_Unlimited.ino:857   robot.walk(NAV_FWD_SPEED, 0.0f, 0.0f)   <- 'w'
Navigation.cpp:27,229,831,878   _robot.walk(0.0f, 0.0f, turn)       <- pivot
Navigation.cpp:1031             _robot.walk(maju,  0.0f, turn)      <- ikut dinding
```

**`strafe` diberi `0.0f` di SETIAP tempat. Tidak pernah dipakai satu kali pun.**

Jadi geser-kanan, geser-kiri dan mundur bukan fitur yang perlu dibuat — tinggal
**dibuka jadi perintah serial**. Satu `case` baru di `.ino` sudah cukup, misal
`W<maju> <geser> <putar>` dengan tiga nilai −1..1.

Tiga akibat yang langsung menyentuh pekerjaan kita:

1. **Menyamping adalah aktuator penengahan yang paling tepat.** Meluruskan capit
   ke korban itu soal simpangan LATERAL, bukan sudut. Menyamping menyelesaikannya
   langsung — tanpa memutar, tanpa deadband 6°, dan tanpa memakan anggaran
   ±20° putar-badan.
2. **Mundur menutup satu-satunya jalan buntu di FSM.** `S_A_MAJU` sekarang harus
   GAGAL kalau kelewat dekat, karena tidak ada cara mundur.
3. **Berputar TANPA IMU.** `walk(0, 0, ±v)` memutar badan hanya dengan mesin
   gait. Gerbang IMU di `pivotRelatif` itu untuk pivot **lingkar tertutup**
   (menuju heading tertentu) — bukan syarat untuk bisa berputar sama sekali.
   Artinya walau IMU mati, robot tetap **bisa** berputar kalau perintahnya ada.

### Capit di firmware v1.11 — apa yang SEBENARNYA terpasang

Vincent bilang capitnya sudah diprogram. Dibaca ke sumbernya, yang terpasang di
robot **belum** menjalankannya:

```
Misi.cpp:660  static bool sekuensAmbil(...) { return millis() - sejak >= 1500; }   <- INI yang dipanggil
Misi.cpp:670  /* ---------------------------------------------------------------
Misi.cpp:692     static bool sekuensAmbil(...)   <- versi LENGKAP, di dalam komentar
Misi.cpp:746  --------------------------------------------------------------- */
```

Sekuens lengkapnya **ada dan sudah ditulis**, tapi masih di dalam blok komentar
baris 670–746 — **tidak dikompilasi**. Yang dipanggil `MISI_LENGAN` tetap stub
kosong: tunggu 1,5 detik, lalu ruas berikutnya. Jadi di robot, capit **belum
bergerak sendiri sama sekali**.

Mungkin Vincent sudah menyiapkannya tapi belum melepas komentarnya, atau sudah
di laptopnya dan belum dikirim. Yang jelas: **v1.11 yang ada di folder kamu
belum**.

**Lengan mana?** Seluruh 10 ruas korban/safe-zone memakai **`ARM_DEPAN`**.
`ARM_BELAKANG` **tidak dipakai satu kali pun** di tabel misi.

**Pemicunya?** `AKS_AMBIL` di ujung ruas → `MISI_LENGAN`. Tidak ada pemicu dari
Pi, dan tidak menunggu Pi.

**Tapi Pi BISA menggerakkan capit langsung** — huruf kecil = depan, besar =
belakang:

| Perintah | Lengan |
|---|---|
| `g0` / `g100` | capit DEPAN tutup/buka |
| `a<r> <h>` | posisi lengan DEPAN |
| `G0` / `G100` | capit BELAKANG |
| `A<r> <h>` | posisi lengan BELAKANG |
| `n` | matikan servo KEDUA lengan |

Semuanya sudah ada tombolnya di kartu **Jejak korban** (tab Korban).

### Tata letak tab dirapikan

Tab Robot dulu menumpuk **9 kartu** — harus scroll jauh. Sekarang lima tab,
masing-masing 2–4 kartu:

| Tab | Isi |
|---|---|
| **Robot** | Kendali robot · Kendali misi · Daya & suhu Pi |
| **Misi** | Jalankan misi · Panjang ruas · Urutan misi |
| **Korban** | Jejak korban · Ambil korban · Uji berurutan · Penilaian slot |
| **Manual** | Kirim perintah langsung · Perintah v1.7+ |
| **Kalibrasi** | Kalibrasi vision · Kompas arena · Knob RAM |

### JEJAK: dummy tidak pernah jadi SASARAN

Bug sebelumnya halus. Gerbang kelas hanya menahan **gerakan** — sasarannya
sendiri tetap dipilih dari deteksi **terbesar apa pun kelasnya**. Jadi kalau
dummy lebih besar dari korban di sebelahnya, bearing terisi dari dummy: panah
merah menunjuk kiri/kanan, robot diam, dan **korban asli diabaikan karena kalah
besar**.

Sekarang kelasnya **disaring dulu, baru dipilih yang terbesar**. Dummy tidak
pernah jadi sasaran sama sekali, di JEJAK maupun CENTER. Kartu Jejak menambah
baris **diabaikan** yang menyebut berapa dummy terlihat tapi dilewati.

### Firmware v1.10 — profil gait, bukan mapping

Dicek berkas per berkas terhadap v1.9, sesudah CR dibuang (v1.9/v1.10 disimpan
dengan line ending Windows, jadi `diff` mentah membesar-besarkan):

| Berkas | Beda nyata |
|---|---|
| `Misi.cpp` | 225 baris |
| `MISI.md` | 209 baris |
| `Navigation.cpp` | 76 baris |
| `config.h` | 65 baris |
| `HexaGait.cpp/h` | 58 baris |
| **`Imu.cpp` / `Imu.h`** | **0 — tidak disentuh lagi** |
| `LidarArray`, `HexaArm`, `ArmInverse`, `Calib`, `HexaServos` | **0** |

Isinya: **profil gait baru `PRF_KAIL`** (`T4`) untuk R-9 — kaki depan mengait ke
depan-atas, belakang naik — plus state `MISI_SETEL` yang menunggu badan tenang
sesudah ganti profil, dan penghitung pivot-ulang. Bukan pekerjaan mapping.

**Tiga hal yang TETAP belum ada di v1.10:**

1. **Pemicu vision.** `AKS_KONFIRM` ada di `enum`, tapi **nol ruas** memakainya.
2. **Capit.** `sekuensAmbil()` masih stub kosong; versi lengkapnya di dalam
   blok komentar.
3. **IMU.** Tidak disentuh sejak v1.8.

### Pemicu vision dari NAMA RUAS — jalan tanpa menunggu Vincent

Firmware belum punya pemicu vision, tapi status `m` **selalu menyebut ruas yang
sedang dijalankan**:

```
  ruas        : 12 dari 0..33  --  K-1 angkat korban
```

Nama itu sudah cukup. HUD mencocokkannya dengan `RUAS_VISION`
(`"ANGKAT KORBAN"`), lalu masuk state **`AWAS_KORBAN`**: kamera menyala, `Juri`
menilai, hasilnya tampil di kartu **Jalankan misi** dan masuk log.

**Yang TIDAK dilakukannya: bergerak.** Nol perintah gerak, antrean tetap kosong.
Firmware sedang menjalankan ruasnya sendiri; menyelanya di tengah jalan akan
merusak lintasan yang sudah diukur. Ini pengamatan, bukan pengambilalihan.

Begitu Vincent memasang `AKS_KONFIRM` di ruas korban, jalur `KONFIRM_FW` yang
sudah ada langsung menjawab `m2`/`m3` — nyalakan `auto_konfirm` di tab Kalibrasi.

### Menjalankan misi dari HUD — sudah bisa sejak dulu

Tombol **"Mulai misi (m1)"** sudah ada di kartu Robot; sekarang ada kartu
**Jalankan misi** sendiri dengan status ruas dan hasil vision.

Tiga syarat, dan firmware **menolak dengan pesan jelas** kalau salah satu belum:

1. servo menyala (`b`)
2. kompas arena lengkap — `Gagal: kompas arena belum lengkap -- 'c0'..'c3' lalu 'e', atau 'E'.`
3. pivot terkalibrasi — `Gagal: pivot belum dikalibrasi -- 'C' lalu 'S' sekali saja.`

⚠️ Syarat 2 dan 3 **butuh IMU hidup**. Selama IMU mati, `m1` ditolak — dan itu
benar, bukan bug.

### Kalibrasi vision — urutannya

Knob-nya sudah lama ada di tab Kalibrasi dan berlaku seketika; yang belum ada
urutannya. Sekarang ada kartu **Kalibrasi vision**:

1. **`cx_offset_px`** — garis tengah. Paling menentukan: salah di sini membuat
   capit selalu meleset ke sisi yang sama, dan PID **tidak akan pernah**
   memperbaikinya.
2. **`hfov_deg`** — mengubah arti semua angka derajat.
3. **`bbox_h_min/max`** — pita tinggi, longgarkan secukupnya saja.
4. **`roi_bawah_frac`** — pita yang ditutupi capit.
5. **`tengah_tol_px`** — mulai 8. Kalau tidak pernah mengendap, **longgarkan**,
   jangan naikkan gain.

### Firmware v1.9 — rombakan besar, dan apa artinya untuk vision

**IMU: v1.9 tidak menyentuhnya sama sekali.** `Imu.cpp` dan `Imu.h`
**byte-identik** dengan v1.8 (selisih 222 baris di `diff` mentah itu murni
CRLF). Jadi v1.9 **tidak menjawab dan tidak memperbaiki** masalah IMU — itu
tetap perkara kabel/daya di `Serial2`. Hal yang sama berlaku untuk
`LidarArray.cpp`: 1440 baris "berubah" ternyata **0 perubahan nyata**.

**Yang benar-benar berubah:** `Mission.cpp/h` → **`Misi.cpp/h`** (tulis ulang),
plus `MISI.md` (dokumentasi Vincent sendiri), `Navigation.cpp` (221 baris),
`HexaGait.cpp` (10 baris), `config.h` (48 baris).

**FSM-nya sekarang berbasis TABEL, bukan state per ruangan.** 34 baris `RUAS[]`,
tiap baris menyatakan: belok relatif, kemudi, profil medan, syarat henti, nilai,
dan aksi di ujung. Pivot **tidak lagi jadi state** — mesinnya memutar badan
sendiri saat arah ruas berikutnya berbeda. State tinggal 8:
`DIAM · JALAN · PIVOT · LENGAN · KONFIRM · UKUR · SELESAI · GAGAL`.

**Geser menyamping SUDAH ADA** — `HNT_SISI`, dipakai di ruas 6, 11, 16, 22.
Tapi baca catatan Vincent: *"sumbu geser tidak bisa diminta dari luar:
perpindahannya terkuantisasi satu langkah gait penuh, jadi 'geser 10 cm'
mustahil."* Jadi menyamping itu **lup tertutup di dalam firmware** menuju jarak
dinding tertentu — **bukan** perintah `geser <mm>` yang bisa dipakai penengahan
capit. Rencana strafe untuk centering **tidak bisa dipakai** seperti dugaan saya
sebelumnya.

#### Serah-terima vision — hook-nya ada, tapi belum bisa dipakai

| Aksi ujung ruas | Yang terjadi | Menunggu Pi? |
|---|---|---|
| `AKS_KONFIRM` | berhenti, cetak "MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)" | **YA** |
| `AKS_AMBIL` / `AKS_TARUH` | `MISI_LENGAN`, jalankan sekuens lengan | **TIDAK** |
| `AKS_TIDAK_ADA` | sambung ke ruas berikutnya tanpa berhenti | tidak |

Dua ganjalan, dan keduanya di sisi firmware:

1. **Tidak ada satu pun ruas yang memakai `AKS_KONFIRM`.** Dicek ke tabelnya:
   nol. Jadi misi v1.9 **tidak pernah berhenti menunggu keputusan** sama sekali.
2. **`AKS_AMBIL` tidak menunggu Pi.** `sekuensAmbil()` yang aktif itu stub
   kosong — `return millis() - sejak >= 1500` — lalu `ruasBerikut()` jalan
   sendiri. Versi lengkapnya ada, tapi **di dalam blok komentar**, menunggu
   capit terpasang.

**Sisi HUD sudah disiapkan** (state `KONFIRM_FW`): begitu teks firmware memuat
"KONFIRMASI", HUD menyalakan kamera, menilai dengan gerbang KETAT, lalu menjawab
`m2`/`m3`. Knob `auto_konfirm` **default MATI** — menjawab otomatis berarti misi
lanjut karena kamera bilang begitu, dan itu harus dinyalakan sadar.

**Yang perlu diminta ke Vincent, kecil dan spesifik:** pasang `AKS_KONFIRM` di
ruas-ruas korban (1, 7, 15, 20, 28), atau buat `AKS_AMBIL` menunggu `m2`/`m3`
seperti `AKS_KONFIRM`. Tanpa itu, tidak ada jendela apa pun untuk vision —
robot lewat begitu saja.

### Kalibrasi kompas arena — sudah ada di firmware

Semua perintahnya **sudah ada**, tidak perlu minta Vincent menambah apa pun.
Kartu **"Kalibrasi kompas arena"** ada di tab Robot.

Kompas arena = **empat sudut yaw IMU** yang dicatat saat robot benar-benar
menghadap tiap dinding. `o0`–`o3`, mode arena, `m1`, dan pivot akhir v1.8
semuanya bergantung padanya.

| Perintah | Fungsi | Butuh IMU? | Mode |
|---|---|---|---|
| `k` | cetak 4 arah yang tercatat | **tidak** | BACA |
| `E` | muat 4 arah dari EEPROM 1792 ke RAM | **tidak** | KENDALI |
| `c0`–`c3` | catat UTARA/TIMUR/SELATAN/BARAT | **YA** | KENDALI |
| `e` | simpan ke EEPROM 1792 | tidak | **Kirim PAKSA** |

**Urutannya:** hadapkan robot ke dinding **dulu** → `c0` … `c3` → `k` untuk
memeriksa keempatnya → baru `e` untuk menyimpan.

⚠️ **`c0`–`c3` mustahil selama IMU mati.** `kompasCatat()` membaca
`_imu.yawDeg()` dan langsung keluar dengan *"Navigation: Tidak ada data sudut
IMU."* Tidak ada jalan memutar: `_headArah[]` **hanya** bisa diisi dari IMU atau
dari EEPROM — firmware tidak punya cara menyetel keempat sudut itu sebagai
angka. (`head.utara` ada di tabel Calib tapi bertanda `P_BELUM_DIPAKAI` — slot
kosong, tidak ada kode yang membacanya.)

Jadi selama IMU mati, yang masih bisa dilakukan cuma **melihat** (`k`) dan
**memuat ulang** (`E`) hasil kalibrasi Vincent yang sudah tersimpan.

**`e` sengaja tetap di daftar terlarang.** Dia menulis EEPROM, dan tulisan
EEPROM yang tidak disengaja itulah dugaan terkuat penyebab "kalibrasi berubah
padahal tidak disentuh" dulu. Tombolnya memakai Kirim PAKSA + konfirmasi, dan
tercatat `[PAKSA]` di log.

**Keempat arah TIDAK harus berjarak 90°.** Firmware sengaja memakai sudut yang
**tercatat**, bukan mengasumsikan keempatnya tegak lurus sempurna di IMU —
`headingAntara()` di v1.8 menulis alasannya terang-terangan. Jadi catat apa
adanya, jangan "dirapikan" jadi 0/90/180/270.

### MAJU bisa, BELOK tidak → hampir pasti IMU

Gejala ini punya satu sebab yang dominan, dan bukan serial. Buktinya justru di
gejalanya sendiri: `w` (maju) dan `T0`–`T3` (profil medan) **jalan**, berarti
perintah sampai dan dieksekusi.

Bedanya ada di firmware:

| Perintah | Jalur | Butuh IMU? |
|---|---|---|
| `w` maju | `robot.walk()` langsung | **tidak** |
| `T0`–`T3` profil | `robot.profileXxx()` | **tidak** |
| `O<der>` pivot | `nav.pivotRelatif()` | **YA** |
| `o0`–`o3`, `m1`, mode arena | `nav.pivotKe()` | **YA** |

Dan ini bagian yang membuatnya sulit dilacak:

```cpp
void Navigation::pivotRelatif(float der) {
    if (!_imu.hasData()) return;      // DIAM. Tidak mencetak apa pun.
```

`O` **gagal tanpa satu pesan pun**. Sementara `pivotKe` (dipakai `o0`–`o3`)
mencetak `Gagal: Tidak ada data IMU.` Jadi:

**Uji pembeda, urut:**

1. **Tombol "Aliran yaw (y)"** di tab Manual. Aman, hanya mencetak.
   Tidak ada baris yaw sama sekali → **IMU mati**, dan itu jawabannya.
2. **Tombol "o0"** (robot akan berputar kalau IMU hidup). Kalau muncul
   `Gagal: Tidak ada data IMU.` → terkonfirmasi.
3. Kalau yaw mengalir normal tapi `O` tetap diam, barulah lihat
   `Peringatan: pivot belum dikalibrasi` → jalankan `C` lalu `S`.

**IMU-nya di mana:** Yahboom 10-axis, protokol WIT, di **`Serial2` @ 230400
baud** — UART, bukan I2C. Di Teensy 4.1 itu **pin 7 (RX2) dan 8 (TX2)**.

`_have` di `Imu.cpp` diset `true` sekali dan **tidak pernah dikembalikan ke
false**. Jadi kalau `hasData()` bernilai false, IMU belum pernah mengirim satu
frame sah pun **sejak Teensy menyala** — bukan putus di tengah jalan.

⚠️ **Curigai pengerjaan kelistrikan terakhir.** Rel daya Teensy baru saja
diutak-atik (pad VUSB–VIN dikikir, rencana menyalakan Teensy dan Pi bersamaan).
Kalau ada kabel Pi yang menyentuh pin 7/8, atau IMU kehilangan catu dayanya,
gejalanya persis ini. Periksa juga apakah ada yang mencoba UART Pi↔Teensy di
pin yang sama — itu **bentrok langsung** dengan IMU.

Selama IMU mati, **penengahan tingkat KASAR tidak bisa diuji** (butuh `O`).
Tingkat HALUS memakai `r0 0 <yaw>` yang **tidak butuh IMU**, jadi itu masih bisa
dicoba sendiri lewat kotak teks Manual.

### "Gagal: masih ada ruas yang panjangnya BELUM DIUKUR"

**Bukan bug, dan bukan misi yang belum selesai.** Itu penjaga `tabelSiap()` di
`Misi.cpp` — panjang tiap ruas harus diukur di arena sungguhan, dan yang belum
diisi ditandai `-1`. Firmware **menolak berangkat daripada berjalan menebak**.

Di **v1.11** yang kosong tinggal **satu**:

```
/*30*/  "R-11 longsor (lebar jalan 30)"   HNT_ODO   nilai = -1
```

32 ruas lain sudah terisi. Satu angka itu yang menahan seluruh misi.

**Isi lewat kartu "Panjang ruas" di tab Robot:** ketik ruas `30` + cm, tekan
**Setel panjang (m7)**. Kalau belum tahu angkanya, tekan **Mode ukur (m6)** —
robot berjalan di ruas itu tanpa syarat henti sambil mencetak jarak tempuh;
kamu yang menghentikan dengan STOP di ujung ruas, lalu masukkan angkanya.

⚠️ `m7` berlaku di **RAM firmware**, tidak menulis EEPROM — hilang kalau Teensy
mati. **Catat angkanya**, dan minta Vincent menaruhnya di tabel `Misi.cpp`
supaya permanen.

`tabelSiap()` juga menolak karena empat sebab lain, tapi semuanya **salah tulis
tabel**, bukan salah setel: ruas buta ke depan tanpa batas odometri, ambang
sensor depan di bawah `FRONT_STOP_CM`, ruas geser dengan kemudi MENENGAH, dan
pemicu sensor belakang di luar jangkauan. Kalau salah satu muncul, itu untuk
Vincent, bukan untukmu.

### BUG: kalibrasi tidak bisa diubah

Kotak kalibrasi ditimpa ulang lewat `innerHTML` **tiap 300 ms** bersama seluruh
`tarik()`. Elemen `<input>` yang sedang diketik **dihapus sebelum `onchange`
sempat jalan**, jadi nilainya tidak pernah terkirim — dan dari luar terlihat
seperti kalibrasinya memang tidak bisa diubah.

Sekarang barisnya dibangun **sekali**, lalu tiap poll cuma memperbarui nilai —
dan itu pun **dilewati kalau kotaknya sedang dipegang** (`document.activeElement`).

Bug kedua di sisi Python: field bertipe **teks** selalu ditolak, karena semua
nilai dipaksa lewat `float()`. Jadi `metode_tengah` tidak akan pernah bisa
diubah dari halaman. Sekarang teks ditangani terpisah, dan yang punya daftar
pilihan tampil sebagai **dropdown** — salah ketik satu huruf di situ diam-diam
mengubah metode kendali robot.

### Penengahan: UKUR-SAAT-DIAM (bawaan sekarang)

Riwayat jujurnya: **P → PID → sekali-tembak → ukur-saat-diam.** Sekali-tembak
menang telak di simulasi dan **kalah di robot nyata** — simulasi saya
mengasumsikan bacaan kamera selalu menggambarkan pose sekarang, dan itu tidak
benar.

> **"Late information is false information."**

Itu diagnosis yang tepat, dan mengubah bentuk solusinya. Frame yang lahir
sebelum gerakan terakhir selesai **bukan data berisik — itu data SALAH**. Dia
menggambarkan pose yang sudah tidak ada lagi. Merata-ratakannya dengan bacaan
yang benar justru **mencemari** hasilnya; median maupun rata-rata sama saja.

Yang benar: **buang**, jangan saring.

| Langkah | Apa yang terjadi |
|---|---|
| 1 | Sesudah badan diperintah bergerak, **semua** bacaan ditolak sampai badan diam (`t_boleh_ukur`) |
| 2 | Bacaan yang lolos di-EMA (`ema_alpha = 0.45`) |
| 3 | **Belum cukup sampel → belum bergerak** (`diam_sampel_min = 3`) |
| 4 | Langkah = galat × `langkah_gain`, dijepit `langkah_min_deg` .. `maks_langkah_deg` |

Langkah 3 itu yang paling menolong: diam sebentar jauh lebih murah daripada
bergerak ke arah yang salah lalu harus dikoreksi balik.

Langkah sebanding jaraknya, persis seperti yang diminta — jauh = kasar,
dekat = halus:

```
galat 7,0 der  ->  langkah 2,00 der
galat 1,5 der  ->  langkah 0,83 der
```

Batas bawah `langkah_min_deg` ada supaya koreksi kecil tidak jadi nol lalu
menggantung selamanya di ambang toleransi.

Hasil uji dengan derau ±0,3°: **4 gerakan, nol bolak-balik**, galat akhir
0,56°.

⚠️ **Ketelitian dibatasi derau detektor, bukan kendalinya.** Dengan goyangan
bbox ±0,3° kamu tidak akan pernah mengendap lebih rapat dari itu. Jadi jangan
setel `tengah_tol_px` lebih ketat daripada goyangan bbox-nya sendiri — yang
terjadi cuma robot bergerak selamanya mengejar derau.

Metode lain tetap bisa dipilih dari dropdown: `sekali`, `pid`, `p`.

### MODE FOKUS — matikan video, biar Pi fokus melihat

Tombol di kartu Kalibrasi vision. Saat menyala: **berhenti mengirim video dan
berhenti menggambar kotak di atas frame**. Encode JPEG 1280×720 memakan inti
yang seharusnya untuk inferensi.

Angka simpangan, log, dan seluruh kendali **tetap jalan** — yang berhenti cuma
gambarnya. Justru saat menyetel penengahan, gambar yang paling tidak dibutuhkan:
angka simpangan lebih berguna daripada melihat kotaknya bergerak.

### SEKALI TEMBAK — dicoba, kalah di robot nyata

Gejala PID: berhasil tengah, tapi **~15 detik**, dengan koreksi kelebihan kanan
lalu kiri lalu kanan lagi.

**Sebabnya dua, dan keduanya salah desain saya:**

1. **Umpan balik dipakai untuk hubungan yang SUDAH DIKETAHUI.** Kamera menempel
   di badan. Memutar badan θ mengubah bearing persis −θ. Itu **geometri**, bukan
   misteri yang perlu diraba selangkah demi selangkah. PID meraba-raba sesuatu
   yang sebenarnya bisa dihitung sekali.
2. **BUG: penyaring median dibawa melintasi gerakan.** Median 5 sampel menahan
   bacaan dari pose **lama**. Sesudah badan bergerak, kita masih mengoreksi
   berdasar posisi sebelum bergerak → kelewatan → balik lagi. Persis
   "kelebihan kanan lalu kiri". Sekarang penyaring **dikosongkan** tiap kali
   badan digerakkan.

**Metode bawaan sekarang `metode_tengah = "sekali"`:** ukur → koreksi
**seluruh** galat sekaligus → ukur lagi untuk memastikan.

Hasil uji simulasi, galat awal 8°:

| Metode | Gerakan sampai tengah |
|---|---|
| **sekali** | **1** |
| pid | 8 |

**Faktor skala DIPELAJARI sendiri.** Idealnya badan berputar θ → bearing berubah
−θ, tapi kamera **tidak duduk di sumbu putar**: memutar badan ikut menggeser
kamera ke samping, dan di jarak 20 cm pergeseran itu terbaca sebagai sudut
tambahan. Jadi `skala_yaw` dikoreksi dari respons nyata:

```
nyata = Δbearing / Δyaw          (idealnya +1)
skala_baru = skala*0.6 + (1/nyata)*0.4    dijepit 0,4 .. 2,0
```

Dengan respons berlebih 1,35 (parallax nyata), sekali-tembak tetap selesai dalam
**3 gerakan** dan bolak-balik ≤ 1 kali. Matikan dengan `skala_belajar = False`
kalau mau angka tetap.

**PID tetap ada** — `metode_tengah = "pid"` — untuk membandingkan, dan `"p"`
untuk proporsional polos. Semua knob di tab Kalibrasi, berlaku seketika.

### PID penengahan halus — dan kenapa PID bukan obat utamanya

Gejala: putar-badan jauh lebih akurat daripada pivot kaki, tapi gerakannya
menyentak dan tidak pernah benar-benar mengendap.

**Angka yang menjelaskannya:** `BODY_SLEW_DEG_S = 60.0` der/detik. Koreksi 5°
selesai dalam **83 milidetik**. Kamera kita mengukur jauh lebih lambat dari itu.
Jadi lup ini punya **waktu mati**: kita mengukur, mengirim, badan sudah selesai
bergerak, lalu keputusan berikutnya dibuat dari frame yang sudah basi.
Lup posisi + waktu mati + gain 0,8 = persis "drifting, tidak pernah tengah".

**Tiga hal yang menyelesaikannya, dan tidak satu pun bernama I atau D:**

1. **Batas langkah** (`maks_langkah_deg = 2.0`). Laju ramp 60°/s milik firmware
   tidak bisa diubah dari sini, tapi kita bisa tidak pernah memintanya melompat
   jauh. Ini rem yang sebenarnya.
2. **Tunggu sampai diam** sebelum mengukur lagi. Waktu tunggunya **dihitung**:
   `|perubahan| / 60 der/detik + diam_margin_s`, bukan angka mati.
3. **Median 5 sampel** (`saring_n`). Satu frame nyasar tidak menggeser median;
   rata-rata akan tergeser.

**PID-nya tetap ada, dengan catatan jujur per suku:**

| Suku | Nilai | Gunanya DI SINI |
|---|---|---|
| **Kp** | 0.35 | inti. Turun dari 0,8 karena ada waktu mati |
| **Ki** | 0.05 | **satu** guna: menghapus galat tetap dari pemasangan kamera (`cx_offset_px` belum pas, kamera tidak persis di sumbu putar). Di lup posisi dia **tidak** menambah ketepatan |
| **Kd** | 0.10 | meredam, dihitung dari galat yang **sudah disaring** dan sengaja kecil — turunan dari sensor lambat & berisik itu berbahaya |

Dua penjaga yang wajib ada:

- **Anti-windup:** integral hanya menumpuk **di dekat sasaran** dan dijepit.
  Tanpa itu dia menumpuk selama gerakan besar lalu dilepas sekaligus jadi
  sentakan — persis gejala yang mau dihilangkan.
- **dt liar diabaikan** (`0,02 < dt < 2,0`). HUD yang baru bangun atau frame
  tersendat akan membuat turunan meledak.

**Toleransi sekarang dalam PIKSEL** (`tengah_tol_px = 8`), satuan yang
benar-benar kamu lihat: 8 px di 1280 px / 70,4° = **0,44°** — jauh lebih ketat
dari 1,5° sebelumnya. Kalau ternyata servo tidak sanggup sehalus itu, naikkan.

Semua knob ada di tab Kalibrasi, berlaku seketika. `pid_aktif = False`
mengembalikan ke P sederhana kalau perlu membandingkan.

### Mendekat sesudah tengah — ternyata tidak perlu

Kekhawatiran "capit turun lalu maju, vision tertutup separuh layar" **tidak
berlaku**, dan alasannya geometris:

```
LENGAN_SIAP_MM  = 120 mm      dari PUSAT BADAN
LENGAN_AMBIL_MM = 150 mm      dari PUSAT BADAN
dudukan LiDAR depan ~50 mm di depan pusat badan
```

**Lengan menjulur 120 → 150 mm untuk mencapit.** Tiga sentimeter terakhir
dikerjakan **lengan**, bukan robot yang berjalan. Jadi urutannya:

1. tengahkan (vision) →
2. berhenti di **LiDAR depan ≈ 10 cm** (= korban 150 mm dari pusat badan) →
3. turunkan lengan, julurkan, jepit — **tanpa berjalan sama sekali**

Tidak ada mendekat sambil capit turun, jadi tidak ada masalah okulasi. `capit_cm`
sudah diubah 12 → **10 cm** mengikuti geometri itu, bukan tebakan lagi.

⚠️ 10 cm jatuh dekat pita hantu LiDAR depan yang Vincent dokumentasikan (3,2 dan
9 cm). Periksa `l` saat robot berhenti di situ.

**Kalau ternyata tetap harus berjalan dengan lengan turun**, ada `roi_bawah_frac`:
buang deteksi yang pusatnya di pita bawah yang ditutupi capit. Default 0.
Ukur sekali — turunkan lengan, lihat stream, catat di ketinggian berapa capit
mulai terlihat. Ini penting bukan cuma karena tertutup: **capit oranye di depan
lensa itu justru mirip korban.**

**Dan sesudah berhenti, berhenti juga melacak.** Korban itu benda mati; dia tidak
akan bergerak. Vision sudah selesai tugasnya begitu robot berhenti di posisi.
Yang menjaga selama lengan menjulur cukup LiDAR depan: kalau bacaannya melompat
ke jarak dinding, korban bergeser atau jatuh — batalkan.

### Penengahan: kenapa pivot kaki SELAMANYA tidak cukup

Gejalanya: robot tidak pernah benar-benar tengah — kadang berlebihan, kadang
"bergerak tapi tidak jadi". Sebabnya **struktural, bukan tuning**:

`HEADING_TOLERANCE_DEG = 6.0` di firmware. Pivot menganggap dirinya sudah lurus
dalam ±6°, jadi **permintaan di bawah 6° dibuang tanpa satu pun pesan**. Itu
persis "berbuat gerakan namun karena sudah di tengah, gerakan tersebut tidak
jadi".

Di jarak 20 cm, 6° = **2,1 cm**. Kalau capit butuh ±1 cm, pivot gait **tidak
akan pernah** cukup — tidak ada nilai P, I atau D yang memperbaikinya. Menambah
integrator justru memperburuk: dengan deadband, integral menumpuk lalu dilepas
sekaligus jadi sentakan besar.

**Jadi dipakai dua aktuator, bukan satu:**

| Tingkat | Perintah | Jangkauan | Resolusi | Melangkah? |
|---|---|---|---|---|
| **KASAR** (>8°) | `O<der>` pivot gait | bebas | **6°** (deadband) | ya |
| **HALUS** (≤8°) | `r0 0 <yaw>` putar badan | ±20° | pecahan derajat | **tidak** |

Putar-badan itu **pose offset, bukan mode jalan** — tidak ada gait, tidak ada
deadband. Itu yang membuat ±1,5° mungkin. Tersedia juga `t<x> 0 0` (geser badan
±40 mm) untuk trim lateral milimeter.

**Tiga perbaikan lain di jalur yang sama:**

1. **Gain pivot 0,6, bukan 1,0.** Ada waktu mati ~2,5 detik antara mengukur dan
   selesai bergerak. Meminta seluruh galat dijamin melewati sasaran — itu
   penyebab "gerakan berlebihan".
2. **Permintaan dilantai ke > 6° dan dibulatkan KE ATAS.** `f"{6.5:.0f}"`
   menghasilkan `"6"` — tepat di deadband, dibuang. Bug ini ditemukan oleh uji,
   bukan di arena.
3. **Halus dikerjakan SESUDAH maju, bukan sebelum.** Berjalan menghapus
   penengahan sehalus apa pun, jadi urutannya sekarang:
   `TENGAH (kasar) → MAJU → HALUS (presisi) → SIAP → JEPIT → ANGKAT`.

**Badan wajib netral sebelum berjalan.** Badan yang ditinggal menyerong 15° lalu
disuruh `w` akan berjalan miring — gait mengarah ke depan **kaki**, kamera dan
capit ke depan **badan**, dan keduanya tidak sama lagi. STOP, Abort dan pindah
ke MAJU semuanya mengirim `r0 0 0` + `t0 0 0`. Kartu AMBIL menampilkan pose
badan yang sedang aktif.

### Dummy tidak pernah dikejar

`jejak_hanya_korban` (default **True**) memasang gerbang kelas yang sama di
**JEJAK, CENTER dan AMBIL**. Sebelumnya hanya rantai AMBIL yang menolak dummy —
jadi robot tetap berjalan dan menengahkan diri ke dummy lebih dulu, lalu baru
menolak di detik terakhir, sesudah seluruh ongkos geraknya terbayar.

Kelas yang **belum diketahui** juga ditolak: robot diam sampai ada keputusan,
bukan bergerak sambil menebak.

### State yang BELUM DIPROGRAM

Lima state ini sengaja diam — `langkah_fsm()` langsung `return`:

`DEKATI` → `CENGKERAM` → `VERIF` → `ANTAR_SZ` → `LETAKKAN`

Itu rantai **lama**, dari sebelum rantai `AMBIL` dibuat. Rantai AMBIL (`AMBIL_TENGAH`
… `AMBIL_ANGKAT`) sudah benar-benar jalan, tapi **jalur otomatis dari `PUTUS`
masih bermuara ke rantai lama**, bukan ke AMBIL.

Yang berubah: dulu diamnya **tidak kelihatan** — robot berhenti, tidak ada pesan,
operator menunggu sesuatu yang memang tidak akan pernah terjadi. Sekarang:

- kolom **state** jadi **kuning** dan diberi label `(BELUM DIPROGRAM)`;
- menekan **State berikutnya** ke sana menulis di log bahwa tujuannya belum
  diprogram dan menyarankan tombol yang benar;
- ada tombol **Lewati misi ini**.

**Beda dua tombol itu:**

| Tombol | Cakupan |
|---|---|
| **State berikutnya** | satu **sub-state** di dalam misi yang sama |
| **Lewati misi ini** | **seluruh misi** ditandai LEWAT, lanjut ke baris berikutnya di tabel `MISI` |

Di rantai yang belum diprogram, "State berikutnya" cuma memindahkan dari satu
state diam ke state diam berikutnya — lima kali, lalu mentok. "Lewati misi ini"
yang jalan keluarnya.

**Pilihan yang belum diambil:** menyambung `PUTUS` langsung ke `AMBIL_TENGAH`
supaya jalur otomatis benar-benar bisa menyelesaikan penyelamatan. Belum
dilakukan dengan sengaja — `capit_cm` masih tebakan yang belum diukur, dan
menjalankan rantai AMBIL otomatis di tengah misi dengan angka yang belum
terverifikasi berisiko menabrak korban. Sambungkan sesudah `capit_cm` diukur.

### STOP, PAUSE, RESUME, ABORT

Tiga tingkat penghentian, sengaja dibedakan:

| Tombol | Yang dikirim | FSM | State | Keluar dari situ |
|---|---|---|---|---|
| **PAUSE** | `s` | beku | **disimpan** | Resume |
| **STOP semua** | `s` | **dikunci** | → IDLE | Lepas STOP |
| **Abort mission** | `m0` + `s` | **dikunci** | → IDLE, misi ditandai GAGAL | Lepas STOP |
| **LEMAS** | `x` | **dikunci** | → GAGAL | Lepas STOP |

**Bug yang diperbaiki:** dulu `stop` cuma mengirim `s` dan mengosongkan antrean.
Frame berikutnya FSM melihat antrean kosong, menjadwalkan perintah **baru**, dan
robot lanjut sendiri — jadi berhenti sekejap lalu jalan lagi selama korban masih
terlihat. Sekarang STOP memasang **kunci** (`misi.halt`) yang:

1. menghentikan `langkah_fsm` **di baris paling atas**, sebelum pemeriksaan batas
   waktu — kalau di bawahnya, robot yang di-STOP masih bisa "gagal karena
   timeout" dan pindah state sendiri;
2. menahan `Aksi.putar`, jadi perintah yang **sudah** terlanjur masuk antrean pun
   tidak jalan;
3. **menolak tombol gerak dari halaman** selama terkunci. Tanpa ini, "stop semua
   aktivitas" cuma berlaku untuk FSM sementara tombol manual tetap bisa
   menjalankan robot — itu bukan stop.

Yang tetap boleh saat terkunci: Lepas STOP, LEMAS, Abort, Pause, ganti mode,
kalibrasi, Restart. Semuanya tidak menggerakkan robot.

**`m0` pada Abort itu wajib.** Tanpa itu `Mission` di firmware masih hidup dan
`navUpdate()` menyalakan navigasi lagi sendiri — HUD berhenti, robot tidak.
Firmware sendiri melakukannya begitu di `case 's'` dan `case 'x'`.

**Resume me-nolkan jam state.** Misi yang dijeda dua menit akan langsung kena
batas waktu begitu dilanjutkan, dan itu terbaca sebagai gagal palsu.

### LEMAS: kalau servo tetap kaku

Sudah diperiksa langsung ke sumber v1.8 — sisi firmware **benar**:

```
case 'x':  misi.batal() → nav.navBerhenti() → nav.remJarakLepas() → robot.disarm()
HexaServos::allOff():  setPWM(ch, 0, 0) untuk 32 channel = output benar-benar MATI
```

Tidak ada satu pun perintah yang menyalakan servo sendiri; hanya `b` yang
memanggil `arm()`, dan `a`/`g` menolak dengan *"Servo masih lemas — ketik 'b'
dulu."* kalau belum di-arm.

Jadi kalau servo masih kaku sesudah LEMAS, dua kemungkinan, dan **log serial yang
memisahkannya**:

1. **Baris `Servo NONAKTIF (PWM mati, servo bebas).` TIDAK muncul** → perintahnya
   tidak sampai. Periksa penghitung `rx` di strip status.
2. **Baris itu MUNCUL tapi servo tetap kaku** → ini perilaku servo, bukan bug.
   Servo **digital** umumnya **menahan posisi terakhir** saat sinyal PWM hilang;
   yang jadi lemas tanpa sinyal itu servo analog. Tidak ada kode yang bisa
   memperbaikinya — sinyalnya memang sudah mati.
   Obatnya perangkat keras: **memutus catu daya rel servo** lewat relay/MOSFET
   yang dikendalikan Teensy. Itu pekerjaan tim elektrik, dan sekalian jadi tombol
   darurat yang benar.

### Kartu "Daya & suhu Pi" (tab Robot)

Tujuh baris, semuanya dari Pi sendiri — tidak ada sensor tambahan:

| Baris | Sumber | Catatan |
|---|---|---|
| **status** | gabungan | NORMAL / UNDER-VOLTAGE / OVER-CURRENT USB / DI-THROTTLE / FREKUENSI DIBATASI / BATAS SUHU / PANAS |
| **tegangan 5V** | `vcgencmd pmic_read_adc` → `EXT5V` | plus **terendah** sejak HUD hidup |
| **konsumsi** | Σ (V × I) tiap rel PMIC | plus **puncak**; ini konsumsi papan Pi, bukan seluruh robot |
| **arus total** | Σ I rel PMIC | |
| **suhu** | `thermal_zone0` | plus tertinggi |
| **over-current** | log kernel (`journalctl -k`, fallback `dmesg`) | Pi **tidak punya** bit status untuk ini |
| **bit throttle** | `vcgencmd get_throttled` | termasuk bit "pernah terjadi" yang menempel sampai reboot |

Plus grafik ~3 menit terakhir: tegangan sebagai garis (sumbu **dipaku 4,6–5,3 V**,
bukan auto-skala — auto-skala membuat riak 10 mV terlihat seperti tebing), garis
putus-putus di 4,8 V sebagai ambang, dan konsumsi sebagai isian di belakangnya.

⚠️ **Jangan terlalu percaya angka voltnya.** ADC PMIC dibaca beberapa kali per
detik. Brownout yang menjatuhkan Pi berlangsung **10 milidetik** dan tidak akan
pernah muncul di situ — persis keterbatasan yang sama dengan multimeter yang
merata-rata. Yang benar-benar menangkapnya cuma **bit throttle**: perangkat
kerasnya sendiri yang melatch, dan `0x10000` menempel sampai reboot. Kalau volt
terlihat adem tapi "pernah under-voltage" menyala, **percaya bitnya**.

Yang **tidak** bisa ditampilkan, dan alasannya:

- **Undercurrent** — Pi tidak punya status ini dan tidak mungkin punya. Catu
  yang tidak sanggup memberi arus tidak muncul sebagai "arus kurang"; dia
  muncul sebagai **tegangan jatuh**, yaitu baris under-voltage.
- **Over-current** hanya terbaca dari log kernel, bukan status. Kalau user Pi
  tidak bisa membaca log, kolomnya bilang begitu — bukan menampilkan `0` yang
  menyesatkan. Perbaikannya: `sudo usermod -aG adm bima` lalu reboot.
- **Konsumsi Teensy dan servo** tidak ada di sini sama sekali. PMIC hanya
  melihat rel Pi. Untuk arus robot butuh sensor terpisah (INA219/INA226 di
  jalur baterai), dan itu bacaannya masuk lewat Teensy, bukan lewat Pi.
- Di **Pi 4 dan sebelumnya** `pmic_read_adc` tidak ada — kolom tegangan dan
  konsumsi kosong dengan sebabnya ditulis, bukan diam.

### STATUS SEKARANG: mode hemat DIMATIKAN

Atas keputusan tim, Pi kembali ke **overclock penuh** — kemampuan komputasi
dinilai lebih esensial daripada margin daya, dan tim elektrik sedang menggarap
catu dayanya sendiri (menyalakan Teensy dan Pi bersamaan dari baterai).

Yang dikembalikan:

```bash
cd ~/M && ./hemat_daya.sh --balik && sudo reboot
```

`--balik` memulihkan `config.txt` dari cadangan, mengembalikan boot ke
`graphical.target`, dan membuang `--threads 2 --width 640 --height 480 --fps 30`
dari layanan. Kalau cadangannya hilang, dia juga membuka lagi baris yang
sempat dikomentari `#hemat#` — jadi tidak ada yang tertinggal setengah jalan.

**`jejak_hz` juga dikembalikan ke `0` (tanpa batas)** di `mission_hud.py`.
Knob itu bagian dari paket hemat daya, bukan bagian dari `hemat_daya.sh`, jadi
`--balik` tidak menyentuhnya.

⚠️ Ini mengembalikan ambang brownout ke posisi semula. Yang **tidak** ikut
dikembalikan, karena tidak mengurangi kemampuan komputasi sama sekali: batas
JPEG ~15 Hz dan hanya saat ada yang menonton, plus indikator `daya` di strip
status. Keduanya tetap menyala.

**Kalau Pi mati lagi di JEJAK:** knob tercepat adalah tab **Kalibrasi →
`jejak_hz` → isi `8`**. Berlaku seketika, tanpa restart, tanpa SSH. Itu yang
dulu menghentikan Pi mati tiap kali masuk mode JEJAK — inferensi 640 px memakai
3 inti penuh, dan lonjakan arusnya yang menjatuhkan tegangan. Multimeter tidak
bisa melihatnya karena dia merata-rata; strip `daya` di HUD bisa, karena dia
membaca bit `throttled` milik Pi sendiri.

### Yang benar-benar menyelesaikannya

Penghematan di atas **menggeser ambang, tidak menghapusnya**. Obat sebenarnya:

- **Kapasitor bulk 2200–4700 µF low-ESR** langsung di terminal 5 V Pi (bukan di
  BEC). Dia menyuplai lonjakan milidetik secara lokal. Komponen paling murah,
  efeknya paling besar untuk masalah ini.
- BEC ≥5 A kontinu, kabel tebal dan pendek.
- Rel servo dan rel Pi terpisah, **hanya ground yang disatukan**.
- **Jangan** pakai `usb_max_current_enable=1` sebagai jalan pintas.

### Kipas

```
dtparam=fan_temp0=45000,fan_temp0_hyst=5000,fan_temp0_speed=100
dtparam=fan_temp1=50000,fan_temp1_hyst=5000,fan_temp1_speed=150
dtparam=fan_temp2=55000,fan_temp2_hyst=5000,fan_temp2_speed=200
dtparam=fan_temp3=60000,fan_temp3_hyst=5000,fan_temp3_speed=255
```

Suhu mili-derajat, kecepatan 0–255. Tapi kipas juga menarik arus dari rel yang
sama — kerjakan setelah dayanya beres.

---

## 10. Kalau bermasalah

| Gejala | Sebab | Tindakan |
|---|---|---|
| **Semua field `-`, video tetap hidup** | JavaScript halaman mati (SyntaxError) — video `<img>` tidak butuh JS | buka konsol browser (F12); jalankan `test_mission_hud.py` (uji 18 memarse JS dengan `node --check`) |
| `serial rx 0 B` sementara `tx` naik | ada yang memakan balasan Teensy | `systemctl is-active ModemManager brltty` → matikan; `sudo fuser -v /dev/ttyACM0` |
| `serial` merah + sebabnya tertulis | ikuti sebab yang tertulis | `cek_serial.py` |
| `LOOP BEKU n detik` | loop utama tersendat/berhenti | `journalctl -u r2c-hud -n 50` — jejak galat ikut tercetak |
| Pi mati saat JEJAK | brownout transien | §9 |
| `Permission denied` saat `./run_hud.sh` | `scp` tidak membawa bit executable | `chmod +x run_hud.sh` |
| Port 5000 dipakai | aplikasi lain | `sudo ss -ltnp "sport = :5000"` atau `--web-port 5001` |
| Teensy tidak muncul di `lsusb` | VIN mati / kabel di laptop | nyalakan daya robot; pindahkan kabel ke Pi |
| `W:onnxruntime … /sys/class/drm/card0` | onnxruntime mencari GPU | abaikan, wajar di Pi |

Perintah harian:

```bash
journalctl -u r2c-hud -f       # log HUD yang sedang jalan
sudo systemctl restart r2c-hud # sesudah upload versi baru
sudo systemctl stop r2c-hud    # sebelum menjalankan manual / cek_serial.py
```

Port web, kamera, dan port serial **tidak bisa dipakai dua proses sekaligus**.

### Pelajaran yang sudah mahal

- **Halaman kosong tidak memberi tahu apa-apa.** Sekarang HUD menerbitkan state
  sebelum loop dimulai, membungkus tiap iterasi dengan penangkap galat, dan
  menampilkan umur data. Kalau kosong lagi, dia sendiri yang menyebutkan sebabnya.
- **`[TX] m` hanya membuktikan kita mengirim.** Penghitung `rx` yang memisahkan
  "tidak terkirim" dari "tidak dibalas".
- **`\n` di dalam string Python biasa jadi baris baru sungguhan** dan memutus
  string JavaScript. `HALAMAN` sekarang raw string (`r"""`), dan uji 18 memarse
  JS-nya dengan `node --check` tiap kali uji dijalankan.

---

## 11. Yang masih ditunggu

**Dari Vincent (firmware):**

1. `Serial1` untuk Pi — blocker kalau jadi pakai UART.
2. Verifikasi arah ch0 & ch2 dengan `l`. Mode `f`/`F` mengemudi dari ch0.
3. Ukur slip odometri (`D80` vs meteran) → tulis `Ds` ke `config.h`.
4. Perintah **geser samping** (hemat ~20 detik per ruang) dan **mundur**
   (menutup satu-satunya jalan buntu di FSM).
5. `HEADING_TOLERANCE_DEG` jadi parameter `Q` — sekarang centering mentok di 6°
   (= ±2,1 cm di 20 cm).
6. Pemicu korban lewat odometri ruas, menggantikan pemicu ch2 yang hanya sah
   untuk K-1.
7. Tombol RUN di GPIO lewat `handleCmd()`, non-toggle, abaikan tekan setelah
   robot bergerak.
8. Perbaikan CRLF: `\n` yang datang tepat sesudah `\r` diabaikan, supaya
   mengetik dari Windows tidak memicu rem darurat tiap Enter.

**Dari sisi vision:**

- **Ukur ulang batas jarak.** Seluruh arsitektur slot dibangun di atas asumsi
  "akurat hanya di 20 cm". Kalau ternyata masih benar membedakan korban/dummy di
  40 cm, seluruh ceruk K-1 muat dalam satu frame dan geser 8 cm tidak perlu lagi.
  Uji dengan JEJAK di 20/30/40/50/60 cm × asli/dummy × serong kiri/kanan.
- Dataset di arena pada pose deployment, termasuk frame negatif yang jahat.
- Kunci exposure dan white balance saat runtime.
- Kalibrasi `cx_offset_px` dan pita `bbox_h` di depan K-1 yang sungguhan.

**PERIKSA SEBELUM LOMBA:** tabel `MISI` di `mission_hud.py` — pasangan
K-4/SZ-4/K-5/SZ-5 masih bisa dibaca dua cara dari panah di peta guidebook
halaman 21. Salah pasang = penyelamatan **tidak sah**.

---

## 12. Arsip versi

```bash
./simpan_arsip.sh r10          # SEBELUM mengubah apa pun
```

Snapshot masuk ke `arsip/<tanggal>_<label>/` — semua berkas kita plus
`kalib_*.json` yang sedang berlaku, karena kode tanpa angkanya bukan snapshot.

`arsip/RIWAYAT.md` mencatat **semua** rilis r1–r9 beserta apa dan kenapa yang
berubah, termasuk bug yang mahal.

**Jujur soal isinya:** berkas mentah r1–r8 sudah **tidak ada**. Tiap versi baru
menimpa yang lama, di PC maupun di Pi (`scp` ke `~/M`), dan folder ini tidak
pernah dipasangi git. Yang bisa diselamatkan cuma catatannya, dan itu ada
lengkap di `RIWAYAT.md`. Snapshot sungguhan mulai dari **r9**.

Kalau mau riwayat yang benar-benar utuh mulai sekarang, `git init` di folder ini
lebih baik daripada `simpan_arsip.sh` — skrip ini hanya jaring pengaman supaya
tidak ada lagi versi yang hilang tanpa jejak.

## Memutar sekian derajat (dan kenapa itu tidak sesederhana kelihatannya)

Firmware punya **dua** jalan memutar badan, dan keduanya tidak setara.

| perintah | cara | sudut | butuh IMU? |
|---|---|---|---|
| `O<der>` | pivot gait — kaki melangkah | bebas | **YA** |
| `o0..o3` | pivot ke mata angin arena | tetap | **YA** |
| `r0 0 <yaw>` | badan berputar di atas kaki yang tetap menapak | **±20° (`BODY_MAX_ROT_DEG`)** | tidak |

`nav.pivotRelatif()` — yang dipanggil `O` — **kembali diam-diam** kalau tidak ada
data IMU. Tidak ada satu baris pun di log serial. Gejalanya sama persis dengan
kabel putus, dan inilah sebabnya tombol `O90` di tab Manual tidak pernah berbuat
apa-apa sejak 6 September. `o0` sebaliknya *mencetak* `Gagal: Tidak ada data IMU.`
— itu sebabnya tombol diagnosa memakai `o0`, bukan `O`.

Jadi selama IMU belum hidup, **`r0 0 <yaw>` satu-satunya rotasi yang benar-benar
jalan** — dengan dua batas yang harus diingat:

1. **±20° keras dari firmware.** HUD memakai `badan_yaw_maks = 18°` supaya tidak
   pernah menyentuh batasnya.
2. **`r` itu perintah POSISI, bukan langkah.** `r0 0 18` dua kali **tidak**
   menghasilkan 36°; yang kedua tidak menggerakkan apa pun. HUD melacak pose
   yang sedang berlaku (`misi.badan_yaw`), menambahkan permintaanmu ke sana,
   lalu memotongnya — dan kalau potongannya membuat gerakannya nol, itu
   **dikatakan** (`pose badan sudah mentok …`), bukan dikirim diam-diam sebagai
   perintah yang terlihat berhasil di log.

### Kotak "Putar sekian derajat" (tab Manual)

Isi derajatnya, pilih metode, tekan KIRI atau KANAN. Tanda mengikuti firmware:
**+ = KIRI**, − = KANAN.

- **auto** — memilih `badan` selama sudutnya masih muat, `gait` kalau tidak.
  Ini yang dipakai sehari-hari selagi IMU mati.
- **badan (r)** — dipaksa lewat pose badan. Kalau sudah mentok, HUD menolak dan
  menyuruh **Nolkan pose** dulu.
- **gait (O)** — dipaksa lewat pivot gait. Dibulatkan **menjauh dari nol** supaya
  tidak pernah mendarat di `HEADING_TOLERANCE_DEG = 6°`, yang akan dibuang
  firmware tanpa pesan.

Untuk 35° seperti yang diminta: `auto` akan memakai **gait**, karena 35 tidak muat
di ±18. Selama IMU mati, cara menempuh 35° tanpa IMU adalah **dua kali 18° dengan
"Nolkan pose" di antaranya** — dan itu hanya memutar badan, bukan kaki, jadi
hasilnya tidak permanen: begitu pose dinolkan, badan kembali menghadap arah semula.
Rotasi permanen tanpa IMU **belum ada di firmware**; sumbu yaw di `Hexapod::walk()`
ada sejak awal tapi **keenam pemanggilnya selalu mengisi `0.0f`**, jadi tidak ada
satu perintah pun yang bisa mencapainya. Itu permintaan satu baris untuk Vincent.

## Lengan: yang depan bersendi, yang belakang tidak

| | lengan DEPAN (`a`, `g`) | lengan BELAKANG (`A`, `G`) |
|---|---|---|
| sendi | bahu + siku (dipecahkan IK dari jangkauan/tinggi) + pergelangan | **tidak ada** |
| grip | `g<0-100>` | `G<0-100>` |
| posisi | `a<jangkauan> <tinggi> [pergelangan]`, mm dari **PUSAT BADAN** | ditentukan letak **BADAN** |

`A<r> <h>` **ditolak firmware** dengan pesannya sendiri
(`Hexapod_Unlimited.ino:817`): *"Lengan BELAKANG tidak punya sendi — hanya grip."*
Tombol `A70 20` yang sempat ada di HUD sudah dibuang; ia terlihat berhasil lalu
tidak menggerakkan apa pun.

Tombol di kartu **Jejak korban** sekarang: `Lengan SIAP` / `TURUN (jepit)` /
`ANGKAT` memakai `lengan_r`, `lengan_h`, `lengan_angkat` dari kalibrasi, plus
kotak pergelangan (−90..90°). Argumen ketiga `a` hanya disentuh kalau diminta,
jadi jangkauan & tinggi ikut dikirim ulang — tanpa itu pergelangan tertinggal di
sudut lama.

**Jadi capit depan memang sudah bisa dipakai sekarang**, seluruhnya dari Pi, tanpa
menunggu Vincent. Yang belum ada adalah *urutan otomatis di dalam misi*:
`sekuensAmbil`/`sekuensTaruh` di `Misi.cpp` masih di dalam blok komentar
(baris 670–746), dan yang dikompilasi adalah stub 1500 ms di baris 660.

## Bug 11 Sep 2026 — tab Robot kosong, daya/suhu kosong, kalibrasi kosong

Tiga gejala, **satu akar**, plus satu bug kecil yang berdiri sendiri.

### Gejala kecil: tab pertama kosong sampai diklik

`.panel{display:none}` / `.panel.aktif{display:block}`. Tombol "Robot" sudah
`class=aktif`, tapi **panelnya tidak** — dan tidak ada yang memanggil `tab(0)`
saat halaman dimuat. Jadi kelima panel tersembunyi sampai tab pertama diklik.
Perbaikan: `<div class="panel aktif">` pada panel 0.

### Akar yang sebenarnya: satu id hilang membunuh SELURUH pembaruan

Saat tab dirapikan, beberapa kartu dipindah dan sebagian elemennya ikut hilang,
tapi `tarik()` masih memanggilnya. `$('state2')` mengembalikan `null`,
`.textContent=` melempar `TypeError`, dan **sisa `tarik()` tidak pernah jalan** —
daya, suhu, tegangan, bit throttle, 53 baris kalibrasi, semuanya berhenti di
baris itu.

Yang membuatnya mahal: dari luar gejalanya **"kartu kosong"**, bukan "ada error".
Tidak ada yang merah, tidak ada yang berhenti, halaman tetap terasa hidup karena
bilah atas diperbarui sebelum baris yang melempar. Tidak ada alasan untuk membuka
console. Delapan id yang hilang: `state2`, `next2`, `stdesc`, `lewat`, `slot`,
`penonton`, `stbox`, `gagalbox`.

Tiga perbaikan, berlapis:

1. **Elemennya dikembalikan.** Kartu "Keadaan sekarang" (`stbox`, `stdesc`,
   `lewat`, `slot`, `gagalbox`) kembali di tab Misi; `penonton` kembali ke bilah
   atas. `state2`/`next2` cuma cermin dari `state`/`next` yang sudah ada di bilah
   atas — barisnya dihapus, bukan elemennya dibuat lagi.
2. **`tarik()` dibungkus penangkap galat.** Kalau ada lagi yang lolos,
   kegagalannya **dikatakan** di bilah galat (`BUG TAMPILAN: …`) dan polling
   berikutnya tetap jalan — bukan diam sambil terlihat normal.
3. **Diuji, supaya tidak bisa terulang.** Bagian 37 menyisir setiap `$('id')` di
   JavaScript dan mencocokkannya ke `id=` di HTML. Kelas bug ini sekarang gagal
   di CI, bukan di arena.

### "Kok kalibrasi lain gaada?"

Itu gejala yang sama — daftarnya berhenti terisi di baris yang melempar. Tidak
ada yang sengaja disembunyikan: `/state` mengirim **53** knob, yaitu seluruh isi
`Kalib` dikurangi `kelas_korban` dan `kelas_dummy` (nama kelas model, bukan
tombol putar). Sesudah perbaikan, ketiga kartu di tab Kalibrasi terisi penuh.

## Jarak dari kamera — bisa, tapi berhenti tepat sebelum jarak capit

Jawaban singkat: **ya**, dan lebih teliti daripada dugaan — tapi **tidak sampai
10 cm**, dan alasannya geometri, bukan kualitas model.

### Rumusnya

    d = f · H / h_px          f = (lebar/2) / tan(hfov/2) = 907 px di 1280

`h_px` tinggi bbox, `H` tinggi efektif korban. Ketelitiannya justru bagus di
pita kerja kita, karena `|∂d/∂h| = d/h` mengecil saat mendekat:

| jarak | tinggi bbox (H=12 cm) | 5 px noise = |
|---|---|---|
| 60 cm | 182 px | 1,7 cm |
| 40 cm | 272 px | 0,7 cm |
| 30 cm | 363 px | 0,4 cm |
| 25 cm | 436 px | **0,3 cm** |
| 20 cm | 544 px | **0,2 cm** |

Pada 25 cm, 5 piksel noise bernilai **3 milimeter**. Itu lebih halus daripada
kuantisasi LiDAR.

### Dan di sinilah ia berhenti

Korban 12 cm mengisi **seluruh tinggi frame pada 15 cm**:

| jarak | H=8 cm | H=10 cm | H=12 cm | H=15 cm |
|---|---|---|---|---|
| 20 cm | 0,50 | 0,63 | 0,76 | 0,95 |
| 15 cm | 0,67 | 0,84 | **1,01 ✂** | **1,26 ✂** |
| 10 cm | **1,01 ✂** | **1,26 ✂** | **1,51 ✂** | **1,89 ✂** |

✂ = lebih tinggi dari frame. Pada `capit_cm = 10` kepalanya di atas frame dan
kakinya di bawah — **berapa pun bagusnya modelnya**. Lensa yang lebih lebar
menggeser batas ini, tidak menghapusnya.

Yang membuatnya berbahaya bukan cuma "tidak akurat", tapi **arah galatnya**.
Bbox terpotong punya `h_px` lebih kecil dari yang sebenarnya, dan `d = f·H/h_px`
membuatnya terbaca **lebih jauh**. Robot mengira masih ada 15 cm padahal sudah
8, lalu maju, lalu menabrak korban yang mau diselamatkannya. Jadi HUD
**mendeteksi pemotongan dan membuang bacaannya** — tidak menebak, tidak
"mengoreksi". Informasinya memang sudah hilang dari frame.

Metode alternatif (titik pijak di lantai, `d = tinggi_kamera / tan θ`) tidak
menolong di jarak itu: dengan vfov 43,3° dan kamera ~15 cm, tepi bawah frame
menyentuh lantai di ~38 cm. Di 10 cm kaki korban **di bawah frame**. Itu baru
mungkin kalau badan menunduk (`r0 -20 0`) dan sudut kamera diukur — pekerjaan
tersendiri, bukan satu knob.

### Jadi gunanya apa

**Memeriksa silang LiDAR, di pita di mana LiDAR paling rawan.** LiDAR depan
punya pita hantu di **3,2 dan 9 cm** yang Vincent dokumentasikan, dan target
capit kita **10 cm** — tepat di sebelahnya. Satu-satunya cara tahu LiDAR sedang
berhalusinasi adalah sensor kedua yang salahnya tidak berkorelasi. Kamera
memberi itu, dan justru teliti di 20–40 cm.

Di `AMBIL_MAJU`, kalau kamera dan LiDAR berselisih lebih dari
`vision_lidar_beda_maks_cm` (6 cm), robot **berhenti dan bilang**:

    LiDAR 9 cm tapi kamera 30 cm (beda +21 cm, batas 6). Salah satu salah
    -- di dekat 10 cm tersangkanya pita hantu LiDAR.

Bukan memilih sendiri sensor mana yang dipercaya lalu menutup capit di udara.
Bisa dimatikan lewat `vision_silang_on` kalau kamera bermasalah di arena.

Kartu **Ambil korban** sekarang punya tiga baris: jarak LiDAR, jarak kamera
(dengan sebabnya kalau kosong), dan bedanya — merah kalau lewat ambang.

### Kalibrasi — jangan ukur tinggi bonekanya

Yang dibutuhkan rumus bukan tinggi fisik korban, melainkan tinggi yang
**dilihat model**: bbox YOLO jarang mepet, sering memotong kaki, dan lensanya
tidak sempurna. Tinggi fisik yang "benar" justru membuat rumusnya meleset.

Caranya: taruh korban di jarak yang kamu ukur sendiri dengan penggaris
(**30–40 cm paling enak** — bbox utuh dan besar), pastikan terdeteksi (Mulai
JEJAK), isi jaraknya di tab Kalibrasi, tekan **Kalibrasi jarak vision**. Satu
pengukuran menyerap bbox longgar, distorsi lensa dan kaki terpotong sekaligus.
RAM saja, seperti seluruh kalibrasi lain.

Kalau bbox-nya menyentuh tepi frame saat menekan, HUD menolak dan menyuruh
memundurkan korban — kalibrasi dari bbox terpotong akan menanam galat ke
**semua** pengukuran berikutnya.

## Metode baru: LENGAN yang maju, bukan badan (ukur R2C, 11 Sep 2026)

Ukuran yang diambil di robot: **jangkauan depan 25 cm, tinggi 12 cm, vektor
28 cm** ke titik tengah lubang capit.

**Ketiganya konsisten.** `hypot(25, 12) = 27,73` vs 28 yang diukur — meleset
2,7 mm, di dalam ketelitian penggaris. HUD menghitung ini tiap detik dan
menampilkannya di baris **geometri lengan**, merah kalau tidak cocok, karena
salah ketik di sini membuat lengan menjulur ke tempat yang bukan tempat korban
**tanpa satu pun pesan error** — firmware dengan senang hati mengerjakan
koordinat yang sah tapi keliru.

### Kenapa ini perbaikan besar, bukan cuma cara lain

Titik kerjanya pindah dari **10 cm ke 25 cm**, dan itu memindahkannya dari
tempat kedua sensor paling lemah ke tempat keduanya paling kuat:

| | 10 cm (badan maju) | 25 cm (lengan maju) |
|---|---|---|
| bbox kamera | **terpotong** — kamera buta | 0,60 tinggi frame, utuh |
| ketelitian kamera | — | **2,9 mm** (5 px noise) |
| LiDAR | tepat di sebelah pita hantu 9 cm | jauh dari pita hantu |
| silang dua sensor | tidak mungkin | **jalan** |

Jadi jawaban "vision tidak bisa mengukur di jarak capit" yang kutulis kemarin
**tidak lagi berlaku** untuk metode ini. Di 25 cm vision justru sensor terbaik
yang kita punya.

### Knob baru

| knob | nilai | arti |
|---|---|---|
| `mode_ambil` | **lengan** / badan | "lengan" = berhenti di 25 cm lalu julurkan; "badan" = cara lama |
| `lengan_jangkau_cm` | 25,0 | jangkauan depan, dari pusat badan |
| `lengan_tinggi_cm` | **−12,0** | tinggi. **Tandanya belum dipastikan** — lihat di bawah |
| `lengan_vektor_ukur_cm` | 28,0 | yang kamu ukur, untuk diperiksa silang |

Rantai AMBIL sekarang berhenti di `jarak_ambil_cm()`, satu fungsi yang dipakai
semua tempat — bukan `capit_cm` yang tersebar di beberapa baris.

`AMBIL_SIAP` mengirim `g100` **lalu** `a250 -120`: capit dibuka dulu, baru
lengan menjulur. Urutan terbalik akan menyodok korban dengan capit tertutup dan
menggesernya — lalu seluruh penengahan yang barusan dikerjakan jadi sia-sia.

### Satu hal yang BELUM bisa kupastikan: tanda tingginya

`hypot()` membuang tanda, jadi pemeriksaan konsistensi di atas **tidak bisa
membedakan lengan naik 12 cm dari lengan turun 12 cm** — keduanya lolos. Ini
batas nyata dari pemeriksaan itu, bukan sesuatu yang bisa ditutup dengan kode.

Default-nya **−12** (turun), karena korban tergeletak di lantai dan K-3/K-4
cuma 1 cm dari lantai menurut guidebook, jadi pusat badan hampir pasti di atas
titik capit. Kalau ternyata terbalik, ganti satu knob.

Cara memastikannya: tekan **Uji jangkauan lengan** di kartu Ambil korban, lalu
lihat lengannya bergerak ke mana. Tombol itu mengirim pose jepit saja, tanpa
menjalankan rantai AMBIL dan tanpa menyentuh capit.

### Yang perlu diperiksa di robot, bukan di sini

**1. Apakah lengan benar-benar sampai 250 mm.** `ARM_ORIGINS` di `config.h`
masih ditandai Vincent "ANGKA 50 MASIH PERKIRAAN", dan IK-nya dibatasi
`UPPERARM_LENGTH + FOREARM_LENGTH`. Kalau 250 mm di luar itu, firmware menjawab
`!! Di luar jangkauan lengan -- sudut TIDAK dikirim` dan lengannya **tidak
bergerak** — bukan bergerak salah. Itu gagal yang aman, dan itu jawaban yang
dicari tombol Uji jangkauan.

**2. Toleransinya.** Yang menentukan bukan kamera. Kamera memberi ±3 mm di
25 cm; yang mengikat adalah **lebar bukaan capit dikurangi lebar korban,
dibagi dua**. Ukur bukaan capit saat `g100`, dan lebar korban di titik yang
dijepit — selisihnya yang jadi anggaran, dan dari situ `capit_tol_cm` (sekarang
1,5 cm) bisa disetel dengan alasan, bukan ditebak.

**3. Ukuran korban — aku TIDAK tahu, dan tidak perlu tahu.** Guidebook cuma
menyebut "oranye, ≥ 33 g" dan menaut model STL; dimensinya tidak ada di sana.
`korban_tinggi_cm = 12.0` itu **angka tempelan buatanku**, bukan hasil ukur —
jangan dipercaya. Tapi memang tidak perlu diukur: tekan **Kalibrasi jarak
vision** dengan korban di 25 cm dan angka itu diselesaikan sendiri, sekaligus
menyerap bbox longgar, distorsi lensa, kaki terpotong, **dan** selisih titik
nol kamera vs titik nol lengan. Kalibrasi di 25 cm juga berarti ia paling tepat
persis di jarak yang dipakai.

## Ukuran korban terukur: 9 × 8,5 cm (R2C, 11 Sep 2026)

`korban_tinggi_cm` sekarang **9,0** — angka ukur, bukan lagi tempelan 12,0 yang
kubuat sendiri. Dan itu menggeser batas penting:

| | dengan tebakan 12 cm | dengan ukuran nyata 9 cm |
|---|---|---|
| bbox mulai terpotong di | 15,1 cm | **11,3 cm** |
| bbox di 25 cm | 435 px (0,60) | **327 px (0,45)** |
| 5 px noise di 25 cm | 2,9 mm | **3,8 mm** |

Cara lama (berhenti di 10 cm) memang **di dalam** zona buta — 10 < 11,3. Cara
baru (25 cm) jauh di luarnya, dan bbox-nya duduk nyaman di tengah pita
`bbox_h_min/max`.

### Lebar dipakai untuk MEMERIKSA, bukan mengukur

Memutar korban pada sumbu tegak **tidak mengubah tingginya**, cuma lebarnya.
Jadi tinggi = besaran yang kokoh untuk jarak, lebar = petunjuk orientasi yang
murah. Rasio menghadap kamera = 8,5/9 = **0,94**.

Di luar `rasio_min..rasio_maks` (0,55–1,60), jaraknya **dibuang** dan sebabnya
ditulis — karena bbox dengan bentuk salah bukan satu korban utuh: terpotong
tepi frame, bergabung dengan dummy sebelahnya, atau separuh tertutup papan
14×17 di K-3/K-4. Bisa dimatikan lewat `rasio_periksa`.

### Kalibrasi per posisi K-1..K-5

Lima knob: `tinggi_k1` … `tinggi_k5`, semuanya mulai **0 = belum dikalibrasi**
(bukan "tingginya nol") dan jatuh ke `korban_tinggi_cm` global. Tanpa aturan
itu, posisi yang belum sempat dikalibrasi akan membuat jaraknya meledak tanpa
ada yang menyadarinya.

Kenapa perlu, padahal tinggi tidak berubah saat korban diputar? Yang berubah
bukan korbannya, melainkan **cara ia terlihat**: K-3/K-4 tertimpa papan dan
cuma 1 cm dari lantai, SZ-2/SZ-4 setinggi 4 cm — bbox-nya terpotong tanggul
atau papan dengan cara yang berbeda-beda di tiap ruang. Satu angka global tidak
bisa mewakili kelimanya.

Tombol **Kalibrasi jarak vision** menulis ke posisi yang **sedang dikerjakan**
kalau robot ada di ruang korban, dan ke global kalau tidak. Jadi kalibrasi tiap
posisi tinggal: berdiri di ruangnya, ukur jaraknya, tekan. Baris **posisi
kalib** di kartu Ambil korban menunjukkan mana yang akan ditulis.

### Ambang dalam PIKSEL, bukan cm

`bbox_tol_px` = 20 px. Ambangnya di piksel karena **di situlah noise-nya
hidup** — menyetelnya dalam cm berarti menyetel bayangan dari besaran yang
sebenarnya. Padanan cm-nya ikut jarak (`|dd/dh| = d/h`):

| jarak | bbox | 20 px setara |
|---|---|---|
| 25 cm | 327 px | **1,53 cm** |
| 50 cm | 163 px | 6,1 cm |

Di 25 cm itu **sepadan dengan `capit_tol_cm` = 1,5 cm** — jadi keduanya tidak
saling bertengkar. Baris **bbox diharap** menampilkan kedua satuannya sekaligus
supaya yang satu bisa disetel dari yang lain, bukan masing-masing ditebak.

### Yang tidak lagi jadi pertanyaan

Lebar bukaan capit **diurus Vincent**, jadi anggaran toleransi mekanisnya
bukan lagi bagian kita. Yang tinggal di sisi vision cuma dua: ketelitian
(±3,8 mm di 25 cm — cukup jauh di bawah apa pun yang masuk akal untuk bukaan
capit) dan kalibrasi per posisi di atas.

## Flash Teensy dari Raspberry Pi

Tiga berkas baru:

| berkas | jalan di | gunanya |
|---|---|---|
| `siapkan_teensy_pi.sh` | Pi, **sekali saja** | pasang arduino-cli, core Teensy, pustaka, teensy_loader_cli, udev |
| `kirim_teensy.bat` | laptop (CMD) | kirim satu versi Vincent ke `~/teensy` di Pi |
| `flash_teensy.py` | Pi | pilih versi, periksa pustaka, compile, flash |

Alurnya:

```
kirim_teensy.bat                                       (laptop, pilih versi)
ssh bima@terra-core "python3 ~/M/flash_teensy.py --coba --terbaru"   (PERIKSA)
ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"       (FLASH)
```

`--coba` menyiapkan, mengekstrak zip, memindai `#include` dan melaporkan
pustaka — **tanpa** compile dan **tanpa** menyentuh Teensy. Selalu jalankan itu
dulu. `-t` pada perintah flash penting: kalau Teensy minta tombol PROGRAM
ditekan, pesannya harus benar-benar muncul di layarmu, bukan tertahan di buffer.

### Adaptasi ke file Vincent — ini bagian yang sesungguhnya

Nama foldernya tidak konsisten. Yang benar-benar ada di disk:

```
Hexapod_KRSRI_2026-ver1_4      "ver", garis bawah
Hexapod_KRSRI_2026-Ver1_8      "Ver" huruf besar
Hexapod_KRSRI_2026-v1.61       "v", titik
Hexapod_Unlimited_v1.9         nama proyek ganti
Hexapod_Unlimited_v1.12.zip    ZIP, belum diekstrak
```

Jadi skrip **tidak menebak pola nama**: ia mencari semua yang mengandung
`.ino`, folder maupun zip.

**Dan versi TIDAK dipakai untuk mengurutkan.** `v1.61` terurai jadi (1, 61),
dan (1, 61) > (1, 12) — padahal v1.61 itu versi KRSRI lama, jauh sebelum
Unlimited v1.12. Agaknya "1.6 revisi 1", tapi tidak ada parser yang bisa tahu
itu dari namanya. Tes menangkap ini saat ditulis; kalau dibiarkan, `--terbaru`
akan mem-flash firmware paling tua ke robot — kesalahan yang **persis sama**
dengan bug `dir /s /o-d` dulu, cuma lewat pintu lain.

Jadi: **urutkan menurut waktu berkas, tampilkan nomor versi, dan kalau
keduanya tidak sepakat — katakan.** Skrip mencetak peringatan dan tetap
meminta kamu memeriksa baris terakhir sebelum flash.

Itu juga sebabnya `kirim_teensy.bat` memakai **`scp -p`**. Tanpa `-p`, scp
menulis ulang semua stempel waktu dengan waktu sekarang; sesudah dua kali
kirim seluruh versi jadi seumur dan satu-satunya petunjuk yang benar hilang.
Kalau waktunya seri, skrip jatuh ke nomor versi sebagai kunci kedua — bukan
ke urutan abjad, yang tidak berarti apa-apa.

Hal lain yang diadaptasi per versi:

- **Zip diekstrak sendiri** ke folder sementara. v1.12 memang datang sebagai
  zip dan tidak pernah diekstrak; melewatkannya berarti versi terbaru tidak
  kelihatan sama sekali.
- **Nama folder vs nama `.ino`.** Arduino mewajibkan keduanya sama dan menolak
  dengan pesan yang tidak menyebut sebabnya. Kalau berbeda, skrip membuat
  **salinan** bernama benar — file Vincent tidak pernah disentuh.
- **`#include` dipindai ulang tiap versi**, bukan dari daftar tetap. Kalau
  Vincent menambah pustaka, yang muncul adalah "pustaka X belum ada" beserta
  perintah pasangnya — bukan ratusan baris error kompilasi. Dan kalau yang
  tidak dikenal itu justru berkas Vincent sendiri (`Calib.h`, `config.h`),
  skrip bilang **kirimannya tidak lengkap**, bukan pustakanya kurang.
- **HUD dimatikan dan dinyalakan lagi** di sekitar flash, karena ia memegang
  `/dev/ttyACM0`. Penyalaannya di blok `finally` — kalau flash gagal dan HUD
  tertinggal mati, gejala berikutnya jadi "site cannot be reached", salah
  petunjuk yang sudah pernah memakan waktu sekali di proyek ini.
- **Compile gagal = Teensy tidak disentuh sama sekali.** Urutannya compile
  dulu sampai ada `.hex`, baru port dibuka.

### Kenapa tidak ada tombol flash di HUD

Godaannya besar, tapi HUD memegang port serial yang dibutuhkan flashing, jadi
HUD harus mati dulu — dan tombol yang mematikan servernya sendiri lalu
berharap bisa melapor hasilnya itu rancangan yang salah sejak awal.

### Yang belum bisa kuuji

Compile dan flash sungguhan tidak bisa dijalankan dari sini — tidak ada Teensy
dan tidak ada arduino-cli. Yang **sudah** diuji dengan berkas Vincent yang
asli: penemuan versi, urutan waktu vs nomor (termasuk jebakan v1.61),
ekstraksi zip v1.12, pencocokan nama folder/`.ino`, dan pemindaian `#include`
(hasilnya tepat dua: VL53L1X dan Adafruit PWM Servo Driver, tanpa satu pun
header Vincent ikut salah dituduh).

Reboot lunak 134 baud juga belum teruji di perangkat. Kalau gagal, skrip
bilang begitu dan menyuruh menekan tombol PROGRAM — jadi gagalnya terlihat,
bukan menggantung.

## v1.12: sekuens capit akhirnya BENAR-BENAR dikompilasi

Di v1.11 `sekuensAmbil`/`sekuensTaruh` masih di dalam blok komentar dan yang
jalan adalah stub 1500 ms. **Di v1.12 keduanya hidup** dan dipanggil di
`Misi.cpp:1540`. Ada `ArmInverse.cpp` dan `HexaArm.cpp` baru, pergelangan ikut
IK lewat `moveArmGrip()`, dan pose yang tak terjangkau **ditolak dengan sebab**
alih-alih ter-clamp diam-diam.

Konstanta yang penting buat kita, dari `config.h`:

| konstanta | nilai | arti |
|---|---|---|
| `KORBAN_JARAK_CM` | **24** | gerbang `HNT_DEPAN` di kelima ruas AMBIL |
| `LIDAR_DEPAN_MM` | 62,0 | LiDAR depan, di depan pusat badan |
| `KORBAN_CAPIT_MM` | 24×10 + 62 = **302** | titik capit dari pusat badan |
| `KORBAN_TINGGI_MM` | 40,0 | tinggi titik jepit **dari lantai** |

**Vincent sampai di 24 cm, kamu mengukur 25 cm.** Itu kesepakatan dalam 1 cm
dari dua arah yang sama sekali berbeda — dan cukup kuat untuk dipercaya.

`lengan_jangkau_cm` diselaraskan **25 → 24** supaya HUD dan firmware berhenti
di angka yang sama. Kalau berbeda, yang satu terus menyuruh maju sementara
yang lain merasa sudah sampai. Di 24 cm bbox masih utuh (340 px, 5 px noise =
3,5 mm), jadi tidak ada yang hilang dari sisi vision.

Yang **belum** berubah di v1.12: `AKS_KONFIRM` masih cuma 2 kemunculan
(handler + cetak), tidak pernah dipakai di tabel. Jadi vision masih belum bisa
menyela misi — sama seperti v1.10 dan v1.11.

Satu hal yang perlu diperhatikan sekarang: **firmware sudah punya sekuens
ambilnya sendiri.** Rantai AMBIL di HUD dan `sekuensAmbil` di Teensy sekarang
sama-sama bisa menggerakkan lengan. Jangan jalankan keduanya bersamaan —
pakai rantai HUD untuk uji manual di luar misi, dan biarkan firmware yang
bekerja saat `m1` berjalan.

Catatan kecil: komentar `Misi.cpp:415` menyebut **"4 servo depan, 1 belakang,
dikonfirmasi 7 Sep 2026"** — jadi 3 sendi + grip di depan, grip saja di
belakang. Cocok dengan yang kamu sebut.

## Lup Teensy ↔ Pi: apa yang sudah ada, dan satu baris yang belum

Pertanyaannya: bisakah Teensy memberitahu Pi untuk masuk mode vision, Pi
mengambil alih gerakan, lalu memberitahu Teensy "posisi sudah pas, silakan
capit dan lanjutkan misi"?

**Setengahnya sudah ada di v1.12, dan setengahnya lagi sudah ada sejak v1.9 —
cuma belum disambung.**

### Arah 1: Teensy → Pi. **SUDAH ADA** (baru di v1.12)

`Misi.cpp:1521`, begitu robot berhenti di depan korban:

```cpp
KORBAN_SERIAL.print("#KORBAN ");
KORBAN_SERIAL.print(x.aksi == AKS_AMBIL ? "AMBIL " : "TARUH ");
KORBAN_SERIAL.println(_i);
```

`KORBAN_SERIAL` = `Serial` (USB ke Pi, 115200) — jadi ini datang di port yang
**sudah** dibaca HUD. Ini pertama kalinya firmware berbicara **duluan** ke Pi;
sebelumnya HUD cuma bisa menebak dari nama ruas.

HUD sekarang memparsinya (`RE_PEMICU`) dan bertindak menurut knob
`pemicu_korban`.

### Arah 2: Pi → Teensy. **MEKANISMENYA ADA, PEMICUNYA TIDAK**

`AKS_KONFIRM` → `MISI_KONFIRM` → `m2`/`m3` sudah lengkap dan berfungsi:

```cpp
void Misi::jawab(bool lanjut) {
    if (_stat != MISI_KONFIRM) { ...tolak... }
    if (lanjut) { Serial.println("Dicatat: LANJUT."); ruasBerikut(); return; }
    ...
}
```

`m2` → `ruasBerikut()` → ruas AMBIL berikutnya → `sekuensAmbil` jalan → misi
lanjut. **Persis yang kamu minta.** Tapi `AKS_KONFIRM` masih cuma **2
kemunculan** di seluruh v1.12 (handler + cetak) — **tidak pernah dipakai di
tabel `RUAS[]`**. Jadi robot tidak pernah sampai ke sana.

### Yang kurang: satu baris per korban

Sisipkan satu ruas `AKS_KONFIRM` **sebelum** tiap ruas `AKS_AMBIL`:

```
{ "K-1 konfirmasi vision", BLK_LURUS, KMD_KANAN, PRF_DATAR, false,
  HNT_LANGSUNG, 0, AKS_KONFIRM, ARM_DEPAN },
{ "K-1 angkat korban",     ...,                              AKS_AMBIL, ARM_DEPAN },
```

Lima baris, nol mekanisme baru. Sesudah itu lupnya tertutup penuh:

```
Teensy berhenti  →  "#KORBAN AMBIL 1"  →  parkir di KONFIRM
                                              ↓
                         Pi: vision, tengahkan badan (r/O)
                                              ↓
                    Pi: netralkan badan, kirim "m2"
                                              ↓
       Teensy: ruasBerikut() → sekuensAmbil → capit → misi lanjut
```

### Kenapa bawaannya "lihat", bukan "ambil_alih"

Vincent menulis pemicunya **kirim-lalu-lanjut**, dan berkata begitu di
komentarnya: *"Kirim saja, tidak menunggu jawaban: TAHAP 1 memang tidak
memakai balasannya."* Sebaris di bawahnya Teensy masuk `MISI_LENGAN` dan mulai
menurunkan capit. Ia **tidak sedang menunggu siapa pun.**

Kalau Pi ikut menggerakkan badan di detik itu, ada **dua penguasa untuk satu
robot** — dan yang kalah adalah korban yang tersenggol capit yang sedang
turun.

Jadi `pemicu_korban` punya tiga nilai:

| nilai | yang dikerjakan | aman dengan v1.12 apa adanya? |
|---|---|---|
| `mati` | abaikan | ya |
| **`lihat`** (bawaan) | vision menilai, **nol** perintah gerak | **ya** |
| `ambil_alih` | tengahkan lalu jawab `m2` | hanya kalau ada ruas KONFIRM |

Dan `ambil_alih` **tidak memercayai knob-nya sendiri**: ia mencari **bukti**
bahwa firmware benar-benar parkir (baris `MENUNGGU KONFIRMASI`). Tanpa bukti
itu ia turun sendiri ke `lihat` dan menulis sebabnya lengkap, termasuk apa
yang perlu Vincent tambahkan. Niat yang ditulis di knob tidak cukup untuk
menggerakkan robot di sebelah korban.

### Serah terima balik

Kalau rantai AMBIL dimulai oleh pemicu firmware (`dari_firmware`), ujungnya
**bukan** rantai capit HUD — dua sekuens lengan untuk satu lengan hanya saling
menimpa. Sesudah penengahan halus selesai, HUD:

1. **menetralkan badan dulu** (`r0 0 0`, `t0 0 0`), lalu
2. mengirim `m2`.

Urutannya bukan selera. Pose badan itu perintah **posisi** yang masih berlaku;
kalau ditinggalkan menyerong, firmware menjalankan sekuens lengannya pada badan
yang miring — dan seluruh penengahan yang barusan dikerjakan justru berbalik
jadi galat. Tesnya memeriksa `m2` memang datang **sesudah** `r0 0 0`.

`auto_konfirm` (bawaan **mati**) menentukan apakah `m2` dikirim sendiri atau
kamu yang menekan. Kalau mati, HUD tetap menetralkan badan dan bilang bahwa
`m2` menunggumu.

Baris **pemicu firmware** dan **yang mengambil** di kartu Ambil korban
menunjukkan keadaannya sekarang.

## Kenapa kompasnya kosong — regex LiDAR yang menelannya

EEPROM memang ada isinya, dan firmware memang memuatnya: `Navigation.cpp:10`
memanggil `kompasMuat(false)` saat boot, dan `kompasTabel()` (perintah `k`)
mencetaknya **tanpa menyentuh IMU sama sekali**. Jadi angkanya ada di RAM
Teensy sejak detik pertama.

Yang salah ada di sisi HUD, dan ada dua lapis:

**1. HUD tidak pernah menampilkannya.** Kartu "Kalibrasi kompas arena" cuma
berisi tombol — tidak ada satu pun baris yang menunjukkan keempat arahnya.
Hasil `k` hanya lewat di log serial lalu tergulung hilang.

**2. Lebih buruk: parsernya menelan baris itu.** Keluaran `k` berbentuk

```
0 UTARA	: 12.5 der
```

dan `RE_LIDAR` — `^\s*([0-5])\s+([A-Z\-]+)\s*:\s*(.+?)...` — **menerimanya**:
`ch=0`, `nama='UTARA'`. Jadi tiap kali `k` ditekan, keempat baris kompas masuk
ke `lidar_ch0..3` dan `lidar_UTARA`, sementara angka kompasnya sendiri tidak
pernah sampai ke tempat yang menampilkannya. Dari luar gejalanya persis
"kompasnya kosong padahal EEPROM ada isinya".

(Kebetulan `lidar_chN` tidak pernah dibaca siapa pun, jadi pencemarannya belum
merugikan. Itu keberuntungan, bukan rancangan.)

Perbaikannya tiga bagian:

- Parser kompas sendiri, dijalankan **sebelum** `RE_LIDAR`, dipagari header
  `--- KOMPAS ARENA ---` supaya ia tidak menebak dari bentuk baris saja. Tabel
  ditutup oleh baris pertama yang bukan anggotanya, jadi tabel LiDAR
  sesudahnya kembali dibaca normal.
- Kartu kompas sekarang **menampilkan keempat arah**, hijau kalau tercatat,
  kuning kalau "belum dicatat", plus umur bacaannya dan peringatan kalau belum
  lengkap.
- HUD mengirim `k` **sekali tiap sambungan** supaya kartunya terisi sendiri.
  Sekali, bukan tiap poll: angkanya tidak berubah sendiri, dan empat baris per
  detik demi angka yang diam itu hanya menenggelamkan log.

Catatan: `belum dicatat` disimpan sebagai `None`, bukan `0` — **0 derajat itu
arah yang sah**, dan menyamakan keduanya akan membuat UTARA yang benar-benar
tercatat di 0 der terlihat seperti belum dikalibrasi.

## Bisakah misi dimulai sekarang? Belum — tapi bukan ruas 30 yang menghalangi

> ✅ **SUDAH SELESAI (13 Sep 2026).** IMU hidup. Akarnya bukan baud dan bukan
> kabel: **HUD yang menolak informasi yang sudah dikirim Teensy**. Seluruh
> bagian ini dan dua bagian di bawahnya (`C` ditolak, tersangka baud)
> disimpan sebagai catatan cara menyempitkan masalah — bukan sebagai keadaan
> sekarang. Jangan menyetel ulang `IMU_BAUD` atas dasar tulisan di sini.

Ruas 30 masih `-1` di v1.12 (`Misi.cpp:559`). Yang kuperbaiki dulu itu
**tombol m7** untuk mengisinya, bukan firmware-nya — jadi tidak ada yang
berubah di sisi Vincent.

Tapi membaca v1.12 baris per baris, urutan penolakannya ternyata begini:

| # | penjaga | butuh IMU? | keadaan sekarang |
|---|---|---|---|
| 1 | `!_imu.hasData()` (`Navigation.cpp:617`) | **YA** | **INI yang menghentikanmu** |
| 2 | `kompasLengkap()` | tidak | lolos kalau EEPROM terisi |
| 3 | `pivotTerkalibrasi()` | tidak | lolos kalau EEPROM terisi |
| 4 | `tabelSiap()` — ruas 30 | tidak | masih `-1` |

Jadi **selama IMU mati, ruas 30 bahkan belum sempat diperiksa.** Mengisinya
tidak akan mengubah apa pun sampai IMU hidup. Gerbang 2 dan 3 cuma membaca
EEPROM — keduanya tidak butuh IMU untuk *diperiksa*, hanya untuk *dipakai*.

### Dan ada jalan memutar untuk ruas 30

`m4 <awal> <akhir>` menjalankan misi pada **rentang** ruas, dan
`tabelSiap(dari, sampai)` hanya memeriksa rentang itu:

```cpp
if (n >= 3)      misi.mulaiDari((uint8_t)p[1], (uint8_t)p[2]);   // m4 6 13
```

Jadi **`m4 0 29`** menempuh HOME sampai K-5 — hampir seluruh misi — dan ruas 30
tidak ikut diperiksa sama sekali. Tombolnya sudah ada di kartu Jalankan misi,
dengan kotak awal/akhir yang bisa kamu ubah.

Itu tidak melewati gerbang IMU. Urutan yang benar tetap: **hidupkan IMU dulu**
(Serial2, pin 7/8, 230400) — sesudah itu `m4 0 29` bisa berangkat hari itu juga
tanpa menunggu ruas 30 diukur.

## `C` juga ditolak — dan itu bukan masalah keempat, itu masalah yang sama

> ✅ **SELESAI 13 Sep 2026** — lihat penanda di bagian sebelumnya.

`Navigation.cpp:460`:

```cpp
void Navigation::kalibrasiPivot(uint8_t siklus) {
    if (!_imu.hasData()) { Serial.println("Gagal: Tidak ada data IMU."); return; }
```

Dan ini bukan gerbang yang bisa diakali: kalibrasi pivot **mengukur perubahan
yaw** — seluruh keluarannya (`_degCCW`, `_degCW`, `_pivotSign`) lahir dari
bacaan IMU. Tanpa IMU, tidak ada yang bisa dikalibrasi, bukan sekadar tidak
boleh.

Daftar lengkap yang terhalang satu kegagalan yang sama:

| perintah | gerbang |
|---|---|
| `C` kalibrasi pivot | `Navigation.cpp:460` |
| `c0..c3` catat kompas | `Navigation.cpp:65` |
| `O` pivot relatif | kembali **diam-diam**, tanpa pesan |
| `o0..o3` hadap arah | `"Gagal: Tidak ada data IMU."` |
| `m1` / `m4` mulai misi | `Navigation.cpp:617` |

Selama berhari-hari ini menyamar jadi lima masalah berbeda. Sebenarnya satu.
**Menyetel yang lain tidak akan menolong sampai IMU hidup**, dan itu berarti
tidak ada gunanya mengukur ruas 30 sekarang.

### Tersangka utama: baud, bukan kabel

> ⚠️ **TEBAKAN INI MELESET.** Baud-nya tidak pernah salah. Ditinggalkan apa
> adanya karena cara menyempitkannya (`sidik_imu.py`, memisahkan "tidak
> terkirim" dari "tidak dibalas") tetap berguna — tapi kesimpulannya jangan
> dipakai.

`Imu::begin()` hanya **membuka** port:

```cpp
void Imu::begin() {
    IMU_SERIAL.addMemoryForRead(_rxExtra, sizeof(_rxExtra));
    IMU_SERIAL.begin(IMU_BAUD);
}
```

Ia **tidak pernah mengirim apa pun** ke modul — tidak ada perintah pindah
baud, tidak ada permintaan mulai mengirim. Jadi seluruh sambungan bergantung
pada modul yang **sudah** berada di baud yang sama.

Dan komentar Vincent sendiri, `Imu.cpp:19`:

> *"Pastikan IMU_BAUD di config.h sudah diubah ke 230400"*

Kata **diubah** itu petunjuknya — angka aslinya bukan 230400. Modul WIT
(Yahboom 10-axis memakai protokol WIT) keluar pabrik pada **9600**. Modul di
9600 + Teensy di 230400 = **senyap total, tanpa satu pun pesan error**, gejala
yang sama persis dengan kabel putus.

### `sidik_imu.py` — membuktikannya hari ini, tanpa alat tambahan

**Koreksi:** aku pernah bilang Type-C kosong di modul itu mungkin colokan
daya. Kemungkinan besar **keliru**. Di Yahboom 10-axis itu biasanya antarmuka
USB-serial (CH340/CP2102 di papan yang sama). Kalau benar, colok saja ke Pi —
modulnya muncul sebagai `/dev/ttyUSB0` dan bisa didengarkan langsung, **tanpa
lewat Teensy**.

```
python3 ~/M/sidik_imu.py            # pindai semua port x 9 baud
python3 ~/M/sidik_imu.py --uji      # uji parsernya sendiri, tanpa perangkat
```

Ia mendengarkan tiap baud, menghitung frame WIT yang **sah** (header `0x55`,
11 byte, checksum benar), memilah jenis paketnya, dan mencetak roll/pitch/yaw
yang terbaca supaya kamu bisa memutar modulnya dan melihat angkanya ikut
berubah — bukti bahwa itu data sungguhan, bukan kebetulan.

Dua kemungkinan hasil, dan **keduanya** memajukan:

- **Ketemu di baud X.** Modul hidup, masalahnya cuma angka. Skrip mencetak
  dua jalan keluar dan menyarankan yang paling murah: ubah `IMU_BAUD` di
  `config.h:629` ikut modul (satu baris, mudah dibalik, tidak bisa membuat
  modul jadi tak bisa dihubungi).
- **Tidak ketemu di baud mana pun.** Itu juga jawaban: **berhenti menyetel
  angka.** Yang tersisa daya, kabel, atau modulnya. Skrip mencetak urutan
  memeriksanya — termasuk kabel Type-C yang cuma untuk mengecas (tidak punya
  jalur data), dan TX/RX yang tertukar (TX modul → pin 7, RX modul → pin 8).

Parsernya ditulis meniru `Imu::update()` firmware, lalu **diadu dengan
implementasi firmware pada 200 aliran byte acak** — nol perbedaan. Itu bukan
kerapian: kalau skrip ini menemukan frame yang firmware tidak temukan,
diagnosanya akan menuduh yang salah, dan diagnosa yang salah lebih mahal
daripada tidak punya diagnosa.

Satu hal lagi yang diperiksa: kalau ada frame sah tapi **tidak ada paket
0x53 (SUDUT)**, skrip bilang begitu — hanya 0x53 yang mengisi `yawDeg()`, jadi
`hasData()` tetap false walau modulnya ramai mengirim akselerasi dan giro.

## Bug 11 Sep 2026 — tes mati di Pi karena satu berkas tidak ikut terkirim

```
FileNotFoundError: '/home/bima/M/sidik_imu.py'
```

Tes 54 memuat `sidik_imu.py` untuk mengadu parser WIT-nya dengan firmware.
Tapi `sidik_imu.py` **tidak ada di daftar `BERKAS`** `upload_to_pi.bat`. Di
laptop semuanya hijau; di Pi tesnya mati — dan **600 tes lain yang tidak ada
hubungannya ikut tidak pernah jalan.**

Yang membuatnya lebih dari sekadar berkas lupa dikirim: **satu berkas
pendukung yang hilang menggugurkan seluruh berkas tes.** Itu rapuh, dan akan
terulang dengan berkas lain suatu saat.

Tiga perbaikan, berlapis:

1. **Jalur diselesaikan relatif ke berkas tes**, bukan ke direktori kerja.
   Tes dijalankan dari mana-mana (systemd, ssh satu baris, upload_to_pi.bat);
   jalur relatif ke cwd menunjuk tempat berbeda tiap kali.
2. **Berkas yang hilang DILEWATI dengan sebabnya**, bukan meledak.
   `sidik_imu.py` itu alat diagnosa yang berdiri sendiri — ketidakhadirannya
   bukan alasan menggugurkan 600 tes yang lain.
3. **Penjaga struktural (bagian 55).** Tes menyisir dirinya sendiri untuk
   nama berkas `.py` yang dimuatnya, lalu memeriksa tiap nama itu ada di
   daftar `BERKAS` `upload_to_pi.bat`. Lain kali ada berkas pendukung baru
   dan lupa dimasukkan, yang gagal adalah tes di laptop — bukan seluruh
   berkas tes di Pi.

`sidik_imu.py` dan `flash_teensy.py` sekarang ada di daftar `BERKAS`.

Di Pi (tanpa `.bat`) penjaga bagian 55 melewati dirinya sendiri dan bilang
begitu — ia memang cuma bisa bekerja di sisi laptop.

## Susur dinding tidak jalan — HUD-nya yang menolak, bukan firmware

`f`/`F`/`p`/`P` ada di daftar `TERLARANG` whitelist HUD. Jadi setiap kali
Vincent menekannya lewat web, yang terjadi adalah **HUD menolaknya sendiri** —
firmware tidak pernah ditanya. Dari luar itu terbaca "fiturnya tidak jalan".

Alasan lamanya, *"robot langsung berjalan"*, **tidak konsisten**: `w` juga
membuat robot berjalan, dan `w` selalu boleh di mode KENDALI. Menjaga satu
tapi tidak yang lain bukan kehati-hatian, itu ketidaksengajaan.

Sekarang keempatnya diperlakukan seperti `w`: **butuh MODE KENDALI, punya
tombol sendiri, dan dikonfirmasi sebelum dikirim.**

- `f` / `F` — susur dinding kiri/kanan, murni sensor sisi.
- `p` / `P` — sama, **plus kunci kompas arena**: butuh IMU hidup dan kompas
  lengkap. Selama IMU mati, pakai `f`/`F`.

### Dan `C` juga — ini perlu kuluruskan

Kemarin kubilang `C` ditolak karena `Navigation.cpp:460` memeriksa
`_imu.hasData()`. Itu **benar tapi bukan yang kamu lihat**: `C` juga ada di
`TERLARANG`, jadi **HUD menolaknya lebih dulu** dan firmware tidak pernah
sampai memeriksa apa pun. Pesan "Gagal: Tidak ada data IMU." yang seharusnya
muncul itu tidak pernah terlihat.

Bahwa `C` memblokir loop firmware ~10 detik itu alasan **memperingatkan**,
bukan **melarang**. Sekarang ada tombolnya dengan konfirmasi yang menyebutkan
log akan berhenti sebentar — dan penolakan yang muncul akan datang dari
firmware, dengan sebab yang sebenarnya.

Yang **tetap** terlarang, karena alasannya berbeda dan masih berlaku:
`S` `W` `e` (menulis EEPROM), `B` `z` (pajangan), `x` (LEMAS — robot ambruk).
Semuanya masih bisa lewat tombol **Kirim PAKSA**.

## Banjir serial: `s` tidak menghentikan CETAKAN

`s` menghentikan **gerak** — `misi.batal`, `nav.navBerhenti`, `robot.stop`.
Ia tidak menyentuh aliran cetak sama sekali.

Yang membanjiri log itu dua toggle firmware, dan **satu-satunya yang
mematikannya huruf yang sama lagi**:

| perintah | aliran | dimatikan oleh |
|---|---|---|
| `y` | yaw, tiap 50–5000 ms | `y` lagi |
| `L` | tabel LiDAR | `L` lagi |

Itulah kenapa log terus membanjir walau robotnya sudah diam. Ditambah poll HUD
sendiri (`m` + `l` tiap detik), log 200 baris bisa habis dalam hitungan detik.

Tiga perbaikan:

**1. Tombol HENING.** Mematikan semua aliran firmware yang sedang hidup —
dan **bukan toggle buta**. HUD membaca keadaan aliran dari kalimat firmware
sendiri (`"Aliran yaw HIDUP"` / `"Aliran yaw berhenti"`), lalu mengirim huruf
pematiannya **hanya** untuk yang terbaca hidup. Mengirim `y` ke aliran yang
sudah mati justru **menyalakannya** — kebalikan dari yang diminta, dan itu
jenis tombol yang membuat orang berhenti memercayai tombol. Keadaannya dibaca
dari firmware, bukan ditebak dari perintah kita, karena Vincent bisa mengetik
`y` langsung lewat serial dan tebakan kita akan salah tanpa ada yang tahu.

**2. Tombol Poll HUD.** Mematikan `m`+`l` tiap detik. Kartu jarak dan status
akan **membeku** di angka terakhir selama mati — itu disebutkan di tooltip-nya,
karena membeku diam-diam lebih buruk daripada log yang ramai.

**3. Kartu "Riwayat perintah" — ini inti masalahnya.** Perintah yang kamu
kirim beserta jawabannya, disimpan **terpisah** dari log serial dan **tidak
bisa tenggelam**. Log utama memuat 200 baris; satu aliran yaw pada 100 ms
menghabiskannya dalam 20 detik, jadi percobaan tiga menit lalu sudah hilang
persis saat Vincent ingin membandingkannya dengan percobaan berikutnya. Itu
bukan log yang penuh — itu riwayat yang hilang.

Rinciannya: jawaban dikumpulkan dalam jendela 2,5 detik sesudah perintah,
baris aliran rutin **tidak pernah** masuk (kalau tidak, riwayatnya ikut penuh
oleh hal yang sama dan perbaikannya sia-sia), dan tiap perintah dibatasi 12
baris jawaban supaya satu perintah cerewet seperti `d` tidak menghabiskan
seluruh riwayat. Tesnya membanjiri log dengan 500 baris lalu memastikan
riwayatnya utuh.

## Urutan kerja flash Teensy dari Pi

`siapkan_teensy_pi.sh` sekarang **ikut di `upload_to_pi.bat`**, jadi tidak
perlu scp manual. Ia tertinggal karena penjaga bagian 55 hanya memeriksa
berkas yang **dimuat tes** — `sidik_imu.py` tertangkap, `siapkan_teensy_pi.sh`
tidak, karena tidak ada tes yang memuatnya. Penjaganya sekarang diperluas:
**tiap `.py`/`.sh` di folder proyek** harus ada di daftar `BERKAS`, kecuali
yang memang milik laptop saja (`upload_to_pi.sh`, `detect.py`) — dan
pengecualiannya ditulis dengan sebabnya, bukan didiamkan.

### Urutannya

```
1. upload_to_pi.bat                     (laptop -> ~/M, sekali tiap ada perubahan)
2. ssh bima@terra-core "bash ~/M/siapkan_teensy_pi.sh"      (SEKALI SEUMUR Pi)
3. kirim_teensy.bat                     (laptop -> ~/teensy, tiap versi baru)
4. ssh bima@terra-core "python3 ~/M/flash_teensy.py --coba --terbaru"   (PERIKSA)
5. ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"       (FLASH)
```

Langkah 1 dan 2 sekali saja. Sesudah itu tiap versi baru dari Vincent cuma
**3 → 4 → 5**.

Dua langkah yang tidak boleh dilompati:

- **Langkah 2 sebelum 5.** `flash_teensy.py` butuh `arduino-cli`, core Teensy,
  dan `teensy_loader_cli`. Tanpa itu ia berhenti dengan pesan, bukan merusak
  apa pun — tapi tetap membuang waktu.
- **Langkah 4 sebelum 5.** `--coba` mengekstrak zip, memeriksa nama folder vs
  `.ino`, dan memindai `#include` — semuanya **tanpa** menyentuh Teensy. Di
  situ juga terlihat versi mana yang akan dipakai, dan peringatan kalau urutan
  nomor versi dan urutan waktu tidak sepakat (masalah v1.61).

### Kalau tetap mau scp manual

```
scp siapkan_teensy_pi.sh bima@terra-core:~/M/
ssh bima@terra-core "bash ~/M/siapkan_teensy_pi.sh"
```

Jalankan dari folder `moses\` di CMD. Tidak perlu `chmod +x` karena dipanggil
lewat `bash <berkas>`, bukan dieksekusi langsung.

## `arduino-cli` tidak ada — dan jebakan berikutnya yang hampir kena

```
Compile (teensy:avr:teensy41) -- ini beberapa menit di Pi...
perintah tidak ada: arduino-cli
!! COMPILE GAGAL -- Teensy TIDAK disentuh sama sekali.
```

Langkah 2 (`siapkan_teensy_pi.sh`) belum pernah dijalankan — wajar, berkasnya
baru saja masuk daftar upload. Tapi kegagalannya memperlihatkan **dua cacat
rancangan di `flash_teensy.py`**, dan keduanya sudah diperbaiki.

### 1. Alat diperiksa di DEPAN, bukan saat dipakai

Versi sebelumnya baru tahu `arduino-cli` tidak ada **setelah** mengekstrak
zip, mencocokkan nama folder dengan `.ino`, dan memindai seluruh `#include` —
lalu berhenti dengan **"COMPILE GAGAL"**, kalimat yang terdengar seperti
kodenya yang salah. Padahal yang kurang cuma satu pemasangan, dan itu sudah
bisa diketahui di detik pertama.

Sekarang pemeriksaannya paling depan, dan menyebutkan perintah perbaikannya:

```
!! Alatnya belum lengkap: arduino-cli, teensy_loader_cli
   Jalankan SEKALI di Pi ini:
     bash ~/M/siapkan_teensy_pi.sh
   Teensy TIDAK disentuh sama sekali.
```

Yang diperiksa ikut mode: `--coba` dan `--hanya-compile` tidak menuntut
`teensy_loader_cli`, karena keduanya memang tidak menyentuh Teensy.

Baris `[?] ... (arduino-cli belum bisa ditanya)` juga hilang. Tanda tanya itu
terlalu lunak — ia membuat keadaan "tidak bisa diperiksa" terlihat seperti
"mungkin tidak apa-apa", lalu skripnya melaju ke compile yang pasti gagal.

### 2. Jebakan yang hampir kena berikutnya: `ssh` non-interaktif

Ini yang akan menggigit **setelah** kamu menjalankan `siapkan_teensy_pi.sh`,
dan gejalanya membingungkan: pemasangan sukses, `arduino-cli` jelas ada di
`~/bin`, tapi

```
ssh pi "python3 ~/M/flash_teensy.py --terbaru"
```

tetap bilang tidak ada. Sebabnya: `siapkan_teensy_pi.sh` menambahkan `~/bin`
ke PATH lewat `~/.bashrc`, tapi `ssh host "perintah"` itu shell
**non-interaktif**, dan `.bashrc` bawaan Raspberry Pi OS keluar di baris
pertamanya justru untuk shell non-interaktif:

```bash
case $- in *i*) ;; *) return;; esac
```

Jadi baris `export PATH` itu **tidak pernah dijalankan**. Berhasil kalau kamu
`ssh` lalu mengetik perintahnya; gagal kalau satu baris — dan tidak ada satu
pun pesan yang menunjuk ke sebabnya.

Perbaikannya: `flash_teensy.py` **tidak lagi bergantung pada shell**. Ia
memasukkan sendiri `~/bin`, `~/.local/bin`, `/usr/local/bin` ke PATH
prosesnya. Diuji dengan meniru keadaan itu (`env -i PATH=/usr/bin:/bin`) dan
alatnya tetap ketemu. Kalau `arduino-cli` ada di `~/bin` tapi tetap tidak
bisa dipakai, skrip menyebut itu secara khusus dan menyuruh memeriksa izin
berkasnya — bukan mengulang saran memasang ulang.

`siapkan_teensy_pi.sh` juga menguji sendiri lewat `ssh localhost` di akhir,
supaya keadaannya ketahuan saat memasang, bukan saat mau flash.

### Jadi sekarang

```
1. upload_to_pi.bat                                     (kirim ulang)
2. ssh bima@terra-core "bash ~/M/siapkan_teensy_pi.sh"  (SEKALI, 10-20 menit)
3. ssh bima@terra-core "python3 ~/M/flash_teensy.py --coba --terbaru"
4. ssh bima@terra-core -t "python3 ~/M/flash_teensy.py --terbaru"
```

**Kalau kamu SUDAH di Pi** (prompt `bima@terra-core:~ $`) — tanpa `ssh`, dan
**tanpa `-t`**:

```
bash ~/M/siapkan_teensy_pi.sh                            (SEKALI)
python3 ~/M/flash_teensy.py --coba --terbaru
python3 ~/M/flash_teensy.py --hanya-compile --terbaru
python3 ~/M/flash_teensy.py --terbaru
```

`-t` itu **pilihan `ssh`**, bukan perintah shell. Menempelkannya di depan saat
sudah di Pi menghasilkan `-bash: -t: command not found`. Itu kesalahan
petunjukku, bukan kesalahan pemakainya — jadi sekarang `--coba` mencetak kedua
bentuknya di akhir keluarannya, di tempat ia dibutuhkan.

Langkah 3 sekarang benar-benar berarti: kalau alatnya belum lengkap ia bilang
di baris kedua, bukan setelah setengah menit bekerja.

## "Flash sukses tapi Teensy tidak nyambung ke mission HUD"

Dua pernyataan yang terasa bertentangan, padahal **keduanya bisa benar
sekaligus** — dan itu kekurangan di `flash_teensy.py`, bukan kebetulan.

### `dmesg | grep serial` tidak bisa menjawab ini

Keluaran yang kamu tempel hanya berisi UART **internal** Pi (`ttyAMA10`,
`ttyS0`) dan Bluetooth. Itu bukan bukti Teensy tidak ada, karena baris yang
muncul saat Teensy dicolok berbentuk:

```
usb 1-1.2: new full-speed USB device number 5 using xhci_hcd
cdc_acm 1-1.2:1.2: ttyACM0: USB ACM device
usbcore: registered new interface driver cdc_acm
```

**Tidak satu pun mengandung kata "serial".** Jadi `grep serial` menyaringnya
habis — hasilnya kosong apa pun keadaannya. Yang benar:

```
ls /dev/ttyACM*
lsusb
dmesg | grep -iE 'acm|teensy|usb 1-'
.venv/bin/python ~/M/cek_serial.py        # enam pemeriksaan berurutan
```

`cek_serial.py` sudah ada di `~/M` dan memakai `cari_port()` yang **sama**
dengan HUD, jadi yang diuji benar-benar jalur yang dipakai HUD.

### Tersangka utama: Teensy tertinggal di mode BOOTLOADER

Sesudah `teensy_loader_cli` menulis sketsa, Teensy harus **reboot** ke sketsa
itu. Kalau ia tertinggal di HalfKay (bootloader), ia muncul sebagai perangkat
**HID — bukan serial**. Jadi `/dev/ttyACM*` tidak ada sama sekali, dan HUD
jatuh ke SIMULASI. Menulis berhasil; menjalankan tidak.

Yang biasanya menyelesaikan, berurutan:

1. Tekan tombol **PROGRAM** di Teensy sekali — itu me-reboot-nya.
2. Cabut-colok kabel USB Teensy ke Pi.
3. `ls /dev/ttyACM*` lalu `lsusb`.
4. `.venv/bin/python ~/M/cek_serial.py`.

### Dan ini cacat di skripku, yang sudah diperbaiki

Versi sebelumnya mencetak **"FLASH SELESAI"**, menyalakan HUD lagi, lalu
pulang — tanpa pernah memeriksa apakah Teensy benar-benar kembali. Kalimat itu
menyatakan lebih banyak daripada yang diketahuinya: yang selesai itu
**menulis**, bukan **berjalan**.

Sekarang sesudah menulis, skrip:

1. **Menunggu port serial kembali** (sampai 20 detik). Node yang dicari yang
   **baru** — daftar port dicatat sebelum flash — supaya perangkat lain tidak
   disalahartikan sebagai Teensy yang sudah kembali.
2. **Membuktikan firmware benar-benar jalan**, bukan cuma node-nya ada. Node
   bisa muncul dari bootloader atau dari sketsa yang berhenti di `setup()`.
   Jadi skrip menunggu sapaan `"Memulai Hexapod Unlimited..."`, dan — karena
   firmware hanya menunggu USB serial 3 detik sehingga sapaannya bisa sudah
   lewat saat kita membuka port — mengirim `m` sebagai cadangan. Apa pun yang
   menjawab berarti firmware hidup dan mendengarkan. `m` hanya **membaca**.
3. Kalau port tidak kembali, **mengatakannya** dengan empat langkah di atas,
   termasuk catatan soal `grep serial` yang menyesatkan.

Kalimatnya juga diubah dari "FLASH SELESAI" jadi **"FLASH SELESAI menulis"**,
lalu baris terpisah untuk hasil pemeriksaannya. Bedanya kecil di layar, besar
artinya.

Logikanya bisa diuji tanpa Teensy dan tanpa `arduino-cli`:

```
python3 ~/M/flash_teensy.py --uji        # 15 tes
```

Yang diuji bukan pyserial, melainkan **keputusan** skrip: kapan ia menyatakan
firmware hidup, kapan diam, dan kapan ia jujur bilang tidak bisa memeriksa.
Ketiganya pernah tercampur jadi satu kalimat "flash sukses".

## Terbukti: Teensy tertinggal di HalfKay — dan pemeriksaan barunya menangkapnya

Buktinya lengkap di `lsusb`:

```
Bus 003 Device 006: ID 16c0:0478 Van Ooijen Technische Informatica
                                 Teensy Halfkay Bootloader
```

Bukan "Teensyduino Serial" — **"Halfkay Bootloader"**. Di mode itu Teensy
perangkat HID, jadi `/dev/ttyACM*` tidak ada, `comports()` cuma melihat
`ttyAMA10` (UART internal Pi), dan HUD jatuh ke SIMULASI. `cek_serial.py`
mengatakannya dengan benar di keenam langkahnya.

Perbandingan dua kali flash di berkas log yang sama memperlihatkan perbaikannya
bekerja:

| | versi lama | versi baru |
|---|---|---|
| sesudah menulis | `FLASH SELESAI` | `FLASH SELESAI menulis.` |
| pemeriksaan | tidak ada | `port kembali: /dev/ttyACM0` |
| bukti firmware | tidak ada | `FIRMWARE HIDUP -- sapaan firmware terbaca` |

Versi lama menyalakan HUD lagi dan melaporkan sukses di atas papan yang
sedang tertinggal di bootloader. Versi baru mengatakan apa yang benar-benar
terjadi.

### Dan sekarang ia memulihkan sendiri

Kalau port tidak kembali, skrip menjalankan

```
teensy_loader_cli --mcu=TEENSY41 -b
```

`-b` menyuruh HalfKay **boot** tanpa memprogram apa pun — persis yang
dibutuhkan kalau sketsanya sudah ditulis tapi papannya tertinggal di
bootloader. Lalu ia menunggu lagi 12 detik dan melaporkan `PULIH:` kalau
berhasil.

Ini bukan kenyamanan. Tanpa ini satu-satunya jalan keluar adalah menekan
tombol fisik di Teensy — dan seluruh gunanya flash-dari-Pi justru supaya tidak
perlu ada orang di sebelah robot. Tombol fisik sebagai satu-satunya pemulihan
meniadakan alasan alat ini ada.

### Kenapa yang pertama tertinggal di HalfKay — aku tidak tahu pasti

Lebih baik kukatakan daripada mengarang. Yang bisa dibaca dari lognya:

- Reboot lunak 134 baud **terkirim**, tapi `teensy_loader_cli` masih mencetak
  `Waiting for Teensy device... (hint: press the reset button)` — jadi
  pemicunya tidak langsung bekerja saat itu.
- Sesudah `Programming...` ia mencetak `Booting`, artinya perintah boot
  **dikirim**. Tapi papannya tetap berakhir di HalfKay.

Tiga kemungkinan yang semuanya masuk akal, tidak bisa kupersempit dari jarak
ini:

1. **Balapan dengan HUD.** Versi lama menyalakan `r2c-hud` segera sesudah
   "Booting", jadi HUD membuka port saat papan masih enumerasi ulang.
2. **Boot yang tidak jadi.** HalfKay menerima perintah boot tapi sketsanya
   tidak sampai `Serial.begin()`.
3. **Daya.** Ini tersangka yang sah khusus di robot ini: servo menarik arus
   besar saat PWM menyala, dan Teensy yang brownout di awal boot bisa jatuh
   kembali ke bootloader. Kartu **Daya & suhu Pi** masih bisa menjawab ini
   sesudah kejadiannya lewat — bit throttle melatch di perangkat keras dan
   menempel sampai reboot.

Yang berubah bukan dugaan itu, melainkan **keterlihatannya**: kejadian yang
sama sekarang dilaporkan, dicoba dipulihkan, dan kalau gagal disebutkan
ketiga tersangka ini beserta cara memeriksanya.

## Korban digendong di pose netral (R) — dan apa artinya untuk vision

Vincent memberi tahu bahwa saat `AKS_AMBIL`, korban berakhir di **posisi
netral (`R`)**. Angkanya dari `config.h` v1.12:

```c
#define REHAT_JANGKAUAN    80.0f   // mm dari pusat badan, ke depan
#define REHAT_TINGGI      160.0f   // mm DI ATAS bidang pusat badan
#define REHAT_PERGELANGAN -40.0f   // capit menunduk supaya tidak menonjol
```

Komentar di atasnya menyebutkan gunanya: *"lengan dilipat ke ATAS badan supaya
tidak menghalangi LiDAR depan dan tidak tersangkut saat menaiki tangga"* —
disetel di robot 11 Sep, bukan dihitung.

Yang **tidak** disebut di situ: **kamera.**

### Masalahnya, dan kenapa gerbang kelas tidak menolong

Korban yang digendong **terdeteksi sebagai korban** — ia memang korban. Jadi
`jejak_hanya_korban`, yang kita pasang supaya dummy tidak dikejar, di sini
justru meloloskannya: kelasnya benar, ukurannya besar (dekat kamera),
keyakinannya tinggi. Ia akan menang sebagai sasaran atas korban sungguhan
yang masih di lantai beberapa puluh sentimeter jauhnya.

Akibatnya JEJAK/CENTER menengahkan badan ke barang yang **sudah ada di
tangannya sendiri**, lalu rantai AMBIL maju ke arahnya. Robot mengejar
muatannya — dan tiap langkahnya terlihat benar dari dalam.

### Dua penjaga, dan yang utama tidak bergantung geometri

**1. `tolak_saat_membawa` (bawaan NYALA).** Firmware sudah mengumumkan isi
capitnya di baris status (`Misi.cpp:1710`):

```
  membawa     : depan KORBAN, belakang kosong
```

`RE_KV` HUD sudah menangkap baris itu sejak dulu — tidak perlu parser baru,
cuma perlu **dibaca**. Selagi salah satu capit memegang korban, vision tidak
mengejar apa pun dan mengatakan sebabnya sekali (bukan tiap frame).

Keadaannya datang dari **firmware**, bukan dari catatan HUD sendiri. Itu
disengaja: HUD bisa di-restart, halaman bisa dimuat ulang, dan misi bisa
dijalankan dari Teensy tanpa HUD tahu apa-apa. Firmware yang selalu tahu isi
capitnya.

**2. `roi_atas_frac` (bawaan 0 = belum diukur).** Pasangan dari
`roi_bawah_frac`, tapi untuk pita **atas** — karena di pose `R` korban duduk
tinggi, bukan rendah. Deteksi yang pusatnya di pita itu dibuang dengan alasan
`"di pita atas -- korban yang digendong"`.

Bawaannya **0**, bukan angka tebakan. Aku tidak tahu di mana kamera dipasang
relatif ke lengan yang terlipat, dan menebaknya berarti bisa membuang sasaran
yang sah di bagian atas frame. Cara mengukurnya: gendong korban, lihat stream,
catat di ketinggian berapa ia mulai terlihat, isi fraksinya.

Penjaga 1 bekerja tanpa pengukuran apa pun, jadi ia yang menanggung keamanan;
penjaga 2 menambah ketelitian kalau suatu saat kamu perlu vision tetap hidup
sambil menggendong.

Baris **capit membawa** di kartu Ambil korban menunjukkan keadaannya, kuning
kalau terisi.

### Yang masih perlu diperiksa dengan mata

Apakah korban di pose `R` benar-benar masuk frame kamera — dan kalau ya,
seberapa banyak. 80 mm ke depan dan 160 mm ke atas itu **dekat dan tinggi**,
jadi dugaanku ya, tapi itu dugaan: posisi dan sudut kamera tidak ada di
`config.h`. Satu kali gendong + lihat stream menjawabnya, dan sekaligus
memberi angka untuk `roi_atas_frac`.

## Kenapa vision cuma mendeteksi lalu capit turun sendiri

Gejala di arena 12 Sep: masuk area 1, pemicu vision jalan, vision **mendeteksi**
tapi **tidak menggerakkan robot**, lalu **capit turun sendiri**.

**Triggernya tidak hilang. Justru sampai.** Yang terjadi itu persis perilaku
yang dirancang v1.12, dan masing-masing separuhnya benar:

1. `#KORBAN AMBIL 1` terkirim → HUD membacanya.
2. `pemicu_korban` bawaannya **`lihat`** → vision menilai, **nol** perintah
   gerak. Itu sebabnya robot tidak bergerak.
3. v1.12 **tidak menunggu** — sebaris sesudah mengirim pemicu ia
   `masuk(MISI_LENGAN)` dan sekuens capit mulai. Itu capit yang turun sendiri.

Dan menaikkan `pemicu_korban` ke `ambil_alih` **tidak akan menolong**, karena
HUD mencari **bukti** firmware sedang parkir dan tidak menemukannya — lalu
turun sendiri ke `lihat` dan menulis sebabnya. Itu bukan HUD yang rewel:
menggerakkan badan selagi sekuens lengan berjalan berarti dua penguasa untuk
satu robot, dan yang kalah korban yang tersenggol capit yang sedang turun.

Jadi yang kurang ada **di firmware**, bukan di Python. Karena itu kali ini
patch-nya di sana.

## Patch firmware: `Hexapod_Unlimited_v1.12_R2C.zip`

Dua perubahan atas v1.12 Vincent. Rinciannya di `PATCH_R2C.md` di dalam zip.

**1. `sekuensAmbil` fase 4 → pose netral `R`** (yang diminta). Satu jebakan
kerangka acuan yang hampir kena: `REHAT_TINGGI` diukur dari **bidang pusat
badan**, sedangkan fase 0–3 memakai tinggi dari **lantai** (lewat `lt`). Jadi
fase 4 **tanpa `lt`** — menambahkannya menggeser pose sebesar tinggi badan
(100/115 mm) dan melempar lengan ke luar jangkauan, dan korban tertinggal
menggantung di pose jepit. Bukan tebakan: `sekuensTaruh` fase 4 milik Vincent
sendiri sudah memakai pola itu.

**2. `m8` — tunggu Raspi sebelum sekuens AMBIL, bawaan MATI.** Ruas
`AKS_AMBIL` berhenti di `MISI_KONFIRM` **sesudah** mengirim pemicu; Raspi
meluruskan badan lalu menjawab `m2`; **baru** sekuens lengan jalan.

Tabel `RUAS[]` **tidak disentuh** — menyisipkan baris `AKS_KONFIRM` akan
menggeser seluruh indeks sesudahnya, dan bersamanya panjang ruas hasil `m7`,
acuan "ruas 30", dan rentang `m4 0 29`. Satu flag runtime tidak menggeser apa
pun.

Bagian yang paling mudah salah di perubahan ini: `MISI_KONFIRM` sekarang
dipakai **dua** hal dengan ujung berbeda — ruas `AKS_KONFIRM` (→ ruas
berikutnya) dan parkir vision (→ `MISI_LENGAN`). `_parkirVision`
membedakannya. Tanpa pembeda itu `m2` akan **melompati pengambilannya** dan
robot berjalan dengan capit kosong tanpa satu pun pesan. Logikanya
ditransliterasi ke Python dan diuji 13 kasus, termasuk kasus capit-kosong itu.

### Urutan memakainya

```
m8                          (atau tombol "Balik tunggu-vision (m8)" di HUD)
pemicu_korban = ambil_alih  (tab Kalibrasi)
m1  atau  m4 0 29
```

Baris **tunggu vision** di kartu Ambil korban menunjukkan keadaannya, dibaca
dari kalimat firmware sendiri — bukan ditebak dari perintah yang HUD kirim,
karena Vincent bisa mengetik `m8` langsung lewat serial.

### Belum dikompilasi

Tidak ada `arduino-cli` + core Teensy di tempat patch ini dibuat. Yang sudah
diperiksa: keseimbangan kurung **identik** dengan aslinya (selisih `(` vs `)`
di `Misi.cpp` memang −9 sebelum **dan** sesudah — warisan komentar Vincent),
dan logika `m2` lewat 13 tes. Compile dulu di Pi sebelum flash:

```
python3 ~/M/flash_teensy.py --hanya-compile --terbaru
```

Itu berhenti sebelum menyentuh Teensy.

## Ya, Pi bisa memproses zip langsung — dan sudah terbukti

Dari keluaran `--coba` di Pi:

```
   2. Hexapod_Unlimited_v1.12_R2C.zip         v1.12  2026-09-12 09:25  [zip]
-> Hexapod_Unlimited_v1.12_R2C.zip  v1.12
   sketsa: /tmp/flashteensy-iisfs3p8/ekstrak/Hexapod_Unlimited
```

Zip-nya diekstrak ke folder sementara, `.ino`-nya ditemukan, dan kedua pustaka
terdeteksi. Tidak perlu membuka zip-nya sendiri. Dukungan zip ini memang dibuat
karena v1.12 pertama kali datang sebagai zip dan tidak pernah diekstrak —
melewatkannya berarti versi terbaru tidak kelihatan sama sekali.

Folder sementaranya dibuang sendiri sesudah selesai; **file Vincent tidak
pernah disentuh**.

### Satu hal baru yang muncul dari situ: nomor versi KEMBAR

Sekarang ada dua entri yang sama-sama terurai jadi `v1.12` — folder aslinya dan
zip patch-nya. Di situ nomor versi **berhenti jadi pembeda**, dan yang
membedakan tinggal namanya. Itu persis jenis kebingungan yang dulu membuat
v1.61 ter-flash padahal yang diminta v1.8, jadi sekarang dikatakan:

```
  ! Nomor versi KEMBAR: v1.12.
    Nomornya tidak bisa membedakan keduanya -- BACA NAMANYA.
    Yang dipilih '--terbaru' = baris paling bawah (berkas termuda).
```

`--terbaru` memilih yang berkasnya termuda, yaitu zip R2C — yang memang kamu
mau. Tapi sekarang itu terlihat, bukan perlu dipercaya.

Kalau mau memilih dengan pasti, sebut namanya:

```
python3 ~/M/flash_teensy.py --versi R2C --hanya-compile    # patch
python3 ~/M/flash_teensy.py --versi v1.12 --coba           # hati-hati: cocok KEDUANYA
```

`--versi` mencocokkan teks yang **mengandung**. `R2C` cocok tepat satu, jadi ia
pembeda yang pasti. `v1.12` cocok **keduanya** dan diselesaikan oleh waktu —
dan kalau itu terjadi, skrip menyebutkan apa saja yang cocok:

```
  ! 'v1.12' cocok dengan 2 entri:
      Hexapod_Unlimited_v1.12
      Hexapod_Unlimited_v1.12_R2C.zip
    Diambil yang TERMUDA: Hexapod_Unlimited_v1.12_R2C.zip
    Sebut teks yang lebih khas kalau bukan itu yang kamu mau.
```

Hasilnya kebetulan yang kamu mau, tapi itu **pilihan, bukan kepastian** — jadi
sekarang terlihat sebelum Teensy disentuh, bukan sesudah.

## Kenapa capit tidak menunggu, dan kenapa tidak naik ke R — patch belum di-flash

Kedua hal yang kamu lihat itu **memang belum ada di chip**. Yang terpasang masih
v1.12 asli: terakhir kali kita sampai `--coba`, dan `--hanya-compile` belum
dijalankan. Jadi:

- **Capit tidak menunggu** → `m8` belum ada di firmware yang jalan.
- **Capit tidak naik ke R** → fase 4 masih GENDONG di firmware yang jalan.

Dan pengamatanmu soal kotak abu-abu → hijau/merah → abu-abu lagi dalam 1–2
detik itu **tepat**, serta menjelaskan dirinya sendiri: vision menyala di state
`AWAS_KORBAN`, yang keluar begitu nama ruas firmware tidak lagi mengandung
"ANGKAT KORBAN". Firmware melewati ruas itu dalam ~1,5 detik, jadi jendelanya
memang sependek itu. Dengan patch, firmware **parkir** di ruas yang sama, jadi
namanya tidak berganti dan vision tetap hidup.

### Tapi ada jebakan di patch-ku yang kuperbaiki sebelum kamu flash

`m8` bawaannya kubuat **MATI** — dengan alasan masuk akal waktu itu: parkir yang
tidak dijawab akan menggantung sampai jam kontes habis. Akibatnya kamu akan
flash patch itu, **dan tetap melihat gejala yang sama**, karena tidak ada yang
menyalakan `m8`.

Jadi sekarang parkirnya punya **batas waktu 40 detik**, dan habisnya waktu
**tidak menggagalkan apa pun**: sekuens lengan jalan memakai posisi meja yang
sudah diukur — persis perilaku tanpa `m8`. Itu yang membuat bawaan **NYALA**
aman: `m8` NYALA berarti "coba pakai vision; kalau tidak ada jawaban, kerjakan
cara lama", dan itu tidak pernah lebih buruk daripada MATI.

40 detik **diturunkan dari batas HUD**, bukan dipilih enak. HUD memberi
`AMBIL_TENGAH` 30 detik. Kalau keduanya sama, keduanya habis pada detik yang
sama dan yang terjadi jadi lomba — capit turun tanpa HUD sempat mengatakan apa
pun. Dengan 40, HUD **selalu bicara lebih dulu**, dan capit baru turun 10 detik
kemudian. Tesnya memeriksa urutan itu.

`pemicu_korban` juga kuubah bawaannya ke **`ambil_alih`**. Sebelumnya `lihat`,
dan itu benar selama firmware tidak pernah menunggu — tapi pasangannya harus
ikut berubah, kalau tidak parkir firmware tidak akan pernah dijawab. Penjaga
buktinya **tetap** berlaku: kalau firmware yang terpasang belum punya parkir,
`ambil_alih` turun sendiri ke `lihat` dan menyebut sebabnya. Jadi bawaan ini
tidak bisa membuat robot bergerak di saat yang salah, apa pun firmware yang
terpasang.

## Capit menutup ke 10%, bukan 0

Laporan: pada `g0` capit menabrak dirinya sendiri dan motornya panas. Diperbaiki
di **dua** sisi, dan yang penting sisi firmware:

`GRIP_PERSEN_MIN` 5 → **10** di `config.h`. Angka itu dipakai dua tempat:

1. `sekuensAmbil` fase 2 menutup ke `GRIP_PERSEN_MIN` → urutan otomatis
   terlindungi.
2. perintah `g` di-clamp ke `GRIP_PERSEN_MIN..MAKS` → **`g0` yang diketik
   manual pun berhenti di 10%**.

Memperbaiki hanya di HUD akan meninggalkan jalur kedua terbuka — dan jalur
kedua itu justru yang dipakai saat trial-and-error. HUD juga ikut: `capit_tutup`
dan rantai AMBIL sekarang mengirim `g10`, dan tesnya memastikan `g0` tidak
pernah lagi dikirim dari mana pun.

## Saklar daya kamera — kartu paling atas di tab Misi

Mematikan di situ **benar-benar melepas perangkat kameranya**
(`thread.release()`), bukan cuma berhenti menggambar kotak. Itu bedanya dengan
**mode FOKUS**, yang berhenti mengirim video tapi kamera dan inferensi tetap
jalan — FOKUS untuk menghemat CPU, saklar ini untuk menghemat **daya**.

Alasannya nyata di robot ini: C922 menarik daya terus-menerus selama terbuka,
dan tegangan 4,9 V sudah cukup membuat USB macet dan bit throttle Pi melatch.

Selagi mati, `pastikan()` menolak menyambung dan menyebutkan sebabnya, `read()`
tidak mengembalikan frame, jadi vision ikut berhenti sendiri. Menyalakan lagi
me-nolkan jeda coba-sambung supaya tidak menunggu 3 detik sia-sia.

Yang dituju akhirnya — kamera menyala **hanya saat Teensy memintanya** lewat
pemicu `#KORBAN`, lalu mati sendiri sesudah korban diambil — belum dipasang,
karena monitoring masih dibutuhkan. Pemicunya sudah terbaca HUD, jadi
menyambungkannya nanti tinggal satu baris di `tangani_pemicu()`.

## "Stuck di vision, cuma gerak sedikit" — bukan timer 40 detik

40 detik itu **tenggat, bukan jadwal**. Kalau ia yang mengakhiri, artinya ada
yang salah — bukan ada yang lambat. Dan memang ada dua yang salah, keduanya di
sisi Raspi:

### 1. Toleransinya menuntut yang tidak mungkin

`tengah_tol_px` = **8 px** = **0,44°**. Derau bbox frame-ke-frame sendiri lebih
besar dari itu: satu piksel bergeser di bbox 340 px sudah menggeser pusatnya.
Jadi syaratnya tidak pernah terpenuhi — robot melangkah kecil, mengukur, masih
di luar, melangkah lagi. Selamanya. Itu "gerak sedikit" yang kamu lihat.

Sekarang **20 px = 1,1°**, seperti usulmu. Cukup ketat untuk capit, dan bisa
diendapkan.

### 2. `O` tidak menggerakkan apa pun, dan HUD terus mengirimnya

Galat di atas `yaw_kasar_deg` (8°) dikirim ke **pivot gait `O`** — yang
**butuh IMU**. IMU mati, `nav.pivotRelatif()` kembali diam-diam, galatnya tetap
sama, lalu `O` dikirim lagi. **Selamanya.** Yang benar-benar bergerak cuma
`r0 0 0` yang menetralkan badan sebelum tiap percobaan.

Sekarang HUD **mengukur** apakah `O` bekerja: kalau dua kali berturut-turut
galatnya tidak berubah sesudah `O`, ia berhenti memakainya dan mengatakannya.
Sesudah itu penengahan hanya memakai putar badan (±18°), dan kalau sasarannya
di luar itu ia **GAGAL dengan sebab**:

> sasaran +25,0 der, di luar jangkauan putar badan (±18). Pivot gait 'O' tidak
> menggerakkan apa pun — hampir pasti IMU mati, jadi tidak ada cara melangkah
> memutar. Putar robot dengan tangan sampai korban kira-kira di depan, lalu
> ulangi.

Operator bisa memutar robot dengan tangan; HUD tidak bisa, dan berpura-pura
bisa cuma membuang jam kontes. Baris **pivot gait** di kartu Ambil korban
menunjukkan keadaannya.

## Pemicu serah-terima datang dari Pi, bukan dari jam — persis usulmu

Syarat tengah sekarang harus **BERTAHAN 2 detik tanpa putus**
(`tengah_tahan_s`). Satu frame yang kebetulan di tengah bukan bukti robot
**diam** di tengah — bisa jadi ia sedang melewatinya. Putus sekali saja
menolkan hitungannya.

Begitu syaratnya bertahan penuh, Pi:

1. menetralkan pose badan (`r0 0 0`, `t0 0 0`),
2. mengirim **`m2`**,
3. dan firmware mengambil korban → mengangkat ke **pose R** → lanjut ke ruas
   berikutnya.

Siklusnya berulang sendiri di korban berikutnya, karena pemicu `#KORBAN` datang
lagi dari firmware.

Baris **tengah bertahan** menunjukkan hitungannya berjalan (`1,2 / 2,0 s`), jadi
"sedang menunggu" bisa dibedakan dari "macet".

### Dan kamera ikut siklusnya

Knob `kamera_ikut_pemicu` (bawaan **MATI**, karena monitoring masih kamu
butuhkan):

```
pemicu #KORBAN tiba      -> kamera dinyalakan
dwell penuh, m2 dikirim  -> kamera dimatikan
```

Jadi kamera hidup hanya selama jendela yang membutuhkannya — beberapa detik per
korban alih-alih sepanjang misi. Ini yang kamu minta soal hemat daya; nyalakan
knob-nya di tab Kalibrasi kalau daya jadi masalah di arena.

Selain hemat daya, mematikan vision sesudah serah-terima juga menutup risiko
lain: korban yang sedang digendong terdeteksi sebagai korban, dan kamera yang
masih hidup bisa menjadikannya sasaran baru.

### Catatan tentang 40 detik

Batas itu tetap ada dan tetap berguna — ia yang mencegah misi menggantung kalau
Pi benar-benar tidak bisa menengahkan. Tapi dengan dwell, penengahan yang sehat
selesai dalam beberapa detik, jadi 40 detik seharusnya tidak pernah tercapai.
Kalau ia tercapai, itu tanda: periksa baris **pivot gait** dan **tengah
bertahan** di HUD — salah satunya akan menjelaskan kenapa.

## Revert 12 September: metode vision kembali ke versi 10 September

Laporanmu: *"mode jejak sekarang tidak bisa mengikuti korban dengan kotak abu-abu
bertuliskan disaring, padahal vision versi 10 september sudah optimal."*

Itu sudah dibandingkan langsung dengan `mission_hud_10September.py` di folder
`moses`, bukan dikira-kira. Hasil bandingannya pendek dan jelas:

**Gerbang saringnya TIDAK berubah sama sekali.** `Juri.saring()` identik, dan
semua knob gerbang dari 10 September — `bbox_h_min/max`, `jejak_h_min/max`,
`roi_half`, `cx_offset_px`, `conf_min`, `k_of_n` — masih bernilai sama persis.
Satu-satunya knob lama yang nilainya berubah adalah `tengah_tol_px`
(8 → 20), dan itu permintaanmu sendiri, bukan bagian penglihatan.

**Yang berubah letaknya, bukan isinya.** 12 September tiga gerbang baru ditaruh
di depan pemilihan sasaran di mode JEJAK/A_TENGAH:

1. gerbang muatan — `sedang_membawa()` → `lolos = []`
2. saring kelas sebelum memilih yang terbesar — `calon = [...]`
3. pita ROI atas — `roi_atas_frac` (ini tidak aktif, nilainya masih 0)

### Label "disaring" itu yang menunjuk pelakunya

Ini bisa dibaca dari kodenya tanpa menebak. Label kotak abu-abu diambil di
`anotasi()`:

```python
sebab = (alasan or {}).get((x1, y1), "disaring")
```

`alasan` HANYA diisi oleh `Juri.saring()`. Artinya setiap deteksi yang
**ditolak saring** selalu punya sebab tertulis: "di luar ROI", "terlalu jauh
0.04<0.06", "di pita yang ditutupi capit". Kotak abu-abu yang label sebabnya
cuma kata bawaan **"disaring"** berarti deteksi itu **lolos** saring lalu
dibuang **sesudahnya** — dan satu-satunya baris yang melakukan itu adalah
`lolos = []` di gerbang muatan.

Jadi gejalanya bukan gerbang geometris yang terlalu ketat. Gerbang muatan yang
menyala.

### Kenapa gerbang muatan salah menyala

Ia membaca baris status firmware:

```
  membawa     : depan KORBAN, belakang kosong
```

Baris itu bertahan di `link.terakhir` **sampai status berikutnya datang**.
Sekali tercetak, ia masih berbunyi "membawa" lama sesudah korban diletakkan —
dan selama itu HUD menolak mengejar apa pun. Idenya masih benar (robot memang
tidak boleh mengejar muatannya sendiri), tapi gerbang yang bergantung pada
keadaan basi tidak boleh memegang kendali penglihatan.

### Yang dikembalikan

| | 10 Sep | 12 Sep (rusak) | sekarang |
|---|---|---|---|
| pilih sasaran JEJAK | `max(lolos)` | `max(calon)` sesudah gerbang muatan + saring kelas | `max(lolos)` |
| pilih sasaran CENTER | `max(lolos)` | saring kelas dulu | `max(lolos)` |
| `tolak_saat_membawa` | tidak ada | `True` | **`False`** |
| `vision_silang_on` | tidak ada | `True` | **`False`** |
| `roi_atas_frac` | tidak ada | `0.0` (pasif) | `0.0` (pasif) |
| gerbang kelas | di `sasaran_sah()` | di pemilihan sasaran | di `sasaran_sah()` |

Gerbang kelas **tidak hilang**. Ia kembali ke tempat asalnya: `sasaran_sah()`,
yang menahan **gerakan**. Dummy masih tidak akan pernah dikejar. Bedanya,
kotaknya sekarang tetap terlihat apa adanya, dan korban di sebelahnya tidak
ikut hilang dari penglihatan.

### `vision_silang_on` juga dimatikan — dan kenapa itu penting

Silang-periksa vision vs LiDAR di `S_A_MAJU` memanggil `misi.gagal()` kalau
bedanya lewat 6 cm. Masalahnya, `tinggi_k1..k5` masih **0**: angka kamera itu
taksiran dari tinggi nominal 9 cm, belum pernah dicocokkan ke posisi mana pun.
Menghentikan pendekatan yang sehat karena taksiran adalah persis bagian
"mengacaukan gerakan robot" yang kamu keluhkan.

Sekarang ada dua lapis:

* `vision_silang_on = False` — bawaan, tidak ikut campur sama sekali.
* Kalaupun dinyalakan, selama `tinggi_k1..k5` masih 0 bedanya cuma **dicatat di
  log** (`[SILANG] ... TIDAK dijadikan kegagalan`) dan misi lanjut pakai LiDAR.
  Ia baru boleh menghentikan misi sesudah K1–K5 benar-benar diukur.

### Yang TIDAK dikembalikan, dan alasannya

Ini permintaanmu sendiri hari ini, dan tidak ada hubungannya dengan kotak
abu-abu:

* **`tengah_tol_px = 20` + `tengah_tahan_s = 2`** — pemicu serah-terima dari Pi.
* **Deteksi `O` yang tidak menggerakkan apa pun** — ia menambah gerakan (jatuh
  ke putar badan), bukan mengurangi.
* **Saklar daya kamera** di tab Misi.
* **`g10`** untuk capit tertutup.
* **Pengukuran jarak dari kamera** — tetap jalan, tapi murni bacaan. Rasio bbox
  yang tidak wajar menolkan `jarak_vis`, bukan sasaran.

### Uji yang mengunci ini

Seksi **67** di `test_mission_hud.py` membaca sumbernya sendiri dan memastikan
blok JEJAK tidak lagi memuat `lolos = []`, `sedang_membawa(`, atau
`jejak_hanya_korban`; lalu membuktikan invariannya secara langsung — setiap
deteksi yang ditolak `saring()` punya sebab tertulis, jadi kata "disaring"
tidak akan pernah muncul lagi dari jalur itu. Total **828 uji lulus**.

---

# 13 September 2026 — lup vision ↔ capit ditutup

Bagian ini menggantikan status di seluruh dokumen di atasnya. Kalau ada yang
bertentangan, yang di sini yang berlaku.

## Yang sudah SELESAI dan tidak perlu dikerjakan lagi

| dulu | sekarang |
|---|---|
| **IMU mati** — lima perintah (`C`, `c0..c3`, `O`, `o0..o3`, `m1`/`m4`) tertahan satu gerbang yang sama | **Selesai.** Akarnya HUD yang menolak informasi yang sudah dikirim Teensy — bug sisi kita, bukan baud dan bukan kabel |
| **Pi mati di JEJAK** (brownout transien) | **Aman** |
| **Hantu sensor depan** (340× "kosong" menghadap SELATAN, benda mantap di 8,6 cm menghadap BARAT) | **Aman** |
| **Penengahan macet "gerak sedikit-sedikit"** | **Selesai** — metode vision kembali ke versi 10 September |

Tebakan lama soal `IMU_BAUD` 9600 vs 230400 **meleset**. Bagian itu sudah
diberi penanda, tapi jangan dipakai sebagai petunjuk.

## Trigger operasional

```
m4 0 23        HOME sampai depan tangga
```

23 ruas itu memuat **4 ruas AMBIL** (1, 8, 16, 21) dan **3 ruas TARUH**
(6, 10, 19). Ruas 24 ke atas (tangga R-9 dan seterusnya) belum ikut.

Ukuran korban yang sudah diukur sendiri: **lebar 9 cm × tinggi 9,5 cm.**

## Kenapa robot tidak bergerak sesudah trigger — dan kenapa itu BUKAN konflik serial

Gejala di arena: `m4 0 23`, robot sampai di depan korban, Teensy menembak
Raspi, YOLO menyala, kotak muncul — lalu **robot diam dan capit turun sendiri**.

Triggernya tidak hilang, dan tidak ada informasi yang tertelan di kabel.
Buktinya justru vision yang menyala. Yang terjadi: **HUD menolak bergerak
dengan sengaja**, dan alasannya benar.

```
Misi.cpp:1567   KORBAN_SERIAL.println("#KORBAN AMBIL 1")
Misi.cpp:1585   masuk(MISI_LENGAN)          <-- SEBARIS DI BAWAHNYA
```

v1.12 asli mengirim pemicu lalu **langsung** menurunkan capit — komentar
Vincent sendiri menyebutnya *"Kirim saja, tidak menunggu jawaban."* Ruas AMBIL
tidak pernah mencetak `MENUNGGU KONFIRMASI`. Maka di sisi HUD:

```python
parkir = "KONFIRMASI" in link.state_teensy().upper()      # False
if cara == "ambil_alih" and not parkir:
    cara = "lihat"                                        # -> S_AWAS
```

dan `S_AWAS` isinya persis *"MENGAMATI saja. Tidak satu pun perintah gerak
dikirim."* Itu interlock yang benar: menggerakkan badan sementara sekuens
lengan berjalan berarti dua penguasa untuk satu robot, dan yang kalah korban
yang tersenggol capit yang sedang turun.

Jendelanya juga cuma **±4,5 detik** (`LENGAN_JEDA_MS` 900 × 5 fase) sebelum
`ruasBerikut()` jalan sendiri dan `nav.navUpdate()` merebut kaki lagi. Jadi
memaksa HUD mengirim perintah gerak pun tidak akan menyelesaikannya.

## Perbaikannya: firmware PARKIR, dan HUD berhenti mempercayai status basi

### Sisi firmware — `Hexapod_Unlimited_v1.12_R2C.zip`

Tiga hal, semuanya sudah ada di dalam zip:

1. **`m8` — parkir sebelum sekuens AMBIL, bawaan NYALA.** Ruas `AKS_AMBIL`
   berhenti di `MISI_KONFIRM` **sesudah** mengirim pemicu, mengumumkan
   `=== PARKIR UNTUK VISION ===`, dan menunggu `m2` dari Raspi.
2. **`sekuensAmbil` fase 4 → pose netral `R`**, bukan GENDONG.
3. **`GRIP_PERSEN_MIN` 5 → 10** — `g0` manual pun berhenti di 10%.

Tabel `RUAS[]` tidak disentuh sama sekali, jadi `m4 0 23`, `m7` dan acuan
"ruas 30" tetap menunjuk tempat yang sama.

```
bash ~/M/siapkan_teensy_pi.sh                              # sekali saja
python3 ~/M/flash_teensy.py --versi R2C --hanya-compile
python3 ~/M/flash_teensy.py --versi R2C
```

`--versi R2C` cocok tepat satu entri — aman dari jebakan nomor versi kembar.

### Sisi HUD — status basi tidak lagi dihitung sebagai bukti

Ini bug laten yang akan menggigit **sesudah** flash, dan gejalanya identik
dengan sebelum flash — jadi ia gampang disalah-baca sebagai "patchnya tidak
jalan". Karena itu ditutup bersamaan.

`parkir` dibaca dari `state_teensy()`, yang datang dari poll `m` **1× per
detik**. `tangani_pemicu()` dipanggil pada frame yang sama saat baris
`#KORBAN` masuk, **sekali saja**. Jadi:

```
firmware:  cetak "#KORBAN AMBIL 1"  ->  parkir di MISI_KONFIRM
HUD:       baca baris itu detik ini juga
HUD:       cek state_teensy() -> status dari <=1 detik LALU: "BERJALAN"
HUD:       parkir=False -> turun ke 'lihat' -> nol gerak
firmware:  parkir sampai batas waktu habis -> capit turun
```

Sekarang keputusannya **dua langkah**:

1. `#KORBAN` masuk → dicatat sebagai **tertunda**, HUD mengirim `m` sekali.
2. Diputuskan begitu **bukti segar** datang, mana pun yang lebih dulu:
   - baris `=== PARKIR UNTUK VISION ===` dari firmware — datang di burst
     serial yang sama dengan pemicunya, tidak butuh poll sama sekali; atau
   - baris `state` baru yang menyebut `KONFIRMASI` — dan **harus** baru
     (`n_status` naik), bukan yang tersimpan sejak sebelum pemicu.
3. Lewat `PEMICU_TUNGGU_BUKTI_S` (2 detik) tanpa bukti apa pun → baru turun ke
   `lihat`, dengan sebab lengkap dan jalan keluarnya disebut.

`parkir_visi` juga **dimatikan** setiap kali status segar menunjukkan firmware
sudah tidak di KONFIRM — bukti tidak boleh menempel sesudah lewat.

## Pembagian tugas: Raspi HANYA menengahkan

Keputusan R2C. Turun-naiknya capit **seluruhnya** milik Teensy; Pi tidak
pernah mengirim `g` atau `a` di jalur misi.

```
Teensy  berhenti di HNT_DEPAN 24 cm, kirim #KORBAN, PARKIR
  ↓
Pi      AMBIL_TENGAH  pivot kaki 'O' sampai galat ≤ 8°
        AMBIL_HALUS   putar badan sampai ≤ 20 px BERTAHAN 2 detik
        netralkan badan (r0 0 0, t0 0 0), kirim 'm2'
  ↓
Teensy  sekuens capit -> angkat ke pose R -> ruas berikutnya
```

### `S_A_MAJU` dilewati di jalur misi — dan itu memperbaiki bug, bukan cuma menghemat langkah

Ini yang paling mudah terlewat. Firmware menghentikan robot di
`KORBAN_JARAK_CM` = **24 cm**, dan pose lengannya

```
KORBAN_CAPIT_MM = 24 × 10 + LIDAR_DEPAN_MM(62) = 302 mm
```

**dihitung untuk robot yang berdiri di 24 cm itu.** `S_A_MAJU` akan berjalan
maju sampai LiDAR = `capit_cm` = 10 cm. Kalau ia ikut jalan, Pi memajukan
robot ~14 cm lalu firmware tetap menjulurkan lengan seolah masih di 24 cm —
capit lewat jauh di belakang korban, tanpa satu pun pesan yang menyebut kenapa.

Jadi saat `dari_firmware`: `AMBIL_TENGAH` → **langsung** `AMBIL_HALUS`.
Rantai penuh (`MAJU`/`SIAP`/`JEPIT`/`ANGKAT`) tetap ada untuk tombol
**Mulai AMBIL otomatis** — itu alat uji di meja, bukan jalur misi.

Ikutannya: `capit_cm`, `ambil_maks_cm` dan silang vision–LiDAR semuanya hidup
di `S_A_MAJU`, jadi di jalur misi **tidak ada satu pun gerbang jarak** yang
bisa menolak korban. Tidak perlu di-bypass; ia memang sudah tidak dilewati.

### Kapan Pi yakin korban sudah di tengah

```
tol   = min(tengah_tol_deg 1,5° , tengah_tol_px 20 px ÷ 18,2 px/° = 1,10°)
tahan = tengah_tahan_s 2,0 detik, PUTUS SEKALI -> hitungan balik ke nol
```

Begitu syarat itu bertahan penuh, `m2` dikirim **saat itu juga** — tidak
pernah menunggu timer. Batas waktu itu tenggat, bukan jadwal.

### Anggaran waktu — ubah keduanya atau jangan sama sekali

| | |
|---|---|
| `MISI_VISI_BATAS_MS` (firmware) | **20 detik** |
| `BATAS[S_A_TENGAH]` | **8 detik** |
| `BATAS[S_A_HALUS]` | **8 detik** |
| sisa margin | **4 detik** |

HUD selalu kehabisan waktu lebih dulu, jadi yang terbaca selalu
`GAGAL: batas waktu di state AMBIL_HALUS` — bukan capit yang turun diam-diam.
Bagian **69** `test_mission_hud.py` menjaga pasangan angka ini.

Sebelumnya anggaran HUD 30 + 25 = 55 detik melawan batas firmware 40 detik —
invariant "HUD bicara lebih dulu" yang ditulis di dokumen lama itu sudah bocor
begitu `AMBIL_HALUS` masuk jalur. Sekarang dijaga tes, bukan diingat.

### `auto_konfirm` bawaan NYALA

Dulu MATI, dan itu benar selama penengahan otomatis belum jadi jalur resmi.
Sekarang ia jalur resminya: firmware **parkir menunggu `m2`**, jadi
`auto_konfirm` MATI berarti parkir itu tidak pernah dijawab dan capit baru
turun sesudah batas waktu habis — persis gejala yang sedang diperbaiki.
Knob-nya tetap ada di tab Kalibrasi untuk dimatikan saat menyetel.

## Mode BACA dihapus, filter serial dihapus

Keduanya atas keputusan R2C, dan keduanya permanen.

**Mode BACA tidak ada lagi.** HUD selalu boleh menggerakkan robot sejak detik
pertama. Tombol ganti mode dibuang; `--auto` masih diterima supaya perintah
lama tidak galat, tapi tidak berpengaruh.

**Tidak ada whitelist dan tidak ada daftar terlarang.** `PERINTAH_BACA` dan
`TERLARANG` dibuang seluruhnya, tombol **Kirim PAKSA** ikut hilang karena
sudah tidak ada yang perlu ditembus. Semua perintah lewat apa adanya, dua arah.

Alasannya bukan kemalasan:

1. Keselamatan dipegang **mekanik** — itu tempat yang benar untuknya.
2. Saringan yang menolak **diam-diam** menyembunyikan jawaban firmware yang
   justru sedang dicari. `C` pernah begitu berhari-hari: HUD menolaknya lebih
   dulu, jadi pesan `Gagal: Tidak ada data IMU.` yang seharusnya muncul tidak
   pernah terlihat, dan **satu masalah menyamar jadi lima**.
3. Tiap perintah baru di firmware berarti satu baris baru di daftar. Daftar
   yang tertinggal satu versi terbaca sebagai "fiturnya tidak jalan".

⚠️ Konsekuensinya disebut terbuka: **`x` mematikan seluruh PWM dan robot
AMBRUK**; **`S`, `W`, `e` menulis EEPROM**. Tidak ada lagi yang menahannya
dari halaman.

**Kotak kirim serial sekarang kartu paling atas tab Manual** — dulu terkubur
di dalam kartu diagnosa IMU.

## Yang masih harus diukur, bukan ditebak

### `KORBAN_DEKAT_MM` hampir pasti terlalu kecil

Perlu diluruskan dulu: **`KORBAN_TINGGI_MM = 40` itu BUKAN tinggi korban.**
`config.h:128` menyebutnya sendiri — *"tinggi titik jepit DARI LANTAI"*, yaitu
di ketinggian berapa capit mencengkeram. 40 mm pada korban 95 mm ≈ 42% tinggi
badan, masuk akal untuk "cengkeram di bawah lengan" seperti aturan guidebook.

Yang bermasalah konstanta di sebelahnya. Fase 0 `sekuensAmbil` komentarnya
*"mendekat DARI ATAS korban"*, pada

```
KORBAN_TINGGI_MM 40 + KORBAN_DEKAT_MM 40 = 80 mm dari lantai
```

Korban **95 mm**. Capit yang menyapu masuk di 80 mm itu **15 mm di bawah
puncak korban** — ia tidak lewat dari atas, ia menabraknya dan menjatuhkannya
sebelum sempat turun menjepit. Supaya benar-benar lolos, `KORBAN_DEKAT_MM`
perlu ≥ 55, realistisnya **60–70** (40 + 65 = 105 mm, lolos 10 mm di atas
kepala).

**Belum diubah** — ini perlu diukur dengan boneka sungguhan, bukan diputuskan
dari angka.

### Pose GENDONG menghalangi LiDAR depan — hipotesis, belum diuji

Selama patch belum di-flash, fase 4 masih GENDONG: capit **280 mm** dari pusat
badan. Sensor depan duduk di **62 mm**. Jadi muatannya ada **±21,8 cm tepat di
depan sensor depan**, dan ruas ber-`HNT_DEPAN` sesudah K-1 ambangnya:

| ruas | ambang | vs 21,8 cm |
|---|---|---|
| 4 — ke tembok | 15 cm | lolos tipis |
| 14 — putar kiri lalu maju | 21 cm | di ambang |
| 18 — R-6 ke SZ-3 | 25 cm | **berhenti seketika** |
| 22 — ke depan tangga | 21 cm | di ambang |

Kalau benar, sesudah mengangkat K-1 robot berhenti sendiri di ruas-ruas itu
tanpa ada tembok. **Cara mengujinya 30 detik:** gendong korban, ketik `l`,
lihat channel DEPAN.

Pose `R` menaruh lengan 80 mm ke depan dan terlipat di atas badan — komentar
Vincent menyebut alasannya persis ini: *"tidak menghalangi LiDAR depan"*. Jadi
flash patch-nya sekaligus menutup hipotesis ini.

### Geometri vision di 24 cm — aman, sudah dihitung

Vertical FOV 43,3° → tinggi pandangan ≈ 0,794 × jarak. Korban 9,5 cm:

| | |
|---|---|
| fraksi bbox di 24 cm | **0,50** |
| gerbang MENILAI `bbox_h` 0,35–0,90 | jarak **13,3 – 34,2 cm** |
| gerbang MENGEJAR `jejak_h` 0,06–0,98 | jarak **12 – 198 cm** |

24 cm jatuh di tengah kedua pita. Tidak ada korban yang ditolak karena jarak
di jalur ini; yang benar-benar ditolak cuma lebih dekat dari ~12 cm, dan robot
tidak pernah semaju itu.

## Urutan mencobanya

```
1. bash ~/M/siapkan_teensy_pi.sh                          (sekali saja)
2. python3 ~/M/flash_teensy.py --versi R2C --hanya-compile
3. python3 ~/M/flash_teensy.py --versi R2C
4. upload_to_pi.bat                                       (dari CMD Windows)
5. b                                                      (berdiri)
6. m                                                      -> "tunggu visi : NYALA"
7. m4 1 1                                                 (ruas K-1 saja dulu)
```

Yang harus terlihat di log kalau lupnya jalan:

```
#KORBAN AMBIL 1
=== PARKIR UNTUK VISION -- menunggu 'm2' dari Raspi ===
[PEMICU] #KORBAN AMBIL ruas 1 -- AMBIL ALIH. Firmware parkir di KONFIRM...
[SERAH]  tengah dalam 0,8 der, BERTAHAN 2,0 detik. Badan dinetralkan, 'm2' dikirim
```

Kalau baris `[SERAH]` tidak pernah muncul, baca dua baris di kartu Ambil
korban: **pivot gait** dan **tengah bertahan**. Salah satunya akan menyebut
sebabnya.

---

## Dua bug yang muncul sesudah firmware benar (13 Sep 2026, sore)

Firmware sudah parkir menunggu vision — itu terbukti di arena. Yang tersisa
dua hal di sisi Pi, dan keduanya **saling menyamarkan**.

### Bug 1 — serah-terima direbut balik, tiap frame

Log dari arena:

```
[AWAS] 1 dari 0..32 -- K-1 angkat korban -> vision melihat RAGU margin +0.00 conf 0.00
       (tidak ada yang mencapai k-of-N)
[AWAS] masuk ruas korban: 1 dari 0..32 -- K-1 angkat korban. Kamera NYALA...
[AWAS] 1 dari 0..32 -- K-1 angkat korban -> vision melihat RAGU margin +0.00 conf 0.00
```

Dua hal janggal di situ, dan keduanya menunjuk sebab yang sama. Pertama,
state-nya `AWAS` — **bukan** `AMBIL_TENGAH`, jadi serah-terimanya tidak
berjalan. Kedua, `RAGU margin +0.00 conf 0.00` berulang: margin nol persis
bukan "sasarannya ambigu", itu **voting yang tidak pernah sempat mengumpulkan
satu frame pun**.

Sebabnya di blok serah-terima. Kedua cabangnya dulu berbunyi:

```python
if "KONFIRMASI" in fw and misi.state not in (S_KONFIRM, S_GAGAL):      -> S_KONFIRM
elif any(... RUAS_VISION ...) and misi.state not in (S_AWAS, S_KONFIRM, S_GAGAL):  -> S_AWAS
```

Rantai AMBIL **tidak ada** di daftar pengecualian itu. Padahal selama ruas
AMBIL berjalan, keduanya terus bernilai benar:

- `ruas_fw()` terus berbunyi `K-1 angkat korban`;
- `fw` terus berbunyi `MENUNGGU KONFIRMASI -- PARKIR VISION` — **justru karena
  firmware sedang parkir menunggu kita.**

Jadi begitu `tangani_pemicu()` memindahkan state ke `AMBIL_TENGAH`, frame
berikutnya menariknya kembali ke `S_AWAS` atau `S_KONFIRM` — keduanya state
**menonton**, nol perintah gerak. Frame berikutnya lagi, dan seterusnya. Tiap
tarikan memanggil `juri.reset()`, jadi votingnya tidak pernah sampai k-of-N.

Yang merebut adalah **bukti parkir yang sama yang baru saja kita tunggu**. Itu
yang membuatnya hampir tidak mungkin dilihat dari luar.

**Perbaikan:** satu penjaga `_sedang_serah` — rantai AMBIL **atau** pemicu yang
masih tertunda — dipasang di kedua cabang.

### Bug 2 — "bergerak ke kiri lalu melawan ke kanan"

Ini yang terlihat fatal, dan ia **konflik Raspi–Teensy yang sebenarnya** —
tapi tidak ada satu pun perintah yang bertabrakan di kabel. Barisnya justru
ada di log, cuma tidak terbaca sebagai penyebab:

```
Pivot MULAI menuju 176.8 der (sekarang 275.3 der).
```

−98,5°. Tidak ada penengahan kamera yang pernah meminta sebesar itu — kalibrasi
`pivot_gain` 0,6 berarti galat 164° dulu. Itu bukan perintah Pi. Itu
**firmware memulai ruas berikutnya**: batas parkir habis → sekuens lengan
jalan → `ruasBerikut()` → `MISI_PIVOT` ke arah ruas 2.

Jadi urutannya: Pi mengirim `O` untuk menengahkan (robot ke kiri) → di detik
yang sama firmware menyerah menunggu dan memulai ruas 2 (robot ke kanan).
Dua penguasa menarik satu robot, dan tidak ada baris serial yang mengatakan
bahwa itu sedang terjadi.

Akar penyebabnya Bug 1: Pi tidak pernah sampai mengirim `m2`, jadi parkirnya
selalu berakhir lewat batas waktu.

**Perbaikan:** Pi berhenti seketika begitu firmware tidak lagi parkir.
Dideteksi lewat **tepi**, bukan nilai — parkir harus pernah *terlihat* dulu,
baru hilangnya berarti sesuatu; tanpa itu serah-terima yang buktinya datang
dari baris pengumuman akan membatalkan dirinya sendiri sebelum poll `m`
pertama sempat menjawab. Yang tercetak sekarang:

```
[SERAH GAGAL] firmware BERHENTI parkir sebelum penengahan selesai -- batas
waktu 'm8' habis, atau ada yang mengirim m2/m0. Firmware sekarang menjalankan
ruas berikutnya sendiri, jadi Pi BERHENTI mengirim perintah gerak...
```

Pi **tidak** mengirim `s`. Firmware yang jalan sendiri itu perilaku
fallback yang memang dirancang ("tidak pernah lebih buruk daripada `m8` MATI");
menghentikannya berarti membuang sisa misi demi satu korban.

## Dummy tidak pernah jadi sasaran

Laporan yang sama: ada dummy di sebelah korban, robot menengahkan diri ke
**dummy**.

Sebabnya satu baris:

```python
t = max(lolos, key=lambda d: (d[3] - d[1]))      # yang TERBESAR, titik
```

Kelas tidak dilihat sama sekali. Dummy yang kebetulan lebih dekat kamera selalu
menghasilkan bbox lebih tinggi, jadi dummy menang. `sasaran_sah()` lalu menahan
gerakan — jadi robot **mematung menghadap dummy** sementara korban berdiri di
sebelahnya dan tidak pernah dipilih.

`sasaran_sah()` menjawab *"boleh bergerak?"*. Itu pertanyaan yang berbeda dari
*"yang mana?"*, dan selama keduanya dijawab satu fungsi, jawaban yang benar
untuk pertanyaan kedua tidak pernah ada.

Sekarang ada `pilih_sasaran()`, dipakai **sama persis** di JEJAK, CENTER dan
AMBIL_TENGAH:

1. di antara yang lolos saring, ambil yang berkelas **korban** — yang terbesar;
2. tidak ada korban sama sekali → **tidak ada sasaran**, robot diam, dan
   jumlah dummy yang diabaikan tampil di kartu vision
   (`N dummy terlihat, DIABAIKAN`) plus satu baris `[SASARAN]` di log;
3. `jejak_hanya_korban` dimatikan → kembali ke perilaku lama.

⚠️ **Ini bukan pengulangan regresi 12 September.** Yang dulu merusak JEJAK
adalah `lolos = []` — memotong daftar yang dipakai **menggambar**, sehingga
kotaknya jadi abu-abu bertuliskan "disaring" tanpa sebab. `pilih_sasaran()`
tidak menyentuh `lolos` sama sekali: dummy tetap digambar, tetap merah, tetap
terlihat. Yang berubah cuma mana yang dijadikan sasaran. Tesnya membuktikan itu
dengan menjalankannya, bukan dengan membaca sumbernya — bagian **67** dan
**71**.

`sasaran_sah()` tetap ada sebagai lapis kedua. Dua lapis di sini disengaja:
yang satu memilih, yang satu menahan.

## Membaca log kalau lupnya masih macet

| yang terlihat | artinya |
|---|---|
| `[AWAS] ... RAGU margin +0.00 conf 0.00` berulang | serah-terima direbut balik — Bug 1, seharusnya sudah tidak terjadi |
| `[SASARAN] N deteksi ... semuanya 'dummy'` | korban belum terlihat; robot menunggu, bukan macet |
| `[SERAH GAGAL]` | penengahan kehabisan waktu; periksa **pivot gait** dan **tengah bertahan** |
| `Pivot MULAI menuju <x> der` | **firmware** memulai ruas berikutnya — bukan perintah Pi |
| `[SERAH] tengah dalam ... BERTAHAN ... 'm2' dikirim` | lupnya jalan |
