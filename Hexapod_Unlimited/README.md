# Hexapod Unlimited

Firmware Teensy 4.1 untuk robot hexapod berkaki enam: gait tripod, kinematika invers kaki dan lengan, IMU 10-axis, enam LiDAR ToF **VL53L1X**, dan navigasi otonom ikut-dinding. Semua kalibrasi permanen di EEPROM.

**Status singkat (Agustus 2026).** Kaki, gait, body kinematics, pivot, dan navigasi ikut-dinding sudah jalan. Lengan belum terpasang fisik sehingga IK-nya belum teruji. Keenam LiDAR hidup sejak September 2026, jadi mode ikut-dinding **kiri** dan **kanan** dua-duanya bisa dipakai — dengan catatan arah fisik ch0 dan ch2 masih perlu dikonfirmasi dengan `l` (lihat bagian peta channel).

---

## Daftar isi

1. [Arsitektur](#1-arsitektur)
2. [Perangkat keras & bus](#2-perangkat-keras--bus)
3. [Boot aman: robot menyala dalam keadaan lemas](#3-boot-aman-robot-menyala-dalam-keadaan-lemas)
4. [Gait](#4-gait)
5. [Body kinematics](#5-body-kinematics)
6. [LiDAR](#6-lidar)
7. [Navigasi ikut-dinding — cara kerja & rumus](#7-navigasi-ikut-dinding--cara-kerja--rumus)
8. [Pivot & kompas arena](#8-pivot--kompas-arena)
9. [Lengan](#9-lengan)
10. [Peta EEPROM](#10-peta-eeprom)
11. [Prosedur bring-up & kalibrasi](#11-prosedur-bring-up--kalibrasi)
12. [Daftar perintah serial](#12-daftar-perintah-serial)
12b. [Menyetel parameter tanpa kompilasi ulang](#12b-menyetel-parameter-tanpa-kompilasi-ulang)
13. [Harness simulasi PC](#13-harness-simulasi-pc)
14. [Apa yang berubah dari program lama](#14-apa-yang-berubah-dari-program-lama)
15. [Yang masih menunggu](#15-yang-masih-menunggu)

---

## 1. Arsitektur

Satu class fasad, `Hexapod`, membungkus seluruh rantai dari perintah gerak sampai pulsa servo. `.ino` dan modul misi hanya bicara ke fasad itu.

```
perintah serial / Navigation
            |
            v
   Hexapod::walk(maju, geser, putar)      -1..1
            |
            v
       HexaGait          slew vektor gerak, ramp profil medan,
            |            tripod stance/swing -> legTargets[6] (frame badan)
            v
   Hexapod::legSolve()   + pose badan (rotasi & geser, sudah di-ramp)
            |            + zOff per kaki dari EEPROM 2048
            v
 LegInverseKinematics    -> sudut coxa / femur / tibia
            |
            v
   Hexapod::angleToPulse()   invert, offset, trim per servo
            |
            v
      HexaServos         commit 18 pulse tiap SERVO_COMMIT_MS (20 ms)
```

### Modul

| Berkas | Tanggung jawab |
|---|---|
| `Hexapod_Unlimited.ino` | Objek global, `setup()`, `loop()`, parser perintah serial |
| `Hexapod.*` | Fasad. Pose badan, IK per kaki, konversi pulse, arm/disarm, diagnostik `d` |
| `HexaGait.*` | Generator gait tripod berbasis waktu |
| `LegInverseKinematics.*` | IK 3-DOF satu kaki |
| `HexaServos.*` | Dua PCA9685, gerbang keselamatan PWM, stagger/ramp saat menyalakan |
| `HexaArm.*` + `ArmInverse.*` | Dua lengan 2-DOF + penjepit, IK planar 2 link |
| `Imu.*` | Parser protokol WIT (Yahboom 10-axis) |
| `LidarArray.*` | 6× VL53L0X lewat mux TCA9548A, round-robin non-blokir, filter |
| `Navigation.*` | Kompas arena, pivot, ikut-dinding — semuanya state machine non-blokir |
| `Calib.*` | Blob parameter & kalibrasi servo di EEPROM 0 |
| `EEMap.h` | Tata letak EEPROM + penjaga `static_assert` |
| `config.h` | Konstanta perangkat keras & geometri |
| `types.h` | `Vec3`, helper sudut, rotasi badan. Murni, bisa diuji di PC |

### Loop utama

`loop()` memanggil semuanya sekali per iterasi, tanpa `delay()`:

```cpp
imu.update();          // parse byte WIT yang masuk (dibatasi IMU_MAX_BYTE_UPDATE)
lidar.update();        // SATU sensor per putaran, tidak pernah menunggu
demoUpdate();          // demo body kinematics 'B'
yawStreamUpdate();     // aliran 'y'
lidarStreamUpdate();   // aliran 'L'
nav.navUpdate();       // ikut-dinding / pivot / belok arena -- satu langkah
robot.update();        // ramp pose badan -> gait -> IK -> commit servo
// lalu parser serial
```

Satu-satunya jalur yang masih memblokir adalah `kalibrasiPivot()` (`C`), dan itu disengaja — ia mengukur rotasi selama beberapa siklus penuh.

### Urutan inisialisasi yang WAJIB dijaga

`gCalib` adalah objek global yang ter-*zero-init*. Kalau `Calib::load()` tidak dipanggil paling awal:

* `SERVO_PULSE_MIN` = `SERVO_PULSE_MAX` = **0** → `angleToPulse()` selalu mengembalikan 0 µs → PWM mati → **seluruh servo lemas, robot diam total**;
* `GAIT_SLEW_RATE` = 0 → vektor gerak tak pernah naik → perintah `w` tidak berefek;
* `SERVO_INVERT[]` = 0 → kaki kiri tidak terbalik.

```cpp
Calib::load();   // <-- PALING AWAL
imu.begin();
robot.begin();   // loadServoMap() -> _servos.begin() (PWM mati) -> loadZOff()
lidar.begin();
nav.begin();
```

---

## 2. Perangkat keras & bus

Tiga bus I2C terpisah supaya tidak ada tabrakan alamat:

| Bus | Pin | Isi |
|---|---|---|
| `Wire` | SDA 18 / SCL 19 | Mux LiDAR TCA9548A `0x70` + 6× VL53L1X `0x29` |
| `Wire1` | SDA 17 / SCL 16 | PCA9685 driver 0 `0x41` |
| `Wire2` | SDA 25 / SCL 24 | PCA9685 driver 1 `0x40` |
| `Serial2` | RX 7 / TX 8 | IMU Yahboom 10-axis, protokol WIT, 230400 baud |

> **Jangan pernah menyatukan bus LiDAR dengan PCA9685.** Alamat ALL-CALL bawaan PCA9685 juga `0x70` — sama dengan TCA9548A — dan akan bentrok.

Pustaka **"VL53L1X by Pololu"** wajib terpasang — itu pustaka **terpisah**, bukan versi baru dari "VL53L0X by Pololu". Karena `LidarArray.cpp` ada di dalam folder sketsa, tanpa pustaka itu **seluruh sketsa gagal dikompilasi**.

### Geometri kaki

`COXA_LENGTH` 20 mm, `FEMUR_LENGTH` 80 mm, `TIBIA_LENGTH` 90 mm. Sumber kebenarannya `legacy-2026/TES_GERAK/kinematics.h` — program yang sudah terbukti berdiri. Jangan diubah tanpa mengukur ulang kaki fisik.

Konvensi frame, sama di seluruh firmware: **+X kanan, +Y depan, +Z atas**, origin di pusat badan.

---

## 3. Boot aman: robot menyala dalam keadaan lemas

`setup()` **tidak menggerakkan servo sama sekali**. `HexaServos::begin()` memanggil `setPWM(ch, 0, 0)` pada 16 channel di kedua driver — output benar-benar mati, bukan pulsa 0 µs.

Konsekuensi praktisnya: **Teensy tidak perlu dilepas dari robot untuk upload.** Sesudah upload, Teensy reboot otomatis, robot tetap terkulai, dan Anda sempat membaca pesan boot sebelum memutuskan menyalakan servo.

* **`b`** menghitung pose berdiri **dulu**, baru menyalakan PWM — servo langsung menuju sasaran yang benar, tidak lewat 1500 µs.
* Pengaktifan **pertama** dilakukan satu per satu, jeda 60 ms per servo (`staggerTo`), karena posisi fisik servo belum diketahui. Ini menyebar lonjakan arus 18 servo supaya BEC tidak drop dan Teensy tidak brownout.
* Pengaktifan **berikutnya** memakai ramp 400 ms (`rampTo`) karena posisi terakhir sudah diketahui.
* **`x`** mematikan PWM kapan saja.
* Servo lengan default **nonaktif**; hidup otomatis saat perintah lengan pertama dipakai.

### ⚠ `DEMO_BOOT` — pengecualian sementara untuk pajangan

`DEMO_BOOT` di `config.h` **membatalkan perlindungan di atas**: robot berdiri sendiri beberapa detik sesudah menyala, termasuk sesudah **setiap kali program diunggah**. Ia ada karena diminta untuk pajangan sesaat.

```
DEMO_BOOT 0   <- kembali ke boot lemas yang aman
```

Urutannya persis sama dengan mengetik `b` → `z10 1` → tunggu → `0`, dan perintahnya benar-benar dilewatkan ke `handleCmd()` yang sama — bukan disalin — jadi demo tidak bisa menyimpang dari perilaku perintah manualnya.

| tahap | waktu | yang terjadi |
|---|---|---|
| tunda | `DEMO_BOOT_TUNDA` 3000 ms | servo **tetap lemas**, hitung mundur dicetak tiap detik |
| berdiri | + `DEMO_BOOT_BERDIRI` 1500 ms | `b` — servo hidup, kaki menetap |
| goyang | + `DEMO_BOOT_LAMA` 20000 ms | `z10 1` — badan mengayun 10°, kaki tetap menapak |
| selesai | — | `0` — pose badan dinolkan, robot **tetap berdiri** |

Yang menjaganya tetap aman sejauh mungkin:

* **Tunda 3 detik sebelum servo hidup.** Jangan dikecilkan di bawah 2 detik — ini satu-satunya jendela untuk menjauhkan tangan atau membatalkan.
* **Ketikan apa pun membatalkan**, diproses sebelum karakternya masuk ke parser, jadi perintah yang diketik tetap berjalan normal sesudahnya. Kalau goyang sudah terlanjur jalan ia sengaja **dibiarkan** — sejak operator menyentuh keyboard, dialah yang pegang kendali — dan keadaan itu ikut dicetak, bukan dibiarkan jadi kejutan.
* **State machine, bukan `delay()`.** Selama `delay()` parser serial mati, IMU dan LiDAR berhenti diperbarui, dan servo tidak di-commit — persis kesalahan yang sudah dibersihkan dari `pivotKe()`.
* **`d` ikut mengingatkan** bahwa `DEMO_BOOT` aktif, supaya orang berikutnya yang membuka diagnostik tidak terkejut.

Diuji di `sim_boot` lewat `loop()` yang asli: servo terbukti masih lemas pada t=2,8 s, hidup pada t=4,1 s, puncak roll 10,00° selama goyang, kembali ke 0,00° pada t=27 s dengan servo tetap hidup, dan **tidak** menghidupkan dirinya lagi selama 15 detik berikutnya. Jalur pembatalan diuji di proses terpisah — servo tidak pernah hidup sama sekali.

---

## 4. Gait

Tripod: grup `{0,2,4}` dan `{1,3,5}` bergantian, beda fase 0,5.

* **Stance** — telapak menapak, bergeser lurus dari `+½·stepLength` ke `−½` dengan kecepatan konstan. Kecepatan konstan itu yang membuatnya tidak menyeret.
* **Swing** — trajektori **sikloid**: `dz = stepHeight · (1 − cos 2πs) / 2`. Kecepatan nol di liftoff dan touchdown, jadi mendarat lembut.
* **Komponen putar** — tiap kaki mendapat sumbangan rotasi lewat cross product `ω × r`:

  ```
  sx = ( vx − vyaw · ry / 100 ) · stepLength
  sy = ( vy + vyaw · rx / 100 ) · stepLength
  ```

  Kaki depan dan belakang jadi melangkah ke arah berlawanan → badan berputar.
* **Normalisasi** — kalau ada kaki yang vektornya melebihi `stepLength`, keenamnya dipangkas bersama-sama dengan faktor yang sama. Tanpa ini, maju sambil berputar bisa memaksa satu servo jauh melampaui yang lain.
* **Slew** — vektor gerak di-ramp `GAIT_SLEW_RATE` (3,0 satuan/detik). Ini peredam terakhir sebelum servo: perintah navigasi yang berderau tidak pernah sampai ke kaki sebagai hentakan.
* **Profil medan** di-ramp dengan konstanta waktu `GAIT_PROFILE_TAU`, jadi mengganti tinggi badan tidak membuat badan melonjak.

Semua berbasis `dt` sungguhan, bukan hitungan iterasi, sehingga tidak tergantung kecepatan loop.

---

## 5. Body kinematics

Kaki tetap menapak di tempat; yang bergerak hanya badan. Gunanya: menunduk lewat celah, memiringkan badan di tanjakan, dan nanti stabilisasi otomatis dari IMU.

Nama sumbu mengikuti **fisika**, bukan kebiasaan firmware:

| | sumbu putar | positif berarti |
|---|---|---|
| **roll** | +Y (depan) | miring **KANAN** |
| **pitch** | +X (kanan) | **MENDONGAK** |
| **yaw** | +Z (atas) | belok **KIRI** (CCW) |

Yang dihitung `rotatePointInv()` adalah **invers** rotasi badan: bila badan berputar `R` terhadap dunia sementara telapak diam di tanah, di frame badan telapak tampak berputar `R⁻¹`. Karena `R = Rz(yaw)·Ry(roll)·Rx(pitch)`, maka `R⁻¹ = Rx(−pitch)·Ry(−roll)·Rz(−yaw)` — **urutannya dibalik**, bukan sekadar sudutnya dinegatifkan (menegatifkan sudut saja hanya benar untuk sudut kecil).

### Pose badan di-ramp

Pose badan diterapkan **sesudah** gait, jadi ia melewati kedua peredam yang sudah ada (`GAIT_SLEW_RATE` dan `GAIT_PROFILE_TAU`). Dulu ia diterapkan mentah. Diukur di simulasi, satu siklus commit 20 ms:

| Perintah | Lonjakan sebelum | Lonjakan sesudah | Siklus sampai diam |
|---|---|---|---|
| `r20 0 0` | **49,20°** (547 µs) | 2,79° (31 µs) | 16 (≈320 ms) |
| `r0 20 0` | **43,29°** (481 µs) | 2,42° (27 µs) | 16 |
| `t0 0 -40` | **29,76°** (331 µs) | 1,91° (21 µs) | 16 |
| `b60` (lewat profil gait) | 0,40° | 0,40° | 78 — memang sudah di-ramp |

49° dalam 20 ms setara menyuruh servo berputar ~2400°/detik. Sekarang `setBodyRotation()` / `setBodyTranslation()` hanya menetapkan **sasaran**; `slewBodyPose()` di awal `update()` merayapkannya dengan `BODY_SLEW_DEG_S` (60 °/s) dan `BODY_SLEW_MM_S` (120 mm/s). Ramp berlaju tetap, bukan low-pass — waktu tempuhnya bisa diprediksi dan tak ada ekor panjang.

Demo `B` tidak terpengaruh: sapuan sinusnya paling cepat hanya ~21 °/detik. Yang dulu menyentak adalah perintah manual `r`/`t`, bukan demonya.

### Batas & deteksi di luar jangkauan

Semua nilai di-clamp ke `BODY_MAX_ROT_DEG` (20°) dan `BODY_MAX_TRANS_MM` (40 mm). Bila pose yang diminta tak bisa dituruti, ini terdeteksi lewat **dua** jalur:

1. **Target di luar jangkauan IK** — jarak `D` melebihi `femur + tibia` atau kurang dari `|femur − tibia|`.
2. **Sudut servo keluar dari 0–180°** — misalnya coxa diminta memutar 120° padahal servo mentok di 90°. Ini sempat lolos: IK hanya memeriksa jarak, jadi ia melaporkan "aman" sementara `angleToPulse()` diam-diam meng-clamp.

`d` menandai kaki bermasalah di kolom `rng`, dan `loop()` memperingatkan maksimal 1× per detik.

### Goyang roll bergelombang (`z`) — mode pajangan

Perintah `z` mengayunkan badan pada sumbu roll terus-menerus sampai dihentikan, sementara **keenam telapak tetap menapak di lantai**. Itu yang membuatnya enak dilihat: badan berayun di atas kaki yang diam.

```
z            hidup/mati dengan parameter terakhir (default 12°, 2 detik)
z<amp> <per> amplitudo derajat, periode detik  -- mis. z8 4 (pelan, anggun)
z<amp> <per> <fase>   fase pitch derajat; 90 membuat badan menelusuri KERUCUT
```

Bedanya dengan `B`: demo `B` menyapu enam sumbu sekali jalan lalu berhenti sendiri untuk **verifikasi**; `z` berayun tanpa henti untuk **dipajang**.

Fase pitch itu yang mengubahnya dari metronom jadi gelombang berputar — `pitch = amp·sin(ωt + fase)`. Dengan fase 90° badan tidak sekadar miring kiri-kanan, tapi menelusuri kerucut penuh.

**Penjaga bentuk gelombang.** Pose badan di-ramp `BODY_SLEW_DEG_S` (60 °/detik). Kalau laju puncak sinus melebihi itu, ramp memotong puncaknya dan yang keluar bukan gelombang lagi melainkan **segitiga**. Laju puncak sinus = `amp · 2π / periode`, jadi periode minimum yang aman adalah `amp · 2π / 60`.

`z` menghitungnya dan menaikkan periode bila perlu — dengan mengatakannya, bukan diam-diam:

```
>>> z12 0.5
Periode dinaikkan 0.50 -> 1.26 detik supaya sinusnya tidak terpotong ramp.
  (turunkan amplitudo kalau ingin ayunan lebih cepat)
```

Tanpa penjaga itu, `z12 0.5` menuntut 151 °/detik — tiga kali di atas batas ramp. Diverifikasi di `sim_goyang` dengan mengukur pose yang benar-benar keluar sesudah melewati ramp; keempat preset menghasilkan puncak dan laju **100%** terhadap sinus ideal.

Perintah apa pun yang menyentuh pose badan menghentikannya lebih dulu (`b`, `r`, `t`, `0`, `B`, `x`, Enter), jadi tidak pernah ada dua sumber yang berebut menulis pose.

**Cara membaca demo `B`:** kalau body kinematics benar, telapak **tidak boleh bergeser di lantai** — badan mengayun di atas kaki yang diam. Kalau telapak ikut menyeret, ada yang salah di rantai transform.

---

## 6. LiDAR

Enam **VL53L1X** lewat mux TCA9548A. `update()` memajukan **satu** sensor per pemanggilan tanpa busy-wait, memakai `dataReady()` dari pustaka. Filter: median-3 (buang spike) → EMA (`LIDAR_EMA_ALPHA` 0,4).

### Mode jarak — Long justru bukan yang terjauh di arena

| Mode | Gelap | Cahaya terang | Anggaran waktu minimum |
|---|---|---|---|
| **Short** | 136 cm | **135 cm** | 20 ms |
| Medium | 290 cm | 76 cm | 33 ms |
| Long | 360 cm | 73 cm | 33 ms |

Navigasi tidak pernah memakai jarak di atas `NAV_PELAN_CM` (50 cm) untuk mengemudi — `LIDAR_MAX_CM` hanya memilah "jauh". Jadi **Short** yang dipakai: cakupannya berlipat dari yang dibutuhkan, hampir kebal cahaya sekitar, dan anggaran 20 ms mempercepat laju sampel dari ~33 ms — yang langsung memperbaiki suku turunan PD, karena turunan dihitung pada laju sampel LiDAR (bagian 7.3).

Semuanya di `config.h`: `LIDAR_MODE`, `LIDAR_BUDGET_US`, `LIDAR_PERIOD_MS`, `LIDAR_MAX_CM`, `LIDAR_ROI_SEMPIT`. Kombinasi mode dan anggaran waktu dijaga `static_assert` — salah pasang gagal saat kompilasi, bukan berakhir sebagai `setMeasurementTimingBudget()` yang ditolak diam-diam di lapangan.

### Tiga keadaan, bukan dua

| Nilai | Arti | Tampilan `l` |
|---|---|---|
| angka cm | ada objek dalam jangkauan | `60 cm` |
| `LIDAR_JAUH` (999) | sensor **sehat**, tak ada objek dalam `LIDAR_MAX_CM` | `jauh` |
| `LIDAR_MATI` (−1) | sensor tidak merespons | `MATI` |

Pembedaan ini pondasi navigasi: "lorong terbuka" dan "kabel putus" menuntut reaksi yang **berlawanan**. Caranya, `_lastResp` mencatat kapan sensor terakhir menjawab **apa pun**, terpisah dari `_lastOk` yang mencatat pembacaan dalam jangkauan.

### Ketiganya dipisahkan lewat `range_status`, bukan ambang jarak

Ini beda paling penting dari VL53L0X, dan yang paling mudah luput saat porting. VL53L0X mengembalikan **8190 mm** sebagai sentinel "tidak ada target". **VL53L1X tidak punya sentinel apa pun** — `range_mm` selalu berisi angka hasil hitungan, sah atau tidak:

```cpp
ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;   // tanpa syarat
```

Jadi kode gaya VL53L0X yang menyaring dengan `mm >= 8000` bukan cuma tidak berfungsi — ia **menerima angka sampah sebagai jarak sah**, dan robot mengira ada dinding di tempat yang kosong. Satu-satunya sumber kebenaran adalah `range_status`, yang dipilah jadi tiga golongan:

| Golongan | Status | Perlakuan | `_lastResp` disegarkan? |
|---|---|---|---|
| Sah | `RangeValid`, `RangeValidNoWrapCheckFail`, `RangeValidMinRangeClipped` | pakai jaraknya | ya |
| Tak ada target | `SignalFail`, `OutOfBoundsFail`, `WrapTargetFail` | tandai `LIDAR_JAUH` | ya |
| Pengukuran buruk | `SigmaFail`, `XtalkSignalFail`, `MinRangeFail`, `HardwareFail`, … | buang | **tidak** |

Kolom terakhir itu yang membuatnya **fail-safe**. Sensor yang terus-menerus menghasilkan sampah tidak disegarkan, jadi ia jatuh ke `LIDAR_MATI` sesudah `LIDAR_TIMEOUT_MS` dan navigasi berhenti. Kalau ia ikut disegarkan, ia akan tampak "jauh" selamanya — yaitu **"tidak ada halangan"** — justru saat sensornya paling tidak bisa dipercaya.

`RangeValidMinRangeClipped` sengaja masuk golongan sah: artinya objek **sangat dekat** dan angkanya dipangkas ke batas bawah. Menolaknya akan membuat objek yang paling dekat justru dilaporkan "jauh".

Diuji di `sim_lidar` untuk kesembilan status:

```
objek 60 cm (RangeValid)                 ->    60  jarak cm
objek 150 cm > LIDAR_MAX_CM              ->   999  LIDAR_JAUH
objek sangat dekat (MinRangeClipped)     ->     4  jarak cm
tak ada target (SignalFail)              ->   999  LIDAR_JAUH
pengukuran buruk (SigmaFail)             ->    -1  LIDAR_MATI   <- fail-safe
terlalu dekat utk diukur (MinRangeFail)  ->    -1  LIDAR_MATI   <- fail-safe

jumlahHidup() saat semua tak melihat apa pun : 6 dari 6
jumlahHidup() saat semua menghasilkan sampah : 0 dari 6
```

> **Catatan sejarah.** Bug yang setara pernah ada di versi VL53L0X: penyaring `mm >= 8000` diletakkan sebaris dengan timeout, yaitu **sebelum** `_lastResp` disegarkan, sehingga sensor sehat yang menghadap ruang terbuka dilaporkan `LIDAR_MATI` dan `f`/`F` berhenti tepat di tikungan. Bentuknya berbeda, sumbernya sama: menyimpulkan kesehatan sensor dari angka jarak.

### Peta channel: urutan kabelnya TERBALIK

Kode lama menganggap channel 0 menghadap depan. Uji fisik menunjukkan urutan kabel sebenarnya **kebalikannya** — channel `n` memegang arah yang dulu diberi indeks `5 − n`:

| Channel mux | Arah fisik sebenarnya | Dikira kode lama |
|---|---|---|
| 0 | **kiri depan** — dari pola, belum diuji | depan |
| 1 | kiri belakang | kanan depan |
| 2 | **belakang** — dari pola, belum diuji | kanan belakang |
| 3 | kanan belakang | belakang |
| 4 | kanan depan | kiri belakang |
| 5 | **depan** | kiri depan |

Empat baris diperiksa satu per satu di robot; pola yang sama meramalkan dua sisanya, ch0 dan ch2. Waktu itu ramalannya tidak bisa diuji karena kedua sensor itu justru yang rusak fisik — konsistensinya bersifat kebetulan, bukan bukti. Sejak September 2026 keenamnya hidup, jadi **ch0 dan ch2 sekarang perlu diverifikasi langsung dengan `l`** sebelum mode `f` / `F` dipercaya di lapangan.

**Ini yang membuat navigasi tak pernah bisa dites.** `LIDAR_FRONT` menunjuk channel 0, dan channel 0 justru salah satu sensor yang mati. Jadi `f`/`F` selalu berhenti seketika dengan *"sensor DEPAN tidak merespons"* — gain, turunan PD, dan batas kemudi sama sekali tidak relevan selama itu belum benar.

`LIDAR_NAMA[]` ikut diurutkan menurut arah fisik. Keduanya dijaga saat kompilasi:

```cpp
static_assert(((1u << LIDAR_FRONT) | (1u << LIDAR_KANAN_D) | ... ) == 0x3Fu,
              "LIDAR_* di config.h harus enam channel BERBEDA dalam 0..5");
```

### Bacaan pendek palsu saat tidak ada objek

Gejala lapangan: tak ada apa pun di depan sensor, tapi `l` sering menunjukkan **5–10 cm**. Ada dua sebab yang sama sekali berbeda, dan perintah `l` sekarang memisahkannya karena ikut mencetak jawaban **mentah** dari sensor:

```
0 KIRI-DPN  : jauh (di atas 120 cm)   [mentah 90 mm, wrap target fail]
1 KIRI-BLK  : MATI -- tidak merespons [mentah 700 mm, sigma fail]
4 KANAN-DPN : 8 cm                    [mentah 80 mm, range valid]
5 DEPAN     : 60 cm                   [mentah 600 mm, range valid]
```

* **`wrap target fail`** — objeknya justru **terlalu jauh**. Sensor mengukur fase, dan target di luar jangkauan tak-ambigu bisa "berputar" (aliasing) menjadi jarak yang tampak pendek. VL53L1X mendeteksinya dengan mengukur pada dua laju pulsa lalu membandingkan; status ini artinya aliasing **terdeteksi**, jadi angkanya dibuang dan sensor dilaporkan `LIDAR_JAUH`. Ini sudah ditangani.
* **`range valid` dengan angka pendek** — sensornya benar-benar melihat sesuatu. Ini **bukan** masalah program: kemungkinan besar crosstalk dari kaca penutup, atau bagian badan/kabel robot yang terserempet kerucut 27°.

Tes yang memutuskan: **tutup sensor rapat dengan kain hitam.** Kain hitam menyerap IR, jadi hasil yang benar adalah `signal fail` (tak ada target). Kalau ia tetap melapor 5–10 cm `range valid`, cahayanya tidak pernah keluar dari modul — itu crosstalk kaca penutup, definitif. Pustaka Pololu **tidak** mengimplementasikan kalibrasi crosstalk kaca penutup; obatnya melepas kaca penutup, atau pindah ke port ST API penuh (`pololu/vl53l1x-st-api-arduino`).

#### Bug yang ikut ketemu: median-3 dilucuti tepat saat dibutuhkan

Histori median dinolkan (`_histN = 0`) setiap kali sensor melapor "jauh" — memang benar, jarak dekat dan jauh tidak boleh tercampur. Tapi baris berikutnya berbunyi:

```cpp
int m = (_histN < 3) ? cm : median3(...);   // sampel pertama LOLOS mentah-mentah
_jauh[_cur] = false;                        // dan langsung membalik keadaan
```

Jadi sampel pertama sesudah keadaan "jauh" melewati median tanpa disaring, **dan** langsung mengubah sensor dari "tak ada objek" menjadi "ada dinding 8 cm". Satu hantu tunggal cukup. Filter yang dipasang untuk membuang spike justru dilucuti persis pada saat ia paling dibutuhkan.

Sekarang keadaan "jauh" hanya ditinggalkan sesudah **tiga sampel dalam jangkauan berturut-turut**. Diukur di `sim_hantu`:

| Skenario | Sebelum | Sesudah |
|---|---|---|
| 1 dari 4 sampel hantu 8 cm | melapor **8 cm, 25% waktu** | tetap "jauh" |
| 1 dari 8 sampel hantu 10 cm | melapor **9 cm, 12% waktu** | tetap "jauh" |
| objek nyata 40 cm menetap | 40 cm | 40 cm |

Ongkosnya 3 × `LIDAR_PERIOD_MS` (~75 ms) sebelum objek baru diakui; pada 5,8 cm/detik robot hanya maju 0,4 mm.

#### `j` — mengubah "sering 5 cm" jadi data

Satu cuplikan `l` tidak bisa membedakan hantu yang **menetap** dari yang **berubah-ubah**. Perintah `j` mengumpulkan statistik mentah selama beberapa detik (non-blokir), lalu meringkas sebaran dan histogram statusnya:

```
j            semua channel, 5 detik
j<ch> <detik>   satu channel, mis. j5 10
```

Tiga hasil yang mungkin, dan masing-masing menunjuk sebab yang berbeda:

```
      range valid : 83 (100%)
      jarak sah : 49 .. 51 mm, rata 49 mm, sebaran 2 mm
      -> nyaris TIDAK bersebaran: benda TETAP di depan sensor

      signal fail : 83 (100%)                     <- ruang kosong yang benar

      range valid : 83 (100%)
      jarak sah : 45 .. 399 mm, sebaran 354 mm
      -> sebaran LEBAR: pantulan tak menentu, bukan benda tetap
```

**Sebaran** itu kesimpulannya. Crosstalk kaca penutup dan bagian robot yang terserempet kerucut menghasilkan angka yang nyaris tak bergoyang (beberapa mm). Pantulan tak menentu bersebaran puluhan sampai ratusan mm. Di ruang kosong yang benar, yang wajar adalah **100% `signal fail`**.

#### `J` — memisahkan crosstalk antar-sensor dari pantulan yang melekat

`j` bisa memastikan hantunya **menetap**, tapi tidak bisa memberi tahu *dari mana*. Dua sebab menghasilkan angka yang identik:

* **crosstalk antar-sensor** — keenam VL53L1X memancar **bersamaan terus-menerus** di mode kontinu, dan pancaran tetangga masuk ke penerima sensor ini;
* **pantulan melekat** — kaca penutup, bibir lubang braket, atau bagian robot tepat di depan sensor.

`u` mengukur tiap sensor **dua kali**: sekali saat kelima sensor lain ikut memancar, lalu sekali lagi sesudah kelimanya di-`stopContinuous()`. Itu percobaan yang tidak bisa dilakukan dengan tangan.

```
u              sapu SEMUA sensor berurutan, lalu tabel kesimpulan
u<ch> <detik>  satu sensor saja, mis. u5 4
```

```
--- TABEL UJI ISOLASI ---
  channel        A: semua   B: sendiri   kesimpulan
  ch0 KIRI-DPN     750 mm     750 mm    target NYATA, bukan hantu
  ch1 KIRI-BLK      29 mm    -kosong-   hantu HILANG -> crosstalk antar-sensor
  ch5 DEPAN         45 mm    -kosong-   hantu HILANG -> crosstalk antar-sensor

  KESIMPULAN: crosstalk ANTAR-SENSOR di semua yang bermasalah.
  Bisa diperbaiki di perangkat lunak: ukur BERGILIRAN, bukan serentak.
```

Bacaan menetap di atas 150 mm dianggap benda sungguhan — tanpa batas itu, sensor yang kebetulan menghadap dinding ikut dituduh bermasalah.

> **Kenapa `u`, bukan `J`.** Huruf besar-kecil di firmware ini dipakai untuk pasangan **simetris** — `f`/`F` kiri-kanan, `g`/`G` grip depan-belakang, `a`/`A` lengan. Memakai `j`/`J` untuk dua fungsi yang berbeda melanggar pola itu dan terbukti langsung tertukar saat dipakai. `u` dan `U` dua-duanya bebas, jadi salah ketik pun tidak melakukan apa-apa yang berbahaya.

> **Batas yang jujur:** gerbang median menyaring hantu **sesekali**. Kalau crosstalk muncul di hampir setiap pengukuran, tiga sampel berturut-turut akan lolos — dan memang seharusnya begitu: kalau sensor terus-menerus bersikeras ada objek di 8 cm dengan status `range valid`, perangkat lunak tidak punya dasar untuk membantahnya. Itu wilayah optik, bukan kode.

### `I` memindai **dan** memulihkan

`begin()` hanya berjalan sekali saat boot dan `_isReady` tidak pernah ditinjau ulang — jadi sensor yang gagal init mati untuk seluruh sesi, padahal `I` melaporkan modulnya "ADA". Sekarang `I` mencoba init ulang tiap channel yang ada di bus tapi tidak bekerja. **Dua** kondisi diperiksa:

| Baris di `l` | Keadaan internal | Dipulihkan `I`? |
|---|---|---|
| `tidak di-init saat begin()` | `_isReady` false | ya |
| `MATI -- tidak merespons` | `_isReady` **true**, berhenti kirim data | ya |
| angka / `jauh` | sehat | tidak disentuh |

Baris kedua mudah terlewat: sensor yang lolos init lalu berhenti mengirim tetap ber-`_isReady` true.

```
ch0 (KIRI-DPN ) : VL53L0X @0x29 ADA -> INIT ULANG BERHASIL (tadinya MATI)
ch1 (KIRI-BLK ) : VL53L0X @0x29 ADA tapi INIT ULANG GAGAL
ch5 (DEPAN    ) : VL53L0X @0x29 ADA, sudah aktif
  1 sensor dipulihkan tanpa reset papan. Ketik 'l' untuk memastikan.
  1 sensor menjawab di bus tapi menolak init -- curigai daya/kabel, bukan program.
```

Perbedaan dua pesan terakhir itu gunanya: **"INIT ULANG GAGAL" memisahkan masalah perangkat keras dari masalah program.**

Ikut benar bersamanya: `begin()` dan `I` memakai `initSensor()` yang sama (konfigurasinya tak bisa menyimpang); sensor yang di-init ulang dibuang riwayat filternya (kalau tidak, `getDistance()` sempat menyajikan jarak basi sebagai data baru); `_muxOk` akhirnya diperbarui oleh `I` — dulu hanya diisi saat boot, jadi mux yang baru disambungkan dianggap hilang selamanya dan `navMulai()` menolak jalan tanpa sebab yang terlihat.

Satu lagi: nilai balik `setMeasurementTimingBudget(20000)` dulu dibuang. Dengan periode VCSEL yang diperpanjang untuk long range, permintaan 20 ms bisa **ditolak diam-diam** — sensor lalu memakai anggaran bawaannya (33 ms). Sekarang penolakan itu dicetak.

---

## 7. Navigasi ikut-dinding — cara kerja & rumus

Inti algoritmanya cuma dua keputusan yang dihitung ulang tiap `navUpdate()`: **seberapa cepat maju** (dari sensor depan) dan **seberapa keras berbelok** (dari sensor samping). Keduanya keluar sebagai angka −1..1.

```
navUpdate()  ->  robot.walk(maju, 0, turn)  ->  HexaGait  ->  IK  ->  servo
```

Non-blokir: satu langkah per pemanggilan, tidak ada loop tunggu. Perintah serial tetap terproses, dan `s` / `x` / Enter selalu bisa menyela.

Sensor yang dipakai: **depan** (`LIDAR_FRONT`) selalu, plus **satu** sensor samping — `LIDAR_KIRI_D` untuk mode `f`, `LIDAR_KANAN_D` untuk mode `F`.

### 7.1 Kecepatan maju, dari sensor depan

| Bacaan depan | Tindakan |
|---|---|
| `LIDAR_MATI` | **berhenti total** — jangan pernah jalan buta ke depan |
| `LIDAR_JAUH` | maju penuh |
| angka cm | rumus di bawah |

```
maju = NAV_FWD_SPEED × clamp( (depan − FRONT_STOP_CM) / (NAV_PELAN_CM − FRONT_STOP_CM), NAV_MAJU_MIN, 1 )
     = 0,8 × clamp( (depan − 20) / 30 , 0,15 , 1 )
```

| depan | maju |
|---|---|
| ≥ 50 cm | 0,80 |
| 40 cm | 0,53 |
| 30 cm | 0,27 |
| 25 cm | 0,13 |
| **≤ 20 cm** | **0,00** → cabang halangan: `turn = −sisi × NAV_BELOK_CMD` (0,60), memutar **menjauhi** dinding yang diikuti |

Bukan rem mendadak: melambat linier mulai 50 cm, berhenti dan berputar di 20 cm. Kalau berbelok lebih dari `NAV_BELOK_BATAS_MS` (8 detik) tanpa jalan keluar, robot dianggap terjebak dan berhenti sendiri.

### 7.2 Kemudi terhadap dinding — DUA PITA, bukan satu rumus

```
jarak < wall.min (15 cm)  -> TERLALU DEKAT : putar menjauh kekuatan tetap + melambat
selebihnya                -> PD normal terhadap wall.setpoint (19 cm)
```

Pita dekat adalah tambahan; sebelumnya hanya ada PD. Alasannya di §7.9. Bacaan yang mustahil disaring satu lapis lebih bawah, di `LidarArray` — lihat §7.11.

**Pita PD:**

```
err  = jarakSamping − WALL_SETPOINT_CM            (+ artinya TERLALU JAUH dari dinding)
turn = sisi × ( WALL_KP · err  +  WALL_KD · ė )
```

`sisi = +1` untuk ikut dinding **kiri**, `−1` untuk **kanan**. Konvensi `turn` positif = CCW = **belok kiri**.

Cek tandanya untuk mode `F` (dinding kanan, `sisi = −1`): terlalu jauh → `err` positif → `turn` negatif → belok kanan → **mendekat ke dinding**. Benar. Terlalu dekat → tandanya membalik sendiri.

Kekuatan menjauh di pita dekat naik dari separuh di `wall.min` sampai penuh tepat di `LIDAR_MIN_CM` sensor itu — yaitu di jarak saat kaki sudah menyentuh dinding. Lebar ramp-nya ikut menyesuaikan kalau `wall.min` disetel, jadi tidak ada angka ketiga yang bisa lupa diubah.

Dengan gain sekarang (`WALL_KP` 0,008 / `WALL_KD` 0,030, setpoint 19 cm), keadaan mantap (`ė` = 0):

| jarak samping | pita | turn | arah |
|---|---|---|---|
| 3 cm | — | — | dipetakan ke `LIDAR_JAUH` oleh `LIDAR_MIN_CM` (§7.11) |
| 8 cm | dekat | **+0,50** | belok kiri — menjauh, laju maju −50% |
| 11 cm | dekat | +0,39 | menjauh, laju maju −29% |
| 15 cm | dekat | +0,25 | ambang, laju maju −0% |
| 19 cm | PD | 0,000 | lurus |
| 30 cm | PD | −0,088 | belok kanan — mendekat |
| 50 cm | PD | −0,248 | mendekat |
| ≥ 81,5 cm | PD | **−0,500** | kena batas `NAV_WALL_TURN_MAX` |

**Suku P** mengurus *di mana* robot berada. **Suku D** mengurus *seberapa cepat ia mendekat* — itu remnya, supaya tidak menabrak karena terlanjur. `ė` dalam cm/detik:

| laju perubahan jarak | sumbangan D |
|---|---|
| 1 cm/detik | 0,030 |
| 5 cm/detik | 0,150 |
| 10 cm/detik | 0,300 |

### 7.3 Turunan dihitung pada laju SAMPEL, bukan laju loop

Dua hal membuat `WALL_KD` dulu tidak melakukan apa yang terlihat dilakukannya:

1. **`getDistance()` membulatkan ke integer cm.** Selisih antar bacaan selalu kelipatan 1 cm, jadi turunannya berbentuk tangga.
2. **Pembaginya `dt` loop, dijepit di 0,001 s.** Loop Teensy jauh lebih cepat dari laju sampel LiDAR (~30 ms), jadi `err` tidak berubah di hampir semua iterasi (`ė` = 0) lalu melonjak sekali: satu lompatan pembulatan 1 cm bernilai **10,0 satuan putar**, langsung ter-clamp ke 1,00.

Itu bukan redaman, melainkan impuls satu-iterasi — yang kebetulan tak terasa karena `GAIT_SLEW_RATE` menyaringnya habis. Tidak berbahaya, tapi juga tidak berguna.

Sekarang `LidarArray` menyediakan `jarakHalus()` (nilai EMA float, tanpa pembulatan) dan `stempelSampel()`. Turunan dihitung **sekali tiap sampel LiDAR baru**, memakai jarak waktu antar sampel yang sebenarnya, lalu **ditahan** sampai sampel berikutnya — sinyal kontinu, bukan paku.

### 7.4 Kalau dinding hilang

```
samping == LIDAR_JAUH  ->  turn = sisi × NAV_CARI_CMD   (0,35)
```

Belok **ke arah** dinding dengan kekuatan tetap sampai ketemu lagi — ini yang terjadi di tikungan luar atau mulut lorong. Turunan di-reset supaya tidak melonjak saat dinding muncul kembali. Kalau `samping == LIDAR_MATI`, navigasi berhenti: tidak ada acuan untuk dikemudikan.

### 7.5 Batas, lalu ke gait

```cpp
turn = clamp(turn, ±NAV_WALL_TURN_MAX);            // ±0,50, SEMUA mode
if (arenaTerkunci()) turn += kemudiHeading(arahArena);
turn = clamp(turn, ±1,0);
_robot.walk(maju, 0.0f, turn);
```

Batas ±0,50 dulu hanya diterapkan di mode arena. Mode `f`/`F` polos justru yang tidak dibatasi — padahal itu mode tanpa acuan heading, yang paling butuh pagar. Dengan `WALL_KP` 0,008 kemudi P baru menjenuh pada jarak ~75 cm, sehingga batas ini kembali berfungsi sebagai pagar darurat, bukan pembatas utama.

### 7.6 Pemilihan gain

Dipilih lewat sweep di simulasi (lorong 60 cm, 40 detik per uji, dua titik awal, dengan derau sensor):

| kp | kd | RMS sisa dari tengah | paling dekat ke dinding | goyangan perintah/detik |
|---|---|---|---|---|
| 0,030 | 0,010 (lama) | 10,6 cm | **0,9 cm — menabrak** | 0,1 |
| 0,008 | 0,020 | 5,2 cm | 1,9 cm | 1,2 |
| **0,008** | **0,030** | **3,2 cm** | **5,7 cm** | 1,8 |
| 0,008 | 0,040 | 2,1 cm | 8,4 cm | 2,1 |

`kd` 0,040 sedikit lebih rapi, tapi goyangan perintahnya di kondisi berderau menembus 7 satuan/detik — di atas `GAIT_SLEW_RATE` 3,0, jadi gait tidak sanggup mengikutinya dan sisanya terbuang.

> **`CALIB_VERSION` wajib dinaikkan setiap default `PARAM_DEFS` diubah** (sekarang 9). Gain lama sudah terlanjur tersimpan di EEPROM alamat 0, dan blob versi lama tetap lolos CRC — tanpa kenaikan versi, robot memuat kembali nilai lama dan perubahan tidak berefek apa pun.

### 7.7 Sudut pasang sensor — kenapa 90° itu penting

> **Koreksi.** Versi terdahulu bagian ini menyimpulkan robot ini **tidak akan bisa** menyusul dinding dari tengah lorong, karena diasumsikan keenam LiDAR duduk di cincin 60° sehingga sensor sampingnya menyerong. **Asumsi itu salah.** Keempat LiDAR samping menghadap tegak lurus dinding — hanya `ch5` (depan) dan `ch2` (belakang) yang searah sumbu panjang. Baris yang berlaku untuk robot ini adalah baris 90°, dan baris itu berhasil.

Sweep yang sama dijalankan untuk tiga sudut pasang sensor samping:

| Sudut sensor dari depan | Menyusul dinding dari tengah lorong | Menjaga jarak setelah dekat |
|---|---|---|
| **90° (tegak lurus) — robot ini** | **berhasil**, RMS sisa 1,9 cm, kemudi tak pernah jenuh | baik (RMS 0,7 cm) |
| 75° | selalu menabrak, semua gain | baik |
| 60° | selalu menabrak, semua gain | baik |

Mekanisme kegagalan pada sensor menyerong adalah umpan balik positif, dan tidak ada gain yang bisa membalikkannya: sinar yang miring ke depan **memanjang** saat robot menoleh ke arah dinding — pada sinar 60°, menoleh 30° membuat bacaan hampir **dua kali lipat** padahal jarak tegak lurusnya tidak berubah. Kendali membaca "makin jauh" lalu menoleh lebih dalam.

Sinar **tegak lurus** juga memanjang saat robot menoleh (`d = d⊥ / cos θ`), tapi memanjang untuk **kedua arah toleh** — simetris, jadi tidak ada arah yang diuntungkan dan tidak ada umpan balik positif. Itulah sebabnya 90° berhasil dan 75° tidak.

Praktisnya untuk robot ini: mode `f`/`F` **boleh** dimulai dari tengah lorong. Tetap disarankan mulai kira-kira sejajar lorong, karena simpang heading awal yang besar tetap memperbesar bacaan lewat `1/cos θ`.

Tabel ini tetap disimpan sebagai peringatan: **jangan menyalin gain ini ke robot yang sensor sampingnya menyerong.**

### 7.8 Penjaga keselamatan

* `navMulai()` memeriksa **sensor yang benar-benar dipakai mode itu** sebelum melangkah, dan menyebut nama serta nomor channel-nya. Dulu robot mulai berjalan lalu berhenti satu iterasi kemudian — terlihat seperti menolak jalan tanpa sebab.
* Menolak mulai bila servo lemas atau mux LiDAR tak terdeteksi.
* Berhenti sendiri bila servo dilemaskan di tengah jalan.
* `s`, `x`, Enter, dan `w` semuanya membatalkan. Ini wajib: tanpa itu `navUpdate()` akan memerintahkan gerak lagi di iterasi berikutnya, sehingga robot tampak "menolak berhenti".

### 7.9 Jarak dinding: yang diukur sensor ≠ celah kaki

Saat uji fisik, kaki robot menggesek dinding walaupun sensor melaporkan jarak yang kelihatannya aman. Penyebabnya dua hal yang menumpuk, keduanya geometri.

**1. Yang menabrak dinding bukan badan, tapi kaki tengah.** Pangkal coxa kaki tengah ada di x = ±90 mm, ditambah `STAND_RADIUS` 70 mm → ujung kakinya **160 mm dari pusat badan**. Diukur di `sim_dinding` dari `HexaGait` yang asli, angka itu tidak bertambah saat berjalan maupun saat memutar di batas `NAV_WALL_TURN_MAX` — kaki tengah bergeser di sumbu Y, bukan X. Jadi 160 mm adalah lebar separuh yang sebenarnya.

**2. Sensor mengukur dari sisi badan, bukan dari ujung kaki.** Keempat LiDAR samping (ch0/ch1 kiri, ch3/ch4 kanan) menghadap **tegak lurus** dinding; hanya ch5 (depan) dan ch2 (belakang) yang searah sumbu panjang. Saat robot sejajar lorong:

```
x_dinding = x_sensor + jarak_terbaca
```

`x_sensor` diukur ~**50 mm** dari gambar tata letak (ch0/ch1 di 49,8 mm; ch3/ch4 di 52,0 mm). Artinya **kaki sudah menyentuh dinding saat sensor masih membaca 160 − 50 = 110 mm** — sensor tidak akan pernah melaporkan angka lebih kecil dari itu untuk dinding sungguhan.

Celah ujung kaki ke dinding (dihitung `sim_dinding`):

| setpoint terbaca | x_sensor 4,0 cm | x_sensor 5,0 cm | x_sensor 6,5 cm |
|---|---|---|---|
| 13 cm (lama) | +1,0 cm | **+2,0 cm** | +3,5 cm |
| 15 cm | +3,0 cm | +4,0 cm | +5,5 cm |
| 17 cm | +5,0 cm | +6,0 cm | +7,5 cm |
| **19 cm (default baru)** | +7,0 cm | **+8,0 cm** | +9,5 cm |
| 21 cm | +9,0 cm | +10,0 cm | +11,5 cm |

Pada setelan lama celahnya cuma 2 cm — satu ayunan kemudi cukup untuk menyentuh.

**Kenapa PD saja tidak cukup.** `wall.kp` 0,008 sengaja lembut supaya kemudi tidak menjenuh saat dinding jauh (§7.5). Konsekuensinya, berada 10 cm **terlalu dekat** hanya menghasilkan koreksi 0,08 dari 1,00 — nyaris tak terasa. Satu gain proporsional tidak bisa memenuhi dua kebutuhan yang berlawanan, jadi respons dekat dipisah jadi pitanya sendiri (§7.2): di bawah `wall.min` robot memutar menjauh dengan kekuatan tetap **dan** memperlambat laju maju, supaya kemudi sempat bekerja sebelum kaki sampai ke dinding.

**Bacaan yang mustahil** tidak lagi diurus di sini — lihat §7.11.

Diukur `sim_dinding` (lorong 60 cm, mulai dari posisi yang **sudah** terlalu dekat, derau ±1 cm, 40 detik):

| aturan | celah kaki min sesudah pulih | celah kaki rata |
|---|---|---|
| lama (satu PD, setpoint 13) | 1,8 cm | 2,0 cm |
| **baru (tiga pita, setpoint 19)** | **6,9 cm** | **8,7 cm** |

Dengan hantu 3 cm disuntikkan: tanpa ambang, celah rata membengkak ke 18,8 cm (robot kabur ke seberang lorong); dengan ambang 8 cm, tetap 6,0 cm.

**Cara menyetel yang paling tepat**, tanpa perlu menebak `x_sensor`: berdirikan robot di samping dinding, atur dengan tangan sampai celah ujung kaki tengah ke dinding sesuai selera (mis. 8 cm), lalu baca `L`. Angka sensor samping saat itu **adalah** `wall.setpoint` yang benar. Setel dengan `Qwall.setpoint <angka>` (berlaku langsung, tidak perlu `b`), lalu `W` untuk menyimpan.

### 7.10 Bisakah yaw IMU membantu? Ya — tapi bukan sebagai kompas

Mode `f`/`F` sekarang **sama sekali tidak memakai IMU**: kemudinya PD murni pada jarak LiDAR. Pertanyaannya wajar — apakah yaw bisa membantu? `sim_yaw` membandingkan tiga arsitektur pada plant lorong yang sama persis, memakai `LidarArray` dan `Imu` yang asli:

| | Hukum kendali |
|---|---|
| **A** (sekarang) | `turn = sisi × (kp·err + kd·ė)` langsung pada jarak terbaca |
| **B** | sama, tapi jarak dikoreksi dulu: `d⊥ = d_terbaca × cos θ` |
| **C** | **kaskade** — jarak jadi lingkar LUAR yang lambat (menghasilkan sudut hadap yang diminta), heading + giro jadi lingkar DALAM yang cepat |
| **D** | P pada jarak, redaman dari **giro saja** — tidak menyentuh yaw sama sekali |
| **E** | seperti sekarang (P dan D pada jarak) **ditambah** redaman giro |

Acuan arah lorong **tidak** diambil dari kompas arena. Ia ditaksir dari LiDAR sendiri:

```
d(d⊥)/dt = −v · sin θ        ->   θ_amatan = −asin( ḋ⊥ / v )
acuan     = yaw − θ_amatan   (ditapis, tau ~4 detik)
```

Yaw dipakai sebagai **acuan relatif**, bukan sebagai arah mata angin. Bedanya besar — lihat skenario 5.

> **Catatan jujur:** rancangan pertama memakai rata-rata bergerak dari yaw saja sebagai acuan. Itu **gagal** di skenario 3 — mulai menyerong 20°, acuannya ikut mengunci 20° yang salah, tidak ada yang mengoreksi, kaki menabrak (RMS 8,0 cm). Rata-rata yaw tidak punya acuan luar. Penaksir di atas punya: dinding sendiri yang jadi acuannya.

Hasil (lorong 60 cm, 40 detik per uji, RMS diukur dari posisi badan yang SEJATI):

RMS sisa (cm), diukur dari posisi badan yang SEJATI:

| skenario | A | B | **C** | D | E |
|---|---|---|---|---|---|
| 1. menjaga jarak, derau ±1 cm | 0,12 | 0,10 | 0,33 | 0,12 | 0,09 |
| 2. menyusul dari tengah lorong | 0,71 | 0,59 | **0,34** | 4,45 ✗ | 0,85 |
| 3. mulai menyerong 20° | 1,70 | 1,71 | **1,06** | 6,05 ✗ **kaki menyentuh** | 2,33 ✗ **kaki menyentuh** |
| 4. sensor berderau ±3 cm | 1,53 | 1,27 | **1,25** | 0,19 | 1,02 |
| 5. kompas meleset tetap 15° | 0,16 | 0,07 | 0,30 | 0,13 | 0,12 |
| goyangan perintah/detik | 5–9 | 5–9 | **1,6–1,8** | **0,1** | 6,7–11,5 |
| kemudi jenuh, skenario 4 | 32% | 31% | **0%** | **0%** | 27% |

**C adalah satu-satunya yang tidak pernah kalah.** Yang lain masing-masing punya lubang:

* **Koreksi kosinus (B) hampir tidak berguna sendirian.** Pada sudut menyerong yang wajar inflasinya kecil — 20° hanya 6,4%, yaitu 1,2 cm pada setpoint 19 cm. Sudah di bawah derau sensor.
* **Giro saja (D) menang telak saat MENJAGA, dan gagal total saat MENYUSUL.** Goyangan perintahnya cuma 0,1/detik dan kekebalan derau sensornya terbaik dari semuanya — tapi dari tengah lorong ia tidak pernah sampai, dan dari posisi menyerong ia menabrak. Disapu di `k_giro` 0,000 sampai 0,008: **gagal di semua nilai, termasuk nol**. Jadi bukan redamannya yang kekencangan — P saja memang tidak punya **antisipasi**. Jarak ada dua integrasi di belakang perintah putar (`turn → laju yaw → sudut → laju jarak → jarak`); giro meredam *sudut*, dan itu integrasi yang salah untuk meredam *jarak*.
* **Menambah giro ke PD yang ada (E) justru memperburuk.** Redaman giro berebut dengan suku D jarak: goyangannya naik ke 6,7–11,5/detik, kemudi masih menjenuh 27%, dan di skenario menyerong kakinya tetap menyentuh. Dua peredam pada integrasi yang berbeda saling melawan.
* **Kenapa C berhasil sementara D dan E tidak:** kaskade mengubah *tugas* lingkar jarak. Ia tidak lagi memilih laju putar (dua integrasi jauh dari jarak), melainkan memilih **sudut hadap** — satu integrasi. Lingkar orde satu jauh lebih mudah distabilkan, dan redaman gironya lalu bekerja di tempat yang benar, yaitu lingkar dalam.
* **Yang benar-benar menang adalah kaskade (C), dan alasannya redaman.** Suku D sekarang tidak lagi menurunkan sinyal jarak yang berderau; redamannya datang dari **giro**, yang bersih dan berlaju tinggi. Di skenario sensor berderau, C tidak pernah menjenuhkan kemudi (**0%** vs **33%**) dan goyangan perintahnya seperlima.
* **Goyangan itu bukan soal kosmetik.** `GAIT_SLEW_RATE` 3,0 satuan/detik: perintah yang bergoyang lebih cepat dari itu tidak pernah sampai ke kaki. A menghasilkan 5–9 goyangan/detik — sebagian besar usahanya terbuang di penapis gait. C menghasilkan 1,6.
* **Kompas yang meleset TETAP tidak berpengaruh** (skenario 5), karena acuannya ditaksir ulang terus-menerus dan meleset itu ikut terserap. Ini menjawab kekhawatiran paling umum soal memakai magnetometer di dekat motor dan rangka besi.

**Satu syarat keras — derau yaw:**

| derau yaw | goyang A | goyang C | keterangan |
|---|---|---|---|
| ±0,0° | 5,1 | 0,3 | gait sanggup ikut |
| ±0,5° | 5,5 | 1,6 | gait sanggup ikut |
| ±1,0° | 5,6 | 3,0 | pas di `GAIT_SLEW_RATE` |
| ±2,0° | 5,6 | 5,9 | **C sudah lebih kasar dari A** |
| ±5,0° | 5,6 | 14,5 | jauh lebih kasar |

Derau yaw masuk langsung ke lingkar dalam lewat `HEADING_KP`. RMS-nya tetap baik sampai ±5° (0,19–0,35 cm) — jadi ini soal kehalusan dan efisiensi, bukan kestabilan. **Ambangnya ~±1°.** Cek dengan aliran `y` pada robot yang berdiri diam, lalu ulangi sambil berjalan — motor dan rangka arena adalah sumber gangguan magnet yang nyata.

**Kesimpulan: kaskade (C), atau tidak sama sekali.** Tidak ada jalan tengah yang murah — D dan E sudah membuktikan itu.

**Belum dipasang di firmware,** dan tidak boleh dipasang sebelum satu angka diukur: **derau yaw di robot sungguhan, sambil berjalan.** Di bawah ±1° kaskade jelas menang; di atas ±2° ia justru lebih kasar dari yang sekarang. Angka itu belum pernah diukur, jadi keputusannya belum bisa diambil dari simulasi saja.

Kalau nanti dipasang, ia membawa tiga kewajiban baru: mode `f`/`F` jadi bergantung pada IMU yang hidup (sekarang tidak), lingkar luar butuh angka **laju maju dalam cm/detik** (tersedia dari odometri `GerakStore`, tapi berarti kalibrasi `C`+`S` jadi syarat), dan ada 3–4 parameter baru untuk disetel. Sebaiknya dipasang sebagai **mode terpisah** (huruf baru), bukan menimpa `f`/`F` yang sudah teruji, supaya keduanya bisa di-A/B di robot sungguhan.

### 7.11 Hantu sensor DEPAN — robot berbelok di lorong yang kosong

Gejala lapangan: sedang ikut dinding, di depan tidak ada apa-apa, robot tiba-tiba berbelok ke kiri.

Rantai sebabnya pendek dan seluruhnya bisa dilacak:

1. Sensor robot ini melaporkan **"tak ada objek dalam jangkauan" sebagai bacaan ~5 cm berstatus `RangeValid`** — bukan sebagai `SignalFail`. Ini diketahui dari uji fisik: jangkauan akurat ~70 cm, dan di luar itu angkanya jatuh ke 5 cm, bukan membesar.
2. `depan = 5` lolos semua penyaring dan masuk ke cabang `depan <= FRONT_STOP_CM (20)`.
3. Cabang itu menjalankan `turn = −sisi × NAV_BELOK_CMD`. Untuk mode `F` (dinding kanan, `sisi = −1`) hasilnya **+0,60 = berputar ke kiri**. Persis yang terlihat.

`wall.hantu` (v8) tidak menolong: ia hanya dipasang di jalur sensor **samping**. Sensor depan tidak pernah terlindungi.

**Perbaikannya pindah satu lapis ke bawah,** ke `LidarArray`, lewat `LIDAR_MIN_CM[]` di `config.h` — batas bawah **per arah**, diturunkan dari jangkauan kaki:

| arah | ujung kaki | dudukan sensor | mustahil di bawah | `LIDAR_MIN_CM` |
|---|---|---|---|---|
| samping (ch0,1,3,4) | 160 mm | 50 mm | 110 mm | 10 cm |
| depan (ch5) | 139 mm | 62 mm | 77 mm | 7 cm |
| belakang (ch2) | 139 mm | 66 mm | 73 mm | 7 cm |

Bacaan di bawah batas itu dipetakan ke **`LIDAR_JAUH`**, bukan ditolak begitu saja — karena di sensor ini bacaan pendek memang **berarti** kosong. Ini bukan "menyaring pakai ambang jarak" seperti kode VL53L0X lama yang sudah dibuang: ambangnya menyatakan sesuatu yang **mustahil secara fisik**, bukan sesuatu yang sekadar tampak aneh.

Gerbangnya sama dengan gerbang median: butuh **tiga sampel berturut-turut** sebelum keadaan "jauh" diumumkan. Tanpa itu, satu bacaan pendek yang menyimpang saat dinding sungguhan sedang terlihat bisa membalik sensor jadi "kosong" — arah kesalahan yang paling berbahaya untuk sensor depan.

`LIDAR_MAX_CM` ikut turun **120 → 70 cm**, sesuai jangkauan akurat yang terukur. Angka lama tidak pernah menolong, karena mekanisme gagalnya memang bukan "angka membesar".

Diuji di `sim_depan`:

| uji | hasil |
|---|---|
| lorong depan kosong (hantu 5 cm) | depan = jauh, `turn` **+0,000**, maju penuh 0,80 |
| halangan sungguhan 15 cm | depan = 15 cm, `turn` +0,599, maju 0,000 — **tetap menghindar** |
| satu sampel pendek menyimpang saat dinding 40 cm | bertahan di 40 cm |
| hantu menetap | baru berubah jadi "jauh" |
| dinding 45 / 65 / 75 / 90 cm | terbaca / terbaca / jauh / jauh |

`wall.hantu` **dibuang** (`CALIB_VERSION` → 9): pekerjaannya sudah diambil alih dan menyimpannya di dua tempat hanya mengundang keduanya berbeda.

#### Akibat lanjutan yang ikut ketemu: mencari dinding tanpa batas waktu

Sesudah perubahan di atas, hantu di sensor **samping** juga jadi `LIDAR_JAUH` = "dinding hilang", yang menjalankan pencarian `turn = sisi × NAV_CARI_CMD`. Kalau dindingnya tidak pernah muncul lagi — sensor samping macet di hantu, misalnya — perintah putar tetap itu membuat robot **berjalan melingkar selamanya** di tengah arena. Ditangkap oleh uji 4 `sim_dinding`, bukan oleh pembacaan kode.

`NAV_CARI_BATAS_MS` (10 detik ≈ 86 cm perjalanan) menghentikannya dengan pesan yang menyebut sensornya. Hanya berlaku di mode `f`/`F` polos — di mode arena `kemudiHeading()` yang mengunci arah, jadi dinding hilang di sana tidak membuatnya melingkar.

#### Bisakah LiDAR belakang dipakai sebagai odometri?

Bisa, tapi terbatas, dan **bukan** pengganti odometri gait:

* **Jangkauannya cuma 70 cm.** Sesudah berjalan 70 cm dari dinding belakang, sensornya jatuh ke hantu dan angkanya habis. Jadi ia mengukur "sudah berapa jauh dari dinding di belakang", bukan "sudah berapa jauh berjalan".
* **Butuh dinding di belakang.** Di tengah lorong panjang tidak ada acuan.
* **`ch2` sudah hidup** sejak September 2026, jadi ide ini akhirnya bisa diuji — tapi dua keberatan di atas tetap berlaku.

Yang sudah tersedia dan lebih murah: **odometri gait**, sudah berjalan di firmware ini (`HexaGait::_jarakMm`, dibaca lewat perintah `D` — lihat §12), bukan lagi sekadar ide dengan rumus lurus sederhana. Tiap tick gait, `HexaGait::update()` menambah:

```
jarak += skala x min(2, 1/duty) x maju x step_length x f x (dt / cycle_time)
```

* **`maju`** — komponen maju perintah (`_curY`, sudah di-slew, bukan `_tgtY` mentah).
* **`min(2, 1/duty)`** — koefisien 2,0 dari rumus lama HANYA benar saat `gait.duty` = 0,5 (jendela tumpu kedua tripod pas menutupi satu siklus tanpa celah/tindih). Di atas 0,5 jendela tumpunya saling tindih, jadi badan sungguh maju `sy/duty` per siklus, bukan `2 x sy` — dan `gait.duty` bisa disetel operator kapan saja lewat `Qgait.duty` (rentang sah 0,3 .. 0,7), jadi koefisiennya harus ikut duty, bukan angka tetap.
* **`f`** — faktor normalisasi langkah yang sama dipakai saat badan maju sambil berputar (bagian 4 di `HexaGait::update()`, mencegah servo terbakar); tanpa ini jarak susur-dinding terhitung lebih jauh dari jarak lurus padahal bukan slip.
* **`dt / cycle_time`** — dihitung dari `dt` dan `cycleTime` yang SAMA (dengan clamp yang sama) yang dipakai fase gait itu sendiri untuk melangkah, jadi odometer tak pernah berselisih dari seberapa jauh kaki sungguh maju.

Angka mentah itu adalah geometri, bukan jarak sebenarnya — selisihnya dengan meteran adalah slip mekanis, dan itu disetel lewat **faktor skala** `Ds<faktor>` (0,5 .. 1,5, default 1,0). Faktor ini sengaja **hanya di RAM**, bukan `Calib`: menambah satu `float` ke `CalibBlob` menaikkan `CALIB_VERSION` dan membuang seluruh gain yang sudah disetel di EEPROM, harga yang tidak sepadan sebelum angkanya diketahui dari lapangan. Begitu diketahui, tulis sebagai konstanta di `config.h`.

`D<cm>` memasang **rem jarak**: begitu jarak tempuh melewati `<cm>`, navigasi berhenti otomatis (mode manapun, termasuk `w` manual dan `f`/`F` — cocok untuk mengukur slip jalan lurus maupun slip susur-dinding dengan alat yang sama). Itu berlaku di mana saja, tanpa dinding, tanpa batas 70 cm. Kelemahannya tetap slip — dan justru di situlah LiDAR belakang berguna: sebagai **koreksi absolut jarak dekat** saat dinding belakang masih terlihat, mis. untuk tahu sudah berapa jauh melewati satu tikungan. Peran itu wajar; peran "odometer utama" tidak.

---

## 8. Pivot & kompas arena

### Pivot non-blokir

`pivotKe()` dulu memblokir loop utama sampai **20 detik**. Selama itu parser serial mati total: `x` tidak bisa melemaskan servo, dan satu-satunya rem adalah jalan keluar "tekan Enter" yang ditanam di dalam loop pivot itu sendiri — jalur berhenti kedua yang harus diingat terpisah.

Sekarang pivot adalah **mode navigasi biasa**, `NAV_PIVOT`, dengan dua fase:

| Fase | Isi |
|---|---|
| `FASE_PIVOT` | Kendali PD tertutup menuju `_pivotTarget`, satu langkah per `navUpdate()` |
| `FASE_SETTLE` | Perintah putar sudah nol, menunggu `PIVOT_SETTLE_MS` (800 ms) sampai kaki *settle* |

`pivotKe()` kini hanya **memulai** lalu langsung kembali. Konsekuensinya: `s`/`x`/Enter membatalkan lewat `navBerhenti()` yang sama; perintah lain tetap bisa dipakai selagi robot berputar; IMU yang lepas di tengah pivot menghentikan pivot (dulu yaw membeku dan robot berputar sampai timeout); dan pivot tidak lagi menuntut LiDAR.

### Satu rumus, bukan dua

Fase belok arena sebenarnya sudah menjalankan pivot yang sama, tapi rumusnya ditulis ulang dengan **gain berbeda**: fase belok memakai `HEADING_KP`/`HEADING_KD` dari `Calib`, sedangkan `pivotKe()` memakai `PIVOT_KP`/`PIVOT_KD` dari `config.h`. Nilai defaultnya kebetulan persis sama, jadi tidak pernah terlihat — tapi menyetel yang satu diam-diam tidak menyentuh yang lain.

Keduanya sekarang lewat `pivotLangkah()`, dan `PIVOT_KP`/`PIVOT_KD` **dihapus** dari `config.h`:

```
err  = wrap180(target − yaw)
turn = clamp( pivotSign × ( HEADING_KP · err − HEADING_KD · gyroZ ), ±1 )
kalau |err| > HEADING_TOLERANCE_DEG dan |turn| < PIVOT_MIN_CMD:
    turn = ±PIVOT_MIN_CMD          (dorongan minimal, mengikuti tanda turn)
```

Suku D diambil murni dari gyro Z, bukan dari selisih yaw — turunannya jadi tidak berisik.

Selesai bila `|err| ≤ HEADING_TOLERANCE_DEG` (6°) **dan bertahan** selama `PIVOT_DIAM_MS` (500 ms). Histeresis itu mencegah berhenti karena kebetulan melintas.

### `_pivotSign` — kenapa kalibrasi `C` itu wajib

`_pivotSign` menyatakan apakah perintah putar menaikkan atau menurunkan pembacaan yaw. Nilainya bergantung arah pemasangan IMU dan **tidak bisa ditebak dari kode**.

Mode arena **menolak** jalan tanpa kalibrasi; `o`/`O` hanya **memperingatkan**. Bedanya gejala: pivot dengan tanda terbalik ketahuan sendiri lewat timeout 20 detik, sementara di fase jalan mode arena tanda yang salah hanya melengkungkan lintasan diam-diam tanpa gejala apa pun.

### Mode terkunci arena (`p` / `P`)

Lorong arena sejajar sumbu mata angin, jadi heading arena dipakai sebagai acuan **sudut** dan dinding sebagai koreksi **lateral**. Keduanya tidak berebut.

Saat mentok, robot berbelok ke mata angin berikutnya, bukan berputar buta: ikut dinding kiri → belok kanan → indeks arah +1 (Utara→Timur→Selatan→Barat); ikut dinding kanan → −1. Beloknya kendali tertutup dan tetap non-blokir.

Prasyarat: keempat arah arena sudah dicatat (`c0`–`c3` lalu `e`, atau `E` untuk memuat), dan pivot sudah dikalibrasi. Saat `p` ditekan, robot mengunci ke arah arena **terdekat** dari hadapnya sekarang — jadi arahkan robot kira-kira sejajar lorong lebih dulu.

---

## 9. Lengan

Dua lengan, masing-masing **BAHU, SIKU, GRIP**. Kedua sendi pertama menyapu satu bidang **vertikal** yang menghadap keluar dari badan; tidak ada sendi pemutar di pangkal, jadi untuk membidik objek yang tidak segaris, **badan robot** yang harus diarahkan. Karena itu IK 2 link memang bentuk yang tepat.

Lengan dipasang **depan & belakang**, bukan kanan & kiri — `ARM_ORIGINS` menaruh offset di sumbu **Y** (`{0, +50, 30}` dan `{0, −50, 30}`). Slot kalibrasi: depan `18`/`19`/`20`, belakang `21`/`22`/`23`. Sebelumnya kedua lengan sama-sama memakai slot 18–20, jadi lengan kedua diam-diam meminjam offset/trim milik yang pertama.

Target di luar jangkauan **ditolak** — sudutnya tidak dikirim sama sekali. IK memeriksa batas **luar** (`UPPERARM + FOREARM`) dan batas **dalam** (`|UPPERARM − FOREARM|`, lingkaran mati di sekitar bahu yang tak bisa dicapai betapapun siku ditekuk).

**Masih perlu diverifikasi fisik:** papan PCA mana yang memegang lengan depan (kalau `a` vs `A` tertukar, tukar saja dua baris `ARM_PIN_MAP_*` di `config.h`); jarak pangkal bahu dari pusat badan (50 mm) dan tingginya (30 mm); serta `UPPERARM_LENGTH`/`FOREARM_LENGTH` yang masih angka percobaan.

---

## 10. Peta EEPROM

| Alamat | Isi | Ditulis oleh | Dibaca firmware |
|---|---|---|---|
| 0 | `CalibBlob` (~272 B) — param, offset, trim, invert | firmware sendiri | ✅ `Calib::load()` |
| 1024 | `ServoMap` (126 B) — invert & trim hasil uji fisik | TES_SERVO / SET_HOME | ✅ `Hexapod::loadServoMap()` |
| 1792 | `KompasStore` (24 B) — 4 arah arena | TES_IMU / `Navigation` | ✅ `Navigation::kompasMuat()` |
| 2048 | `GerakStore` (80 B) — pivot, odometri, rata badan, `zOff` | TES_GERAK | ✅ `Hexapod::loadZOff()` |

Alamat sengaja berjauhan: keempat blok ditulis empat program yang tidak pernah dikompilasi bersama, jadi **alamat tetap itulah kontraknya**, dan celah di antaranya adalah ruang tumbuh. Total terpakai hanya 502 B dari 4284 B.

`ServoMap` di 1024 **menang atas** default `Calib`, karena itulah data yang benar-benar diukur di robot. `drv[]`/`ch[]` sengaja **tidak** diimpor: penomoran driver di `servo_map.h` kebalikan dari `HexaServos`, dan `SERVO_PIN_MAP` sudah menunjuk servo fisik yang sama — menyalinnya justru akan menukar sisi kiri-kanan.

### `EEMap.h` — satu definisi, dijaga saat kompilasi

Dulu struct-nya disalin ke beberapa file. Salinan yang bisa menyimpang diam-diam dari program penulisnya adalah kerapuhan terbesar di sini — checksum tetap lolos, tapi field-nya salah baca. Sekarang ketiganya hanya ada di `EEMap.h`, dengan dua lapis penjaga:

```cpp
static_assert(sizeof(GerakStore) == 80, "GerakStore berubah -- samakan dgn TES_GERAK.ino");
static_assert(EE_CALIB_ADDR + sizeof(CalibBlob) <= EE_SERVOMAP_ADDR,
              "CalibBlob tumbuh melewati ServoMap (1024) -- geser EE_SERVOMAP_ADDR");
```

Sudah diuji dengan sengaja merusaknya — keduanya **menggagalkan kompilasi** dengan pesan yang menyebutkan file mana yang harus disamakan. Ini menggantikan kerusakan senyap saat runtime dengan error saat build.

Perintah **`M`** mencetak peta dan membandingkan `EE_TOTAL_BYTES` dengan `EEPROM.length()` yang sebenarnya — penangkap kalau firmware dipindah ke papan dengan EEPROM lebih kecil.

Pembacaan `GerakStore` **memverifikasi checksum**, bukan hanya magic + versi. Tanpa itu, EEPROM yang separuh tertulis lolos dan `zOff` terisi sampah.

Penulisan `S` bersifat **baca-ubah-tulis**: hanya `ccw`, `cw`, `maju`, `sign` yang diganti — `zoff[6]`, `lvlR`/`lvlP`, `refR`/`refP`, dan `jac[4]` milik TES_GERAK dipertahankan utuh.

---

## 11. Prosedur bring-up & kalibrasi

Serial Monitor 115200 baud, mode *Newline*.

**Langkah 0 — periksa dulu, jangan berdiri dulu.**
Baca log boot. Semua enam LiDAR OK? `Calib` valid? `ServoMap` termuat? Lalu `I` (pindai + pulihkan LiDAR) dan `l` (tabel jarak). Geser tangan di depan tiap sensor dan pastikan nama arahnya cocok dengan posisi fisik — seluruh navigasi bergantung pada itu.

**Langkah 1 — kalibrasi kaki (`TES_GERAK`, program terpisah).**
Karena sagging servo, posisi berdiri awal biasanya membuat ada kaki menggantung. Cari `zOff` per kaki dengan `TES_GERAK.ino`, simpan ke EEPROM 2048 dengan `W`. Firmware ini membacanya otomatis saat boot.

**Langkah 2 — berdiri & diagnostik.**
Topang robot, ketik `b`. Lalu `d` untuk memeriksa seluruh rantai IK per kaki. Kolom `rng` harus `ok` semua.

**Langkah 3 — body kinematics.**
`B` untuk demo sapuan 6 sumbu. Telapak tidak boleh menyeret di lantai. Periksa arah tiap sumbu dengan `r`/`t` sebelum menyalahkan gait.

**Langkah 4 — kompas arena.**
Hadapkan robot ke Utara, hidupkan aliran `y`, tunggu angkanya tenang, baru `c0`. Ulangi `c1`–`c3`. Simpan dengan `e`.

**Langkah 5 — kalibrasi pivot.**
`C` (memblokir, biarkan selesai) lalu **`S`** untuk menyimpan. Tanpa `S` hasilnya hilang saat reset. Periksa dengan `K`.

**Langkah 6 — uji pivot.**
`o1` lalu amati kolom simpang di aliran `y`. Harus di dalam 6°.

**Langkah 7 — navigasi.**
Setel dulu jarak dindingnya (§7.9): berdirikan robot di samping dinding, atur dengan tangan sampai celah ujung kaki tengah ~8 cm, baca `L`, lalu `Qwall.setpoint <angka sensor samping>` dan `W`. Baru letakkan robot **di jarak setpoint itu** dan kira-kira sejajar lorong. `F` untuk ikut dinding kanan. Tangan tetap di `x` — tidak ada sensor tabrakan samping.

---

## 12. Daftar perintah serial

### Keselamatan & diagnostik
| | |
|---|---|
| `x` | **LEMAS** — semua PWM mati, servo bebas. Rem paling aman |
| `d` | Dump diagnostik: status PWM, sumber ServoMap & zOff, panjang link, profil gait, lalu per kaki `zOff`, input IK, sudut, flag invert, sudut servo, pulse akhir, dan status jangkauan |
| `h` | Bantuan |
| `M` | Peta EEPROM + kapasitas chip sebenarnya |

### Parameter kalibrasi
| | |
|---|---|
| `q` | Daftar semua parameter: nilai, default, rentang sah, kapan berlaku |
| `q<nama>` | Lihat satu parameter (nama boleh disingkat, mis. `qwall.kp`) |
| `Q<nama> <nilai>` | Ubah parameter — berlaku di RAM seketika |
| `W` | Simpan seluruh blok ke EEPROM 0 |

### Gerak dasar
| | |
|---|---|
| `b` | Berdiri diam (sekaligus menyalakan servo bila masih lemas) |
| `b<mm>` | Sama, sekaligus atur tinggi badan (40–160, di-ramp) |
| `w` | Jalan maju |
| `s` | Stop (servo tetap hidup) |
| Enter kosong | Rem darurat |

### Body kinematics
| | |
|---|---|
| `r` | Cetak pose badan sekarang |
| `r<roll> <pitch> <yaw>` | Set rotasi, derajat. roll+ = miring KANAN, pitch+ = MENDONGAK, yaw+ = belok KIRI |
| `t<x> <y> <z>` | Set geser badan, mm |
| `0` | Nolkan pose badan |
| `B` | Demo sapuan 6 sumbu (18 detik) |
| `z` | Goyang roll bergelombang, terus-menerus — untuk pajangan |
| `z<amp> <periode> <fase>` | Amplitudo derajat, periode detik, fase pitch derajat |

### LiDAR
| | |
|---|---|
| `l` | Tabel jarak keenam sensor |
| `L` / `L<ms>` | Hidup/matikan aliran (50–5000 ms, default 300) |
| `I` | Pindai bus I2C **dan init ulang** sensor yang mati |

### Navigasi
| | |
|---|---|
| `f` / `F` | Ikut dinding KIRI / KANAN — murni LiDAR |
| `p` / `P` | Ikut dinding KIRI / KANAN + terkunci kompas arena |
| `v` | Status navigasi/pivot + jarak sekitar |
| `s` / `x` / Enter | Hentikan |
| `D` | Cetak jarak tempuh (odometer gait), status rem jarak, dan skala slip |
| `D<cm>` | Pasang rem jarak — berhenti otomatis (dan misi gagal dengan bersih bila sedang berjalan) begitu jarak tempuh melewati `<cm>`. Berlaku di mode manapun, termasuk `w` manual dan `f`/`F` |
| `D0` | Lepas rem jarak dan nolkan jarak tempuh |
| `Ds<faktor>` | Setel faktor skala slip odometri, RAM saja (0,5 .. 1,5) — lihat §7 |

### Kompas & pivot
| | |
|---|---|
| `c[0-3]` | Catat arah arena (0=U, 1=T, 2=S, 3=B) |
| `k` | Tabel kompas |
| `e` / `E` | Simpan / muat kompas dari EEPROM 1792 |
| `y` / `y<ms>` | Aliran yaw + simpangan ke arah arena terdekat |
| `o[0-3]` | Pivot ke arah arena (non-blokir) |
| `O<derajat>` | Pivot relatif (non-blokir) |
| `C[siklus]` | Kalibrasi pivot — **masih memblokir** sampai selesai |
| `S` | Simpan hasil `C` ke EEPROM 2048 |
| `K` | Tabel kalibrasi gerak |

### Lengan
| | |
|---|---|
| `a<jangkauan> <tinggi>` | Lengan depan, mm dari pusat badan |
| `A<jangkauan> <tinggi>` | Lengan belakang |
| `g<0-100>` / `G<0-100>` | Grip depan / belakang (0 = menutup) |
| `n` | Matikan servo kedua lengan |

---

## 12b. Menyetel parameter tanpa kompilasi ulang

`Calib::setParam()`, `findParam()`, dan `save()` sudah lama ada tapi **tidak pernah dipanggil siapa pun**. Akibatnya tiap penyetelan gain menuntut edit `PARAM_DEFS`, kompilasi ulang, **dan** kenaikan `CALIB_VERSION`. Sekarang ketiganya tersambung ke `q` / `Q` / `W`.

```
>>> qwall
Awalan 'wall' ambigu:
   wall.kp
   wall.kd
   wall.setpoint
   wall.min

>>> Qwall.kp 0.012
wall.kp : 0.008 -> 0.012
  Masih di RAM. Ketik 'W' supaya bertahan sesudah reset.
```

Nama boleh disingkat selama awalannya unik — kalau ambigu, kandidatnya dicetak.

### Kapan sebuah parameter benar-benar berlaku

Ini yang paling mudah membingungkan: tidak semua parameter berpengaruh seketika. `ParamDef` sekarang membawa kolom `berlaku`, dan `Q` mencetaknya:

| Golongan | Isi | Perilaku |
|---|---|---|
| `langsung` | `wall.*`, `heading.*`, `gait.duty`, `gait.slew_rate`, `gait.*_tau` | dibaca tiap loop — efeknya seketika |
| `ketik 'b'` | `gait.step_height`, `gait.step_length`, `gait.cycle_time` | baru masuk lewat `profileFlat()` |
| `servo lemas` | `pulse.min/max`, `arm.pulse.min/max` | lihat di bawah |
| `belum dipakai` | `stab.*`, `head.*`, `arena.mirror` | slotnya ada, kodenya belum membaca |

Tanpa kolom ini, mengubah `gait.step_height` lalu melihat robot tidak berubah apa-apa terlihat seperti perintahnya gagal.

### Dua penjaga

**Clamp tidak lagi diam-diam.** `setParam()` memangkas ke `[lo,hi]` tanpa memberi tahu. Perintahnya sekarang membandingkan yang diminta dengan yang tersimpan:

```
>>> Qwall.kp 99
wall.kp : 0.012 -> 0.100
  (diminta 99.000, DI-CLAMP ke rentang sah 0.000 .. 0.100)
```

**`pulse.*` ditolak selagi servo aktif.** Mengubah pemetaan sudut→pulse menggeser 18 servo sekaligus tanpa ramp — kelas bahaya yang sama dengan lonjakan pose badan di bagian 5, dan di sini tidak ada peredam apa pun:

```
>>> Qpulse.min 700
Ditolak: 'pulse.min' menggeser semua servo sekaligus tanpa ramp.
        Ketik 'x' (lemas) dulu, ubah, lalu 'b' lagi.
```

### Bug yang baru ketahuan gara-gara ini

Menguji `W` membuka sesuatu yang lebih besar: **blok kalibrasi di EEPROM 0 tidak pernah sekali pun berhasil dimuat.**

```cpp
gCalib.crc = crc16(&gCalib, sizeof(CalibBlob) - sizeof(gCalib.crc));   // SALAH
```

`sizeof(CalibBlob)` = 272, tapi field `crc` ada di byte **268–269** — ada 2 byte padding di ekor struct. Jadi `272 − 2 = 270` berarti hash ikut **menelan field `crc` itu sendiri**. `save()` mem-hash crc lama lalu menimpanya dengan crc baru; `load()` mem-hash rentang yang sama yang kini berisi crc baru, mendapat angka berbeda, dan **selalu menolak**.

Gejalanya cuma satu baris di boot — `"Calib: EEPROM kosong/rusak/versi beda -> default dipakai & ditulis ulang"` — yang gampang dikira normal. Padahal artinya tiap boot jatuh ke `applyDefaults()` lalu menulis ulang 272 byte, dan seluruh disiplin `CALIB_VERSION` selama ini tidak pernah sempat teruji karena blob-nya memang tak pernah dimuat.

Perbaikannya `offsetof(CalibBlob, crc)`. `ServoMap` kebetulan lolos (sizeof 126, crc di 124 — pas tanpa padding), tapi bentuknya ikut disamakan supaya kekeliruan yang sama tidak menular saat struct berubah.

Tidak perlu menaikkan `CALIB_VERSION`: blob lama memang tak pernah sah, jadi boot pertama sesudah perbaikan menolaknya, menulis ulang dengan CRC benar, dan sembuh sendiri.

---

## 13. Harness simulasi PC

Di `../test-pc/` ada stub `Arduino.h`, `EEPROM.h`, `Wire.h`, `Adafruit_PWMServoDriver.h`, dan `VL53L0X.h` yang memungkinkan **`Navigation`, `Hexapod`, `Imu`, dan `LidarArray` yang asli** dikompilasi dan dijalankan di PC. Yang dipalsukan hanya jam, bus I2C, EEPROM, dan aliran byte sensor — logikanya tidak disalin, jadi yang diuji benar-benar kode yang di-upload ke robot.

Jam bisa dimajukan sesuka hati, yaw disuapkan lewat frame WIT `0x55 0x53` sungguhan (jadi parser IMU ikut teruji), dan jarak LiDAR disuapkan per channel mux.

| Program | Menguji |
|---|---|
| `sim_pivot` | Pivot non-blokir: kembali seketika, sampai target, bisa dibatalkan, timeout |
| `sim_body` | Besar lonjakan pose badan per commit — angka di bagian 5 |
| `sim_lidar` | Tiga keadaan LiDAR dari kesembilan `range_status` VL53L1X, termasuk perilaku fail-safe |
| `sim_open` | Perilaku saat dinding samping hilang vs sensor putus |
| `sim_reinit` | Pemulihan lewat `I` pada skenario gejala nyata |
| `sim_wall` | Sweep gain ikut-dinding — tabel di bagian 7.6 dan 7.7 |
| `sim_dinding` | Celah ujung kaki ke dinding: geometri, tabel setpoint, dan lingkar tertutup dari posisi terlalu dekat — bagian 7.9 |
| `sim_yaw` | Lima arsitektur kendali dibandingkan: PD jarak, koreksi kosinus, kaskade jarak→heading, giro saja, PD+giro — bagian 7.10 |
| `sim_depan` | Hantu sensor depan: lorong kosong vs halangan nyata, gerbang tiga sampel, batas atas 70 cm — bagian 7.11 |
| `sim_boot` | Demo `DEMO_BOOT`: urutan, waktu, dan jalur pembatalannya — bagian 3 |
| `sim_laju` | Laju maju vs `step_length`/`cycle_time`/`duty`, diukur dari `HexaGait` asli |
| `sim_servolaju` | Tuntutan kecepatan sudut servo per commit pada tiap setelan gait |
| `sim_peta` | `f` ditolak & `F` jalan dengan dua sensor rusak |
| `sim_param` | `q`/`Q`/`W` lewat parser asli, termasuk clamp, penolakan, dan persistensi EEPROM |
| `sim_goyang` | Bentuk gelombang `z` sesudah melewati ramp — sinus utuh atau segitiga terpotong |
| `sim_hantu` | Hantu pendek sesekali di tengah keadaan "tak ada target" — sebelum vs sesudah gerbang median |
| `sim_jejak` | `j` memisahkan hantu menetap, ruang kosong, dan pantulan berubah-ubah |
| `sim_isolasi` | `u` memisahkan crosstalk antar-sensor dari pantulan yang melekat |

Dua kegunaan yang terbukti: **pemeriksaan sintaks** (`g++ -fsyntax-only -Wall -Wextra` atas seluruh sketsa, menangkap typo dan `switch` yang kurang case sebelum menyentuh papan), dan **perbandingan perilaku** sebelum/sesudah perubahan.

> **Batasnya jujur:** model gerak robot di `sim_wall` dan `sim_dinding` kasar — perintah putar → laju yaw → laju lateral. Itu memang lingkar umpan balik yang dominan, jadi *perbandingan* antar gain bermakna; angka mutlaknya tetap harus disetel di robot sungguhan.

Perlu `g++` (WSL, MinGW, atau Linux). Lihat `../test-pc/README.md`.

---

## 14. Apa yang berubah dari program lama

### Sinkronisasi dengan TES_GERAK — robot tidak bisa berdiri

Firmware sempat tidak bisa berdiri: keenam kaki justru terangkat ke arah badan. Dua data menyimpang dari `legacy-2026/TES_GERAK`:

1. **Panjang link kaki.** `config.h` memakai coxa 23 / femur 54 / tibia 69, padahal ukuran fisiknya **20 / 80 / 90**. IK menghitung femur −35,11° dan lutut 127,46°, padahal yang benar −10,57° dan 82,02°.
2. **Tabel `invert`.** Rumus lama `(i >= 9 && i < 18)` — "balik semua sendi kaki kiri" — salah di **9 dari 18** flag. Pola fisik yang sebenarnya tidak seragam:

   ```
   coxa  : dibalik di KEENAM kaki
   femur : dibalik hanya di sisi KANAN (kaki 0,1,2)
   tibia : dibalik hanya di sisi KIRI  (kaki 3,4,5)
   ```

Gabungan keduanya membuat femur digerakkan ke **+35,1° (NAIK)** di keenam kaki padahal seharusnya −10,6° (turun); telapak berakhir 19 mm **di atas** pangkal coxa. Pulse pose berdiri yang benar sekarang: kaki 0–2 coxa 1500 / femur 1617 / tibia 1411; kaki 3–5 coxa 1500 / femur 1383 / tibia 1589.

### Keselamatan & boot

* Boot dalam keadaan PWM mati; upload tidak perlu melepas Teensy.
* Pengaktifan pertama di-stagger 60 ms per servo supaya BEC tidak drop.
* `commit()` punya gerbang: tidak mengirim apa pun sebelum `enable()`.

### Body kinematics

* `rotatePointInv()` dulu memberikan `rollRad` ke rotasi sumbu X dan `pitchRad` ke sumbu Y — **tertukar** untuk frame ini, sehingga perintah "roll" menghasilkan gerak pitch.
* Deteksi di luar jangkauan diperluas: `angleToPulse()` ikut menandai saat sudut servo keluar 0–180° atau trim mendorong pulse melewati batas. Dulu IK melaporkan "aman" karena hanya memeriksa jarak.
* **Pose badan sekarang di-ramp** (`BODY_SLEW_DEG_S` / `BODY_SLEW_MM_S`). Dulu mentah — `r20 0 0` = 49° perubahan femur dalam satu commit 20 ms.

### LiDAR

* **Gerbang median-3**: keadaan "jauh" hanya ditinggalkan sesudah tiga sampel dalam jangkauan berturut-turut. Dulu sampel pertama sesudah "jauh" melewati median mentah-mentah dan langsung membalik keadaan sensor, sehingga satu bacaan hantu cukup untuk memunculkan "dinding" 5–10 cm yang tidak ada.
* **`l` mencetak jawaban mentah + `range_status`**, supaya bacaan yang tak masuk akal bisa dilacak ke sumbernya tanpa menebak.
* **Pindah dari VL53L0X ke VL53L1X.** Pustaka terpisah; `setSignalRateLimit()` dan `setVcselPulsePeriod()` tidak ada lagi dan diganti `setDistanceMode()`; `startContinuous()` argumennya kini jeda antar pengukuran (bukan "0 = secepatnya"); polling register `0x13` diganti `dataReady()`; dan kesahihan dibaca dari `range_status`, bukan dari ambang jarak. Mode Short dipilih karena di arena bercahaya justru lebih jauh daripada Long.
* `LidarArray` dulu **tidak pernah diinstansiasi** — kodenya ikut dikompilasi tapi tak satu instruksi pun dieksekusi.
* Tiga keadaan (`cm` / `JAUH` / `MATI`) menggantikan `-1` yang menyamakan "tak ada objek" dengan "sensor rusak".
* **Bug 8190 mm**: jawaban "tak ada target" dibuang sebelum `_lastResp` disegarkan, sehingga ruang terbuka terbaca sebagai sensor putus — membatalkan seluruh desain tiga keadaan.
* **Peta channel dibalik** sesuai wiring sebenarnya. Ini yang membuat navigasi tak pernah bisa jalan.
* Round-robin `update()` tidak lagi tertahan oleh satu sensor macet.
* `begin()` memeriksa mux **lebih dulu**, supaya mux mati tidak dilaporkan sebagai enam sensor rusak.
* `I` sekarang **memulihkan**, bukan cuma melapor; `_muxOk` ikut diperbarui; `initSensor()` dipakai bersama; nilai balik `setMeasurementTimingBudget()` diperiksa.
* `static_assert` menjamin keenam `LIDAR_*` menunjuk channel berbeda.

### Navigasi

* Ikut-dinding dibuat **non-blokir** dan parameter `WALL_*` di `Calib` akhirnya dibaca (sebelumnya tidak pernah disentuh satu baris pun).
* **`pivotKe()` jadi state machine** `NAV_PIVOT`; jalan keluar "tekan Enter" khusus dibuang.
* **`pivotLangkah()` menyatukan** pivot manual dengan fase belok arena; `PIVOT_KP`/`PIVOT_KD` dihapus dari `config.h`.
* **Turunan PD dihitung pada laju sampel LiDAR**, dari nilai EMA float.
* **`NAV_WALL_TURN_MAX` berlaku di semua mode**, bukan hanya mode arena.
* **Gain disetel ulang** — `wall.kp` 0,030 → 0,008, `wall.kd` 0,010 → 0,030.
* **Kemudi dinding jadi dua pita** (§7.2, §7.9): `wall.setpoint` 13 → 19 cm, plus parameter baru `wall.min` (15 cm, pita "terlalu dekat"). Sebabnya fisik: ujung kaki tengah menjulur 160 mm dari pusat badan sedangkan sensor sampingnya duduk di ~50 mm, jadi setpoint 13 cm menyisakan celah kaki hanya 2 cm.
* **`LIDAR_MIN_CM` — batas bawah geometri per arah** (§7.11). Bacaan di bawahnya dipetakan ke `LIDAR_JAUH`, karena di sensor ini bacaan pendek justru berarti "tak ada objek dalam jangkauan". Memperbaiki robot yang berbelok sendiri di lorong depan yang kosong. Menggantikan `wall.hantu` yang hanya melindungi sensor samping; `CALIB_VERSION` → 9.
* **`LIDAR_MAX_CM` 120 → 70 cm**, sesuai jangkauan akurat yang terukur.
* **`NAV_CARI_BATAS_MS`**: berhenti kalau dinding samping hilang lebih dari 10 detik, supaya robot tidak berjalan melingkar selamanya mencari dinding yang tak akan muncul.
* `navMulai()` memeriksa sensor yang dipakai sebelum melangkah.
* `navBerhenti()` tidak lagi mencetak "Navigasi BERHENTI" saat tidak ada yang berjalan.
* `navMulai()`, `pivotKe()`, dan `kalibrasiPivot()` saling menghentikan dengan pesan jelas, bukan menimpa mode diam-diam.
* Mode terkunci arena ditambahkan: heading arena sebagai acuan sudut, dinding sebagai koreksi lateral.
* `kalibrasiPivot()`: kedua pengukuran dulu memakai akumulator yang sama, padahal `tungguYaw()` menolkannya di awal — putaran utama terbuang dan hasilnya hanya berisi rotasi sisa pengereman.
* Hasil kalibrasi pivot kini **permanen** di EEPROM 2048 (`S`), dulu hanya di RAM.

### IMU

* Resinkronisasi sejati: buang byte satu per satu sampai header + checksum cocok.
* Jalan keluar untuk lonjakan yaw yang **menetap** — tanpa itu `_yaw` bisa membeku selamanya dan navigasi berjalan di atas heading basi.
* Batas byte per `update()` supaya loop utama tidak kelaparan saat IMU membanjiri serial.

### Lengan

* Dari "kanan & kiri" menjadi **depan & belakang**; `ARM_ORIGINS` pindah dari sumbu X ke Y.
* `moveArmTarget()` dulu mengurangkan offset ke samping dari koordinat jangkauan — dua sumbu berbeda.
* Slot kalibrasi lengan kedua dipisah (21–23); dulu keduanya memakai 18–20.
* IK lengan memeriksa batas **dalam**, bukan hanya batas luar.

### EEPROM & struktur

* `EEMap.h` jadi satu-satunya definisi; salinan yang bisa menyimpang dihapus.
* `static_assert` menjaga ukuran struct dan batas antar blok.
* `GerakStore` diverifikasi checksum, bukan hanya magic + versi.
* Blok bantuan `h` yang tercetak dua kali dibersihkan.
* `Navigation::majuKini()` / `turnKini()` dibuka untuk telemetri dan uji otomatis.
* **CRC `CalibBlob` dihitung sampai `offsetof(crc)`**, bukan `sizeof - sizeof(crc)` — rumus lama ikut menelan field crc sendiri karena padding di ekor, sehingga blok EEPROM 0 tidak pernah sekali pun berhasil dimuat.
* **Perintah `z`** ditambahkan: goyang roll bergelombang non-blokir untuk pajangan, dengan penjaga yang menaikkan periode supaya ramp pose badan tidak memotong sinusnya jadi segitiga.
* **`Calib::setParam()` / `findParam()` / `save()` disambungkan** ke perintah `q` / `Q` / `W`; `ParamDef` diberi kolom `berlaku`; clamp dilaporkan; `pulse.*` ditolak selagi servo aktif.

---

## 15. Yang masih menunggu

* **Verifikasi arah ch0 dan ch2 dengan `l`.** Keenam LiDAR sudah hidup (September 2026), tapi pemetaan arah kedua channel ini berasal dari ramalan pola dan belum pernah diuji langsung — dulu tidak bisa, karena keduanya yang rusak. Mode `f` / `F` mengemudi dari ch0, jadi ini yang pertama diperiksa.
* **Stabilisasi badan dari IMU** (`setStabilization`) masih dikomentari. Body kinematics-nya sudah siap dan ter-ramp — tinggal menyambungkan roll/pitch IMU ke `setBodyRotation()` (jangan menulis `_roll`/`_pitch` langsung; ramp sudah menangani perataan, jadi low-pass `STAB_TAU` tidak perlu). Dua hal **harus diuji fisik dulu**: (a) sumbu IMU belum tentu sejajar dengan frame robot — cocokkan dengan `r`/`B`; (b) `stab.sign_roll` / `stab.sign_pitch` di `Calib` belum dipakai sama sekali.
* **`kalibrasiPivot()` masih memblokir**, tapi memang tidak ada yang perlu disela. Ini satu-satunya jalur pemblokir yang tersisa.
* **Menggabungkan kompas arena dengan ikut-dinding** — "ikut dinding sampai lorong habis, lalu pivot ke Utara" — belum ada, tapi sekarang jauh lebih dekat: keduanya sudah jadi mode di state machine yang sama.
* **Deteksi korban** belum ada sama sekali.
* **Jalur yang tidak terhubung ke perintah apa pun:** `profileStairs()` / `profileCrouch()` / `profileNarrow()`, `Hexapod::jog()` beserta `TUNE_PIN_MAP`, `Calib::begin()`, `Hexapod::legAngles()` (dipakai harness), `Imu::tare()` / `rollDeg()` / `pitchDeg()` / `accelZ()` / `magMagnitude()`, `Hexapod::armDepan()` / `armBelakang()`, dan `Navigation::degCCW()` / `degCW()` / `mmMaju()` / `pivotTerkalibrasi()`.
* **Konstanta mati:** `STAB_MAX_DEG`, `STAB_DEADBAND_DEG`, `STAB_TAU`, `CONTROL_HZ`, `PROFILE_LOOP`, `SERVO_FREQ`, `HEAD_UTARA`/`TIMUR`/`SELATAN`/`BARAT`, plus slot `K_STAB_SIGN_ROLL` / `K_STAB_SIGN_PITCH` / `K_ARENA_MIRROR`. Menghapus slot `K_*` menuntut kenaikan `CALIB_VERSION`, jadi biarkan sampai stabilisasi disambung.
* **`Imu::_ax` dan `_ay`** diisi tiap frame akselerometer lalu tidak pernah dibaca siapa pun.
* Belum ada perintah **reset ke default**. `Calib::applyDefaults()` juga mereset `offset`/`trim`/`invert`, jadi memanggilnya saat berjalan akan membuang data ServoMap sampai boot berikutnya — perlu dipasangkan dengan `loadServoMap()` bila mau dibuka.
* **`CONTROL_HZ` dan `PROFILE_LOOP`** di `config.h` tidak diimplementasikan. `loop()` berjalan bebas, bukan laju tetap; `dt` diambil dari `millis()` di dalam `HexaGait` sendiri.
* **`HexaGait` memakai `millis()` internal**, sedangkan `Motion` di TES_GERAK menerima `dt` dari pemanggil. Versi TES_GERAK bisa disimulasikan kering tanpa menggerakkan servo; firmware belum bisa (walaupun harness di bagian 13 sudah menutup sebagian kebutuhan itu).
