# Hexapod Unlimited

Firmware Teensy 4.1 untuk robot hexapod berkaki enam: gait tripod, kinematika invers kaki dan lengan, IMU 10-axis, enam LiDAR ToF, dan navigasi otonom ikut-dinding. Semua kalibrasi permanen di EEPROM.

**Status singkat (Agustus 2026).** Kaki, gait, body kinematics, pivot, dan navigasi ikut-dinding sudah jalan. Lengan belum terpasang fisik sehingga IK-nya belum teruji. Dua dari enam LiDAR rusak fisik — akibatnya mode ikut-dinding **kiri** belum bisa dipakai, mode **kanan** bisa.

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
| `Wire` | SDA 18 / SCL 19 | Mux LiDAR TCA9548A `0x70` + 6× VL53L0X `0x29` |
| `Wire1` | SDA 17 / SCL 16 | PCA9685 driver 0 `0x41` |
| `Wire2` | SDA 25 / SCL 24 | PCA9685 driver 1 `0x40` |
| `Serial2` | RX 7 / TX 8 | IMU Yahboom 10-axis, protokol WIT, 230400 baud |

> **Jangan pernah menyatukan bus LiDAR dengan PCA9685.** Alamat ALL-CALL bawaan PCA9685 juga `0x70` — sama dengan TCA9548A — dan akan bentrok.

Pustaka **"VL53L0X by Pololu"** wajib terpasang. Karena `LidarArray.cpp` ada di dalam folder sketsa, tanpa pustaka itu **seluruh sketsa gagal dikompilasi**.

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

**Cara membaca demo `B`:** kalau body kinematics benar, telapak **tidak boleh bergeser di lantai** — badan mengayun di atas kaki yang diam. Kalau telapak ikut menyeret, ada yang salah di rantai transform.

---

## 6. LiDAR

Enam VL53L0X lewat mux TCA9548A. `update()` memajukan **satu** sensor per pemanggilan tanpa busy-wait, dengan polling bit status interupsi. Filter: median-3 (buang spike) → EMA (`LIDAR_EMA_ALPHA` 0,4).

### Tiga keadaan, bukan dua

| Nilai | Arti | Tampilan `l` |
|---|---|---|
| angka cm | ada objek dalam jangkauan | `60 cm` |
| `LIDAR_JAUH` (999) | sensor **sehat**, tak ada objek dalam `LIDAR_MAX_CM` | `jauh` |
| `LIDAR_MATI` (−1) | sensor tidak merespons | `MATI` |

Pembedaan ini pondasi navigasi: "lorong terbuka" dan "kabel putus" menuntut reaksi yang **berlawanan**. Caranya, `_lastResp` mencatat kapan sensor terakhir menjawab **apa pun**, terpisah dari `_lastOk` yang mencatat pembacaan dalam jangkauan.

> **Jebakan yang sempat membatalkan seluruh desain ini.** Pustaka Pololu mengembalikan tepat **8190 mm** untuk "tidak ada target". Penyaring lama membuang `mm >= 8000` di baris yang sama dengan timeout — yaitu **sebelum** `_lastResp` disegarkan. Jadi jawaban paling normal dari sensor yang menghadap ruang terbuka dibuang sebagai omong kosong, dan sesudah `LIDAR_TIMEOUT_MS` sensor itu dilaporkan `LIDAR_MATI`. Cabang `cm > LIDAR_MAX_CM` pun nyaris tak pernah tercapai, karena VL53L0X melompat langsung dari jarak terukur ke 8190 — tidak merayap lewat 250 cm. Sekarang hanya `timeout` dan `0xFFFF` yang berarti tidak menjawab.

### Peta channel: urutan kabelnya TERBALIK

Kode lama menganggap channel 0 menghadap depan. Uji fisik menunjukkan urutan kabel sebenarnya **kebalikannya** — channel `n` memegang arah yang dulu diberi indeks `5 − n`:

| Channel mux | Arah fisik sebenarnya | Dikira kode lama |
|---|---|---|
| 0 | **kiri depan** — rusak fisik | depan |
| 1 | kiri belakang | kanan depan |
| 2 | **belakang** — rusak fisik | kanan belakang |
| 3 | kanan belakang | belakang |
| 4 | kanan depan | kiri belakang |
| 5 | **depan** | kiri depan |

Empat baris diperiksa satu per satu di robot; pola yang sama meramalkan dua sisanya, dan ramalan itu cocok dengan dua sensor yang memang rusak — jadi keenamnya konsisten.

**Ini yang membuat navigasi tak pernah bisa dites.** `LIDAR_FRONT` menunjuk channel 0, dan channel 0 justru salah satu sensor yang mati. Jadi `f`/`F` selalu berhenti seketika dengan *"sensor DEPAN tidak merespons"* — gain, turunan PD, dan batas kemudi sama sekali tidak relevan selama itu belum benar.

`LIDAR_NAMA[]` ikut diurutkan menurut arah fisik. Keduanya dijaga saat kompilasi:

```cpp
static_assert(((1u << LIDAR_FRONT) | (1u << LIDAR_FRONT_R) | ... ) == 0x3Fu,
              "LIDAR_* di config.h harus enam channel BERBEDA dalam 0..5");
```

### `I` memindai **dan** memulihkan

`begin()` hanya berjalan sekali saat boot dan `_isReady` tidak pernah ditinjau ulang — jadi sensor yang gagal init mati untuk seluruh sesi, padahal `I` melaporkan modulnya "ADA". Sekarang `I` mencoba init ulang tiap channel yang ada di bus tapi tidak bekerja. **Dua** kondisi diperiksa:

| Baris di `l` | Keadaan internal | Dipulihkan `I`? |
|---|---|---|
| `tidak di-init saat begin()` | `_isReady` false | ya |
| `MATI -- tidak merespons` | `_isReady` **true**, berhenti kirim data | ya |
| angka / `jauh` | sehat | tidak disentuh |

Baris kedua mudah terlewat: sensor yang lolos init lalu berhenti mengirim tetap ber-`_isReady` true.

```
ch0 (DEPAN-KI ) : VL53L0X @0x29 ADA -> INIT ULANG BERHASIL (tadinya MATI)
ch1 (BLKG-KI  ) : VL53L0X @0x29 ADA tapi INIT ULANG GAGAL
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

Sensor yang dipakai: **depan** (`LIDAR_FRONT`) selalu, plus **satu** sensor samping — `LIDAR_FRONT_L` untuk mode `f`, `LIDAR_FRONT_R` untuk mode `F`.

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

### 7.2 Kemudi PD, dari sensor samping

```
err  = jarakSamping − WALL_SETPOINT_CM            (+ artinya TERLALU JAUH dari dinding)
turn = sisi × ( WALL_KP · err  +  WALL_KD · ė )
```

`sisi = +1` untuk ikut dinding **kiri**, `−1` untuk **kanan**. Konvensi `turn` positif = CCW = **belok kiri**.

Cek tandanya untuk mode `F` (dinding kanan, `sisi = −1`): terlalu jauh → `err` positif → `turn` negatif → belok kanan → **mendekat ke dinding**. Benar. Terlalu dekat → tandanya membalik sendiri.

Dengan gain sekarang (`WALL_KP` 0,008 / `WALL_KD` 0,030, setpoint 13 cm), keadaan mantap (`ė` = 0):

| jarak samping | err | turn | arah |
|---|---|---|---|
| 4 cm | −9 | **+0,072** | belok kiri — menjauh |
| 8 cm | −5 | +0,040 | menjauh |
| 13 cm | 0 | 0,000 | lurus |
| 20 cm | +7 | −0,056 | belok kanan — mendekat |
| 30 cm | +17 | −0,136 | mendekat |
| 50 cm | +37 | −0,296 | mendekat |
| ≥ 75,5 cm | ≥ +62,5 | **−0,500** | kena batas `NAV_WALL_TURN_MAX` |

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

> **`CALIB_VERSION` wajib dinaikkan setiap default `PARAM_DEFS` diubah** (sekarang 7). Gain lama sudah terlanjur tersimpan di EEPROM alamat 0, dan blob versi lama tetap lolos CRC — tanpa kenaikan versi, robot memuat kembali nilai lama dan perubahan tidak berefek apa pun.

### 7.7 Yang TIDAK bisa diperbaiki dengan gain: sudut pasang sensor

Sweep yang sama dijalankan untuk tiga sudut pasang sensor samping:

| Sudut sensor dari depan | Menyusul dinding dari tengah lorong | Menjaga jarak setelah dekat |
|---|---|---|
| 90° (tegak lurus) | berhasil, paling dekat 5,7 cm | baik (RMS 0,4 cm) |
| 75° | **selalu menabrak**, semua gain | baik (RMS 0,6 cm) |
| 60° | **selalu menabrak**, semua gain | baik (RMS 0,6 cm) |

Mekanismenya umpan balik positif, dan tidak ada gain yang bisa membalikkannya. Sinar yang miring ke depan **memanjang** saat robot menoleh ke arah dinding: pada sinar 60°, menoleh 30° membuat bacaan hampir **dua kali lipat** padahal jarak tegak lurusnya tidak berubah. Kendali membaca "makin jauh" lalu menoleh lebih dalam — sampai menabrak.

Praktisnya:

* untuk **menjaga** jarak setelah dekat, kendali ini sudah cukup — mulai dengan robot ~13 cm dari dinding dan kira-kira sejajar lorong;
* untuk **mendekati** dinding dari tengah lorong, arahkan sensor samping tegak lurus, atau pakai mode `p`/`P` yang mengunci heading arena — di sana arah hadap diurus IMU, jadi kopling geometris ini tidak bisa mengumpan balik.

### 7.8 Penjaga keselamatan

* `navMulai()` memeriksa **sensor yang benar-benar dipakai mode itu** sebelum melangkah, dan menyebut nama serta nomor channel-nya. Dulu robot mulai berjalan lalu berhenti satu iterasi kemudian — terlihat seperti menolak jalan tanpa sebab.
* Menolak mulai bila servo lemas atau mux LiDAR tak terdeteksi.
* Berhenti sendiri bila servo dilemaskan di tengah jalan.
* `s`, `x`, Enter, dan `w` semuanya membatalkan. Ini wajib: tanpa itu `navUpdate()` akan memerintahkan gerak lagi di iterasi berikutnya, sehingga robot tampak "menolak berhenti".

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
Letakkan robot **~13 cm dari dinding** dan kira-kira sejajar lorong. `F` untuk ikut dinding kanan. Tangan tetap di `x` — tidak ada sensor tabrakan samping.

---

## 12. Daftar perintah serial

### Keselamatan & diagnostik
| | |
|---|---|
| `x` | **LEMAS** — semua PWM mati, servo bebas. Rem paling aman |
| `d` | Dump diagnostik: status PWM, sumber ServoMap & zOff, panjang link, profil gait, lalu per kaki `zOff`, input IK, sudut, flag invert, sudut servo, pulse akhir, dan status jangkauan |
| `h` | Bantuan |
| `M` | Peta EEPROM + kapasitas chip sebenarnya |

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

## 13. Harness simulasi PC

Di `../test-pc/` ada stub `Arduino.h`, `EEPROM.h`, `Wire.h`, `Adafruit_PWMServoDriver.h`, dan `VL53L0X.h` yang memungkinkan **`Navigation`, `Hexapod`, `Imu`, dan `LidarArray` yang asli** dikompilasi dan dijalankan di PC. Yang dipalsukan hanya jam, bus I2C, EEPROM, dan aliran byte sensor — logikanya tidak disalin, jadi yang diuji benar-benar kode yang di-upload ke robot.

Jam bisa dimajukan sesuka hati, yaw disuapkan lewat frame WIT `0x55 0x53` sungguhan (jadi parser IMU ikut teruji), dan jarak LiDAR disuapkan per channel mux.

| Program | Menguji |
|---|---|
| `sim_pivot` | Pivot non-blokir: kembali seketika, sampai target, bisa dibatalkan, timeout |
| `sim_body` | Besar lonjakan pose badan per commit — angka di bagian 5 |
| `sim_lidar` | Tiga keadaan LiDAR benar-benar terbedakan |
| `sim_open` | Perilaku saat dinding samping hilang vs sensor putus |
| `sim_reinit` | Pemulihan lewat `I` pada skenario gejala nyata |
| `sim_wall` | Sweep gain ikut-dinding — tabel di bagian 7.6 dan 7.7 |
| `sim_peta` | `f` ditolak & `F` jalan dengan dua sensor rusak |

Dua kegunaan yang terbukti: **pemeriksaan sintaks** (`g++ -fsyntax-only -Wall -Wextra` atas seluruh sketsa, menangkap typo dan `switch` yang kurang case sebelum menyentuh papan), dan **perbandingan perilaku** sebelum/sesudah perubahan.

> **Batasnya jujur:** model gerak robot di `sim_wall` kasar — perintah putar → laju yaw → laju lateral. Itu memang lingkar umpan balik yang dominan, jadi *perbandingan* antar gain bermakna; angka mutlaknya tetap harus disetel di robot sungguhan.

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
* **Gain disetel ulang** — `wall.kp` 0,030 → 0,008, `wall.kd` 0,010 → 0,030, `CALIB_VERSION` → 7.
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

---

## 15. Yang masih menunggu

* **Dua LiDAR rusak fisik** — kiri depan (ch0) dan belakang (ch2). Mode `f` belum bisa dipakai sampai ch0 diperbaiki. Setelah diperbaiki, konfirmasi arah fisik keduanya dengan `l`: pemetaannya berasal dari ramalan pola, bukan uji langsung.
* **Stabilisasi badan dari IMU** (`setStabilization`) masih dikomentari. Body kinematics-nya sudah siap dan ter-ramp — tinggal menyambungkan roll/pitch IMU ke `setBodyRotation()` (jangan menulis `_roll`/`_pitch` langsung; ramp sudah menangani perataan, jadi low-pass `STAB_TAU` tidak perlu). Dua hal **harus diuji fisik dulu**: (a) sumbu IMU belum tentu sejajar dengan frame robot — cocokkan dengan `r`/`B`; (b) `stab.sign_roll` / `stab.sign_pitch` di `Calib` belum dipakai sama sekali.
* **`kalibrasiPivot()` masih memblokir**, tapi memang tidak ada yang perlu disela. Ini satu-satunya jalur pemblokir yang tersisa.
* **Menggabungkan kompas arena dengan ikut-dinding** — "ikut dinding sampai lorong habis, lalu pivot ke Utara" — belum ada, tapi sekarang jauh lebih dekat: keduanya sudah jadi mode di state machine yang sama.
* **Deteksi korban** belum ada sama sekali.
* **Jalur yang tidak terhubung ke perintah apa pun:** `profileStairs()` / `profileCrouch()` / `profileNarrow()`, `Hexapod::jog()` beserta `TUNE_PIN_MAP`, `Calib::setParam()` / `findParam()` / `save()`, `Imu::tare()`, dan parameter `head.*` serta `arena.mirror` di `Calib`. Yang paling terasa: **gain PD belum bisa disetel dari Serial Monitor** karena `setParam()` belum dipanggil siapa pun — setiap perubahan gain masih menuntut kompilasi ulang **dan** kenaikan `CALIB_VERSION`.
* **`CONTROL_HZ` dan `PROFILE_LOOP`** di `config.h` tidak diimplementasikan. `loop()` berjalan bebas, bukan laju tetap; `dt` diambil dari `millis()` di dalam `HexaGait` sendiri.
* **`HexaGait` memakai `millis()` internal**, sedangkan `Motion` di TES_GERAK menerima `dt` dari pemanggil. Versi TES_GERAK bisa disimulasikan kering tanpa menggerakkan servo; firmware belum bisa (walaupun harness di bagian 13 sudah menutup sebagian kebutuhan itu).
