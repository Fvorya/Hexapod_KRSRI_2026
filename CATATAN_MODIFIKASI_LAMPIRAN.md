# Lampiran — usul dari referensi luar

Pelengkap `CATATAN_MODIFIKASI.md`. Berkas pertama berisi apa yang **hilang
tombolnya**; berkas ini berisi apa yang **tidak pernah ada**, dari membandingkan
firmware ini dengan literatur robot berkaki dan firmware hexapod lain.

Sumbernya di bagian paling bawah. Tiap usul saya beri kelas:
**A** = temuan inspeksi yang tidak butuh referensi, **B** = dari referensi luar
dan cocok, **C** = dari referensi luar dan saya TOLAK, dengan alasannya.

Satu bingkai yang membuat urutannya masuk akal: bonus guidebook =
`total x 300 / detik` (`Skor.h`). Waktu bukan tie-breaker, ia pengali. Jadi apa
pun yang memangkas **pengulangan** — sensor yang salah baca, robot yang ambruk,
ruas yang harus diulang `m3` — membayar lebih besar daripada kelihatannya.

---

## STATUS — PROMPT 1 SUDAH DIKERJAKAN (18 Sep 2026)

| # | Butir | Kelas | Status |
|---|---|---|---|
| A1 | Profil waktu loop | A | **SELESAI.** Baris `PROF` nyata. Yang berubah dari dugaan cuma satu: `CONTROL_HZ` ternyata tetap TIDAK dipakai sebagai pembatas laju — dan memang tidak boleh, karena seluruh kendali sudah berbasis `dt`. Ia dipakai sebagai acuan `util` saja. Komentarnya diperbaiki supaya jujur. |
| A2 | OLED berbagi bus dengan driver servo | A | **BELUM DIUJI — dan belum dibuktikan salah.** Butir 3 PROMPT 1 sengaja dilewati: menguji hipotesis ini menuntut robot menyala lalu membaca baris `PROF`, dan menebak bukan pilihan. `Tampilan.cpp:154` **tidak disentuh**; laju gambarnya masih 125 ms. Yang sudah berubah: alat untuk menjawabnya kini ada. Cara menjawabnya: nyalakan robot, berdiri diam, dan lihat apakah `max` di baris `PROF` melonjak ke belasan ms secara berkala (periode ~125 ms). |
| A3 | Deteksi terguling | A | **SELESAI.** Tiga ambang + saklar di `config.h`, ketiganya ditandai BELUM DIUKUR. Ditambah satu penjaga yang tidak ada di usul: robot LEMAS bukan terguling (ia sedang ditopang di meja), dan IMU tanpa data bukan bukti apa-apa — `accelZ()` mengembalikan 0 sebelum frame pertama, dan 0 itu justru memenuhi syarat terguling. |
| B1 | Offset jarak per sensor LiDAR | B | **SELESAI**, sebagai `Yd`. RAM saja seperti yang diusulkan. Satu tambahan yang tidak ada di usul tapi masuk lingkup: pencatatan DITOLAK kalau sensornya `MATI`/`JAUH`. |
| B2 | Kompensasi CG | B | **BELUM** — menunggu prompt 4 (butuh geometri kaki benar + profil waktu loop terukur). |
| B3 | Tegangan bus servo | B | **BELUM** — menunggu pembagi tegangan. |

Yang di TOLAK (kelas C: watchdog, CPG, gait wave, free gait) **tidak
dikerjakan** dan tidak ditawarkan lagi.

---

## KELAS A — temuan inspeksi, belum masuk catatan pertama

### A1. Tidak ada satu pun pengukuran waktu loop — padahal config.h menjanjikannya

`CONTROL_HZ 100` dan `PROFILE_LOOP 1` ada di `config.h:1344-1345`, dan
`PROFILE_LOOP` bahkan berkomentar *"1 = cetak PROF avg/max/util tiap detik"*.
**Keduanya tidak dibaca di satu tempat pun** — sudah saya grep seluruh pohon.
Jadi loop berjalan bebas tanpa pacu, dan tidak ada yang tahu berapa lama satu
putaran memakan waktu.

Itu bukan cuma kosmetik. `HexaGait::dtSeconds()` meng-clamp `dt` ke 0,05 detik.
Kalau satu putaran pernah melewati 50 ms, gait **diam-diam melangkah lebih
pendek daripada yang dihitung odometer** — dan odometer memakai `dPhase` dari
`dt` yang sama, jadi keduanya salah bersama-sama dan saling membenarkan.

Usulnya persis yang sudah dijanjikan benderanya: ~10 baris di `loop()`, catat
`min/avg/maks` dan berapa kali `dt` kena clamp, cetak sekali per detik.
**Kerjakan ini lebih dulu**, karena A2 tidak bisa dibuktikan tanpanya.

### A2. OLED berbagi bus dengan driver servo kedua

`config.h:728` → `SERVO_1_I2C_BUS Wire2`. `Tampilan.h:42` → `OLED_I2C_BUS Wire2`.
Bus yang sama.

`Tampilan.cpp:154` menggambar tiap 125 ms, dan `oled.display()` mengirim seluruh
penyangga 128x32 = 512 byte secara **memblokir**. Pada 400 kHz itu ~12 ms
sekali gambar, sebelum ongkos per-potongan. Periode commit servo 20 ms
(`SERVO_COMMIT_MS`). Artinya sekali tiap 125 ms, seluruh loop — commit servo,
pompa IMU, giliran LiDAR — berhenti selama lebih dari setengah periode servo.

**Saya belum mengukurnya, dan memang tidak bisa sampai A1 ada.** Jadi ini
hipotesis, bukan temuan. Tapi ia satu-satunya hipotesis yang menjelaskan
"kadang gerakan tersentak" tanpa menyalahkan mekanik. Kalau terbukti, tiga
jalan, dari termurah:

1. turunkan laju gambar 125 → 500 ms (1 baris; OLED cuma dibaca manusia);
2. gambar **berpotong** — satu seperempat layar per pemanggilan, `display()`
   penuh hanya saat layar berganti;
3. pindahkan OLED ke `Wire` (bus LiDAR) — tapi di sana ada mux 0x70 dan
   pemulihan bus yang menggoyang SCL sebagai GPIO; jangan.

Saya condong ke (1): satu baris, dan tidak ada yang hilang.

### A3. Tidak ada deteksi terguling

`Imu::accelZ()` ada dan dibaca — tapi **hanya dicetak** di aliran yaw. Komentar
di `Hexapod_Unlimited.ino:164` menuliskannya sendiri: *"Inilah satu-satunya
angka yang tahu"* robot sudah terguling. Tidak ada yang bertindak atasnya.

Robot ini menaiki tangga dengan margin guling 29,4 mm (`cek_kail` MERAH). Kalau
ia terguling di tengah misi, gait tetap berjalan, servo tetap memaksa kaki ke
sasaran IK yang tidak berarti apa-apa lagi, dan yang rusak bisa lebih dari
skornya.

Usul: di `loop()`, kalau `|accelZ| < 0,5 g` **atau** `|roll| > 45°` bertahan
lebih dari ~400 ms → `misi.batal()` + `nav.navBerhenti()` + `robot.disarm()` +
teriak ke Serial dan OLED. ~12 baris. Ambang 45° dan 400 ms harus disetel di
robot — taruh sebagai `#define`, bukan hard-code.

Catatan jujur: ini **menambah satu cara baru misi bisa berhenti sendiri**, dan
salah setel berarti robot menyerah di tanjakan 27,7° yang memang normal. Karena
itu ambangnya jangan dekat-dekat kemiringan arena, dan `accelZ` lebih aman
dipercaya daripada `roll`.

---

## KELAS B — dari referensi luar, saya usulkan

### B1. Kalibrasi offset per sensor LiDAR ⭐

ST menyatakan VL53L1X butuh **tiga** kalibrasi — RefSPAD, offset, dan crosstalk
— dan RefSPAD serta crosstalk **wajib diulang begitu ada kaca/akrilik penutup
di atas modul**. Repeatability tipikalnya ±0,15%–1% tergantung anggaran waktu
dan cahaya sekitar.

Firmware ini tidak mengalibrasi satu pun dari ketiganya. Yang ada cuma dua
`#define` global `WALL_BIAS_KIRI_CM` / `WALL_BIAS_KANAN_CM`, dan itu bias
**sudut**, bukan offset jarak per sensor. Sementara itu `WALL_KAKI_CM 7,0`
diturunkan dari satu pengukuran ("kedua LiDAR kanan membaca ~7 cm") yang
**mengasumsikan keenam sensor sepakat**.

Pustaka Pololu VL53L1X tidak mengekspos API kalibrasi ST, jadi jangan tukar
pustaka. Versi malasnya cukup:

```
Yd<ch> <cm>   "sensor ch sedang <cm> dari dinding, catat selisihnya"
```

6 float offset, dikurangkan di satu tempat di `LidarArray`. Prosedurnya: robot
menghadap dinding, ukur meteran, ketik. Lima menit sekali pasang, dan sesudah
itu `WALL_SETPOINT_CM` akhirnya berarti jarak yang sama di keenam sensor.

**Penyimpanannya RAM dulu, bukan `Calib::save()`.** Menambah 6 float ke
`CalibBlob` menaikkan `CALIB_VERSION` — persis wipe yang butir ini justru
dijadwalkan mendahului. Jadi `Yd` menyusul masuk `PARAM_DEFS` di batch (butir 7),
bersama `wall.bias_*` dan `odo.skala` yang nasibnya sama. Sampai saat itu ia
berperilaku seperti `Ds` dan `Y0`: ulangi tiap robot menyala.

Kelas `LIDAR_MODE_SHORT` yang sudah dipakai **sudah benar** menurut ST — mode
pendek memang yang paling tahan cahaya sekitar, dan `setROISize(4,4)` di
`LidarArray.cpp:251` juga sejalan dengan anjuran mempersempit ROI di sekitar
pusat optik.

### B2. Kompensasi CG: geser badan ke tripod penumpu ⭐⭐

Ini usul terkuat dari literatur, dan yang paling langsung menjawab masalah yang
sudah kamu catat sendiri.

Literatur robot berkaki mendefinisikan **stability margin** = jarak terpendek
dari proyeksi CG ke tepi poligon tumpu; makin besar makin stabil. Teknik
bakunya: selama satu tripod menumpu, **geser badan ke arah pusat poligon tripod
itu**, supaya CG tidak pernah dekat tepi. Itu menaikkan margin **tanpa**
memperlebar stance.

Kenapa ini cocok di sini: `cek_kail` MERAH di 29,4 mm terhadap ambang 30, dan
`config.h` sudah mencatat bahwa **melebarkan kaki justru memperburuk margin di
lereng**. Jadi jalan "perlebar stance" sudah dicoba dan sudah gagal. Yang belum
dicoba adalah memindahkan CG-nya, dan mekanismenya **sudah ada seluruhnya**:
`setBodyTranslation()`, ramp `BODY_SLEW_MM_S`, dan `_trans` yang sudah masuk
rantai IK.

Bentuk paling malas: di `HexaGait::update()`, saat melangkah, hitung satu
pergeseran x/y kecil berlawanan fase tripod, sebesar `GAIT_CG_MM` (knop baru,
mulai dari 0 = mati), dan tambahkan ke `_footHome` — **bukan** lewat
`setBodyTranslation()`, supaya pose badan perintah operator tidak ikut
tertimpa. ~12 baris.

Harga yang harus diakui di muka: ia memakan sebagian jangkauan IK, dan kaki
belakang sudah memakai 95%-nya. Mulai dari `GAIT_CG_MM 0`, naikkan sambil
melihat `T` melaporkan "(di luar jangkauan IK)".

### B3. Pemantauan tegangan bus servo

Firmware Phoenix (PhantomX/Lynxmotion, rujukan de-facto hexapod hobi) punya
*"voltage monitoring with automatic shutdown at low battery thresholds"* sebagai
fitur baku. Firmware ini tidak punya sama sekali — tidak ada `analogRead`
di seluruh pohon.

Sementara itu `CLAUDE.md` menaruh **"Tegangan catu servo lengan (DS3225 butuh
6,8 V)"** di urutan kedua daftar ukur lapangan, dan mencatat capit melendut
70 mm saat terjulur. Dan literatur brownout mencatat justru pola ini: lonjakan
arus servo menjatuhkan rel, dan yang terlihat di permukaan adalah gerakan yang
lemah atau meleset — bukan tegangan.

Butuh hardware: satu pembagi tegangan ke satu pin ADC Teensy. Sesudah itu
firmwarenya kecil — baca 10 Hz, tampilkan di `d`/`v`/OLED, dan **catat nilai
minimum sejak boot**. Angka minimum itu yang penting: sag terjadi saat 18 servo
bergerak bersamaan, jadi pembacaan sesaat saat robot diam tidak pernah
menangkapnya.

Saya **tidak** mengusulkan auto-shutdown-nya. Robot yang melemaskan diri sendiri
di tengah tangga karena satu sag sesaat lebih mahal daripada sag itu sendiri.
Ukur dulu, baru putuskan apakah ambangnya layak ada.

### B4. Trajektori ayun Bézier — layak, tapi TUNDA

Literatur konsisten: dibanding sikloid, lintasan Bézier menghasilkan
**kecepatan dan torsi sendi lebih kecil** dan **fluktuasi sikap badan lebih
kecil**, karena sikloid membayar percepatan/perlambatan yang tidak perlu.

Tapi baca implementasi yang ada dulu (`HexaGait.cpp`, cabang SWING): ia sikloid
yang **sudah benar** — kecepatan nol di liftoff dan touchdown, yang memang
sumber utama sentakan. Keuntungan Bézier di atas titik itu nyata tapi tipis,
dan ia menyentuh satu-satunya bagian firmware yang sudah terbukti bekerja di
arena.

Tunda sampai ada keluhan yang memang berbunyi *"badan bergoyang saat
melangkah"*. Kalau itu muncul, kerjakan B2 dulu — ia menjawab gejala yang sama
dengan risiko lebih kecil.

---

## KELAS C — dari referensi luar, saya TOLAK

### C1. Watchdog Teensy (WDT_T4)

Anjuran baku untuk firmware embedded, dan **salah untuk robot ini**.

Firmware ini boot dalam keadaan **LEMAS** — itu keputusan keselamatan yang
benar dan ditulis besar-besar di `HexaServos::begin()`. Konsekuensinya: reset
watchdog di tengah R-9 = PWM padam = robot **ambruk dari tangga**. Sementara
kalau firmware benar-benar hang, PWM PCA9685 **tetap menahan pulsa terakhir**,
jadi robot membeku sambil berdiri.

Membeku berdiri lebih murah daripada ambruk. Watchdog di sini menukar kegagalan
yang aman dengan kegagalan yang merusak.

Gantinya, gratis dari A1: kalau satu putaran loop melewati ambang, **cetak dan
bunyikan**, jangan reset. Operator masih punya `x` dan Enter.

### C2. CPG (central pattern generator)

Muncul di hampir semua literatur hexapod modern, termasuk yang terbaru. Ia
mengganti generator fase eksplisit dengan osilator berkopel.

Yang ada sekarang deterministik: satu `_phase`, dua tripod, dan seluruh
matematikanya murni sehingga bisa diuji di PC (`types.h` sengaja tanpa
`Arduino.h` — komentarnya menyebut itu). Menukarnya dengan osilator berarti
menukar sesuatu yang bisa diuji dengan sesuatu yang harus dipercaya. Dan tidak
ada satu pun keluhan di `CLAUDE.md` yang dijawab CPG.

### C3. Gait wave/ripple dan fault-tolerant gait 5 kaki

Wave/ripple menaikkan margin statik dengan menaruh 4–5 kaki di tanah, tapi
faktor duty-nya jauh lebih besar — jelas lebih lambat. Dengan bonus
`total x 300 / detik`, lambat itu hukuman langsung.

Fault-tolerant gait (jalan dengan 5 kaki saat satu servo mati) menarik untuk
lomba, tapi ia menuntut **deteksi kaki mati**, dan robot ini tidak punya umpan
balik servo sama sekali — PCA9685 buta. Tanpa deteksi, gait-nya tidak punya
pemicu.

### C4. Perencanaan pijakan (footstep planning / free gait)

Butuh peta medan. Enam VL53L1X yang menghadap mendatar tidak menghasilkan peta
medan. Tabel `Ruas[]` 33 baris sudah jadi perencananya, dan keunggulannya bisa
dibaca dan dikoreksi manusia di pit.

---

## Urutan gabungan yang saya usulkan

Gabungan dengan urutan di catatan pertama:

| # | Butir | Asal | Biaya | Kenapa di sini | Status 18 Sep |
|---|---|---|---|---|---|
| 1 | Profil waktu loop | A1 | ~10 baris | Prasyarat A2; benderanya sudah ada di config.h | **SELESAI** |
| 2 | `Yo`/`Yi`/`Yj`/`Yz` | catatan 1 | sedang | Tak menyentuh EEPROM, membuka ukur-koreksi tanpa flash | **SELESAI** |
| 3 | Laju gambar OLED | A2 | 1 baris | Hanya kalau butir 1 membuktikannya | **DILEWATI** — butuh robot |
| 4 | Deteksi terguling | A3 | ~12 baris | Melindungi perangkat keras, bukan skor | **SELESAI** |
| 5 | Offset LiDAR `Yd` | B1 | ~15 baris | Tiap ruas berumpan-balik ikut membaik | **SELESAI** |
| 6 | Ukur geometri kaki | catatan 1 | lapangan | Penutup yang sudah diminta CLAUDE.md | **MENUNGGU PENGUKURAN** |
| 7 | Batch `CALIB_VERSION` | catatan 1 | sedang | Sesudah 6, supaya default langsung benar | belum (butuh 6) |
| 8 | `Th`/`Tl`/… + putar di `w` | catatan 1 | kecil | Tidak memblokir apa pun | belum (putar di `w` sudah ada) |
| 9 | Kompensasi CG | B2 | ~12 baris | Butuh 1 dan 6 lebih dulu supaya terukur | belum (butuh 6) |
| 10 | Tegangan bus servo | B3 | HW + kecil | Butuh pembagi tegangan dulu | belum (butuh HW) |

Butir 1–5 semuanya bisa jalan **sebelum** wipe `CALIB_VERSION`, dan tidak satu
pun dari keduanya menghalangi yang lain.

**Yang benar-benar terjadi sesudah dikerjakan:** empat dari lima butir pertama
selesai, dan satu (A2) ternyata memang **tidak bisa** dikerjakan tanpa robot —
dugaan di atas bahwa ia "1 baris" benar, tapi keputusannya bukan keputusan
kode. Justru itu urutan yang diusulkan membayar: A1 dikerjakan lebih dulu,
dan hasilnya sekarang **bisa menjawab A2** — begitu ada yang menyalakan robot
dan membaca satu baris `PROF`.

Satu hal yang tidak terduga dan layak dicatat: **A3 menuntut dua penjaga yang
tidak ada di usulnya**, dan keduanya bukan hiasan. Robot yang LEMAS sedang
ditopang di meja, bukan terguling — tanpa penjaga itu setiap kali robot
diangkat miring dalam keadaan lemas, misi dibatalkan. Dan `accelZ()` sebelum
frame IMU pertama bernilai 0, yang justru **memenuhi** syarat terguling;
deteksinya akan menembak di detik pertama setiap kali servo dihidupkan.

---

## Sumber

Lintasan kaki & gait:
- [Spider-Leg-Inspired Structural Design and Bézier Foot Trajectory Planning for Stable Walking of a Hexapod Robot](https://doi.org/10.3390/biomimetics11050352)
- [Smart Gait: A Gait Optimization Framework for Hexapod Robots — Chinese Journal of Mechanical Engineering](https://cjme.springeropen.com/articles/10.1186/s10033-024-01000-0)
- [Gait and trajectory rolling planning and control of hexapod robots for disaster rescue applications](https://www.sciencedirect.com/science/article/abs/pii/S0921889017300209)

Stabilitas, poligon tumpu, CG:
- [Fault-Tolerant Tripod Gait Planning and Verification of a Hexapod Robot (MDPI)](https://www.mdpi.com/2076-3417/10/8/2959)
- [SPINDER: an open-source 18-DoF hexapod robot with hierarchical CPG control and analytic IK (Scientific Reports)](https://www.nature.com/articles/s41598-026-56856-0)
- [Fundamentals of Hexapod Robot — Hackaday.io](https://hackaday.io/project/21904-hexapod-modelling-path-planning-and-control/log/62326-3-fundamentals-of-hexapod-robot)
- [Fault Tolerant Free Gait and Footstep Planning for Hexapod Robot Based on Monte-Carlo Tree (arXiv)](https://arxiv.org/pdf/2006.07550)

Firmware hexapod pembanding:
- [KurtE/Phantom_Phoenix — Lynxmotion Phoenix untuk PhantomX](https://github.com/KurtE/Phantom_Phoenix)
- [KurtE/Arduino_Phoenix_Parts](https://github.com/KurtE/Arduino_Phoenix_Parts)

LiDAR VL53L1X:
- [Datasheet VL53L1X (ST)](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [VL53L1X calibration with cover glass — ST Community](https://community.st.com/t5/imaging-sensors/vl53l1x-calibration-with-cover-glass/td-p/228242)
- [Optimizing VL53L1X for Close-Range Accuracy — ST Community](https://community.st.com/t5/imaging-sensors/optimizing-vl53l1x-for-close-range-accuracy-10mm-200mm-best/td-p/798588)
- [VL53L1 Ranging instability in strong ambient light — ST Community](https://community.st.com/t5/mems-sensors/vl53l1-ranging-instability-in-strong-ambient-light/td-p/316650)

Watchdog & catu daya:
- [tonton81/WDT_T4 — Watchdog untuk Teensy 4](https://github.com/tonton81/WDT_T4)
- [WDT_T4 — Watchdog Library for Teensy 4 (PJRC Forum)](https://forum.pjrc.com/threads/59257-WDT_T4-Watchdog-Library-for-Teensy-4)
- [Dealing with pesky servo-induced brownouts — Hackaday.io](https://hackaday.io/project/3339-neurobytes/log/37589-dealing-with-pesky-servo-induced-brownouts)
- [roboRIO Brownout and Understanding Current Draw — FIRST Robotics](https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-brownouts.html)
