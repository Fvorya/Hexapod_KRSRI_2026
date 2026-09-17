# Prompt kerja untuk AI lain

Tiga prompt, berurutan. **Jalankan satu per sesi**, jangan digabung — batch 2
menunggu pengukuran fisik dari batch 1, dan batch 3 membuang kalibrasi
tersimpan sehingga tidak boleh dijalankan sebelum angkanya diketahui.

Salin blok di dalam pagar kode, apa adanya.

---

## PROMPT 1 — lima butir, tidak menyentuh tata letak EEPROM

```
Kamu mengerjakan firmware Teensy 4.1 untuk robot hexapod KRSRI/SAR Unlimited
2026 di repo Hexapod_KRSRI_2026, branch hexapod-v1.18. Sketsa Arduino ada di
Hexapod_Unlimited/.

BACA DULU, SEBELUM MENGUBAH APA PUN:
  1. CLAUDE.md                        — keputusan arsitektur & status fitur
  2. CATATAN_MODIFIKASI.md            — hasil inspeksi: apa yang hilang tombolnya
  3. CATATAN_MODIFIKASI_LAMPIRAN.md   — usul dari referensi luar, kelas A/B/C
  4. Hexapod_Unlimited/config.h       — hampir tiap angka punya tabel alasannya
     tepat di atasnya. Jangan mengubah angka tanpa membaca tabel itu.

ATURAN KERAS, langgar satu pun berarti pekerjaannya ditolak:

- BAHASA. Seluruh komentar dan seluruh keluaran Serial dalam BAHASA INDONESIA,
  meniru gaya yang sudah ada: padat, menjelaskan ALASAN dan bukan mekanisme,
  dan menyebut angka hasil pengukuran berikut tanggalnya. Jangan menulis
  komentar berbahasa Inggris. Jangan menulis komentar yang cuma mengulang kode
  ("// set the flag"). Lihat HexaGait.h dan Hexapod.h sebagai contoh baku.
- JANGAN MENGARANG ANGKA. Repo ini membedakan dengan tegas "terukur" dari
  "tebakan" (lihat bagian "Yang menunggu" di CLAUDE.md). Kalau sebuah ambang
  belum pernah diukur, tulis nilai awal yang konservatif, jadikan #define di
  config.h, dan katakan di komentarnya bahwa ia BELUM DIUKUR.
- AKHIR BARIS. Repo LF, salinan di disk CRLF (core.autocrlf=true). Penyuntingan
  berbasis jangkar multi-baris HARUS memakai akhir baris yang ada di DISK.
- JANGAN mengubah tata letak EEPROM. Jangan menambah/menghapus baris di
  PARAM_DEFS. Jangan menaikkan CALIB_VERSION. Kelima butir di bawah sengaja
  dipilih karena tidak satu pun menuntutnya — menaikkannya akan membuang
  seluruh kalibrasi yang sudah disetel di robot.
- JANGAN mengerjakan apa pun di luar lima butir di bawah. Kelas C di lampiran
  (watchdog Teensy, CPG, gait wave/ripple, footstep planning) sudah DITOLAK
  berikut alasannya — jangan ditawarkan lagi.
- SEDIKIT LEBIH BAIK. Diff terpendek yang benar-benar bekerja. Tidak ada
  abstraksi yang tidak diminta, tidak ada kelas baru untuk satu pemakai, tidak
  ada berkas baru kalau fungsinya muat di berkas yang sudah ada.

VERIFIKASI. Sesudah tiap butir, WAJIB jalan dan WAJIB lulus:

    arduino-cli compile -b teensy:avr:teensy41 --warnings all Hexapod_Unlimited

Empat pemeriksaan lain di CLAUDE.md tidak menyentuh kode yang diubah di sini,
tapi jalankan python cek_tabel_misi.py sekali di akhir untuk memastikan tidak
ada yang tersenggol. Tidak ada program uji di PC untuk butir-butir ini — repo
ini kehilangan test-pc/ (lihat CLAUDE.md). Kompilasi adalah satu-satunya
penjaga, jadi jangan bergantung padanya untuk membuktikan logika benar; baca
ulang sendiri.

KERJAKAN BERURUTAN, satu commit per butir:

--- BUTIR 1: profil waktu loop -----------------------------------------------

CONTROL_HZ dan PROFILE_LOOP ada di config.h:1344-1345 dan TIDAK DIBACA di satu
tempat pun. Komentar PROFILE_LOOP sudah menjanjikan 'cetak "PROF avg/max/util"
tiap detik (saat tak tuning)'. Wujudkan janji itu, jangan ubah janjinya.

- Ukur satu putaran loop() penuh dengan micros().
- Kumpulkan per jendela 1 detik: min, rata-rata, maks, dan BERAPA KALI putaran
  melewati 50 ms. Angka terakhir itu yang paling penting dan sebutkan
  alasannya di komentar: HexaGait::dtSeconds() meng-clamp dt ke 0,05 detik,
  jadi setiap putaran yang melewatinya membuat gait melangkah lebih pendek
  daripada yang dicatat odometer -- dan odometer memakai dPhase dari dt yang
  sama, sehingga keduanya salah bersama-sama dan saling membenarkan.
- Cetak satu baris per detik, di belakang #if PROFILE_LOOP.
- Senyap saat yawOn atau lidOn aktif (itu arti "saat tak tuning").
- CONTROL_HZ: JANGAN membuat pembatas laju loop. Semuanya sudah berbasis dt,
  jadi pembatas laju tidak menambah apa pun. Cukup perbaiki komentarnya supaya
  jujur bahwa angka itu tidak dipaksakan di mana pun.

--- BUTIR 2: konsol kalibrasi Y ----------------------------------------------

Perluas perintah 'Y' di handleCmd() (Hexapod_Unlimited.ino). 'Yt' yang sudah
ada adalah polanya: huruf kedua memilih subperintah. Ke-52 huruf besar-kecil
sudah terpakai, jadi JANGAN mengambil huruf baru di tingkat atas.

  Yo<slot> <der>   offset SUDUT per servo -> gOffset[]
  Yi<slot> <0|1>   invert per servo       -> gInvert[]
  Yj<slot> <us>    jog pulse MENTAH       -> Hexapod::jog()
  Yz<kaki> <mm>    offset tinggi telapak  -> EEPROM 2048, field zoff[]

Rinciannya:

Yo — tiru Hexapod::setTrim() persis (Hexapod.cpp:68). Bedanya: gOffset[] satuan
  DERAJAT (koreksi datum sudut), gTrim[] satuan mikrodetik (gigi horn). Dua
  pekerjaan berbeda; katakan bedanya di komentar. gOffset[] sudah ada di jalur
  servo (Hexapod.cpp:468 dan HexaArm.cpp:88) dan sudah ikut Calib::save(), jadi
  penyimpanannya lewat 'W' yang sudah ada -- BUKAN 'YtW'. Sebutkan itu di teks
  bantuannya, karena dua tombol simpan yang berbeda untuk dua hal yang
  bersebelahan adalah jebakan. Batas wajar +-30 der, clamp, dan katakan kalau
  di-clamp.
  Tambahkan barisnya ke cetakTrim() atau buat cetakOffset() sendiri. Kalau
  menumpang cetakTrim(), JANGAN mengubah awalan "#TRIM" atau jumlah kolomnya --
  HUD Raspi mencocokkan awalan itu (lihat komentar di Hexapod.cpp:89).

Yi — gInvert[] sudah ikut ditulis simpanServoMap() (Hexapod.cpp:123) dan sudah
  ikut dicetak cetakTrim(), jadi penyimpanan dan tampilannya GRATIS.
  WAJIB DITOLAK saat robot.isArmed(): membalik invert menggeser satu servo
  hampir 180 der seketika, tanpa ramp. Kelas bahaya yang sama dengan
  P_SERVO_LEMAS di perintah 'Q' -- tiru pesan penolakannya.

Yj — Hexapod::jog() SUDAH ADA (Hexapod.cpp:280) dan tidak pernah dipanggil
  siapa pun. Cukup sambungkan. Dua jebakan yang WAJIB kamu tulis di komentar
  dan di teks bantuan:
    a. jog() memakai TUNE_PIN_MAP (config.h:656), BUKAN ruang slot yang sama
       dengan 'Yt'/SLOT_NAMA. Untuk slot 0..17 (keenam kaki) urutannya
       kebetulan sama; untuk 18 ke atas TIDAK. TUNE_PIN_MAP hanya memuat 3
       servo per lengan, jadi grip DEPAN (slot yang dipetakan {0,15}) TIDAK
       BISA dijangkau 'Yj' sama sekali. Katakan itu, jangan diam-diam.
    b. jog() menulis PWM mentah tanpa memeriksa _enabled. Jadi 'Yj' pada robot
       yang sedang lemas akan MENGHIDUPKAN satu kanal itu saja, dan kanal itu
       tetap hidup sampai 'x'. Itu memang yang dibutuhkan saat memasang horn --
       tapi harus tertulis, bukan ditemukan sendiri.

Yz — baca-ubah-tulis GerakStore di EEPROM 2048. Polanya ada di
  Navigation::gerakSimpan() (Navigation.cpp:851): blok itu MILIK BERSAMA dengan
  sketsa TES_GERAK, jadi baca dulu seluruh struct, ubah satu field, hitung
  ulang eeSum(), tulis balik. Menulis nol ke field milik TES_GERAK membuang
  data yang cuma bisa didapat dengan menjalankan sketsa itu.
  Sesudah menulis, _zOff[] di Hexapod harus ikut disegarkan. loadZOff() sekarang
  private -- naikkan ke public atau tambahkan pembungkus publik; pilih yang
  diff-nya lebih pendek.

Terakhir: tambahkan keempatnya ke teks bantuan 'h', dan perbarui bagian
"KESELAMATAN & DIAGNOSTIK" / "EEPROM & KALIBRASI GERAK" supaya konsisten.

--- BUTIR 3: laju gambar OLED (BERSYARAT) ------------------------------------

HIPOTESIS, BELUM TERUKUR. OLED ada di Wire2 (Tampilan.h:42) dan Wire2 juga
SERVO_1_I2C_BUS (config.h:728) -- bus yang sama. Tampilan.cpp:154 memanggil
oled.display() tiap 125 ms, dan itu mengirim 512 byte secara MEMBLOKIR: pada
400 kHz kira-kira 12 ms, sementara periode commit servo 20 ms.

JANGAN langsung mengubahnya. Jalankan butir 1 dulu, minta pengguna menyalakan
robot dan membaca baris PROF, lalu:
  - kalau maks putaran loop memang melonjak ke belasan ms secara berkala ->
    ubah 125 menjadi 500 di Tampilan.cpp:154. Satu baris. OLED cuma dibaca
    manusia; 2 Hz lebih dari cukup.
  - kalau tidak -> JANGAN UBAH APA PUN, dan catat di CATATAN_MODIFIKASI_LAMPIRAN.md
    bahwa hipotesis A2 sudah diuji dan gugur, berikut angkanya.
Kalau pengguna tidak bisa menjalankan robot sekarang, LEWATI butir ini dan
katakan terus terang bahwa ia dilewati -- jangan tebak.

--- BUTIR 4: deteksi terguling -----------------------------------------------

Imu::accelZ() dibaca tapi HANYA dicetak di aliran yaw. Komentar di
Hexapod_Unlimited.ino:164 menuliskannya sendiri: itu satu-satunya angka yang
tahu robot sudah terguling. Tidak ada yang bertindak atasnya, padahal robot ini
menaiki tangga dengan margin guling 29,4 mm (lihat "cek_kail MERAH" di CLAUDE.md).

Di loop(), kalau |accelZ| di bawah ambang ATAU |roll| di atas ambang, bertahan
lebih lama dari jeda tunda -> misi.batal(), nav.navBerhenti(), robot.disarm(),
dan teriak ke Serial. Tiga angkanya jadi #define di config.h.

Empat hal yang WAJIB benar:
  - Ambangnya BELUM DIUKUR. Tulis itu di komentar. Nilai awal yang saya
    sarankan: |az| < 0,5 g, |roll| > 45 der, tunda 400 ms -- tapi tandai
    ketiganya sebagai titik awal untuk disetel di robot, bukan hasil ukur.
  - JANGAN menaruh ambang roll dekat kemiringan arena. Profil TANJAK sengaja
    bekerja pada 27,7 der (config.h sekitar baris 420) dan itu NORMAL. Ambang
    yang terlalu dekat akan membuat robot menyerah di tanjakan yang seharusnya
    ia naiki.
  - accelZ lebih layak dipercaya daripada roll di sini; katakan alasannya.
  - Butuh saklar MATI (#define, baku HIDUP). Ini menambah satu cara baru misi
    bisa berhenti sendiri, dan saat menyetel ambangnya orang perlu bisa
    mematikannya.

--- BUTIR 5: offset jarak per sensor LiDAR -----------------------------------

ST mewajibkan kalibrasi offset per sensor VL53L1X, dan wajib diulang begitu ada
kaca/akrilik penutup. Firmware ini tidak mengalibrasi satu pun dari enam
sensornya; yang ada cuma WALL_BIAS_KIRI_CM/KANAN_CM, dan itu bias SUDUT, bukan
offset jarak. Sementara WALL_KAKI_CM 7,0 diturunkan dari satu pengukuran yang
mengasumsikan keenam sensor sepakat.

  Yd            cetak tabel keenam offset
  Yd<ch> <cm>   "sensor ch sedang <cm> dari dinding" -> catat selisih bacaan
  Yd!           nolkan semua

- 6 float di LidarArray, dikurangkan di SATU tempat -- titik paling hilir yang
  masih sebelum semua pembaca. Cari sendiri di LidarArray.cpp di mana median +
  EMA selesai; jangan menaruhnya di dua tempat.
- JANGAN mengganti pustaka. Pustaka Pololu VL53L1X tidak mengekspos API
  kalibrasi ST, dan menukarnya dengan VL53L1X_ULD akan menulis ulang seluruh
  mesin state non-blokir yang sudah bekerja.
- RAM SAJA. Menyimpannya menuntut baris baru di PARAM_DEFS, dan itu menaikkan
  CALIB_VERSION -- dilarang di batch ini. Cetak "RAM saja, ulangi tiap robot
  menyala", persis seperti yang sudah dilakukan 'Ds' dan 'Y0'.
- Tolak pencatatan kalau sensornya sedang MATI atau JAUH (LIDAR_MATI / LIDAR_JAUH
  di config.h) -- mencatat offset dari bacaan tak sah adalah cara paling rapi
  merusak keenam sensor sekaligus.

--- SESUDAH SELESAI ----------------------------------------------------------

Perbarui:
  - Hexapod_Unlimited/README.md dan teks bantuan 'h' dengan perintah baru.
  - CATATAN_MODIFIKASI.md dan CATATAN_MODIFIKASI_LAMPIRAN.md: tandai butir yang
    selesai, dan kalau ada yang ternyata berbeda dari dugaan di sana, TULIS
    HASIL SEBENARNYA. Kedua berkas itu catatan kerja, bukan brosur.

Lalu laporkan, jujur: butir mana yang selesai, butir mana yang dilewati dan
kenapa, angka mana yang kamu tulis sebagai tebakan, dan apa yang harus diukur
pengguna di robot sebelum angka-angka itu bisa dipercaya.
```

---

## PROMPT 2 — sesudah geometri kaki diukur di robot

Jalankan **hanya sesudah** pengguna mengukur sudut femur dan sudut dalam lutut
dengan busur derajat pada `b100` (lihat "Geometri kaki salah" di `CLAUDE.md`).
Prompt ini membuang kalibrasi tersimpan — sekali, sengaja.

```
Lanjutan pekerjaan di repo Hexapod_KRSRI_2026, branch hexapod-v1.18. Aturan
keras, gaya komentar, dan cara verifikasi SAMA PERSIS dengan sesi sebelumnya --
baca ulang CLAUDE.md, CATATAN_MODIFIKASI.md, dan CATATAN_MODIFIKASI_LAMPIRAN.md
sebelum menyentuh apa pun.

Batch ini SATU-SATUNYA yang boleh menaikkan CALIB_VERSION, dan ia menaikkannya
TEPAT SEKALI. Kerjakan usul B di CATATAN_MODIFIKASI.md bagian 3:

- Buang 5 baris PARAM_DEFS yang bertanda P_BELUM_DIPAKAI dan memang tidak
  punya pembaca: head.utara, head.timur, head.selatan, head.barat, arena.mirror.
  JANGAN membuang stab.* -- ketiganya juga P_BELUM_DIPAKAI tapi masih calon
  pemakai (usul C, rata badan dari IMU).
- Tambah 11 baris: leg.coxa, leg.femur, leg.tibia (P_SERVO_LEMAS);
  stand.height, stand.radius (P_PERLU_B); odo.skala, odo.geser,
  wall.bias_kiri, wall.bias_kanan, arm.hand_len, korban.tinggi (P_LANGSUNG).
- Pindahkan #define yang bersangkutan dari config.h ke Calib.h, memakai pola
  yang SUDAH ADA di sana: #define GAIT_STEP_HEIGHT gParam[K_...]. Tidak satu
  pun tempat pemakaian boleh berubah -- sudah diperiksa, tak ada yang memakai
  konstanta ini di inisialisasi statis. Kalau ternyata ada, BERHENTI dan lapor.
- Rentang [lo,hi] tiap baris harus masuk akal secara fisik, bukan angka bulat
  asal. Turunkan dari komentar yang sudah ada di config.h.
- Nilai def leg.* dan stand.*: pakai angka HASIL UKUR yang saya berikan di
  bawah, bukan angka lama yang sudah terbukti salah.

  [DI SINI PENGGUNA MENEMPELKAN HASIL UKURNYA]

- Sambungkan 'Ds' ke odo.skala dan 'Y0' ke wall.bias_* supaya keduanya
  berhenti hilang tiap robot menyala. Sambungkan juga offset LiDAR dari butir 5
  sesi lalu kalau sudah ada -- kalau perlu tambah 6 baris lagi untuk itu,
  lakukan SEKARANG, karena ini satu-satunya kesempatan gratis.
- Di teks 'W', ingatkan bahwa 'q' menandai '*' pada parameter yang berbeda dari
  default, dan itulah daftar yang harus diketik ulang sesudah CALIB_VERSION
  naik. Jangan menulis alat migrasi -- 'q' sudah jadi alatnya.

Verifikasi: arduino-cli compile + keempat cek di CLAUDE.md. Lalu laporkan
daftar parameter yang nilainya akan hilang saat pengguna mem-flash, supaya ia
bisa memfotonya dulu.
```

---

## PROMPT 3 — butir kecil, kapan saja sesudah prompt 1

Tidak bergantung pada prompt 2. Bisa dikerjakan paralel.

```
Lanjutan repo Hexapod_KRSRI_2026, branch hexapod-v1.18. Aturan keras dan gaya
komentar SAMA dengan sesi sebelumnya -- baca ulang CLAUDE.md dan
CATATAN_MODIFIKASI.md dulu. JANGAN menaikkan CALIB_VERSION.

Dua butir kecil:

A. KOLOM PROFIL GAIT BISA DISETEL LANGSUNG. Sekarang tinggi langkah hanya bisa
   diubah lewat 'Qgait.step_height' yang bertanda P_PERLU_B, dan 'b' sekaligus
   menolkan pose badan dan mengembalikan profil ke DATAR. Tambahkan, meniru
   pola 'b<mm>' yang sudah ada:
     Th<mm> tinggi langkah   Tl<mm> panjang langkah   Tc<ms> waktu siklus
     Tr<mm> radius kaki      Tb<mm> tinggi badan
   Isinya: ambil robot.gaitProfile(), ganti SATU kolom, pasang lagi.

   JEBAKAN YANG WAJIB DIURUS. setGaitProfile() -> pasangProfil() ->
   HexaGait::setProfile() MENGHAPUS offset kaki (lihat komentarnya di
   HexaGait.h). Jadi 'Th60' saat profil KAIL/TANJAK aktif akan meratakan bentuk
   kakinya diam-diam. Tambahkan HexaGait::setKolomProfil() yang hanya menyentuh
   _tgtProf dan TIDAK menyentuh _tgtOff, plus pembungkusnya di Hexapod, dan
   pakai itu. Tulis di komentarnya kenapa ini TIDAK melanggar invarian "satu
   pintu pemasangan profil": ini penyetelan kolom, bukan pemasangan profil.

B. SUMBU PUTAR DI PERINTAH GERAK MANUAL. Hexapod::walk() sudah menerima
   (maju, geser, putar) dan HexaGait sudah menormalkan ketiganya bersama --
   Navigation.cpp:1626 sudah memakainya. Yang tidak punya jalan masuk hanya
   operator: ketiga pemanggil walk() di .ino mengisi sumbu putar 0.0f.

   Buka sebagai argumen KEEMPAT: 'w <maju> <geser> [detik] [putar]'. Jangan
   menukar urutannya -- 'w 0.5 0 3' sudah ada di README dan di kepala orang,
   dan artinya harus tetap sama. Penjaga yang sudah ada (jarakArahJalan(),
   GERAK_AMAN_CM, penjaga buta, batas waktu) berlaku apa adanya; jangan
   longgarkan satu pun. Perbarui teks bantuan dan README.

JANGAN membuka mode gerak menerus tanpa batas waktu. Batas 0,5-30 detik itu
satu-satunya penjaga saat jarakArahJalan() mengembalikan -1, dan alasannya
tertulis panjang di .ino sekitar baris 1240.

Verifikasi: arduino-cli compile -b teensy:avr:teensy41 --warnings all
Hexapod_Unlimited, lalu python cek_tabel_misi.py.
```

---

## Yang sengaja TIDAK ada di ketiga prompt

- **Kompensasi CG** (lampiran B2) — usul terkuat dari referensi, tapi ia
  memakan jangkauan IK yang kaki belakang sudah pakai 95%-nya. Butuh geometri
  kaki yang benar lebih dulu (prompt 2), dan butuh profil waktu loop untuk
  mengukur akibatnya (prompt 1). Prompt keempat, sesudah keduanya.
- **Tegangan bus servo** (lampiran B3) — butuh pembagi tegangan ke pin ADC.
  Perangkat keras dulu.
- **Rata badan dari IMU** (catatan 1, usul C) — belum kamu putuskan, dan ia
  satu-satunya usul yang bisa menjatuhkan robot kalau tandanya terbalik.
- **Bézier, watchdog, CPG, gait wave, footstep planning** — sudah ditolak
  berikut alasannya. Prompt 1 menyebutkan penolakannya secara tegas supaya AI
  lain tidak menawarkannya lagi.
