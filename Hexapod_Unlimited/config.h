#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define NUM_SERVOS 18 // 18
#define NUM_TUNE_SERVOS 24 // 24
#define ARM_NUM_SERVOS 3 // 3 per lengan

// --- Dimensi Kaki Hexapod --- //
// SUMBER KEBENARAN: legacy-2026/TES_GERAK/kinematics.h (program yang sudah
// terbukti berdiri). Nilai lama 23/54/69 SALAH -- femur meleset 26 mm dan
// tibia 21 mm, sehingga IK menghitung femur -35 der (seharusnya -10,6 der)
// dan lutut 127 der (seharusnya 82 der) pada pose berdiri baku.
// Jangan diubah tanpa mengukur ulang kaki fisik.
#define COXA_LENGTH  20.0f
#define FEMUR_LENGTH 80.0f
#define TIBIA_LENGTH 90.0f

#define UPPERARM_LENGTH 20.0f // panjang ini masih tes
#define FOREARM_LENGTH 24.0f // panjang ini masih tes

#define STAND_HEIGHT  100.0f // Tinggi badan dari tanah
#define STAND_RADIUS   70.0f // Jauh kaki ke pangkal coxa

// Posisi pangkal coxa tiap kaki dari pusat (mm).
const float BODY_LEG_ORIGINS[6][3] = {
    { 45.0f,  78.0f, 0.0f},  // 0 (Kaki kanan depan)
    { 90.0f,   0.0f, 0.0f},  // 1 (Kaki kanan)
    { 45.0f, -78.0f, 0.0f},  // 2 (Kaki kanan belakang)
    {-45.0f, -78.0f, 0.0f},  // 3 (Kaki kiri belakang)
    {-90.0f,   0.0f, 0.0f},  // 4 (Kaki kiri)
    {-45.0f,  78.0f, 0.0f}   // 5 (Kaki kiri depan)
};

// Arah hadap kaki (derajat, dari +X, CCW positif).
const float BODY_LEG_ANGLE[6] = {
     60.0f,    // 0 (Kaki kanan depan)
      0.0f,    // 1 (Kaki kanan)
    -60.0f,    // 2 (Kaki kanan belakang)
   -120.0f,    // 3 (Kaki kiri belakang)
    180.0f,    // 4 (Kaki kiri)
    120.0f     // 5 (Kaki kiri depan)
};

// Peta pin servo kaki (driver 0 = PCA9685 address (0x41 Wire1)/driver 1 = PCA9685 address (0x40 Wire2))
const uint8_t SERVO_PIN_MAP[NUM_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0 (coxa,femur,tibia)
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2}   // Kaki 5
};

const uint8_t TUNE_PIN_MAP[NUM_TUNE_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2},  // Kaki 5
    {0, 12}, {0, 13}, {0, 14}, // Lengan kanan (base,shoulder,gripper)
    {1, 12}, {1, 13}, {1, 14}  // Lengan kiri
};

// --- Lengan: DEPAN & BELAKANG (bukan kanan/kiri) --- //
// Tiap lengan = BAHU, SIKU, GRIP. Dua sendi pertama menyapu satu bidang
// VERTIKAL; tidak ada sendi pemutar di pangkal, jadi untuk membidik objek
// yang tidak segaris, BADAN robot yang harus diarahkan.
//
// BELUM DIVERIFIKASI FISIK: mana dari kedua papan PCA yang memegang lengan
// depan. Kalau saat diuji ('a' vs 'A') yang bergerak justru tertukar,
// TUKAR SAJA kedua baris di bawah -- tidak ada yang lain perlu diubah.
const uint8_t ARM_PIN_MAP_DEPAN[ARM_NUM_SERVOS][2]    = { {0, 12}, {0, 13}, {0, 14} }; // bahu, siku, grip
const uint8_t ARM_PIN_MAP_BELAKANG[ARM_NUM_SERVOS][2] = { {1, 12}, {1, 13}, {1, 14} }; // bahu, siku, grip

// Pangkal BAHU tiap lengan (x, y, z) dari pusat badan, mm.
// Lengan depan menghadap +Y (maju), lengan belakang menghadap -Y (mundur).
// Versi lama keliru menaruh offset di sumbu X (kanan/kiri) -- itu sisa dari
// asumsi "lengan kanan & kiri" yang ternyata salah.
// ANGKA 50 MASIH PERKIRAAN -- ukur ulang saat lengan terpasang.
#define ARM_DEPAN     0
#define ARM_BELAKANG  1
const float ARM_ORIGINS[2][3] = {
    { 0.0f,  50.0f, 30.0f},  // ARM_DEPAN    : 50 mm di depan pusat, 30 mm di atas
    { 0.0f, -50.0f, 30.0f}   // ARM_BELAKANG : 50 mm di belakang pusat
};

// BUS I2C (Wire SDA 18 / SCL 19 Teensy 4.1)
#define LIDAR_I2C_BUS    Wire
#define SERVO_0_I2C_BUS  Wire1
#define SERVO_1_I2C_BUS  Wire2

#define SERVO_I2C_CLOCK  400000
#define LIDAR_I2C_CLOCK  400000

// --- LiDAR (VL53L1X) --- //
#define NUM_LIDAR        6
#define I2C_MUX_ADDR     0x70
#define LIDAR_EMA_ALPHA  0.4f    // bobot sampel baru (median dulu, lalu EMA)
#define LIDAR_TIMEOUT_MS 300     // sensor dianggap mati bila tak ada data valid

// MODE JARAK. Long TERLIHAT paling menggoda, tapi di arena bercahaya justru
// paling pendek -- jangkauannya anjlok karena cahaya sekitar:
//
//      mode     gelap    cahaya terang    anggaran waktu minimum
//      Short    136 cm       135 cm             20 ms
//      Medium   290 cm        76 cm             33 ms
//      Long     360 cm        73 cm             33 ms
//
// Navigasi tidak pernah memakai jarak di atas NAV_PELAN_CM (50 cm) untuk
// mengemudi, jadi Short sudah lebih dari cukup -- hampir kebal cahaya sekitar,
// dan anggaran 20 ms mempercepat laju sampel, yang langsung memperbaiki suku
// turunan PD karena turunan dihitung pada laju sampel LiDAR.
#define LIDAR_MODE_SHORT   0
#define LIDAR_MODE_MEDIUM  1
#define LIDAR_MODE_LONG    2
#define LIDAR_MODE       LIDAR_MODE_SHORT

#define LIDAR_BUDGET_US  20000   // anggaran waktu per pengukuran (us)
#define LIDAR_PERIOD_MS  25      // jeda antar pengukuran (ms), >= anggaran waktu
// BATAS ATAS. Sempat 70 cm: hasil uji fisik saat bacaan di luar itu tidak
// melaporkan angka besar melainkan JATUH KE HANTU 5 cm. Sesudah
// LIDAR_ROI_SEMPIT dinyalakan, hantu itu hilang ('j5 10' -> 100% signal fail),
// jadi mekanisme gagal yang membuat 70 masuk akal sudah tidak berlaku.
//
// 130 cm = batas mode SHORT menurut lembar data VL53L1X. Menaikkannya sampai
// batas mode berarti dinding sejauh 1 meter terbaca sebagai ANGKA, bukan
// sebagai "jauh" -- di mode arena itu bedanya antara mengemudi menuju dinding
// yang terlihat dan menyalakan perintah cari karena dinding dianggap hilang.
//
// Kalau naik ke Medium/Long, angka ini ikut naik (Medium ~290, Long ~360) DAN
// LIDAR_BUDGET_US harus >= 33000 -- static_assert di LidarArray.cpp menjaganya.
#define LIDAR_MAX_CM     130     // di atas ini dianggap "jauh", bukan rusak

// BATAS BAWAH PER ARAH -- bacaan di bawah ini MUSTAHIL berasal dari benda
// sungguhan, karena kaki robot sudah menabraknya lebih dulu:
//
//   samping (ch0,1,3,4) : ujung kaki tengah 160 mm - sensor 50 mm = 110 mm
//   depan   (ch5)       : ujung kaki depan  139 mm - sensor 62 mm =  77 mm
//   belakang(ch2)       : ujung kaki blkg   139 mm - sensor 66 mm =  73 mm
//
// Ini BUKAN sekadar penyaring hantu. Di robot ini "tak ada objek dalam
// jangkauan" MUNCUL SEBAGAI bacaan pendek 5 cm, bukan sebagai status
// SignalFail -- jadi bacaan di bawah batas ini artinya justru KOSONG, kebalikan
// dari "ada halangan sangat dekat". Menafsirkannya sebagai halangan membuat
// robot berbelok menghindari lorong yang sebenarnya terbuka lebar.
//
// Diambil sedikit di bawah angka geometri di atas supaya benda nyata yang
// benar-benar mepet tetap terbaca. Urutan indeks = channel mux.
//
// SEMPAT DITURUNKAN KE 2 cm SEMUANYA, lalu dikembalikan: sim_depan langsung
// merah. Dengan ambang 2, hantu 5 cm di sensor depan dibaca sebagai halangan
// sungguhan, mode arena berbelok menghindari lorong yang kosong, dan maju
// jatuh ke 0 -- persis gejala yang dulu membuat 'm1' berputar di tempat.
//
// Menurunkannya juga tidak menambah kemampuan ukur apa pun: kaki sudah
// menabrak benda sebelum sensor bisa membacanya sedekat itu, jadi bacaan di
// bawah angka geometri di atas TIDAK MUNGKIN berasal dari benda nyata.
// SISI DITURUNKAN 10 -> 4 cm. Sebabnya ditemukan di arena: ambang 10 pada
// sensor SAMPING membuat jerat, bukan sekadar penyaring.
//
// Kaki menyentuh dinding saat bacaan 11 cm, jadi rentang 10..11 cm sudah
// hampir menempel -- dan tepat di bawah 10 bacaannya dipetakan ke LIDAR_JAUH,
// yang bagi ikut-dinding berarti "dinding HILANG". Reaksinya
// 'turn = sisi * NAV_CARI_CMD', yaitu MEMBELOK KE ARAH dinding untuk
// mencarinya. Ikut dinding kanan -> sisi = -1 -> membelok ke kanan, masuk ke
// dinding yang sebenarnya sudah menempel.
//
// Jadi tandanya TERBALIK persis di daerah yang paling berbahaya: makin dekat,
// makin keras robot merapat. Itu yang terlihat sebagai "sering serong ke
// kanan" saat ikut dinding kanan.
//
// Dengan 4 cm, seluruh rentang 4..13 cm dipercaya dan ditangani pita
// "terlalu dekat", yang mendorong MENJAUH dengan kekuatan penuh. Sensor
// samping yang macet di bacaan pendek tidak lagi lolos tanpa terdeteksi:
// NAV_DEKAT_BATAS_MS menghentikan robot sesudah 10 detik di pita itu.
//
// DEPAN dan BELAKANG tetap 7. Keduanya tidak punya jerat ini -- "jauh" di
// sensor depan berarti "lorong kosong", bukan "kejar dindingnya" -- sementara
// hantu 5 cm di ch5 punya bukti sim tersendiri (sim_depan) bahwa tanpa ambang
// itu robot berbelok menghindari lorong yang terbuka lebar.
const uint8_t LIDAR_MIN_CM[6] = {
     4,  // ch0 kiri depan   (samping)
     4,  // ch1 kiri belakang(samping)
     7,  // ch2 belakang
     4,  // ch3 kanan blkg   (samping)
     4,  // ch4 kanan depan  (samping)
     7   // ch5 depan
};

// JARAK BACAAN SAAT KAKI SAMPING MENYENTUH DINDING. Bukan penyaring sensor --
// ini geometri badan: ujung kaki tengah 160 mm - dudukan sensor 50 mm = 110 mm.
//
// Dipakai pita "terlalu dekat" di Navigation sebagai ujung rampnya: dorongan
// menjauh separuh di wall.min, PENUH di sini. Dulu ia meminjam
// LIDAR_MIN_CM[samping], yang kebetulan bernilai mirip -- dua makna menumpang
// pada satu angka. Dipisah supaya menyetel penyaring sensor tidak diam-diam
// menggeser titik dorongan penuh.
#define WALL_KAKI_CM     11.0f

// PENJAGA "terkurung di pita terlalu dekat" DIBUANG, sesudah ia menghentikan
// misi di ruas turunan dua kali berturut-turut: sensor kanan bertahan 11,7 cm
// sementara robot sebenarnya sudah di tengah dan tidak punya ke mana bergeser.
//
// Sempat dicoba menyempitkannya jadi "hanya salah kalau sisi SEBERANG lega",
// tapi itu bocor: robot yang didorong hantu menyeberang sampai mepet dinding
// lawan, dan di situ kedua sisi sempit sehingga penjaganya diam lagi. Tidak
// ada cara bersih membedakan lorong sempit dari sensor macet hanya dari jarak.
//
// Yang menggantikannya: MISI_RUAS_BATAS_MS (90 detik) untuk ruas misi, dan
// tangan pengguna untuk 'F'/'P' manual. Harganya nyata -- sensor yang macet
// di bawah wall.min pada sisi yang diikuti kini membuat robot menyusuri
// dinding hantu tanpa batas waktu di mode manual.

// Sempat ditambal ke 11 cm saat trial, lalu DICABUT kembali ke angka
// geometrinya. Riwayatnya layak diingat karena diagnosisnya yang berguna,
// bukan tambalannya: sensor depan melaporkan hantu menetap berstatus
// 'range valid' yang BERPINDAH, 3,2 cm lalu 9 cm. Pada 9 cm ia lolos ambang
// ini, dibaca sebagai halangan sungguhan, dan karena 9 <= FRONT_STOP_CM mode
// arena berbelok terus-menerus sampai batas waktu misi.
//
// Yang menyelesaikannya LIDAR_ROI_SEMPIT, bukan ambang ini: sesudah kerucut
// dipersempit ke 15 der, 'j5 10' melaporkan 100% signal fail. Artinya
// hantunya tertangkap di PINGGIR bidang pandang -- kaki, braket, atau
// pancaran sensor tetangga -- bukan di sumbu optik.
//
// Kalau ia muncul lagi: JANGAN naikkan angka ini. Ia sudah terbukti bisa
// berpindah, dan mengejarnya berakhir di FRONT_STOP_CM 20, yaitu sensor
// depan mati fungsi. Mulai dari 'u5 4' untuk memisahkan crosstalk
// antar-sensor dari benda nyata di sumbu.

// ROI 4x4 menyempitkan bidang pandang dari ~27 der ke ~15 der. Berguna kalau
// dicurigai sensor tetangga saling melihat (crosstalk) -- dua sensor sisi yang
// sama menghadap arah yang SAMA di robot ini, jadi kerucut 27 der keduanya
// benar-benar tumpang tindih. MATI secara default: bidang pandang sempit juga
// membuat dinding lebih mudah luput saat robot menyerongi dinding.
#define LIDAR_ROI_SEMPIT 1       // 1 = setROISize(4,4)
// DINYALAKAN, dan terbukti di arena: hantu menetap di sensor depan hilang
// sepenuhnya sesudahnya ('j5 10' -> 100% signal fail). Jadi baris "MATI
// secara default" di atas sudah tidak berlaku untuk robot ini.
//
// Harganya tetap seperti yang tertulis: bidang pandang 15 der membuat dinding
// samping lebih mudah luput saat badan menyerong. Kalau ikut-dinding terasa
// lebih goyah dari sebelumnya, itu ini -- jangan dikejar dengan menyetel
// wall.kp.

// Tiga keadaan yang dikembalikan getDistance(). Membedakan "tak ada objek
// dalam jangkauan" dari "sensor putus" itu penting: untuk wall-follow,
// keduanya butuh reaksi yang berlawanan.
#define LIDAR_JAUH       999     // sensor sehat, tak ada objek dalam jangkauan
#define LIDAR_MATI       (-1)    // sensor tidak merespons / belum ada data

// ARAH FISIK -> CHANNEL MUX. Hasil uji fisik Agustus 2026.
//
// JANGAN mengasumsikan channel 0 menghadap depan. Urutan kabel di robot ini
// ternyata TERBALIK terhadap urutan yang diasumsikan kode lama: channel n
// memegang arah yang dulu diberi indeks (5 - n). Keempat arah yang sempat
// diperiksa satu per satu semuanya cocok dengan pola itu, dan pola yang sama
// meramalkan dua sisanya (ch0 dan ch2), yang waktu itu rusak fisik sehingga
// tidak bisa diuji. Sejak September 2026 keenam sensor hidup -- jadi arah ch0
// dan ch2 SEKARANG BISA, dan PERLU, diuji langsung dengan 'l'. Sampai itu
// dilakukan keduanya masih berstatus ramalan pola, bukan hasil ukur.
//
//   ch0 = KIRI DEPAN     ch1 = KIRI BELAKANG    ch2 = BELAKANG
//   ch3 = KANAN BELAKANG ch4 = KANAN DEPAN      ch5 = DEPAN
//
// ARAH BERKAS (bukan sekadar posisi dudukan). Ini BUKAN cincin berjarak 60 der:
//
//   ch0, ch1 : menghadap KIRI  -- TEGAK LURUS dinding
//   ch3, ch4 : menghadap KANAN -- TEGAK LURUS dinding
//   ch5      : menghadap DEPAN     ch2 : menghadap BELAKANG
//
// Nama "kiri depan / kiri belakang" hanya menerangkan DI MANA sensornya duduk
// di badan, bukan ke mana ia memandang. Bedanya penting untuk wall-follow:
// berkas tegak lurus mengukur jarak dinding apa adanya, sedangkan berkas
// menyerong akan memanjang sebesar 1/cos(sudut serong) dan menghasilkan umpan
// balik positif saat robot menoleh (lihat README bagian 7.7).
//
// Sensor samping duduk ~50 mm dari pusat badan (diukur dari gambar tata letak:
// ch0/ch1 di 49,8 mm, ch3/ch4 di 52,0 mm), sedangkan ujung kaki TENGAH ada di
// 160 mm. Jadi kaki sudah menyentuh dinding saat sensor masih membaca ~11 cm --
// itulah dasar wall.setpoint / wall.min / wall.hantu di Calib.cpp.
//
// Akibat pemetaan lama: navigasi membaca channel 0 sebagai "depan", padahal
// channel 0 justru salah satu sensor yang mati -- jadi 'f'/'F' selalu langsung
// berhenti dengan "sensor DEPAN tidak merespons", berapa pun gain-nya.
// Nama = ARAH BERKAS dulu, posisi dudukan belakangan. Nama lama (FRONT_R,
// BACK_R, ...) terbaca seperti "menghadap depan / menghadap belakang" padahal
// keempatnya menghadap ke SAMPING -- salah baca yang sudah terjadi berkali-kali.
//   _D = duduk di paruh depan badan, _B = duduk di paruh belakang.
#define LIDAR_FRONT      5   // satu-satunya yang menghadap DEPAN
// JARAK MEMBUJUR antara dudukan sensor DEPAN dan BELAKANG di sisi yang sama,
// diukur mistar dari tengah lensa ke tengah lensa. Ini yang mengubah selisih
// dua bacaan jadi SUDUT: sudut = atan((d_belakang - d_depan) / jarak ini.
//
// Ia juga yang menentukan resolusinya. Dengan bacaan halus ~0,2 cm, dasar
// 11 cm memberi atan(0,2/11) = 1,0 der per langkah. Di bawah ~8 cm sudutnya
// tenggelam di derau sensor dan hasilnya tidak berguna.
//
// Ukuran mekanis, per-robot. Ganti bila dudukan sensor dipindah.
#define WALL_BASE_CM     11.0f

// SIMPANGAN PEMASANGAN pasangan sensor tiap sisi, (belakang - depan) dalam cm
// saat badan SEJAJAR dinding. Diukur di robot dengan 'Y0': 2,00 cm di kedua
// sisi, sama persis dua kali pengukuran. Tanpa dikurangkan, robot mengira
// dirinya menyerong 10 der padahal lurus.
//
// Nilai awal saja -- 'Y0' menimpanya kapan pun. Ada di sini supaya turunan
// dari sudut sudah benar sejak menyala, tanpa perlu mengetik apa pun.
// Ambang sensor DEPAN untuk ruas terakhir di bawah turunan, cm. 'm5 <cm>'
// menimpanya saat jalan.
#define MISI_DEPAN_CM_DEF   40

// Jarak yang HARUS ditempuh dulu di ruas terakhir sebelum bacaan sensor depan
// dipercaya. Di ujung turunan badan masih miring: kaki depan sudah di lantai
// datar sementara kaki belakang masih di bidang miring, jadi berkas depan
// masih menembak TANAH. Sim mengukurnya 15 cm -- di bawah FRONT_STOP_CM, jadi
// mode arena langsung berbelok dan misi gagal sebelum ruas ini mulai.
//
// 30 cm kira-kira satu panjang badan: cukup untuk keenam kaki lepas dari
// bidang miring. Selama jarak itu sensor depan tetap DIABAIKAN, persis seperti
// di ruas turunan; sesudahnya baru dinyalakan dan diawasi.
#define MISI_AKHIR_MIN_CM   30

#define WALL_BIAS_KIRI_CM   2.0f
#define WALL_BIAS_KANAN_CM  2.0f

#define LIDAR_KANAN_D    4   // menghadap kanan, dudukan depan
#define LIDAR_KANAN_B    3   // menghadap kanan, dudukan belakang
#define LIDAR_BACK       2   // satu-satunya yang menghadap BELAKANG
                             // hidup sejak Sept 2026; belum dipakai navigasi
#define LIDAR_KIRI_B     1   // menghadap kiri, dudukan belakang
#define LIDAR_KIRI_D     0   // menghadap kiri, dudukan depan
                             // hidup sejak Sept 2026; arah fisik belum diuji ('l')

// --- IMU --- //
#define IMU_MAX_YAW_JUMP 30.0f   // derajat/sample; lonjakan > ini ditolak (gangguan magnet)
// Berapa penolakan BERTURUT-TURUT sebelum nilai baru diterima paksa.
// Tanpa jalan keluar ini, satu pergeseran heading yang MENETAP (mis. IMU
// re-referensi, atau medan magnet arena berubah permanen) membuat _yaw
// membeku selamanya dan navigasi berjalan di atas heading basi.
#define IMU_MAX_YAW_TOLAK  10
// Batas byte yang diproses per update(). IMU 230400 baud memasok ~23 kB/s;
// tanpa batas, update() bisa terus terjebak menguras serial dan loop utama
// (termasuk parser perintah) tak pernah kebagian giliran.
#define IMU_MAX_BYTE_UPDATE 512
#define IMU_SERIAL       Serial2
#define IMU_BAUD         230400   // Yahboom 10-axis (protokol WIT, frame 0x55)

// --- Navigasi --- //
#define HEADING_TOLERANCE_DEG   6.0f     // Toleransi heading dianggap "lurus" (const)
#define FRONT_STOP_CM     20       // Berhenti/belok bila depan < ini (const)
#define NAV_FWD_SPEED     0.8f     // Kecepatan maju normal (0..1) (const)

// FAKTOR SLIP ODOMETRI -- diukur di lantai arena, 2026-09-02.
// Hasilnya: TIDAK ADA slip yang perlu dikoreksi. Faktornya 1,0.
//
// Tiga lari, semuanya D50 dengan skala 1,0, diukur meteran:
//
//   lurus 'w',  kemarin  : meteran 46,0  odometer 51,4  -> 0,895
//   dinding 'P', hari ini: meteran 51,5  odometer 51,4  -> 1,002
//   lurus 'w',  hari ini : meteran 52,0  odometer 51,4  -> 1,012
//
// Dua yang terakhir sepakat dalam 1%. Yang pertama sendirian, dan selisihnya
// 6 cm -- kira-kira jarak antara ujung kaki tengah dan tepi badan, jadi
// hampir pasti titik acuan awal dan akhir tidak sama. Ia dibuang.
//
// Lengkungan lintasan sempat dicurigai sebagai penyebab, karena 'w' berjalan
// terbuka tanpa koreksi arah apa pun. Diperiksa dan DITOLAK: yaw hanya
// bergeser 175,7 -> 170,6 der selama lari itu, dan untuk busur dengan total
// belok 5,1 der rasio tali-busur terhadap busur adalah 2*sin(t/2)/t = 0,9997.
// Lengkungan sebesar itu memendekkan jarak 0,03%, bukan 10,5%.
//
// Kalau suatu saat mengukur ulang: pakai SATU titik badan yang sama untuk
// tanda awal dan pengukuran akhir. Itu satu-satunya sumber galat yang pernah
// benar-benar muncul di sini, dan besarnya sekelas dengan slip yang dicari.
#define ODO_SKALA_DEF     1.0f     // tanpa koreksi; 'Ds' menimpanya di RAM (0,5..1,5)

// --- Wall-following & penghindaran halangan (non-blokir) --- //
#define NAV_PELAN_CM       50    // mulai melambat bila halangan depan di bawah ini
#define NAV_MAJU_MIN       0.15f // faktor kecepatan terendah saat mendekati halangan
#define NAV_BELOK_CMD      0.60f // kekuatan putar saat menghindar halangan depan
#define NAV_CARI_CMD       0.35f // kekuatan putar saat dinding samping hilang (tikungan)
#define NAV_BELOK_BATAS_MS 8000  // berbelok lebih lama dari ini = dianggap terjebak
// Dinding samping hilang lebih lama dari ini -> berhenti, jangan berputar
// selamanya. Tanpa batas ini, sensor samping yang macet melapor "kosong"
// (mis. jatuh ke hantu) membuat robot memutar NAV_CARI_CMD tanpa henti dan
// berjalan melingkar di tengah arena. Hanya berlaku di mode 'f'/'F' polos:
// di mode arena heading yang mengunci arah, jadi dinding hilang tidak
// membuatnya melingkar. 10 detik ~ 86 cm perjalanan pada laju sekarang, cukup
// untuk melewati mulut lorong yang lebar.
#define NAV_CARI_BATAS_MS 10000

// --- Mode terkunci kompas arena --- //
// Heading arena jadi acuan SUDUT (lorong arena sejajar sumbu mata angin),
// dinding jadi koreksi LATERAL. Keduanya tidak berebut: satu mengurus arah
// hadap, satu mengurus posisi di tengah lorong.
#define NAV_WALL_TURN_MAX  0.50f // batas sumbangan kemudi dari dinding
#define NAV_PIVOT_BATAS_MS 12000 // timeout satu kali belok 90 derajat

// GAIN PIVOT SEKARANG DI Calib: heading.kp / heading.kd (lihat kemudiHeading()).
// Dulu ada PIVOT_KP/PIVOT_KD di sini yang nilainya PERSIS SAMA dengan default
// heading.kp/kd, tapi hanya dibaca pivotKe(). Akibatnya fase belok arena dan
// pivot manual punya knob terpisah tanpa alasan, dan knob yang di config.h
// tidak bisa disetel tanpa kompilasi ulang. Sekarang keduanya satu knob.
#define PIVOT_MIN_CMD   0.25f    // Minimal gaya agar tidak cuma "menggeliat"
#define PIVOT_DIAM_MS   500      // Harus di dalam toleransi selama ms ini
#define PIVOT_SETTLE_MS 800      // Sesudah sampai: perintah putar 0, tunggu kaki diam
#define PIVOT_BATAS_MS  20000    // Batas waktu timeout (20 detik)

// --- Peta EEPROM --- //
// Alamat DIPATOK karena keempat blok ditulis program berbeda yang tidak
// pernah dikompilasi bersama; alamat inilah kontraknya. Jarak antar blok
// itu ruang tumbuh yang disengaja, bukan pemborosan (total terpakai ~502 B
// dari 4284 B). Struct + penjaga static_assert-nya ada di EEMap.h.
//    0 : CalibBlob   (272 B) firmware sendiri -- param, offset, trim, invert
// 1024 : ServoMap    (126 B) TES_SERVO/SET_HOME -- invert & trim hasil uji fisik
// 1792 : KompasStore  (24 B) TES_IMU -- 4 arah arena
// 2048 : GerakStore   (80 B) TES_GERAK -- pivot, odometri, rata badan, zOff
#define EE_CALIB_ADDR      0     // Blok kalibrasi firmware (Calib.cpp)
#define EE_SERVOMAP_ADDR 1024    // Peta servo hasil kalibrasi fisik
#define EE_KOMPAS_ADDR   1792    // Alamat memori untuk data kompas arena
#define EE_GERAK_ADDR    2048    // Kalibrasi gerak dari TES_GERAK

// Kapasitas EEPROM papan sasaran. Teensy 4.0/4.1 = 4284 byte (flash-emulated).
// Ini hanya dugaan saat kompilasi -- eeMapPeriksa() membandingkannya dengan
// EEPROM.length() yang sebenarnya saat boot.
#define EE_TOTAL_BYTES  4284

// --- Lain-Lain --- //
// #define PIN_BUTTON_START  30   // Tombol mulai (INPUT_PULLUP)
// #define PIN_LED_FOUND     13   // LED tanda korban ditemukan (cek aturan lomba)

// Stabilisasi badan (IMU)
#define STAB_MAX_DEG      15.0f   // Clamp koreksi roll/pitch (const)
#define STAB_DEADBAND_DEG 1.0f    // Abaikan getaran kecil (const)

// --- Body kinematics (pose badan manual & demo uji) --- //
// Batas ini bukan batas mekanis kaki, melainkan pagar supaya perintah uji
// tidak langsung melempar IK ke luar jangkauan. Kalau muncul peringatan
// "di luar jangkauan IK", kecilkan angkanya atau turunkan STAND_HEIGHT.
#define BODY_MAX_ROT_DEG   20.0f  // clamp roll/pitch/yaw perintah manual
#define BODY_MAX_TRANS_MM  40.0f  // clamp geser badan X/Y/Z

// LAJU RAMP POSE BADAN. Tanpa ini, 'r20 0 0' mengubah sudut femur ~49 der
// dalam SATU siklus commit 20 ms (~550 us lompatan pulse) -- servo disuruh
// bergerak ~2400 der/detik dan robot menyentak keras. Vektor gerak gait sudah
// di-slew (GAIT_SLEW_RATE) dan profil medan sudah di-ramp (GAIT_PROFILE_TAU),
// tapi pose badan dulu diterapkan MENTAH karena letaknya sesudah gait.
// Demo 'B' tidak terpengaruh: sapuan sinusnya paling cepat ~21 der/detik.
#define BODY_SLEW_DEG_S    60.0f  // laju maks rotasi badan, derajat/detik
#define BODY_SLEW_MM_S    120.0f  // laju maks geser badan, mm/detik
#define BODY_DEMO_ROT_DEG  10.0f  // amplitudo rotasi saat demo 'B'

// --- DEMO OTOMATIS SAAT MENYALA -- SEMENTARA, UNTUK PAJANGAN --------------
//
// MEMATIKANNYA: ganti satu angka di bawah jadi 0, lalu unggah ulang.
//
// PERINGATAN KESELAMATAN. Robot ini sengaja dirancang menyala dalam keadaan
// LEMAS (PWM mati) -- lihat bagian 3 README. Itu yang membuat reset dan
// unggah ulang tidak pernah menghentak 18 servo. Demo ini MEMBATALKAN
// perlindungan tersebut: robot akan berdiri sendiri beberapa detik sesudah
// menyala, termasuk sesudah setiap kali program diunggah.
//
// Karena itu ada DEMO_BOOT_TUNDA: jeda sebelum servo dihidupkan, supaya ada
// waktu menjauhkan tangan atau menekan tombol apa pun untuk membatalkan.
// JANGAN mengecilkannya di bawah 2 detik.
//
// Urutannya persis sama dengan mengetik: 'b' -> 'z10 1' -> tunggu -> '0'.
// Perintahnya benar-benar dilewatkan ke parser yang sama, bukan disalin --
// jadi demo ini tidak bisa menyimpang dari perilaku perintah manualnya.
#define DEMO_BOOT          0      // 0 = MATI (kembali ke boot lemas yang aman)
#define DEMO_BOOT_TUNDA    3000   // ms, jeda sebelum servo dihidupkan
#define DEMO_BOOT_BERDIRI  1500   // ms, tunggu kaki menetap sesudah berdiri
#define DEMO_BOOT_LAMA    20000   // ms, lama goyang berjalan
#define DEMO_BOOT_PERINTAH "z10 2"   // dijalankan apa adanya lewat handleCmd()
#define BODY_DEMO_TRANS_MM 25.0f  // amplitudo translasi saat demo 'B'
#define BODY_DEMO_PHASE_S   3.0f  // detik per sumbu (6 sumbu = 18 detik)

#define SERVO_PWM_FREQ    50      // Hz, frekuensi sinyal PCA9685 (50-330 Hz)
#define SERVO_COMMIT_MS   20      // ms, periode kirim 18 pulse (20=50Hz, 10=100Hz)

// Loop kontrol laju-tetap (dt deterministik untuk gait/PID/stabilisasi).
#define CONTROL_HZ        100     // Hz, tick loop utama (servo commit tetap di SERVO_COMMIT_MS)
#define PROFILE_LOOP      1       // 1 = cetak "PROF avg/max/util" tiap detik (saat tak tuning)
#define GAIT_DEBUG        0       // 1 = cetak fase gait tiap 200 ms (hanya saat melangkah)

#endif
