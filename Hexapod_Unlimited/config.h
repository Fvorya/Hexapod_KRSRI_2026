#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Papan salah di Arduino IDE = puluhan galat soal Wire2 yang menyesatkan.
// Ditangkap di sini supaya galatnya satu baris.
#if defined(ARDUINO) && !defined(__IMXRT1062__)
#error "Papan salah. Pilih Tools > Board > Teensy 4.1 -- papan lain tidak punya Wire2 maupun Serial.addMemoryForRead()."
#endif

#define NUM_SERVOS 18
#define NUM_TUNE_SERVOS 24
// Lengan tidak simetris: DEPAN 4 servo (bahu, siku, pergelangan, grip),
// BELAKANG 1 servo (grip saja, tanpa IK).
#define ARM_N_DEPAN    4
#define ARM_N_BELAKANG 1
#define ARM_NUM_SERVOS 4   // maksimum per lengan; ukuran array di HexaArm

// Slot kalibrasi per lengan di EEPROM. JANGAN diubah: ukurannya ikut
// menentukan ukuran CalibBlob, dan blob yang berubah ukuran dibuang.
#define ARM_SLOT_N     3

// Id servo di dalam satu lengan. Grip beda id di kedua lengan -- pakai
// Hexapod::gripId(), jangan angkanya langsung.
#define ARM_ID_BAHU          0
#define ARM_ID_SIKU          1
#define ARM_ID_PERGELANGAN   2
#define ARM_ID_GRIP_DEPAN    3
#define ARM_ID_GRIP_BELAKANG 0

// SERVO LENGAN: sendi DS3225 (500..2500 us, 180 der), grip MG90S
// (500..2400 us). Keduanya berbagi rentang arm.pulse.min/max di Calib; robot
// yang pernah 'W' dengan 1000..2000 harus disetel ulang ke 500..2500, kalau
// tidak tiap sudut sendi keluar setengahnya. Persen grip di bawah memotong
// ujung rentang supaya MG90S tidak ditekan ke stop internalnya.
// DS3225 juga dijual versi 270 der -- kalau itu yang terpasang, sudut IK
// jadi 1,5x terlalu besar.
//
// BASIS tiap sendi = sudut servo saat sudut geometrisnya 0. Bahu, pergelangan,
// grip bertanda (berayun di sekitar 0) -> 90. Siku sudut DALAM 0..180 -> 0.
// Siku maksimum = 180 - ARM_BASE_SIKU, jadi menaikkan basis siku memotong
// lipatan maksimum derajat per derajat. Siku berputar terbalik: pakai 180 dan
// invert slot 19, jangan 90.
#define ARM_BASE_BAHU         90.0f
#define ARM_BASE_SIKU          0.0f
#define ARM_BASE_PERGELANGAN  90.0f
#define ARM_BASE_GRIP         90.0f

#define GRIP_PERSEN_MIN   5.0f
#define GRIP_PERSEN_MAKS 95.0f

// Bukaan capit sekuens korban, persen. Korban 8,5 cm masih muat; bukaan lebih
// lebar menyapu reruntuhan dan tetangga 8 cm. Angka yang SAMA ada di Raspi
// (Kalib.capit_buka_persen di mission_hud.py) -- ubah keduanya bersama.
#define KORBAN_GRIP_BUKA  30.0f

// Tutupan capit sekuens korban, persen. Terpisah dari GRIP_PERSEN_MIN supaya
// perintah manual 'g5' tetap bisa dikerjakan walau sekuens memakai angka lain.
// Menutup tepat di pagar servo membuat rahang menekan mentok dan menarik arus
// penuh sepanjang korban digendong.
#define KORBAN_GRIP_TUTUP  5.0f

// Condong badan saat capit turun: angkanya di tabel kalibrasi (condong.mm,
// condong.jeda, condong.yaw), disetel dengan 'Q' lalu 'W'. Kalau kurang maju,
// naikkan condong.mm -- jangan menggeser KORBAN_JARAK_CM, itu memajukan KAKI
// dan lengan yang turun menabrak reruntuhan.

// Pose REHAT ('R'), sudut sendi geometris. Lengan terlipat di atas badan,
// bebas dari LiDAR depan dan anak tangga. Disetel di robot.
#define REHAT_BAHU         40.0f
#define REHAT_SIKU        100.0f
#define REHAT_PERGELANGAN -20.0f

// ====================================================================
// SEKUENS KORBAN: robot berhenti pada gerbang jarak, lalu lengan memainkan
// pose sendi tetap. Kamera meluruskan kiri-kanan sebelum ruas ini.
// ====================================================================
// 0 = ruas AMBIL/TARUH tetap berhenti dan memicu Raspi, tapi tidak satu servo
// lengan pun digerakkan.
#define LENGAN_KORBAN_AKTIF 1

// Gerbang HNT_DEPAN ruas AMBIL. Bilangan bulat: dipakai di kolom `nilai`
// tabel, dan cek_tabel_misi.py membacanya.
#define KORBAN_JARAK_CM      25

// Pose sekuens korban, SUDUT SENDI GEOMETRIS (satuan 'as'), dibidik di robot.
// Hard-coded karena datum bahu masih diragukan: pose yang terbukti tidak boleh
// bergeser diam-diam saat geometri disetel. SIAP dipakai dua kali (mendekat
// dengan capit terbuka, lalu menarik korban); korban digendong di REHAT.
#define KORBAN_SIAP_BAHU     -50.0f
#define KORBAN_SIAP_SIKU     120.0f
// Pergelangan menengadah membuat capit menabrak lantai saat siku turun ke
// JEPIT. Dekat rail: angleToPulse() meng-clamp diam-diam, jadi kalau
// pergelangan tidak sampai, ini tersangka pertama.
#define KORBAN_SIAP_PRG      -60.0f

#define KORBAN_JEPIT_BAHU    -50.0f
#define KORBAN_JEPIT_SIKU     15.0f
#define KORBAN_JEPIT_PRG     -10.0f

// TARUH cuma satu pose: dari gendong langsung ke titik lepas.
#define KORBAN_LEPAS_BAHU    -50.0f
#define KORBAN_LEPAS_SIKU     30.0f
#define KORBAN_LEPAS_PRG     -20.0f

// Jatah waktu tiap fase sekuens. BUTA: servo tidak memberi umpan balik
// posisi, jadi fase berganti murni dari jam. Sendi bergerak BERURUTAN (bahu,
// lalu siku+pergelangan), jadi lamanya jumlah, bukan maksimum. Langkah
// terberat (rehat -> siap, 180 der) = 1,70 detik dengan ARM_SLEW_DEG_S 150 dan
// ARM_ACCEL_DEG_S2 600 -- sisa 400 ms (19%). Jatah yang sama juga memayungi
// translasi condong (60 mm pada 40 mm/detik = 1,5 detik). Jangan dipotong
// tanpa menaikkan slew; cek_lengan_laju.py menjaga keduanya.
#define LENGAN_JEDA_MS      2100

// Laju maksimum sendi lengan, der/detik. Tanpa batas ini servo menyentak pada
// laju penuh DS3225 (~460 der/detik): melempar korban yang digenggam dan
// menggoyang badan di tangga. Menaikkannya TIDAK mempercepat misi -- fase
// tetap menunggu LENGAN_JEDA_MS. Di atas ~232 justru lebih lambat, karena
// waktu melecut (v/a) mendominasi.
#define ARM_SLEW_DEG_S      150.0f

// Batas percepatan lengan, der/detik^2. Membuat profil gerak trapesium
// alih-alih kotak (gerak 'as' dari REHAT "patah-patah"). Menambah v/a = 0,25
// detik per tahap. Gerak lebih pendek dari 37,5 der tidak pernah sampai laju
// penuh. Menurunkan ke 400 memotong sisa jatah jadi 150 ms.
#define ARM_ACCEL_DEG_S2    600.0f

// Laju lipat ke REHAT sambil menggenggam: lebih lambat supaya boneka tidak
// terlempar. 95 der butuh 1583 ms, jadi fase terakhir sekuens sengaja kosong
// dan lipatan mendapat dua jatah.
#define LENGAN_SLEW_REHAT_DEG_S  60.0f

// Laju mengangkat korban (fase 4 AMBIL). Gerakannya hampir seluruhnya siku dan
// menekan boneka ke dalam capit, jadi boleh lebih cepat daripada lipatan.
// Turunkan ke LENGAN_SLEW_REHAT_DEG_S kalau boneka terlepas saat diangkat.
#define LENGAN_SLEW_ANGKAT_DEG_S 90.0f

// Pose ANGKAT (fase 4 AMBIL), dibidik di robot. Bukan SIAP: kembali ke SIAP
// sambil menggenggam mengayunkan pergelangan 40 der tanpa perlu.
#define KORBAN_ANGKAT_BAHU   -50.0f
#define KORBAN_ANGKAT_SIKU    50.0f
#define KORBAN_ANGKAT_PRG    -10.0f

// ====================================================================
// DIMENSI KAKI -- TERBUKTI SALAH, BELUM DIPERBAIKI (diukur 14 Sep 2026).
//
// Terukur pada 'b100', lantai rata:
//   tinggi poros femur dari lantai   75 mm   (perintah 100)
//   mendatar sumbu coxa -> telapak   80 mm   (perintah  70)
//   coxa -> femur                    20 mm   (cocok)
//   femur -> lutut                   55 mm   (tertulis 80)
//   lutut -> telapak                ~65 mm   (tertulis 90)
//
// Dua kesalahan yang harus dibereskan BERSAMA: panjang link, dan datum sudut
// lutut yang meleset ~+24 der. Membetulkan salah satunya saja memindahkan
// robot ke tempat ketiga yang juga salah.
//
// Ukuran yang mengunci: pada 'b100', ketik 'd' (kolom sdeg c/f/t), lalu ukur
// dengan busur sudut femur dari mendatar (model -10,6) dan sudut dalam lutut
// (model 82,0; dugaan 106). Radius tapak yang melonjak di 'b130' berarti
// galatnya berskala -- curigai SERVO_PULSE_MIN/MAX atau servo 270 der.
//
// Akibat sekarang: tinggi badan nyata 75 mm (pose lengan beracuan lantai
// meleset 25 mm), dan ujung kaki tengah 170 mm dari pusat badan, bukan 160.
// ====================================================================
#define COXA_LENGTH  20.0f
#define FEMUR_LENGTH 80.0f
#define TIBIA_LENGTH 90.0f

// Lengan, poros ke poros, diukur 11 Sep 2026. Jangkauan 8..166 mm dari bahu.
#define UPPERARM_LENGTH 87.0f  // bahu  -> siku
#define FOREARM_LENGTH  79.0f  // siku  -> pergelangan

// Pergelangan -> poros grip. Rahang capit di luar poros belum diukur.
#define HAND_LENGTH    120.0f  // pergelangan -> poros grip

#define STAND_HEIGHT  100.0f // Tinggi badan dari tanah
#define STAND_RADIUS   70.0f // Jauh kaki ke pangkal coxa

// ====================================================================
// PROFIL KAIL / TANJAK (R-9)
//
// Offset kaki terhadap posisi netral, mm, frame badan (+y depan, +z atas).
// Kaki depan naik ke anak tangga (36 mm) sambil membuka ke depan untuk
// mengait; kaki belakang MEMANJANG ke bawah karena masih di tapak bawah.
// Selisih keduanya yang menjaga badan datar di bidang miring.
//
// Sudut yang dikompensasi = atan((DEPAN_NAIK + BELAKANG_TURUN) / 156), 156 mm
// = jarak pangkal kaki depan ke belakang. 40 + 42 -> 27,7 der; bidang R-9
// terukur 27,3 der. Jangan naikkan DEPAN_NAIK lebih jauh: badan menunduk dan
// kaki depan menekan muka anak tangga alih-alih mengait.
//
// Batasnya jangkauan IK, ditukar 1:1 dengan tinggi badan:
//     badan 115 -> belakang turun maks 30 mm (24,2 der)
//     badan 105 -> maks 40 mm (27,1 der)
//     badan 100 -> maks 45 mm (28,6 der)   <-- dipakai
//     badan  90 -> maks 55 mm (31,3 der)
// Karena itu KAIL tidak memakai tinggi badan TANGGA (115).
// ====================================================================

// Siklus gait KAIL, ms di atas gait.cycle_time. Badan maju 2 x stepLength
// per siklus: pada 900 ms dan langkah 60 mm, +300 -> 10,0 cm/detik. Kalau kaki
// depan menyangkut bibir anak tangga, yang kurang biasanya tinggi langkah,
// bukan siklus.
#define KAIL_CYCLE_TAMBAH_MS  300.0f

// Pose lengan depan selama profil TANJAK, sudut geometris (satuan 'as').
// Dipasang profileTanjak(), jadi ikut pada misi, 'U', dan 'T5'. Lengan
// terlipat supaya tidak menyapu muka anak tangga.
#define TANJAK_LENGAN_BAHU    0.0f
#define TANJAK_LENGAN_SIKU   90.0f
#define TANJAK_LENGAN_PRG    90.0f

// TIDAK DIPAKAI sejak profileNarrow() disetel ulang di arena; disimpan
// sebagai catatan percobaan bentuk TANJAK untuk R-11. Pada radius 60 tinggi
// badan 115 di luar jangkauan kaki nyata (hypot(60-20, 115) = 121,8 > 120 mm),
// jadi badan yang lebih tinggi butuh radius yang lebih rapat.
#define SEMPIT_PITCH_DEG      5.0f
#define SEMPIT_RADIUS_KAKI   45.0f
#define SEMPIT_TINGGI_BADAN 110.0f

// Pitch badan profil TANJAK, der (+ = mendongak). Memindahkan titik berat ke
// kaki yang mendaki dan mengangkat sudut bawah-depan sasis dari bibir anak
// tangga terakhir (~1,7 mm per derajat). 'b' menolkan rotasi badan -- ketik
// 'T5' lagi sesudahnya. Plafonnya BODY_MAX_ROT_DEG.
#define TANJAK_PITCH_DEG     10.0f

// PUNCAK TANJAKAN (HNT_PUNCAK), dibaca gyro. Diukur sebagai SELISIH terhadap
// pitch di awal ruas, bukan terhadap nol: tare() tidak pernah dipanggil
// (simpangan pemasangan IMU terbawa apa adanya), dan profil TANJAK sendiri
// memiringkan badan. Diukur di tangga, 18 Sep 2026:
//     kaki anak tangga (awal)   pitch -9,4   <- acuan
//     seluruh kaki di tangga    pitch 18,5   simpang 27,9
//     selesai menaiki (R-10)    pitch -4,1   simpang  5,3
// R-10 sendiri miring, karena itu ambang DATAR bukan nol. Jarak 5 der antara
// NAIK dan DATAR adalah histeresisnya.
#define PUNCAK_NAIK_DEG    15.0f   // |pitch-awal| di atas ini = MENDAKI
#define PUNCAK_DATAR_DEG   10.0f   // turun ke bawah ini = SUDAH DI ATAS
#define PUNCAK_DATAR_MS      400   // BELUM DIUKUR: harus bertahan selama ini

#define KAIL_TINGGI_BADAN   100.0f  // BUKAN 115 milik TANGGA -- lihat tabel
#define KAIL_DEPAN_MAJU      60.0f  // kaki depan membuka ke DEPAN
#define KAIL_DEPAN_NAIK      40.0f  // kaki depan NAIK ke tapak berikutnya
#define KAIL_BELAKANG_TURUN  42.0f  // kaki belakang MEMANJANG KE BAWAH
#define KAIL_RADIUS_KAKI     60.0f  // stance DIPERSEMPIT dari STAND_RADIUS 70

// Kaki TENGAH didorong keluar: memperkecil ayunan coxa dan melebarkan
// poligon tumpuan. Murah (kaki tengah cuma memakai 66% jangkauan), tapi tiap
// 10 mm melebarkan robot 20 mm -- batasnya lebar lorong, bukan IK.
//   keluar  0 -> ayun 53 der (jejak +-150 mm)
//   keluar 30 -> ayun 37 der (jejak +-180 mm)  <-- dipakai
//   keluar 40 -> ayun 33 der (jejak +-190 mm)
#define KAIL_TENGAH_KELUAR   30.0f

// Kaki BELAKANG membuka ke belakang. Mahal: dengan telapak menggantung 42 mm
// ia sudah hampir lurus, dan kaki lurus lemah ke arah radial -- arah ia
// mendorong badan naik. Terukur, langkah 60:
//   mundur  0 -> D 94,0%
//   mundur  5 -> D 95,4%  <-- dipakai
//   mundur 10 -> D 96,8%  tidak bersisa
//   mundur 30 -> MENTOK
#define KAIL_BELAKANG_MUNDUR  5.0f

// Kaki belakang dilebarkan dari garis lurusnya supaya jejak tidak menyatu.
// Ongkosnya coxa mengayuh lagi, dan margin di LERENG justru memburuk (sisi
// pengikat poligon berpindah ke garis telapak-belakang -> kaki-tengah-seberang):
//   lebar  0 -> jejak +-45, ayun  0,0 der, margin 63 / lereng 39 mm
//   lebar 15 -> jejak +-60, ayun 12,0 der, margin 66 / lereng 37 mm  <--
//   lebar 30 -> jejak +-75, ayun 20,2 der, margin 72 / lereng 34 mm
// Kalau di tangga selip lagi, INI yang pertama dikembalikan ke 0.
#define KAIL_BELAKANG_LEBAR  15.0f

// Sudut telapak kaki tengah, der dari arah hadap coxa (- = mundur). Radius
// tetap, jadi jangkauan IK sama. Dari pusat badan tidak berpengaruh; baru
// bicara saat CG bergeser. Margin guling (mm), lebar belakang 15:
//   CG        sudut -30  -20    0  +20  +30
//   +60 mm       41   48   59   69   69     korban di lengan depan
//     0 mm       67   67   67   67   67
//   -60 mm       55   52   43   34   28     mendongak di bidang 27,7 der
//   -90 mm       28   25   18   10    5
// R-9 menggeser CG ~73 mm ke belakang. Negatif menolong saat CG mundur;
// kalau robot terguling ke DEPAN di tangga, coba positif.
#define KAIL_TENGAH_SUDUT   0.0f

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
// DEPAN: bahu, siku, pergelangan, grip -- bahu dan siku menyapu satu bidang
// vertikal, tanpa sendi pemutar; untuk membidik objek, BADAN yang diarahkan.
// BELAKANG: cuma grip.
const uint8_t ARM_PIN_MAP_DEPAN[ARM_N_DEPAN][2] = {
    {0, 12},   // bahu
    {0, 13},   // siku
    {0, 14},   // pergelangan
    {0, 15}    // grip
};
const uint8_t ARM_PIN_MAP_BELAKANG[ARM_N_BELAKANG][2] = { {1, 12} };   // grip

// Pangkal BAHU (x, y, z) dari pusat badan, mm. Lengan depan menghadap +Y.
// Baris BELAKANG tidak dipakai IK (lengan itu tidak punya bahu).
#define ARM_DEPAN     0
#define ARM_BELAKANG  1
const float ARM_ORIGINS[2][3] = {
    { 0.0f,  85.0f, 45.0f},  // ARM_DEPAN    : DIUKUR 11 Sep 2026 -- 85 mm depan, 45 mm atas
    { 0.0f, -50.0f, 30.0f}   // ARM_BELAKANG : 50 mm di belakang pusat
};

// BUS I2C. Wire (SDA 18 / SCL 19) untuk LiDAR, Wire1/Wire2 untuk dua PCA9685.
// Kalau mux LiDAR harus pindah ke bus servo: alamat ALL-CALL PCA9685 juga
// 0x70, sama dengan TCA9548 -- matikan dulu ALL-CALL kedua PCA9685 (MODE1
// bit 0, sesudah setPWMFreq), atau pindahkan mux ke 0x71 lewat A0..A2.
#define LIDAR_I2C_BUS    Wire
#define SERVO_0_I2C_BUS  Wire1
#define SERVO_1_I2C_BUS  Wire2

// Pin mentah bus LiDAR, untuk pemulihan bus (SCL digoyang sebagai GPIO).
// WAJIB ikut berubah kalau LIDAR_I2C_BUS pindah: Wire 18/19, Wire1 17/16,
// Wire2 25/24 (SDA/SCL).
#define LIDAR_I2C_SDA    18
#define LIDAR_I2C_SCL    19

#define SERVO_I2C_CLOCK  400000
#define LIDAR_I2C_CLOCK  400000

// --- LiDAR (VL53L1X) --- //
#define NUM_LIDAR        6
#define I2C_MUX_ADDR     0x70
#define LIDAR_EMA_ALPHA  0.4f    // bobot sampel baru (median dulu, lalu EMA)
#define LIDAR_TIMEOUT_MS 300     // sensor dianggap mati bila tak ada data valid

// MODE JARAK. Di arena bercahaya Long justru paling pendek:
//      mode     gelap    cahaya terang    anggaran minimum
//      Short    136 cm       135 cm             20 ms
//      Medium   290 cm        76 cm             33 ms
//      Long     360 cm        73 cm             33 ms
// Navigasi tidak mengemudi dengan jarak di atas 50 cm, jadi Short cukup, dan
// anggarannya yang pendek mempercepat laju sampel (turunan PD lebih baik).
#define LIDAR_MODE_SHORT   0
#define LIDAR_MODE_MEDIUM  1
#define LIDAR_MODE_LONG    2
#define LIDAR_MODE       LIDAR_MODE_SHORT

#define LIDAR_BUDGET_US  20000   // anggaran waktu per pengukuran (us)
#define LIDAR_PERIOD_MS  25      // jeda antar pengukuran (ms), >= anggaran waktu
// Batas atas mode Short. Dinding 1 m terbaca sebagai angka, bukan "jauh".
// Medium/Long: naikkan ini (~290/~360) DAN LIDAR_BUDGET_US >= 33000 --
// static_assert di LidarArray.cpp menjaganya.
#define LIDAR_MAX_CM     130     // di atas ini dianggap "jauh", bukan rusak

// BATAS BAWAH PER ARAH. Di robot ini "tak ada objek" muncul sebagai bacaan
// HANTU ~5 cm, bukan SignalFail -- jadi bacaan di bawah batas ini artinya
// KOSONG, bukan halangan dekat. Urutan indeks = channel mux.
//
// Samping 4, bukan 10: dengan 10, bacaan tepat di bawahnya dipetakan ke
// LIDAR_JAUH ("dinding hilang"), dan ikut-dinding membelok MENUJU dinding
// untuk mencarinya -- makin dekat, makin keras merapat. Dengan 4, rentang
// dekat ditangani pita "terlalu dekat" yang mendorong menjauh.
//
// Depan dan belakang 7: di sana "jauh" berarti lorong kosong, dan tanpa
// ambang ini hantu 5 cm di depan membuat mode arena berbelok menghindari
// lorong yang terbuka lebar.
const uint8_t LIDAR_MIN_CM[6] = {
     4,  // ch0 kiri depan   (samping)
     4,  // ch1 kiri belakang(samping)
     7,  // ch2 belakang
     4,  // ch3 kanan blkg   (samping)
     4,  // ch4 kanan depan  (samping)
     7   // ch5 depan
};

// Bacaan LiDAR samping saat kaki tengah MENYENTUH dinding, DIUKUR di robot
// (14 Sep 2026, profil DATAR). Ujung ramp pita "terlalu dekat": dorongan
// menjauh separuh di wall.min, PENUH di sini. Profil yang kaki tengahnya
// lebih keluar (KAIL/TANJAK) menyentuh pada bacaan yang lebih besar.
#define WALL_KAKI_CM      7.0f

// ROI 4x4: bidang pandang ~27 -> ~15 der. Menghapus hantu menetap di sensor
// depan ('j5 10' -> 100% signal fail) -- hantunya tertangkap di pinggir
// kerucut, bukan di sumbu. Harganya dinding samping lebih mudah luput saat
// badan menyerong. Kalau hantu muncul lagi, JANGAN naikkan LIDAR_MIN_CM;
// mulai dari 'u5 4' untuk memisahkan crosstalk dari benda nyata.
#define LIDAR_ROI_SEMPIT 1       // 1 = setROISize(4,4)

// Tiga keadaan getDistance(): angka, jauh, atau mati. Ikut-dinding butuh
// reaksi berlawanan untuk "tak ada objek" dan "sensor putus".
#define LIDAR_JAUH       999     // sensor sehat, tak ada objek dalam jangkauan
#define LIDAR_MATI       (-1)    // sensor tidak merespons / belum ada data

// ARAH FISIK -> CHANNEL MUX. Urutan kabel TERBALIK dari kode lama (ch n =
// indeks lama 5-n). Arah ch0 dan ch2 belum diverifikasi langsung -- periksa
// dengan 'l'.
//   ch0 = KIRI DEPAN     ch1 = KIRI BELAKANG    ch2 = BELAKANG
//   ch3 = KANAN BELAKANG ch4 = KANAN DEPAN      ch5 = DEPAN
// Keempat sensor samping menghadap TEGAK LURUS dinding; "depan/belakang"
// pada namanya cuma letak dudukan. _D = paruh depan badan, _B = paruh belakang.
#define LIDAR_FRONT      5   // satu-satunya yang menghadap DEPAN
// Jarak membujur dudukan sensor depan-belakang di sisi yang sama, lensa ke
// lensa. Sudut dinding = atan((d_belakang - d_depan) / jarak ini). Di bawah
// ~8 cm sudutnya tenggelam di derau. Ganti bila dudukan dipindah.
#define WALL_BASE_CM     11.0f

// Selisih terbesar (belakang - depan) yang masih dianggap dinding; lebih dari
// ini berkasnya menembak celah. Memagari sudut terbaca maksimum di
// atan(4/11) = 20 der -- ambang koreksi harus di bawahnya (cek_koreksi.cpp).
#define SISI_BEDA_MAKS_CM  4.0f

// Simpangan pemasangan pasangan sensor (belakang - depan) saat badan sejajar,
// diukur dengan 'Y0'. Tanpa ini robot mengira dirinya serong 10 der. 'Y0'
// menimpanya kapan pun.
#define WALL_BIAS_KIRI_CM   2.0f
#define WALL_BIAS_KANAN_CM  2.0f

#define LIDAR_KANAN_D    4   // menghadap kanan, dudukan depan
#define LIDAR_KANAN_B    3   // menghadap kanan, dudukan belakang
#define LIDAR_BACK       2   // satu-satunya yang menghadap BELAKANG
#define LIDAR_KIRI_B     1   // menghadap kiri, dudukan belakang
#define LIDAR_KIRI_D     0   // menghadap kiri, dudukan depan

// --- IMU --- //
#define IMU_MAX_YAW_JUMP 30.0f   // derajat/sample; lonjakan > ini ditolak (gangguan magnet)

// Pencatatan kompas ('c0'..'c3') merata-rata sebentar dan MENOLAK heading yang
// belum tenang, atau yang jaraknya ke mata angin lain jauh dari 90 der.
#define KOMPAS_SAMPEL_MS      400    // lama merata-rata satu pencatatan
#define KOMPAS_SEBAR_MAKS_DER   3.0f // sebaran maks dalam jendela itu
#define KOMPAS_GAP_TOL_DER     10.0f // toleransi jarak antar mata angin dari 90
// Penolakan berturut-turut sebelum yaw baru diterima paksa. Tanpa jalan
// keluar ini, pergeseran heading yang menetap membekukan yaw selamanya.
#define IMU_MAX_YAW_TOLAK  10
// Batas byte serial IMU per update(), supaya loop utama tetap kebagian giliran.
#define IMU_MAX_BYTE_UPDATE 512
#define IMU_SERIAL       Serial2
#define IMU_BAUD         230400   // Yahboom 10-axis (protokol WIT, frame 0x55)

// --- DETEKSI TERGULING ---
// accelZ dipercaya lebih dulu daripada roll (roll memakai magnetometer, mudah
// dibohongi besi arena). Saat terguling |accelZ| jatuh ke ~0.
//
// Di robot ini IMU terpasang sehingga DATAR terbaca roll +-180 der, jadi yang
// diukur JARAK ke tegak: min(|roll|, 180-|roll|). Tanda accelZ membedakan
// tegak dari terbalik (keduanya |az| ~ 1 g). Kalau IMU dipasang ulang, ukur
// tanda accelZ lagi lewat 'd'.
//
// Roll 45, bukan 30: TANJAK bekerja normal di 27,7 der.
#define TERGULING_AKTIF      1        // 0 = MATI. Baku HIDUP.
#define TERGULING_AZ_TEGAK  -1.0f     // DIUKUR: tanda accelZ saat berdiri
#define TERGULING_AZ_G       0.5f     // BELUM DIUKUR: accelZ tegak di bawah ini
#define TERGULING_ROLL_DEG  45.0f     // BELUM DIUKUR: simpangan roll di atas ini
#define TERGULING_TUNDA_MS   400      // BELUM DIUKUR: harus bertahan selama ini

// KOMPENSASI MAJU SAAT PERATAAN ('V' dan ruas HNT_SISI/HNT_GESER).
// Geser menyamping menyeret badan ke depan tiap siklus; LiDAR belakang melihat
// bacaannya NAIK. Robot ditarik mundur hanya saat bacaan DI ATAS sasaran --
// satu arah, jadi tidak pernah didorong maju ke reruntuhan. Sasaran 10 cm
// menyisakan 3 cm di atas LIDAR_MIN_CM[belakang] sebelum penggarisnya diam.
// Toleransi 0: derau 1-2 cm memberi bias mundur lemah selama perataan. Kalau
// robot terlihat merayap mundur saat 'V', naikkan TOL, bukan turunkan MUNDUR.
#define RATA_BLK_SASARAN_CM  10
#define RATA_BLK_TOL_CM      0
#define RATA_BLK_MUNDUR    0.20f

// Lantai jarak ruas MUNDUR. Bukan wall.min: wall.min menjaga ikut-dinding di
// SAMPING, sedangkan mundur menuju dinding dengan sengaja dan pelan.
// BELUM DIUKUR untuk kaki BELAKANG -- angkanya diambil dari kaki tengah.
// Ukur: dorong robot mundur sampai telapak belakang menyentuh tembok, baca
// LiDAR belakang.
#define MUNDUR_MIN_CM      WALL_KAKI_CM

// Berapa kali misi mencoba menghidupkan ulang LiDAR yang putus (sama dengan
// 'I'), dihitung PER MISI. Tiap percobaan menahan lup utama sampai 1,6 detik
// sementara jam kontes jalan terus; sensor yang tidak pulih sekali pindai
// hampir tidak pernah pulih pada percobaan kedua.
#define LIDAR_ULANG_MAKS      1

// Batas waktu 'J' (setel jarak belakang) dan ruas MUNDUR.
#define MUNDUR_BATAS_MS    12000

// Batas parkir menunggu Raspi, ms. VISI: menunggu 'm2' sesudah '#KORBAN
// AMBIL'; habis waktu sekuens tetap jalan dengan sudut tetap. LEPAS: menunggu
// 'm9' sesudah capit selesai. Tanpa Raspi ketik 'm8 0' supaya tidak menunggu.
#define MISI_VISI_BATAS_MS  20000
#define MISI_LEPAS_BATAS_MS  5000

// --- Raspi 5: pemicu deteksi korban --- //
// Teensy memberi tahu Raspi KAPAN robot berhenti di depan korban:
//   #KORBAN AMBIL 8
//   #KORBAN TARUH 10
// (angka = nomor ruas). Serial = kabel USB; ganti ke Serial1 (pin 0/1) kalau
// Raspi disambung lewat UART GPIO.
//
// Raspi hanya boleh menjawab 'm2', 'm3', 'm9', dan hanya saat firmware
// menunggunya. Byte lain masuk parser penuh -- 'W' menulis kalibrasi, 'm0'
// membatalkan misi. Selama parkir vision, kendali kaki milik Raspi ('r', 'O').
#define KORBAN_SERIAL    Serial
#define KORBAN_BAUD      115200

// --- Navigasi --- //
#define HEADING_TOLERANCE_DEG   6.0f     // Toleransi heading dianggap "lurus" (const)
// Berhenti/belok bila depan di bawah ini. Ambang HNT_DEPAN tabel wajib DI ATAS
// angka ini (tabelSiap() menolak), karena navigasi berhenti sendiri lebih dulu.
// Jangan turunkan tanpa menurunkan LIDAR_MIN_CM[depan] (7): di bawahnya bacaan
// berarti "kosong", jadi pita halangan yang masih terlihat cuma 7..8 cm.
#define FRONT_STOP_CM      8       // Berhenti/belok bila depan < ini (const)
#define NAV_FWD_SPEED     0.8f     // Kecepatan maju normal (0..1) (const)

// Faktor slip odometri, diukur di lantai arena 2 Sep 2026: TIDAK ada slip
// (meteran 51,5 / 52,0 vs odometer 51,4 -> 1,00). Kalau mengukur ulang, pakai
// SATU titik badan yang sama untuk tanda awal dan akhir -- itu satu-satunya
// sumber galat yang pernah muncul, sekelas dengan slip yang dicari.
#define ODO_SKALA_DEF     1.0f     // tanpa koreksi; 'Ds' menimpanya di RAM (0,5..1,5)

// --- Wall-following & penghindaran halangan (non-blokir) --- //
#define NAV_PELAN_CM       50    // mulai melambat bila halangan depan di bawah ini
#define NAV_MAJU_MIN       0.15f // faktor kecepatan terendah saat mendekati halangan
#define NAV_BELOK_CMD      0.60f // kekuatan putar saat menghindar halangan depan
#define NAV_CARI_CMD       0.35f // kekuatan putar saat dinding samping hilang (tikungan)
#define NAV_BELOK_BATAS_MS 8000  // berbelok lebih lama dari ini = dianggap terjebak
// Dinding samping hilang lebih lama dari ini -> berhenti, jangan berputar
// selamanya. Hanya mode 'f'/'F' polos; di mode arena heading yang mengunci.
#define NAV_CARI_BATAS_MS 10000

// Mode arena: heading arena = acuan SUDUT, dinding = koreksi LATERAL.
#define NAV_WALL_TURN_MAX  0.50f // batas sumbangan kemudi dari dinding

// KOREKSI SAMBIL BERHENTI, hanya lewat 'U' (R-9). Di tanjakan, kaki ayun
// mendarat di tempat yang salah kalau badan berputar sambil melangkah, jadi
// robot berhenti dulu, memutar sampai sejajar, baru jalan. Sudut dari
// sudutDinding(), sah sampai +-20 der.
#define NAV_KOREKSI_SUDUT_DEG    6.0f   // masuk koreksi bila |sudut| lewat ini
#define NAV_KOREKSI_JARAK_CM     4.0f   // atau bila jarak meleset sejauh ini
// Keluar lebih ketat daripada masuk: histeresis, supaya tidak berhenti-jalan
// tepat di ambang.
#define NAV_KOREKSI_KELUAR_DEG   2.0f   // keluar bila sisa sudut di bawah ini
// Galat JARAK ditutup dengan membidik serong kecil lalu berjalan.
#define NAV_KOREKSI_JARAK_K      0.5f   // der sudut bidik per cm galat jarak
#define NAV_KOREKSI_BIDIK_MAKS   8.0f   // batas sudut bidik
// Menyerah lalu jalan lagi -- diam selamanya di tangga lebih buruk.
#define NAV_KOREKSI_BATAS_MS     3000

// MENENGAH: kemiringan di lorong (selisih kedua sisi / 2) untuk dorongan
// menjauh penuh. Dorongan proporsional, bukan bang-bang terhadap "dinding
// mana yang lebih dekat" -- di tengah lorong derau membalik jawaban itu dan
// kemudi berbalik 7x per 10 sampel.
#define NAV_TENGAH_PITA_CM 3.0f

// Gain pivot ada di Calib (heading.kp/kd), satu knob dengan mode arena.
#define PIVOT_MIN_CMD   0.25f    // Minimal gaya agar tidak cuma "menggeliat"
#define PIVOT_DIAM_MS   500      // Harus di dalam toleransi selama ms ini
#define PIVOT_SETTLE_MS 800      // Sesudah sampai: perintah putar 0, tunggu kaki diam
#define PIVOT_BATAS_MS  20000    // Batas waktu timeout (20 detik)

// --- Peta EEPROM --- //
// Alamat DIPATOK: blok-blok ditulis program berbeda, alamat inilah kontraknya.
// Struct dan static_assert-nya di EEMap.h.
//    0 : CalibBlob   firmware -- param, offset, trim, invert
// 1024 : ServoMap    invert & trim hasil uji fisik (menang atas Calib)
// 1792 : KompasStore 4 arah arena
// 2048 : GerakStore  pivot, odometri, rata badan, zOff
#define EE_CALIB_ADDR      0     // Blok kalibrasi firmware (Calib.cpp)
#define EE_SERVOMAP_ADDR 1024    // Peta servo hasil kalibrasi fisik
#define EE_KOMPAS_ADDR   1792    // Alamat memori untuk data kompas arena
#define EE_GERAK_ADDR    2048    // Kalibrasi gerak dari TES_GERAK
#define EE_PROFIL_ADDR   2304    // Enam profil medan, terpisah dari kalibrasi lama

// Kapasitas EEPROM Teensy 4.x. eeMapPeriksa() mencocokkannya saat boot.
#define EE_TOTAL_BYTES  4284

// --- Body kinematics (pose badan manual & demo uji) --- //
// Pagar perintah, bukan batas mekanis kaki.
#define BODY_MAX_ROT_DEG   20.0f  // clamp roll/pitch/yaw perintah manual
// Pada badan 100 / radius 70: geser maju 80 mm menuntut 94% jangkauan kaki
// belakang; geser menyamping 80 mm 96% kaki tengah. Kalau kaki tengah mentok
// saat Raspi menengahkan badan ('t<x> 0 0'), lihat ke sini.
#define BODY_MAX_TRANS_MM  80.0f  // clamp geser badan X/Y/Z

// Laju ramp pose badan. Tanpa ini 'r20 0 0' melompatkan femur ~49 der dalam
// satu commit 20 ms dan robot menyentak.
#define BODY_SLEW_DEG_S    60.0f  // laju maks rotasi badan, derajat/detik
#define BODY_SLEW_MM_S    120.0f  // laju maks geser badan, mm/detik

// Laju condong, mm/detik. Lambat supaya badan tidak goyang tepat di atas
// korban. condong.mm maks 79 / 40 = 1975 ms, muat di satu LENGAN_JEDA_MS.
#define KORBAN_CONDONG_LAJU_MM_S  40.0f

// Badan ditarik mundur sejauh ini sebelum lengan melipat korban ke REHAT,
// supaya busur boneka tidak membentur dinding yang dihadapi. Kaki diam.
#define KORBAN_ANGKAT_MUNDUR_MM   30.0f
#define BODY_DEMO_ROT_DEG  10.0f  // amplitudo rotasi saat demo 'B'

// --- DEMO OTOMATIS SAAT MENYALA (pajangan) ---
// PERINGATAN: membatalkan boot LEMAS yang aman -- robot berdiri sendiri
// sesudah menyala dan sesudah tiap upload. DEMO_BOOT_TUNDA memberi waktu
// menjauhkan tangan; jangan di bawah 2 detik.
#define DEMO_BOOT          0      // 0 = MATI (kembali ke boot lemas yang aman)
#define DEMO_BOOT_TUNDA    3000   // ms, jeda sebelum servo dihidupkan
#define DEMO_BOOT_BERDIRI  1500   // ms, tunggu kaki menetap sesudah berdiri
#define DEMO_BOOT_LAMA    20000   // ms, lama goyang berjalan
#define DEMO_BOOT_PERINTAH "B"    // dijalankan apa adanya lewat handleCmd()
#define BODY_DEMO_TRANS_MM 25.0f  // amplitudo translasi saat demo 'B'
#define BODY_DEMO_PHASE_S   3.0f  // detik per sumbu (6 sumbu = 18 detik)

#define SERVO_PWM_FREQ    50      // Hz, frekuensi sinyal PCA9685 (50-330 Hz)
#define SERVO_COMMIT_MS   20      // ms, periode kirim 18 pulse (20=50Hz, 10=100Hz)

// Acuan utilisasi baris PROF, BUKAN pembatas laju: loop() berjalan bebas dan
// seluruh kendali berbasis dt sungguhan.
#define CONTROL_HZ        100     // Hz, acuan utilisasi di baris PROF (BUKAN pembatas laju)
// Mati: baris PROF tiap detik mendorong bukti percobaan keluar dari penyangga
// log HUD. Nyalakan hanya saat mengukur waktu loop.
#define PROFILE_LOOP      0       // 1 = cetak "PROF avg/max/util" tiap detik (saat tak tuning)
#define GAIT_DEBUG        0       // 1 = cetak fase gait tiap 200 ms (hanya saat melangkah)

#endif