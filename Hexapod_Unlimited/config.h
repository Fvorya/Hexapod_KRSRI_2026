#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Papan salah di Arduino IDE = puluhan baris galat soal 'Wire2' dan
// 'addMemoryForRead' -- keduanya milik inti Teensy 4, dan sebabnya terkubur
// di tengah tumpukan. Ditangkap di sini supaya galatnya satu baris dan jelas.
// Build PC (cek_*.cpp) tidak mendefinisikan ARDUINO, jadi ia tidak kena.
#if defined(ARDUINO) && !defined(__IMXRT1062__)
#error "Papan salah. Pilih Tools > Board > Teensy 4.1 -- papan lain tidak punya Wire2 maupun Serial.addMemoryForRead()."
#endif

#define NUM_SERVOS 18 // 18
#define NUM_TUNE_SERVOS 24 // 24
// Lengan TIDAK simetris (dikonfirmasi operator 7 Sep 2026):
//   DEPAN    4 servo -- bahu, siku, pergelangan, grip
//   BELAKANG 1 servo -- grip saja; tidak ada sendi, jadi tidak ada IK
#define ARM_N_DEPAN    4
#define ARM_N_BELAKANG 1
#define ARM_NUM_SERVOS 4   // maksimum per lengan; ukuran array di HexaArm

// Slot kalibrasi per lengan yang SUDAH dialokasikan di EEPROM. JANGAN diubah:
// TOTAL_SERVOS (Calib.h) menentukan ukuran CalibBlob, dan ukurannya berubah
// membuang blob yang tersimpan -- gain dinding ikut hilang -- walau
// CALIB_VERSION tidak dinaikkan. 4 + 1 = 5 slot terpakai dari 6 yang ada.
#define ARM_SLOT_N     3

// Id servo DI DALAM satu lengan. Grip TIDAK berada di id yang sama pada kedua
// lengan, jadi jangan menulis angkanya langsung -- pakai Hexapod::gripId().
#define ARM_ID_BAHU          0
#define ARM_ID_SIKU          1
#define ARM_ID_PERGELANGAN   2
#define ARM_ID_GRIP_DEPAN    3
#define ARM_ID_GRIP_BELAKANG 0

// MODEL SERVO LENGAN (dikonfirmasi operator 11 Sep 2026):
//   sendi (bahu, siku, pergelangan) = DS3225   -- 500..2500 us, 180 der
//   grip depan & belakang           = MG90S    -- 500..2400 us, ~180 der
//
// Keduanya berbagi SATU rentang, arm.pulse.min/max (Calib, default 1000..2000).
// Dua akibatnya:
//
// 1. angleToPulse() memetakan 0..180 der geometris ke SELURUH rentang itu.
//    Pada 1000..2000 us, DS3225 cuma menempuh 90 der -- jadi setiap sudut
//    hasil IK keluar SETENGAH dari yang diminta, diam-diam. Karena itu
//    defaultnya dinaikkan ke 500..2500 (Calib.cpp, 11 Sep 2026).
//
//    TAPI DEFAULT SAJA TIDAK SAMPAI KE ROBOT. Calib::load() memulihkan blob
//    EEPROM 0 yang sudah tersimpan, jadi robot yang pernah 'W' tetap memakai
//    1000/2000 sampai disuruh: 'Qarm.pulse.min 500', 'Qarm.pulse.max 2500',
//    lalu 'W'. Periksa lewat baris "lengan pulse (us)" di 'd'.
//
// 2. Sesudah dilebarkan, ujung atas rentang itu di LUAR jangkauan MG90S.
//    setGrip(100) akan menyuruhnya ke 2500 us: servo menekan stop internal
//    dan macet menahan arus sampai gearnya habis. Persen di bawah memotong
//    kedua ujungnya -- pada 500..2500 us hasilnya 600..2400 us, pas di dalam
//    MG90S dan tetap lewat jalur offset/trim/invert per slot seperti biasa.
//
// Ukur ulang saat capit terpasang. Naikkan MAKS pelan-pelan sampai jepitan
// cukup membuka; JANGAN sampai servo mendengung saat diam -- itu bunyi macet.
//
// PERIKSA DULU VARIANNYA: DS3225 dijual dalam versi 180 der DAN 270 der.
// Kalau yang terpasang 270 der, anggapan 180 der di angleToPulse() bohong
// dan tiap sudut IK jadi 1,5x terlalu besar.
// BASIS (baseline) tiap sendi lengan: sudut servo saat sudut geometrisnya 0.
// Bukan hiasan -- inilah yang menentukan bagian mana dari 0..180 der servo
// yang benar-benar terpakai, dan salah pilih membuat clampf() di
// angleToPulse() memakan separuh jangkauan tanpa bilang-bilang.
//
//   bahu/pergelangan/grip : sudutnya BERTANDA, berayun di sekitar 0 -> 90
//   siku                  : sudut DALAM dari acos, selalu 0..180   -> 0
//
// Kalau siku berputar terbalik: pakai 180 DI SINI dan invert slot 19,
// jangan kembali ke 90. Lihat cek_lengan.cpp.
//
// ARM_BASE_SIKU ITU KENOP, BUKAN PILIHAN DUA ARAH. Karena servoAngle
// di-clamp ke 0..180:
//
//     siku maksimum        = 180 - ARM_BASE_SIKU
//     sudut siku saat 1500 =  90 - ARM_BASE_SIKU   (pose pemasangan horn)
//
//   0  -> horn dipasang pada siku 90 (bentuk L), siku terpakai 0..180
//  45  -> horn pada siku 45,                     siku terpakai 0..135
//  90  -> horn pada siku 0 (LURUS),              siku terpakai 0..90
//
// Tiap derajat yang ditambahkan ke baseline dibayar satu derajat lipatan
// maksimum. Yang memilihkan angkanya adalah lipatan terbesar yang dipakai
// misi, bukan selera: pose menggendong korban butuh siku ~130 der, jadi
// baseline di atas 50 membuat korban tak bisa ditarik masuk sama sekali.
// Ukur ARM_ORIGINS dulu -- semua angka itu relatif terhadapnya.

#define ARM_BASE_BAHU         90.0f
#define ARM_BASE_SIKU          0.0f
#define ARM_BASE_PERGELANGAN  90.0f
#define ARM_BASE_GRIP         90.0f

#define GRIP_PERSEN_MIN   5.0f
#define GRIP_PERSEN_MAKS 95.0f

// LEBAR BUKA CAPIT sekuens korban, persen. BUKAN GRIP_PERSEN_MAKS.
//
// 20, turun dari 50 lalu dari 95 (R2C 15 Sep 2026): "dengan g50, bukaan
// terlalu lebar sehingga dummy ataupun reruntuhan bisa diambil capit."
// Korban lebarnya 8,5 cm dan itu masih muat di 20; yang hilang cuma sapuan
// rahang yang tidak pernah dibutuhkan. Di ruang korban, tetangga terdekat
// 8 cm -- rahang yang terbuka lebih lebar dari itu menyenggolnya lebih dulu.
//
// Angka yang SAMA ada di sisi Raspi (Kalib.capit_buka_persen di
// mission_hud.py). Kalau yang ini diubah, ubah di sana juga -- keduanya
// menggerakkan satu capit.
#define KORBAN_GRIP_BUKA  20.0f

// BADAN CONDONG KE DEPAN saat capit turun, mm. Hanya di ruas yang ditandai
// `condong` di tabel (K-3 dan K-4).
//
// Masalah yang ditutupnya, laporan R2C 15 Sep 2026: saat capit sudah turun,
// robot kurang maju. Tapi memajukan robotnya SENDIRI tidak bisa -- pada jarak
// yang lebih dekat, lengan yang turun dari REHAT ke pose JEPIT sudah menabrak
// reruntuhan yang menutupi korban dalam perjalanannya.
//
// Yang dipindah BADANnya, bukan kakinya: kaki tetap di tempat yang aman, dan
// badan digeser ke depan di atasnya tepat sebelum lengan turun. Jalur turun
// lengan ikut bergeser maju bersama badan, jadi ia melewati reruntuhan di
// tempat yang sama seperti sebelumnya -- yang berubah cuma di mana capit
// berakhir.
//
// ANGKANYA PINDAH KE TABEL KALIBRASI (Calib.h: condong.mm, condong.jeda,
// condong.yaw). Disetel di arena dengan 'Qcondong.mm 30' lalu 'W', tanpa
// flash ulang. Kalau masih kurang maju, naikkan condong.mm -- jangan
// menggeser gerbang KORBAN_JARAK_CM, karena itu memajukan KAKI dan
// mengembalikan tabrakan lengan ke reruntuhan.

// Pose REHAT (perintah 'R'): lengan dilipat ke ATAS badan supaya tidak
// menghalangi LiDAR depan dan tidak tersangkut saat menaiki tangga.
// DISETEL DI ROBOT, bukan dihitung -- yang dipercaya bentuk nyatanya, bukan
// angka yang enak dilihat.
//
// SUDUT SENDI sejak 15 Sep 2026, sama seperti pose korban. Sebelumnya ia
// titik capit lewat IK ('a80 160 -40'), dan titik itu ikut bergeser tiap
// kali ARM_ORIGINS atau panjang link disetel -- padahal yang harus dihindari
// lengan adalah LiDAR dan kabel yang nyata, yang tidak ikut bergeser.
#define REHAT_BAHU         40.0f
#define REHAT_SIKU        100.0f
#define REHAT_PERGELANGAN -20.0f

// ====================================================================
// SEKUENS KORBAN -- TAHAP 1: GERBANG JARAK + POSE TETAP
//
// Tidak ada IK dari sensor di sini. Robot maju sampai LiDAR depan membaca
// KORBAN_JARAK_CM, berhenti, lalu lengan memainkan pose yang sudah disetel
// sekali. Satu angka dari sensor, sisanya tetap. Kamera mengurus pelurusan
// KIRI-KANAN sebelum ruas ini -- lengan tidak punya sendi untuk itu.
//
// TINGGI DIUKUR DARI LANTAI, bukan dari pusat badan, dan diubah ke kerangka
// badan saat dipakai memakai tinggi badan yang SEDANG BERLAKU
// (gaitProfile().standHeight). Tanpa itu ruas korban berprofil TANGGA
// (badan 115) akan meleset 15 mm terhadap yang berprofil DATAR (badan 100) --
// dua ruas korban memang memakai TANGGA.
//
// Jarak DIUKUR SAMPAI CAPIT, bukan sampai pergelangan: moveArmGrip() yang
// mengurus HAND_LENGTH dan menjaga tapak mendatar.
//
// Seluruh amplopnya disapu cek_korban.cpp. Pada tinggi jepit 40 mm, gerbang
// yang sah cuma 22,0..27,1 cm -- sempit, dan 24 ada di tengahnya. Di bawah
// 22 cm bahu menuntut sudut di bawah 0 der servo dan ter-clamp DIAM-DIAM.
// ====================================================================
// SAKLAR: lengan belum dikalibrasi (capit melendut ~70 mm saat terjulur
// mendatar, datum bahu masih diragukan), jadi ruas AMBIL/TARUH sementara cuma
// DILEWATI -- robot tetap berhenti di depan korban dan tetap memicu Raspi,
// tapi tidak satu servo lengan pun digerakkan. Sekuens di Misi.cpp utuh;
// balikkan ke 1 sesudah lendutan dan tinggi jepit diukur di robot.
#define LENGAN_KORBAN_AKTIF 1

#define LIDAR_DEPAN_MM       62.0f   // dudukan sensor depan, mm di depan pusat badan

// Gerbang HNT_DEPAN di ruas AMBIL. BILANGAN BULAT: ia dipakai apa adanya di
// kolom `nilai` tabel RUAS[], dan cek_tabel_misi.py membaca kolom itu.
#define KORBAN_JARAK_CM      25

#define KORBAN_TINGGI_MM     40.0f   // tinggi titik jepit DARI LANTAI
#define KORBAN_DEKAT_MM      40.0f   // mendekat/menjauh sejauh ini DI ATAS titik jepit
#define KORBAN_TARUH_MM      40.0f   // tinggi LEPAS dari lantai (SZ setinggi lantai)

// Sudut TAPAK saat menjepit & melepas. 0 = mendatar ke depan, negatif =
// menunduk. Bisa disetel karena pergelangan sekarang ikut IK (moveArmGrip).
//
// DIBIARKAN 0 karena bentuk capit dan boneka belum diukur -- yang menentukan
// sudut cengkeraman terbaik itu keduanya, bukan kinematika. Tapi kinematika
// punya pendapat yang jelas soal harganya, dan ini terukur: jendela gerbang
// yang sah pada profil TANGGA melebar dari 3,0 cm (tapak 0) jadi 7,5 cm
// (tapak -30). Menunduk memindahkan pergelangan menjauh dari lantai untuk
// titik capit yang sama, dan itu melawan batas yang mengikat -- bahu tidak
// boleh turun di bawah 0 der servo. Sapuannya dicetak cek_korban.cpp.
#define KORBAN_TAPAK_DER      0.0f

// Pose sekuens korban, SUDUT SENDI GEOMETRIS -- bukan IK, bukan titik capit.
// Dibidik dengan tangan di robot lalu dituliskan keras, satuan yang sama
// dengan perintah 'as<bahu> <siku> <pergelangan>'.
//
// Dipakai hard-coded justru KARENA amplop IK di sini sempit dan datum bahu
// masih diragukan: pose yang sudah terbukti di robot tidak boleh berubah
// diam-diam saat KORBAN_CAPIT_MM atau tinggi profil disetel. Gerbang
// KORBAN_JARAK_CM tetap yang menentukan robot berhenti di mana; sesudah
// berhenti, lengan cuma memainkan tiga pose ini.
//
// SIAP dipakai dua kali dalam sekuens AMBIL: mendekat dari atas dengan capit
// terbuka, lalu menarik korban yang sudah terjepit. Ia BUKAN pose menggendong
// -- kedua sekuens berakhir di REHAT, dan di sanalah korban dibawa berjalan.
// SIAP ada di antaranya supaya lipatan tidak menyapu capit melintasi tempat
// korban berdiri.
#define KORBAN_SIAP_BAHU     -50.0f
#define KORBAN_SIAP_SIKU     120.0f
// -90 sejak 16 Sep 2026, diukur R2C di robot: pada +90 capit menabrak tanah
// saat siku turun dari SIAP ke JEPIT. Pergelangan yang menengadah memutar
// capit ke bawah sepanjang busur turunnya siku, dan lantailah yang duluan
// ditemuinya.
//
// MASIH DI RAIL, cuma rail yang sebelah: servoAngle = ARM_BASE_PERGELANGAN
// + offset + geo, jadi -90 memberi servo 0 persis (atau 180 kalau kanal ini
// di-invert). angleToPulse() meng-clamp tanpa penanda apa pun, jadi trim
// sekecil apa pun ke arah luar hilang diam-diam. Kalau pergelangan terlihat
// tidak sampai, ini tersangka pertamanya -- pakai -85 untuk menjauh 5 der.
#define KORBAN_SIAP_PRG      -60.0f
                                      // positif apa pun membuatnya ter-clamp

#define KORBAN_JEPIT_BAHU    -40.0f
#define KORBAN_JEPIT_SIKU     10.0f
#define KORBAN_JEPIT_PRG     -10.0f

// TARUH cuma satu pose: dari gendong langsung ke titik lepas.
#define KORBAN_LEPAS_BAHU    -50.0f
#define KORBAN_LEPAS_SIKU     40.0f
#define KORBAN_LEPAS_PRG     -20.0f

// Pose menggendong, diukur DARI LANTAI dan ke CAPIT. 280 mm itu JAUH DI DEPAN
// badan -- ujung kaki depan cuma 139 mm -- jadi korban menjulur ~14 cm di
// depan robot dan bisa menyenggol dinding saat pivot di lorong sempit.
// Inilah angka pertama yang harus disetel ulang dengan boneka di tangan;
// cek_korban.cpp menjaga supaya penyetelannya tidak diam-diam ter-clamp.
#define LENGAN_GENDONG_MM    280.0f
#define LENGAN_GENDONG_TGI   140.0f

// Waktu tiap langkah sekuens. BUTA: tidak ada umpan balik posisi dari servo
// mana pun, jadi satu-satunya cara "menunggu sampai" adalah menunggu.
//
// 1100, bukan 900: dengan slew 120 der/s langkah terbesar (REHAT -> SIAP,
// 130,1 der pada PERGELANGAN) butuh 1084 ms, dan jatah yang lebih pendek
// membuat fase berikutnya mulai sementara lengan masih berjalan -- capit
// menyapu korban dalam perjalanan turun. Langkah terbesar DI DALAM sekuens
// sendiri cuma 90 der (SIAP -> JEPIT, 750 ms); yang menuntut 1100 adalah
// pose pertama, dari mana pun lengan kebetulan ditinggalkan.
//
// cek_korban.cpp yang menemukannya, dan assert-nya tetap di sana --
// menyetel salah satu dari keduanya tanpa yang lain akan gagal di PC.
// NAIK 1100 -> 2100 (15 Sep 2026) waktu sendi lengan dibuat BERURUTAN.
// Dulu ketiga sendi berangkat serentak, jadi satu langkah selesai secepat
// sendi yang TERLAMA. Sekarang bahu selesai dulu, baru siku, baru
// pergelangan, jadi lamanya = JUMLAH ketiganya. Langkah terberat sekuens
// korban (rehat -> siap) naik 110 der -> 230 der, 917 ms -> 1917 ms, dan
// 1100 ms tidak lagi cukup untuk satu pun dari lima langkahnya.
//
// 2100 menyisakan 183 ms (10%) di langkah terberat. Harganya sekuens korban
// yang kira-kira dua kali lebih lama; kalau ruas AMBIL/TARUH jadi terlalu
// lama di arena, yang disetel ARM_SLEW_DEG_S, bukan angka ini -- memotong
// jeda cuma mengembalikan gejala capit menutup sebelum lengan sampai.
#define LENGAN_JEDA_MS      2100

// LAJU MAKS SENDI LENGAN, derajat/detik. Perintah lengan menetapkan SASARAN;
// yang menggerakkan HexaArm::commit(), satu langkah tiap 20 ms.
//
// Tanpa ini tiap perintah lengan adalah lompatan: servo menerima pulsa pose
// baru sekaligus dan menyentak dengan kecepatan penuhnya (DS3225 ~460 der/s
// tanpa beban). Menyentak sambil menggenggam korban melemparkannya, dan
// menyentak sambil robot berdiri di tangga menggoyang seluruh badan.
//
// 120 der/s = seperempat laju bebas DS3225, jadi ia benar-benar membatasi,
// bukan sekadar angka yang kebetulan lebih besar dari yang sanggup dicapai
// servo. Langkah terbesar sekuens korban harus tetap muat di LENGAN_JEDA_MS;
// cek_korban.cpp yang menjaganya, jadi menurunkan angka ini tanpa menaikkan
// jeda akan GAGAL di PC, bukan di arena.
// NAIK 120 -> 150 pada 16 Sep 2026, diminta R2C: sekuens AMBIL terasa lambat
// di arena. Naik seperempat, bukan dua kali lipat -- alasan angka 120 dipilih
// (menyentak sambil menggenggam korban melemparkannya, menyentak di tangga
// menggoyang badan) masih berlaku, cuma marginnya yang dipakai.
//
// SELURUH margin jatah ikut melebar, jadi tidak ada yang perlu disetel ulang.
// Langkah terberat sekuens korban, rehat -> siap, 230 der berurutan:
//     120 der/detik = 1917 ms dari jatah 2100  (sisa 183 ms,  9%)
//     150 der/detik = 1533 ms dari jatah 2100  (sisa 567 ms, 27%)
//
// LENGAN_JEDA_MS sengaja TIDAK ikut diturunkan. Memotongnya akan memperpendek
// seluruh ruas AMBIL, tapi jatah yang sama juga memayungi translasi condong
// (60 mm pada 40 mm/detik = 1500 ms) -- dan itu tidak ikut lebih cepat. Satu
// perubahan, satu hal yang berubah.
#define ARM_SLEW_DEG_S      150.0f

// BATAS PERCEPATAN LENGAN, derajat/detik^2. Tanpa ini slew lengan berprofil
// KOTAK: 0 ke 150 der/detik seketika saat tahap mulai, dan 150 ke 0 seketika
// saat sampai. Dari arena, 'as0 0 0' dari REHAT: "sangat patah-patah dan
// sangat cepat". Kaki tidak pernah kena ini karena sasarannya sendiri sudah
// mulus (sikloid + ramp vektor); sasaran lengan lompatan, jadi kehalusannya
// harus dibuat di HexaArm::commit().
//
// 600 dipilih dari jatah waktu, bukan dari rasa. Trapesium menambah tepat
// v/a per tahap dibanding kecepatan tetap:
//
//     jarak lecut (v^2 / 2a) = 150^2 / 1200 = 18,8 der
//     tambahan waktu (v / a) = 150 / 600    = 0,25 detik per tahap
//
// Langkah terberat sekuens korban, rehat -> siap, kira-kira bahu 90 + siku 90
// + pergelangan 50 der. Dengan siku dan pergelangan kini SATU tahap:
//
//     kecepatan tetap  90/150 + 90/150            = 1,20 detik
//     dengan percepatan  1,20 + 2 x 0,25          = 1,70 detik
//
// Masih di dalam LENGAN_JEDA_MS 2100, sisa 400 ms (19%). Menurunkan angka ini
// ke 400 membuat sisanya cuma 150 ms -- jangan, tanpa menaikkan jeda dulu.
//
// Gerakan pendek ikut terurus sendiri: yang lebih dekat dari 2 x 18,8 = 37,5
// der tidak pernah sampai laju penuh, puncaknya sqrt(a x jarak). Pindah 10
// der berpuncak 77 der/detik, bukan 150.
#define ARM_ACCEL_DEG_S2    600.0f

// LAJU LIPAT KE REHAT, derajat/detik. Setengah laju biasa, dan cuma untuk
// langkah terakhir kedua sekuens korban: lipatan itu dikerjakan sambil capit
// MENGGENGGAM, dan lengan yang menyentak sambil membawa boneka melemparkannya.
//
// Langkah terbesarnya 95 der, jadi 60 der/detik butuh 1583 ms -- lebih lama
// daripada satu LENGAN_JEDA_MS. Karena itu fase terakhir sekuens sengaja
// dibiarkan KOSONG: lipatannya mendapat DUA jatah, 2200 ms. cek_korban.cpp
// yang menjaga keduanya tetap cocok.
#define LENGAN_SLEW_REHAT_DEG_S  60.0f

// LAJU NAIK KEMBALI KE SIAP, derajat/detik. Hanya untuk fase 4 sekuens AMBIL,
// yaitu mengangkat korban lurus keluar dari kantong reruntuhan.
//
// Di antara 150 penuh dan 60 milik lipatan, dan alasannya bentuk geraknya
// sendiri. Lipatan ke REHAT mengayunkan BAHU 90 der, dan bahu itulah yang
// melempar boneka keluar dari capit. Naik ke SIAP hampir seluruhnya SIKU
// (110 der) dengan bahu diam, dan arah gerakannya menekan boneka ke dalam
// capit, bukan melemparkannya.
//
// INI PILIHAN FISIK, BUKAN KEHARUSAN WAKTU. Angka ini lahir saat fase 4
// masih menuju pose SIAP, yang siku-nya 110 der dan pada 60 der/detik cuma
// menyisakan 7% dari jatah LENGAN_JEDA_MS. Pose ANGKAT jauh lebih dekat
// (siku 40 der), jadi 60 pun sekarang muat dengan kelegaan ~50%.
//
// Dipertahankan terpisah karena bedanya nyata di boneka, bukan di jam:
// silakan turunkan ke LENGAN_SLEW_REHAT_DEG_S kalau boneka terlepas saat
// diangkat. cek_lengan_laju.py mengukur ulang jatahnya tiap kali disetel.
#define LENGAN_SLEW_ANGKAT_DEG_S 90.0f

// POSE ANGKAT: fase 4 sekuens AMBIL, sesudah capit menutup. Dari arena
// 17 Sep 2026, dibidik dengan tangan lalu dibaca lewat 'as-40 50 -20'.
//
// BUKAN pose SIAP. SIAP (-50, 120, -70) dibentuk untuk mendekat dari atas
// dengan capit KOSONG dan menganga; menempuhnya balik sambil menggenggam
// boneka mengayunkan pergelangan 40 der tanpa keperluan. Pose ini cuma
// mengangkat: siku naik 40 der, bahu dan pergelangan nyaris diam.
#define KORBAN_ANGKAT_BAHU   -50.0f
#define KORBAN_ANGKAT_SIKU    50.0f
#define KORBAN_ANGKAT_PRG    -10.0f

// Titik capit yang diminta, mm dari pusat badan. Diturunkan dari gerbangnya
// sendiri supaya menggeser gerbang ikut menggeser lengan -- dua angka yang
// harus cocok tidak boleh ditulis dua kali.
#define KORBAN_CAPIT_MM  ((float)KORBAN_JARAK_CM * 10.0f + LIDAR_DEPAN_MM)

// --- Dimensi Kaki Hexapod --- //
// SUMBER KEBENARAN: legacy-2026/TES_GERAK/kinematics.h (program yang sudah
// terbukti berdiri). Nilai lama 23/54/69 SALAH -- femur meleset 26 mm dan
// tibia 21 mm, sehingga IK menghitung femur -35 der (seharusnya -10,6 der)
// dan lutut 127 der (seharusnya 82 der) pada pose berdiri baku.
// Jangan diubah tanpa mengukur ulang kaki fisik.
// ====================================================================
// DIUKUR DI ROBOT 14 Sep 2026 -- ANGKA DI BAWAH INI TERBUKTI SALAH,
// TAPI BELUM DIPERBAIKI. Perbaikannya menunggu satu ukuran lagi.
//
// Yang terukur, profil DATAR (perintah 'b100', servo hidup, lantai rata):
//   tinggi sumbu poros femur dari lantai    :  75 mm   (perintah 100)
//   jarak mendatar sumbu coxa -> telapak    :  80 mm   (perintah  70)
//   poros coxa -> poros femur               :  20 mm   (cocok, COXA_LENGTH)
//   poros femur -> poros lutut              :  55 mm   (di sini tertulis 80)
//   poros lutut -> ujung telapak            : ~65 mm   (di sini tertulis 90)
//   beda tinggi gir coxa -> poros femur     :  70 mm   (tidak dipakai IK,
//                                                       sumbu coxa tegak)
//
// DUA kesalahan, bukan satu, dan keduanya harus dibereskan bersama:
//
// 1. Panjang link salah. 80/90 memberi tinggi 100 mm pada radius 70; yang
//    terjadi 75 mm pada radius 80.
//
// 2. Datum sudut LUTUT meleset sekitar +24 der. Dengan link 55/65, telapak
//    di radius 80 / tinggi 75 menuntut femur -10,8 der (perintahnya -10,6 --
//    femur tepat sasaran) dan lutut 106,0 der, sementara perintahnya 82,0.
//    Teori "lutut nyata = lutut perintah + 24 der" memberi 74,8 mm / 80,2 mm
//    pada 'b100' -- terukur 75 / 80. Pada 'b130' ia memberi 92,3 mm,
//    sementara terukur 95..105 mm; jadi teorinya belum terkunci.
//
// Membetulkan link SAJA akan membuat robot berdiri di tempat ketiga yang
// juga salah. Jangan sentuh salah satunya sendirian.
//
// UKURAN YANG MENGUNCI, belum diambil: robot berdiri 'b100', ketik 'd' untuk
// melihat kolom sdeg c/f/t, lalu ukur dengan busur derajat pada kaki tengah
//   - sudut femur dari mendatar        (model:  -10,6 der)
//   - sudut DALAM lutut, femur-tibia   (model:   82,0 der; dugaan: 106 der)
//   - radius tapak pada 'b130'         (dugaan: ~81 mm, hampir tak berubah)
// Radius yang melonjak di 'b130' berarti galatnya BERSKALA, bukan offset
// tetap, dan tersangkanya pindah ke SERVO_PULSE_MIN/MAX atau servo 270 der
// yang dikira 180 der.
//
// AKIBAT YANG SUDAH BERLAKU SEKARANG, sebelum apa pun diperbaiki:
//   - Tinggi badan nyata 75 mm, bukan 100. Tiap pose lengan yang beracuan
//     lantai meleset 25 mm dari yang dihitung model.
//   - Jejak kaki nyata LEBIH LEBAR: ujung kaki tengah 90 + 80 = 170 mm dari
//     pusat badan, bukan 160. Lihat WALL_KAKI_CM.
//   - Margin guling di cek_kail dihitung di atas poligon tumpuan yang salah.
//
// Kedua legacy (TES_GERAK dan KALIBRASI) memakai 20/80/90 yang sama, jadi
// "sumber kebenaran" itu angka yang disalin turun-temurun tanpa pernah
// diukur. Nilai lama 23/54/69 yang dulu dibuang justru dekat dengan kaki
// yang sebenarnya.
// ====================================================================
#define COXA_LENGTH  20.0f
#define FEMUR_LENGTH 80.0f
#define TIBIA_LENGTH 90.0f

// Diukur di lengan fisik, 11 Sep 2026, POROS KE POROS (dari gear tiap servo)
// -- persis referensi yang dipakai ArmInverse. Menggantikan 20/24 yang cuma
// tebakan. Jangkauan jadi 8..166 mm dari bahu, bukan 4..44: empat kali lipat.
#define UPPERARM_LENGTH 87.0f  // bahu  -> siku
#define FOREARM_LENGTH  79.0f  // siku  -> pergelangan

// Pergelangan -> grip, POROS KE POROS.
//
// IKUT IK lewat moveArmGrip(), sebagai VEKTOR: ia dimundurkan dari titik
// sasaran searah tapak, jadi ia ikut berputar bersama sudut tapak, bukan
// sekadar dikurangkan. Yang TIDAK memakainya: moveArmTarget() dan perintah
// 'a', yang sasarannya memang poros PERGELANGAN.
//
// YANG BELUM MASUK MODEL: rahang capit menjulur di luar poros grip, dan
// panjangnya belum pernah diukur. Jadi yang ditaruh moveArmGrip() di titik
// sasaran adalah POROS SERVO GRIP, bukan titik jepitnya. Sesudah rahangnya
// diukur, tambahkan panjangnya ke sini -- arahnya sama persis, jadi satu
// angka cukup -- lalu jalankan cek_korban.cpp: jendela gerbangnya bergeser
// sejauh itu juga.
#define HAND_LENGTH    120.0f  // pergelangan -> poros grip


#define STAND_HEIGHT  100.0f // Tinggi badan dari tanah
#define STAND_RADIUS   70.0f // Jauh kaki ke pangkal coxa

// PROFIL KAIL -- bentuk khusus untuk R-9 (ruas 24), satu-satunya rintangan
// yang tidak bisa dilalui dengan bentuk badan seragam.
//
// Ketiganya offset TERHADAP posisi netral, mm, di frame badan (+y depan,
// +z atas), dan cuma berlaku pada profil KAIL. Kaki TENGAH sengaja tidak
// disentuh: ia yang menanggung badan selagi depan dan belakang bekerja.
//
// BENTUKNYA: kaki depan naik ke tapak di atas (anak tangga R-9 setinggi
// 36 mm) sambil membuka ke depan untuk mengait; kaki belakang MEMANJANG KE
// BAWAH karena ia masih di tapak yang lebih rendah. Selisih keduanya yang
// membuat badan tetap DATAR di bidang yang menanjak -- bukan kaki belakang
// yang ikut naik, yang justru menjungkitkan hidung.
//
// SUDUT YANG DIKOMPENSASI = atan((DEPAN_NAIK + BELAKANG_TURUN) / 156), dengan
// 156 mm jarak pangkal kaki depan ke belakang. 40 + 35 = 75 mm -> 25,7 der.
// Bidang R-9 terukur 27,3 der, jadi kompensasinya HAMPIR penuh, kurang ~1,6
// der. Itu disengaja; lihat batasnya di bawah.
//
// BATAS KERASNYA jangkauan IK, dan ia ditukar 1:1 dengan tinggi badan --
// diukur dengan cek_kail.cpp, tiap baris satu siklus gait penuh:
//
//     badan 115 mm -> belakang turun maks 30 mm (24,2 der)
//     badan 105 mm -> maks 40 mm (27,1 der)
//     badan 100 mm -> maks 45 mm (28,6 der)   <-- yang dipakai
//     badan  90 mm -> maks 55 mm (31,3 der)
//
// Karena itu KAIL TIDAK memakai tinggi badan TANGGA (115): 115 menyisakan
// cuma 30 mm dan itu sudah mentok sebelum sudut bidangnya tercapai. Pada 100
// mm sisa jatahnya 45, dan 35 memberi 10 mm kelegaan supaya kemiringan yang
// BERUBAH sepanjang tanjakan (terukur 5,9 -> 13,0 -> 24,9 -> 27,3 der) tidak
// langsung membuat IK mentok. Kaki yang mentok tidak sampai ke titik yang
// diperintahkan, dan bentuk badannya jadi tidak bisa diramalkan.
//
// 100 mm juga BUKAN penurunan kelegaan dari keadaan sebelumnya: sampai 8 Sep
// ruas ini berprofil DATAR, yang tinggi badannya memang 100, dan dengan itu
// robot sempat menempuh 62 dari ~103 cm.
//
// ANGKANYA BELUM DIUJI DI TANGGA -- yang sudah dibuktikan cuma bahwa IK
// sanggup mencapainya. Pasang dengan 'T4' sambil robot berdiri dan lihat
// bentuknya sebelum mempercayakannya ke misi.
// SIKLUS GAIT KAIL, ms DI ATAS gait.cycle_time. Ditulis tersendiri 17 Sep
// 2026, diminta R2C: sebelumnya KAIL memanggil profileFlat() dan ikut memakai
// siklus DATAR, jadi menyetel laju R-9 mustahil tanpa ikut mengubah seluruh
// misi.
//
// +300 adalah angka DATAR, jadi bawaannya tidak mengubah apa pun -- ini knop,
// bukan penyetelan. Bandingnya: TANGGA +200, MERUNDUK +200, TANJAK +400.
//
// Angkanya langsung jadi laju, karena badan maju 2 x stepLength per siklus.
// Pada gait.cycle_time 900 dan langkah 60 mm: +300 -> 1200 ms -> 10,0
// cm/detik, +500 -> 1400 ms -> 8,6, +700 -> 1600 ms -> 7,5.
//
// Naikkan kalau kaki depan menyangkut bibir anak tangga di R-9: ayunan yang
// lebih lama memberi telapak waktu melewati mukanya. Kalau itu tidak cukup,
// yang kurang tinggi langkahnya, bukan siklusnya -- lihat catatan TANJAK.
#define KAIL_CYCLE_TAMBAH_MS  300.0f

// PITCH BADAN PROFIL TANJAK, derajat. Disetel di robot 18 Sep 2026 dengan
// 'r0 5 0': tanpa ini robot cenderung jatuh ke belakang di tanjakan R-9.
//
// Tandanya mengikuti setBodyRotation(): pitch + = MENDONGAK. Di bidang miring
// itu memindahkan titik berat ke arah kaki yang mendaki, yaitu menjauhi tepi
// jungkir di belakang.
//
// HILANG SAAT 'b'. Perintah itu menolkan seluruh rotasi badan, jadi 'T5' lalu
// 'b' meninggalkan profil TANJAK tanpa pitch-nya. Itu memang benar -- 'b'
// adalah pose netral -- tapi jangan kaget kalau robot jadi mudah jatuh lagi
// sesudah menekannya. Ketik 'T5' sekali lagi, atau 'U' yang memasang seluruh
// rangkaiannya.
#define TANJAK_PITCH_DEG      5.0f

#define KAIL_TINGGI_BADAN   100.0f  // BUKAN 115 milik TANGGA -- lihat tabel
#define KAIL_DEPAN_MAJU      60.0f  // kaki depan membuka ke DEPAN
#define KAIL_DEPAN_NAIK      40.0f  // kaki depan NAIK ke tapak berikutnya
#define KAIL_BELAKANG_TURUN  42.0f  // kaki belakang MEMANJANG KE BAWAH
#define KAIL_RADIUS_KAKI     60.0f  // stance DIPERSEMPIT dari STAND_RADIUS 70

// Kaki TENGAH didorong KELUAR. Merekalah yang mengayuh paling jauh di KAIL:
// seluruh langkah tegak lurus terhadap arah hadapnya, jadi ayunan coxa =
// 2*atan(langkah/2 / radius) dan radius 60 membuatnya 60 der per siklus.
// Mendorong keluar MURAH -- kaki tengah cuma memakai 66% jangkauannya --
// dan sekaligus melebarkan poligon tumpuan, yang membayar kembali jejak
// belakang yang menyempit waktu kaki belakang diluruskan.
//   keluar  0 -> ayun 53 der (jejak +-150 mm)
//   keluar 30 -> ayun 37 der (jejak +-180 mm)  <-- dipakai
//   keluar 40 -> ayun 33 der (jejak +-190 mm)
// BATASNYA BUKAN IK MELAINKAN LEBAR LORONG: tiap 10 mm di sini melebarkan
// robot 20 mm. Ukur R-9 sebelum menaikkannya lagi.
#define KAIL_TENGAH_KELUAR   30.0f

// Kaki BELAKANG membuka lebih ke belakang. Ini MAHAL, tidak seperti di atas:
// jangkauan kaki belakang = radius + mundur + setengah langkah, dan dengan
// telapak menggantung 42 mm di bawah badan ia sudah 92% terpakai. Terukur,
// dengan langkah 60:
//   mundur  0 -> telapak y -138, D 94,0%
//   mundur  5 -> telapak y -143, D 95,4%  <-- dipakai
//   mundur 10 -> telapak y -148, D 96,8%  tidak bersisa
//   mundur 20 -> telapak y -158, D 99%
//   mundur 30 -> telapak y -168, MENTOK
// Turun dari 10 ke 5 waktu KAIL_BELAKANG_LEBAR dipasang: melebarkan telapak
// ikut menambah jangkauan (jarak telapak ke poros coxa naik), dan 96,8%
// tidak menyisakan apa pun untuk trim servo atau badan yang melendut.
// Kaki belakang yang hampir lurus justru LEMAH ke arah radial -- dan radial
// itulah arah ia mendorong badan naik. Jadi 96% bukan sekadar angka aman IK.
#define KAIL_BELAKANG_MUNDUR  5.0f

// Kaki belakang DILEBARKAN lagi dari garis lurusnya. Meluruskan telapak
// tepat di belakang poros coxa membuat coxa diam total, tapi jejaknya jadi
// +-45 mm -- terlalu menyatu. Ini menariknya kembali keluar, dan ONGKOSNYA
// coxa yang mengayuh lagi. Terukur, langkah 60:
//   lebar  0 -> jejak +-45, ayun coxa  0,0 der, margin 63 / lereng 39 mm
//   lebar 15 -> jejak +-60, ayun coxa 12,0 der, margin 66 / lereng 37 mm  <--
//   lebar 30 -> jejak +-75, ayun coxa 20,2 der, margin 72 / lereng 34 mm
//
// PERHATIKAN kolom lereng: melebarkan justru MEMPERBURUKnya. Saat CG mundur
// ke bawah lereng, sisi pengikat poligon bukan lagi sisi samping melainkan
// garis telapak-belakang ke kaki-tengah-seberang, dan menggeser telapak
// belakang KELUAR memutar garis itu mendekat ke CG. Jadi melebarkan membeli
// kestabilan di tanah datar dan membayarnya sedikit di tangga -- pilihan
// yang sah, tapi bukan perbaikan gratis.
// Aslinya (sebelum diluruskan) 41,5 der, jadi 15 mm masih menahan 71%
// manfaat pelurusan. Kalau di tangga ternyata selip lagi, INI yang pertama
// dikembalikan ke 0, bukan yang lain.
#define KAIL_BELAKANG_LEBAR  15.0f

// Sudut telapak kaki TENGAH, der dari arah hadap coxa-nya.
// NEGATIF = telapak MUNDUR, positif = MAJU. Radiusnya tidak berubah, jadi
// jangkauan IK-nya sama persis -- ini murni soal seimbang.
//
// Diukur dari PUSAT BADAN sudut ini tidak berpengaruh sama sekali (margin
// 67 mm untuk -30 sampai +30): di tiap tripod sisi pengikatnya adalah garis
// depan-belakang di SISI YANG SAMA, dan kaki tengah ada di sisi seberang.
// Ia baru bicara begitu CG bergeser. Margin guling (mm), lebar belakang 15:
//
//   CG        sudut -30  -20    0  +20  +30
//   +60 mm       41   48   59   69   69     korban di lengan depan
//     0 mm       67   67   67   67   67
//   -60 mm       55   52   43   34   28     mendongak di bidang 27,7 der
//   -90 mm       28   25   18   10    5
//
// R-9 melakukan KEDUANYA sekaligus: lereng 27,7 der menggeser CG ~73 mm ke
// BELAKANG (tinggi CG * tan), sementara korban K-4 yang digendong 280 mm di
// depan menariknya sebagian kembali. Selisihnya tidak diketahui -- massa
// bonekanya belum ditimbang. -20 dipilih karena di seluruh paruh CG <= -30
// ia tidak pernah rugi dan mulai untung, jadi ia taruhan yang aman terhadap
// ketidaktahuan itu. Kalau robot ternyata terguling ke DEPAN di tangga,
// balik tandanya jadi +20.
#define KAIL_TENGAH_SUDUT   0.0f

// LUTUT KAKI DEPAN DIKUNCI selama profil KAIL. 1 = kunci, 0 = IK biasa.
//
// Sebabnya dari tangga, bukan dari teori: kaki depan sering terpeleset.
// IK tiga sendi memakai tibia untuk ikut mengejar titik telapak, jadi lutut
// MENGAYUH sepanjang stance -- dan tiap derajat ayunan itu menggeser titik
// sentuh di ujung kaki, yang di permukaan anak tangga berarti menggaruk.
// Dengan lutut dikunci, yang bergerak tinggal coxa dan femur: telapak
// berayun pada BUSUR berjari-jari tetap mengelilingi sendi femur.
//
// Harganya: telapak tidak lagi menuruti lintasan yang diminta gait. Yang
// tersisa hanya komponen ARAHnya; komponen radial hilang. Seberapa besar
// selisihnya dicetak cek_kail.cpp -- baca di sana sebelum menaikkan
// KAIL_DEPAN_MAJU/NAIK, karena keduanya tidak lagi berpindah 1:1 ke telapak.
//
// Sudut kuncinya TIDAK ditulis di sini: ia diambil dari pose netral kaki
// itu sendiri (sesudah offset KAIL), jadi bentuk berdirinya tetap persis
// dan hanya ayunannya yang berubah.
#define KAIL_LUTUT_KUNCI 0


// NAIK 11 Sep 2026 sesudah uji T4 di tangga: maju 40->60, turun 35->42.
// Operator melaporkan kaki depan sering terpeleset dan buritan masih bisa
// diangkat. Sapuan cek_kail memang menyisakan ruang di kedua arah -- pada
// badan 100 mm kaki depan boleh maju sampai 70 dan kaki belakang turun
// sampai 45. Angka lama cuma mengimbangi bidang 25,7 der, di BAWAH R-9
// yang 27,3 der, jadi badan masih mendongak dan beban lari ke belakang --
// itulah yang membuat kaki depan kehilangan tumpuan. Sekarang 27,7 der.
//
// Sengaja BUKAN nilai maksimum (70/45): keduanya angka terakhir yang masih
// lolos sapuan berlangkah 5-10 mm, jadi tanpa kelegaan sama sekali. Satu
// mm keausan servo atau trim yang meleset sudah cukup membuatnya mentok.
//
// JANGAN naikkan KAIL_DEPAN_NAIK untuk mengejar sudut lebih besar: itu
// melewati 27,3 der dan membuat badan MENUNDUK di tanjakan, sehingga kaki
// depan menekan tegak lurus ke muka anak tangga alih-alih mengait tapaknya.


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
// DEPAN: bahu, siku, pergelangan, grip. Bahu dan siku menyapu satu bidang
// VERTIKAL; tidak ada sendi pemutar di pangkal, jadi untuk membidik objek yang
// tidak segaris, BADAN robot yang harus diarahkan. Pergelangan adalah sudut
// KETIGA yang disetel sendiri, bukan turunan dari bahu+siku -- ia tidak ikut
// dihitung IK.
// BELAKANG: cuma grip. Tidak ada yang bisa dijangkau; posisi capit belakang
// sepenuhnya ditentukan letak BADAN.
const uint8_t ARM_PIN_MAP_DEPAN[ARM_N_DEPAN][2] = {
    {0, 12},   // bahu
    {0, 13},   // siku
    {0, 14},   // pergelangan
    {0, 15}    // grip
};
const uint8_t ARM_PIN_MAP_BELAKANG[ARM_N_BELAKANG][2] = { {1, 12} };   // grip

// Pangkal BAHU lengan DEPAN (x, y, z) dari pusat badan, mm. Baris BELAKANG
// tinggal sisa: lengan itu tidak punya bahu, jadi angkanya tidak dipakai IK.
// Lengan depan menghadap +Y (maju), lengan belakang menghadap -Y (mundur).
// Versi lama keliru menaruh offset di sumbu X (kanan/kiri) -- itu sisa dari
// asumsi "lengan kanan & kiri" yang ternyata salah.
// DEPAN sudah diukur 11 Sep 2026 (85/45). BELAKANG masih tebakan lama, dan
// memang tidak penting: moveArmTarget() menolak ARM_BELAKANG di baris
// pertama, jadi angkanya tidak pernah masuk perhitungan apa pun.
//
// Akibat 85/45: bahu duduk 145 mm di atas lantai sedangkan lengan cuma
// menjangkau 166 mm, jadi PERGELANGAN hampir tidak bisa mencapai lantai
// (cuma dalam 81 mm di depan bahu, itu pun tepat di batas singular).
// Bukan masalah: capit ada 120 mm di depan pergelangan, jadi pose ambil
// yang benar menaruh pergelangan di sekitar tinggi +20, bukan di lantai.
// Lihat cek_lengan.cpp.
#define ARM_DEPAN     0
#define ARM_BELAKANG  1
const float ARM_ORIGINS[2][3] = {
    { 0.0f,  85.0f, 45.0f},  // ARM_DEPAN    : DIUKUR 11 Sep 2026 -- 85 mm depan, 45 mm atas
    { 0.0f, -50.0f, 30.0f}   // ARM_BELAKANG : 50 mm di belakang pusat
};

// BUS I2C (Wire SDA 18 / SCL 19 Teensy 4.1)
//
// TERUKUR 16 Sep 2026, dan BELUM DIPERBAIKI: bus Wire mati. Pin 18 (SDA)
// terbaca RENDAH bahkan dengan kabelnya dilepas, tetap rendah sesudah
// sembilan pulsa clock, dan tetap rendah sesudah catu dicabut. Diuji ulang
// dengan sketsa contoh Wire/Scanner, di luar firmware ini:
//
//     Wire   (18/19)  TIDAK ADA alamat sama sekali
//     Wire1  (16/17)  0x41 PCA9685        ketemu
//     Wire2  (24/25)  0x40 PCA9685, 0x3C  ketemu
//
// Jadi perangkat I2C di dalam Teensy sehat, 3V3 dan GND sehat, dan yang mati
// cuma satu bus. Selama pin 18 belum diperbaiki, LiDAR tidak akan hidup.
//
// MEMINDAHKAN BUS INI ADA HARGANYA, dan harganya bukan soal pin. Alamat
// ALL-CALL PCA9685 juga 0x70, sama dengan TCA9548 -- jadi mux yang ditaruh
// satu bus dengan driver servo membuat DUA perangkat menjawab 0x70, dan
// selectMux() yang menulis 1<<ch akan terbaca driver servo sebagai penunjuk
// register. Kalau bus ini harus pindah nanti, matikan dulu ALL-CALL kedua
// PCA9685 (MODE1 bit 0, baca-ubah-tulis SESUDAH setPWMFreq), atau ubah
// alamat mux lewat pin A0/A1/A2 ke 0x71.
#define LIDAR_I2C_BUS    Wire
#define SERVO_0_I2C_BUS  Wire1
#define SERVO_1_I2C_BUS  Wire2

// Nomor pin mentah bus LiDAR. Objek Wire memakainya sendiri, tapi PEMULIHAN
// BUS harus menggoyang SCL sebagai GPIO biasa -- dan itu tidak bisa
// ditanyakan ke objek Wire. WAJIB ikut berubah kalau LIDAR_I2C_BUS pindah:
// Wire 18/19, Wire1 17/16, Wire2 25/24 (SDA/SCL).
#define LIDAR_I2C_SDA    18
#define LIDAR_I2C_SCL    19

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

// JARAK BACAAN SAAT KAKI SAMPING MENYENTUH DINDING. Bukan penyaring sensor,
// dan sejak 14 Sep 2026 bukan hitungan: DIUKUR DI ROBOT. Kaki kanan tengah
// didorong sampai menyentuh dinding kanan, kedua LiDAR kanan membaca ~7 cm.
//
// Nilai lama 11,0 dihitung "ujung kaki tengah 160 mm - dudukan sensor 50 mm",
// dan KEDUA angka itu meleset: ujung kaki tengah yang sebenarnya 170 mm
// (poros coxa 90 + radius tapak terukur 80), dan dudukan sensornya jelas
// bukan 50 mm, karena 170 - 70 menuntut 100 mm. Mana yang salah tidak perlu
// dipecahkan: yang dipakai rumus adalah BACAANNYA, dan bacaan itu sekarang
// terukur langsung.
//
// Akibat angka lama: pita ramp WALL_MIN_CM - WALL_KAKI_CM cuma 1 cm, jadi
// kemudi melompat dari PD murni ke dorongan penuh dalam satu sentimeter --
// kelas masalah yang sama dengan bang-bang MENENGAH yang sudah dibuang.
// Dengan 7,0 pitanya 5 cm dan dorongan penuh jatuh tepat di titik sentuh.
//
// Dipakai pita "terlalu dekat" di Navigation sebagai ujung rampnya: dorongan
// menjauh separuh di wall.min, PENUH di sini. Dulu ia meminjam
// LIDAR_MIN_CM[samping], yang kebetulan bernilai mirip -- dua makna menumpang
// pada satu angka. Dipisah supaya menyetel penyaring sensor tidak diam-diam
// menggeser titik dorongan penuh.
#define WALL_KAKI_CM      7.0f

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
// berpindah, dan mengejarnya berakhir di FRONT_STOP_CM (20 saat itu, kini
// 12), yaitu sensor
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

// SELISIH TERBESAR (belakang - depan) yang masih dianggap dinding, cm.
// Lewat dari ini sudutDinding() mengembalikan NaN: bukan dinding miring,
// melainkan salah satu berkas menembak celah, pintu, atau tikungan.
//
// Ia juga memagari sudut TERBESAR yang bisa dibaca sama sekali:
// atan(4,0 / 11,0) = 20,0 der. Ambang koreksi mana pun harus di bawah itu,
// kalau tidak ia tidak pernah menyala. cek_koreksi.cpp membuktikannya.
//
// Dulu static const di Navigation.cpp; dipindah 17 Sep 2026 supaya uji PC
// membaca angka yang sama, bukan salinannya.
#define SISI_BEDA_MAKS_CM  4.0f

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

// --- PEMERIKSAAN PENCATATAN KOMPAS ('c0'..'c3') ---
//
// Dipasang 16 Sep 2026 sesudah laporan R2C: sesudah kabel IMU dipindah lewat
// header, tabel kompas berantakan -- selisih SELATAN ke TIMUR cuma 30 der,
// bukan 90. Gejala itu TIDAK terlihat saat mencatat; ia baru ketahuan jauh
// kemudian, saat robot berjalan ke arah yang salah di arena.
//
// 'c<n>' dulu mengambil SATU sampel seketika. Satu sampel tidak bisa
// membedakan heading yang tenang dari heading yang sedang melompat-lompat,
// jadi sekarang ia merata-rata sebentar dan MENOLAK kalau sebarannya lebar.
#define KOMPAS_SAMPEL_MS      400    // lama merata-rata satu pencatatan
#define KOMPAS_SEBAR_MAKS_DER   3.0f // sebaran maks dalam jendela itu
#define KOMPAS_GAP_TOL_DER     10.0f // toleransi jarak antar mata angin dari 90
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

// --- DETEKSI TERGULING ----------------------------------------------------
//
// KETIGA ANGKA DI BAWAH BELUM DIUKUR. Ini titik awal untuk disetel DI ROBOT,
// bukan hasil pengukuran -- dan salah setel berarti robot menyerah di tanjakan
// yang seharusnya ia naiki.
//
// accelZ mentah sudah lama dibaca dan dicetak aliran 'y'; komentar di sana
// menuliskannya sendiri: itulah SATU-SATUNYA angka yang tahu papan IMU
// menghadap ke mana (+1 g tegak, -1 g terbalik). Sampai sekarang tidak ada
// yang bertindak atasnya -- padahal robot ini menaiki tangga dengan margin
// guling 29,4 mm (cek_kail MERAH, CLAUDE.md). Kalau ia terguling di tengah
// misi, gait TETAP berjalan dan servo TETAP memaksa kaki ke sasaran IK yang
// sudah tidak berarti apa-apa; yang rusak bisa lebih dari skornya.
//
// URUTAN PERCAYA: accelZ DULU, roll belakangan. roll datang dari fusi yang
// mengandalkan magnetometer; di arena berangka besi itu jauh lebih mudah
// dibohongi daripada satu sumbu percepatan. Saat robot benar-benar terguling,
// |az| jatuh ke sekitar nol karena gravitasi pindah ke sumbu lain -- itulah
// tanda yang dipakai.
//
// ROLL SENGAJA JAUH DARI KEMIRINGAN ARENA. Profil TANJAK bekerja pada 27,7 der
// (lihat blok KAIL di atas) dan itu NORMAL. Ambang yang mendekatinya membuat
// robot menyerah di tanjakan -- karena itu 45, bukan 30.
//
// Saklar MATI disediakan karena ini MENAMBAH satu cara baru misi bisa berhenti
// sendiri: saat menyetel ambangnya di robot, orang harus bisa mematikannya.
// DIUKUR DI ROBOT 18 Sep 2026, sesudah pemeriksa ini melemaskan servo pada
// robot yang berdiri tegak sempurna:
//
//     accelZ -0.99 g, roll 176.3 der
//
// Dua anggapan yang keliru, dan keduanya baru terlihat di robot ini:
//
//   1. DATAR ITU ROLL 0. Tidak di sini -- IMU terpasang sehingga datar
//      terbaca +-180 der. 'fabsf(roll) > 45' karena itu menyala terus.
//      Yang benar JARAK ke tegak, dan tegak boleh 0 ATAU +-180:
//      min(|roll|, 180-|roll|).
//   2. TANDA accelZ TIDAK PENTING. Justru itu satu-satunya yang membedakan
//      tegak dari terbalik: keduanya memberi |accelZ| ~ 1 g. Dengan fabsf(),
//      robot yang benar-benar terbalik lolos sebagai "tegak" -- pemeriksa
//      terguling yang tidak bisa melihat robot terguling.
//
// TERGULING_AZ_TEGAK adalah TANDA accelZ saat robot berdiri, bukan besarnya.
// -1 dibaca langsung dari log di atas. Kalau IMU dipasang ulang, ukur lagi:
// ketik 'b', lihat accelZ di 'd', ambil tandanya saja.
#define TERGULING_AKTIF      1        // 0 = MATI. Baku HIDUP.
#define TERGULING_AZ_TEGAK  -1.0f     // DIUKUR: tanda accelZ saat berdiri
#define TERGULING_AZ_G       0.5f     // BELUM DIUKUR: accelZ tegak di bawah ini
#define TERGULING_ROLL_DEG  45.0f     // BELUM DIUKUR: simpangan roll di atas ini
#define TERGULING_TUNDA_MS   400      // BELUM DIUKUR: harus bertahan selama ini

// Batas parkir menunggu Raspi, milidetik.
//
// VISI: sesudah '#KORBAN AMBIL' terkirim, firmware diam menunggu 'm2'. Habis
// waktu TIDAK menggagalkan apa pun -- sekuens tetap jalan memakai sudut
// tetap, persis perilaku tanpa vision. Jadi mode ini tidak pernah lebih buruk
// daripada mematikannya.
//
// LEPAS: sesudah capit selesai, firmware memberi tahu Raspi lalu menunggu
// 'm9' ("aku sudah tidak memegang kaki"). Jauh lebih pendek karena Raspi
// hanya perlu menetralkan badan, bukan mencari korban.
// KOMPENSASI MAJU SAAT PERATAAN ('V').
//
// Geser menyamping tidak pernah murni menyamping: sapuan kaki menyeret badan
// ke depan beberapa sentimeter tiap siklus, dan pada ruas K-3/K-4 beberapa
// sentimeter itu yang membuat capit menabrak reruntuhan. LiDAR BELAKANG yang
// melihatnya -- robot yang maju menjauh dari dinding belakang, jadi bacaannya
// NAIK.
//
// SASARAN MUTLAK 10 cm, bukan "pertahankan bacaan saat 'V' dimulai" (R2C
// 15 Sep 2026: "saat menggunakan perintah V, lidar belakang diusahakan
// 10 cm"). Bedanya nyata: acuan relatif cuma menahan hanyut, sedangkan
// sasaran mutlak ikut MEMPERBAIKI posisi yang sudah kelewat maju sebelum 'V'
// dimulai.
//
// 10 dan bukan lebih kecil: LIDAR_MIN_CM[belakang] = 7, dan di bawah angka
// itu bacaan dilaporkan 'jauh' alih-alih jarak. Sasaran 10 menyisakan 3 cm
// sebelum penggarisnya berhenti berbicara.
//
// Kompensasinya SATU ARAH -- lihat RATA_BLK_MUNDUR. Robot hanya ditarik
// mundur saat bacaan DI ATAS sasaran; bacaan di bawah 10 dibiarkan. Jadi
// sasaran ini adalah batas atas, bukan titik yang dikejar dari dua sisi, dan
// robot tidak pernah didorong maju ke arah reruntuhan.
#define RATA_BLK_SASARAN_CM  10
//
// TOLERANSI 0, atas keputusan R2C 15 Sep 2026. Artinya tiap kenaikan bacaan
// belakang, sekecil apa pun, langsung dikompensasi.
//
// Harganya diketahui dan diterima: bacaan LiDAR berderau satu-dua sentimeter,
// jadi pada 0 kompensasinya menyala di sekitar separuh tick meski robot tidak
// benar-benar hanyut. Efeknya bukan goyangan -- komponen mundur ini kecil dan
// hanya SATU arah -- melainkan bias mundur lemah sepanjang perataan. Kalau
// suatu saat robot terlihat merayap mundur selama 'V', angka inilah yang
// dinaikkan lagi, bukan RATA_BLK_MUNDUR yang diturunkan.
#define RATA_BLK_TOL_CM      0
// Laju mundur kompensasi. Kecil dengan sengaja -- ia berjalan BERSAMAAN
// dengan geser, jadi yang dikoreksi hanyut, bukan perpindahan.
#define RATA_BLK_MUNDUR    0.20f

// Batas waktu perintah 'J' (setel jarak belakang, dua arah). Sama besarnya
// dengan perataan: keduanya gerak lambat berumpan-balik LiDAR, dan keduanya
// butuh jaring terakhir.
#define MUNDUR_BATAS_MS    12000

#define MISI_VISI_BATAS_MS  20000
#define MISI_LEPAS_BATAS_MS  5000

// --- Raspi 5: pemicu deteksi korban --- //
// Teensy tidak ikut mendeteksi apa pun. Ia cuma memberi tahu Raspi KAPAN
// robot sudah berhenti di depan korban; isi kamera urusan Raspi. Satu baris
// teks, sekali per ruas korban, tanpa jawaban balik:
//
//   #KORBAN AMBIL 8
//   #KORBAN TARUH 10
//
// Angka di belakang = nomor ruas, sama dengan yang dipakai 'm4'/'m6'.
//
// Default Serial = kabel USB Teensy (nol kabel tambahan; Raspi membaca
// /dev/ttyACM0 dan kebagian seluruh log misi sekalian). Kalau USB dipegang
// laptop operator dan Raspi dicantol ke UART GPIO, ganti ke Serial1 (pin
// 0=RX, 1=TX) -- lalu tambahkan Serial1 ke ../test-pc/stub/Arduino.h, yang
// sekarang cuma punya Serial dan Serial2, supaya cek PC tetap bisa dibangun.
//
// RASPI BOLEH MENJAWAB, TAPI HANYA 'm2', 'm3' DAN 'm9'. Ketiganya jawaban
// atas pertanyaan yang firmware ajukan sendiri ('#KORBAN AMBIL', '#LEPAS'),
// dan hanya diterima saat firmware memang sedang menunggunya. Byte lain
// masuk ke parser perintah penuh: 'W' menulis kalibrasi, 'm0' membatalkan
// misi di tengah arena. Jangan kirim apa pun di luar ketiga itu.
//
// Selama parkir vision, kendali KAKI milik Raspi: ia menengahkan badan lewat
// 'r0 0 <yaw>' dan pivot 'O'. ruasSehat() tidak berjalan di MISI_KONFIRM,
// jadi perintah itu tidak menggagalkan misi -- tapi hanya di state itu.
#define KORBAN_SERIAL    Serial
#define KORBAN_BAUD      115200

// --- Navigasi --- //
#define HEADING_TOLERANCE_DEG   6.0f     // Toleransi heading dianggap "lurus" (const)
// TURUN 20 -> 12, 9 Sep 2026, atas pengamatan operator: LiDAR depan membaca
// 10-15 cm masih aman, badan belum menyentuh apa pun di sana. Angka 20 dulu
// dipilih konservatif, dan harganya nyata -- setiap ruas yang mau berhenti
// rapat ke tembok ditolak tabelSiap(), karena ambang HNT_DEPAN wajib DI ATAS
// konstanta ini.
//
// JANGAN turunkan lebih jauh tanpa memindahkan LIDAR_MIN_CM[depan] (7 cm)
// lebih dulu. Di bawah 7 cm bacaan diumumkan "kosong", bukan "mepet", jadi
// pita tempat halangan masih bisa dikenali cuma 7..12 cm. Pada 12,7 cm/detik
// pita selebar 5 cm itu ~400 ms, cukup untuk beberapa sampel LiDAR. Pita
// 7..10 tidak.
// 8, TURUN dari 12 pada 15 Sep 2026, dan ini TIDAK berdiri sendiri.
// Ruas 15 ("putar KIRI lalu maju") kini berhenti di 10 cm supaya robot tidak
// menempel pada reruntuhan yang menutupi K-3/K-4 -- kamera butuh ruang untuk
// melihat korbannya. tabelSiap() menolak ambang HNT_DEPAN yang <= angka ini,
// dan alasannya nyata: navigasi berhenti sendiri sebelum pemicu misi menyala,
// jadi ruasnya menggantung sampai batas waktu.
//
// HARGANYA NYATA DAN GLOBAL: seluruh arena kini berhenti 4 cm lebih rapat ke
// halangan. Kalau ruas 15 dikembalikan ke >= 13 cm, kembalikan angka ini ke
// 12 juga -- menurunkannya sendirian tidak ada gunanya.
#define FRONT_STOP_CM      8       // Berhenti/belok bila depan < ini (const)
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

// --- KOREKSI SAMBIL BERHENTI -- R-9 lewat 'U' -----------------------------
//
// Diminta R2C 17 Sep 2026. Di tanjakan, mengoreksi sambil melangkah berarti
// kaki ayun mendarat di tempat yang sudah bergeser sejak ayunannya dimulai.
// Jadi robot BERHENTI dulu, memutar badan sampai sejajar dinding kanan, baru
// jalan lagi.
//
// Menyala HANYA lewat 'U' (Navigation::koreksiDiam). Seluruh misi dan
// perintah navigasi lain tetap mengoreksi sambil berjalan seperti biasa.
//
// Sudutnya dari sudutDinding(), yaitu KEDUA LiDAR kanan -- dudukan depan dan
// belakang pada basis WALL_BASE_CM. Ia sah sampai sekitar +-20 der, karena
// sudutDinding() menolak |beda| di atas SISI_BEDA_MAKS_CM 4,0 cm.
#define NAV_KOREKSI_SUDUT_DEG    6.0f   // masuk koreksi bila |sudut| lewat ini
#define NAV_KOREKSI_JARAK_CM     4.0f   // atau bila jarak meleset sejauh ini

// KELUAR jauh lebih ketat daripada MASUK, dan itu histeresis yang disengaja.
// Ambang tunggal membuat robot berhenti-jalan-berhenti tepat di ambangnya --
// kelas masalah yang sama dengan relay wall.min yang sudah dibuang dari lup
// kemudi ini.
#define NAV_KOREKSI_KELUAR_DEG   2.0f   // keluar bila sisa sudut di bawah ini

// Galat JARAK tidak bisa ditutup dengan memutar di tempat. Yang dikerjakan:
// membidik sudut serong kecil ke arah dinding, lalu BERJALAN menutupnya.
// Karena itu sasaran koreksi bukan selalu nol derajat.
#define NAV_KOREKSI_JARAK_K      0.5f   // der sudut bidik per cm galat jarak
#define NAV_KOREKSI_BIDIK_MAKS   8.0f   // batas sudut bidik

// Menyerah lalu jalan lagi. Berdiri selamanya di tengah tanjakan lebih buruk
// daripada berjalan agak miring: rem jarak tidak pernah tercapai, dan operator
// cuma melihat robot diam tanpa sebab.
#define NAV_KOREKSI_BATAS_MS     3000

// MENENGAH: seberapa MIRING robot di lorong sebelum dorongan menjauh penuh.
// Bukan jarak ke dinding melainkan SELISIH kedua sisi dibagi dua, jadi 3 cm
// di sini berarti satu dinding 6 cm lebih dekat daripada yang lain.
//
// Ada karena dorongan menjauh dulu bang-bang terhadap "dinding mana yang
// lebih dekat". Di lorong sempit yang robotnya sudah DI TENGAH, pertanyaan
// itu dijawab oleh derau: dk 11,4/dn 11,6 lalu 11,6/11,4 membalik jawabannya,
// dan tiap pembalikan membalik kemudi sebesar NAV_WALL_TURN_MAX PENUH.
// Terukur pada derau +-0,2 cm: kemudi berbalik 7x per 10 sampel dengan
// amplitudo 1,00, padahal PD-nya sendiri cuma +-0,001 -- robot sudah benar.
// Dengan pita ini dorongan ikut BESARNYA kemiringan, jadi ia hilang sendiri
// tepat di tempat pertanyaannya jadi tidak berarti.
#define NAV_TENGAH_PITA_CM 3.0f

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
#define EE_PROFIL_ADDR   2304    // Enam profil medan, terpisah dari kalibrasi lama

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
// 40 -> 80 pada 16 Sep 2026, supaya condong maju 5..8 cm muat. Angka ini
// pagar, bukan batas mekanis; yang menentukan jangkauan IK kaki. Diperiksa
// pada STAND_HEIGHT 100 dan STAND_RADIUS 70, kaki yang paling tertarik:
//
//   geser MAJU 80 mm : kaki belakang menuntut 160 mm dari 170 (94%)
//   geser MAJU 60 mm : 146 mm (86%)
//   geser MAJU 50 mm : 139 mm (82%)
//
// Clamp ini juga berlaku untuk X dan Z. Geser MENYAMPING 80 mm menuntut 164 mm
// (96%) pada kaki tengah -- lebih ketat daripada arah maju. Vision menulis X
// lewat 't<x> 0 0' dan sekarang boleh sampai 80; kalau kaki tengah mulai
// mentok saat Raspi menengahkan badan, ke sinilah harus dilihat.
#define BODY_MAX_TRANS_MM  80.0f  // clamp geser badan X/Y/Z

// LAJU RAMP POSE BADAN. Tanpa ini, 'r20 0 0' mengubah sudut femur ~49 der
// dalam SATU siklus commit 20 ms (~550 us lompatan pulse) -- servo disuruh
// bergerak ~2400 der/detik dan robot menyentak keras. Vektor gerak gait sudah
// di-slew (GAIT_SLEW_RATE) dan profil medan sudah di-ramp (GAIT_PROFILE_TAU),
// tapi pose badan dulu diterapkan MENTAH karena letaknya sesudah gait.
// Demo 'B' tidak terpengaruh: sapuan sinusnya paling cepat ~21 der/detik.
#define BODY_SLEW_DEG_S    60.0f  // laju maks rotasi badan, derajat/detik
#define BODY_SLEW_MM_S    120.0f  // laju maks geser badan, mm/detik

// LAJU CONDONG, mm/detik. Sengaja jauh di bawah BODY_SLEW_MM_S: badan yang
// menggeser cepat sambil kaki diam menggoyang seluruh robot, dan goyangan itu
// terjadi tepat di atas korban yang akan dicapit.
//
// 40 mm/detik dipilih dari jatah fasenya, bukan dari rasa: translasi dapat
// SATU jatah LENGAN_JEDA_MS, dan condong.mm boleh sampai 79 mm. 79 / 40 =
// 1975 ms, masih muat di 2100 ms. Menurunkan angka ini tanpa menaikkan
// LENGAN_JEDA_MS berarti lengan mulai turun sebelum badan sampai.
#define KORBAN_CONDONG_LAJU_MM_S  40.0f

// MUNDUR SAAT MENGANGKAT, mm di belakang pose netral. Diminta R2C 17 Sep 2026:
// korban yang terangkat sering menabrak dinding DI BELAKANGNYA, yaitu dinding
// yang dihadapi robot.
//
// Sebabnya geometri, bukan pose lengan. Capit menggenggam di ujung lengan yang
// menjulur ke depan, dan melipatnya ke REHAT mengayunkan boneka naik melalui
// busur -- penampangnya saat itu jauh lebih besar daripada capit kosong yang
// dipakai menyetel pose. Di ceruk sempit, busur itulah yang membentur.
//
// Badan ditarik mundur sejauh ini SEBELUM lengan mulai melipat, lalu pulang ke
// nol sesudah lipatannya selesai. Kaki tidak bergerak: ini geser badan, sama
// seperti condong, jadi ia tidak mengganggu odometri ruas berikutnya.
#define KORBAN_ANGKAT_MUNDUR_MM   20.0f
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
#define DEMO_BOOT_PERINTAH "B"    // dijalankan apa adanya lewat handleCmd()
#define BODY_DEMO_TRANS_MM 25.0f  // amplitudo translasi saat demo 'B'
#define BODY_DEMO_PHASE_S   3.0f  // detik per sumbu (6 sumbu = 18 detik)

#define SERVO_PWM_FREQ    50      // Hz, frekuensi sinyal PCA9685 (50-330 Hz)
#define SERVO_COMMIT_MS   20      // ms, periode kirim 18 pulse (20=50Hz, 10=100Hz)

// ACUAN LAJU LOOP, BUKAN PEMBATAS -- dan itu disengaja, bukan utang.
//
// Sampai 18 Sep 2026 komentar di sini menjanjikan "tick loop utama", padahal
// angka ini TIDAK DIBACA di satu tempat pun. loop() berjalan bebas, dan itu
// memang bentuk yang benar untuk firmware ini: seluruh kendali sudah berbasis
// dt sungguhan (HexaGait::dtSeconds(), slewBodyPose()), jadi menambahkan
// pembatas laju tidak menambah apa pun -- ia hanya membuang waktu yang bisa
// dipakai memproses serial dan LiDAR.
//
// Satu-satunya pemakainya sekarang baris "PROF" di bawah: sebagai ACUAN
// persentase utilisasi (rata-rata putaran dibagi periode 10 ms). Angka 100 Hz
// adalah target rancangan, bukan janji yang ditegakkan siapa pun.
#define CONTROL_HZ        100     // Hz, acuan utilisasi di baris PROF (BUKAN pembatas laju)
#define PROFILE_LOOP      1       // 1 = cetak "PROF avg/max/util" tiap detik (saat tak tuning)
#define GAIT_DEBUG        0       // 1 = cetak fase gait tiap 200 ms (hanya saat melangkah)

#endif
