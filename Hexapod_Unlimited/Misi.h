#ifndef MISI_H
#define MISI_H

// ====================================================================
// LAPISAN MISI KRSRI -- BERBASIS TABEL RUAS.
//
// Pengganti Mission.* yang lama. Yang lama menulis satu state per potongan
// lintasan: 12 state untuk seperempat rute. Rute penuh guidebook 2026 punya
// 20-an potongan, jadi pola itu berakhir di ~60 state yang masing-masing
// MENYALIN logika penjaganya sendiri -- dan penjaga yang disalin adalah
// penjaga yang suatu saat lupa disalin. (Cacatnya sudah terbukti sekali:
// berjalan() di versi lama tidak menyebut lima state ruas berjarak, sehingga
// 'm0' di tengah lantai pecah diam-diam tidak melakukan apa pun.)
//
// Di sini lintasan adalah DATA, bukan kode: satu baris RUAS[] per potongan.
// Mesin statusnya cuma lima: JALAN, PIVOT, SETEL, LENGAN, KONFIRM. Menambah
// potongan lintasan = menambah satu baris tabel; menyetel arena = mengubah
// angka, bukan mengubah alur.
//
// ATURAN POKOK YANG DIPERTAHANKAN dari versi lama (semuanya mahal dipelajari,
// jangan dibuang):
//   1. Misi TIDAK PERNAH memanggil robot.walk(). Vektor gerak milik
//      Navigation seorang diri; misi menyetir lewat navMulai()/navBerhenti().
//      Dua penulis pada vektor yang sama = robot yang "menolak berhenti".
//   2. update() dipanggil SEBELUM nav.navUpdate(). Keduanya membaca sampel
//      LiDAR yang sama; yang lebih dulu berhak memutuskan.
//   3. Pemicu jarak dihitung HANYA saat ada sampel LiDAR BARU (stempelSampel),
//      bukan tiap iterasi loop. Tanpa itu "3 sampel berturut-turut" cuma
//      berarti tiga kali membaca angka yang sama.
//   4. LIDAR_JAUH bukan "sangat dekat" dan bukan "sudah lewat". Sensor mati,
//      sensor jauh, dan jarak sungguhan adalah TIGA keadaan berbeda.
//   5. Ketentuan kontes: robot boleh diletakkan menghadap ke mana saja lalu
//      harus berangkat ke arah ruas pertama. Karena mode arena mengunci ke
//      mata angin TERDEKAT, misi WAJIB pivot lebih dulu -- start yang
//      menyimpang > 45 der akan mengunci arah yang salah tanpa pesan error.
//
// SUMBER ANGKA: Guidebook SAR UNLIMITED 2026 (halaman 21-30). Angka yang
// benar-benar tertulis di sana sudah terisi di tabel. Angka yang TIDAK ada di
// guidebook (panjang lorong, jarak antar-ruang) diisi -1 = BELUM DIUKUR, dan
// misi MENOLAK berangkat selama masih ada yang -1. Menebak panjang ruas
// berarti mengganti profil gait di tempat yang salah; di bibir turunan itu
// jatuh.
// ====================================================================
#include <Arduino.h>
#include "config.h"
#include "Navigation.h"   // Hexapod, LidarArray, ModeNav ikut lewat sini

// --- BELOK MASUK RUAS, relatif terhadap ruas SEBELUMNYA ---
//
// Arah tiap ruas TIDAK ditulis sebagai mata angin mutlak, melainkan sebagai
// seperempat putaran dari ruas sebelumnya, lalu ARAH MUTLAKNYA DIHITUNG
// (hitungArah()), satu-satunya tempat di firmware yang menghitungnya:
//
//     arah[i] = (arah[i-1] + belok[i]) % 4,   arah[0] = MISI_ARAH_BERANGKAT
//
// Dua alasan, dan keduanya soal mapping:
//
// 1. YANG SALAH SAAT MAPPING SELALU SATU TIKUNGAN, BUKAN SATU RUAS. Kalau
//    ternyata belokan di ujung lorong itu ke KIRI dan bukan ke KANAN, dengan
//    mata angin mutlak SEMUA ruas sesudahnya ikut salah dan harus diketik
//    ulang satu per satu. Dengan belok relatif, mengubah satu baris
//    memperbaiki seluruh sisa lintasan sendiri.
// 2. YANG BISA DILIHAT ORANG DI ARENA ADALAH TIKUNGAN, bukan mata angin.
//    "di ujung R-4 belok kanan" bisa dicocokkan dengan arena sambil berdiri
//    di sana; "ruas 8 menghadap TIMUR" tidak bisa, sampai dihitung dulu.
//
// Angka jangkarnya diambil dari misi adik tingkat (Mission.cpp Ver1_8) supaya
// hasilnya identik untuk ruas yang sudah pernah dia jalankan:
//     MISI_ARAH_AWAL    = 0 (UTARA)  -> ruas 0 BLK_LURUS  -> UTARA
//     MISI_ARAH_KORBAN1 = 3 (BARAT)  -> ruas 1 BLK_KIRI   -> BARAT
//     pivot balik ke lorong = UTARA  -> ruas 2 BLK_KANAN  -> UTARA
// Ketiganya keluar sendiri dari tabel, bukan ditulis ulang.
enum Belok : uint8_t {
    BLK_LURUS = 0,   // teruskan arah ruas sebelumnya
    BLK_KANAN = 1,   // seperempat searah jarum jam (U->T->S->B)
    BLK_BALIK = 2,   // setengah putaran
    BLK_KIRI  = 3    // seperempat berlawanan jarum jam
};

// Arah ruas PERTAMA. Ketentuan kontes: robot boleh diletakkan menghadap ke
// mana saja (arahnya ditentukan juri), lalu harus berangkat dari HOME -- jadi
// yang tetap bukan hadap awalnya, melainkan arah berangkatnya. Sama dengan
// MISI_ARAH_AWAL milik adik tingkat.
#define MISI_ARAH_BERANGKAT 0

// --- Bagaimana ruas ini dikemudikan menyamping ---
enum Kemudi : uint8_t {
    KMD_KANAN = 0,   // ikut dinding KANAN
    KMD_KIRI,        // ikut dinding KIRI
    KMD_TENGAH       // garis tengah lorong (selisih kiri-kanan)
};

// --- Profil gait yang dipakai sepanjang ruas ---
enum Profil : uint8_t {
    PRF_DATAR = 0,
    PRF_TANGGA,      // kaki +35 mm, badan +15 mm, siklus +400 ms
    PRF_MERUNDUK,    // langkah -15 mm, badan -20 mm, siklus +200 ms
    PRF_SEMPIT,      // radius berdiri -25 mm: lorong yang lebih sempit dari badan
    PRF_KAIL,        // R-9: kaki depan mengait ke depan-atas, belakang naik
    PRF_TANJAK       // R-9 lambat: langkah 75 mm, siklus 1300 ms, depan maju 60
};

// --- Apa yang mengakhiri ruas ini ---
enum Henti : uint8_t {
    HNT_ODO = 0,     // odometri gait: sudah menempuh `nilai` cm
    HNT_DEPAN,       // LiDAR depan <= `nilai` cm
    HNT_BELAKANG,    // LiDAR belakang >= titik nol + `nilai` cm (jarak tempuh)
    HNT_LANGSUNG,    // tidak berjalan sama sekali -- ruas ini hanya aksi
    HNT_SISI,        // GESER menyamping sampai dinding sisi = `nilai` cm
    HNT_MUNDUR,      // BERJALAN MUNDUR sampai LiDAR belakang = `nilai` cm
    HNT_PUNCAK       // NAIK lalu DATAR lagi (gyro), atau depan <= `nilai` cm
};

// HNT_PUNCAK vs HNT_DEPAN -- keduanya membaca LiDAR DEPAN, dan yang satu
// menambahkan syarat yang tidak dimiliki yang lain:
//
//   HNT_DEPAN   berhenti begitu depan <= `nilai`. Titik.
//   HNT_PUNCAK  berhenti kalau badan sudah MENDAKI lalu DATAR lagi, ATAU
//               depan <= `nilai`. Keduanya baru berlaku sesudah mendaki
//               terlihat.
//
// Ada karena odometri gait di tangga itu tebakan: ia menghitung siklus gait,
// bukan jarak tempuh, dan tiap siklus di anak tangga memindahkan robot sejauh
// yang tidak diketahui. Percobaan 6 Sep 2026 berhenti di 62 dari ~103 cm.
//
// GERBANG MENDAKI bukan hiasan. Tanpanya ruas berakhir SEKETIKA di kaki
// tangga, karena di sana badan memang datar; dan berkas LiDAR depan yang
// menyentuh muka anak tangga pertama akan mengakhirinya juga.
//
// `nilai` satuannya sama dengan HNT_DEPAN: cm ke dinding di depan. NOL berarti
// dinding depan DIMATIKAN -- gyro sendirian.
//
// R-9 memakai 0 sejak 18 Sep 2026. Dicoba dengan 40 lebih dulu, dan di
// tanjakan berkas depan sering mengenai MUKA ANAK TANGGA alih-alih dinding
// seberang: ruasnya berakhir di tengah pendakian. Gerbang mendaki tidak
// menolong -- begitu robot benar-benar mendaki, gerbang terbuka dan anak
// tangga berikutnya langsung memicu.
// Ambang gyro TIDAK di tabel melainkan di config.h (PUNCAK_*): ia milik robot
// dan pemasangan IMU-nya, bukan milik satu ruas.
//
// IMU BISU -> gerbang mendaki dilewati dan hanya LiDAR depan yang berlaku.
// Menunggu gyro yang tidak pernah datang berarti robot berjalan sampai batas
// waktu ruas, di tangga.
//
// TAPI DENGAN `nilai` 0, JALAN KELUAR ITU IKUT TERTUTUP: IMU bisu dan dinding
// dimatikan berarti TIDAK ADA yang mengakhiri ruas, dan yang tersisa cuma
// batas waktu ruas -- di tanjakan. tabelSiap() memperingatkannya saat 'm1',
// bukan membiarkannya ditemukan di arena.

// HNT_MUNDUR vs HNT_BELAKANG -- keduanya membaca sensor yang SAMA dan
// artinya berlawanan. Salah pilih berarti robot berjalan ke arah yang salah,
// jadi bedanya ditulis di sini sekali dan untuk seterusnya:
//
//   HNT_BELAKANG  `nilai` = JARAK TEMPUH. Robot MAJU, dinding START dipakai
//                 sebagai penggaris, berhenti pada bacaan `titik nol +
//                 nilai`. Titik nolnya dicatat saat ruas masuk.
//   HNT_MUNDUR    `nilai` = JARAK MUTLAK ke dinding belakang. Robot MUNDUR
//                 sampai bacaannya turun ke `nilai`. Tidak ada titik nol.
//
// Ini memakai mesin 'J<cm>' (NAV_SETEL_BLK) apa adanya, jadi seluruh
// penjaganya ikut: sasaran di bawah WALL_MIN_CM ditolak sebelum berangkat,
// sensor belakang yang mati di tengah gerakan menghentikan misi, dan
// MUNDUR_BATAS_MS membatasi lamanya. Tidak ada gerakan baru yang ditulis
// untuk ini -- kalau mundurnya bermasalah, perbaikannya di Navigation.
//
// Gunanya standoff per korban: "sejauh mungkin dari tembok yang dihadapi,
// mepet ke tembok seberang". Sebelum ini standoff itu hanya bisa diatur dari
// Raspi lewat 'J', jadi ia hidup di HUD dan bukan di tabel misi.

// HNT_SISI: ruas ini tidak MAJU sama sekali, ia bergeser menyamping sampai
// sensor sisinya membaca `nilai` cm. SISI MANA dibaca dari kolom `kemudi`
// (KMD_KANAN / KMD_KIRI) -- kolom itu memang menganggur di ruas yang tidak
// menyusuri dinding, dan menaruhnya di sana lebih baik daripada memakai tanda
// `nilai`, yang di seluruh tabel ini sudah punya arti lain: negatif = BELUM
// DIUKUR. KMD_TENGAH ditolak tabelSiap().
//
// Ini ada karena sumbu geser tidak bisa diminta dari luar: perpindahannya
// terkuantisasi satu langkah gait penuh, jadi "geser 10 cm" mustahil. Yang
// bisa dilakukan adalah menutup lupnya di dalam firmware -- lihat
// Navigation::ratakanMulai().

// --- Apa yang dikerjakan di UJUNG ruas, sesudah berhenti ---
//
// Pivot TIDAK ada di daftar ini dengan sengaja. Tiap ruas menyatakan `arah`
// yang dituju, dan mesinnya memutar badan sendiri saat masuk ruas yang
// arahnya berbeda dari arah sekarang. Versi lama menulis pivot sebagai state
// tersendiri di antara tiap pasang potongan, dan itulah separuh isi FSM-nya.
enum Aksi : uint8_t {
    AKS_TIDAK_ADA = 0,  // langsung sambung ke ruas berikutnya (tanpa berhenti)
    AKS_AMBIL,          // angkat korban dengan lengan `aksiA`
                        //   henti WAJIB HNT_DEPAN: gerbang jaraknyalah yang
                        //   menaruh korban di dalam amplop jangkauan lengan
    AKS_TARUH,          // taruh korban di safe zone
    AKS_KONFIRM         // berhenti, tunggu keputusan operator ('m2'/'m3')
};

struct Ruas {
    const char* nama;
    Belok       belok;         // belok masuk, RELATIF terhadap ruas sebelumnya
    Kemudi      kemudi;
    Profil      profil;
    bool        abaikanDepan;  // ruas ini berjalan BUTA ke depan (lihat catatan)
    Henti       henti;
    float       nilai;         // arti tergantung `henti`; <0 = BELUM DIUKUR
    Aksi        aksi;
    uint8_t     aksiA;         // lengan: ARM_DEPAN / ARM_BELAKANG

    // BELOK PECAHAN, derajat, DI ATAS `belok` yang seperempat-seperempat.
    // Sama dengan yang dilakukan 'O<derajat>' dari serial, tapi ia MUTLAK,
    // bukan relatif terhadap hadap robot saat itu: acuannya mata angin arena
    // hasil `belok`, ditambah sudut ini. Pivot yang relatif terhadap yaw
    // sekarang menumpuk galatnya sepanjang 33 ruas; yang ini tidak.
    //
    // MENUMPUK antar ruas. Sekali satu ruas menyerong 45 der, ruas BLK_LURUS
    // sesudahnya tetap 45 der -- "lurus" berarti meneruskan arah ruas tadi,
    // dan itu tetap benar saat arahnya bukan mata angin. Untuk kembali ke
    // mata angin murni, tulis lawannya secara tegas (-45).
    //
    // Kolom TERAKHIR dan bernilai 0 kalau tidak ditulis: inisialisasi agregat
    // C++ menolkan anggota yang tidak disebut, jadi 33 baris lama tidak perlu
    // disentuh sama sekali.
    float       putar = 0.0f;

    // CONDONG. true = tambahkan DUA fase sesudah badan digeser maju: jeda
    // konfirmasi mata selebar KORBAN_CONDONG_JEDA_MS, lalu pelurusan badan ke
    // heading ruas ini. Dipakai K-3 dan K-4, tempat korbannya tertutup
    // reruntuhan dan operator perlu melihat sendiri sebelum lengan turun.
    //
    // BUKAN saklar translasi maju. Sejak 16 Sep 2026 badan digeser maju di
    // SETIAP ruas AMBIL, sebesar Ruas::condongMm atau KORBAN_CONDONG_MM --
    // keluhan "kurang maju saat capit turun" sama di tiap korban, bukan cuma
    // di ruas yang korbannya tertutup. Kolom ini cuma membayar dua fase
    // tambahan, jadi ruas AMBIL biasa tidak ikut menanggungnya.
    //
    // Kolom TERAKHIR dan bernilai false kalau tidak ditulis, sama seperti
    // `putar`: inisialisasi agregat C++ menolkan anggota yang tidak disebut.
    bool        condong = false;

    // JAGA JARAK BELAKANG selama ruas GESER (HNT_SISI). true = selama menggeser,
    // robot ditarik mundur setiap LiDAR belakang membaca di atas
    // RATA_BLK_SASARAN_CM; false = perataan murni menyamping.
    //
    // Baku true, karena hanyut maju saat menggeser itu gejala umum, bukan
    // kekhususan satu ruas. Ditulis false pada ruas yang memang tidak punya
    // acuan di belakang: di sana bacaan belakang mengukur benda lain, dan
    // menariknya mundur memindahkan robot menjauhi sasaran ruas berikutnya.
    //
    // Tidak berarti apa-apa pada ruas selain HNT_SISI -- hanya rataUpdate()
    // yang membacanya.
    //
    // Kolom TERAKHIR, sama seperti `putar` dan `condong`. Nilai bakunya di
    // sini, jadi baris yang tidak menyebutnya tidak perlu disentuh.
    bool        jagaBelakang = true;

    // MUNDUR SEBELUM LENGAN TURUN, mm. Badan ditarik ke BELAKANG sebanyak ini
    // pada fase SIAP, lalu fase BADAN MAJU mendorongnya balik ke
    // KORBAN_CONDONG_MM sebelum capit menutup.
    //
    // Gunanya membebaskan busur turun. SIAP ke JEPIT itu ayunan siku 110 der
    // dengan jari-jari FOREARM_LENGTH, dan di tengah ayunan capit menjulur
    // LEBIH JAUH ke depan daripada di kedua ujungnya. Korban yang berdiri
    // lebih dekat dari nominal berada di dalam busur itu dan dipukul dari
    // atas. Menarik badan mundur memindahkan seluruh busur menjauh; dorongan
    // maju sesudahnya yang mempertemukan capit dengan korban, mendatar.
    //
    // PER KORBAN, karena jarak berdiri korban berbeda tiap kantong: K-3/K-4
    // punya ruang, K-5 ada tangga di belakangnya. 0 = tidak mundur sama
    // sekali, dan itu nilai bakunya -- baris lama tidak perlu disentuh.
    //
    // BATASNYA DIPERIKSA tabelSiap(): mundurMm sendiri tidak boleh melewati
    // BODY_MAX_TRANS_MM, dan mundurMm + KORBAN_CONDONG_MM harus muat dalam
    // SATU jatah LENGAN_JEDA_MS pada KORBAN_CONDONG_LAJU_MM_S. Fase yang
    // kehabisan jatah ditimpa fase berikutnya di tengah gerak, dan capit
    // menutup sebelum badan sampai.
    float       mundurMm = 0.0f;

    // MAJU SEBELUM CAPIT MENUTUP, mm. Pasangan maju dari `mundurMm`: fase
    // BADAN MAJU mendorong badan ke sini, lalu capit menutup.
    //
    // 0 = pakai KORBAN_CONDONG_MM (param Calib global). Itu nilai bakunya,
    // jadi 31 baris lama berjalan persis seperti sebelumnya. Isi kolom ini
    // pada korban yang butuh angka sendiri; kalau SEMUA korban sudah punya,
    // 'Qcondong.mm 0' lalu 'W' mematikan yang global tanpa flash ulang.
    //
    // Nol tidak bisa berarti "tidak maju sama sekali" -- untuk itu kosongkan
    // kolom ini DAN setel condong.mm ke 0. Ini harga dari nilai baku yang
    // harus 0 supaya baris lama tidak perlu disentuh, dan lebih murah
    // daripada menambah kolom bendera kedua.
    float       condongMm = 0.0f;
};

// Tabel lintasan. Definisinya di Misi.cpp; jumlahnya dibuka supaya .ino bisa
// memvalidasi indeks perintah 'm7 <idx> <cm>' tanpa menebak.
extern const Ruas RUAS[];
extern const uint8_t RUAS_N;

// Kapasitas salinan panjang ruas yang bisa disetel operator (_cm[]). Tabelnya
// ada di flash dan array ini di RAM, jadi keduanya tidak bisa saling
// menyesuaikan sendiri -- penjaganya static_assert di Misi.cpp. Tanpa itu,
// menambah baris ke-25 ke tabel akan menulis melewati ujung array tanpa satu
// pun keluhan, dan yang tertimpa adalah anggota kelas di sebelahnya.
// Cuma ukuran dua array RAM (_arah[] uint8 + _cm[] float) -- TIDAK menyentuh
// EEPROM, jadi menaikkannya tidak membuang kalibrasi apa pun. 28 -> 36 pada
// 8 Sep 2026 saat tabel tumbuh ke 34 baris; sisanya ruang tumbuh.
#define RUAS_MAKS 34

enum StatMisi : uint8_t {
    MISI_DIAM = 0,
    MISI_JALAN,        // menjalankan RUAS[_i]
    MISI_PIVOT,        // aksi ujung: sedang memutar badan
    MISI_SETEL,        // profil gait baru dipasang -- menunggu badan tenang
    MISI_LENGAN,       // aksi ujung: sekuens lengan (pose tetap, buta)
    MISI_KONFIRM,      // aksi ujung: menunggu 'm2'/'m3'
    // Capit sudah selesai, tapi Raspi mungkin MASIH memegang kaki untuk
    // menahan posisi. State sendiri, bukan MISI_KONFIRM yang dipakai ulang:
    // KONFIRM sudah punya dua ujung (ruas AKS_KONFIRM dan parkir vision), dan
    // ujung ketiga di satu state adalah tempat paling mudah untuk salah.
    MISI_LEPAS,        // aksi ujung: menunggu 'm9' -- Raspi melepas kaki
    MISI_UKUR,         // MODE UKUR: jalan tanpa syarat henti, operator yang menyetop
    MISI_SELESAI,
    MISI_GAGAL
};

class Misi {
public:
    Misi(Hexapod& robot, Navigation& nav, LidarArray& lidar);

    void mulai();                     // 'm1'  -- selalu dari ruas 0
    // 'm4 <idx> [sampai]' -- jalankan SEBAGIAN lintasan.
    //
    // Batas akhir ada karena tabel yang belum lengkap di UJUNG lintasan dulu
    // memblokir pengujian bagian yang datanya sudah kuat: tabelSiap() menyapu
    // seluruh tabel, jadi satu ruas -1 di ruas 18 membuat robot menolak
    // melangkah dari HOME. Sekarang yang diperiksa hanya ruas yang benar-benar
    // akan dijalani, dan itu memang semantik yang lebih tepat.
    void mulaiDari(uint8_t idx, uint8_t sampai = 255);

    // 'm6 <idx>' -- MODE UKUR. Menjalankan satu ruas dengan profil, kemudi, dan
    // arah miliknya sendiri, TANPA syarat henti: operator yang menghentikan di
    // ujung ruas ('s'/Enter/'m0'), lalu robot mencetak sendiri berapa cm yang
    // ditempuhnya. Itu angka yang dicari saat mapping.
    //
    // Sengaja TIDAK lewat tabelSiap(): ruas yang mau diukur justru yang
    // panjangnya masih -1, jadi menuntut tabel lengkap lebih dulu membuat mode
    // ini mustahil dipakai untuk tujuannya sendiri. Yang tetap diperiksa cuma
    // syarat yang bikin robot celaka: kompas, kalibrasi pivot, servo.
    void ukur(uint8_t idx);
    void batal(const char* alasan);   // 'm0', juga 'x' / 's' / Enter
    void update();                    // tiap loop, SEBELUM nav.navUpdate()
    void status();                    // 'm'
    void tabel();                     // 'm4' tanpa argumen: cetak seluruh tabel
    void jawab(bool korban);          // 'm2' / 'm3'
    void setRuasCm(uint8_t idx, float cm);   // 'm7 <idx> <cm>'

    // 'aa' / 'at' -- jalankan SEKUENS LENGAN saja, tanpa ruas di belakangnya.
    // Dipakai untuk menyetel pose korban di meja: sekuens yang sama persis
    // yang akan dijalankan misi, dengan jeda antar pose yang sama.
    //
    // Lewat Misi dan bukan lewat loop sendiri di .ino karena sekuens ini
    // BUTA dan berjeda: ia butuh state, penjaga servo, dan rem 'm0'/'s' yang
    // semuanya sudah ada di sini. Saklar LENGAN_KORBAN_AKTIF sengaja TIDAK
    // berlaku -- yang mengetik perintahnya memang sedang menyetel lengan.
    void ujiLengan(bool ambil);

    // SERAH-TERIMA KENDALI DENGAN RASPI. Dua perintah, satu di tiap ujung.
    //
    // Masalah yang ditutupnya: ruasSehat() menggagalkan misi begitu navMode()
    // berubah, dan itu termasuk perubahan yang datang dari serial ('o', 'O',
    // ...). Jadi selama MISI_JALAN, Raspi TIDAK BISA menengahkan badan tanpa
    // mematikan misi. Satu-satunya tempat yang aman adalah state yang tidak
    // memanggil ruasSehat() dan yang navigasinya sudah diam -- dan itu
    // persis MISI_KONFIRM / MISI_LEPAS.
    //
    // Alurnya, satu penguasa pada satu waktu:
    //   Teensy  #KORBAN AMBIL <ruas>  -> parkir, kendali kaki MILIK RASPI
    //   Raspi   menengahkan, tahan, badan TIDAK dinetralkan
    //   Raspi   'm2'                  -> kendali kaki KEMBALI ke Teensy
    //   Teensy  sekuens capit
    //   Teensy  #LEPAS <ruas>         -> tanya: masih memegang kaki?
    //   Raspi   'm9'                  -> tidak lagi; misi boleh lanjut
    void setTungguVision(int mode);   // 'm8' / 'm8 1' / 'm8 0'
    bool tungguVision() const { return _tungguVision; }
    void lepasKendali();              // 'm9' -- Raspi melepas kaki

    StatMisi stat() const { return _stat; }

    // Dibuka untuk OLED: layar di badan robot harus bisa menyebut ruas dan
    // jam kontes tanpa laptop. Keduanya baca-saja.
    uint8_t  ruasKini() const { return _i; }
    uint32_t waktuMisiDetik() const {
        return _tMisi ? ((millis() - _tMisi) / 1000UL) : 0UL;
    }

    // AKTIF = ada sesuatu yang harus dihentikan kalau operator menekan rem.
    // SATU definisi untuk seluruh kelas: tiap state selain DIAM/SELESAI/GAGAL.
    // Versi lama menuliskan daftarnya satu per satu dan lupa lima state, jadi
    // 'm0' di tengah lantai pecah tidak menghentikan apa pun. Ditulis begini
    // supaya state baru ikut terhitung TANPA ada yang perlu ingat.
    bool berjalan() const {
        return _stat != MISI_DIAM && _stat != MISI_SELESAI && _stat != MISI_GAGAL;
    }

private:
    Hexapod&    _robot;
    Navigation& _nav;
    LidarArray& _lidar;

    StatMisi _stat = MISI_DIAM;
    uint8_t  _i    = 0;          // indeks ruas yang sedang dijalani
    uint8_t  _iAkhir = 255;      // ruas TERAKHIR yang akan dijalani (inklusif)
    uint8_t  _arah[RUAS_MAKS];   // mata angin MUTLAK tiap ruas, hasil hitungArah()
    float    _serong[RUAS_MAKS]; // simpangan der dari mata angin itu, menumpuk
    uint32_t _t0   = 0;          // saat masuk state (batas waktu ruas)
    uint32_t _tMisi = 0;         // saat 'm1' ditekan (jam kontes 5 menit)

    // Panjang ruas yang bisa disetel operator. Disalin dari RUAS[].nilai saat
    // menyala supaya tabel di flash tetap jadi acuan yang tidak berubah.
    float    _cm[RUAS_MAKS];

    float    _ruasAwal = 0.0f;   // odometer saat ruas dimulai, cm
    float    _blkAwal  = -1.0f;  // bacaan LiDAR belakang di titik nol, cm
    uint8_t  _n        = 0;      // sampel berturut-turut yang memenuhi syarat
    uint32_t _stempel  = 0;      // stempel sampel yang terakhir dihitung
    uint32_t _serongT0 = 0;      // sejak kapan heading keluar toleransi (0 = tidak)

    // HNT_PUNCAK. Ketiganya dinolkan tiap ruas mulai, di ruasJalan().
    //
    // _pitchAwal ACUAN ruas ini, bukan nol: pitchDeg() mentah dan badan sudah
    // dimiringkan profil TANJAK. NAN = ruas ini tidak memakainya, atau IMU
    // bisu waktu ruas mulai.
    float    _pitchAwal    = NAN;
    bool     _naikTerlihat = false;   // sudah pernah melewati PUNCAK_NAIK_DEG
    uint32_t _datarT0      = 0;       // sejak kapan datar lagi (0 = belum)
    uint32_t _tenangT0 = 0;      // sejak kapan ramp profil selesai (0 = belum)
    uint8_t  _pivotUlang = 0;    // berapa kali pivot masuk ruas ini diulang
    // Pivot masuk ruas ini BARU SAJA selesai, dan LiDAR belum dibaca sejak
    // itu. Dipakai sekali lalu dinolkan -- lihat pasangProfil().
    bool     _pivotBaru = false;
    // Yaw saat ruas berhenti, untuk mengukur serong yang muncul sesudahnya.
    // NAN = belum ada ruas yang berhenti sejak menyala.
    float    _yawUjung = NAN;
    uint8_t  _langkah  = 0;      // sub-langkah sekuens lengan
    // Sekuens lengan sedang dijalankan lepas dari tabel ('aa'/'at'), jadi
    // MISI_LENGAN tidak boleh membaca RUAS[_i] maupun lanjut ke ruas berikut.
    bool     _uji      = false;
    bool     _ujiAmbil = false;
    // MISI_UKUR selama 'm6' berjalan, MISI_DIAM selebihnya. Dipakai ruasMasuk()
    // untuk memilih state akhir: jalur masuk ruasnya sama persis, yang berbeda
    // cuma apakah ruasnya boleh berhenti sendiri.
    StatMisi _ukurMulai = MISI_DIAM;
    // SATU PER LENGAN. Dulu ini satu bool untuk seluruh robot, padahal
    // ARM_DEPAN dan ARM_BELAKANG memang dua capit yang berdiri sendiri --
    // modelnya tidak sanggup menyatakan "depan penuh, belakang kosong", dan
    // tabel yang mengambil dua korban berturut-turut terlihat sah olehnya.
    bool     _korban[2] = { false, false };

    // BAWAANNYA NYALA. Parkirnya berbatas waktu (MISI_VISI_BATAS_MS) dan
    // habisnya waktu tidak menggagalkan apa pun -- sekuens tetap jalan
    // memakai sudut tetap, persis perilaku tanpa vision. Jadi NYALA berarti
    // "coba pakai kamera; kalau tidak ada jawaban, kerjakan cara lama".
    bool     _tungguVision = true;
    // MISI_KONFIRM dipakai DUA hal dengan ujung berbeda: ruas AKS_KONFIRM
    // (-> ruasBerikut) dan parkir vision (-> MISI_LENGAN). Ini yang
    // membedakannya. Tanpa pembeda ini 'm2' akan MELOMPATI pengambilannya dan
    // robot berjalan ke ruas berikutnya dengan capit kosong.
    // DETEKSI DI RASPI SEDANG JALAN? Dibuka '#KORBAN', ditutup '#LEPAS'.
    // Bukan salinan _parkirVision: jendela vision dibuka SEBELUM parkir dan
    // masih terbuka sesudah parkir dilepas, jadi satu bendera tidak bisa
    // menjawab dua pertanyaan. Yang ini dipakai batal() dan gagal() untuk
    // tahu apakah Raspi masih perlu diberi tahu supaya berhenti mendeteksi.
    bool     _visiJalan   = false;
    bool     _parkirVision = false;

    const char* _sebab = nullptr;

    const Ruas& r() const { return RUAS[_i]; }
    float       cmKini() const { return _cm[_i]; }
    uint32_t    lewat() const { return millis() - _t0; }

    void hitungArah();
    float headingRuas(uint8_t i) const;

public:
    // --- ARENA CERMIN -------------------------------------------------------
    //
    // Lapangan bisa dipasang sebagai cerminnya: yang di kiri jadi di kanan.
    // Saklarnya 'arena.mirror' (K_ARENA_MIRROR), dibalik tombol D3 atau
    // 'Qarena.mirror 1'.
    //
    // SATU PINTU, bukan tabel kedua. Menyalin RUAS[] jadi versi cermin berarti
    // 30 baris yang harus disunting dua kali seumur hidup proyek, dan yang
    // kedua pasti tertinggal. Ketiga pengakses ini yang dipakai SELURUH kode
    // yang membaca kolom berarah, jadi mencerminkan misi cuma membalik saklar.
    //
    // Yang dicerminkan hanya TIGA kolom, dan itu cukup:
    //   belok   BLK_KIRI <-> BLK_KANAN. LURUS dan BALIK tidak berubah, dan
    //           karena hitungArah() menumpuk kolom ini, seluruh mata angin
    //           misi ikut tercermin sendiri (TIMUR <-> BARAT, UTARA dan
    //           SELATAN tetap).
    //   kemudi  KMD_KIRI <-> KMD_KANAN. KMD_TENGAH tidak berarah.
    //   putar   dinegasikan; serong 45 der ke kiri jadi 45 der ke kanan.
    //
    // Yang TIDAK dicerminkan: jarak, profil, aksi, lengan. Lengan cuma satu di
    // depan, dan jarak tidak punya sisi.
    static bool arenaCermin();
    Belok  belokRuas(uint8_t i)  const;
    Kemudi kemudiRuas(uint8_t i) const;
    float  putarRuas(uint8_t i)  const;

private:
    bool  ruasSerong(uint8_t i) const;            // belok relatif -> mata angin mutlak tiap ruas
    float selisihYaw() const;
    void masuk(StatMisi s);
    void gagal(const char* sebab);
    // Tutup jendela deteksi Raspi ('#LEPAS'). Aman dipanggil berkali-kali.
    void lepasVisi();
    bool siapJalan(uint8_t idx, uint8_t sampai);  // kompas, pivot, sensor
    float koreksiYawRuas() const;
    bool tabelSiap(uint8_t dari, uint8_t sampai);  // ruas yang akan dijalani saja
    void ruasMasuk();             // pivot dulu bila perlu, lalu jalankan RUAS[_i]
    void ruasBerangkat();         // pasangProfil() -> tunggu bila perlu -> ruasJalan()
    bool pasangProfil();          // pasang profil gait; true = ruas ini harus menunggu
    bool ruasJalan();             // profil + kemudi + navMulai untuk RUAS[_i]
    bool ruasSehat();             // penjaga: nav masih milik kita & arah benar
    bool ruasSelesai();           // syarat henti RUAS[_i] terpenuhi?
    // berpoin=false: ruas DILEWATI, bukan diselesaikan. Poin hanya dicatat
    // untuk ruas yang benar-benar dikerjakan.
    void ruasBerikut(bool berpoin = true);

    // LEWATI ruas ini dan teruskan misi. Diminta R2C 18 Sep 2026: di lomba,
    // ruas yang gagal lebih baik ditinggalkan daripada menghentikan seluruh
    // lari. Sebab 'lunak' -- pivot meleset, ruas kehabisan waktu, heading
    // hilang, perataan ditolak -- semuanya lewat sini sekarang.
    //
    // Yang TETAP membatalkan misi cuma tiga: servo lemas, LiDAR mati, dan
    // waktu kontes habis. Ketiganya berarti robot tidak bisa lagi dipercaya
    // bergerak, bukan sekadar satu ruas yang meleset.
    void lewati(const char* sebab);

    // Ada LiDAR yang BENAR-BENAR tidak merespons? Bukan 'jauh', bukan bacaan
    // buruk -- tidak menjawab sama sekali.
    bool lidarMati();
    void cetakRuas(uint8_t idx);
};

#endif
