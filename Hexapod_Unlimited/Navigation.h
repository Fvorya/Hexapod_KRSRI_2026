#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "Imu.h"
#include "Hexapod.h"
#include "LidarArray.h"

// Mode navigasi otonom. Sengaja hanya "ikut dinding kiri/kanan" -- itu yang
// dibutuhkan di arena berlorong; penjelajahan bebas butuh peta, bukan sensor.
enum ModeNav : uint8_t {
    NAV_DIAM = 0,
    NAV_DINDING_KIRI,        // ikut dinding murni (tanpa acuan kompas)
    NAV_DINDING_KANAN,
    NAV_ARENA_KIRI,          // ikut dinding + terkunci heading arena
    NAV_ARENA_KANAN,
    NAV_PIVOT,               // berputar di tempat menuju satu heading ('o'/'O')
    NAV_RATA,                // geser menyamping sampai dinding sisi mencapai jarak
    NAV_SETEL_BLK            // maju/mundur sampai LiDAR belakang mencapai jarak
};

// Fase internal. FASE_JALAN milik mode arena, FASE_PIVOT/FASE_SETTLE milik
// NAV_PIVOT. Modenya saling eksklusif, jadi _fase, _tPivot dan _diamSejak
// dipakai bersama oleh keduanya.
//
// FASE_BELOK dihapus 6 Sep 2026 bersama belok-otomatis mode arena: lintasan
// datang dari tabel misi, jadi navigasi tidak pernah lagi memilih tikungan
// sendiri, dan fase untuk melakukannya tidak punya jalan masuk.
enum FaseNav : uint8_t {
    FASE_JALAN = 0,   // arena : menyusuri dinding
    FASE_PIVOT,       // pivot : masih berputar menuju _pivotTarget
    FASE_SETTLE       // pivot : perintah putar sudah nol, menunggu kaki diam
};

class Navigation {
public:
    // Menerima referensi dari objek IMU, Hexapod, dan LiDAR secara langsung
    Navigation(Imu& imuRef, Hexapod& robotRef, LidarArray& lidarRef);

    void begin();

    // --- 5. Navigasi otonom (NON-BLOKIR) ---
    // navUpdate() dipanggil tiap loop dan hanya menghitung satu langkah, jadi
    // perintah serial tetap terproses dan 'x' selalu bisa menyela. Ia juga
    // yang menjalankan NAV_PIVOT, sehingga pivot manual ikut non-blokir.
    void navMulai(ModeNav m);
    void navBerhenti(const char* alasan = nullptr);
    void navUpdate();
    void navStatus();
    ModeNav navMode() const { return _mode; }

    // REM JARAK. Menghentikan robot sesudah menempuh jarak tertentu, di mode
    // gerak APA PUN -- termasuk 'w' manual, karena pemeriksaannya duduk di
    // atas jalan keluar NAV_DIAM di navUpdate(). Dipakai untuk mengukur slip
    // gait: jalankan sejauh N cm menurut odometri, lalu ukur dengan meteran.
    //
    // Rem HANYA menolkan; ia tidak pernah menulis vektor gerak, jadi doktrin
    // satu-penulis yang dijaga navBerhenti() tetap utuh.
    void  remJarakPasang(float cm);   // nolkan jarak lalu pasang; cm <= 0 ditolak
    void  remJarakLepas();
    bool  remJarakAda() const     { return _remJarakCm > 0.0f; }
    float remJarakSasaran() const { return _remJarakCm; }

    // SENSOR DEPAN DIABAIKAN pada ruas ini.
    //
    // Di bidang miring TURUN berkas sensor depan menembak LANTAI, bukan
    // halangan. Badan ikut miring, jadi berkasnya sejajar permukaan turunan
    // dan berakhir di lantai datar di bawah: pada tinggi dudukan ~10 cm dan
    // kemiringan 14 der, itu terbaca ~10/tan(14) = 40 cm, lalu MENGECIL
    // sepanjang robot turun. Navigasi membacanya sebagai halangan yang
    // mendekat: melambat di 50 cm, lalu berbelok di 20 cm.
    //
    // Saklar ini mematikan KETIGA aturan depan (mati / halangan / melambat)
    // untuk satu ruas. Konsekuensinya nyata: robot berjalan buta ke depan,
    // jadi ruasnya WAJIB dibatasi hal lain -- odometri misi, atau rem 'D'.
    // navBerhenti() selalu mengembalikannya ke aman, sehingga tidak ada
    // jalan untuk meninggalkannya menyala tanpa sengaja.
    void abaikanDepan(bool ya);
    bool depanDiabaikan() const { return _abaikanDepan; }

    // KEMUDI DINDING: PD atau SAMAR (fuzzy). Alat banding, bukan setelan --
    // karena itu RAM saja, tidak masuk blob Calib. Menambah parameter ke sana
    // mengubah ukuran blob, dan blob lama jadi tidak valid.
    //
    // Yang ditukar HANYA rumus di pita PD. Pita "terlalu dekat" tetap sama
    // untuk keduanya: ia penjaga geometri yang terikat LIDAR_MIN_CM, bukan
    // hukum kendali, dan membandingkan dua hal sekaligus tidak menjawab apa pun.
    // MODE KEMUDI DINDING, dua saklar bebas dalam satu angka:
    //   bit 0 = hukum kendali : 0 PD, 1 SAMAR (fuzzy)
    //   bit 1 = sumber turunan: 0 selisih waktu, 1 SUDUT sepasang sensor
    // Jadi N0..N3 adalah matriks 2x2 yang lengkap untuk membandingkan.
    void setKemudiMode(uint8_t m);
    uint8_t kemudiMode() const { return (uint8_t)((_wallSamar ? 1 : 0) | (_wallSudut ? 2 : 0)); }
    bool kemudiSamar() const { return _wallSamar; }
    bool kemudiSudut() const { return _wallSudut; }

    // Turunan error dinding yang DIHITUNG DARI SUDUT, cm/detik. NAN bila
    // sudutnya tidak tersedia. Robot yang maju v dengan hidung menyerong phi
    // mendekat dinding menutup jaraknya sebesar v*sin(phi) -- jadi turunan
    // yang selama ini diselisihkan terhadap waktu bisa dibaca langsung,
    // tanpa menunggu dua sampel LiDAR berurutan.
    float turunanDariSudut(bool kiri);

    // MENENGAH: kemudi dari SELISIH kedua sisi, bukan dari jarak ke satu
    // dinding. err = (kanan - kiri)/2, jadi setpoint-nya adalah garis tengah
    // lorong berapa pun lebarnya -- wall.setpoint tidak dipakai.
    //
    // Gunanya di lorong sempit: ikut-dinding memilih satu sisi dan membiarkan
    // sisi lain mengurus dirinya sendiri, sehingga di lorong 45 cm robot bisa
    // duduk benar terhadap dinding yang diikuti sambil menggesek dinding
    // seberang. Menengah mengikat keduanya sekaligus.
    //
    // Jatuh kembali ke ikut-dinding biasa begitu salah satu sisi tidak memberi
    // jarak -- di mulut simpangan itu justru yang benar.
    // JARAK SISI dari pasangan sensor. Memakai dudukan DEPAN seperti biasa;
    // dudukan belakang hanya dipakai sebagai cadangan kalau yang depan diam.
    //
    // SEMPAT dipakai juga untuk menolak penghalang -- dugaan bahwa kaki tengah
    // menyeberangi berkas. Data arena membantahnya: selisih ch3/ch4 ternyata
    // TETAP (~2 cm, itu bias pemasangan), bukan berdenyut di frekuensi
    // langkah. Penolaknya dibuang; yang tersisa cuma cadangan sensor mati.
    float jarakSisi(bool kiri);

    void setTengah(bool ya);
    bool tengah() const { return _tengah; }

    // KOREKSI SAMBIL BERHENTI. Diminta R2C 17 Sep 2026 untuk R-9.
    //
    // Menyala: begitu sudut atau jarak dinding keluar ambang, robot berhenti
    // MAJU, memutar badan sampai sejajar, baru berjalan lagi. Mati: kemudi
    // mengoreksi terus sambil melangkah, seperti seluruh sisa firmware ini.
    //
    // Bawaannya MATI, dan hanya 'U' yang menyalakannya. Di tanjakan, kaki ayun
    // yang mendarat sesudah badan berputar mendarat di tempat yang salah; di
    // lantai datar itu tidak jadi soal, dan berhenti tiap beberapa langkah
    // cuma membuang waktu lomba.
    //
    // DILEPAS navBerhenti(), sama seperti abaikanDepan dan kunciHeading: ia
    // milik SATU perjalanan 'U'. 'U' memasangnya sesudah navMulai(), jadi
    // perjalanannya sendiri selamat.
    void koreksiDiam(bool ya);

    // BERHENTI DI PUNCAK TANJAKAN, dari gyro. Dipasang 'U'.
    //
    // Rem jarak tetap terpasang dan tetap berlaku -- yang mana lebih dulu.
    // Gyro tahu KAPAN robot sampai; odometri gait cuma menghitung siklus, dan
    // di anak tangga tiap siklus memindahkan robot sejauh yang tidak
    // diketahui. Diukur 18 Sep 2026: sampai atas pada ~230 cm odometri untuk
    // tangga yang panjang miringnya 103 cm.
    void hentiPuncak(bool ya);
    bool koreksiDiamAktif() const { return _koreksiDiam; }
    bool sedangKoreksi()    const { return _sedangKoreksi; }

    // SUDUT BADAN TERHADAP DINDING, dari SEPASANG sensor di sisi yang sama.
    //
    //     sudut = atan2((d_belakang - d_depan) - bias, WALL_BASE_CM)
    //
    // + = hidung MENYERONG MENDEKAT dinding itu. Berbeda dari _errTurunan yang
    // dipakai PD: yang ini seketika dari satu tarikan bacaan, bukan hasil
    // menyelisihkan satu sensor terhadap waktu.
    //
    // NAN bila salah satu sensor tidak memberi jarak yang bisa dipakai, atau
    // bila kedua sampelnya terlalu jauh berjarak waktu -- pada robot yang
    // berjalan, dua bacaan dari saat yang berbeda bukan satu segitiga.
    float sudutDinding(bool kiri);
    float bedaSisi(bool kiri);        // (d_belakang - d_depan) mentah, cm
    void  kalibrasiSudut();           // robot SEJAJAR lorong -> catat bias
    float biasSisi(bool kiri) const { return kiri ? _biasKiri : _biasKanan; }
    void  sudutTabel();               // cetak sudut & bias kedua sisi

    // --- 1. Kompas Arena ---
    // return false = TIDAK jadi dicatat (sebaran heading terlalu lebar, atau
    // IMU belum memberi data). Pemanggil yang menghitung sendiri arah
    // berikutnya WAJIB memeriksanya: maju ke arah berikutnya sesudah
    // pencatatan yang gagal menaruh seluruh sisa tabel di slot yang salah,
    // dan tabel yang tergeser satu slot terlihat seperti kompas yang acak.
    bool kompasCatat(uint8_t arah); // 0=U, 1=T, 2=S, 3=B
    void kompasSimpan();
    bool kompasMuat(bool cerewet = true);
    void kompasTabel();
    bool kompasLengkap() const;   // keempat arah sudah dicatat?

    // Arah arena terdekat dari sebuah heading. return -1 bila belum ada satu
    // pun arah dicatat. selisihDeg = simpangan ke arah itu (-180..180).
    // Yaw IMU mentah, derajat. Dibuka supaya Misi bisa MENGUKUR pergeseran
    // badan saat ruas berhenti -- robot menyerong sedikit ke kanan tiap kali
    // ruas AMBIL selesai, dan menebak sumbernya lebih mahal daripada
    // mencetaknya. Bukan untuk kemudi: yang menyetir tetap Navigation.
    float yawKini() { return _imu.yawDeg(); }

    // PITCH IMU MENTAH, derajat, plus penanda apakah IMU sudah bicara.
    // Dibuka untuk HNT_PUNCAK: ruas tangga berakhir saat badan kembali datar,
    // dan yang tahu itu cuma IMU.
    //
    // MENTAH, dan itu disengaja. tare() tidak pernah dipanggil di seluruh
    // program, jadi pitchDeg() membawa simpangan pemasangan papan IMU apa
    // adanya -- di robot ini roll saja terbaca 176 der waktu robot berdiri
    // tegak. Pemakainya WAJIB membandingkan terhadap acuannya sendiri, bukan
    // terhadap nol.
    float pitchKini()  { return _imu.pitchDeg(); }
    bool  imuBicara()  { return _imu.hasData(); }

    int8_t arahTerdekat(float yawDeg, float& selisihDeg) const;

    // Apakah robot SEDANG menghadap arah arena ini (dalam HEADING_TOLERANCE_DEG)?
    // Dipakai misi untuk memeriksa hasil pivot awalnya: pivot yang SELESAI dan
    // pivot yang GAGAL sama-sama berakhir di NAV_DIAM, jadi satu-satunya yang
    // membedakan keduanya dari luar adalah heading akhirnya.
    bool diArah(uint8_t arah) const;

    // Heading tercatat satu mata angin, der. NAN bila belum dicatat. Dipakai
    // misi untuk membangun heading yang BUKAN mata angin (mata angin + serong).
    float headingArah(uint8_t arah) const;

    // KUNCI HEADING MUTLAK untuk mode arena. NAN = kembali memilih mata angin
    // terdekat sendiri, yaitu perilaku lama.
    //
    // Ada karena mode arena mengunci ke slot mata angin, dan slot itu dipilih
    // dari yaw saat navMulai(). Pada ruas yang menyerong 45 der, yaw berjarak
    // sama dari DUA mata angin: pilihannya jadi lemparan koin, dan yang salah
    // mengunci badan 90 der dari yang dimaksud. Ruas yang tahu heading-nya
    // sendiri menyebutkannya di sini, jadi tidak ada yang perlu ditebak.
    void  kunciHeading(float der);
    float headingKunci() const { return _headKunci; }
    float headingTerkunci() const;

    // Heading di ANTARA dua mata angin arena, dari kompas yang TERCATAT --
    // bukan dari asumsi bahwa keempatnya berjarak 90 der sempurna di IMU.
    // bagian 0 = tepat di a, 1 = tepat di b, 0,5 = separuh jalan.
    // NAN bila salah satu arah belum dicatat.
    float headingAntara(uint8_t a, uint8_t b, float bagian) const;
    bool  diHeading(float target) const;

    // Simpangan heading sekarang terhadap satu arah arena, -180..180.
    // NAN bila arah itu belum dicatat atau IMU belum punya data. Dipakai misi
    // untuk menilai odometri: simpang theta memendekkan jarak sesungguhnya
    // dengan faktor cos(theta), jadi yang penting besarnya, bukan lulus/tidak
    // terhadap satu ambang sempit milik pivot.
    float simpangArah(uint8_t arah) const;

    // Sama, tapi terhadap heading MUTLAK apa pun -- termasuk yang menyerong
    // dari mata angin. NAN bila target NAN atau IMU belum punya data.
    float simpangHeading(float target) const;

    // Arah arena yang SEDANG dituju mode arena (0..3), -1 bila tidak ada.
    // Berubahnya nilai ini berarti navigasi memilih mata angin lain -- 90 der
    // sekaligus, bukan goyangan.
    int8_t arahDituju() const { return _arahKini; }
    const char* namaArah(uint8_t i) const { return (i < 4) ? _arahNama[i] : "?"; }

    // --- 2. Pivot Tertutup (PD) -- NON-BLOKIR ---
    // pivotKe() hanya MEMULAI pivot lalu langsung kembali; kerjanya dijalankan
    // navUpdate() satu langkah per pemanggilan, sama seperti ikut-dinding.
    // Konsekuensinya: perintah serial tetap terproses selama robot berputar,
    // dan 's' / 'x' / Enter membatalkannya lewat navBerhenti() yang sama --
    // tidak perlu lagi jalan keluar "tekan Enter" khusus di dalam pivot.
    void pivotKe(float targetYaw);
    void pivotKompas(uint8_t arah);
    void pivotRelatif(float der);
    bool pivotSedangJalan() const { return _mode == NAV_PIVOT; }

    // --- RATAKAN KE DINDING SAMPING ---
    //
    // Sumbu geser sudah lama ada di HexaGait, tapi tidak pernah ada yang
    // MENUTUP LUPNYA. Akibatnya terukur di arena 6 Sep 2026: 'w 0 -0.6 0.5'
    // dan 'w 0 -0.6 3' sama-sama memindahkan badan ~32-36 cm, dan dua
    // perintah IDENTIK berbeda tiga kali lipat -- perpindahannya
    // terkuantisasi satu langkah gait penuh, jadi permintaan "geser 10 cm"
    // mustahil dipenuhi dari luar.
    //
    // Yang memperbaikinya bukan semburan yang lebih pendek melainkan SYARAT
    // HENTI YANG DIBACA TIAP TICK. Sama seperti rem jarak, hanya penggarisnya
    // LiDAR samping, bukan odometri.
    //
    // Tinggal di sini dan bukan di .ino supaya perintah 'V' dan ruas HNT_SISI
    // memakai SATU implementasi: dua salinan berarti dua tempat untuk salah,
    // dan yang kedua adalah yang lupa diperbaiki. Sekaligus mempertahankan
    // aturan pokok misi -- hanya Navigation yang menulis vektor gerak.
    //
    // return false = DITOLAK dan robot tidak bergerak sama sekali; sebabnya
    // sudah tercetak. Sasaran yang SUDAH tercapai juga return true tanpa
    // bergerak, jadi pemanggil cukup memeriksa ratakanSedangJalan().
    //
    // jagaBelakang false = jarak belakang TIDAK dijaga selama perataan. Dipakai
    // ruas yang tidak punya acuan di belakang; lihat Ruas::jagaBelakang.
    bool ratakanMulai(bool kiri, int cm, bool jagaBelakang = true);

    // GESER SEJAUH <cm>, diukur ODOMETRI, tanpa penggaris dinding sama sekali.
    //
    // Saudara ratakanMulai() dan memakai seluruh mesin yang sama -- laju,
    // penjaga halangan di arah geser, batas waktu, kompensasi hanyut. Yang
    // berbeda cuma SYARAT BERHENTINYA: jarak lateral yang sudah ditempuh,
    // bukan bacaan sensor.
    //
    // Ada karena di R-11 sisi yang dituju adalah jurang, dan sisi seberangnya
    // dinding yang belum tentu terbaca. ratakanMulai() GAGAL TERTUTUP kalau
    // penggarisnya bisu -- benar untuk perataan, tapi di sana artinya ruasnya
    // tidak jalan sama sekali.
    //
    // Harganya: odometri geser memakai skala slip yang sama dengan odometri
    // maju, dan skala itu dikalibrasi untuk MAJU. Jarak geser karena itu
    // kurang teliti daripada bacaan dinding. Pakai ini hanya kalau dindingnya
    // memang tidak bisa diandalkan.
    bool geserMulai(bool kiri, int cm, bool jagaBelakang = false);

    // SETEL JARAK BELAKANG, berumpan-balik, DUA ARAH.
    //
    // Sasarannya bacaan LiDAR BELAKANG. Bacaan DI ATAS sasaran berarti robot
    // terlalu jauh dari dinding belakang, jadi ia MUNDUR; bacaan di bawah
    // sasaran berarti terlalu dekat, jadi ia MAJU. Arahnya dipilih sendiri
    // dari bacaan, bukan dari tanda argumen -- operator cukup menyebut jarak
    // yang diinginkan.
    //
    // Perintahnya ada karena firmware ini tidak punya gerak maju/mundur
    // berumpan-balik sama sekali. 'w' bisa keduanya, tapi buta terhadap
    // sasaran dan perpindahannya terkuantisasi satu langkah gait -- tidak
    // bisa dipakai memperbaiki selisih beberapa sentimeter.
    //
    // Penjaganya berbeda per arah, dan itu disengaja: mundur diawasi LiDAR
    // BELAKANG, maju diawasi LiDAR DEPAN. Satu sensor tidak bisa menjaga dua
    // arah.
    bool setelBelakangMulai(int cm);
    bool setelBelakangSedangJalan() const { return _mode == NAV_SETEL_BLK; }
    bool setelBelakangTercapai() const    { return _setelBlkOk; }
    // Sudah bergeser berapa jauh pada ruas HNT_GESER yang sedang atau BARU
    // SAJA berjalan. navBerhenti() tidak menghapus acuannya, jadi angka ini
    // masih sah sesaat sesudah geser dihentikan -- itulah yang dipakai
    // Misi untuk melanjutkan dari sisa alih-alih mengulang penuh.
    float geserTempuhCm() const {
        return (_rataOdoCm > 0.0f) ? fabsf(_robot.geserCm() - _rataOdoAwal) : 0.0f;
    }

    bool ratakanSedangJalan() const { return _mode == NAV_RATA; }
    bool ratakanTercapai() const    { return _rataOk; }

    // --- 3. Kalibrasi Kecepatan Putar ---
    void kalibrasiPivot(uint8_t siklus);

    // --- 4. Kalibrasi gerak dari EEPROM 2048 (ditulis TES_GERAK) ---
    // Dulu hasil kalibrasiPivot() hanya hidup di RAM dan hilang tiap reset,
    // padahal TES_GERAK sudah pernah mengukur angka yang sama dan
    // menyimpannya. gerakMuat() mengambilnya; gerakSimpan() menulis balik
    // HANYA field pivot/odometri (zoff, lvl, jac milik TES_GERAK dipertahankan).
    bool gerakMuat(bool cerewet = true);
    void gerakSimpan();
    void gerakTabel();

    // Perintah gerak terakhir yang dikeluarkan navigasi/pivot (-1..1). Sudah
    // dicetak navStatus(); dibuka juga supaya telemetri & uji otomatis bisa
    // membacanya tanpa menebak dari sudut servo.
    float majuKini() const { return _majuKini; }
    float turnKini() const { return _turnKini; }

    float degCCW() const { return _degCCW; }
    float degCW()  const { return _degCW;  }
    float mmMaju() const { return _mmMaju; }
    bool  pivotTerkalibrasi() const { return _pivotKalib; }

private:
    Imu& _imu;
    Hexapod& _robot;   // Pointer ke sistem robot utama
    LidarArray& _lidar;

    ModeNav  _mode = NAV_DIAM;
    float    _errPrev = 0.0f;
    bool     _errAda  = false;
    // Turunan error dinding: dihitung sekali tiap SAMPEL LiDAR baru lalu
    // DITAHAN sampai sampel berikutnya. Kalau dihitung tiap loop, ia nol di
    // hampir semua iterasi (jarak belum berubah) lalu melonjak sekali besar --
    // itu bukan turunan, itu impuls.
    uint32_t _errStempel = 0;    // stempel sampel LiDAR yang dipakai _errPrev
    float    _errTurunan = 0.0f; // cm/detik, ditahan antar sampel

    // Iterasi terakhir masuk pita "terlalu dekat"? Dicetak oleh 'n' supaya
    // saat menyetel terlihat pita mana yang sedang bekerja.
    bool     _pitaDekat = false;

    uint32_t _tBelok  = 0;      // sejak kapan sedang menghindar halangan depan
    uint32_t _tCari   = 0;      // sejak kapan dinding samping hilang (0 = terlihat)

    // --- mode terkunci arena & pivot berdiri sendiri ---
    FaseNav  _fase     = FASE_JALAN;
    int8_t   _arahKini = -1;    // indeks arah arena yang sedang dituju (0..3)
    uint32_t _tPivot   = 0;     // awal fase berjalan (belok arena / pivot / settle)
    uint32_t _diamSejak = 0;    // sejak kapan heading berada di dalam toleransi
    float _remJarakCm = 0.0f;
    bool  _abaikanDepan = false;
    bool  _wallSamar = false;
    // MENYALA sejak awal. Turunan dari sudut terbukti di arena dan di sim:
    // goyang perintah kemudi 4,4/detik -> 0,16/detik, karena pembaginya dasar
    // 11 cm alih-alih selang sampel 25 ms. 'N0' mengembalikannya ke selisih
    // waktu kalau sepasang sensor sisi bermasalah.
    bool  _wallSudut = true;
    bool  _tengah = false;
    bool  _koreksiDiam   = false;   // fitur menyala (hanya 'U')
    bool  _sedangKoreksi = false;   // sedang berdiri memutar badan
    uint32_t _tKoreksi   = 0;       // jam NAV_KOREKSI_BATAS_MS

    // HENTI PUNCAK, dipasang 'U'. Cermin HNT_PUNCAK milik Misi, memakai
    // ambang PUNCAK_* yang SAMA -- yang berbeda cuma siapa yang menghentikan:
    // di sini Navigation, di sana ruasSelesai().
    bool     _puncakAktif  = false;
    float    _puncakAwal   = NAN;   // pitch saat dipasang; NAN = IMU bisu
    bool     _puncakNaik   = false; // sudah pernah melewati PUNCAK_NAIK_DEG
    uint32_t _puncakDatarT0 = 0;
    float _biasKiri = WALL_BIAS_KIRI_CM, _biasKanan = WALL_BIAS_KANAN_CM;   // 0 = rem tidak terpasang
    float    _pivotTarget = 0;  // heading tujuan NAV_PIVOT (derajat absolut)

    bool  arenaTerkunci() const { return _mode == NAV_ARENA_KIRI || _mode == NAV_ARENA_KANAN; }
    float  kemudiHeading(float targetHeading) const;

    // Satu langkah kendali menuju targetYaw: mengisi err (derajat, -180..180)
    // dan mengembalikan perintah putar -1..1 yang sudah diberi dorongan minimal
    // PIVOT_MIN_CMD selama masih di luar toleransi. Dipakai BERSAMA oleh pivot
    // berdiri sendiri dan fase belok arena -- dulu rumusnya ditulis dua kali
    // dengan gain yang berbeda (PIVOT_KP vs HEADING_KP).
    float  pivotLangkah(float targetYaw, float& err) const;
    void   pivotUpdate();       // dipanggil navUpdate() saat _mode == NAV_PIVOT
    void   rataUpdate();        // dipanggil navUpdate() saat _mode == NAV_RATA
    int    jarakGeser(bool keKanan) const;   // -1 = tak ada sensor yang bisa dinilai
    uint8_t  _rataCh   = 255;   // channel LiDAR yang jadi penggaris
    int      _rataCm   = 0;     // jarak sasaran ke dinding itu
    bool     _rataNaik = false; // true = sasaran LEBIH JAUH dari bacaan awal
    bool     _rataOk   = false; // hasil perataan TERAKHIR: sasaran tercapai?
    float    _rataGeser = 0.0f; // vektor geser yang sedang dipakai (+ = kanan)
    float    _rataOdoCm = 0.0f; // >0 = berhenti pada ODOMETRI sejauh ini, bukan sensor
    float    _rataOdoAwal = 0.0f;
    void     setelBelakangUpdate();

    uint32_t _tRata    = 0;
    bool     _rataBlkLapor = false;  // sudah mencetak sekali soal tarik mundur?
    bool     _rataJagaBlk  = true;   // jaga jarak belakang selama perataan ini?

    int      _setelBlkCm = 0;     // sasaran bacaan LiDAR belakang
    bool     _setelBlkOk = false; // hasil TERAKHIR: sasaran tercapai?
    bool     _setelBlkMaju = false;  // arah yang dipilih saat mulai
    uint32_t _tSetelBlk  = 0;
    float    _majuKini = 0.0f, _turnKini = 0.0f;

    float _headArah[4] = { -1, -1, -1, -1 };
    float _headKunci = NAN;     // kunci heading mutlak; NAN = pakai mata angin
    const char* _arahNama[4] = { "UTARA", "TIMUR", "SELATAN", "BARAT" };
    
    float _degCCW = 0, _degCW = 0, _mmMaju = 0;
    int8_t _pivotSign = 1;
    bool _pivotKalib = false;   // sudah punya angka nyata (bukan nol)?

    // Fungsi utilitas internal
    float wrap180(float d) const;
    void gaitPutar(float turn);
    
    // Penundaan yang tetap meng-update gait & IMU. SATU-SATUNYA pemakai yang
    // tersisa adalah kalibrasiPivot(); pivotKe() tidak lagi memblokir.
    bool tungguYaw(uint32_t ms, float& yawAkum);
    bool tunggu(uint32_t ms);
    void updateSistem();
};

#endif