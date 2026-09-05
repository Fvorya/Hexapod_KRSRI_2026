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
    NAV_PIVOT                // berputar di tempat menuju satu heading ('o'/'O')
};

// Fase internal. FASE_JALAN/FASE_BELOK milik mode arena, FASE_PIVOT/FASE_SETTLE
// milik NAV_PIVOT. Modenya saling eksklusif, jadi _fase, _tPivot dan _diamSejak
// dipakai bersama oleh keduanya.
enum FaseNav : uint8_t {
    FASE_JALAN = 0,   // arena : menyusuri dinding
    FASE_BELOK,       // arena : berbelok ke mata angin berikutnya
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
    void kompasCatat(uint8_t arah); // 0=U, 1=T, 2=S, 3=B
    void kompasSimpan();
    bool kompasMuat(bool cerewet = true);
    void kompasTabel();
    bool kompasLengkap() const;   // keempat arah sudah dicatat?

    // Arah arena terdekat dari sebuah heading. return -1 bila belum ada satu
    // pun arah dicatat. selisihDeg = simpangan ke arah itu (-180..180).
    int8_t arahTerdekat(float yawDeg, float& selisihDeg) const;

    // Apakah robot SEDANG menghadap arah arena ini (dalam HEADING_TOLERANCE_DEG)?
    // Dipakai misi untuk memeriksa hasil pivot awalnya: pivot yang SELESAI dan
    // pivot yang GAGAL sama-sama berakhir di NAV_DIAM, jadi satu-satunya yang
    // membedakan keduanya dari luar adalah heading akhirnya.
    bool diArah(uint8_t arah) const;

    // Simpangan heading sekarang terhadap satu arah arena, -180..180.
    // NAN bila arah itu belum dicatat atau IMU belum punya data. Dipakai misi
    // untuk menilai odometri: simpang theta memendekkan jarak sesungguhnya
    // dengan faktor cos(theta), jadi yang penting besarnya, bukan lulus/tidak
    // terhadap satu ambang sempit milik pivot.
    float simpangArah(uint8_t arah) const;

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
    float _biasKiri = WALL_BIAS_KIRI_CM, _biasKanan = WALL_BIAS_KANAN_CM;   // 0 = rem tidak terpasang
    float    _pivotTarget = 0;  // heading tujuan NAV_PIVOT (derajat absolut)

    bool  arenaTerkunci() const { return _mode == NAV_ARENA_KIRI || _mode == NAV_ARENA_KANAN; }
    int8_t arahGeser(int8_t idx, int8_t delta) const { return (int8_t)((idx + delta + 4) % 4); }
    float  kemudiHeading(float targetHeading) const;

    // Satu langkah kendali menuju targetYaw: mengisi err (derajat, -180..180)
    // dan mengembalikan perintah putar -1..1 yang sudah diberi dorongan minimal
    // PIVOT_MIN_CMD selama masih di luar toleransi. Dipakai BERSAMA oleh pivot
    // berdiri sendiri dan fase belok arena -- dulu rumusnya ditulis dua kali
    // dengan gain yang berbeda (PIVOT_KP vs HEADING_KP).
    float  pivotLangkah(float targetYaw, float& err) const;
    void   pivotUpdate();       // dipanggil navUpdate() saat _mode == NAV_PIVOT
    float    _majuKini = 0.0f, _turnKini = 0.0f;

    float _headArah[4] = { -1, -1, -1, -1 };
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