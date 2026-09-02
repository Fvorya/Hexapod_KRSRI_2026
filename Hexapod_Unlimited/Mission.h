#ifndef MISSION_H
#define MISSION_H

// ====================================================================
// LAPISAN MISI KRSRI -- IRISAN PERTAMA: diam -> menuju korban 1 -> konfirmasi.
//
// ATURAN POKOK: kelas ini TIDAK PERNAH memanggil robot.walk() sendiri.
// Vektor gerak sudah dimiliki Navigation::navUpdate(). Dua penulis pada
// vektor yang sama persis kesalahan yang dijaga navBerhenti() di seluruh
// firmware ini -- kalau misi ikut menulis, yang terlihat di lapangan adalah
// robot yang "menolak berhenti" atau berjalan dengan dua perintah bergantian.
// Jadi misi duduk DI ATAS Navigation: ia MENYETIR lewat navMulai() /
// navBerhenti(), lalu MENGAMATI hasilnya.
//
// Tiga cacat Mission.cpp legacy (legacy-2026/HEXAPOD_KRSRI_2026) yang sengaja
// TIDAK dibawa ke sini:
//   1. legacy memanggil walk() langsung dan menulis ulang ikut-dinding sendiri;
//   2. legacy memakai getDistance() dua-keadaan, sehingga sensor depan MATI
//      tak bisa dibedakan dari lorong kosong -- robot maju menabrak dinding
//      sambil mengira jalannya kosong;
//   3. legacy tidak pernah memeriksa apakah navigasinya masih hidup, jadi
//      navigasi yang berhenti sendiri membuat misi menunggu selamanya.
//
// KETENTUAN MULAI KONTES: robot boleh diletakkan menghadap ke MANA SAJA, lalu
// harus berangkat ke UTARA. Karena itu misi TIDAK bisa langsung memanggil
// navMulai(): mode arena mengunci ke mata angin TERDEKAT dari hadap robot saat
// itu (arahTerdekat), jadi start yang menyimpang lebih dari 45 derajat akan
// mengunci arah yang SALAH -- tanpa satu pun pesan error, karena dari sudut
// pandang navigasi itu permintaan yang sah. Maka misi memutar badan lebih dulu
// (MISI_PIVOT_AWAL) dan baru menyerahkan kemudi ke Navigation sesudah heading
// UTARA benar-benar tercapai.
//
// URUTAN PEMANGGILAN: update() harus dipanggil SEBELUM nav.navUpdate() di
// loop(). Keduanya membaca sampel LiDAR yang sama; yang lebih dulu berhak
// memutuskan. Itu yang membuat misi sempat menghentikan navigasi sebelum
// navigasi keburu berbelok menghindari benda yang justru sedang dicari.
// ====================================================================
#include <Arduino.h>
#include "config.h"
#include "Navigation.h"   // Hexapod, LidarArray, ModeNav ikut lewat sini

enum StatMisi : uint8_t {
    MISI_DIAM = 0,       // menunggu perintah 'm1'
    MISI_PIVOT_AWAL,     // menghadapkan badan ke UTARA sebelum mulai berjalan
    MISI_KE_KORBAN1,     // ikut dinding KANAN + kunci arena, mengamati sensor depan
    MISI_PIVOT_KORBAN1,  // sudah di samping korban 1, memutar badan menghadapnya
    MISI_KONFIRM1,       // berhenti di depan sesuatu, MENUNGGU keputusan operator
    MISI_PIVOT_LANTAI,   // sesudah korban dikonfirmasi, memutar balik ke UTARA
    MISI_LANTAI_PECAH,   // menyeberangi lantai pecah dengan profil TANGGA
    MISI_TURUN,          // menuruni bidang miring dengan profil MERUNDUK
    MISI_SELESAI,        // irisan ini habis (langkah ambil korban belum ada)
    MISI_GAGAL           // berhenti karena sebab yang dicetak & disimpan
};

class Mission {
public:
    Mission(Hexapod& robot, Navigation& nav, LidarArray& lidar);

    void mulai();                       // 'm1'
    void batal(const char* alasan);     // 'm0', juga dipanggil 'x' / 's' / Enter
    void update();                      // tiap loop, SEBELUM nav.navUpdate()
    void status();                      // 'm'
    void jawab(bool korban);            // 'm2' = benar korban, 'm3' = bukan
    void setAmbang(float cm);           // 'm9 <cm>'  -- ambang sensor DEPAN
    void setAmbangBlk(float cm);        // 'm8 <cm>'  -- JARAK TEMPUH dari START ke korban 1
    void setLantaiCm(float cm);         // 'm7 <cm>'  -- lebar rintangan lantai pecah
    void setTurunCm(float cm);          // 'm6 <cm>'  -- panjang bidang miring

    StatMisi stat() const { return _stat; }
    bool berjalan() const {
        return _stat == MISI_PIVOT_AWAL   || _stat == MISI_KE_KORBAN1 ||
               _stat == MISI_PIVOT_KORBAN1 || _stat == MISI_KONFIRM1;
    }

private:
    Hexapod&    _robot;
    Navigation& _nav;
    LidarArray& _lidar;

    StatMisi _stat    = MISI_DIAM;
    uint32_t _t0      = 0;        // saat masuk state (untuk batas waktu)
    float    _ambang;             // jarak depan yang dianggap "ada sesuatu", cm
    uint8_t  _dekatN  = 0;        // sampel LiDAR berturut-turut di bawah ambang
    uint32_t _stempel = 0;        // stempel sampel depan yang TERAKHIR dihitung
    bool     _siap    = false;    // boleh memicu? dimatikan sesudah "bukan korban"

    // Pemicu KEDUA: sensor BELAKANG (ch2) mengukur jarak ke dinding START.
    // K-1 duduk DI SAMPING lintasan, jadi sensor depan tidak akan pernah
    // melihatnya -- ini yang benar-benar menghentikan robot di sana. Cara yang
    // sama dipakai program kontes sebelumnya.
    //
    // _blkAwal dicatat SENDIRI oleh 'm1' tepat sesudah pivot ke UTARA selesai,
    // bukan diketik operator. Yang bisa diukur sensor adalah jarak ke DINDING,
    // sedangkan yang diketahui orang adalah jarak TEMPUH; menyimpan titik
    // nolnya membuat keduanya bertemu tanpa aritmetika di kepala saat lomba.
    // Dicatat SESUDAH pivot, bukan sebelum, karena badan yang masih menyerong
    // membuat berkas ch2 memanjang 1/cos(sudut) -- titik nol yang salah.
    float    _blkAwal    = -1.0f; // bacaan ch2 di garis start, cm (<0 = belum)
    float    _ambangBlk;          // jarak TEMPUH dari garis start ke korban 1, cm
    uint8_t  _jauhN     = 0;      // sampel berturut-turut di atas ambang
    uint32_t _stempelBlk = 0;     // stempel sampel belakang yang terakhir dihitung
    bool     _blkSiap   = true;   // SEKALI pakai: melewati garis korban 1 hanya terjadi sekali
    // RUAS BERJARAK sesudah korban 1: lantai pecah lalu turunan. Keduanya
    // memakai ODOMETRI GAIT, bukan LiDAR -- di ruas itu tidak ada acuan mutlak
    // yang searah jalan. Dinding START sudah lama hilang, dan LiDAR depan
    // menghadap mendatar sehingga tidak melihat lantai pecah maupun bibir
    // turunan. Odometri diukur di lantai arena 2026-09-02 dan cocok dalam 1%
    // (lihat ODO_SKALA_DEF), jadi ia layak dipercaya untuk ruas sependek ini.
    //
    // Keduanya WAJIB diisi operator ('m7', 'm6') dan tidak punya default.
    // Menebak lebar rintangan yang belum pernah diukur berarti mengganti
    // profil gait di tempat yang salah -- di bibir turunan, itu jatuh.
    float    _lantaiCm  = -1.0f;  // lebar lantai pecah, cm (<0 = belum disetel)
    float    _turunCm   = -1.0f;  // panjang turunan, cm (<0 = belum disetel)
    float    _ruasAwal  = 0.0f;   // odometer saat ruas berjalan dimulai, cm
    uint32_t _serongT0  = 0;      // sejak kapan heading keluar toleransi (0 = tidak)

    const char* _sebab = nullptr; // sebab MISI_GAGAL, dicetak ulang oleh 'm'

    void     masuk(StatMisi s);
    bool     mulaiJalan();        // navMulai + verifikasi ia BENAR-BENAR jalan
    void     mulaiSusurDinding(); // dipakai sesudah pivot awal selesai
    void     gagal(const char* sebab);

    // Menunggu pivot yang sedang berjalan, lalu memeriksa hasilnya.
    // Dipakai tiga kali (pivot awal, pivot ke korban, pivot balik ke UTARA),
    // dan ketiganya butuh pemeriksaan yang sama: pivot yang SELESAI dan pivot
    // yang timeout/dibatalkan sama-sama berakhir di NAV_DIAM, jadi heading
    // akhir yang membedakannya.
    //   0 = masih berputar, 1 = sudah menghadap, -1 = gagal (sudah dilaporkan)
    int8_t   tungguPivot(uint8_t arah, const char* sebabGagal);

    // Sudah berapa cm sejak ruas ini dimulai, menurut odometri gait.
    float    ruasTempuh() const { return _robot.jarakCm() - _ruasAwal; }

    // Selama ruas berjarak: navigasi masih milik kita DAN masih menghadap
    // arah yang benar? return false berarti sudah dilaporkan gagal.
    bool     ruasSehat(uint8_t arah);
    uint32_t lewat() const { return millis() - _t0; }
};

#endif
