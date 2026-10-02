#ifndef MISI_H
#define MISI_H

// ====================================================================
// LAPISAN MISI KRSRI -- BERBASIS TABEL RUAS.
//
// Lintasan adalah DATA: satu baris RUAS[] per potongan. Menambah potongan =
// menambah baris; menyetel arena = mengubah angka, bukan alur.
//
// ATURAN POKOK (mahal dipelajari, jangan dibuang):
//   1. Misi TIDAK PERNAH memanggil robot.walk(). Vektor gerak milik
//      Navigation; dua penulis = robot yang "menolak berhenti".
//   2. update() dipanggil SEBELUM nav.navUpdate() -- keduanya membaca sampel
//      LiDAR yang sama, yang lebih dulu berhak memutuskan.
//   3. Pemicu jarak dihitung hanya saat ada sampel LiDAR BARU (stempelSampel).
//   4. LIDAR_JAUH bukan "dekat" dan bukan "lewat". Mati, jauh, dan jarak
//      sungguhan adalah tiga keadaan berbeda.
//   5. Robot boleh diletakkan menghadap ke mana saja; misi WAJIB pivot lebih
//      dulu, karena mode arena mengunci ke mata angin TERDEKAT.
//
// `nilai` -1 = BELUM DIUKUR, dan misi menolak berangkat selama masih ada.
// ====================================================================
#include <Arduino.h>
#include "config.h"
#include "Navigation.h"   // Hexapod, LidarArray, ModeNav ikut lewat sini

// BELOK MASUK RUAS, relatif terhadap ruas sebelumnya. Arah mutlak dihitung
// hitungArah(): arah[i] = (arah[i-1] + belok[i]) % 4. Relatif karena yang
// salah saat mapping selalu SATU tikungan -- membetulkannya memperbaiki
// seluruh sisa lintasan sendiri, dan tikungan itulah yang bisa dicocokkan
// sambil berdiri di arena.
enum Belok : uint8_t {
    BLK_LURUS = 0,   // teruskan arah ruas sebelumnya
    BLK_KANAN = 1,   // seperempat searah jarum jam (U->T->S->B)
    BLK_BALIK = 2,   // setengah putaran
    BLK_KIRI  = 3    // seperempat berlawanan jarum jam
};

// Arah ruas PERTAMA. Yang tetap bukan hadap awal robot (ditentukan juri),
// melainkan arah berangkat dari HOME.
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
    PRF_TANGGA,      // langkah +40, panjang +10, siklus +200, badan +15
    PRF_MERUNDUK,    // langkah +10, panjang -15, siklus +200, badan -20
    PRF_SEMPIT,      // R-11: langkah +20, siklus -200, radius & badan baku
    PRF_KAIL,        // R-9: kaki depan mengait ke depan-atas, belakang naik
    PRF_TANJAK       // R-9: langkah +40, panjang -15, siklus +500, pitch badan
};

// --- Apa yang mengakhiri ruas ini ---
enum Henti : uint8_t {
    HNT_ODO = 0,     // odometri gait: sudah menempuh `nilai` cm
    HNT_DEPAN,       // LiDAR depan <= `nilai` cm
    HNT_BELAKANG,    // LiDAR belakang >= titik nol + `nilai` cm (jarak tempuh)
    HNT_LANGSUNG,    // tidak berjalan sama sekali -- ruas ini hanya aksi
    HNT_SISI,        // GESER menyamping sampai dinding sisi = `nilai` cm
    HNT_MUNDUR,      // BERJALAN MUNDUR sampai LiDAR belakang = `nilai` cm
    HNT_PUNCAK,      // NAIK lalu DATAR lagi (gyro), atau depan <= `nilai` cm
    HNT_GESER        // GESER menyamping sejauh `nilai` cm, diukur ODOMETRI
};

// HNT_PUNCAK: berhenti kalau badan sudah MENDAKI lalu DATAR lagi (gyro), atau
// depan <= `nilai`. Keduanya baru berlaku sesudah mendaki terlihat -- tanpa
// gerbang itu ruas berakhir seketika di kaki tangga. `nilai` 0 = dinding depan
// dimatikan, gyro sendirian (dipakai R-9: berkas depan sering mengenai muka
// anak tangga). Ambang gyro di config.h (PUNCAK_*). IMU bisu + `nilai` 0 =
// tidak ada yang mengakhiri ruas selain batas waktu; tabelSiap()
// memperingatkannya.
//
// HNT_BELAKANG vs HNT_MUNDUR, sensor sama, arti berlawanan:
//   HNT_BELAKANG  `nilai` = JARAK TEMPUH. Robot MAJU, dinding START jadi
//                 penggaris dari titik nol yang dicatat saat ruas masuk.
//   HNT_MUNDUR    `nilai` = JARAK MUTLAK ke dinding belakang. Robot MUNDUR
//                 sampai bacaan turun ke `nilai`. Memakai mesin 'J<cm>' dan
//                 seluruh penjaganya (lantai MUNDUR_MIN_CM, MUNDUR_BATAS_MS).
//                 Bacaan belakang JAUH = tidak ada dinding, ruas dilewati.
//
// HNT_SISI vs HNT_GESER, keduanya menyamping; `kemudi` memilih sisinya:
//   HNT_SISI   berhenti pada BACAAN DINDING sisi `kemudi`. Lebih teliti, tapi
//              gagal tertutup kalau penggarisnya bisu. KMD_TENGAH ditolak.
//   HNT_GESER  berhenti pada ODOMETRI geser; `kemudi` = arah yang DITUJU.
//              Tidak butuh dinding, tapi skala slipnya dikalibrasi untuk maju.

// --- Apa yang dikerjakan di UJUNG ruas ---
// Pivot tidak ada di sini: tiap ruas menyatakan arahnya, dan mesinnya memutar
// badan sendiri saat masuk ruas yang arahnya berbeda.
enum Aksi : uint8_t {
    AKS_TIDAK_ADA = 0,  // langsung sambung ke ruas berikutnya (tanpa berhenti)
    AKS_AMBIL,          // angkat korban dengan lengan di kolom `lengan`
                        //   henti HNT_DEPAN (ruas ini mendekat), atau
                        //   HNT_LANGSUNG sesudah ruas pendekat
    AKS_TARUH,          // taruh korban di safe zone
    AKS_KONFIRM         // berhenti, tunggu keputusan operator ('m2'/'m3')
};

// Kolom sesudah `lengan` boleh tidak ditulis: inisialisasi agregat C++
// memberi nilai bakunya.
struct Ruas {
    const char* nama;
    Belok       belok;         // belok masuk, RELATIF terhadap ruas sebelumnya
    Kemudi      kemudi;
    Profil      profil;
    bool        abaikanDepan;  // ruas ini berjalan BUTA ke depan (lihat catatan)
    Henti       henti;
    float       nilai;         // arti tergantung `henti`; <0 = BELUM DIUKUR
    Aksi        aksi;
    uint8_t     lengan;        // ARM_DEPAN / ARM_BELAKANG

    // Belok pecahan, der, di atas `belok`. MUTLAK terhadap mata angin hasil
    // `belok`, bukan terhadap hadap robot -- jadi galat tidak menumpuk. Tapi
    // nilainya MENUMPUK antar ruas: sesudah satu ruas menyerong 45, ruas
    // BLK_LURUS berikutnya tetap 45. Untuk kembali, tulis lawannya (-45).
    float       putar = 0.0f;

    // true = dua fase tambahan sesudah badan condong maju: jeda konfirmasi
    // mata (KORBAN_CONDONG_JEDA_MS) lalu pelurusan ke heading ruas. Untuk
    // korban yang tertutup reruntuhan. BUKAN saklar translasi maju -- badan
    // condong di SETIAP ruas AMBIL.
    bool        condong = false;

    // Ruas geser (HNT_SISI/HNT_GESER): tarik mundur setiap LiDAR belakang
    // membaca di atas RATA_BLK_SASARAN_CM, menahan hanyut maju. false untuk
    // ruas yang tidak punya dinding belakang yang tetap -- di sana bacaannya
    // mengukur benda lain dan malah mengemudi.
    bool        jagaBelakang = true;

    // AMBIL: badan ditarik mundur sejauh ini pada fase SIAP, supaya busur
    // turun capit (ayunan siku 110 der, paling menjulur di tengah) tidak
    // memukul korban yang berdiri dekat. Fase BADAN MAJU lalu mendorongnya ke
    // condongMm. Per korban, karena ruang tiap kantong berbeda. tabelSiap()
    // memeriksa batas BODY_MAX_TRANS_MM dan jatah LENGAN_JEDA_MS.
    float       mundurMm = 0.0f;

    // AMBIL: maju sejauh ini sebelum capit menutup. 0 = pakai KORBAN_CONDONG_MM
    // (Calib global). "Tidak maju sama sekali" = kosongkan ini DAN setel
    // condong.mm ke 0.
    float       condongMm = 0.0f;
};

// DUA TABEL:
//   RUAS_BAKU[]  const di FLASH -- lintasan seperti di-flash. 'm5r' memulangkan
//                RAM ke sini.
//   RUAS[]       di RAM, dan INILAH yang dijalankan. Disalin dari RUAS_BAKU[]
//                saat menyala, boleh diubah dari HUD ('m5s'), jumlah barisnya
//                bisa berubah ('m5+'/'m5-').
// SKOR_RUAS[] di Skor.cpp diindeks dengan nomor ruas yang sama; sisipBaris()/
// hapusBaris() menggeser keduanya, kalau tidak poin jatuh ke ruas yang salah.
extern const Ruas RUAS_BAKU[];
extern Ruas RUAS[];
extern uint8_t RUAS_N;
extern const uint8_t RUAS_BAKU_N;

// Panjang kolam nama per baris, termasuk NUL. HUD memotong nama di
// RUAS_NAMA_MAKS-1 sebelum menghitung CRC -- batas yang berbeda di kedua
// ujung = CRC yang tidak pernah cocok pada nama panjang.
#define RUAS_NAMA_MAKS 40

// Kapasitas tabel RAM. Cuma ukuran array RAM, tidak menyentuh EEPROM. Harus
// lebih besar dari jumlah baris supaya 'm5+' punya slot; dijaga static_assert
// di Misi.cpp.
#define RUAS_MAKS 40

enum StatMisi : uint8_t {
    MISI_DIAM = 0,
    MISI_JALAN,        // menjalankan RUAS[_i]
    MISI_PIVOT,        // aksi ujung: sedang memutar badan
    MISI_SETEL,        // profil gait baru dipasang -- menunggu badan tenang
    MISI_LENGAN,       // aksi ujung: sekuens lengan (pose tetap, buta)
    MISI_KONFIRM,      // aksi ujung: menunggu 'm2'/'m3'
    // Capit selesai, tapi Raspi mungkin masih memegang kaki. State sendiri:
    // KONFIRM sudah punya dua ujung (ruas AKS_KONFIRM dan parkir vision).
    MISI_LEPAS,        // aksi ujung: menunggu 'm9' -- Raspi melepas kaki
    MISI_UKUR,         // MODE UKUR: jalan tanpa syarat henti, operator yang menyetop
    MISI_SELESAI,
    MISI_GAGAL
};

class Misi {
public:
    Misi(Hexapod& robot, Navigation& nav, LidarArray& lidar);

    void mulai();                     // 'm1'  -- selalu dari ruas 0
    // 'm4 <idx> [sampai]' -- jalankan sebagian lintasan. tabelSiap() hanya
    // memeriksa ruas yang akan dijalani, jadi ruas -1 di ujung tidak memblokir
    // pengujian bagian awal.
    void mulaiDari(uint8_t idx, uint8_t sampai = 255);

    // 'm6 <idx>' -- MODE UKUR: satu ruas dengan profil, kemudi, dan arahnya,
    // TANPA syarat henti. Operator menghentikan di ujung ruas, robot mencetak
    // jarak tempuhnya. Tidak lewat tabelSiap() -- ruas yang diukur justru yang
    // masih -1. Kompas, pivot, dan servo tetap diperiksa.
    void ukur(uint8_t idx);
    void batal(const char* alasan);   // 'm0', juga 'x' / 's' / Enter
    void update();                    // tiap loop, SEBELUM nav.navUpdate()
    void status();                    // 'm'
    void tabel();                     // 'm4' tanpa argumen: cetak seluruh tabel
    void jawab(bool korban);          // 'm2' / 'm3'
    void setRuasCm(uint8_t idx, float cm);   // 'm7 <idx> <cm>'

    // --- EDITOR TABEL DARI HUD ('m5'). Semuanya ditolak selama berjalan(). ---
    //
    // 'm5s <idx> <13 kolom> | <nama>': satu baris penuh per perintah. Urutan
    // `v` = urutan anggota struct Ruas = kolom tabelDump() = urutan byte CRC:
    // belok, kemudi, profil, abaikanDepan, henti, nilai, aksi, lengan, putar,
    // condong, jagaBelakang, mundurMm, condongMm.
    bool setBaris(uint8_t idx, const float* v, const char* nama);
    bool sisipBaris(uint8_t idx);     // 'm5+ <idx>' -- baris kosong di idx
    bool hapusBaris(uint8_t idx);     // 'm5- <idx>'
    void tabelBaku();                 // 'm5r' -- RAM kembali ke tabel flash

    // Tabel RAM sama persis dengan flash? Dipakai setup() untuk menangkap
    // urutan inisialisasi global yang menolkan nilai baku diam-diam.
    bool tabelSamaBaku() const;

    // 'm5d' -- '#TABR <idx> <13 kolom> | <nama>' per ruas, bentuk yang sama
    // dengan 'm5s'. NILAI SUMBER, tidak tercermin -- yang diedit tabelnya.
    void tabelDump();

    // 'm5' -- '#TABV <crc16> <n>'. HUD menghitung CRC yang sama dari draft-nya;
    // sesudah Teensy reset CRC kembali ke tabel flash dan HUD mengirim ulang.
    void tabelVersi();
    uint16_t tabelCrc() const;

    // 'aa' / 'at' -- jalankan sekuens lengan saja, untuk menyetel pose di meja.
    // LENGAN_KORBAN_AKTIF sengaja tidak berlaku di sini.
    void ujiLengan(bool ambil);

    // SERAH-TERIMA KENDALI DENGAN RASPI, satu penguasa pada satu waktu:
    //   Teensy  #KORBAN AMBIL <ruas>  -> parkir, kendali kaki MILIK RASPI
    //   Raspi   menengahkan dan menahan badan
    //   Raspi   'm2'                  -> kendali kaki KEMBALI ke Teensy
    //   Teensy  sekuens capit
    //   Teensy  #LEPAS <ruas>         -> tanya: masih memegang kaki?
    //   Raspi   'm9'                  -> tidak lagi; misi boleh lanjut
    // Aman karena MISI_KONFIRM/MISI_LEPAS tidak memanggil ruasSehat().
    void setTungguVision(int mode);   // 'm8' / 'm8 1' / 'm8 0'
    bool tungguVision() const { return _tungguVision; }
    void lepasKendali();              // 'm9' -- Raspi melepas kaki

    StatMisi stat() const { return _stat; }

    // Untuk OLED: ruas dan jam kontes tanpa laptop.
    uint8_t  ruasKini() const { return _i; }
    uint32_t waktuMisiDetik() const {
        return _tMisi ? ((millis() - _tMisi) / 1000UL) : 0UL;
    }

    // AKTIF = ada yang harus dihentikan saat rem ditekan. Ditulis sebagai
    // "bukan DIAM/SELESAI/GAGAL" supaya state baru ikut terhitung sendiri.
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

    // Cache RUAS[].nilai yang bisa disetel operator ('m7').
    float    _cm[RUAS_MAKS];

    float    _ruasAwal = 0.0f;   // odometer saat ruas dimulai, cm
    float    _blkAwal  = -1.0f;  // bacaan LiDAR belakang di titik nol, cm
    uint8_t  _n        = 0;      // sampel berturut-turut yang memenuhi syarat
    uint32_t _stempel  = 0;      // stempel sampel yang terakhir dihitung
    uint32_t _serongT0 = 0;      // sejak kapan heading keluar toleransi (0 = tidak)

    // HNT_PUNCAK, dinolkan tiap ruas mulai. _pitchAwal = acuan ruas ini
    // (pitch mentah, sudah termasuk kemiringan profil); NAN = tidak dipakai.
    float    _pitchAwal    = NAN;
    bool     _naikTerlihat = false;   // sudah pernah melewati PUNCAK_NAIK_DEG
    uint32_t _datarT0      = 0;       // sejak kapan datar lagi (0 = belum)
    uint32_t _tenangT0 = 0;      // sejak kapan ramp profil selesai (0 = belum)
    uint8_t  _pivotUlang = 0;    // berapa kali pivot masuk ruas ini diulang
    // Pivot masuk ruas BARU SAJA selesai dan LiDAR belum dibaca sejak itu.
    // Dipakai sekali lalu dinolkan -- lihat pasangProfil().
    bool     _pivotBaru = false;
    // Yaw saat ruas berhenti, untuk mengukur serong sesudahnya. NAN = belum ada.
    float    _yawUjung = NAN;
    uint8_t  _langkah  = 0;      // sub-langkah sekuens lengan
    // Sekuens lengan dijalankan lepas dari tabel ('aa'/'at').
    bool     _uji      = false;
    bool     _ujiAmbil = false;
    // MISI_UKUR selama 'm6', MISI_DIAM selebihnya -- state akhir ruasMasuk().
    StatMisi _ukurMulai = MISI_DIAM;
    // Satu per lengan: depan dan belakang dua capit yang berdiri sendiri.
    bool     _korban[2] = { false, false };

    // Baku NYALA: parkir berbatas waktu, dan habis waktu tidak menggagalkan
    // apa pun -- sekuens tetap jalan dengan sudut tetap. 'm8 0' tanpa Raspi.
    bool     _tungguVision = true;
    uint8_t  _lidarUlang   = 0;   // jatah pindai ulang, dinolkan tiap misi

    // Ruas berikutnya DILANJUTKAN, bukan diulang: titik nol tidak ditulis
    // ulang. Dipasang pulihkanLidar(), dihabiskan sekali di kepala ruasJalan().
    bool     _lanjutRuas   = false;

    // Keadaan saklar cermin saat _arah[]/_serong[] terakhir dihitung.
    bool     _arahCermin   = false;
    // Jendela deteksi Raspi terbuka? Dibuka '#KORBAN', ditutup '#LEPAS'. Lebih
    // panjang daripada parkir, jadi bukan salinan _parkirVision.
    bool     _visiJalan   = false;
    // Membedakan dua ujung MISI_KONFIRM: ruas AKS_KONFIRM (-> ruasBerikut)
    // dan parkir vision (-> MISI_LENGAN). Tanpa ini 'm2' melompati capit.
    bool     _parkirVision = false;

    const char* _sebab = nullptr;

    const Ruas& r() const { return RUAS[_i]; }
    float       cmKini() const { return _cm[_i]; }
    uint32_t    lewat() const { return millis() - _t0; }

    // WAJIB dipanggil sesudah tiap perubahan tabel RAM: menyegarkan _cm[] dan
    // _arah[], kalau tidak misi tetap memakai angka lama.
    void tabelBerubah();
    // Tabel boleh diubah sekarang? false + pesan kalau misi sedang berjalan.
    bool bolehUbahTabel();
    void salinBaris(uint8_t ke, const Ruas& dari);   // + nama ke kolam RAM

    void hitungArah();

    // Hitung ulang _arah[]/_serong[] BILA saklar cermin berubah sejak
    // perhitungan terakhir. Satu perbandingan bool saat tidak berubah.
    void segarkanArah();
    float headingRuas(uint8_t i) const;

public:
    // --- ARENA CERMIN ---
    // Saklar 'arena.mirror' (K_ARENA_MIRROR), dibalik tombol D3 atau
    // 'Qarena.mirror 1'. Satu tabel, tiga pengakses -- SELURUH kode yang
    // membaca kolom berarah wajib lewat sini:
    //   belok   KIRI <-> KANAN (LURUS/BALIK tetap). Mata angin ikut tercermin
    //           lewat hitungArah(): TIMUR <-> BARAT, UTARA/SELATAN tetap.
    //   kemudi  KIRI <-> KANAN. TENGAH tetap.
    //   putar   dinegasikan.
    // Jarak, profil, aksi, dan lengan tidak dicerminkan.
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
    // berpoin=false: ruas DILEWATI -- poin hanya untuk ruas yang dikerjakan.
    void ruasBerikut(bool berpoin = true);

    // LEWATI ruas ini dan teruskan misi. Sebab lunak (pivot meleset, batas
    // waktu, heading hilang, perataan ditolak) lewat sini. Yang tetap
    // membatalkan: servo lemas, LiDAR mati, waktu kontes habis.
    void lewati(const char* sebab);

    // Ada LiDAR yang TIDAK menjawab sama sekali? (Bukan 'jauh'.)
    bool lidarMati();

    // Coba hidupkan ulang LiDAR yang putus, seperti 'I'. true = semua menjawab
    // lagi. Jatah LIDAR_ULANG_MAKS per misi.
    bool pulihkanLidar();
    void cetakRuas(uint8_t idx);
};

#endif