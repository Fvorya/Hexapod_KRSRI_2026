#include "Mission.h"

// ====================================================================
// AMBANG TRIAL -- setel di arena
// ====================================================================
// AMBANG KORBAN WAJIB LEBIH BESAR DARI FRONT_STOP_CM.
//
// Di mode arena, navUpdate() sendiri sudah bereaksi begitu depan
// <= FRONT_STOP_CM (20 cm): ia masuk FASE_BELOK dan memutar ke mata angin
// berikutnya. Kalau ambang misi ditaruh DI BAWAH angka itu, navigasi selalu
// keburu berbelok lebih dulu dan state konfirmasi tidak pernah tercapai.
//
// Inilah sebabnya VICTIM_REACH_CM 12 cm milik Mission.cpp legacy tidak akan
// pernah bekerja di firmware ini -- dan kenapa cacat itu tidak ketahuan di
// sana: FSM legacy tidak punya lapisan navigasi yang ikut punya pendapat.
#define MISI_KORBAN_CM_DEF 25
static_assert(MISI_KORBAN_CM_DEF > FRONT_STOP_CM,
              "Ambang korban harus > FRONT_STOP_CM -- kalau tidak, navigasi keburu berbelok "
              "dan misi tidak pernah sampai ke state konfirmasi");

// Berapa sampel LiDAR BERTURUT-TURUT di bawah ambang sebelum berhenti.
// Nilainya kecil dengan sengaja: LidarArray sudah menyaring hantu lewat
// median-3 dan LIDAR_MIN_CM, jadi ini cuma lapis terakhir terhadap satu
// pantulan menyimpang. Ongkosnya ~3 x periode sampel sensor depan.
static const uint8_t  MISI_DEKAT_N  = 3;

// Sesudah operator menjawab "bukan korban", pemicu dimatikan sampai sensor
// depan benar-benar terbuka lagi. Tanpa ini robot berhenti lagi seketika pada
// benda yang SAMA -- ia masih berdiri tepat di depannya.
static const int      MISI_REARM_CM = 10;

// Batas waktu satu kali perjalanan menuju korban 1.
static const uint32_t MISI_BATAS_MS = 60000;

// Arah berangkat menurut ketentuan kontes: robot boleh diletakkan menghadap ke
// mana saja, lalu harus berangkat ke UTARA. Indeks kompas arena Navigation:
// 0=UTARA, 1=TIMUR, 2=SELATAN, 3=BARAT -- sama dengan argumen 'c0'..'c3'.
// ponytail: satu arah tetap, bukan parameter. Ubah angka ini kalau ketentuan
// arenanya berubah; jadikan argumen 'm1' hanya kalau nanti benar-benar perlu
// dipilih saat jalan.
static const uint8_t MISI_ARAH_AWAL = 0;

// Arah hadap saat sudah di samping korban 1. Robot menyusuri lorong ke UTARA,
// K-1 ada di sisi BARAT lintasan, jadi badan diputar seperempat menghadapnya.
static const uint8_t MISI_ARAH_KORBAN1 = 3;   // 3 = BARAT

// Batas waktu satu RUAS BERJARAK (lantai pecah / turunan). Lebih longgar dari
// batas menuju korban 1 karena kedua ruas itu dilalui dengan profil lambat:
// TANGGA menaikkan cycleTime dari 900 ke 1800 ms, MERUNDUK ke 1100 ms.
static const uint32_t MISI_RUAS_BATAS_MS = 90000;

// Berapa lama heading boleh keluar toleransi sebelum ruas dinyatakan rusak.
// Bukan nol: mode arena mengoreksi terus, jadi heading wajar melintas keluar
// sesaat. Yang ditangkap adalah robot yang BENAR-BENAR berbelok -- mis. mode
// arena menemukan halangan depan lalu pindah ke mata angin berikutnya, yang
// membuat sisa ruas diukur ke arah yang salah.
static const uint32_t MISI_SERONG_BATAS_MS = 2000;

// JARAK dari garis start sampai robot berada di samping korban 1, diukur
// SENSOR BELAKANG (ch2) -- bukan odometri gait. Misi tidak pernah menyentuh
// Hexapod::jarakCm() maupun rem jarak 'D'.
//
// Disebut "jarak" dan bukan "bacaan sensor" karena titik nolnya (_blkAwal)
// dicatat sendiri oleh 'm1' sesudah pivot ke UTARA, lalu dikurangkan. Jadi
// angka yang diketik operator adalah angka yang bisa diukur meteran di arena,
// sementara yang dibandingkan di dalam kode tetap dua bacaan ch2.
//
// K-1 duduk di ceruk DI SAMPING lintasan, jadi sensor depan tidak akan pernah
// melihatnya: kalau hanya mengandalkan ambang depan, robot melewati K-1 begitu
// saja lalu berhenti karena batas waktu. Ini persis peran yang README sebut
// "koreksi absolut jarak dekat", dan batasannya ikut: jangkauan sensor cuma
// LIDAR_MAX_CM (70 cm) dan harus ada dinding di belakang. Untuk K-1 keduanya
// terpenuhi; untuk korban berikutnya TIDAK, jadi jangan menyalin pola ini ke
// sana tanpa memeriksa ulang.
#define MISI_BLK_CM_DEF 40
static_assert(MISI_BLK_CM_DEF < LIDAR_MAX_CM,
              "Jarak tempuh korban 1 harus di dalam jangkauan sensor belakang -- di luar "
              "itu bacaannya jatuh ke LIDAR_JAUH dan pemicunya tidak pernah menyala");

Mission::Mission(Hexapod& robot, Navigation& nav, LidarArray& lidar)
    : _robot(robot), _nav(nav), _lidar(lidar), _ambang((float)MISI_KORBAN_CM_DEF),
      _ambangBlk((float)MISI_BLK_CM_DEF) {}

void Mission::masuk(StatMisi s) {
    _stat = s;
    _t0 = millis();
    _dekatN = 0;
    _stempel = 0;
    _jauhN = 0;
    _stempelBlk = 0;
}

// navMulai() SUDAH memeriksa servo, mux, sensor depan, sensor samping, IMU,
// kelengkapan kompas arena, dan kalibrasi pivot -- lalu mencetak persis mana
// yang gagal. Jangan diulang di sini: satu pemeriksaan, satu pesan, tidak bisa
// menyimpang. Yang perlu dilakukan hanya membaca kembali apakah ia jadi jalan.
bool Mission::mulaiJalan() {
    _nav.navMulai(NAV_ARENA_KANAN);
    return _nav.navMode() == NAV_ARENA_KANAN;
}

// Dipanggil dua kali: sesudah pivot awal selesai, dan sesudah operator
// menjawab "bukan korban". Keduanya butuh baris keterangan yang sama persis.
void Mission::mulaiSusurDinding() {
    Serial.print("  Berhenti bila ada sesuatu di dalam "); Serial.print(_ambang, 0);
    Serial.println(" cm di depan, lalu menunggu keputusan Anda.");
    Serial.println("  's', 'x', Enter, atau 'm0' membatalkan kapan saja.");
}

void Mission::mulai() {
    if (berjalan()) {
        Serial.println("Misi sudah berjalan. 'm' untuk status, 'm0' untuk membatalkan.");
        return;
    }
    Serial.println("\n=== MISI MULAI -- menuju korban 1 ===");
    Serial.print("  Langkah 1: pivot ke ");
    Serial.print(_nav.namaArah(MISI_ARAH_AWAL));
    Serial.println(" -- posisi awal boleh menghadap ke mana saja.");
    Serial.println("  Langkah 2: ikut dinding KANAN + terkunci kompas arena.");

    // Kedua syarat ini diperiksa DI SINI, bukan dibiarkan gagal di navMulai()
    // sesudah robot terlanjur berputar. Bedanya penting: pivotKe() hanya
    // MEMPERINGATKAN kalau arah putar belum dikalibrasi, sedangkan mode arena
    // MENOLAK. Tanpa penjaga di depan, robot memutar badan sampai 20 detik
    // dulu baru memberi tahu bahwa misinya memang tidak bisa jalan.
    if (!_nav.kompasLengkap()) {
        Serial.println("Gagal: kompas arena belum lengkap -- 'c0'..'c3' lalu 'e', atau 'E'.");
        Serial.println("       Periksa dengan 'k'.");
        return;
    }
    if (!_nav.pivotTerkalibrasi()) {
        Serial.println("Gagal: pivot belum dikalibrasi -- 'C' lalu 'S' sekali saja.");
        Serial.println("       Periksa dengan 'K'. Tanpa itu arah putar masih tebakan.");
        return;
    }

    // Sensor BELAKANG adalah satu-satunya yang bisa melihat korban 1 dilewati.
    // navMulai() tidak memeriksanya (navigasi memang tidak memakainya), jadi
    // tanpa penjaga ini ch2 yang mati berarti robot berjalan melewati K-1 dan
    // baru berhenti karena batas waktu 60 detik -- kegagalan paling mahal yang
    // bisa terjadi di lapangan, dan tanpa satu pun petunjuk penyebabnya.
    if (_lidar.getDistance(LIDAR_BACK) == LIDAR_MATI) {
        Serial.print("Gagal: sensor BELAKANG (channel "); Serial.print(LIDAR_BACK);
        Serial.println(") tidak merespons.");
        Serial.println("       Dialah yang mengukur jarak tempuh dari dinding START,");
        Serial.println("       jadi tanpa dia robot tidak tahu kapan sampai di korban 1.");
        Serial.println("       Ketik 'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return;
    }

    _nav.pivotKompas(MISI_ARAH_AWAL);
    if (!_nav.pivotSedangJalan()) {
        _stat = MISI_DIAM;
        Serial.println("Misi TIDAK jadi dimulai -- pivot awal tidak mau jalan.");
        Serial.println("  Sebabnya tercetak di atas ('b' dulu bila servo masih lemas).");
        return;
    }

    masuk(MISI_PIVOT_AWAL);
    _siap    = true;
    _blkSiap = true;
    _blkAwal = -1.0f;      // dicatat nanti, sesudah badan menghadap UTARA
    _sebab   = nullptr;
    Serial.println("  's', 'x', Enter, atau 'm0' membatalkan kapan saja.");
}

void Mission::batal(const char* alasan) {
    // Pola sama dengan navBerhenti(): kalau tidak ada yang berjalan, jangan
    // mencetak apa-apa. Kalau tidak, tiap 'x' dan Enter akan meninggalkan
    // baris "Misi DIBATALKAN" walau tak ada misi sama sekali.
    if (!berjalan()) { _stat = MISI_DIAM; return; }

    _stat = MISI_DIAM;
    _dekatN = 0;
    _siap = false;
    _sebab = nullptr;
    _nav.navBerhenti(alasan);
    Serial.print("Misi DIBATALKAN");
    if (alasan) { Serial.print(": "); Serial.println(alasan); } else Serial.println(".");
}

void Mission::gagal(const char* sebab) {
    _sebab = sebab;
    _stat  = MISI_GAGAL;
    Serial.print("\n!! MISI GAGAL: "); Serial.println(sebab);
    Serial.println("   Robot berhenti di tempat, servo TETAP HIDUP.");
    Serial.println("   'm' untuk status, 'm1' untuk mengulang, 'x' untuk melemaskan.");
}

int8_t Mission::tungguPivot(uint8_t arah, const char* sebabGagal) {
    if (_nav.pivotSedangJalan()) return 0;
    if (!_nav.diArah(arah)) { gagal(sebabGagal); return -1; }
    return 1;
}

bool Mission::ruasSehat(uint8_t arah) {
    if (_nav.navMode() != NAV_ARENA_KANAN) {
        gagal("navigasi berhenti atau diambil alih di tengah ruas -- sebabnya tercetak di atas.");
        return false;
    }
    if (lewat() > MISI_RUAS_BATAS_MS) {
        _nav.navBerhenti("batas waktu ruas.");
        gagal("ruas tidak selesai dalam batas waktu.");
        return false;
    }
    // Odometri hanya berarti kalau arahnya masih benar. Robot yang berbelok
    // ke mata angin lain tetap menambah jarak, dan sisa ruasnya diukur ke
    // arah yang salah -- itu mengganti profil gait di tempat yang keliru.
    if (_nav.diArah(arah)) {
        _serongT0 = 0;
    } else {
        if (_serongT0 == 0) _serongT0 = millis();
        else if (millis() - _serongT0 > MISI_SERONG_BATAS_MS) {
            _nav.navBerhenti("heading menyimpang di tengah ruas.");
            gagal("robot tidak lagi menghadap arah ruas -- jarak tempuh tidak bisa dipercaya.");
            return false;
        }
    }
    return true;
}

void Mission::update() {
    switch (_stat) {

    case MISI_PIVOT_AWAL:
        // Navigation yang menghitung pivotnya, lengkap dengan timeout
        // PIVOT_BATAS_MS sendiri -- tidak perlu batas waktu kedua di sini.
        // Yang tersisa cuma menunggu ia berhenti lalu MEMERIKSA hasilnya:
        // pivot yang selesai dan pivot yang dibatalkan/timeout sama-sama
        // berakhir di NAV_DIAM, jadi heading akhir yang membedakannya.
        {
            int8_t p = tungguPivot(MISI_ARAH_AWAL,
                        "pivot awal tidak sampai -- dibatalkan, timeout, atau IMU lepas.");
            if (p <= 0) return;
        }
        // Titik nol jarak tempuh, dicatat SEKARANG: badan sudah lurus
        // menghadap UTARA, jadi berkas ch2 tegak lurus dinding START dan
        // angkanya adalah jarak yang sebenarnya. Sebelum pivot badan masih
        // menyerong dan bacaannya memanjang 1/cos(sudut).
        // _blkAwal < 0 berarti ini keberangkatan pertama. State yang sama
        // dipakai ulang sesudah "bukan korban" untuk memutar badan kembali ke
        // UTARA -- di situ titik nolnya JANGAN dicatat ulang, robot sudah
        // terlanjur menempuh jarak.
        if (_blkAwal < 0.0f) {
            int b0 = _lidar.getDistance(LIDAR_BACK);
            if (b0 == LIDAR_MATI || b0 == LIDAR_JAUH) {
                gagal("dinding START tidak terbaca sensor belakang -- titik nol tak bisa dicatat.");
                return;
            }
            _blkAwal = (float)b0;
            // Garis korban 1 harus jatuh DI DALAM jangkauan sensor. Kalau
            // tidak, bacaannya keburu jatuh ke LIDAR_JAUH dan pemicunya tidak
            // pernah menyala -- robot melewati K-1 lalu mati karena batas
            // waktu. Lebih baik menolak sekarang, selagi robot masih di start.
            float sasaran = _blkAwal + _ambangBlk;
            if (sasaran > (float)LIDAR_MAX_CM - 5.0f) {
                Serial.print("  titik nol "); Serial.print(_blkAwal, 1);
                Serial.print(" cm + tempuh "); Serial.print(_ambangBlk, 1);
                Serial.print(" cm = "); Serial.print(sasaran, 1);
                Serial.print(" cm, di luar jangkauan sensor ("); Serial.print(LIDAR_MAX_CM);
                Serial.println(" cm).");
                Serial.println("  Majukan robot lebih dekat ke dinding START, atau kecilkan 'm8'.");
                gagal("garis korban 1 di luar jangkauan sensor belakang.");
                return;
            }
            Serial.print("  titik nol jarak: "); Serial.print(_blkAwal, 1);
            Serial.print(" cm dari dinding START -> berhenti di bacaan ");
            Serial.print(sasaran, 1); Serial.println(" cm.");
        }

        if (!mulaiJalan()) {
            gagal("gagal memulai ikut dinding -- sebabnya tercetak di atas.");
            return;
        }
        masuk(MISI_KE_KORBAN1);
        Serial.print("\nSudah menghadap ");
        Serial.print(_nav.namaArah(MISI_ARAH_AWAL));
        Serial.println(" -- mulai menyusuri dinding kanan.");
        mulaiSusurDinding();
        break;

    case MISI_KE_KORBAN1: {
        // 1) Apakah navigasi masih milik kita?
        //    Satu pemeriksaan ini menangkap SEMUANYA: navigasi yang berhenti
        //    sendiri (sensor depan mati, terjebak, dinding hilang terlalu lama),
        //    dan navigasi yang diambil alih dari serial ('f', 'F', 'p', 'o',
        //    'C', ...). Alternatifnya menaruh sebelas kait di parser, dan yang
        //    ke-dua belas pasti terlupa. Navigation sudah mencetak sebabnya.
        if (_nav.navMode() != NAV_ARENA_KANAN) {
            gagal("navigasi berhenti atau diambil alih -- sebabnya tercetak di atas.");
            return;
        }

        // 2) Batas waktu. SENGAJA tidak mundur otomatis seperti FSM legacy:
        //    mundur itu buta di robot ini (sensor belakang ch2 tidak terpakai),
        //    dan legacy mengulangi manuver yang sama tanpa henti -- kalau
        //    tersangkut karena maju ke sudut, mundur lurus lalu maju lagi
        //    dengan perintah yang sama akan mengantarnya ke sudut yang sama.
        if (lewat() > MISI_BATAS_MS) {
            _nav.navBerhenti("batas waktu misi.");
            gagal("tidak sampai ke korban 1 dalam batas waktu.");
            return;
        }

        // 2b) PEMICU UTAMA korban 1: sudah berapa jauh dari dinding START.
        //     Ditaruh sebelum pemicu depan karena inilah yang benar-benar
        //     menghentikan robot di samping K-1; pemicu depan cuma jaring
        //     pengaman kalau ada sesuatu yang menghalangi lebih dulu.
        if (_blkSiap) {
            int blk = _lidar.getDistance(LIDAR_BACK);
            // LIDAR_JAUH SENGAJA tidak dianggap "sudah lewat 40 cm". Di robot
            // ini bacaan di luar jangkauan DAN bacaan hantu sama-sama muncul
            // sebagai JAUH (lihat LIDAR_MIN_CM), jadi memicu dari situ berarti
            // berhenti di tempat acak. Kalau dinding START benar-benar hilang
            // dari pandangan, biar batas waktu yang menghentikannya -- itu
            // berhenti dengan sebab yang tercetak, bukan salah tebak.
            if (blk != LIDAR_MATI && blk != LIDAR_JAUH) {
                uint32_t sb = _lidar.stempelSampel(LIDAR_BACK);
                if (sb != _stempelBlk) {          // hitung HANYA sampel baru
                    _stempelBlk = sb;
                    if ((float)blk < _blkAwal + _ambangBlk) _jauhN = 0;
                    else if (++_jauhN >= MISI_DEKAT_N) {
                        _blkSiap = false;         // peristiwa sekali seumur perjalanan
                        _nav.navBerhenti("sudah sampai garis korban 1.");
                        Serial.println("\n=== SAMPAI DI SAMPING KORBAN 1 ===");
                        Serial.print("  bacaan belakang "); Serial.print(blk);
                        Serial.print(" cm = tempuh "); Serial.print((float)blk - _blkAwal, 1);
                        Serial.print(" cm dari garis start (target "); Serial.print(_ambangBlk, 0);
                        Serial.println(" cm)");
                        Serial.print("  Memutar badan menghadap ");
                        Serial.print(_nav.namaArah(MISI_ARAH_KORBAN1)); Serial.println(".");

                        _nav.pivotKompas(MISI_ARAH_KORBAN1);
                        if (!_nav.pivotSedangJalan()) {
                            gagal("pivot ke arah korban 1 tidak mau jalan.");
                            return;
                        }
                        masuk(MISI_PIVOT_KORBAN1);
                        return;
                    }
                }
            } else {
                _jauhN = 0;
            }
        }

        int depan = _lidar.getDistance(LIDAR_FRONT);

        // 3) Tiga keadaan LiDAR, tiga perlakuan berbeda. Inilah yang tidak bisa
        //    dilakukan Mission legacy: di sana ketiganya sama-sama -1.
        if (depan == LIDAR_MATI) {
            // Navigasi yang berhak menghentikan (dan sudah melakukannya);
            // cabang 1 di atas menangkapnya iterasi berikutnya. Di sini cukup
            // jangan menghitung apa pun -- jangan sampai sensor putus terhitung
            // sebagai "sudah sampai".
            _dekatN = 0;
            return;
        }
        if (depan == LIDAR_JAUH) {
            _dekatN = 0;
            _siap = true;           // depan terbuka -> pemicu boleh aktif lagi
            return;
        }

        // 4) Penjaga picu ulang sesudah "bukan korban".
        if (!_siap) {
            if (depan > (int)(_ambang + (float)MISI_REARM_CM)) _siap = true;
            _dekatN = 0;
            return;
        }

        // 5) Hitung HANYA saat ada sampel LiDAR BARU.
        //    update() dipanggil ribuan kali per detik, sedangkan sensor depan
        //    hanya menghasilkan satu pengukuran tiap ~150 ms (round-robin enam
        //    channel). Tanpa gerbang ini, "3 sampel berturut-turut" cuma berarti
        //    tiga iterasi loop yang membaca angka yang sama persis -- penjaganya
        //    lolos seketika dan tidak menyaring apa-apa. Masalah & obatnya sama
        //    dengan suku turunan PD dinding di Navigation.
        uint32_t st = _lidar.stempelSampel(LIDAR_FRONT);
        if (st == _stempel) return;
        _stempel = st;

        if (depan > (int)_ambang) { _dekatN = 0; return; }

        if (++_dekatN < MISI_DEKAT_N) return;

        _nav.navBerhenti("ada sesuatu di depan -- menunggu keputusan operator.");
        masuk(MISI_KONFIRM1);
        Serial.println("\n=== BERHENTI: ADA SESUATU DI DEPAN ===");
        Serial.print("  jarak depan : "); Serial.print(depan); Serial.println(" cm");
        Serial.println("  Robot ini BELUM punya deteksi korban -- benda ini bisa saja");
        Serial.println("  dinding, kaki meja, atau sepatu. Anda yang memutuskan.");
        Serial.println("    'm2' = benar korban   -> misi lanjut");
        Serial.println("    'm3' = bukan korban   -> jalan lagi");
        Serial.println("    'm0' = batalkan misi  |  'x' = lemas darurat");
        break;
    }

    case MISI_PIVOT_KORBAN1:
        // Pola yang sama dengan MISI_PIVOT_AWAL: Navigation punya timeout
        // sendiri, jadi tinggal menunggu ia berhenti lalu memeriksa headingnya.
        {
            int8_t p = tungguPivot(MISI_ARAH_KORBAN1,
                        "pivot ke arah korban 1 tidak sampai -- dibatalkan, timeout, atau IMU lepas.");
            if (p <= 0) return;
        }
        masuk(MISI_KONFIRM1);
        Serial.print("\n=== BERHENTI MENGHADAP ");
        Serial.print(_nav.namaArah(MISI_ARAH_KORBAN1));
        Serial.println(" -- KORBAN 1 SEHARUSNYA DI DEPAN ===");
        Serial.print("  jarak depan sekarang : ");
        {
            int d = _lidar.getDistance(LIDAR_FRONT);
            if      (d == LIDAR_MATI) Serial.println("MATI");
            else if (d == LIDAR_JAUH) Serial.println("jauh -- tak ada apa pun dalam jangkauan");
            else { Serial.print(d); Serial.println(" cm"); }
        }
        Serial.println("  Robot ini BELUM punya deteksi korban -- Anda yang memutuskan.");
        Serial.println("    'm2' = benar korban   -> misi lanjut");
        Serial.println("    'm3' = bukan korban   -> jalan lagi");
        Serial.println("    'm0' = batalkan misi  |  'x' = lemas darurat");
        break;

    case MISI_PIVOT_LANTAI: {
        int8_t p = tungguPivot(MISI_ARAH_AWAL,
                    "pivot balik ke arah lorong tidak sampai -- dibatalkan, timeout, atau IMU lepas.");
        if (p <= 0) return;

        // Profil TANGGA: kaki +35 mm supaya tidak menyangkut bibir ubin pecah,
        // badan +10 mm supaya sasis tidak mengandas, siklus +900 ms sehingga
        // tiap langkah punya waktu untuk mendarat. Pergantian profil di-ramp
        // GAIT_PROFILE_TAU, jadi aman dipanggil sambil robot berjalan.
        _robot.profileStairs();

        if (!mulaiJalan()) {
            gagal("gagal memulai ikut dinding untuk ruas lantai pecah.");
            return;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        masuk(MISI_LANTAI_PECAH);
        Serial.println("\n=== RUAS LANTAI PECAH ===");
        Serial.print("  profil TANGGA, ikut dinding kanan, berhenti sesudah ");
        Serial.print(_lantaiCm, 0); Serial.println(" cm menurut odometri.");
        break;
    }

    case MISI_LANTAI_PECAH:
        if (!ruasSehat(MISI_ARAH_AWAL)) return;
        if (ruasTempuh() < _lantaiCm) return;

        // Sampai di ujung lantai pecah. Profil MERUNDUK untuk turunan 1:4:
        // langkah 45 mm memangkas jarak jatuh kaki tiap langkah (langkah x
        // tan 14 der), badan -20 mm menurunkan titik berat DAN melipat kaki
        // sehingga sisa jangkauan ke bawah bertambah di bibir turunan.
        _robot.profileCrouch();
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        masuk(MISI_TURUN);
        Serial.println("\n=== RUAS TURUNAN ===");
        Serial.print("  profil MERUNDUK, berhenti sesudah ");
        Serial.print(_turunCm, 0); Serial.println(" cm menurut odometri.");
        break;

    case MISI_TURUN:
        if (!ruasSehat(MISI_ARAH_AWAL)) return;
        if (ruasTempuh() < _turunCm) return;

        _nav.navBerhenti("ujung turunan tercapai.");
        _robot.profileFlat();
        masuk(MISI_SELESAI);
        Serial.println("\n=== SELESAI: DI BAWAH TURUNAN ===");
        Serial.println("  Profil dikembalikan ke DATAR.");
        Serial.println("  Irisan misi ini habis di sini. 'm1' untuk mengulang dari awal.");
        break;

    case MISI_KONFIRM1:
        // Navigasi memang sudah DIAM di sini (kita yang menghentikannya), jadi
        // penjaga servo Navigation tidak lagi berlaku. Jaga sendiri.
        if (!_robot.isArmed()) gagal("servo dilemaskan saat menunggu konfirmasi.");
        break;

    default:
        break;   // DIAM / SELESAI / GAGAL: tidak ada yang perlu dikerjakan
    }
}

void Mission::jawab(bool korban) {
    if (_stat != MISI_KONFIRM1) {
        Serial.println("Tidak sedang menunggu konfirmasi. Ketik 'm' untuk melihat status misi.");
        return;
    }

    if (korban) {
        Serial.println("Dicatat: KORBAN.");
        Serial.println("  Langkah 'ambil korban' DILEWATI -- lengan belum terpasang fisik.");

        // Kedua ruas berikutnya diukur odometri, dan angkanya harus datang
        // dari meteran. Menebak lebar lantai pecah berarti mengganti profil
        // gait di tempat yang salah, dan di bibir turunan itu jatuh.
        if (_lantaiCm < 0.0f || _turunCm < 0.0f) {
            masuk(MISI_SELESAI);
            Serial.println("  Misi berhenti di sini: jarak ruas berikutnya belum disetel.");
            if (_lantaiCm < 0.0f) Serial.println("    'm7 <cm>' = lebar rintangan lantai pecah");
            if (_turunCm  < 0.0f) Serial.println("    'm6 <cm>' = panjang bidang miring");
            Serial.println("  Ukur dengan meteran, setel, lalu 'm1' lagi.");
            return;
        }

        Serial.print("  Memutar badan kembali ke ");
        Serial.print(_nav.namaArah(MISI_ARAH_AWAL));
        Serial.println(" untuk menghadap lantai pecah.");

        _nav.pivotKompas(MISI_ARAH_AWAL);
        if (!_nav.pivotSedangJalan()) { gagal("pivot balik ke arah lorong tidak mau jalan."); return; }
        masuk(MISI_PIVOT_LANTAI);
        return;
    }

    Serial.println("Dicatat: BUKAN korban.");
    // Badan sedang menghadap arah korban, bukan arah lorong. Kalau langsung
    // navMulai(), mode arena akan mengunci mata angin TERDEKAT dari hadap
    // SEKARANG -- yaitu arah korban itu sendiri -- dan robot berjalan menabrak
    // dinding samping. Jadi putar balik dulu, lewat state yang sama dengan
    // keberangkatan; _blkAwal yang sudah terisi menjaga titik nol tidak
    // dicatat ulang.
    Serial.print("  Memutar badan kembali ke ");
    Serial.print(_nav.namaArah(MISI_ARAH_AWAL)); Serial.println(", lalu menyusuri dinding lagi.");

    _nav.pivotKompas(MISI_ARAH_AWAL);
    if (!_nav.pivotSedangJalan()) { gagal("pivot kembali ke arah lorong tidak mau jalan."); return; }
    masuk(MISI_PIVOT_AWAL);
    _siap = false;      // jangan langsung memicu lagi pada benda yang SAMA
    Serial.print("  Pemicu depan dimatikan sampai terbuka lagi -- di atas ");
    Serial.print(_ambang + (float)MISI_REARM_CM, 0);
    Serial.println(" cm, atau 'jauh'.");
}

// Kedua setter ini sengaja TIDAK punya default. Nilai <0 berarti "belum
// diukur", dan 'm2' menolak melanjutkan ke ruas berjarak selama masih begitu.
static void setRuas(const char* nama, float& tujuan, float cm, const char* perintah) {
    const float lo = 5.0f, hi = 400.0f;
    float v = clampf(cm, lo, hi);
    Serial.print(nama); Serial.print(": ");
    if (tujuan < 0.0f) Serial.print("belum disetel"); else Serial.print(tujuan, 1);
    Serial.print(" -> "); Serial.print(v, 1); Serial.println(" cm");
    if (fabsf(v - cm) > 1e-3f) {
        Serial.print("  (diminta "); Serial.print(cm, 1);
        Serial.print(", DI-CLAMP ke "); Serial.print(lo, 0);
        Serial.print(" .. "); Serial.print(hi, 0); Serial.println(")");
    }
    Serial.print("  Diukur ODOMETRI GAIT, bukan LiDAR -- di ruas itu tidak ada acuan");
    Serial.println(" mutlak yang searah jalan.");
    Serial.print("  Hanya di RAM. Setel ulang dengan '"); Serial.print(perintah);
    Serial.println("' tiap robot menyala.");
    tujuan = v;
}

void Mission::setLantaiCm(float cm) { setRuas("lebar lantai pecah", _lantaiCm, cm, "m7 <cm>"); }
void Mission::setTurunCm (float cm) { setRuas("panjang turunan",    _turunCm,  cm, "m6 <cm>"); }

void Mission::setAmbangBlk(float cm) {
    // Jarak TEMPUH, bukan bacaan sensor. Batas atasnya menyisakan ruang untuk
    // titik nol: bacaan akhir = titik nol + tempuh, dan itu harus tetap di
    // dalam LIDAR_MAX_CM. Pemeriksaan yang sebenarnya dilakukan 'm1' saat
    // titik nolnya sudah diketahui -- ini cuma pagar kasarnya.
    const float lo = 5.0f;
    const float hi = (float)LIDAR_MAX_CM - 15.0f;
    float v = clampf(cm, lo, hi);

    Serial.print("jarak korban 1: "); Serial.print(_ambangBlk, 1);
    Serial.print(" -> "); Serial.print(v, 1); Serial.println(" cm dari garis start");
    Serial.println("  DIUKUR SENSOR BELAKANG (ch2), bukan odometri gait.");
    Serial.println("  = bacaan ch2 sekarang dikurangi bacaan ch2 di garis start.");
    if (fabsf(v - cm) > 1e-3f) {
        Serial.print("  (diminta "); Serial.print(cm, 1);
        Serial.print(", DI-CLAMP ke rentang sah "); Serial.print(lo, 1);
        Serial.print(" .. "); Serial.print(hi, 1); Serial.println(")");
    }
    _ambangBlk = v;
    Serial.println("  Hanya di RAM -- belum ada slot EEPROM untuk parameter misi.");
}

void Mission::setAmbang(float cm) {
    // Batas bawah bukan selera: di bawah FRONT_STOP_CM navigasi selalu keburu
    // berbelok dan misi tak pernah sampai ke konfirmasi. Batas atas menjaga
    // ambang tetap di dalam jangkauan akurat sensor.
    const float lo = (float)FRONT_STOP_CM + 1.0f;
    const float hi = (float)LIDAR_MAX_CM - 5.0f;
    float v = clampf(cm, lo, hi);

    Serial.print("ambang korban : "); Serial.print(_ambang, 1);
    Serial.print(" -> "); Serial.print(v, 1); Serial.println(" cm");
    if (fabsf(v - cm) > 1e-3f) {
        Serial.print("  (diminta "); Serial.print(cm, 1);
        Serial.print(", DI-CLAMP ke rentang sah "); Serial.print(lo, 1);
        Serial.print(" .. "); Serial.print(hi, 1); Serial.println(")");
        Serial.println("  Batas bawah = FRONT_STOP_CM + 1: di bawah itu navigasi");
        Serial.println("  keburu berbelok sebelum misi sempat berhenti.");
    }
    _ambang = v;
    Serial.println("  Hanya di RAM -- belum ada slot EEPROM untuk parameter misi.");
}

void Mission::status() {
    Serial.println("\n--- STATUS MISI ---");
    Serial.print("  state       : ");
    switch (_stat) {
        case MISI_DIAM:        Serial.println("DIAM (ketik 'm1' untuk mulai)"); break;
        case MISI_PIVOT_AWAL:  Serial.print("PIVOT -- menghadapkan badan ke ");
                               Serial.println(_nav.namaArah(MISI_ARAH_AWAL)); break;
        case MISI_PIVOT_LANTAI: Serial.println("PIVOT -- memutar balik ke UTARA"); break;
        case MISI_LANTAI_PECAH: Serial.print("LANTAI PECAH -- profil TANGGA, tempuh ");
                               Serial.print(ruasTempuh(), 1); Serial.print(" dari ");
                               Serial.print(_lantaiCm, 0); Serial.println(" cm"); break;
        case MISI_TURUN:       Serial.print("TURUNAN -- profil MERUNDUK, tempuh ");
                               Serial.print(ruasTempuh(), 1); Serial.print(" dari ");
                               Serial.print(_turunCm, 0); Serial.println(" cm"); break;
        case MISI_PIVOT_KORBAN1: Serial.print("PIVOT -- memutar menghadap korban 1 (");
                               Serial.print(_nav.namaArah(MISI_ARAH_KORBAN1));
                               Serial.println(")"); break;
        case MISI_KE_KORBAN1:  Serial.println("MENUJU KORBAN 1"); break;
        case MISI_KONFIRM1:    Serial.println("MENUNGGU KONFIRMASI ('m2' korban / 'm3' bukan)"); break;
        case MISI_SELESAI:     Serial.println("SELESAI (irisan pertama)"); break;
        case MISI_GAGAL:       Serial.println("GAGAL"); break;
    }
    if (_stat == MISI_GAGAL && _sebab) { Serial.print("  sebab       : "); Serial.println(_sebab); }

    Serial.print("  ambang depan: "); Serial.print(_ambang, 1);
    Serial.print(" cm  (navigasi berbelok sendiri di "); Serial.print(FRONT_STOP_CM);
    Serial.println(" cm)");
    Serial.print("  jarak korban: "); Serial.print(_ambangBlk, 1);
    Serial.print(" cm dari garis start, diukur SENSOR BELAKANG -- ");
    Serial.println(_blkSiap ? "pemicu aktif" : "SUDAH terlewati");
    Serial.println("                 (pemicu korban 1 TIDAK memakai odometri)");
    Serial.print("  lantai pecah: ");
    if (_lantaiCm < 0.0f) Serial.println("BELUM disetel -- 'm7 <cm>'");
    else { Serial.print(_lantaiCm, 1); Serial.println(" cm, odometri gait"); }
    Serial.print("  turunan     : ");
    if (_turunCm < 0.0f) Serial.println("BELUM disetel -- 'm6 <cm>'");
    else { Serial.print(_turunCm, 1); Serial.println(" cm, odometri gait"); }
    Serial.print("  titik nol   : ");
    if (_blkAwal < 0.0f) Serial.println("belum dicatat (dicatat 'm1' sesudah pivot ke UTARA)");
    else { Serial.print(_blkAwal, 1); Serial.print(" cm -> berhenti di bacaan ");
           Serial.print(_blkAwal + _ambangBlk, 1); Serial.println(" cm"); }

    if (_stat == MISI_KE_KORBAN1) {
        Serial.print("  berjalan    : "); Serial.print(lewat() / 1000);
        Serial.print(" dari "); Serial.print(MISI_BATAS_MS / 1000); Serial.println(" detik");
        Serial.print("  sampel dekat: "); Serial.print(_dekatN);
        Serial.print(" dari "); Serial.println(MISI_DEKAT_N);
        Serial.print("  pemicu      : ");
        Serial.println(_siap ? "aktif" : "MATI -- menunggu depan terbuka lagi");
    }

    int depan = _lidar.getDistance(LIDAR_FRONT);
    Serial.print("  depan       : ");
    if      (depan == LIDAR_MATI) Serial.println("MATI -- sensor tidak merespons");
    else if (depan == LIDAR_JAUH) Serial.println("jauh");
    else { Serial.print(depan); Serial.println(" cm"); }

    int blk = _lidar.getDistance(LIDAR_BACK);
    Serial.print("  belakang    : ");
    if      (blk == LIDAR_MATI) Serial.println("MATI -- pemicu korban 1 TIDAK BEKERJA");
    else if (blk == LIDAR_JAUH) Serial.println("jauh -- dinding START di luar 70 cm atau hantu");
    else { Serial.print(blk); Serial.print(" cm dari dinding START  (sampel jauh: ");
           Serial.print(_jauhN); Serial.print(" dari "); Serial.print(MISI_DEKAT_N);
           Serial.println(")"); }

    Serial.print("  navigasi    : ");
    Serial.println(_nav.navMode() == NAV_ARENA_KANAN ? "ikut dinding KANAN + kunci arena"
                                                     : "TIDAK sedang dipegang misi");
    Serial.println("  ('v' untuk rincian navigasi, 'l' untuk tabel LiDAR)");
}
