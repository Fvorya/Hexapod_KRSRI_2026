#include "Navigation.h"
#include "Calib.h"  // Memanggil ini agar bisa membaca GAIT_CYCLE_TIME dari sistem
#include "EEMap.h"  // KompasStore, GerakStore, eeSum -- satu definisi bersama

// Konstruktor disambungkan ke Hexapod
Navigation::Navigation(Imu& imuRef, Hexapod& robotRef, LidarArray& lidarRef)
    : _imu(imuRef), _robot(robotRef), _lidar(lidarRef) {}

void Navigation::begin() {
    kompasMuat(false);
    gerakMuat(true);     // pivot & odometri hasil kalibrasi TES_GERAK
}

// ====================================================================
// UTILITAS INTERNAL
// ====================================================================

float Navigation::wrap180(float d) const {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Meneruskan perintah putar langsung ke fungsi walk milik Hexapod
void Navigation::gaitPutar(float turn) {
    // walk(maju, geser, putar)
    _robot.walk(0.0f, 0.0f, turn); 
}

// Sangat Krusial: Selama fungsi delay tunggu(), robot harus terus di-update!
void Navigation::updateSistem() {
    _imu.update();
    _robot.update(); // Menggerakkan gait, Kinematika, dan servo secara mandiri
}

bool Navigation::tunggu(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        updateSistem();
        // Cek interupsi serial atau tombol di sini jika diperlukan
    }
    return true;
}

bool Navigation::tungguYaw(uint32_t ms, float& yawAkum) {
    uint32_t t0 = millis();
    float yawPrev = _imu.yawDeg();
    yawAkum = 0;
    
    while (millis() - t0 < ms) {
        updateSistem();
        float yKini = _imu.yawDeg();
        yawAkum += wrap180(yKini - yawPrev);
        yawPrev = yKini;
    }
    return true;
}

// ====================================================================
// 1. KOMPAS ARENA
// ====================================================================

void Navigation::kompasCatat(uint8_t arah) {
    if (arah > 3) return;
    if (!_imu.hasData()) { Serial.println("Navigation: Tidak ada data sudut IMU."); return; }
    
    _headArah[arah] = _imu.yawDeg();
    Serial.print("Tercatat "); Serial.print(_arahNama[arah]);
    Serial.print(" = "); Serial.print(_headArah[arah], 1); Serial.println(" derajat");
}

void Navigation::kompasSimpan() {
    KompasStore s;
    memset(&s, 0, sizeof(s));
    s.m0 = 0xC0; s.m1 = 0x3A; s.ver = 1;
    for (uint8_t i = 0; i < 4; i++) s.head[i] = _headArah[i];
    s.sum = eeSum(&s, offsetof(KompasStore, sum));
    EEPROM.put(EE_KOMPAS_ADDR, s);
    Serial.println("Kompas Arena disimpan ke EEPROM 1792.");
}

bool Navigation::kompasMuat(bool cerewet) {
    KompasStore s;
    EEPROM.get(EE_KOMPAS_ADDR, s);
    if (s.m0 != 0xC0 || s.m1 != 0x3A || s.ver != 1 ||
        s.sum != eeSum(&s, offsetof(KompasStore, sum))) {
        if (cerewet) Serial.println("EEPROM kompas kosong/rusak.");
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) _headArah[i] = s.head[i];
    if (cerewet) Serial.println("4 arah dimuat dari EEPROM.");
    return true;
}

int8_t Navigation::arahTerdekat(float yawDeg, float& selisihDeg) const {
    int8_t terbaik = -1;
    float  minAbs  = 1e9f;
    for (uint8_t i = 0; i < 4; i++) {
        if (_headArah[i] < 0) continue;          // arah ini belum dicatat
        float d = wrap180(_headArah[i] - yawDeg);
        if (fabsf(d) < minAbs) { minAbs = fabsf(d); terbaik = (int8_t)i; selisihDeg = d; }
    }
    if (terbaik < 0) selisihDeg = 0.0f;
    return terbaik;
}

float Navigation::simpangArah(uint8_t arah) const {
    if (arah > 3 || _headArah[arah] < 0.0f || !_imu.hasData()) return NAN;
    return wrap180(_headArah[arah] - _imu.yawDeg());
}

bool Navigation::diArah(uint8_t arah) const {
    if (arah > 3 || _headArah[arah] < 0.0f || !_imu.hasData()) return false;
    return fabsf(wrap180(_headArah[arah] - _imu.yawDeg())) <= HEADING_TOLERANCE_DEG;
}

bool Navigation::kompasLengkap() const {
    for (uint8_t i = 0; i < 4; i++) if (_headArah[i] < 0) return false;
    return true;
}

void Navigation::kompasTabel() {
    Serial.println("\n--- KOMPAS ARENA ---");
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(i); Serial.print(" "); Serial.print(_arahNama[i]);
        if (_headArah[i] < 0) { Serial.println("\t: belum dicatat"); continue; }
        Serial.print("\t: "); Serial.print(_headArah[i], 1); Serial.println(" der");
    }
}

// ====================================================================
// 2. PIVOT TERTUTUP (PD) -- NON-BLOKIR
// ====================================================================
// pivotKe() hanya MEMULAI; pivotUpdate() menjalankan satu langkah tiap
// navUpdate(). Dulu pivotKe() memblokir loop utama sampai 20 detik: selama itu
// parser serial mati total, 'x' tidak bisa melemaskan servo, dan satu-satunya
// rem adalah jalan keluar "tekan Enter" yang ditanam di dalam loop-nya sendiri.
// Sekarang pivot memakai jalur berhenti yang sama dengan navigasi ikut-dinding.

// Satu langkah kendali menuju targetYaw. Suku D murni dari gyro Z (di dalam
// kemudiHeading), dan _pivotSign sudah diserap di sana juga. Dorongan minimal
// diberikan SESUDAH penandaan arah, jadi arahnya ikut tanda turn.
float Navigation::pivotLangkah(float targetYaw, float& err) const {
    err = wrap180(targetYaw - _imu.yawDeg());
    float turn = kemudiHeading(targetYaw);
    if (fabsf(err) > HEADING_TOLERANCE_DEG && fabsf(turn) < PIVOT_MIN_CMD)
        turn = (turn >= 0 ? PIVOT_MIN_CMD : -PIVOT_MIN_CMD);
    return turn;
}

void Navigation::pivotKe(float targetYaw) {
    if (!_imu.hasData()) { Serial.println("Gagal: Tidak ada data IMU."); return; }
    if (!_robot.isArmed()) {
        Serial.println("Gagal: servo masih lemas. Ketik 'b' dulu supaya robot berdiri.");
        return;
    }

    // Beda dengan mode arena yang MENOLAK jalan tanpa kalibrasi: pivot dengan
    // arah putar terbalik ketahuan sendiri lewat timeout 20 detik, jadi cukup
    // diperingatkan. Yang berbahaya adalah mode arena -- di fase jalan, tanda
    // yang salah hanya melengkungkan lintasan diam-diam tanpa gejala.
    if (!_pivotKalib) {
        Serial.println("Peringatan: pivot belum dikalibrasi -- arah putar masih tebakan.");
        Serial.println("            Kalau robot berputar MENJAUHI target, jalankan 'C' lalu 'S'.");
    }

    if (_mode != NAV_DIAM) navBerhenti("diambil alih perintah pivot.");

    _mode        = NAV_PIVOT;
    _fase        = FASE_PIVOT;
    _pivotTarget = targetYaw;
    _tPivot      = millis();
    _diamSejak   = 0;
    _majuKini    = _turnKini = 0.0f;

    Serial.print("Pivot MULAI menuju "); Serial.print(targetYaw, 1);
    Serial.print(" der (sekarang ");     Serial.print(_imu.yawDeg(), 1);
    Serial.println(" der).");
    Serial.println("  's', 'x', atau Enter untuk membatalkan.");
}

// Satu langkah pivot. Dipanggil navUpdate() saat _mode == NAV_PIVOT.
void Navigation::pivotUpdate() {
    uint32_t now = millis();

    // Fase 2: target tercapai, perintah putar sudah nol, tinggal menunggu kaki
    // settle ke home. Pengganti tunggu(800) yang dulu memblokir di ujung.
    if (_fase == FASE_SETTLE) {
        if (now - _tPivot >= PIVOT_SETTLE_MS) {
            _mode = NAV_DIAM;
            _majuKini = _turnKini = 0.0f;
            Serial.print("Pivot SELESAI: yaw "); Serial.print(_imu.yawDeg(), 1);
            Serial.print(" der, simpang ");
            Serial.print(wrap180(_pivotTarget - _imu.yawDeg()), 1);
            Serial.println(" der.");
        }
        return;
    }

    // IMU bisa lepas di tengah pivot (kabel Serial2). Tanpa penjaga ini yaw
    // membeku di nilai terakhir dan robot berputar terus sampai timeout.
    if (!_imu.hasData()) { navBerhenti("data IMU hilang saat pivot."); return; }

    if (now - _tPivot > PIVOT_BATAS_MS) {
        navBerhenti("pivot gagal mencapai target (timeout).");
        return;
    }

    float err;
    float turn = pivotLangkah(_pivotTarget, err);
    _majuKini = 0.0f;
    _turnKini = turn;
    _robot.walk(0.0f, 0.0f, turn);

    // Histeresis: harus berada di dalam toleransi selama PIVOT_DIAM_MS, bukan
    // sekadar menyentuhnya sesaat saat melintas.
    if (fabsf(err) <= HEADING_TOLERANCE_DEG) {
        if (_diamSejak == 0) _diamSejak = now;
        if (now - _diamSejak >= PIVOT_DIAM_MS) {
            _robot.stop();
            _turnKini  = 0.0f;
            _fase      = FASE_SETTLE;
            _tPivot    = now;          // dipakai ulang: awal hitungan settle
            _diamSejak = 0;
        }
    } else {
        _diamSejak = 0;
    }
}

void Navigation::pivotKompas(uint8_t arah) {
    if (arah > 3 || _headArah[arah] < 0) {
        Serial.println("Arah belum dicatat atau tidak valid.");
        return;
    }
    Serial.print("Navigation: Pivot menuju "); Serial.println(_arahNama[arah]);
    pivotKe(_headArah[arah]);
}

void Navigation::pivotRelatif(float der) {
    if (!_imu.hasData()) return;
    float t = _imu.yawDeg() + der;
    while (t >= 360.0f) t -= 360.0f;
    while (t < 0)       t += 360.0f;
    pivotKe(t);
}

// ====================================================================
// 3. KALIBRASI PIVOT
// ====================================================================

void Navigation::kalibrasiPivot(uint8_t siklus) {
    if (!_imu.hasData()) { Serial.println("Gagal: Tidak ada data IMU."); return; }
    if (!_robot.isArmed()) {
        Serial.println("Gagal: servo masih lemas. Ketik 'b' dulu supaya robot berdiri.");
        return;
    }
    if (siklus == 0) siklus = 4;

    // kalibrasiPivot() MASIH memblokir (tunggu/tungguYaw). Selama itu navUpdate()
    // tidak dipanggil, jadi mode apa pun yang sedang berjalan akan membeku lalu
    // berebut perintah gait dengan kalibrasi. Hentikan dulu, jangan biarkan
    // keduanya menulis vektor gerak yang sama.
    if (_mode != NAV_DIAM) navBerhenti("diambil alih kalibrasi pivot.");

    uint32_t lama = (uint32_t)(siklus * GAIT_CYCLE_TIME);
    float hasil[2];

    for (uint8_t arah = 0; arah < 2; arah++) {
        float cmd = arah ? -1.0f : 1.0f;
        gaitPutar(cmd);
        tunggu(500); // Tunggu *slew* naik mulus

        // BUG LAMA: kedua pengukuran memakai variabel yang sama, padahal
        // tungguYaw() menolkan akumulatornya di awal. Akibatnya putaran utama
        // TERBUANG dan hasil[] cuma berisi rotasi sisa pengereman 1 detik --
        // derajat/siklus jadi jauh terlalu kecil. Sekarang dipisah lalu
        // dijumlahkan, karena keduanya sama-sama rotasi akibat perintah ini.
        float utama = 0.0f, sisaRem = 0.0f;
        tungguYaw(lama, utama);

        gaitPutar(0);
        tungguYaw(1000, sisaRem);   // sisa *settle* pengereman

        hasil[arah] = utama + sisaRem;
    }

    _degCCW = hasil[0] / siklus;
    _degCW  = hasil[1] / siklus;
    _pivotSign = (_degCCW >= 0) ? +1 : -1;
    _pivotKalib = (fabsf(_degCCW) > 0.1f || fabsf(_degCW) > 0.1f);

    Serial.print("Kalibrasi selesai. CCW "); Serial.print(_degCCW, 2);
    Serial.print(" der/siklus, CW ");        Serial.print(_degCW, 2);
    Serial.print(" der/siklus, tanda putar "); Serial.println(_pivotSign);
    Serial.println("Ketik 'S' untuk menyimpannya ke EEPROM 2048 (kalau tidak, hilang saat reset).");
}

// ====================================================================
// 4. KALIBRASI GERAK DI EEPROM 2048 (blok milik TES_GERAK)
// ====================================================================

bool Navigation::gerakMuat(bool cerewet) {
    GerakStore s;
    EEPROM.get(EE_GERAK_ADDR, s);

    if (s.m0 != 0x6E || s.m1 != 0x2C || s.ver != 2 ||
        s.sum != eeSum(&s, offsetof(GerakStore, sum))) {
        if (cerewet) Serial.println("Navigation: kalibrasi pivot EEPROM 2048 belum ada -> jalankan 'C' lalu 'S'.");
        _pivotKalib = false;
        return false;
    }

    _degCCW = s.ccw; _degCW = s.cw; _mmMaju = s.maju;
    // JANGAN pernah biarkan 0: gaitPutar(_pivotSign * turn) akan selalu nol
    // dan pivot berputar-putar 20 detik tanpa menggerakkan apa pun.
    _pivotSign = (s.sign >= 0) ? +1 : -1;
    _pivotKalib = (fabsf(_degCCW) > 0.1f || fabsf(_degCW) > 0.1f);

    if (cerewet) {
        Serial.print("Navigation: pivot dimuat dari EEPROM 2048 -> CCW ");
        Serial.print(_degCCW, 2); Serial.print(" / CW "); Serial.print(_degCW, 2);
        Serial.println(" der per siklus.");
    }
    return _pivotKalib;
}

// Baca-ubah-tulis: field milik TES_GERAK (zoff, lvlR/lvlP, refR/refP, jac)
// DIPERTAHANKAN apa adanya. Kalau blok belum ada, dibuat baru dengan field
// itu bernilai nol -- jadi jangan pakai ini untuk menimpa kalibrasi kaki.
void Navigation::gerakSimpan() {
    GerakStore s;
    EEPROM.get(EE_GERAK_ADDR, s);

    bool sah = (s.m0 == 0x6E && s.m1 == 0x2C && s.ver == 2 &&
                s.sum == eeSum(&s, offsetof(GerakStore, sum)));
    if (!sah) {
        memset(&s, 0, sizeof(s));
        s.m0 = 0x6E; s.m1 = 0x2C; s.ver = 2;
        Serial.println("Navigation: blok 2048 belum sah -> dibuat baru (zOff & rata badan = 0).");
    }

    s.ccw = _degCCW; s.cw = _degCW; s.maju = _mmMaju; s.sign = _pivotSign;
    s.sum = eeSum(&s, offsetof(GerakStore, sum));
    EEPROM.put(EE_GERAK_ADDR, s);

    Serial.println("Navigation: kalibrasi pivot disimpan ke EEPROM 2048.");
}

void Navigation::gerakTabel() {
    Serial.println("\n--- KALIBRASI GERAK (EEPROM 2048) ---");
    Serial.print("  status      : ");
    Serial.println(_pivotKalib ? "terkalibrasi" : "BELUM (jalankan 'C' lalu 'S')");
    Serial.print("  CCW         : "); Serial.print(_degCCW, 2); Serial.println(" der/siklus");
    Serial.print("  CW          : "); Serial.print(_degCW, 2);  Serial.println(" der/siklus");
    Serial.print("  maju        : "); Serial.print(_mmMaju, 1); Serial.println(" mm/siklus");
    Serial.print("  tanda putar : "); Serial.println(_pivotSign);
}

// ====================================================================
// 5. NAVIGASI OTONOM: IKUT DINDING (NON-BLOKIR)
// ====================================================================
// Berbeda dari pivotKe() yang memblokir sampai 20 detik, ini hanya menghitung
// SATU langkah tiap dipanggil. Perintah serial tetap terproses, dan 'x' /
// Enter selalu bisa menyela.
//
// Di sinilah pembedaan tiga keadaan LiDAR terbayar:
//   jarak cm   -> kemudikan PD terhadap dinding
//   LIDAR_JAUH -> dinding hilang (tikungan/celah) -> cari dengan membelok
//   LIDAR_MATI -> sensor putus -> BERHENTI, jangan jalan buta
// Kalau ketiganya disamakan (seperti kode lama), lorong terbuka akan
// diperlakukan sama dengan sensor rusak.

void Navigation::navMulai(ModeNav m) {
    if (m == NAV_DIAM) { navBerhenti("diminta berhenti."); return; }
    if (!_robot.isArmed()) {
        Serial.println("Gagal: servo masih lemas. Ketik 'b' dulu supaya robot berdiri.");
        return;
    }
    if (!_lidar.muxTerdeteksi()) {
        Serial.println("Gagal: LiDAR tidak terdeteksi. Ketik 'I' untuk memindai bus.");
        return;
    }

    // Periksa dua sensor yang BENAR-BENAR dipakai mode ini, sebelum melangkah.
    // Tanpa ini robot mulai berjalan lalu navUpdate() menghentikannya satu
    // iterasi kemudian -- dari luar terlihat seperti "menolak jalan tanpa
    // sebab", dan pesannya tidak menyebut sensor mana yang bermasalah.
    const bool    kiri     = (m == NAV_DINDING_KIRI || m == NAV_ARENA_KIRI);
    const uint8_t idSisiCk = kiri ? LIDAR_KIRI_D : LIDAR_KANAN_D;

    // Saat sensor depan sengaja diabaikan, keadaannya tidak lagi menentukan:
    // aturan yang akan memakainya sudah dimatikan semua.
    if (!_abaikanDepan && _lidar.getDistance(LIDAR_FRONT) == LIDAR_MATI) {
        Serial.print("Gagal: sensor DEPAN (channel "); Serial.print(LIDAR_FRONT);
        Serial.println(") tidak merespons -- jangan pernah berjalan buta ke depan.");
        Serial.println("       Ketik 'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return;
    }
    if (_lidar.getDistance(idSisiCk) == LIDAR_MATI) {
        Serial.print("Gagal: sensor samping "); Serial.print(LidarArray::nama(idSisiCk));
        Serial.print(" (channel "); Serial.print(idSisiCk);
        Serial.println(") tidak merespons.");
        Serial.print("       Mode ini mengemudi dari sensor itu. Coba sisi sebelahnya ('");
        Serial.print(kiri ? 'F' : 'f'); Serial.println("'), atau 'I' untuk init ulang.");
        return;
    }
    bool arena = (m == NAV_ARENA_KIRI || m == NAV_ARENA_KANAN);
    if (arena) {
        if (!_imu.hasData()) {
            Serial.println("Gagal: tidak ada data IMU."); return;
        }
        if (!kompasLengkap()) {
            Serial.println("Gagal: kompas arena belum lengkap. Catat keempat arah");
            Serial.println("       dengan 'c0'..'c3' lalu 'e', atau muat dengan 'E'.");
            return;
        }
        // Kunci ke arah arena TERDEKAT dari hadap robot sekarang. Robot
        // diasumsikan sudah kira-kira sejajar lorong saat perintah diberikan.
        float selisih = 0.0f;
        _arahKini = arahTerdekat(_imu.yawDeg(), selisih);
        if (_arahKini < 0) { Serial.println("Gagal: tak ada arah arena yang cocok."); return; }
        // WAJIB, bukan sekadar peringatan. Mode arena mengemudi berdasarkan
        // selisih heading, dan _pivotSign-lah yang menentukan ke arah mana
        // perintah putar menggeser yaw. Kalau tandanya salah, robot berbelok
        // MENJAUHI target -- di fase belok tertangkap timeout, tapi di fase
        // jalan ia hanya melengkung diam-diam ke arah yang keliru.
        // _pivotSign tidak bisa ditebak dari kode: bergantung pemasangan IMU.
        if (!_pivotKalib) {
            Serial.println("Gagal: pivot belum dikalibrasi, arah putar belum diketahui.");
            Serial.println("       Jalankan 'C' (kalibrasi) lalu 'S' (simpan) sekali saja.");
            Serial.println("       Tanpa itu mode arena bisa mengemudi ke arah yang salah.");
            return;
        }
    }

    // Kalau ada pivot yang sedang berjalan, hentikan dulu supaya pesannya jelas
    // -- bukan sekadar ditimpa diam-diam oleh _mode = m di bawah.
    if (_mode != NAV_DIAM) navBerhenti("diambil alih perintah navigasi.");

    _mode = m;
    _fase = FASE_JALAN;
    _errAda = false; _errPrev = 0.0f; _errTurunan = 0.0f; _errStempel = 0;
    _pitaDekat = false;
    _tBelok = 0; _tCari = 0; _tPivot = 0; _diamSejak = 0;

    Serial.print("Navigasi MULAI: ikut dinding ");
    Serial.print((m == NAV_DINDING_KIRI || m == NAV_ARENA_KIRI) ? "KIRI" : "KANAN");
    if (arena) {
        Serial.print(", terkunci arah "); Serial.print(_arahNama[_arahKini]);
        Serial.print(" ("); Serial.print(_headArah[_arahKini], 1); Serial.print(" der)");
    }
    Serial.println();
    Serial.println("  's', 'x', atau Enter untuk menghentikan.");
}

// Satu-satunya jalan berhenti untuk SEMUA mode, pivot termasuk. Karena 's',
// 'x', Enter dan 'w' di .ino sudah memanggil ini, pivot otomatis ikut bisa
// dibatalkan tanpa kode khusus.
// ==== KENDALI SAMAR (fuzzy) UNTUK KEMUDI DINDING =========================
//
// Menggantikan SATU baris: rumus PD di pita normal. Masukannya PERSIS sama --
// err dan _errTurunan yang sudah dihitung untuk PD -- supaya yang dibandingkan
// benar-benar hukum kendalinya, bukan jumlah sensor yang dipakai.
//
// Partisi segitiga: tiga himpunan yang pusatnya berjarak sama, sehingga jumlah
// derajat keanggotaan SELALU 1,0 di seluruh rentang.
static void samarPartisi3(float x, float c1, float c2, float c3, float mu[3]) {
    if (x <= c1) { mu[0] = 1.0f; mu[1] = 0.0f; mu[2] = 0.0f; return; }
    if (x >= c3) { mu[0] = 0.0f; mu[1] = 0.0f; mu[2] = 1.0f; return; }
    if (x < c2) { float t = (x - c1) / (c2 - c1); mu[0] = 1.0f - t; mu[1] = t;        mu[2] = 0.0f; }
    else        { float t = (x - c2) / (c3 - c2); mu[0] = 0.0f;     mu[1] = 1.0f - t; mu[2] = t;    }
}

// Pusat himpunan. err dalam cm, turunan dalam cm/detik.
// Pusat sengaja LEBAR. Di luar pusat terjauh keluaran samar MENDATAR, dan
// mendatar berarti kehilangan redaman justru saat simpangan paling besar --
// lima percobaan -- pusat +-6/+-4, +-12/+-10, +-12/+-20, +-30/+-40, lalu
// sama-sama menghasilkan ayunan yang memantul antara kedua dinding lorong
// 45 cm. Sumbu TURUNAN yang paling menuntut: derau kuantisasi saja sudah
// menghasilkan belasan cm/detik, dan mendatar di situ berarti kehilangan
// redaman tepat pada puncak deraunya.
//
// Batas akhirnya DIHITUNG, bukan ditebak lagi. Error: +-30 cm melampaui
// seluruh lebar lorong 45 cm. Turunan: dts dijaga minimum 0,005 detik dan
// satu langkah EMA menggeser 0,4 cm, jadi |derr| tidak bisa melebihi
// 0,4 / 0,005 = 80 cm/detik. Di luar kedua batas itu tidak ada keadaan yang
// bisa dicapai robot, jadi tabel ini tidak pernah mendatar dalam praktik --
// dan itu memang tujuannya: yang dibandingkan BENTUK hukumnya, bukan
// seberapa besar kewenangan yang kebetulan tersisa.
//
// Nilai singleton di luar +-0,50 memang melebihi NAV_WALL_TURN_MAX; PD pun
// begitu, dan clamp di hilir yang mengurusnya.
static const float SAMAR_E_C[3]  = { -30.0f, 0.0f, +30.0f };   // cm
static const float SAMAR_DE_C[3] = { -80.0f, 0.0f, +80.0f };   // cm/detik

// Keluaran tiap aturan (singleton Sugeno orde-0), dalam KERANGKA DINDING yang
// sama dengan kurung PD: + = kemudikan MENDEKAT dinding, - = MENJAUH.
//
// TUJUH dari sembilan nilai SENGAJA disamakan dengan PD di titik pusatnya,
// yaitu 0,008*err + 0,030*turunan. Itu bukan malas: kalau skalanya berbeda,
// yang terbandingkan cuma "mana yang gainnya lebih besar", bukan bentuk
// hukumnya. Percobaan pertama memakai angka yang lebih kecil dan hasilnya
// robot menyeberang menabrak dinding lawan -- redamannya tiga kali lebih
// lemah dari PD, jadi yang teruji hanyalah kendali yang salah setel.
//
// GAGASAN ASLINYA dua sudut asimetris, dan KEDUANYA sudah dikembalikan ke
// nilai PD sesudah diuji. Riwayatnya layak disimpan karena hasilnya negatif
// dan itu justru kesimpulannya:
//
//   (JAUH, MENDEKAT) dilunakkan -> robot "menggok ke kanan" di arena. Saat
//   masih terlalu jauh dan sedang mendekat, rem yang dikurangi membuatnya
//   melewati setpoint dan merapat ke dinding yang diikuti.
//
//   (DEKAT, MENJAUH) dilunakkan -> di lorong 45 cm robot menyeberang dan
//   menyentuh dinding LAWAN (sim: celah seberang -0,15 cm). "Sudah bergerak
//   ke arah yang benar" ternyata bukan alasan untuk mengurangi rem, karena
//   di lorong sesempit ini arah yang benar tetap punya ujung.
//
// Jadi tabelnya sekarang PERSIS PD di seluruh kotaknya: interpolasi bilinear
// atas fungsi linear menghasilkan fungsi itu sendiri. Yang tersisa sebagai
// beda cuma PENJENUHAN di luar +-12 cm / +-20 cm/detik.
//
// Itu kesimpulan yang berguna, bukan kegagalan: kebebasan tabel aturan untuk
// menjadi tak-linear tidak membeli apa pun di lorong yang KEDUA dindingnya
// mengikat. Yang membeli sesuatu justru sumber turunannya (N2/N3) dan kemudi
// menengah (Z1), bukan bentuk hukumnya.
//
// Di kedua sudut itu robot sudah bergerak KE ARAH yang benar, dan suku D
// milik PD justru melawannya: kd 0,030 x 20 cm/det = 0,60, cukup besar untuk
// mengalahkan suku P dan mengemudikan robot kembali ke sisi yang salah.
// Tabel aturan bisa mengatakan "mendekat ke setpoint dari sisi yang benar itu
// bukan masalah" -- kalimat yang canggung ditulis sebagai satu rumus linear.
//
// SEPARUH, bukan nol. Menghapus rem itu sama sekali membuat robot menyeberang
// lorong 45 cm dan menyentuh dinding lawan: di lorong sesempit ini, mendekat
// ke setpoint terlalu cepat TETAP masalah. Angka separuh adalah kompromi yang
// terukur, bukan tebakan -- lihat bagian 5 sim_dinding.
//
//                    turunan:  MENDEKAT   TETAP   MENJAUH
static const float SAMAR_Z[3][3] = {
    /* err DEKAT (terlalu rapat) */ { -2.640f, -0.240f, +2.160f },
    /* err PAS                   */ { -2.400f,  0.000f, +2.400f },
    /* err JAUH  (terlalu lebar) */ { -2.160f, +0.240f, +2.640f },
};

// t-norm PERKALIAN, bukan minimum. Dengan dua partisi yang masing-masing
// berjumlah 1, jumlah kesembilan bobot = 1 x 1 = 1 SECARA PASTI. Jadi tidak
// ada pembagian sama sekali di defuzzifikasi -- dan tanpa pembagian, tidak ada
// pembagi yang bisa menormalkan keluaran kecil jadi keluaran penuh. Itulah
// kesalahan yang membuat kendali samar warisan berperilaku bang-bang.
//
// Hasilnya otomatis terkurung antara nilai singleton terkecil dan terbesar,
// jadi tidak perlu clamp sendiri; clamp NAV_WALL_TURN_MAX di hilir tetap ada.
static float samarKemudi(float err, float derr) {
    float me[3], mde[3];
    samarPartisi3(err,  SAMAR_E_C[0],  SAMAR_E_C[1],  SAMAR_E_C[2],  me);
    samarPartisi3(derr, SAMAR_DE_C[0], SAMAR_DE_C[1], SAMAR_DE_C[2], mde);
    float keluar = 0.0f;
    for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 3; j++)
            keluar += me[i] * mde[j] * SAMAR_Z[i][j];
    return keluar;
}

// ==== SUDUT DINDING DARI SEPASANG SENSOR =================================
//
// Selisih waktu sampel terbesar yang masih boleh. Kedua sensor diambil giliran
// oleh round-robin yang sama, jadi bacaannya TIDAK serentak; pada robot yang
// berjalan ~10 cm/detik, 100 ms berarti 1 cm perjalanan -- seukuran dengan
// selisih yang sedang diukur. Di luar batas ini angkanya bukan segitiga,
// melainkan dua keadaan yang berbeda dikurangkan.
static const uint32_t SUDUT_SKEW_MAKS_MS = 100;

// Selisih terbesar antara kedua sensor satu sisi yang masih bisa dijelaskan
// oleh serong badan. atan(4 / 11) = 20 der, di atas rentang kerja wajar.
// Di luar itu salah satu berkas melihat benda lain, bukan dinding yang sama --
// dan sudut yang dihitung darinya bukan sudut badan.
//
// Data arena membuktikan ini perlu: ch1 (kiri belakang) macet di 5-6 cm
// sementara ch0 membaca 16-29 cm. Tanpa penjaga ini pasangan itu melaporkan
// serong 60 derajat yang tidak pernah terjadi.
static const float SISI_BEDA_MAKS_CM = 4.0f;

float Navigation::jarakSisi(bool kiri) {
    const uint8_t idD = kiri ? LIDAR_KIRI_D : LIDAR_KANAN_D;
    const uint8_t idB = kiri ? LIDAR_KIRI_B : LIDAR_KANAN_B;
    float dD = _lidar.jarakHalus(idD);
    float dB = _lidar.jarakHalus(idB);

    if (dD < 0.0f && dB < 0.0f) return -1.0f;
    if (dD < 0.0f) return dB - (kiri ? _biasKiri : _biasKanan);
    if (dB < 0.0f) return dD;                   // sendirian: tak ada pembanding

    return dD;
}

float Navigation::bedaSisi(bool kiri) {
    const uint8_t idD = kiri ? LIDAR_KIRI_D : LIDAR_KANAN_D;
    const uint8_t idB = kiri ? LIDAR_KIRI_B : LIDAR_KANAN_B;
    float dD = _lidar.jarakHalus(idD);
    float dB = _lidar.jarakHalus(idB);
    if (dD < 0.0f || dB < 0.0f) return NAN;    // salah satu MATI/JAUH/mustahil

    uint32_t tD = _lidar.stempelSampel(idD);
    uint32_t tB = _lidar.stempelSampel(idB);
    uint32_t skew = (tD > tB) ? (tD - tB) : (tB - tD);
    if (skew > SUDUT_SKEW_MAKS_MS) return NAN;

    return dB - dD;
}

float Navigation::sudutDinding(bool kiri) {
    float beda = bedaSisi(kiri);
    if (isnan(beda)) return NAN;
    beda -= (kiri ? _biasKiri : _biasKanan);
    // Selisih di luar batas serong wajar berarti salah satu berkas terhalang,
    // dan sudut yang dihitung darinya bukan sudut badan melainkan sudut kaki.
    if (fabsf(beda) > SISI_BEDA_MAKS_CM) return NAN;
    return atan2f(beda, WALL_BASE_CM) * 57.2957795f;
}

void Navigation::kalibrasiSudut() {
    // Dipanggil saat robot SEJAJAR lorong. Apa pun selisih yang terbaca saat
    // itu adalah simpangan pemasangan, bukan sudut badan -- dan simpangan itu
    // harus dikurangkan, kalau tidak robot mengira dirinya menyerong saat lurus.
    float bk = bedaSisi(true);
    float bn = bedaSisi(false);
    if (isnan(bk) && isnan(bn)) {
        Serial.println("Kalibrasi sudut GAGAL: tidak ada sisi yang memberi sepasang bacaan.");
        Serial.println("  Periksa 'l' -- keempat sensor samping harus memberi angka, bukan MATI/jauh.");
        return;
    }
    Serial.println("\n--- KALIBRASI SUDUT DINDING ---");
    Serial.println("  Robot HARUS sedang sejajar lorong saat perintah ini diberikan.");
    if (!isnan(bk)) { _biasKiri  = bk; Serial.print("  bias KIRI  : "); Serial.print(bk, 2); Serial.println(" cm"); }
    else              Serial.println("  bias KIRI  : dilewati (sepasang bacaan tidak lengkap)");
    if (!isnan(bn)) { _biasKanan = bn; Serial.print("  bias KANAN : "); Serial.print(bn, 2); Serial.println(" cm"); }
    else              Serial.println("  bias KANAN : dilewati (sepasang bacaan tidak lengkap)");
    Serial.println("  RAM saja -- ulangi tiap robot menyala.");
}

void Navigation::sudutTabel() {
    Serial.println("\n--- SUDUT BADAN TERHADAP DINDING ---");
    Serial.print("  dasar membujur (WALL_BASE_CM) : "); Serial.print(WALL_BASE_CM, 1);
    Serial.println(" cm");
    for (uint8_t k = 0; k < 2; k++) {
        bool kiri = (k == 0);
        Serial.print(kiri ? "  KIRI  (ch" : "  KANAN (ch");
        Serial.print(kiri ? LIDAR_KIRI_D : LIDAR_KANAN_D); Serial.print(" depan, ch");
        Serial.print(kiri ? LIDAR_KIRI_B : LIDAR_KANAN_B); Serial.print(" belakang) : ");
        float beda = bedaSisi(kiri);
        if (isnan(beda)) {
            Serial.println("TIDAK BISA -- bacaan tidak lengkap atau sampelnya terlalu berjauhan waktu");
        } else {
            float sudut = sudutDinding(kiri);
            Serial.print("beda "); Serial.print(beda, 2);
            Serial.print(" - bias "); Serial.print(biasSisi(kiri), 2);
            Serial.print(" -> "); Serial.print(sudut, 1); Serial.println(" der");
        }
    }
    Serial.println("  + = hidung menyerong MENDEKAT dinding itu.");
    Serial.println("  'Y0' saat robot sejajar lorong untuk mencatat biasnya.");
}

float Navigation::turunanDariSudut(bool kiri) {
    float phi = sudutDinding(kiri);
    if (isnan(phi)) return NAN;
    // Laju maju dari gait, bukan dari perintah: perintah 0,8 belum tentu
    // sudah terwujud, karena GAIT_SLEW_RATE meramp perubahannya.
    float v = _robot.lajuCms();
    return -v * sinf(phi * 0.0174532925f);
}

void Navigation::setTengah(bool ya) {
    if (_tengah == ya) return;
    _tengah = ya;
    Serial.print("Kemudi lateral: ");
    Serial.println(ya ? "MENENGAH (selisih kiri-kanan)" : "IKUT DINDING (satu sisi)");
    if (ya) {
        Serial.println("  AKTIF sekarang. wall.setpoint tidak dipakai -- sasarannya garis tengah lorong.");
        Serial.println("  Salah satu sisi hilang -> otomatis kembali ikut dinding untuk sementara.");
    }
    Serial.println("  Mode ini hidup di RAM: reset Teensy mengembalikannya ke ikut dinding.");
    _errAda = false; _errTurunan = 0.0f; _errStempel = 0;
}

void Navigation::setKemudiMode(uint8_t m) {
    _wallSamar = (m & 1u) != 0;
    _wallSudut = (m & 2u) != 0;
    Serial.print("Kemudi dinding: ");
    Serial.print(_wallSamar ? "SAMAR (fuzzy, 9 aturan)" : "PD (wall.kp / wall.kd)");
    Serial.print(", turunan dari ");
    Serial.println(_wallSudut ? "SUDUT sepasang sensor" : "selisih waktu");
    if (_wallSudut) {
        Serial.println("  Sudut belum dikalibrasi? Beri 'Y0' saat robot sejajar lorong dulu.");
        Serial.println("  Bila sudutnya tidak tersedia, otomatis jatuh kembali ke selisih waktu.");
    }
    Serial.println("  AKTIF sekarang. Hanya pita normal yang ditukar; pita 'terlalu dekat' sama untuk semuanya.");
    Serial.println("  Mode ini hidup di RAM: reset Teensy mengembalikannya ke N0, dan 'W' tidak menyimpannya.");
    // Riwayat turunan milik pita PD; mulai bersih supaya hukum yang baru tidak
    // mewarisi turunan yang dihitung saat hukum lain sedang berjalan.
    _errAda = false; _errTurunan = 0.0f; _errStempel = 0;
}

void Navigation::abaikanDepan(bool ya) {
    if (_abaikanDepan == ya) return;
    _abaikanDepan = ya;
    if (ya) {
        Serial.println("Sensor DEPAN DIABAIKAN -- robot berjalan buta ke depan.");
        Serial.println("  Hanya untuk bidang miring, tempat berkasnya menembak lantai.");
        Serial.println("  Batasi ruasnya dengan odometri ('D<cm>') atau misi. Berhenti apa pun memulihkannya.");
    } else {
        Serial.println("Sensor DEPAN dipakai lagi.");
    }
}

void Navigation::navBerhenti(const char* alasan) {
    // DI ATAS jalan keluar NAV_DIAM: berhenti apa pun -- 's', 'x', Enter, rem
    // jarak, misi gagal -- harus mengembalikan sensor depan, termasuk saat
    // navigasi memang sudah diam.
    abaikanDepan(false);

    // Sudah diam -> tidak ada yang perlu dihentikan. Dulu baris ini hanya
    // menyaring pemanggilan tanpa alasan, sehingga tiap 's'/'x'/Enter mencetak
    // "Navigasi BERHENTI: ..." walau tak ada navigasi yang berjalan. Pemanggil
    // di .ino semuanya sudah punya robot.stop()/disarm() sendiri.
    if (_mode == NAV_DIAM) return;

    bool pivot = (_mode == NAV_PIVOT);
    _mode = NAV_DIAM;
    _majuKini = _turnKini = 0.0f;
    _robot.stop();
    Serial.print(pivot ? "Pivot BERHENTI" : "Navigasi BERHENTI");
    if (alasan) { Serial.print(": "); Serial.println(alasan); } else Serial.println(".");
}

// PD heading memakai gyro Z murni sebagai suku D (sama dengan pivotKe).
// _pivotSign menyerap perbedaan konvensi tanda antara perintah putar dan
// pembacaan yaw IMU -- itulah gunanya kalibrasi 'C'.
float Navigation::kemudiHeading(float targetHeading) const {
    float err = wrap180(targetHeading - _imu.yawDeg());
    float turn = HEADING_KP * err - HEADING_KD * _imu.gyroZ();
    return clampf(_pivotSign * turn, -1.0f, 1.0f);
}

void Navigation::remJarakPasang(float cm) {
    if (cm <= 0.0f) {
        Serial.println("Rem jarak: sasaran harus lebih dari 0 cm. Tidak dipasang.");
        return;
    }
    _robot.jarakNol();
    _remJarakCm = cm;
    Serial.print("Rem jarak DIPASANG di "); Serial.print(cm, 1);
    Serial.println(" cm. Jarak dinolkan.");
    Serial.println("  Berlaku di mode gerak apa pun, termasuk 'w' manual.");
}

void Navigation::remJarakLepas() {
    if (_remJarakCm <= 0.0f) return;    // tidak terpasang -> jangan mencetak apa-apa
    _remJarakCm = 0.0f;
    Serial.println("Rem jarak DILEPAS.");
}

void Navigation::navUpdate() {
    // REM JARAK diperiksa SEBELUM jalan keluar NAV_DIAM di bawah. Kalau
    // ditaruh sesudahnya, 'w' manual tidak akan pernah terkena rem -- dan
    // justru jalan manual itulah satu-satunya cara berjalan lurus tanpa
    // dinding, yaitu pengukuran slip yang paling bersih.
    if (_remJarakCm > 0.0f && _robot.jarakCm() >= _remJarakCm) {
        _remJarakCm = 0.0f;              // sekali pakai; jangan menyala lagi nanti
        navBerhenti("rem jarak tercapai.");   // mengurus mode navigasi
        _robot.stop();                        // mengurus 'w' manual
        Serial.print("Rem jarak: berhenti di "); Serial.print(_robot.jarakCm(), 1);
        Serial.println(" cm.");
    }

    if (_mode == NAV_DIAM) return;

    // Penjaga yang berlaku untuk SEMUA mode.
    if (!_robot.isArmed()) { navBerhenti("servo dilemaskan."); return; }

    // Pivot berdiri sendiri hanya butuh IMU. Cek LiDAR-nya ditaruh SESUDAH ini
    // supaya 'o'/'O' tetap bisa dipakai saat LiDAR belum terpasang -- dulu
    // pivot memang tidak pernah menyentuh LiDAR sama sekali.
    if (_mode == NAV_PIVOT) { pivotUpdate(); return; }

    if (!_lidar.muxTerdeteksi()) { navBerhenti("LiDAR hilang."); return; }

    // Waktu loop TIDAK dipakai untuk turunan PD -- itu sumber masalahnya dulu.
    // Turunan memakai stempel sampel LiDAR (lihat blok kemudi di bawah).
    uint32_t now = millis();

    const bool    ikutKiri  = (_mode == NAV_DINDING_KIRI || _mode == NAV_ARENA_KIRI);
    // sisi = +1 mengikuti dinding KIRI (yaw+ = belok kiri), -1 untuk kanan
    const int8_t  sisi      = ikutKiri ? +1 : -1;
    const uint8_t idSamping = ikutKiri ? LIDAR_KIRI_D : LIDAR_KANAN_D;

    // ---- FASE BELOK (hanya mode terkunci arena) ----
    // Berbelok ke mata angin berikutnya dengan kendali tertutup, bukan
    // berputar buta selama sekian milidetik. Tetap non-blokir: satu langkah
    // per pemanggilan, sama seperti fase jalan.
    if (arenaTerkunci() && _fase == FASE_BELOK) {
        if (_tPivot == 0) _tPivot = now;
        if (now - _tPivot > NAV_PIVOT_BATAS_MS) {
            navBerhenti("belok ke arah arena gagal (timeout).");
            return;
        }
        // Rumus yang sama persis dengan pivot berdiri sendiri -- termasuk
        // dorongan minimal supaya kaki tidak cuma menggeliat di tempat.
        float err;
        float turn = pivotLangkah(_headArah[_arahKini], err);

        _majuKini = 0.0f; _turnKini = turn;
        _robot.walk(0.0f, 0.0f, turn);

        if (fabsf(err) <= HEADING_TOLERANCE_DEG) {
            if (_diamSejak == 0) _diamSejak = now;
            if (now - _diamSejak >= PIVOT_DIAM_MS) {
                _fase = FASE_JALAN;
                _tPivot = 0; _diamSejak = 0;
                // mulai lagi PD dinding dari bersih
                _errAda = false; _errTurunan = 0.0f; _errStempel = 0;
                Serial.print("Navigasi: sudah menghadap "); Serial.println(_arahNama[_arahKini]);
            }
        } else {
            _diamSejak = 0;
        }
        return;
    }

    int depan   = _lidar.getDistance(LIDAR_FRONT);
    int samping = _lidar.getDistance(idSamping);

    // Satu tempat, bukan tiga: dipaksa "jauh" sebelum aturan mana pun
    // membacanya, sehingga mati/halangan/melambat semuanya ikut mati.
    if (_abaikanDepan) depan = LIDAR_JAUH;

    // 1) Sensor depan putus -> jangan pernah berjalan buta ke depan.
    if (depan == LIDAR_MATI) { navBerhenti("sensor DEPAN tidak merespons."); return; }

    // 2) Halangan di depan -> berputar MENJAUHI dinding yang diikuti.
    if (depan != LIDAR_JAUH && depan <= FRONT_STOP_CM) {
        if (arenaTerkunci()) {
            // Ikut dinding KIRI -> saat mentok, belok KANAN = +90 der searah
            // jarum jam = indeks arah berikutnya. Ikut dinding KANAN -> -1.
            _arahKini = arahGeser(_arahKini, ikutKiri ? +1 : -1);
            _fase = FASE_BELOK;
            _tPivot = 0; _diamSejak = 0;
            _robot.stop();
            Serial.print("Navigasi: halangan depan -> belok ke ");
            Serial.println(_arahNama[_arahKini]);
            return;
        }
        if (_tBelok == 0) _tBelok = now;
        if (now - _tBelok > NAV_BELOK_BATAS_MS) {
            navBerhenti("terjebak -- berbelok terlalu lama tanpa jalan keluar.");
            return;
        }
        _majuKini = 0.0f;
        _turnKini = -sisi * NAV_BELOK_CMD;
        _robot.walk(0.0f, 0.0f, _turnKini);
        return;
    }
    _tBelok = 0;

    // 3) Kecepatan maju, diturunkan mulus saat mendekati halangan.
    float maju = NAV_FWD_SPEED;
    if (depan != LIDAR_JAUH && depan < NAV_PELAN_CM) {
        float k = (float)(depan - FRONT_STOP_CM) /
                  (float)(NAV_PELAN_CM - FRONT_STOP_CM);
        maju = NAV_FWD_SPEED * clampf(k, NAV_MAJU_MIN, 1.0f);
    }

    // 4) Kemudi terhadap dinding samping. DUA PITA, bukan satu rumus PD.
    //
    // Satu gain proporsional tidak bisa memenuhi dua kebutuhan yang berlawanan:
    // lembut saat dinding jauh (supaya kemudi tidak menjenuh dan robot tidak
    // memutar menghadap dinding) DAN tegas saat terlalu dekat. Dengan
    // wall.kp 0,008, berada 10 cm terlalu dekat hanya menghasilkan koreksi
    // 0,08 dari 1,00 -- itulah sebabnya kaki sempat menggesek dinding.
    // Karena itu pita dekat dipisah jadi aturannya sendiri.
    //
    //   jarak < wall.min  -> TERLALU DEKAT: putar menjauh dengan kekuatan tetap
    //                        + kurangi laju maju.
    //   selebihnya        -> PD normal terhadap wall.setpoint.
    //
    // Dulu ada pita KETIGA di sini (wall.hantu) untuk membuang bacaan yang
    // mustahil. Itu sudah pindah ke LidarArray lewat LIDAR_MIN_CM, karena
    // masalahnya bukan milik ikut-dinding saja: sensor DEPAN punya hantu yang
    // sama dan tidak pernah terlindungi di sini. Sekarang bacaan mustahil
    // sampai ke mari sebagai LIDAR_JAUH, dan ditangani cabang "dinding hilang".
    float turn;
    _pitaDekat = false;
    if (samping == LIDAR_MATI) {
        navBerhenti("sensor SAMPING tidak merespons.");
        return;
    } else if (samping == LIDAR_JAUH) {
        // Dinding hilang: tikungan keluar, mulut lorong, atau -- di robot ini --
        // sensor yang jatuh ke bacaan hantu. Membelok ke arah dinding dengan
        // kekuatan tetap sampai ketemu lagi. Turunan di-reset supaya tidak
        // melonjak saat dinding muncul kembali.
        turn = sisi * NAV_CARI_CMD;
        _errAda = false; _errTurunan = 0.0f; _errStempel = 0;

        // Jangan mencari SELAMANYA. Perintah putar tetap tanpa dinding yang
        // pernah muncul lagi = robot berjalan melingkar di tengah arena, dan
        // itu terlihat persis seperti "robot jalan sendiri tanpa alasan".
        // Di mode arena tidak perlu: kemudiHeading() yang mengunci arah, jadi
        // dinding hilang di sana tidak membuatnya melingkar.
        if (!arenaTerkunci()) {
            if (_tCari == 0) _tCari = now;
            else if (now - _tCari > NAV_CARI_BATAS_MS) {
                navBerhenti("dinding samping hilang terlalu lama -- cek sensor samping ('l').");
                return;
            }
        }
    } else {
        // Jarak float (belum dibulatkan ke cm) + stempel waktu sampelnya.
        _tCari = 0;                                    // dinding ketemu lagi
        float jarak = jarakSisi(ikutKiri);
        if (jarak < 0.0f) jarak = (float)samping;      // jaga-jaga, tak boleh terjadi
        uint32_t stempel = _lidar.stempelSampel(idSamping);

        // MENENGAH. Dua tanda yang berbeda, dan menyamakannya keliru:
        //   sisiPD   selalu -1, karena err = (kanan - kiri)/2 sudah membawa
        //            arahnya sendiri -- kanan lebih jauh berarti geser kanan.
        //   sisiDekat menunjuk dinding mana yang TERLALU DEKAT, dan itu bisa
        //            sisi mana pun tanpa bergantung pada tanda err.
        int8_t sisiPD = sisi, sisiDekat = sisi;
        float errTengah = 0.0f;
        bool  adaTengah = false;
        if (_tengah) {
            float dk = jarakSisi(true);
            float dn = jarakSisi(false);
            if (dk >= 0.0f && dn >= 0.0f) {
                adaTengah = true;
                jarak      = fminf(dk, dn);          // pita dekat memakai yang terdekat
                sisiDekat  = (dk < dn) ? +1 : -1;
                sisiPD     = -1;
                errTengah  = (dn - dk) * 0.5f;
            }
        }

        // --- SATU RUMUS, BUKAN DUA PITA YANG BERSAMBUNG PATAH ---
        //
        // Dulu ini 'if (jarak < wall.min) dorong; else PD;'. Tepat di ambang
        // itu keluarannya MELOMPAT: di sisi PD, err = 13 - 17 memberi -0,03;
        // di sisi dorong, 0,5 x NAV_WALL_TURN_MAX memberi 0,25. Lompatan 0,28
        // pada satu batas yang tidak punya histeresis sama sekali.
        //
        // Akibatnya relay: robot yang kebetulan duduk DI SEKITAR wall.min
        // menyeberangi batas itu bolak-balik, dan tiap penyeberangan membalik
        // perintah kemudi. Data arena menunjukkannya langsung -- sensor kanan
        // membaca 10,11,13,11,12,13,13,12,13,14,13,13,14 cm, berayun persis di
        // sekitar wall.min 13.
        //
        // Sekarang keduanya DILEBUR: dalam = 0 di ambang (hasilnya PD murni,
        // jadi sambungannya mulus) dan 1 saat kaki menyentuh dinding (hasilnya
        // dorongan penuh). Tidak ada lagi batas untuk diseberangi.
        float err = adaTengah ? errTengah : (jarak - WALL_SETPOINT_CM);

        if (!_errAda) {
            _errPrev = err; _errStempel = stempel; _errTurunan = 0.0f; _errAda = true;
        } else if (stempel != _errStempel) {
            // Sampel BARU -> perbarui turunan, memakai jarak waktu antar sampel
            // yang sebenarnya. Dulu pembaginya dt loop, yang dijepit di 0,001 s
            // -- satu lompatan pembulatan 1 cm jadi bernilai 10,0 satuan putar.
            float dts = (float)(uint32_t)(stempel - _errStempel) / 1000.0f;
            _errTurunan = (dts > 0.005f && dts < 0.5f) ? (err - _errPrev) / dts : 0.0f;
            _errPrev = err; _errStempel = stempel;
        }
        // di antara sampel: _errTurunan ditahan, bukan dinolkan.
        // Sumber turunan: yang dari sudut seketika, yang dari waktu baru
        // berarti sesudah dua sampel LiDAR berurutan. Sudut yang tidak
        // tersedia jatuh kembali ke selisih waktu, tidak menghentikan robot.
        float derr = _errTurunan;
        if (_wallSudut) {
            float ds = turunanDariSudut(ikutKiri);
            if (!isnan(ds)) derr = ds;
        }
        float pd = sisiPD * (_wallSamar ? samarKemudi(err, derr)
                                        : (WALL_KP * err + WALL_KD * derr));

        float lebar = WALL_MIN_CM - WALL_KAKI_CM;
        if (lebar < 1.0f) lebar = 1.0f;             // jaga-jaga bila disetel rapat
        float dalam = clampf((WALL_MIN_CM - jarak) / lebar, 0.0f, 1.0f);
        float dorong = -sisiDekat * NAV_WALL_TURN_MAX;
        turn = (1.0f - dalam) * pd + dalam * dorong;

        if (dalam > 0.0f) {
            _pitaDekat = true;
            // Melambat supaya kemudi sempat bekerja sebelum kaki sampai ke
            // dinding. Tanpa ini robot menyeret kakinya sambil mengoreksi.
            maju *= (1.0f - 0.5f * dalam);

        }
    }

    // Sumbangan dinding dibatasi di SEMUA mode, bukan hanya mode arena.
    // Tanpa batas ini WALL_KP*err menjenuh ke +-1,00 begitu dinding lebih jauh
    // dari (1/WALL_KP + setpoint) -- robot memutar PENUH menghadap dinding
    // alih-alih menggeser mendekat. Di lorong 60 cm dengan gain lama, simulasi
    // menunjukkan perintah putar jenuh 61% waktu saat mulai dari tengah.
    turn = clampf(turn, -NAV_WALL_TURN_MAX, NAV_WALL_TURN_MAX);

    if (arenaTerkunci()) {
        // Dinding mengoreksi posisi LATERAL; arah hadap diurus heading arena.
        turn += kemudiHeading(_headArah[_arahKini]);
    }
    turn = clampf(turn, -1.0f, 1.0f);

    _majuKini = maju;
    _turnKini = turn;
    _robot.walk(maju, 0.0f, turn);
}

void Navigation::navStatus() {
    Serial.println("\n--- STATUS NAVIGASI ---");
    Serial.print("  mode        : ");
    switch (_mode) {
        case NAV_DIAM:          Serial.println("DIAM"); break;
        case NAV_DINDING_KIRI:  Serial.println("ikut dinding KIRI"); break;
        case NAV_DINDING_KANAN: Serial.println("ikut dinding KANAN"); break;
        case NAV_ARENA_KIRI:    Serial.println("ikut dinding KIRI + kunci arena"); break;
        case NAV_ARENA_KANAN:   Serial.println("ikut dinding KANAN + kunci arena"); break;
        case NAV_PIVOT:         Serial.println("PIVOT di tempat"); break;
    }

    if (_mode == NAV_PIVOT) {
        Serial.print("  fase        : ");
        Serial.println(_fase == FASE_SETTLE ? "SETTLE (kaki menenangkan diri)" : "BERPUTAR");
        Serial.print("  target      : "); Serial.print(_pivotTarget, 1); Serial.println(" der");
        Serial.print("  yaw & error : "); Serial.print(_imu.yawDeg(), 1);
        Serial.print(" der, simpang ");
        Serial.print(wrap180(_pivotTarget - _imu.yawDeg()), 1);
        Serial.println(" der");
    }
    if (arenaTerkunci() && _arahKini >= 0) {
        Serial.print("  fase        : ");
        Serial.println(_fase == FASE_JALAN ? "JALAN" : "BELOK");
        Serial.print("  arah dituju : "); Serial.print(_arahNama[_arahKini]);
        Serial.print(" ("); Serial.print(_headArah[_arahKini], 1); Serial.println(" der)");
        Serial.print("  yaw & error : "); Serial.print(_imu.yawDeg(), 1);
        Serial.print(" der, simpang ");
        Serial.print(wrap180(_headArah[_arahKini] - _imu.yawDeg()), 1);
        Serial.println(" der");
    }

    int depan = _lidar.getDistance(LIDAR_FRONT);
    int kiri  = _lidar.getDistance(LIDAR_KIRI_D);
    int kanan = _lidar.getDistance(LIDAR_KANAN_D);
    const char* lbl[3] = {"depan", "kiri ", "kanan"};
    int val[3] = {depan, kiri, kanan};
    for (uint8_t i = 0; i < 3; i++) {
        Serial.print("  "); Serial.print(lbl[i]); Serial.print("       : ");
        if      (val[i] == LIDAR_MATI) Serial.println("MATI");
        else if (val[i] == LIDAR_JAUH) Serial.println("jauh");
        else { Serial.print(val[i]); Serial.println(" cm"); }
    }
    // Ketiga ambang dicetak bersama supaya bacaan samping di atas bisa langsung
    // dibandingkan tanpa mengingat-ingat isi 'q'.
    Serial.print("  pita dinding: dekat <"); Serial.print(WALL_MIN_CM, 1);
    Serial.print(" | setpoint ");             Serial.print(WALL_SETPOINT_CM, 1);
    Serial.println(" cm");
    Serial.print("  batas mustahil: samping <"); Serial.print(LIDAR_MIN_CM[LIDAR_KANAN_D]);
    Serial.print(" | depan <");                  Serial.print(LIDAR_MIN_CM[LIDAR_FRONT]);
    Serial.println(" cm -> dilaporkan 'jauh', bukan halangan");
    Serial.print("  lateral     : ");
    Serial.println(_tengah ? "MENENGAH (selisih kiri-kanan)" : "IKUT DINDING (satu sisi)");
    Serial.print("  kemudi      : ");
    Serial.print(_wallSamar ? "SAMAR (fuzzy)" : "PD");
    Serial.print(", turunan dari ");
    Serial.println(_wallSudut ? "SUDUT" : "waktu");
    if (_pitaDekat) Serial.println("  !! TERLALU DEKAT -- sedang memutar menjauhi dinding");
    if (_abaikanDepan) Serial.println("  !! SENSOR DEPAN DIABAIKAN -- berjalan buta ke depan ('i0' memulihkan)");
    Serial.print("  berhenti di : "); Serial.print(FRONT_STOP_CM);    Serial.println(" cm");
    Serial.print("  perintah    : maju "); Serial.print(_majuKini, 2);
    Serial.print("  putar ");              Serial.println(_turnKini, 2);
    Serial.print("  sensor hidup: ");      Serial.print(_lidar.jumlahHidup());
    Serial.print(" dari ");                Serial.println(NUM_LIDAR);
}
