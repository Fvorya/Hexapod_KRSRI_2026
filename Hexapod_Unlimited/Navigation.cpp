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

    if (_lidar.getDistance(LIDAR_FRONT) == LIDAR_MATI) {
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
void Navigation::navBerhenti(const char* alasan) {
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
        float jarak = _lidar.jarakHalus(idSamping);
        if (jarak < 0.0f) jarak = (float)samping;      // jaga-jaga, tak boleh terjadi
        uint32_t stempel = _lidar.stempelSampel(idSamping);

        if (jarak < WALL_MIN_CM) {
            // TERLALU DEKAT. Kekuatan menjauh naik dari separuh di ambang
            // wall.min sampai PENUH tepat di LIDAR_MIN_CM sensor itu -- yaitu
            // di jarak saat kaki sudah menyentuh dinding, batas bawah yang
            // sama yang dipakai LidarArray. Lebar ramp-nya menyesuaikan sendiri
            // kalau wall.min disetel, jadi tidak ada angka ketiga yang bisa
            // lupa ikut diubah. Hasilnya tetap dibatasi NAV_WALL_TURN_MAX --
            // ini koreksi lateral, bukan izin untuk berputar di tempat.
            float lebar = WALL_MIN_CM - (float)LIDAR_MIN_CM[idSamping];
            if (lebar < 1.0f) lebar = 1.0f;         // jaga-jaga bila disetel rapat
            float dalam = clampf((WALL_MIN_CM - jarak) / lebar, 0.0f, 1.0f);
            turn = -sisi * NAV_WALL_TURN_MAX * (0.5f + 0.5f * dalam);
            _pitaDekat = true;
            // Melambat supaya kemudi sempat bekerja sebelum kaki sampai ke
            // dinding. Tanpa ini robot menyeret kakinya sambil mengoreksi.
            maju *= (1.0f - 0.5f * dalam);
            // PD dimulai bersih saat keluar dari pita ini, kalau tidak turunan
            // melonjak dari lompatan error antar-pita.
            _errAda = false; _errTurunan = 0.0f; _errStempel = 0;
        } else {
            float err = jarak - WALL_SETPOINT_CM;   // + = terlalu jauh

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
            // di antara sampel: _errTurunan ditahan, bukan dinolkan
            turn = sisi * (WALL_KP * err + WALL_KD * _errTurunan);
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
    if (_pitaDekat) Serial.println("  !! TERLALU DEKAT -- sedang memutar menjauhi dinding");
    Serial.print("  berhenti di : "); Serial.print(FRONT_STOP_CM);    Serial.println(" cm");
    Serial.print("  perintah    : maju "); Serial.print(_majuKini, 2);
    Serial.print("  putar ");              Serial.println(_turnKini, 2);
    Serial.print("  sensor hidup: ");      Serial.print(_lidar.jumlahHidup());
    Serial.print(" dari ");                Serial.println(NUM_LIDAR);
}
