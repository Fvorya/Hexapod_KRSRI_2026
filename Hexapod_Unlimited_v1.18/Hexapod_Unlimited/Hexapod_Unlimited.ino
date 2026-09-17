#include <Arduino.h>
#include "config.h"
#include "Calib.h"      // WAJIB: gParam/gOffset/gTrim/gInvert (pulse min-max, GAIT_*, dll)
#include "Skor.h"       // pembukuan poin menurut tabel penilaian guidebook
#include "Tampilan.h"   // OLED + empat tombol D2..D5
#include "EEMap.h"      // peta EEPROM + penjaga static_assert
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"
#include "Misi.h"

// ====================================================================
// DEKLARASI OBJEK GLOBAL
// ====================================================================
Imu imu;
Hexapod robot;                // Digunakan oleh Navigation
LidarArray lidar;              // 6x VL53L1X lewat mux TCA9548A di bus Wire
Navigation nav(imu, robot, lidar);  // Menyuntikkan referensi IMU, Motion, LiDAR
// Lapisan misi. Ia MENYETIR nav, tidak menggantikannya -- lihat Misi.h.
// Lintasannya DATA (tabel RUAS[] di Misi.cpp), bukan satu state per potongan.
Misi misi(robot, nav, lidar);

// Pembukuan poin. Global karena Misi::ruasBerikut() yang mencatatnya, dan
// menyuntikkannya lewat konstruktor berarti menyentuh empat berkas untuk satu
// angka yang tidak pernah punya lebih dari satu contoh.
Skor gSkor;
Tampilan tampilan;

// Jembatan tombol -> parser perintah, supaya tombol tidak punya jalur kedua
// ke dalam Misi. handleCmd() didefinisikan jauh di bawah.
static void handleCmd(char* s);
static void kirimDariTombol(char* s) { handleCmd(s); }

// ====================================================================
// DEMO BODY KINEMATICS (non-blokir)
// Menyapu 6 sumbu berurutan: roll, pitch, yaw, geser X, Y, Z.
// Tiap sumbu satu putaran sinus penuh (0 -> + -> 0 -> - -> 0) supaya selalu
// kembali ke netral sebelum pindah sumbu -- tak pernah ada lompatan.
// Kaki TETAP DI TEMPAT; yang bergerak hanya badan. Itulah gunanya: kalau
// telapak ikut bergeser di lantai, berarti body kinematics belum benar.
// PAGAR GERAK MANUAL.
//
// Rem jarak "D<cm>" TIDAK bisa dipakai di sini, dan itu bukan soal selera:
// HexaGait::jarakMm hanya menghitung komponen MAJU, jadi (a) geser samping
// murni tidak menambahnya sama sekali, dan (b) gerak MUNDUR justru
// MENGURANGINYA -- rem yang membandingkan "jarak >= sasaran" tak akan pernah
// menggigit, dan robot mundur terus. Dua-duanya sudah terbukti di lantai.
//
// Yang dipakai sebagai gantinya: SENSOR YANG MENGHADAP ARAH JALANNYA. Maju
// dijaga sensor depan, mundur oleh sensor belakang, kepiting oleh pasangan
// sensor sisi yang dituju. Ia tidak peduli tanda, tidak peduli sumbu, dan
// mengukur jarak yang sebenarnya alih-alih menghitung langkah.
//
// Batas waktu tetap ada sebagai jaring terakhir, untuk keadaan yang tidak
// bisa dilihat sensor mana pun: sensor mati, atau robot menyangkut sehingga
// jaraknya tidak pernah berubah.
#define GERAK_AMAN_CM 15   // sama dengan pita "terlalu dekat" milik navigasi
static uint32_t gerakSampai = 0;   // millis() saat gerak manual harus berhenti
static float    gerakMaju   = 0.0f;  // tanda menentukan sensor mana yang menjaga
static float    gerakGeser  = 0.0f;

// Jarak ke arah yang sedang dituju, atau -1 bila tak ada yang bisa dinilai.
// Pasangan sensor sisi diambil yang TERDEKAT: yang menabrak duluan bisa
// dudukan depan maupun belakang, tergantung badan sedang menyerong ke mana.
static int jarakArahJalan() {
    int paling = -1;
    auto pakai = [&](int d) {
        if (d == LIDAR_MATI || d == LIDAR_JAUH) return;
        if (paling < 0 || d < paling) paling = d;
    };
    if (gerakMaju  > 0.0f) pakai(lidar.getDistance(LIDAR_FRONT));
    if (gerakMaju  < 0.0f) pakai(lidar.getDistance(LIDAR_BACK));
    if (gerakGeser > 0.0f) { pakai(lidar.getDistance(LIDAR_KANAN_D));
                             pakai(lidar.getDistance(LIDAR_KANAN_B)); }
    if (gerakGeser < 0.0f) { pakai(lidar.getDistance(LIDAR_KIRI_D));
                             pakai(lidar.getDistance(LIDAR_KIRI_B)); }
    return paling;
}


// Urutan boot ada di bawah parser, tapi 's' harus bisa membatalkannya --
// dan prototipe otomatis Arduino tidak menjangkau fungsi 'static'. Ikut
// dipagari DEMO_BOOT: seluruh mesin boot hilang saat saklarnya 0, dan
// prototipe tanpa definisi gagal di tahap LINK, bukan di tahap compile.
#if DEMO_BOOT
static void bootBatal(const char* alasan);
#endif

static bool     demoOn    = false;
static uint32_t demoT0    = 0;
static int8_t   demoAxis  = -1;

static const char* const DEMO_NAMA[6] = {
    "ROLL  (miring kanan-kiri)", "PITCH (dongak-tunduk)", "YAW   (putar badan)",
    "GESER X (kanan-kiri)",      "GESER Y (maju-mundur)", "GESER Z (naik-turun)"
};

static void demoStop(const char* alasan) {
    demoOn = false; demoAxis = -1;
    robot.setBodyRotation(0, 0, 0);
    robot.setBodyTranslation(0, 0, 0);
    Serial.print("Demo body kinematics berhenti: "); Serial.println(alasan);
}

static void demoUpdate() {
    if (!demoOn) return;

    float t = (millis() - demoT0) / 1000.0f;
    int8_t axis = (int8_t)(t / BODY_DEMO_PHASE_S);
    if (axis >= 6) { demoStop("selesai, pose dinolkan."); return; }

    if (axis != demoAxis) {
        demoAxis = axis;
        Serial.print("  sumbu "); Serial.print(axis + 1);
        Serial.print("/6 : ");   Serial.println(DEMO_NAMA[axis]);
    }

    // sinus penuh dalam satu fase -> mulai & berakhir tepat di nol
    float k = sinf(2.0f * (float)M_PI * (t - axis * BODY_DEMO_PHASE_S) / BODY_DEMO_PHASE_S);
    float rot = BODY_DEMO_ROT_DEG   * k;
    float tr  = BODY_DEMO_TRANS_MM  * k;

    float r = 0, p = 0, y = 0, tx = 0, ty = 0, tz = 0;
    switch (axis) {
        case 0: r  = rot;        break;
        case 1: p  = rot;        break;
        case 2: y  = rot;        break;
        case 3: tx = tr;         break;
        case 4: ty = tr;         break;
        case 5: tz = tr * 0.6f;  break;   // vertikal lebih pendek: paling mudah mentok
    }
    robot.setBodyRotation(r, p, y);
    robot.setBodyTranslation(tx, ty, tz);
}

// ====================================================================
// ALIRAN YAW KE SERIAL MONITOR (non-blokir)
// Dicetak berkala selama dihidupkan, supaya bisa mengamati heading sambil
// memutar robot dengan tangan -- berguna saat mencatat arah arena ('c0'..'c3')
// dan saat memeriksa apakah IMU melayang (drift).
// ====================================================================
static bool     yawOn     = false;
static uint32_t yawT      = 0;
static uint16_t yawJeda   = 200;    // ms antar cetakan (default 5x per detik)

static void yawStreamUpdate() {
    if (!yawOn) return;
    if (millis() - yawT < yawJeda) return;
    yawT = millis();

    if (!imu.hasData()) {
        Serial.println("yaw: menunggu data IMU (cek kabel Serial2 & baud 230400)...");
        return;
    }

    float y = imu.yawDeg();
    Serial.print("yaw ");        Serial.print(y, 1);
    Serial.print(" | roll ");    Serial.print(imu.rollDeg(), 1);
    Serial.print(" | pitch ");   Serial.print(imu.pitchDeg(), 1);
    Serial.print(" der | gyroZ ");
    Serial.print(imu.gyroZ(), 1); Serial.print(" der/s");

    // accelZ MENTAH, bukan hasil tare. Inilah satu-satunya angka yang tahu
    // papan IMU menghadap ke mana: +1 g berarti tegak, -1 g berarti TERBALIK.
    // roll dan pitch tidak bisa menjawabnya -- keduanya relatif terhadap tare(),
    // jadi papan yang terpasang terbalik pun melapor 0 der sesudah ditare, dan
    // yang rusak cuma kompensasi kemiringan di dalam fusi WIT: heading jadi
    // menyusut dan tidak berulang, tanpa satu pun gejala di roll/pitch.
    //
    // magMagnitude juga mentah, satuan cacahan sensor. Nilai mutlaknya tidak
    // berarti; yang berarti PERUBAHANNYA saat robot diam.
    Serial.print(" | az "); Serial.print(imu.accelZ(), 2); Serial.print(" g");
    Serial.print(" | mag "); Serial.print(imu.magMagnitude(), 0);

    // Kalau arah arena sudah dicatat, tunjukkan yang terdekat + simpangannya.
    // Inilah yang membuat aliran ini berguna saat kalibrasi kompas.
    float selisih = 0.0f;
    int8_t idx = nav.arahTerdekat(y, selisih);
    if (idx >= 0) {
        Serial.print(" | terdekat ");   Serial.print(nav.namaArah((uint8_t)idx));
        Serial.print(" (simpang ");     Serial.print(selisih, 1);
        Serial.print(" der)");
    } else {
        Serial.print(" | arah arena belum dicatat");
    }
    Serial.println();
}

// ====================================================================
// GOYANG ROLL BERGELOMBANG (non-blokir) -- untuk pajangan
// Kaki TETAP MENAPAK; hanya badan yang mengayun, sama seperti demo 'B'.
// Bedanya: 'B' menyapu enam sumbu sekali jalan lalu berhenti, sedangkan ini
// berayun terus sampai dihentikan, dengan amplitudo & periode yang bisa diatur.
//
// Fase pitch opsional membuatnya jadi gelombang berputar, bukan metronom:
// pitch = amp*sin(wt + fase). Fase 90 der membuat badan menelusuri kerucut.
// ====================================================================
static bool     goyangOn   = false;
static uint32_t goyangT0   = 0;
static float    goyangAmp  = 12.0f;   // derajat
static float    goyangPer  = 2.0f;    // detik per siklus penuh
static float    goyangFase = 0.0f;    // beda fase pitch, derajat (0 = roll murni)

static void goyangStop(const char* alasan) {
    if (!goyangOn) return;
    goyangOn = false;
    robot.setBodyRotation(0, 0, 0);    // di-ramp pulang, bukan dilepas mendadak
    Serial.print("Goyang roll berhenti: "); Serial.println(alasan);
}

static void goyangUpdate() {
    if (!goyangOn) return;
    float t = (millis() - goyangT0) / 1000.0f;
    float w = 2.0f * (float)M_PI / goyangPer;
    float roll  = goyangAmp * sinf(w * t);
    float pitch = (fabsf(goyangFase) > 0.01f)
                  ? goyangAmp * sinf(w * t + deg2rad(goyangFase))
                  : 0.0f;
    robot.setBodyRotation(roll, pitch, 0.0f);
}

// Aliran LiDAR ke Serial Monitor, pola sama dengan aliran yaw di atas.
static bool     lidOn   = false;
static uint32_t lidT    = 0;
static uint16_t lidJeda = 300;

static void lidarStreamUpdate() {
    if (!lidOn) return;
    if (millis() - lidT < lidJeda) return;
    lidT = millis();
    lidar.cetakBaris();
}

// Ambil sampai maxn bilangan (boleh desimal & negatif) dari "r10 -5 0".
static uint8_t argFloats(const char* s, float* out, uint8_t maxn) {
    uint8_t n = 0;
    const char* p = s + 1;
    while (n < maxn) {
        while (*p == ' ' || *p == ',') p++;
        if (!*p) break;
        char* end;
        float v = strtof(p, &end);
        if (end == p) break;
        out[n++] = v;
        p = end;
    }
    return n;
}

// ====================================================================
// PARAMETER KALIBRASI ('q' lihat, 'Q' ubah, 'W' simpan)
// Menyambungkan Calib::findParam()/setParam()/save() yang selama ini ada
// tapi tidak pernah dipanggil siapa pun -- akibatnya tiap penyetelan gain
// menuntut kompilasi ulang DAN kenaikan CALIB_VERSION.
// ====================================================================

static const char* namaBerlaku(ParamBerlaku b) {
    switch (b) {
        case P_LANGSUNG:     return "langsung";
        case P_PERLU_B:      return "ketik 'b'";
        case P_SERVO_LEMAS:  return "servo lemas";
        default:             return "belum dipakai";
    }
}

// Ambil satu token (sampai spasi) dari s+1. Mengembalikan penunjuk ke sisa
// string sesudah token, atau nullptr bila tidak ada token.
static const char* ambilToken(const char* s, char* out, uint8_t maks) {
    const char* p = s + 1;
    while (*p == ' ') p++;
    if (!*p) return nullptr;
    uint8_t n = 0;
    while (*p && *p != ' ' && *p != '=' && n < maks - 1) out[n++] = *p++;
    out[n] = 0;
    return n ? p : nullptr;
}

// Cocok persis dulu (Calib::findParam), lalu awalan yang unik supaya tidak
// perlu mengetik "gait.profile_tau" lengkap di Serial Monitor.
// -1 = tidak ketemu, -2 = ambigu (kandidatnya sudah dicetak).
static int cariParam(const char* nama) {
    int i = Calib::findParam(nama);
    if (i >= 0) return i;

    int ketemu = -1, n = 0;
    size_t L = strlen(nama);
    for (int k = 0; k < N_PARAMS; k++)
        if (strncmp(nama, PARAM_DEFS[k].name, L) == 0) { ketemu = k; n++; }

    if (n == 1) return ketemu;
    if (n > 1) {
        Serial.print("Awalan '"); Serial.print(nama); Serial.println("' ambigu:");
        for (int k = 0; k < N_PARAMS; k++)
            if (strncmp(nama, PARAM_DEFS[k].name, L) == 0) {
                Serial.print("   "); Serial.println(PARAM_DEFS[k].name);
            }
        return -2;
    }
    return -1;
}

static void cetakSatuParam(int i) {
    const ParamDef& d = PARAM_DEFS[i];
    float v = gParam[i];
    Serial.print("  ");
    Serial.print(d.name);
    for (size_t k = strlen(d.name); k < 18; k++) Serial.print(' ');
    Serial.print(v, 3);
    Serial.print(fabsf(v - d.def) > 1e-6f ? " *" : "  ");
    Serial.print("  def "); Serial.print(d.def, 3);
    Serial.print("  [ ");  Serial.print(d.lo, 2);
    Serial.print(" .. ");  Serial.print(d.hi, 2);
    Serial.print(" ]  ");  Serial.println(namaBerlaku(d.berlaku));
}

static void cetakSemuaParam() {
    Serial.println("\n--- PARAMETER KALIBRASI (EEPROM 0) ---");
    Serial.println("  nama              nilai      default      rentang        berlaku");
    for (int i = 0; i < N_PARAMS; i++) cetakSatuParam(i);
    Serial.println("  '*' = berbeda dari default");
    Serial.println("  Q<nama> <nilai> untuk mengubah, W untuk menyimpan ke EEPROM.");
    Serial.println("  Nama boleh disingkat selama awalannya unik (mis. Qwall.kp 0.012).");
}

static void cetakPoseBadan() {
    Vec3 rt = robot.bodyRotTargetDeg(), tt = robot.bodyTransTarget();
    Serial.print("Pose badan : roll "); Serial.print(rt.x, 1);
    Serial.print("  pitch ");           Serial.print(rt.y, 1);
    Serial.print("  yaw ");             Serial.print(rt.z, 1);
    Serial.println(" der");
    Serial.print("Geser badan: x "); Serial.print(tt.x, 1);
    Serial.print("  y ");            Serial.print(tt.y, 1);
    Serial.print("  z ");            Serial.print(tt.z, 1);
    Serial.println(" mm");

    // Pose di-RAMP (BODY_SLEW_DEG_S / BODY_SLEW_MM_S), jadi tepat sesudah
    // perintah angka di atas adalah SASARAN, bukan yang sudah tercapai.
    if (!robot.bodyPoseSampai()) {
        Serial.print("             (menuju sasaran; sekarang roll ");
        Serial.print(robot.bodyRollDeg(), 1);  Serial.print(" pitch ");
        Serial.print(robot.bodyPitchDeg(), 1); Serial.print(" yaw ");
        Serial.print(robot.bodyYawDeg(), 1);   Serial.println(")");
        Serial.println("             Peringatan jangkauan IK muncul saat sasaran tercapai.");
    } else if (!robot.lastPoseInRange()) {
        Serial.println("!! Ada kaki di luar jangkauan IK -- sudut sudah di-clamp.");
    }
}

// ====================================================================
// PARSER PERINTAH SERIAL
// ====================================================================
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char c = *s;

    // Alat bantu ekstrak angka dari perintah
    bool hasNum = (s[1] >= '0' && s[1] <= '9') || s[1] == '-';
    int  v      = atoi(s + 1);
    uint8_t d1  = (uint8_t)(s[1] - '0'); // Ekstrak digit pertama (0-9)

    switch (c) {
        // --- 1. NAVIGASI: KOMPAS ARENA ---
        case 'c':
            if (d1 > 3) {
                Serial.println("c0 = arah LORONG PERTAMA dari HOME (bukan utara magnet).");
                Serial.println("c1 c2 c3 = seperempat putaran searah jarum jam dari c0.");
                Serial.println("Nama UTARA/TIMUR/SELATAN/BARAT hanya panggilan indeks 0..3.");
                break;
            }
            nav.kompasCatat(d1);
            break;

        case 'k':
            nav.kompasTabel();
            break;

        case 'K':   // Tabel kalibrasi gerak (pivot & odometri) dari EEPROM 2048
            nav.gerakTabel();
            break;

        case 'S':   // Simpan hasil 'C' ke EEPROM 2048 (baca-ubah-tulis)
            nav.gerakSimpan();
            break;

        case 'y': {  // Hidup/matikan aliran yaw. y<ms> untuk mengatur jedanya.
            float p[1] = {0};
            if (argFloats(s, p, 1) >= 1) {
                yawJeda = (uint16_t)clampf(p[0], 50.0f, 5000.0f);
                yawOn = true; yawT = 0;
                Serial.print("Aliran yaw HIDUP, tiap "); Serial.print(yawJeda);
                Serial.println(" ms. Ketik 'y' lagi untuk berhenti.");
            } else {
                yawOn = !yawOn; yawT = 0;
                Serial.println(yawOn ? "Aliran yaw HIDUP. Ketik 'y' lagi untuk berhenti."
                                     : "Aliran yaw berhenti.");
            }
            break;
        }

        case 'l':   // Tabel jarak keenam LiDAR (sekali cetak)
            lidar.cetakTabel();
            break;

        case 'L': {  // Hidup/matikan aliran LiDAR. L<ms> untuk mengatur jedanya.
            float p[1] = {0};
            if (argFloats(s, p, 1) >= 1) {
                lidJeda = (uint16_t)clampf(p[0], 50.0f, 5000.0f);
                lidOn = true; lidT = 0;
                Serial.print("Aliran LiDAR HIDUP, tiap "); Serial.print(lidJeda);
                Serial.println(" ms. Ketik 'L' lagi untuk berhenti.");
            } else {
                lidOn = !lidOn; lidT = 0;
                Serial.println(lidOn ? "Aliran LiDAR HIDUP. Ketik 'L' lagi untuk berhenti."
                                     : "Aliran LiDAR berhenti.");
            }
            break;
        }

        // --- NAVIGASI OTONOM (non-blokir) ---
        case 'f':   // ikut dinding KIRI
            nav.navMulai(NAV_DINDING_KIRI);
            break;

        case 'F':   // ikut dinding KANAN
            nav.navMulai(NAV_DINDING_KANAN);
            break;

        case 'p':   // ikut dinding KIRI + terkunci kompas arena
            nav.navMulai(NAV_ARENA_KIRI);
            break;

        case 'P':   // ikut dinding KANAN + terkunci kompas arena
            nav.navMulai(NAV_ARENA_KANAN);
            break;

        // 'U' = UNDAKAN. Lima perintah yang sudah ada, dirangkai dalam urutan
        // yang benar -- dan urutannya bukan selera:
        //
        //   'i1' HARUS paling akhir. navMulai() memeriksa sensor depan hidup
        //   sebelum melangkah, dan pemeriksaan itu SENGAJA dilewati kalau
        //   depan sudah diabaikan (Navigation.cpp). Mengetik 'i1' duluan
        //   berarti berjalan buta tanpa pernah tahu sensornya masih hidup.
        //
        //   Rem jarak WAJIB. Dengan depan buta, ketiga aturan depan mati;
        //   tidak ada lagi apa pun yang menghentikan robot di ujung tangga.
        //   Itu sebabnya perintah ini menolak jalan tanpa angka.
        //
        // Kemudinya MENENGAH: error = (kanan - kiri)/2, jadi ia mengunci
        // garis tengah lorong dan bukan satu dinding. Dinding KANAN cuma
        // cadangan kalau satu sisi hilang -- sama seperti ruas 24 di tabel.
        case 'U': {
            float p[1] = { 0.0f };
            if (argFloats(s, p, 1) < 1 || p[0] <= 0.0f) {
                Serial.println("U<cm> = NAIK TANGGA. Contoh: 'U103' untuk R-9.");
                Serial.println("  Sama dengan mengetik, berurutan:");
                Serial.println("    T5     profil TANJAK -- langkah 75 mm, satu-satunya yang terbukti naik anak tangga");
                Serial.println("    Z0     kemudi IKUT DINDING, bukan menengah");
                Serial.println("    F      ikut dinding KANAN -- kedua LiDAR kanan yang dipakai");
                Serial.println("    D<cm>  rem jarak");
                Serial.println("    i1     abaikan sensor depan -- PALING AKHIR, lihat komentar");
                Serial.println("  Plus koreksi BERHENTI-DULU: robot berhenti, memutar badan sejajar dinding,");
                Serial.println("  baru jalan lagi. Ukur bias sudutnya dengan 'Y0' di lorong lurus dulu.");
                Serial.println("  Jaraknya WAJIB: dengan depan buta, rem satu-satunya yang menghentikan.");
                Serial.println("  Berhenti apa pun ('s'/'x'/Enter/rem) memulihkan sensor depan.");
                break;
            }
            robot.profileTanjak();
            nav.setTengah(false);
            nav.navMulai(NAV_DINDING_KANAN);
            if (nav.navMode() == NAV_DIAM) {   // navMulai menolak, dan sudah
                Serial.println("NAIK TANGGA dibatalkan -- sensor depan TIDAK jadi diabaikan.");
                break;
            }
            nav.remJarakPasang(p[0]);
            nav.abaikanDepan(true);
            // PALING AKHIR bersama abaikanDepan: navMulai() yang menolak tidak
            // boleh meninggalkan koreksi berhenti-dulu menyala untuk perintah
            // navigasi berikutnya.
            nav.koreksiDiam(true);
            Serial.print("NAIK TANGGA: tanjak + dinding kanan, koreksi sambil berhenti, buta ke depan, rem ");
            Serial.print(p[0], 0); Serial.println(" cm.");
            break;
        }

        // --- MISI (lapisan di atas navigasi) ---
        // argFloats() membaca mulai s+1, jadi angka PERTAMA yang terbaca
        // adalah digit perintahnya sendiri; argumennya ada di p[1] dst.
        case 'm': {
            if (!hasNum) { misi.status(); break; }
            switch (s[1]) {
                case '0': gerakSampai = 0; misi.batal("dihentikan pengguna."); break;
                case '1': misi.mulai();      break;
                case '2': misi.jawab(true);  break;
                case '3': misi.jawab(false); break;
                case '4': {
                    // p[0] adalah digit perintahnya sendiri ('4'), jadi
                    // argumennya mulai di p[1]: 'm4 6 13' = ruas 6..13.
                    float p[3] = {0, 0, 0};
                    uint8_t n = argFloats(s, p, 3);
                    if (n >= 3)      misi.mulaiDari((uint8_t)p[1], (uint8_t)p[2]);
                    else if (n >= 2) misi.mulaiDari((uint8_t)p[1]);
                    else             misi.tabel();
                    break;
                }
                case '6': {
                    float p[2] = {0, 0};
                    if (argFloats(s, p, 2) >= 2) misi.ukur((uint8_t)p[1]);
                    else Serial.println("Format: m6 <idx>, misal m6 2");
                    break;
                }
                case '7': {
                    float p[3] = {0, 0, 0};
                    if (argFloats(s, p, 3) >= 3) misi.setRuasCm((uint8_t)p[1], p[2]);
                    else Serial.println("Format: m7 <idx> <cm>, misal m7 2 55");
                    break;
                }
                case '8': {
                    // m8 = balik, m8 1 = nyala, m8 0 = mati. p[0] digit
                    // perintahnya sendiri, jadi modenya di p[1].
                    float p[2] = {0, 0};
                    misi.setTungguVision(argFloats(s, p, 2) >= 2 ? (int)p[1] : -1);
                    break;
                }
                case '9': misi.lepasKendali(); break;
                default:
                    Serial.println("m=status  m4=tabel lintasan  m1=mulai  m0=batal");
                    Serial.println("m4 <idx>      = mulai dari satu ruas sampai akhir lintasan");
                    Serial.println("m4 <awal> <akhir> = jalankan SEBAGIAN, misal m4 0 13 (HOME..kaki tangga)");
                    Serial.println("m6 <idx>      = MODE UKUR: jalan tanpa henti, robot cetak cm-nya");
                    Serial.println("m7 <idx> <cm> = setel panjang ruas");
                    Serial.println("m2/m3         = saat menunggu konfirmasi: lanjut / ulangi ruas");
                    Serial.println("m8 [0|1]      = ruas AMBIL parkir menunggu Raspi menengahkan (bawaan NYALA)");
                    Serial.println("m9            = dari Raspi: 'aku sudah tidak memegang kaki', misi lanjut");
            }
            break;
        }

        case 'T': {   // profil medan: T = cetak, T0..T3 = pilih
            // Empat profil sudah lama ada di Hexapod (datar/tangga/merunduk/
            // sempit) lengkap dengan ramp GAIT_PROFILE_TAU yang menghaluskan
            // pergantiannya, tapi hanya profileFlat() yang pernah tersambung
            // (lewat 'b'). Tiga sisanya tidak bisa dipanggil dari mana pun.
            // Ini pemicunya: mesinnya sudah ada, tinggal saklarnya.
            //
            // Huruf 'T' bertabrakan dengan 't' (geser badan) -- tidak ada
            // pasangan besar-kecil yang benar-benar kosong lagi di firmware
            // ini. Tabrakan ini yang paling tidak berbahaya: 'T' tidak
            // menggerakkan robot, hanya mengganti profil, dan pergantiannya
            // di-ramp sehingga salah ketik pun tidak menyentak.
            if (!hasNum) {
                GaitProfile p = robot.gaitProfile();
                Serial.println("\n--- PROFIL MEDAN (yang BERLAKU, hasil ramp) ---");
                Serial.print("  tinggi langkah : "); Serial.print(p.stepHeight, 1);  Serial.println(" mm");
                Serial.print("  panjang langkah: "); Serial.print(p.stepLength, 1);  Serial.println(" mm");
                Serial.print("  waktu siklus   : "); Serial.print(p.cycleTime, 0);   Serial.println(" ms");
                Serial.print("  tinggi badan   : "); Serial.print(p.standHeight, 1); Serial.println(" mm");
                Serial.print("  radius kaki    : "); Serial.print(p.standRadius, 1); Serial.println(" mm");

                // SUDUT SENDI TIAP KAKI, dari legAngles() -- fungsi yang sama
                // yang memberi makan servo, bukan salinan rumusnya. Inilah
                // satu-satunya cara menjawab "apakah coxa T4 sudah sama dengan
                // T0?" tanpa busur derajat: catat kolom coxa di T0, ketik T4,
                // bandingkan. Profil yang bentuknya seragam memberi coxa yang
                // sama persis untuk keenam kaki di kedua profil.
                //
                // Dibaca saat robot BERDIRI DIAM. Kalau gait sedang berjalan,
                // angka-angka ini ikut berayun -- yang terbaca ayunannya, bukan
                // bentuk berdirinya.
                Serial.println("  sudut sendi (berdiri diam; der):");
                Serial.println("    kaki   coxa   femur   tibia");
                static const char* NAMA_KAKI[6] = {
                    "0 kn-dp", "1 kn-tg", "2 kn-bl", "3 ki-bl", "4 ki-tg", "5 ki-dp"
                };
                for (int leg = 0; leg < 6; leg++) {
                    float cx, fm, tb;
                    if (!robot.legAngles(leg, cx, fm, tb)) {
                        Serial.print("    "); Serial.print(NAMA_KAKI[leg]);
                        Serial.println("   (di luar jangkauan IK)");
                        continue;
                    }
                    Serial.printf("    %-7s %+6.1f  %+6.1f  %+6.1f\n",
                                  NAMA_KAKI[leg], (double)cx, (double)fm, (double)tb);
                }
                Serial.println("  T0=datar  T1=tangga  T2=merunduk/turunan  T3=sempit  T4=kail  T5=tanjak");
                break;
            }
            switch ((int)v) {
                case 0: robot.profileFlat();   Serial.println("Profil -> 0 DATAR");              break;
                case 1: robot.profileStairs(); Serial.println("Profil -> 1 TANGGA");             break;
                case 2: robot.profileCrouch(); Serial.println("Profil -> 2 MERUNDUK / TURUNAN"); break;
                case 3: robot.profileNarrow(); Serial.println("Profil -> 3 SEMPIT");             break;
                case 4:
                    robot.profileKail();
                    Serial.println("Profil -> 4 KAIL (R-9: depan mengait, belakang naik)");
                    Serial.println("  Bentuknya TIDAK seragam. Lihat robot berdiri dulu sebelum");
                    Serial.println("  mempercayakannya ke misi; angkanya di config.h (KAIL_*).");
                    break;
                case 5:
                    robot.profileTanjak();
                    Serial.println("Profil -> 5 TANJAK (R-9 lambat: langkah 75, siklus 1300)");
                    Serial.println("  Kaki depan MAJU 60 mm -- knop yang selama ini mati di KAIL.");
                    Serial.println("  Ketik 'T' sekarang: kalau ada kaki '(di luar jangkauan IK)',");
                    Serial.println("  turunkan KAIL_DEPAN_MAJU sebelum dipakai di tanjakan.");
                    break;
                default:
                    Serial.println("T0=datar  T1=tangga  T2=merunduk/turunan  T3=sempit  T4=kail  T5=tanjak");
                    break;
            }
            Serial.println("  Berlaku SAMBIL BERJALAN dan di-ramp; tidak perlu 'b'.");
            Serial.println("  'T' tanpa angka mencetak profil yang sedang berlaku.");
            break;
        }

        case 'Z':   // Z = keadaan, Z1 = menengah lorong, Z0 = ikut dinding
            if      (s[1] == '1') nav.setTengah(true);
            else if (s[1] == '0') nav.setTengah(false);
            else {
                Serial.print("Kemudi lateral: ");
                Serial.println(nav.tengah() ? "MENENGAH (selisih kiri-kanan)"
                                            : "IKUT DINDING (satu sisi)");
                Serial.println("  Z1 = menengah, Z0 = ikut dinding. Boleh ditukar SAMBIL BERJALAN.");
            }
            break;

        // 'Y' = keluarga KALIBRASI. Bentuk telanjang dan 'Y0' milik sudut
        // dinding; 'Yt...' milik trim servo, ditambahkan 18 Sep 2026.
        //
        // Berawalan karena KEHABISAN HURUF: seluruh 52 huruf sudah terpakai.
        // Ditumpangkan ke 'Y', bukan 's' atau 'x', dengan sengaja -- salah
        // ketik di dua huruf itu meninggalkan robot berjalan.
        case 'Y':
            if (s[1] == 't') {
                const char* arg = s + 2;
                while (*arg == ' ') arg++;
                if (*arg == '\0')      { robot.cetakTrim();  break; }
                if (*arg == 'W')       { robot.simpanServoMap(); break; }
                if (*arg == '!')       { robot.nolkanTrim(); break; }
                char* akhir;
                long slot = strtol(arg, &akhir, 10);
                if (akhir == arg) {
                    Serial.println("Format: 'Yt' tabel | 'Yt<slot> <us>' setel | 'YtW' simpan | 'Yt!' nolkan");
                    break;
                }
                while (*akhir == ' ' || *akhir == '=') akhir++;
                char* akhir2;
                long us = strtol(akhir, &akhir2, 10);
                if (akhir2 == akhir) {
                    Serial.println("Nilai trim tidak terbaca. Contoh: 'Yt5 -40'");
                    break;
                }
                if (slot < 0 || slot > 255) {
                    Serial.println("Slot di luar jangkauan.");
                    break;
                }
                robot.setTrim((uint8_t)slot, (int16_t)us);
                break;
            }
            if (s[1] == '0') nav.kalibrasiSudut();
            else             nav.sudutTabel();
            break;

        case 'N':   // N = keadaan, N0..N3 = matriks hukum x sumber turunan
            if (s[1] >= '0' && s[1] <= '3') nav.setKemudiMode((uint8_t)(s[1] - '0'));
            else {
                Serial.print("Kemudi dinding: ");
                Serial.print(nav.kemudiSamar() ? "SAMAR (fuzzy)" : "PD");
                Serial.print(", turunan dari ");
                Serial.println(nav.kemudiSudut() ? "SUDUT sepasang sensor" : "selisih waktu");
                Serial.println("  N0 = PD/waktu    N1 = SAMAR/waktu");
                Serial.println("  N2 = PD/SUDUT    N3 = SAMAR/SUDUT");
                Serial.println("  Boleh ditukar SAMBIL BERJALAN.");
            }
            break;

        case 'i':   // i = keadaan, i1 = abaikan sensor depan, i0 = pakai lagi
            if      (s[1] == '1') nav.abaikanDepan(true);
            else if (s[1] == '0') nav.abaikanDepan(false);
            else {
                Serial.print("Sensor depan: ");
                Serial.println(nav.depanDiabaikan() ? "DIABAIKAN (buta ke depan)"
                                                    : "dipakai");
                Serial.println("  i1 = abaikan (hanya untuk turunan), i0 = pakai lagi.");
            }
            break;

        case 'v':   // status navigasi + jarak sekitar
            nav.navStatus();
            break;

        case 'D': {   // odometri: D=cetak, D<cm>=pasang rem, D0=lepas+nolkan
            if (s[1] == 's') {
                // argFloats() membaca mulai s+1, jadi s+1 di sini menaruh
                // titik baca tepat sesudah huruf 's'.
                float p[1] = {0};
                if (argFloats(s + 1, p, 1) >= 1) {
                    robot.setSkalaOdo(p[0]);
                    Serial.print("Skala odometri -> "); Serial.print(robot.skalaOdo(), 3);
                    if (fabsf(robot.skalaOdo() - p[0]) > 1e-3f)
                        Serial.print("  (diminta belum sah, DI-CLAMP ke 0,5 .. 1,5)");
                    Serial.println();
                    Serial.println("  Hanya di RAM. Kalau sudah pasti, tulis ke config.h.");
                } else {
                    Serial.println("Format: Ds<faktor>, misal Ds1.05");
                }
                break;
            }
            float p[1] = {0};
            if (argFloats(s, p, 1) < 1) {                 // 'D' polos
                Serial.print("Jarak maju   : "); Serial.print(robot.jarakCm(), 1);
                Serial.println(" cm sejak terakhir dinolkan");
                Serial.print("  geser     : "); Serial.print(robot.geserCm(), 1);
                Serial.println(" cm  (kanan +, kiri -)");
                Serial.print("  lintasan  : "); Serial.print(robot.lintasCm(), 1);
                Serial.println(" cm  <- INI yang dipakai rem jarak");
                Serial.print("  rem       : ");
                if (nav.remJarakAda()) { Serial.print(nav.remJarakSasaran(), 1);
                                         Serial.println(" cm"); }
                else Serial.println("tidak terpasang");
                Serial.print("  skala     : "); Serial.println(robot.skalaOdo(), 3);
                break;
            }
            if (p[0] <= 0.0f) {                            // 'D0'
                nav.remJarakLepas();
                robot.jarakNol();
                Serial.println("Jarak dinolkan, rem dilepas.");
                break;
            }
            nav.remJarakPasang(p[0]);
            break;
        }

        case 'j': {  // Jejak statistik LiDAR: j (semua) atau j<channel>
            if (lidar.jejakJalan()) { Serial.println("Jejak sedang berjalan, tunggu selesai."); break; }
            float p[2] = {-1, 5};
            uint8_t n = argFloats(s, p, 2);
            lidar.jejakMulai((n >= 1) ? (int8_t)p[0] : -1,
                             (uint16_t)clampf((n >= 2) ? p[1] : 5.0f, 1.0f, 30.0f));
            break;
        }

        // 'u' = uji isolasi. SENGAJA bukan 'J': huruf besar-kecil dipakai di
        // firmware ini untuk pasangan SIMETRIS (f/F kiri-kanan, g/G grip),
        // bukan untuk dua fungsi berbeda. 'j' dan 'J' terlalu mudah tertukar.
        case 'u': {  // u = sapu semua sensor; u<channel> <detik> = satu sensor
            if (lidar.isolasiJalan()) { Serial.println("Uji isolasi sedang berjalan."); break; }
            float p[2] = {-1, 4};
            uint8_t n = argFloats(s, p, 2);
            lidar.isolasiMulai((n >= 1) ? (int8_t)p[0] : -1,
                               (uint16_t)clampf((n >= 2) ? p[1] : 4.0f, 2.0f, 20.0f));
            break;
        }

        case 'I':   // Pindai bus I2C LiDAR + init ulang sensor yang belum aktif
            // I1 = uji PIN, bukan uji bus. Dipakai saat pindaian bilang garis
            // tertahan rendah dan yang tersisa tinggal "pinnya sendiri rusak
            // atau tidak".
            if (s[1] == '1') lidar.periksaPinBus();
            else             lidar.pindaiI2C();
            break;

        case 'M':   // Cetak peta EEPROM + cek kapasitas chip sebenarnya
            eeMapPeriksa(true);
            break;

        // --- PARAMETER KALIBRASI ---
        case 'q': {   // q = daftar semua, q<nama> = satu parameter
            char nm[24];
            if (!ambilToken(s, nm, sizeof(nm))) { cetakSemuaParam(); break; }
            int i = cariParam(nm);
            if (i == -1) { Serial.print("Parameter tidak dikenal: "); Serial.println(nm); }
            else if (i >= 0) cetakSatuParam(i);
            break;
        }

        case 'Q': {   // Q<nama> <nilai>
            char nm[24];
            const char* sisa = ambilToken(s, nm, sizeof(nm));
            if (!sisa) { Serial.println("Format: Q<nama> <nilai>, misal Qwall.kp 0.012"); break; }

            while (*sisa == ' ' || *sisa == '=') sisa++;
            char* akhir;
            float minta = strtof(sisa, &akhir);
            if (akhir == sisa) { Serial.println("Nilai tidak terbaca. Misal: Qwall.kp 0.012"); break; }

            int i = cariParam(nm);
            if (i == -1) { Serial.print("Parameter tidak dikenal: "); Serial.println(nm); break; }
            if (i < 0) break;                    // ambigu, kandidat sudah dicetak

            const ParamDef& d = PARAM_DEFS[i];

            // Mengubah pemetaan sudut->pulse menggeser KEENAM BELAS servo
            // sekaligus tanpa ramp -- kelas bahaya yang sama dengan lonjakan
            // pose badan. Haruskan servo lemas dulu, jangan cuma diperingatkan.
            if (d.berlaku == P_SERVO_LEMAS && robot.isArmed()) {
                Serial.print("Ditolak: '"); Serial.print(d.name);
                Serial.println("' menggeser semua servo sekaligus tanpa ramp.");
                Serial.println("        Ketik 'x' (lemas) dulu, ubah, lalu 'b' lagi.");
                break;
            }

            float sebelum = gParam[i];
            if (!Calib::setParam(d.name, minta)) {   // clamp ke [lo,hi] di dalam
                Serial.println("Gagal menyetel parameter.");
                break;
            }
            float sesudah = gParam[i];

            Serial.print(d.name); Serial.print(" : ");
            Serial.print(sebelum, 3); Serial.print(" -> "); Serial.println(sesudah, 3);

            // Clamp diam-diam persis kelas bug yang sudah berkali-kali menggigit
            // di firmware ini -- jadi katakan kalau permintaannya dipangkas.
            if (fabsf(sesudah - minta) > 1e-6f) {
                Serial.print("  (diminta "); Serial.print(minta, 3);
                Serial.print(", DI-CLAMP ke rentang sah "); Serial.print(d.lo, 3);
                Serial.print(" .. "); Serial.print(d.hi, 3); Serial.println(")");
            }

            switch (d.berlaku) {
                case P_PERLU_B:
                    Serial.println("  Baru berlaku sesudah profil gait di-set ulang -- ketik 'b'.");
                    break;
                case P_BELUM_DIPAKAI:
                    Serial.println("  CATATAN: belum ada kode yang membaca parameter ini.");
                    break;
                default: break;
            }
            Serial.println("  Masih di RAM. Ketik 'W' supaya bertahan sesudah reset.");
            break;
        }

        case 'W':   // Simpan seluruh blok Calib ke EEPROM 0
            Calib::save();
            Serial.println("Parameter disimpan ke EEPROM 0. Bertahan sesudah reset,");
            Serial.println("  KECUALI bila CALIB_VERSION dinaikkan -- blob lama lalu dibuang.");
            break;

        case 'e':
            nav.kompasSimpan();
            break;

        case 'E':
            if (nav.kompasMuat(true)) nav.kompasTabel();
            break;

        // --- 2. NAVIGASI: PIVOT OTOMATIS (PD, NON-BLOKIR) ---
        // Keduanya hanya MEMULAI pivot lalu kembali seketika; kerjanya di
        // nav.navUpdate() yang dipanggil tiap loop. Jadi perintah lain tetap
        // terproses selama robot berputar, dan 's'/'x'/Enter membatalkannya.
        case 'o':
            if (d1 > 3) { Serial.println("o0=UTARA o1=TIMUR o2=SELATAN o3=BARAT"); break; }
            nav.pivotKompas(d1);
            break;

        case 'O':
            if (!hasNum) { Serial.println("Format: O<derajat>, misal O90 atau O-90"); break; }
            nav.pivotRelatif((float)v);
            break;

        // --- 3. NAVIGASI: KALIBRASI ---
        case 'C':
            nav.kalibrasiPivot((uint8_t)(hasNum ? v : 4));
            break;

        // --- 4. BODY KINEMATICS (uji pose badan) ---
        // Kaki tetap menapak di tempat; hanya badan yang bergeser/miring.
        case 'r': {  // r<roll> <pitch> <yaw>  (derajat)
            float a[3] = {0, 0, 0};
            uint8_t n = argFloats(s, a, 3);
            if (n == 0) { cetakPoseBadan(); break; }
            if (demoOn) demoStop("diambil alih perintah manual.");
            if (goyangOn) goyangStop("diambil alih perintah manual.");
            robot.setBodyRotation(a[0], a[1], a[2]);
    // Batas waktu gerak manual. Ditaruh SEBELUM robot.update() supaya vektor
    // nol ikut terpakai di tick yang sama, bukan satu tick sesudahnya.
    if (gerakSampai) {
        int sisa = jarakArahJalan();
        if (sisa >= 0 && sisa <= GERAK_AMAN_CM) {
            gerakSampai = 0; gerakMaju = gerakGeser = 0.0f;
            robot.stop();
            Serial.print("Gerak manual BERHENTI: ada sesuatu ");
            Serial.print(sisa); Serial.println(" cm di arah jalannya.");
        } else if ((int32_t)(millis() - gerakSampai) >= 0) {
            gerakSampai = 0; gerakMaju = gerakGeser = 0.0f;
            robot.stop();
            Serial.println("Gerak manual selesai (batas waktu).");
        }
    }

            robot.update();               // hitung ulang supaya laporan di bawah akurat
            cetakPoseBadan();
            break;
        }

        case 't': {  // t<x> <y> <z>  (mm)
            float a[3] = {0, 0, 0};
            uint8_t n = argFloats(s, a, 3);
            if (n == 0) { cetakPoseBadan(); break; }
            if (demoOn) demoStop("diambil alih perintah manual.");
            if (goyangOn) goyangStop("diambil alih perintah manual.");
            robot.setBodyTranslation(a[0], a[1], a[2]);
            robot.update();
            cetakPoseBadan();
            break;
        }

        case '0':    // Nolkan pose badan (kembali tegak & terpusat)
            if (demoOn) demoStop("dinolkan.");
            if (goyangOn) goyangStop("dinolkan.");
            robot.setBodyRotation(0, 0, 0);
            robot.setBodyTranslation(0, 0, 0);
            Serial.println("Pose badan dinolkan.");
            break;

        case 'z': {  // Goyang roll bergelombang. z<amplitudo> <periode> <fasePitch>
            if (goyangOn) { goyangStop("dihentikan pengguna."); break; }

            float p[3] = {goyangAmp, goyangPer, goyangFase};
            uint8_t n = argFloats(s, p, 3);
            if (n >= 1) goyangAmp  = clampf(p[0], 1.0f, BODY_MAX_ROT_DEG);
            if (n >= 2) goyangPer  = clampf(p[1], 0.2f, 30.0f);
            if (n >= 3) goyangFase = clampf(p[2], -180.0f, 180.0f);

            // Pose badan di-ramp BODY_SLEW_DEG_S. Kalau laju puncak sinus
            // melebihi itu, ramp memotong puncaknya dan yang terlihat bukan
            // gelombang lagi melainkan segitiga. Naikkan periodenya, dan
            // katakan -- jangan diam-diam menghasilkan bentuk yang salah.
            float perMin = goyangAmp * 2.0f * (float)M_PI / BODY_SLEW_DEG_S;
            if (goyangPer < perMin) {
                Serial.print("Periode dinaikkan "); Serial.print(goyangPer, 2);
                Serial.print(" -> ");               Serial.print(perMin, 2);
                Serial.println(" detik supaya sinusnya tidak terpotong ramp.");
                Serial.println("  (turunkan amplitudo kalau ingin ayunan lebih cepat)");
                goyangPer = perMin;
            }

            if (demoOn) demoStop("diambil alih goyang roll.");
            if (!robot.isArmed())
                Serial.println("(servo masih lemas -- ketik 'b' dulu bila ingin melihatnya)");

            goyangOn = true; goyangT0 = millis();

            float lajuPuncak = goyangAmp * 2.0f * (float)M_PI / goyangPer;
            Serial.print("Goyang roll HIDUP: amplitudo "); Serial.print(goyangAmp, 1);
            Serial.print(" der, periode ");               Serial.print(goyangPer, 2);
            Serial.println(" detik.");
            if (fabsf(goyangFase) > 0.01f) {
                Serial.print("  pitch ikut, beda fase "); Serial.print(goyangFase, 0);
                Serial.println(" der -- badan menelusuri kerucut.");
            }
            Serial.print("  laju puncak "); Serial.print(lajuPuncak, 1);
            Serial.print(" der/detik (batas ramp "); Serial.print(BODY_SLEW_DEG_S, 0);
            Serial.println("). 'z' lagi untuk berhenti.");
            break;
        }

        case 'B':    // Demo sapuan 6 sumbu (18 detik), tekan lagi untuk berhenti
            if (goyangOn) goyangStop("diambil alih demo 'B'.");
            if (demoOn) { demoStop("dihentikan pengguna."); break; }
            if (!robot.isArmed()) Serial.println("(servo masih lemas -- ketik 'b' dulu bila ingin melihat gerakannya)");
            demoOn = true; demoT0 = millis(); demoAxis = -1;
            Serial.println("Demo body kinematics: 6 sumbu x 3 detik. 'B' lagi untuk berhenti.");
            break;

        // --- 5. LENGAN (BAHU, SIKU, GRIP) ---
        // 'a' = lengan DEPAN, 'A' = lengan BELAKANG. Keduanya menyapu bidang
        // vertikal; jangkauan diukur dari pusat badan ke arah hadap lengan.
        case 'a': case 'A': {
            uint8_t arm = (c == 'a') ? ARM_DEPAN : ARM_BELAKANG;
            const char* nama = (c == 'a') ? "DEPAN" : "BELAKANG";

            // Lengan BELAKANG cuma grip -- tidak ada sendi yang bisa dijangkau.
            // Ditolak DI SINI dengan sebab yang jelas, bukan dibiarkan jatuh ke
            // pesan "di luar jangkauan" yang menyesatkan.
            if (arm == ARM_BELAKANG) {
                Serial.println("Lengan BELAKANG tidak punya sendi -- hanya grip. Pakai 'G<0-100>'.");
                Serial.println("  Letak capit belakang ditentukan letak BADAN, bukan sudut sendi.");
                break;
            }
            // 'aa' / 'at' = SEKUENS korban, bukan satu pose. Keduanya memanggil
            // sekuens yang sama persis yang dijalankan misi di ruas AMBIL dan
            // TARUH, dengan jeda antar pose yang sama, jadi yang terlihat di
            // meja adalah yang akan terjadi di arena.
            if (s[1] == 'a' || s[1] == 't') {
                misi.ujiLengan(s[1] == 'a');
                break;
            }
            // 'as' = SUDUT LANGSUNG, tanpa IK. argFloats() memakai strtof dan
            // berhenti seketika pada huruf, jadi "as90 ..." mengembalikan 0
            // argumen untuk 'a' biasa -- kedua bentuk tidak bisa tertukar.
            if (s[1] == 's') {
                float q[3] = {0, 0, 0};
                if (argFloats(s + 1, q, 3) < 3) {
                    Serial.println("Format: as<bahu> <siku> <pergelangan>  (der geometris)");
                    Serial.println("  Menembak KETIGA sendi langsung -- tidak lewat IK.");
                    Serial.println("  Netral 'as0 0 0' = baseline config.h (bahu 90, siku 0, prg 90 servo).");
                    Serial.println("  Dipakai untuk membidik pose dengan tangan lalu menuliskannya keras;");
                    Serial.println("  jalur kalibrasinya sama dengan IK, jadi posenya berulang sama.");
                    Serial.println("  'a<jangkauan> <tinggi>' kalau yang diketahui letak capitnya, bukan sudutnya.");
                    break;
                }
                if (!robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); break; }
                if (!robot.armEnabled(arm)) {
                    robot.armEnable(arm, true);
                    Serial.println("Servo lengan DEPAN DIHIDUPKAN.");
                }
                float sv[3];
                const bool sanggup = robot.setSudutLengan(arm, q[0], q[1], q[2], sv);
                static const char* NAMA[3] = { "bahu       ", "siku       ", "pergelangan" };
                for (uint8_t i = 0; i < 3; i++) {
                    Serial.print("  "); Serial.print(NAMA[i]);
                    Serial.print(" geo "); Serial.print(q[i], 1);
                    Serial.print(" der -> servo "); Serial.print(sv[i], 1);
                    Serial.println((sv[i] < 0.0f || sv[i] > 180.0f)
                                   ? " der  << DI LUAR 0..180, DI-CLAMP" : " der");
                }
                if (!sanggup)
                    Serial.println("  Pose TIDAK utuh: yang mentok berhenti di batasnya, sisanya sampai.");
                Serial.println("  Servo merayap 120 der/detik -- beri jeda sebelum mengukur.");
                break;
            }

            float p[3] = {0, 0, 0};
            int nArg = argFloats(s, p, 3);
            if (nArg < 2) {
                Serial.print("Format: "); Serial.print(c);
                Serial.println("<jangkauan> <tinggi> [pergelangan]  (mm, mm, der). Misal: a70 20 -15");
                break;
            }
            if (!robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); break; }

            if (!robot.armEnabled(arm)) {
                robot.armEnable(arm, true);
                Serial.print("Servo lengan "); Serial.print(nama); Serial.println(" DIHIDUPKAN.");
            }
            if (robot.moveArmTarget(arm, p[0], p[1])) {
                Serial.print("Lengan "); Serial.print(nama);
                Serial.print(" -> jangkauan "); Serial.print(p[0], 1);
                Serial.print(" mm, tinggi ");   Serial.print(p[1], 1); Serial.print(" mm");
                // Pergelangan hanya disentuh kalau diminta: tanpa argumen ketiga
                // ia harus TETAP di sudut terakhir, bukan tersentak ke nol.
                if (nArg >= 3) {
                    robot.setPergelangan(arm, p[2]);
                    Serial.print(", pergelangan "); Serial.print(clampf(p[2], -90.0f, 90.0f), 1);
                    Serial.print(" der");
                }
                Serial.println();
            } else {
                Serial.println("!! Di luar jangkauan lengan -- sudut TIDAK dikirim.");
                Serial.print("   Jangkauan sah dari pangkal bahu: ");
                Serial.print(fabsf(UPPERARM_LENGTH - FOREARM_LENGTH), 0); Serial.print(" .. ");
                Serial.print(UPPERARM_LENGTH + FOREARM_LENGTH, 0);        Serial.println(" mm");
            }
            break;
        }

        case 'g': case 'G': {   // g<0-100> penjepit: 0 = menutup, 100 = membuka
            uint8_t arm = (c == 'g') ? ARM_DEPAN : ARM_BELAKANG;
            const char* nama = (c == 'g') ? "DEPAN" : "BELAKANG";

            float p[1] = {0};
            if (argFloats(s, p, 1) < 1) {
                Serial.print("Format: "); Serial.print(c);
                Serial.println("<0-100>   0 = menutup penuh, 100 = membuka penuh");
                break;
            }
            if (!robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); break; }

            if (!robot.armEnabled(arm)) {
                robot.armEnable(arm, true);
                Serial.print("Servo lengan "); Serial.print(nama); Serial.println(" DIHIDUPKAN.");
            }
            robot.setGrip(arm, p[0]);
            Serial.print("Grip "); Serial.print(nama);
            // Persen yang DITERIMA setGrip(), bukan yang diketik: MG90S dipagari
            // GRIP_PERSEN_MIN/MAKS, jadi 'g100' benar-benar berhenti di 95%.
            Serial.print(" -> ");
            Serial.print(clampf(p[0], GRIP_PERSEN_MIN, GRIP_PERSEN_MAKS), 0);
            Serial.println("%");
            break;
        }

        // REHAT: lengan terlipat di ATAS badan, capit menunduk. Ini pose
        // ISTIRAHAT yang diperintah, BUKAN pose pemasangan horn: memasang
        // horn pada pose ini memaksa ARM_BASE_BAHU 0 dan seluruh setengah
        // bawah jangkauan hilang. Lihat config.h.
        //
        // Sasarannya DISETEL DI ROBOT (REHAT_* di config.h), tidak dihitung:
        // yang harus dihindari lengan itu LiDAR dan kabel yang nyata.
        case 'R': {
            if (!robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); break; }
            if (!robot.armEnabled(ARM_DEPAN)) {
                robot.armEnable(ARM_DEPAN, true);
                Serial.println("Servo lengan DEPAN DIHIDUPKAN.");
            }
            // Angkanya DIBIDIK di robot (config.h), bukan diturunkan dari panjang
            // link: pose rehat harus muat di atas badan yang nyata, dengan LiDAR
            // dan kabelnya. Sama persis dengan mengetik 'as40 100 -20'.
            float svR[3];
            if (!robot.setSudutLengan(ARM_DEPAN, REHAT_BAHU, REHAT_SIKU,
                                      REHAT_PERGELANGAN, svR)) {
                Serial.printf("!! Pose rehat MENTOK: servo %.1f %.1f %.1f der\n",
                              (double)svR[0], (double)svR[1], (double)svR[2]);
                Serial.println("   Yang di luar 0..180 di-clamp. Setel REHAT_* di config.h.");
                break;
            }
            Serial.printf("Lengan REHAT -> as%.0f %.0f %.0f  (terlipat di atas badan)\n",
                          (double)REHAT_BAHU, (double)REHAT_SIKU, (double)REHAT_PERGELANGAN);
            Serial.println("  'as0 0 0' untuk kembali ke baseline.");
            break;
        }

        case 'n':   // Matikan kedua servo lengan (kaki tetap hidup)
            robot.armEnable(ARM_DEPAN, false);
            robot.armEnable(ARM_BELAKANG, false);
            Serial.println("Servo kedua lengan DIMATIKAN (kaki tidak terpengaruh).");
            break;

        // --- 6. KESELAMATAN SERVO ---
        case 'x': // Lemas darurat: PWM mati, servo bebas
            gerakSampai = 0;
            misi.batal("servo dilemaskan.");   // WAJIB sebelum nav: kalau tidak,
            nav.navBerhenti("servo dilemaskan.");  // misi menyalakan navigasi lagi
            nav.remJarakLepas();
            if (demoOn) demoStop("servo dilemaskan.");
            if (goyangOn) goyangStop("servo dilemaskan.");
            robot.disarm();
            break;

        case 'd': // Dump diagnostik (kalibrasi + IK per kaki)
            robot.debugDump();
#if DEMO_BOOT
            // Robot yang berdiri sendiri saat dinyalakan itu menyimpang dari
            // rancangan boot-lemas, jadi jangan biarkan ia jadi kejutan bagi
            // orang berikutnya yang membuka diagnostik.
            Serial.println("  CATATAN: DEMO_BOOT aktif -- robot berdiri sendiri saat menyala.");
            Serial.println("           Matikan lewat DEMO_BOOT 0 di config.h.");
#endif
            break;

        // --- 7. KONTROL ROBOT DASAR ---
        case 'b': {   // Berdiri diam (pose netral). Opsional: b<mm> untuk atur tinggi badan.
            if (demoOn) demoStop("kembali ke pose berdiri.");
            if (goyangOn) goyangStop("kembali ke pose berdiri.");
            robot.stop();                        // vektor gerak = 0 -> gait settle ke posisi home
            robot.setBodyTranslation(0, 0, 0);   // buang geser badan
            robot.setBodyRotation(0, 0, 0);      // buang roll/pitch/yaw badan

            if (hasNum) {
                float h = clampf((float)v, 40.0f, 160.0f);
                // Salin profil datar, hanya tinggi badan yang diganti.
                // Perubahan profil di-ramp oleh HexaGait, jadi badan naik/turun mulus.
                robot.setGaitProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH,
                                       GAIT_CYCLE_TIME, h, STAND_RADIUS });
                Serial.print("Berdiri diam. Tinggi badan = ");
                Serial.print(h, 0); Serial.println(" mm");
            } else {
                robot.profileFlat();
                Serial.println("Berdiri diam (pose netral, profil DATAR).");
            }

            // Hidupkan servo bila masih lemas. arm() menghitung pose berdiri
            // DULU, baru menyalakan PWM -- servo langsung menuju sasaran yang
            // benar, dan pengaktifan pertama dilakukan satu per satu (60 ms)
            // supaya arus 18 servo tidak menumpuk.
            if (!robot.isArmed()) robot.arm();
            break;
        }

        case 's': // Stop
            misi.batal("dihentikan pengguna.");
            nav.navBerhenti("dihentikan pengguna.");   // WAJIB: kalau tidak,
            robot.stop();                              // navUpdate() menyalakannya lagi
            nav.remJarakLepas();                       // jangan menyala di perjalanan berikutnya

            // SEMUA YANG BERULANG IKUT BERHENTI. 's' adalah satu-satunya
            // tombol panik yang diketik orang saat ada yang salah, dan
            // sebelum ini ia cuma menghentikan KAKI. Aliran yaw dan LiDAR
            // tetap mencetak, demo dan goyang tetap menggerakkan badan, dan
            // urutan boot tetap berjalan menuju servo hidup -- jadi layar
            // terus bergulir justru saat operator paling perlu membacanya,
            // dan robot masih bergerak sesudah diperintahkan diam.
            //
            // Yang punya fungsi henti dipanggil lewat fungsinya, supaya
            // alasannya ikut tercetak dan pose badan dinolkan sebagaimana
            // mestinya. Dua aliran cetak cuma perlu benderanya dimatikan.
#if DEMO_BOOT
            bootBatal("dihentikan pengguna.");
#endif
            if (demoOn)   demoStop("dihentikan pengguna.");
            if (goyangOn) goyangStop("dihentikan pengguna.");
            if (yawOn) { yawOn = false; Serial.println("Aliran yaw berhenti."); }
            if (lidOn) { lidOn = false; Serial.println("Aliran LiDAR berhenti."); }

            Serial.println("Robot berhenti.");
            break;

        case 'w': {  // Jalan manual. 'w' saja = maju; 'w <maju> <geser> [detik]'
                     // membuka sumbu GESER SAMPING -- geser kanan(+)/kiri(-).
                     // Sumbu itu ada di Hexapod::walk() dan HexaGait sejak awal,
                     // tapi KEENAM pemanggil walk() di firmware ini selalu
                     // mengisinya 0.0f, jadi ia belum pernah bergerak. Uji di
                     // lantai terbuka dulu: 'w0 0.4 2' geser kanan 2 detik.
            // Servo lemas = perintah gerak diterima, dicetak, lalu tidak terjadi
            // apa-apa. Itu bukan cuma sia-sia: ia terbaca seperti sumbu yang
            // rusak, dan sudah sempat menyesatkan sekali (6 Sep 2026 -- tiga
            // uji geser dinyatakan "tidak bergerak" padahal PWM memang padam
            // sejak flash sebelumnya). Hexapod::isArmed() sudah ada sejak awal
            // tapi tidak dipakai perintah gerak mana pun.
            if (!robot.isArmed()) {
                Serial.println("DITOLAK: servo masih LEMAS. Topang robot lalu ketik 'b'.");
                break;
            }
            float p[3] = {0, 0, 0};
            uint8_t n = argFloats(s, p, 3);
            nav.navBerhenti("diambil alih perintah manual.");
            if (n == 0) {
                gerakSampai = 0;
                robot.walk(NAV_FWD_SPEED, 0.0f, 0.0f);
                Serial.println("Robot maju.");
                break;
            }
            float maju  = clampf(p[0], -1.0f, 1.0f);
            float geser = clampf(p[1], -1.0f, 1.0f);
            float detik = (n >= 3) ? clampf(p[2], 0.5f, 30.0f) : 2.0f;
            // Dinilai SEBELUM berangkat: kalau sudah mepet, satu tick pun
            // sudah terlambat pada 13 cm/detik.
            gerakMaju = maju; gerakGeser = geser;
            int sisa = jarakArahJalan();
            if (sisa >= 0 && sisa <= GERAK_AMAN_CM) {
                gerakMaju = gerakGeser = 0.0f;
                Serial.print("DITOLAK: sudah ada sesuatu "); Serial.print(sisa);
                Serial.print(" cm di arah itu (batas aman ");
                Serial.print(GERAK_AMAN_CM); Serial.println(" cm).");
                break;
            }
            // PENJAGA BUTA. jarakArahJalan() mengembalikan -1 saat tak satu pun
            // sensor di arah jalan memberi angka, dan pemanggilnya -- di sini
            // maupun di loop utama -- membaca -1 sebagai "tidak ada halangan".
            // Penjaganya GAGAL TERBUKA, persis pada keadaan yang paling butuh
            // dijaga. 6 September 2026 itulah yang menabrakkan robot berkali-
            // kali: sensor sisi kehilangan sinyal justru saat robot mendekati
            // dinding, gerak jalan terus, dan tidak ada satu baris pun yang
            // memberi tahu operator bahwa ia sedang berjalan tanpa penjaga.
            // Perilakunya sengaja TIDAK diubah -- menolak bergerak setiap kali
            // sensor sisi buta akan membuat robot hampir tak bisa digerakkan,
            // karena di arena keempat sensor sisi memang sering signal fail.
            // Yang ditambahkan hanya kejujurannya.
            if (sisa < 0) {
                Serial.println("! PENJAGA BUTA: tak ada sensor sah di arah itu.");
                Serial.println("  Hanya BATAS WAKTU yang akan menghentikan gerakan ini.");
            }
            gerakSampai = millis() + (uint32_t)(detik * 1000.0f);
            robot.walk(maju, geser, 0.0f);
            Serial.print("Gerak manual: maju "); Serial.print(maju, 2);
            Serial.print("  geser "); Serial.print(geser, 2);
            Serial.print(geser > 0 ? " (KANAN)" : geser < 0 ? " (KIRI)" : "");
            Serial.print("  selama "); Serial.print(detik, 1); Serial.println(" detik.");
            if (geser != 0.0f) {
                // Sudah diuji 6 Sep 2026, dan hasilnya perlu diketahui SEBELUM
                // menekan Enter: perpindahannya terkuantisasi satu langkah
                // penuh, jadi memperpendek durasi hampir tidak mengecilkannya.
                // 'w 0 -0.6 3' -> ~32 cm; 'w 0 -0.6 0.5' -> ~36 cm. Bahkan dua
                // perintah IDENTIK berbeda tiga kali lipat (+3 lalu +9 cm),
                // karena semburan lebih pendek dari satu siklus gait menangkap
                // fase sapuan kaki yang berbeda-beda.
                Serial.println("  ! Sumbu geser TERKUANTISASI satu langkah: durasi hampir tak berpengaruh,");
                Serial.println("    dan dua perintah yang sama bisa berbeda 3x. Jangan pakai untuk offset kecil.");
            }
            break;
        }

        case 'X': {  // X = cetak poin, X0/X1/X2 = lapor kebersihan R-7
            // Kebersihan R-7 TIDAK BISA diukur sensor mana pun di robot ini,
            // jadi ia dilaporkan operator. Tanpa jalur ini, dua angka terbesar
            // di tabel penilaian (100 dan 200) tidak pernah masuk hitungan.
            float p[1] = {0};
            if (argFloats(s, p, 1) >= 1) {
                gSkor.setBersihR7((uint8_t)p[0]);
                Serial.print("Kebersihan R-7 dicatat: tingkat ");
                Serial.println((int)p[0]);
            }
            gSkor.cetak(misi.waktuMisiDetik());
            break;
        }

        case 'J': {  // J<cm> = maju/mundur sampai LiDAR BELAKANG membaca <cm>
            // DUA ARAH, dan arahnya dipilih firmware dari bacaan sekarang:
            // bacaan di ATAS sasaran berarti terlalu jauh dari dinding
            // belakang (MUNDUR), di bawah sasaran berarti terlalu dekat
            // (MAJU). Operator cukup menyebut jarak yang diinginkan.
            //
            // Satu-satunya gerak maju/mundur berumpan-balik di firmware ini.
            // 'w' bisa kedua arah, tapi buta terhadap sasaran dan
            // perpindahannya terkuantisasi satu langkah gait -- tidak bisa
            // dipakai memperbaiki selisih beberapa sentimeter.
            //
            // Seluruh penjaganya ada di Navigation::setelBelakangMulai dan
            // setelBelakangUpdate, sama seperti 'V'. Penjaganya BERBEDA per
            // arah: mundur diawasi LiDAR belakang, maju diawasi LiDAR depan.
            float p[1] = {0};
            if (argFloats(s, p, 1) < 1 || p[0] <= 0.0f) {
                Serial.println("Format: J<cm> maju/mundur sampai LiDAR BELAKANG membaca <cm>.");
                Serial.println("  misal 'J20' = setel jarak dinding belakang jadi 20 cm.");
                Serial.println("  Arah dipilih sendiri: bacaan sekarang lebih besar dari");
                Serial.println("  sasaran -> MUNDUR, lebih kecil -> MAJU. 'l' untuk melihat.");
                break;
            }
            nav.setelBelakangMulai((int)p[0]);
            break;
        }

        case 'H': {  // H<amp> = geser SATU siklus gait, + kanan, - kiri
            // GESER TANPA LiDAR, dan itu memang gunanya: penengahan korban
            // K-3/K-4 beracuan KAMERA, bukan dinding, jadi 'V' tidak bisa
            // dipakai. Rotasi juga tidak -- capit terhalang reruntuhan.
            //
            // SATU SIKLUS PENUH, bukan durasi dalam detik. Uji 6 Sep 2026
            // mencatat 'w 0 -0.6 3' -> ~32 cm dan 'w 0 -0.6 0.5' -> ~36 cm:
            // semburan yang lebih pendek dari satu siklus berhenti di fase
            // sapuan yang berbeda-beda, jadi dua perintah identik berbeda tiga
            // kali lipat. Mengunci durasinya ke satu siklus membuat kuantisasi
            // itu jadi SATUAN, bukan galat -- perpindahannya berulang, dan
            // Raspi bisa memanggilnya berkali-kali untuk jarak yang lebih jauh.
            //
            // Berapa cm per siklus TIDAK diketahui firmware dan tidak ditebak
            // di sini: ia bergantung amplitudo dan profil gait. Ukur sekali
            // dengan penggaris, lalu simpan angkanya di sisi Raspi.
            if (!robot.isArmed()) {
                Serial.println("DITOLAK: servo masih LEMAS. Topang robot lalu ketik 'b'.");
                break;
            }
            float p[1] = {0};
            if (argFloats(s, p, 1) < 1 || p[0] == 0.0f) {
                Serial.println("Format: H<amp> geser satu siklus gait. + kanan, - kiri.");
                Serial.println("  misal 'H0.2' = satu siklus ke kanan pada amplitudo 0,2.");
                Serial.println("  Amplitudo menentukan panjang langkah; ukur cm-nya sekali.");
                break;
            }
            const float amp = clampf(p[0], -1.0f, 1.0f);
            nav.navBerhenti("diambil alih perintah geser satu siklus.");
            gerakMaju = 0.0f; gerakGeser = amp;
            int sisa = jarakArahJalan();
            if (sisa >= 0 && sisa <= GERAK_AMAN_CM) {
                gerakGeser = 0.0f;
                Serial.print("DITOLAK: sudah ada sesuatu "); Serial.print(sisa);
                Serial.print(" cm di arah itu (batas aman ");
                Serial.print(GERAK_AMAN_CM); Serial.println(" cm).");
                break;
            }
            if (sisa < 0)
                Serial.println("! PENJAGA BUTA: tak ada sensor sah di arah itu.");
            // Siklus profil yang SEDANG berlaku, bukan GAIT_CYCLE_TIME tetap:
            // T1/T2 punya siklus yang berbeda, dan memakai angka tetap akan
            // memotong sapuan di tengah pada profil yang lebih lambat.
            const uint32_t siklus = (uint32_t)robot.gaitProfile().cycleTime;
            gerakSampai = millis() + siklus;
            robot.walk(0.0f, amp, 0.0f);
            Serial.print("Geser SATU siklus ("); Serial.print(siklus);
            Serial.print(" ms) amplitudo "); Serial.print(amp, 2);
            Serial.println(amp > 0 ? " (KANAN)" : " (KIRI)");
            break;
        }

        case 'V': {  // V<cm> ratakan ke dinding KANAN, V-<cm> ke dinding KIRI
            // Kenapa satu huruf untuk dua sisi: tandanya sudah membedakan, dan
            // dua perintah terpisah berarti dua tempat untuk salah. Seluruh
            // penjaganya (servo lemas, sensor buta, halangan di arah geser)
            // ada di Navigation::ratakanMulai, dipakai bersama ruas HNT_SISI.
            float p[1] = {0};
            if (argFloats(s, p, 1) < 1 || p[0] == 0.0f) {
                Serial.println("Format: V<cm> ratakan ke dinding KANAN, V-<cm> ke dinding KIRI.");
                Serial.println("  misal 'V16' = geser sampai sensor KANAN-DPN membaca 16 cm,");
                Serial.println("        'V-16' = sampai sensor KIRI-DPN membaca 16 cm.");
                break;
            }
            nav.ratakanMulai(p[0] < 0.0f, (int)fabsf(p[0]));
            break;
        }

        case 'h': // Bantuan
            Serial.println("\n--- BANTUAN PERINTAH SERIAL ---");
            Serial.println("KOMPAS:");
            Serial.println("  c[0-3] : Catat arah (0=U, 1=T, 2=S, 3=B)");
            Serial.println("  k      : Cetak tabel kompas");
            Serial.println("  e / E  : Simpan / Muat dari EEPROM 1792");
            Serial.println("  y      : Hidup/matikan aliran yaw terus-menerus");
            Serial.println("LIDAR:");
            Serial.println("  l      : Tabel jarak keenam sensor");
            Serial.println("  L      : Hidup/matikan aliran LiDAR ( L<ms> untuk atur jeda )");
            Serial.println("  I      : Pindai bus I2C + INIT ULANG sensor yang mati");
            Serial.println("  j      : Jejak statistik semua sensor 5 detik (cari hantu)");
            Serial.println("  j<ch> <detik> : Jejak satu channel, mis. j5 10");
            Serial.println("  u      : Uji isolasi SEMUA sensor -> tabel kesimpulan");
            Serial.println("  u<ch> <detik> : Uji isolasi satu sensor, mis. u5 4");
            Serial.println("NAVIGASI OTONOM (non-blokir):");
            Serial.println("  f      : Jalan mengikuti dinding KIRI");
            Serial.println("  F      : Jalan mengikuti dinding KANAN");
            Serial.println("  p / P  : Ikut dinding KIRI/KANAN + terkunci kompas arena");
            Serial.println("  U<cm>  : NAIK TANGGA -- T4+Z1+F+D<cm>+i1 sekaligus, urutannya dijaga");
            Serial.println("  v      : Status navigasi + jarak sekitar");
            Serial.println("  i1/i0  : Abaikan / pakai lagi sensor depan (turunan; buta ke depan)");
            Serial.println("  N0..N3 : Kemudi dinding. bit0 = SAMAR(fuzzy), bit1 = turunan dari SUDUT");
            Serial.println("           N0=PD/waktu  N1=SAMAR/waktu  N2=PD/SUDUT  N3=SAMAR/SUDUT");
            Serial.println("  Z1/Z0  : Menengah lorong (kiri-kanan) / ikut satu dinding");
            Serial.println("  Y      : Sudut badan terhadap dinding, dari SEPASANG sensor tiap sisi");
            Serial.println("  Y0     : Catat bias pemasangan -- beri saat robot SEJAJAR lorong");
            Serial.println("  T      : Cetak profil medan yang sedang berlaku");
            Serial.println("  T[0-4] : Ganti profil SAMBIL BERJALAN (di-ramp, tanpa 'b')");
            Serial.println("           0=datar  1=tangga  2=merunduk/turunan  3=sempit  4=kail");
            Serial.println("  D      : Jarak tempuh, keadaan rem, dan skala odometri");
            Serial.println("  D<cm>  : Nolkan jarak lalu pasang rem di <cm> (misal D80)");
            Serial.println("  D0     : Nolkan jarak dan lepas rem");
            Serial.println("  Ds<f>  : Faktor slip odometri, RAM saja (misal Ds1.05)");
            Serial.println("  s/x/Enter : Hentikan navigasi");
            Serial.println("  y<ms>  : Aliran yaw dengan jeda tertentu (50-5000, misal y100)");
            Serial.println("EEPROM & KALIBRASI GERAK:");
            Serial.println("  K      : Tabel kalibrasi pivot (EEPROM 2048)");
            Serial.println("  S      : Simpan hasil kalibrasi 'C' ke EEPROM 2048");
            Serial.println("  M      : Cetak peta EEPROM + kapasitas chip");
            Serial.println("PARAMETER (gain PD, gait, pulse -- tanpa kompilasi ulang):");
            Serial.println("  q          : Daftar semua parameter + rentang sahnya");
            Serial.println("  q<nama>    : Lihat satu parameter (mis. qwall)");
            Serial.println("  Q<nm> <nl> : Ubah parameter (mis. Qwall.kp 0.012)");
            Serial.println("  W          : Simpan parameter ke EEPROM 0");
            Serial.println("PIVOT (non-blokir -- 's'/'x'/Enter membatalkan):");
            Serial.println("  o[0-3] : Pivot menuju arah arena");
            Serial.println("  O[der] : Pivot relatif (misal O90)");
            Serial.println("  v      : Status pivot/navigasi yang sedang berjalan");
            Serial.println("  C[sik] : Kalibrasi pivot (MASIH memblokir sampai selesai)");
            Serial.println("GERAK DASAR:");
            Serial.println("  b      : Berdiri diam (sekaligus menghidupkan servo)");
            Serial.println("  b[mm]  : Berdiri diam + atur tinggi badan (40-160, misal b100)");
            Serial.println("  w      : Jalan Maju");
            Serial.println("  V<cm>  : Ratakan ke dinding KANAN (V-<cm> = dinding KIRI)");
            Serial.println("           Geser menyamping sampai sensor sisi membaca <cm>.");
            Serial.println("  s      : Stop (servo tetap hidup)");
            Serial.println("  Enter  : Rem Darurat (vektor gerak = 0)");
            Serial.println("BODY KINEMATICS (kaki diam, badan bergerak):");
            Serial.println("  r            : Cetak pose badan sekarang");
            Serial.println("  r<rol> <pit> <yaw> : Set rotasi badan, derajat (misal: r10 0 0)");
            Serial.println("                 roll+ = miring KANAN, pitch+ = MENDONGAK, yaw+ = belok KIRI");
            Serial.println("  t<x> <y> <z> : Set geser badan, mm (misal: t0 0 -20 untuk merunduk)");
            Serial.println("  J<cm>        : Maju/mundur sampai LiDAR BELAKANG membaca <cm>");
            Serial.println("  X            : Perkiraan poin. X0/X1/X2 = lapor kebersihan R-7");
            Serial.println("  H<amp>       : Geser SATU siklus gait, + kanan / - kiri (tanpa LiDAR)");
            Serial.println("  0            : Nolkan pose badan");
            Serial.println("  B            : Demo sapuan 6 sumbu (18 detik)");
            Serial.println("  z            : Goyang roll bergelombang, terus-menerus (pajangan)");
            Serial.println("  z<amp> <per> <fase> : amplitudo der, periode detik, fase pitch der");
            Serial.println("                 contoh: z12 2   atau  z15 3 90 (badan menelusuri kerucut)");
            Serial.println("LENGAN (bahu, siku, grip -- depan & belakang):");
            Serial.println("  a<jkn> <tgi> [prg] : Lengan DEPAN jangkauan/tinggi mm, pergelangan der");
            Serial.println("                       (tanpa <prg> pergelangan tak disentuh)");
            Serial.println("  aa / at            : Jalankan SEKUENS korban AMBIL / TARUH utuh,");
            Serial.println("                       sama persis dengan yang dipakai misi. 'm0' berhenti.");
            Serial.println("  as<bhu> <sku> <prg>: Tembak KETIGA sendi langsung, der geometris,");
            Serial.println("                       TANPA IK. as0 0 0 = baseline. Untuk hard-code pose.");
            Serial.print(  "                       <jkn> menunjuk PERGELANGAN; grip ");
            Serial.print(HAND_LENGTH, 0); Serial.println(" mm lebih jauh");
            Serial.print(  "  NETRAL       : a");
            Serial.print(fabsf(ARM_ORIGINS[ARM_DEPAN][1]) + UPPERARM_LENGTH, 0);
            Serial.print(" "); Serial.print(ARM_ORIGINS[ARM_DEPAN][2] + FOREARM_LENGTH, 0);
            Serial.println(" 0 lalu g50/G50 -- kelima servo lengan di 1500 us");
            Serial.println("                 (lengan atas mendatar ke depan, lengan bawah tegak)");
            Serial.printf("  REHAT        : as%.0f %.0f %.0f  -- terlipat di atas badan (= perintah 'R')\n",
                          (double)REHAT_BAHU, (double)REHAT_SIKU, (double)REHAT_PERGELANGAN);
            Serial.println("                 (pose istirahat; DIPERINTAH, bukan pose pasang horn)");
            Serial.printf("  KORBAN siap  : as%.0f %.0f %.0f  -- mendekat, mengangkat, menggendong\n",
                          (double)KORBAN_SIAP_BAHU, (double)KORBAN_SIAP_SIKU,
                          (double)KORBAN_SIAP_PRG);
            Serial.printf("  KORBAN jepit : as%.0f %.0f %.0f  -- turun dan menjepit\n",
                          (double)KORBAN_JEPIT_BAHU, (double)KORBAN_JEPIT_SIKU,
                          (double)KORBAN_JEPIT_PRG);
            Serial.printf("  KORBAN lepas : as%.0f %.0f %.0f  -- titik lepas di safe zone\n",
                          (double)KORBAN_LEPAS_BAHU, (double)KORBAN_LEPAS_SIKU,
                          (double)KORBAN_LEPAS_PRG);
            Serial.printf("                 (sudut sendi TETAP, tidak ikut profil; gerbang LiDAR\n");
            Serial.printf("                  depan %d cm -- jalankan utuh dengan aa / at)\n",
                          KORBAN_JARAK_CM);
            Serial.println("  g<0-100>     : Grip depan (0=menutup, 100=membuka)");
            Serial.println("  G<0-100>     : Grip belakang (lengan belakang HANYA punya grip)");
            Serial.println("  R            : Lengan REHAT -- terlipat di atas badan (lihat di atas)");
            Serial.println("  n            : Matikan servo kedua lengan");
            Serial.println("MISI (lapisan di atas navigasi -- lintasan ada di tabel RUAS[]):");
            Serial.println("  m           : Status misi");
            Serial.println("  m4          : TABEL LINTASAN -- semua ruas, mana yang belum diukur");
            Serial.println("  m1          : MULAI dari ruas 0 (HOME)");
            Serial.println("  m4 <idx>    : Mulai dari satu ruas saja -- untuk menguji per rintangan");
            Serial.println("  m6 <idx>    : MODE UKUR -- jalan tanpa syarat henti; hentikan di ujung");
            Serial.println("                ruas, robot mencetak sendiri berapa cm yang ditempuh");
            Serial.println("  m0          : Batalkan misi");
            Serial.println("  m7 <idx><cm>: Setel panjang ruas (misal m7 2 55). RAM saja.");
            Serial.println("  m2/m3       : Saat menunggu konfirmasi -> lanjut / ulangi ruas ini");
            Serial.println("  Capit belum terpasang: ruas korban hanya BERHENTI KOSONG lalu lanjut.");
            Serial.println("KESELAMATAN & DIAGNOSTIK:");
            Serial.println("  x      : LEMAS -- PWM mati, servo bebas");
            Serial.println("  d      : Dump kalibrasi + hasil IK per kaki");
            break;

        default:
            Serial.println("Perintah tidak dikenal. Ketik 'h' untuk bantuan.");
    }
}

// ====================================================================
// DEMO OTOMATIS SAAT MENYALA (sementara -- matikan lewat DEMO_BOOT di config.h)
// ====================================================================
// Non-blokir, seperti semua yang lain di loop utama: ia hanya memeriksa jam
// lalu memanggil handleCmd() dengan perintah yang PERSIS sama dengan yang
// diketik manusia. Tidak ada logika gerak yang disalin ke sini, jadi demo
// tidak bisa berperilaku beda dari perintah manualnya.
//
// Ditulis sebagai state machine, bukan rangkaian delay(). Selama delay()
// parser serial mati -- artinya demo tidak bisa dibatalkan, IMU dan LiDAR
// berhenti diperbarui, dan servo tidak di-commit. Persis kesalahan yang sudah
// dibersihkan dari pivotKe() sebelumnya; jangan dibawa masuk lagi lewat demo.
#if DEMO_BOOT
enum BootFase : uint8_t { BOOT_TUNDA, BOOT_BERDIRI, BOOT_GOYANG, BOOT_SELESAI };
static BootFase bootFase = BOOT_TUNDA;
static uint32_t bootT    = 0;

// Menjalankan satu perintah lewat parser normal. handleCmd() menerima char*
// dan boleh menulisi bufernya, jadi literalnya disalin dulu -- menulisi string
// literal itu perilaku tak terdefinisi.
static void bootJalankan(const char* perintah) {
    char buf[24];
    strncpy(buf, perintah, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    handleCmd(buf);
}

// Membatalkan URUTAN-nya saja. Kalau goyang sudah terlanjur jalan, ia sengaja
// DIBIARKAN: sejak operator menyentuh keyboard, dialah yang memegang kendali,
// dan menghentikan pajangan gara-gara ia mengetik 'l' untuk melihat LiDAR itu
// mengejutkan. Yang hilang cuma '0' otomatis di ujung -- jadi keadaannya
// disebutkan, bukan dibiarkan jadi kejutan.
static void bootBatal(const char* alasan) {
    if (bootFase == BOOT_SELESAI) return;
    bool masihGoyang = (bootFase == BOOT_GOYANG) && goyangOn;
    bootFase = BOOT_SELESAI;
    Serial.print("\nDemo menyala DIBATALKAN: "); Serial.println(alasan);
    if (masihGoyang)
        Serial.println("  goyang MASIH jalan -- 'z' atau '0' untuk menghentikannya.");
}

static void bootUpdate() {
    if (bootFase == BOOT_SELESAI) return;
    uint32_t now = millis();

    switch (bootFase) {
        case BOOT_TUNDA: {
            // Hitung mundur yang terlihat. Operator harus TAHU servo akan
            // menyala, bukan dikejutkan olehnya -- robot ini biasanya menyala
            // dalam keadaan lemas, jadi berdiri sendiri itu tidak terduga.
            static uint32_t tCetak = 0;
            if (now - tCetak >= 1000) {
                tCetak = now;
                uint32_t sisa = (now < DEMO_BOOT_TUNDA) ? (DEMO_BOOT_TUNDA - now) : 0;
                Serial.print("  demo menyala dalam "); Serial.print((sisa + 999) / 1000);
                Serial.println(" detik -- ketik apa saja lalu Enter untuk batal");
            }
            if (now >= DEMO_BOOT_TUNDA) {
                Serial.println("Demo menyala: berdiri.");
                bootJalankan("b");
                bootFase = BOOT_BERDIRI;
                bootT = now;
            }
            break;
        }

        case BOOT_BERDIRI:
            // Beri kaki waktu menetap dulu. Menggoyang badan saat gait masih
            // menuju pose home membuat kedua gerakan bertumpuk.
            if (now - bootT >= DEMO_BOOT_BERDIRI) {
                Serial.print("Demo menyala: '"); Serial.print(DEMO_BOOT_PERINTAH);
                Serial.print("' selama "); Serial.print(DEMO_BOOT_LAMA / 1000);
                Serial.println(" detik.");
                bootJalankan(DEMO_BOOT_PERINTAH);
                bootFase = BOOT_GOYANG;
                bootT = now;
            }
            break;

        case BOOT_GOYANG:
            // goyangOn bisa mati lebih dulu kalau sesuatu mengambil alih pose
            // badan. Kalau begitu, jangan menunggu sampai waktunya habis lalu
            // menimpa apa pun yang sedang berjalan dengan '0'.
            if (!goyangOn) { bootBatal("goyang dihentikan dari luar."); break; }
            if (now - bootT >= DEMO_BOOT_LAMA) {
                bootJalankan("0");
                bootFase = BOOT_SELESAI;
                Serial.println("Demo menyala SELESAI -- robot tetap berdiri.");
                Serial.println("  (matikan permanen lewat DEMO_BOOT 0 di config.h)");
            }
            break;

        default: break;
    }
}
#endif  // DEMO_BOOT

// ====================================================================
// FUNGSI SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {} // Tunggu serial maksimal 3 detik

    // Port pemicu deteksi korban ke Raspi 5. Kalau KORBAN_SERIAL masih
    // Serial (default), baris ini cuma pengulangan tak berbahaya -- ia ada
    // supaya ganti define ke Serial1 tidak perlu ingat menambah begin().
    KORBAN_SERIAL.begin(KORBAN_BAUD);

    Serial.println("\n\nMemulai Hexapod Unlimited...");

    // 0. KALIBRASI DULU -- HARUS SEBELUM robot.begin()!
    //    gCalib adalah global yang ter-zero-init. Tanpa ini SERVO_PULSE_MIN/MAX = 0,
    //    sehingga angleToPulse() selalu menghasilkan 0 us (servo tak dapat sinyal /
    //    lemas total) dan GAIT_SLEW_RATE = 0 (gait tak pernah jalan).
    //    HexaServos::begin(), HexaArm::begin() dan profileFlat() membaca nilai ini.
    if (Calib::load()) {
        Serial.println("Calib: data EEPROM valid dimuat (alamat 0).");
    } else {
        Serial.println("Calib: EEPROM kosong/rusak/versi beda -> default dipakai & ditulis ulang.");
    }

    // 1. Inisialisasi Perangkat Keras
    imu.begin();
    robot.begin();

    if (!lidar.begin())
        Serial.println("LidarArray: ada sensor yang tidak siap -- ketik 'I' untuk memindai bus.");

    // 2. Periksa peta EEPROM terhadap kapasitas chip yang sebenarnya.
    //    Tata letaknya sendiri sudah dijaga static_assert saat kompilasi;
    //    ini menangkap kasus firmware dipindah ke papan dengan EEPROM lebih kecil.
    eeMapPeriksa(true);

    // 3. Muat Data Memori
    nav.begin();

    Serial.println("\nSistem siap -- SERVO MASIH LEMAS (PWM mati).");
    Serial.println("Topang robot, lalu ketik 'b' untuk berdiri. 'x' untuk melemaskan lagi.");
    Serial.println("Ketik 'h' untuk daftar perintah, 'd' untuk diagnostik.");

#if DEMO_BOOT
    Serial.println("\n!! DEMO MENYALA AKTIF -- robot akan BERDIRI SENDIRI.");
    Serial.print(  "   urutan: 'b' -> '"); Serial.print(DEMO_BOOT_PERINTAH);
    Serial.print(  "' "); Serial.print(DEMO_BOOT_LAMA / 1000); Serial.println(" detik -> '0'.");
    Serial.println("   Topang robot SEKARANG. Ketik apa saja lalu Enter untuk membatalkan.");
    Serial.println("   Matikan permanen: DEMO_BOOT 0 di config.h.");
#endif

    // OLED + tombol PALING AKHIR: ia menumpang bus Wire yang baru saja
    // di-init LidarArray, dan layar yang menyala sebelum sensornya siap
    // menampilkan angka yang belum ada.
    tampilan.begin(&misi, &gSkor, kirimDariTombol, &nav);
}

// ====================================================================
// FUNGSI LOOP (Non-Blokir)
// ====================================================================
void loop() {
    // 1. BACA SENSOR (Prioritas Tinggi - Bebas Waktu)
    imu.update();
    lidar.update();     // satu sensor per putaran, non-blokir

    // 2. DEMO BODY KINEMATICS + ALIRAN YAW (keduanya non-blokir)
#if DEMO_BOOT
    bootUpdate();       // urutan demo saat menyala; berhenti sendiri sesudah '0'
#endif
    demoUpdate();
    goyangUpdate();
    yawStreamUpdate();
    lidarStreamUpdate();

    // 3. MISI lalu NAVIGASI OTONOM (keduanya non-blokir).
    //    URUTAN PENTING: misi lebih dulu. Keduanya membaca sampel LiDAR
    //    yang sama, dan yang lebih dulu berhak memutuskan -- itu yang
    //    membuat misi sempat menghentikan navigasi sebelum navigasi keburu
    //    berbelok menghindari benda yang justru sedang dicari.
    misi.update();
    nav.navUpdate();
    tampilan.update();

    // 4. KENDALI GERAK & SERVO (Diatur internal oleh Hexapod)
    robot.update();

    // 5. PERINGATAN JANGKAUAN IK (dibatasi 1x/detik supaya tidak membanjiri)
    static uint32_t tWarn = 0;
    if (!robot.lastPoseInRange() && robot.isArmed() && millis() - tWarn > 1000) {
        tWarn = millis();
        Serial.println("!! Kaki di luar jangkauan IK -- sudut di-clamp, pose tidak dituruti.");
    }

    // 6. PARSER SERIAL MONITOR
    static char buf[40];
    static uint8_t len = 0;
    // Byte sebelumnya, untuk mengenali CRLF. static, karena kedua byte sebuah
    // akhir baris bisa tiba di dua pemanggilan loop() yang berbeda.
    static char chLalu = 0;

    while (Serial.available()) {
        char ch = Serial.read();
        char lalu = chLalu;
        chLalu = ch;

#if DEMO_BOOT
        // Byte APA PUN membatalkan demo menyala -- termasuk Enter kosong, yang
        // memang rem daruratnya. Dibatalkan di sini, sebelum karakternya
        // diproses, supaya perintah yang diketik operator tetap berjalan
        // normal sesudahnya dan tidak berebut dengan urutan demo.
        bootBatal("ada perintah dari pengguna.");
#endif

        // Eksekusi jika ditekan Enter
        if (ch == '\n' || ch == '\r') {
            // Satu akhir baris CRLF adalah DUA byte. Yang kedua bukan baris
            // kosong, jadi ia tidak boleh dibaca sebagai rem darurat: tanpa
            // penjaga ini, terminal apa pun yang mengirim CRLF (konsol
            // Windows, PuTTY bawaan, banyak monitor serial) membuat TIAP
            // perintah langsung disusul rem darurat -- CR menjalankan
            // perintahnya, LF menghentikan robot. Yang terlihat di lapangan
            // adalah robot yang menerima perintah lalu berhenti sendiri.
            //
            // Enter kosong yang SUNGGUHAN tetap jadi rem darurat: byte
            // sebelumnya bukan CR, atau CR itu sendiri yang tiba sendirian
            // dan dialah yang memicu.
            if (ch == '\n' && lalu == '\r') continue;

            buf[len] = 0; // Kunci string

            if (len) {
                handleCmd(buf); // Masuk ke parser
            } else {
                // Fitur Keselamatan: Tekan Enter kosong untuk rem darurat
                gerakSampai = 0;
                misi.batal("rem darurat.");
                nav.navBerhenti("rem darurat.");
                if (demoOn) demoStop("rem darurat.");
                if (goyangOn) goyangStop("rem darurat.");
                robot.stop();
                nav.remJarakLepas();
                Serial.println("!! REM DARURAT (Vektor = 0) !!");
            }
            len = 0; // Bersihkan buffer untuk perintah berikutnya

        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch; // Tampung karakter ke buffer
        }
    }
}
