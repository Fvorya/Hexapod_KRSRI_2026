#include <Arduino.h>
#include "config.h"
#include "Calib.h"      // WAJIB: gParam/gOffset/gTrim/gInvert (pulse min-max, GAIT_*, dll)
#include "EEMap.h"      // peta EEPROM + penjaga static_assert
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"

// ====================================================================
// DEKLARASI OBJEK GLOBAL
// ====================================================================
Imu imu;
Hexapod robot;                // Digunakan oleh Navigation
LidarArray lidar;              // 6x VL53L0X lewat mux TCA9548A di bus Wire
Navigation nav(imu, robot, lidar);  // Menyuntikkan referensi IMU, Motion, LiDAR

// ====================================================================
// DEMO BODY KINEMATICS (non-blokir)
// Menyapu 6 sumbu berurutan: roll, pitch, yaw, geser X, Y, Z.
// Tiap sumbu satu putaran sinus penuh (0 -> + -> 0 -> - -> 0) supaya selalu
// kembali ke netral sebelum pindah sumbu -- tak pernah ada lompatan.
// Kaki TETAP DI TEMPAT; yang bergerak hanya badan. Itulah gunanya: kalau
// telapak ikut bergeser di lantai, berarti body kinematics belum benar.
// ====================================================================
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
    Serial.print(" der | gyroZ ");
    Serial.print(imu.gyroZ(), 1); Serial.print(" der/s");

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
            if (d1 > 3) { Serial.println("c0=UTARA c1=TIMUR c2=SELATAN c3=BARAT"); break; }
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

        case 'v':   // status navigasi + jarak sekitar
            nav.navStatus();
            break;

        case 'I':   // Pindai bus I2C LiDAR + init ulang sensor yang belum aktif
            lidar.pindaiI2C();
            break;

        case 'M':   // Cetak peta EEPROM + cek kapasitas chip sebenarnya
            eeMapPeriksa(true);
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
            robot.setBodyRotation(a[0], a[1], a[2]);
            robot.update();               // hitung ulang supaya laporan di bawah akurat
            cetakPoseBadan();
            break;
        }

        case 't': {  // t<x> <y> <z>  (mm)
            float a[3] = {0, 0, 0};
            uint8_t n = argFloats(s, a, 3);
            if (n == 0) { cetakPoseBadan(); break; }
            if (demoOn) demoStop("diambil alih perintah manual.");
            robot.setBodyTranslation(a[0], a[1], a[2]);
            robot.update();
            cetakPoseBadan();
            break;
        }

        case '0':    // Nolkan pose badan (kembali tegak & terpusat)
            if (demoOn) demoStop("dinolkan.");
            robot.setBodyRotation(0, 0, 0);
            robot.setBodyTranslation(0, 0, 0);
            Serial.println("Pose badan dinolkan.");
            break;

        case 'B':    // Demo sapuan 6 sumbu (18 detik), tekan lagi untuk berhenti
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

            float p[2] = {0, 0};
            if (argFloats(s, p, 2) < 2) {
                Serial.print("Format: "); Serial.print(c);
                Serial.println("<jangkauan> <tinggi>  (mm, dari pusat badan). Misal: a70 20");
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
                Serial.print(" mm, tinggi ");   Serial.print(p[1], 1); Serial.println(" mm");
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
            Serial.print(" -> "); Serial.print(clampf(p[0], 0.0f, 100.0f), 0); Serial.println("%");
            break;
        }

        case 'n':   // Matikan kedua servo lengan (kaki tetap hidup)
            robot.armEnable(ARM_DEPAN, false);
            robot.armEnable(ARM_BELAKANG, false);
            Serial.println("Servo kedua lengan DIMATIKAN (kaki tidak terpengaruh).");
            break;

        // --- 6. KESELAMATAN SERVO ---
        case 'x': // Lemas darurat: PWM mati, servo bebas
            nav.navBerhenti("servo dilemaskan.");
            if (demoOn) demoStop("servo dilemaskan.");
            robot.disarm();
            break;

        case 'd': // Dump diagnostik (kalibrasi + IK per kaki)
            robot.debugDump();
            break;

        // --- 7. KONTROL ROBOT DASAR ---
        case 'b': {   // Berdiri diam (pose netral). Opsional: b<mm> untuk atur tinggi badan.
            if (demoOn) demoStop("kembali ke pose berdiri.");
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
            nav.navBerhenti("dihentikan pengguna.");   // WAJIB: kalau tidak,
            robot.stop();                              // navUpdate() menyalakannya lagi
            Serial.println("Robot berhenti.");
            break;

        case 'w': // Walk (Maju manual)
            nav.navBerhenti("diambil alih perintah manual.");
            robot.walk(NAV_FWD_SPEED, 0.0f, 0.0f);
            Serial.println("Robot maju.");
            break;

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
            Serial.println("NAVIGASI OTONOM (non-blokir):");
            Serial.println("  f      : Jalan mengikuti dinding KIRI");
            Serial.println("  F      : Jalan mengikuti dinding KANAN");
            Serial.println("  p / P  : Ikut dinding KIRI/KANAN + terkunci kompas arena");
            Serial.println("  v      : Status navigasi + jarak sekitar");
            Serial.println("  s/x/Enter : Hentikan navigasi");
            Serial.println("  y<ms>  : Aliran yaw dengan jeda tertentu (50-5000, misal y100)");
            Serial.println("EEPROM & KALIBRASI GERAK:");
            Serial.println("  K      : Tabel kalibrasi pivot (EEPROM 2048)");
            Serial.println("  S      : Simpan hasil kalibrasi 'C' ke EEPROM 2048");
            Serial.println("  M      : Cetak peta EEPROM + kapasitas chip");
            Serial.println("PIVOT (non-blokir -- 's'/'x'/Enter membatalkan):");
            Serial.println("  o[0-3] : Pivot menuju arah arena");
            Serial.println("  O[der] : Pivot relatif (misal O90)");
            Serial.println("  v      : Status pivot/navigasi yang sedang berjalan");
            Serial.println("  C[sik] : Kalibrasi pivot (MASIH memblokir sampai selesai)");
            Serial.println("GERAK DASAR:");
            Serial.println("  b      : Berdiri diam (sekaligus menghidupkan servo)");
            Serial.println("  b[mm]  : Berdiri diam + atur tinggi badan (40-160, misal b100)");
            Serial.println("  w      : Jalan Maju");
            Serial.println("  s      : Stop (servo tetap hidup)");
            Serial.println("  Enter  : Rem Darurat (vektor gerak = 0)");
            Serial.println("BODY KINEMATICS (kaki diam, badan bergerak):");
            Serial.println("  r            : Cetak pose badan sekarang");
            Serial.println("  r<rol> <pit> <yaw> : Set rotasi badan, derajat (misal: r10 0 0)");
            Serial.println("                 roll+ = miring KANAN, pitch+ = MENDONGAK, yaw+ = belok KIRI");
            Serial.println("  t<x> <y> <z> : Set geser badan, mm (misal: t0 0 -20 untuk merunduk)");
            Serial.println("  0            : Nolkan pose badan");
            Serial.println("  B            : Demo sapuan 6 sumbu (18 detik)");
            Serial.println("LENGAN (bahu, siku, grip -- depan & belakang):");
            Serial.println("  a<jkn> <tgi> : Lengan DEPAN ke jangkauan/tinggi mm (misal: a70 20)");
            Serial.println("  A<jkn> <tgi> : Lengan BELAKANG");
            Serial.println("  g<0-100>     : Grip depan (0=menutup, 100=membuka)");
            Serial.println("  G<0-100>     : Grip belakang");
            Serial.println("  n            : Matikan servo kedua lengan");
            Serial.println("KESELAMATAN & DIAGNOSTIK:");
            Serial.println("  x      : LEMAS -- PWM mati, servo bebas");
            Serial.println("  d      : Dump kalibrasi + hasil IK per kaki");
            break;

        default:
            Serial.println("Perintah tidak dikenal. Ketik 'h' untuk bantuan.");
    }
}

// ====================================================================
// FUNGSI SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {} // Tunggu serial maksimal 3 detik

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
}

// ====================================================================
// FUNGSI LOOP (Non-Blokir)
// ====================================================================
void loop() {
    // 1. BACA SENSOR (Prioritas Tinggi - Bebas Waktu)
    imu.update();
    lidar.update();     // satu sensor per putaran, non-blokir

    // 2. DEMO BODY KINEMATICS + ALIRAN YAW (keduanya non-blokir)
    demoUpdate();
    yawStreamUpdate();
    lidarStreamUpdate();

    // 3. NAVIGASI OTONOM (non-blokir; diam saja bila mode NAV_DIAM)
    nav.navUpdate();

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

    while (Serial.available()) {
        char ch = Serial.read();

        // Eksekusi jika ditekan Enter
        if (ch == '\n' || ch == '\r') {
            buf[len] = 0; // Kunci string

            if (len) {
                handleCmd(buf); // Masuk ke parser
            } else {
                // Fitur Keselamatan: Tekan Enter kosong untuk rem darurat
                nav.navBerhenti("rem darurat.");
                if (demoOn) demoStop("rem darurat.");
                robot.stop();
                Serial.println("!! REM DARURAT (Vektor = 0) !!");
            }
            len = 0; // Bersihkan buffer untuk perintah berikutnya

        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch; // Tampung karakter ke buffer
        }
    }
}
