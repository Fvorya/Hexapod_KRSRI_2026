// CELAH KAKI TERHADAP DINDING saat mode ikut-dinding.
//
// sim_wall menyetel gain; simulasi ini menjawab pertanyaan yang berbeda dan
// lebih fisik: seberapa dekat UJUNG KAKI sampai ke dinding? Sensor mengukur
// dari SISI BADAN, sedangkan yang menabrak dinding adalah kaki tengah yang
// menjulur jauh di luar badan. Selisih keduanya harus ikut dihitung, kalau
// tidak setpoint yang kelihatan aman di layar tetap membuat kaki menggesek.
//
// Tiga bagian:
//   1. GEOMETRI  -- titik terlebar robot, diukur dari HexaGait yang asli.
//   2. TABEL     -- celah kaki untuk tiap setpoint, pada beberapa asumsi
//                   posisi pasang sensor.
//   3. LINGKAR TERTUTUP -- jalan menyusur dinding, mulai dari posisi yang
//                   SUDAH terlalu dekat, lalu catat celah kaki terkecil.
//                   Dibandingkan: aturan lama (satu PD, setpoint 13) vs
//                   aturan baru (tiga pita, setpoint 19).
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"
#include "HexaGait.h"

static Imu imu; static Hexapod robot; static LidarArray lidar;
static Navigation nav(imu, robot, lidar);

// --- geometri: diisi bagian 1, dipakai bagian 2 & 3 ---
static float LEBAR_KAKI_CM = 0.0f;   // |x| ujung kaki terjauh, cm

// Keempat LiDAR samping menghadap TEGAK LURUS dinding (ch0/ch1 ke kiri,
// ch3/ch4 ke kanan) -- bukan menyerong. Hanya ch5/ch2 yang searah sumbu
// panjang. Diukur dari gambar tata letak, sensornya duduk ~5,0 cm dari pusat
// badan; nilai itu tetap diuji dalam beberapa variasi karena tidak semua orang
// memasangnya di titik yang sama.
static const float SUDUT_BERKAS_DER = 90.0f;   // dari arah DEPAN
static float X_SENSOR_CM = 5.0f;

// --- plant lorong, sama gaya dengan sim_wall ---
static const float LEBAR_CM   = 60.0f;
static const float PANJANG_CM = 900.0f;
static const float V_PENUH    = 10.7f;   // cm/s pada maju 1.0 (lihat sim_laju)
static const float W_PENUH    = 45.5f;   // der/s pada putar 1.0
static float rx, ry, rth, derauCm;
static uint32_t rngS = 2468;
static float rnd() { rngS = rngS * 1103515245u + 12345u; return ((rngS >> 16) & 0x7FFF) / 32767.0f * 2.0f - 1.0f; }

// Hantu menetap yang dilaporkan sensor asli robot ini (uji 'u': 2,5-5 cm).
// 0 = tidak ada hantu.
static float hantuCm = 0.0f;

static uint16_t jarakSisiKiri() {
    if (hantuCm > 0.0f) return (uint16_t)(hantuCm * 10.0f);
    // Berkas tegak lurus: panjang lintasan = jarak tegak lurus / cos(yaw).
    // Ia memanjang saat robot menoleh KE MANA PUN -- simetris, jadi tidak ada
    // arah yang diuntungkan. Itulah bedanya dengan pemasangan menyerong.
    float a = deg2rad(SUDUT_BERKAS_DER - rth);
    float k = cosf(deg2rad(90.0f) - a);
    if (k < 0.15f) return 8190;
    float d = (rx - X_SENSOR_CM) / k + derauCm * rnd();
    return (uint16_t)std::max(3.0f, d * 10.0f);
}

static void set1X(int ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}
static void suapLidar() {
    float depan = (PANJANG_CM - ry) / std::max(0.3f, cosf(deg2rad(rth)));
    for (int i = 0; i < 6; i++) set1X(i, 8190);
    set1X(LIDAR_FRONT,   (uint16_t)std::max(3.0f, depan * 10.0f));
    set1X(LIDAR_KIRI_D, jarakSisiKiri());
}
static void suapYaw() {
    auto enc = [](float d) { return (int16_t)lrintf(d / 180.0f * 32768.0f); };
    float y = rth; while (y > 180) y -= 360; while (y < -180) y += 360;
    int16_t v2 = enc(y);
    uint8_t f[11] = { 0x55, 0x53, 0,0, 0,0, (uint8_t)(v2 & 0xFF), (uint8_t)(v2 >> 8), 0,0,0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}

// celahMantap = celah minimum SESUDAH jendela pemulihan. Tanpa pemisahan ini
// celahMin selalu sama dengan posisi awal yang memang sengaja dibuat terlalu
// dekat, jadi ia tak bisa membedakan aturan yang menarik robot menjauh dari
// aturan yang membiarkannya menggantung di situ.
struct Hasil { float celahMin, celahMantap, celahRata; bool berhenti; };

static Hasil jalankan(float mulaiX, float derau, int detik) {
    rx = mulaiX; ry = 0; rth = 0; derauCm = derau;
    for (int i = 0; i < 300; i++) { suapLidar(); lidar.update(); __nowMs += 2; }
    suapYaw(); imu.update();
    nav.navMulai(NAV_DINDING_KIRI);

    Hasil h { 1e9f, 1e9f, 0.0f, false };
    const int PULIH = 8000 / 5;   // 8 detik pertama = jendela pemulihan
    double jum = 0; int n = 0;
    const int TICK_MS = 5, total = detik * 1000 / TICK_MS;
    for (int i = 0; i < total; i++) {
        if (nav.navMode() == NAV_DIAM) { h.berhenti = true; break; }
        __nowMs += TICK_MS;
        float dt = TICK_MS / 1000.0f;
        rth += nav.turnKini() * W_PENUH * dt;
        float v = nav.majuKini() * V_PENUH;
        rx -= v * sinf(deg2rad(rth)) * dt;
        ry += v * cosf(deg2rad(rth)) * dt;
        rx = clampf(rx, LEBAR_KAKI_CM, LEBAR_CM - 0.5f);   // kaki tidak bisa menembus dinding

        suapLidar(); suapYaw();
        imu.update(); lidar.update(); nav.navUpdate(); robot.update();

        float celah = rx - LEBAR_KAKI_CM;
        h.celahMin = std::min(h.celahMin, celah);
        if (i > PULIH) { h.celahMantap = std::min(h.celahMantap, celah); jum += celah; n++; }
    }
    h.celahRata = n ? (float)(jum / n) : -1.0f;
    nav.navBerhenti("selesai.");
    return h;
}

// Aturan LAMA: satu PD tanpa pita dekat. Ditiru dengan menyetel ambangnya ke
// nol -- pita hantu & pita dekat jadi tidak pernah aktif, persis seperti kode
// sebelum perubahan ini.
static void pakaiAturanLama() {
    gParam[K_WALL_SETPOINT] = 13.0f;
    gParam[K_WALL_MIN]      = 0.0f;
}
static void pakaiAturanBaru() {
    gParam[K_WALL_SETPOINT] = PARAM_DEFS[K_WALL_SETPOINT].def;
    gParam[K_WALL_MIN]      = PARAM_DEFS[K_WALL_MIN].def;
}

int main() {
    Calib::load();
    robot.begin(); lidar.begin(); robot.arm();
    for (int i = 0; i < 5; i++) { suapYaw(); imu.update(); __nowMs += 10; }

    // ---------- 1. GEOMETRI ----------
    printf("\n=== 1. TITIK TERLEBAR ROBOT ===\n");
    {
        HexaGait g; g.begin(); g.setMoveVector(0, 0.8f, NAV_WALL_TURN_MAX);
        for (int i = 0; i < 400; i++) { __nowMs += 5; g.update(); }
        float xm = 0;
        for (int i = 0; i < 400; i++) {
            __nowMs += 5; g.update();
            for (int L = 0; L < 6; L++) xm = std::max(xm, fabsf(g.legTargets[L].x));
        }
        LEBAR_KAKI_CM = xm / 10.0f;
        printf("  pangkal coxa terluar : %.0f mm\n", 90.0f);
        printf("  + STAND_RADIUS       : %.0f mm\n", (double)STAND_RADIUS);
        printf("  ujung kaki tengah    : %.0f mm dari pusat badan\n", (double)(90.0f + STAND_RADIUS));
        printf("  diukur saat maju 0,8 + putar %.2f: |x| maks %.1f mm\n", NAV_WALL_TURN_MAX, xm);
        printf("  -> lebar separuh yang dipakai simulasi: %.1f cm\n", LEBAR_KAKI_CM);
        if (fabsf(xm - (90.0f + STAND_RADIUS)) > 0.5f) {
            printf("  GAGAL: kaki tengah ternyata bukan titik terlebar\n"); return 1;
        }
    }

    // ---------- 2. TABEL SETPOINT -> CELAH ----------
    printf("\n=== 2. CELAH KAKI UNTUK TIAP SETPOINT ===\n");
    printf("  berkas sensor %.0f der dari depan = TEGAK LURUS dinding\n", SUDUT_BERKAS_DER);
    printf("  x_dinding = x_sensor + jarak_terbaca   (robot sejajar lorong)\n");
    printf("  kaki menyentuh dinding saat bacaan = %.1f - x_sensor\n\n", LEBAR_KAKI_CM);
    printf("  setpoint |   celah ujung kaki ke dinding (cm)\n");
    printf("   terbaca | x_sensor 4,0  x_sensor 5,0  x_sensor 6,5\n");
    printf("  ---------+------------------------------------------\n");
    for (float sp : {13.0f, 15.0f, 17.0f, 19.0f, 21.0f, 24.0f}) {
        printf("   %5.1f cm |", sp);
        for (float xs : {4.0f, 5.0f, 6.5f})
            printf("   %+9.1f  ", xs + sp - LEBAR_KAKI_CM);
        printf("%s\n", sp == 13.0f ? "   <- setelan lama" :
                       (sp == PARAM_DEFS[K_WALL_SETPOINT].def ? "   <- default baru" : ""));
    }

    // ---------- 3. LINGKAR TERTUTUP ----------
    printf("\n=== 3. JALAN MENYUSUR DINDING, MULAI DARI POSISI TERLALU DEKAT ===\n");
    printf("  lorong %.0f cm, mulai %.0f cm dari dinding (celah kaki awal %.1f cm),\n",
           LEBAR_CM, 18.0f, 18.0f - LEBAR_KAKI_CM);
    printf("  derau sensor +-1,0 cm, 40 detik, x_sensor %.1f cm\n\n", X_SENSOR_CM);
    printf("  aturan               | min sepanjang | min sesudah  | rata sesudah\n");
    printf("                       |      uji      | pulih 8 dtk  | pulih 8 dtk\n");
    printf("  ---------------------+---------------+--------------+-------------\n");

    pakaiAturanLama();
    Hasil lama = jalankan(18.0f, 1.0f, 40);
    printf("  LAMA  (PD, sp 13)    |   %6.2f cm   |  %6.2f cm  |  %6.2f cm %s\n",
           lama.celahMin, lama.celahMantap, lama.celahRata,
           lama.berhenti ? " BERHENTI SENDIRI" : "");

    pakaiAturanBaru();
    Hasil baru = jalankan(18.0f, 1.0f, 40);
    printf("  BARU  (3 pita, sp %.0f)|   %6.2f cm   |  %6.2f cm  |  %6.2f cm %s\n",
           (double)PARAM_DEFS[K_WALL_SETPOINT].def, baru.celahMin, baru.celahMantap,
           baru.celahRata, baru.berhenti ? " BERHENTI SENDIRI" : "");

    bool ok = true;
    if (baru.celahMantap <= lama.celahMantap + 2.0f) {
        printf("\n  GAGAL: aturan baru tidak menambah celah kaki secara berarti.\n"); ok = false;
    }
    if (baru.celahMantap < 3.0f) {
        printf("\n  GAGAL: sesudah pulih, kaki masih sampai < 3 cm dari dinding.\n"); ok = false;
    }
    if (baru.celahMin < lama.celahMin) {
        printf("\n  GAGAL: aturan baru justru sempat lebih dekat ke dinding.\n"); ok = false;
    }

    // ---------- 4. KEKEBALAN TERHADAP HANTU ----------
    printf("\n=== 4. SENSOR SAMPING MACET DI HANTU 3 CM ===\n");
    printf("  LIDAR_MIN_CM memetakannya ke LIDAR_JAUH = 'dinding hilang', dan itu\n");
    printf("  memang tafsir yang benar: di sensor ini bacaan pendek justru berarti\n");
    printf("  tak ada objek dalam jangkauan. Tapi kalau dinding TIDAK PERNAH\n");
    printf("  muncul lagi, perintah cari NAV_CARI_CMD membuat robot berjalan\n");
    printf("  MELINGKAR selamanya. NAV_CARI_BATAS_MS yang menghentikannya.\n\n");
    hantuCm = 3.0f;
    pakaiAturanBaru();
    uint32_t t0 = __nowMs;
    Hasil dengan = jalankan(22.0f, 0.0f, 40);
    uint32_t lamaCari = __nowMs - t0;
    printf("  berhenti sendiri : %s\n", dengan.berhenti ? "YA" : "TIDAK");
    printf("  sesudah          : %u ms  (NAV_CARI_BATAS_MS = %d)\n", lamaCari, NAV_CARI_BATAS_MS);
    if (!dengan.berhenti) {
        printf("\n  GAGAL: robot berputar mencari dinding tanpa henti.\n");
        ok = false;
    } else if (lamaCari < (uint32_t)NAV_CARI_BATAS_MS || lamaCari > (uint32_t)NAV_CARI_BATAS_MS + 3000) {
        printf("\n  GAGAL: waktu berhenti tidak wajar.\n");
        ok = false;
    }
    hantuCm = 0.0f;

    printf(ok ? "\nSEMUA UJI LULUS\n" : "\nADA UJI YANG GAGAL\n");
    return ok ? 0 : 1;
}
