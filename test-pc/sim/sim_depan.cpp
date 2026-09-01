// HANTU SENSOR DEPAN: lorong kosong dibaca sebagai halangan mepet.
//
// Gejala yang dilaporkan dari lapangan: saat ikut dinding dan di DEPAN tidak
// ada apa-apa, robot berbelok menjauhi dinding padahal lorongnya terbuka.
//
// Sebabnya sensor robot ini melaporkan "tak ada objek dalam jangkauan" sebagai
// bacaan ~5 cm berstatus RangeValid, BUKAN sebagai SignalFail. Navigasi
// membacanya sebagai "halangan <= FRONT_STOP_CM (20)", lalu menjalankan cabang
// penghindaran: turn = -sisi * NAV_BELOK_CMD. Untuk mode 'F' (dinding kanan,
// sisi = -1) itu berarti +0,60 = berputar KE KIRI -- persis yang terlihat.
//
// wall.hantu dulu hanya melindungi sensor SAMPING. Sekarang penyaringnya ada di
// LidarArray lewat LIDAR_MIN_CM, jadi berlaku untuk semua arah.
//
// Uji 1  gejalanya: hantu depan menetap -> robot HARUS jalan lurus.
// Uji 2  tidak kebablasan: halangan SUNGGUHAN tetap menghentikan/membelokkan.
// Uji 3  gerbang tiga sampel: satu bacaan pendek menyimpang di tengah dinding
//        sungguhan tidak boleh membalik sensor jadi "kosong".
// Uji 4  batas atas baru: 70 cm, sesuai jangkauan akurat yang terukur.
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

static Imu imu; static Hexapod robot; static LidarArray lidar;
static Navigation nav(imu, robot, lidar);

static const float HANTU_CM = 5.0f;    // bacaan "tak ada objek" di robot ini

static void set1X(int ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}

// depanCm < 0 -> pakai hantu. sampingCm selalu dinding nyata di setpoint.
static void suap(float depanCm, float sampingCm) {
    for (int i = 0; i < 6; i++) set1X(i, 8190);
    set1X(LIDAR_FRONT, (uint16_t)lrintf((depanCm < 0 ? HANTU_CM : depanCm) * 10.0f));
    set1X(LIDAR_KANAN_D, (uint16_t)lrintf(sampingCm * 10.0f));
}
static void suapYaw() {
    uint8_t f[11] = { 0x55, 0x53, 0,0, 0,0, 0,0, 0,0,0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}

// Jalankan n detik dengan bacaan tetap; kembalikan rata |turn| dan rata maju.
static void jalan(float depanCm, float sampingCm, int detik,
                  float& turnRata, float& majuRata, bool& berhenti) {
    double tJum = 0, mJum = 0; int n = 0;
    berhenti = false;
    for (int i = 0; i < detik * 200; i++) {
        if (nav.navMode() == NAV_DIAM) { berhenti = true; break; }
        __nowMs += 5;
        suap(depanCm, sampingCm); suapYaw();
        imu.update(); lidar.update(); nav.navUpdate(); robot.update();
        if (i > detik * 100) { tJum += nav.turnKini(); mJum += nav.majuKini(); n++; }
    }
    turnRata = n ? (float)(tJum / n) : 0.0f;
    majuRata = n ? (float)(mJum / n) : 0.0f;
}

static void isiFilter(float depanCm, float sampingCm) {
    for (int i = 0; i < 600; i++) { suap(depanCm, sampingCm); lidar.update(); __nowMs += 2; }
    suapYaw(); imu.update();
}

int main() {
    Calib::load();
    robot.begin(); lidar.begin(); robot.arm();
    const float SP = WALL_SETPOINT_CM;
    bool ok = true;

    printf("\n=== HANTU SENSOR DEPAN ===\n");
    printf("bacaan 'tak ada objek' di robot ini = %.0f cm, status RangeValid\n", HANTU_CM);
    printf("LIDAR_MIN_CM depan = %d cm, samping = %d cm, LIDAR_MAX_CM = %d cm\n",
           LIDAR_MIN_CM[LIDAR_FRONT], LIDAR_MIN_CM[LIDAR_KANAN_D], LIDAR_MAX_CM);
    printf("FRONT_STOP_CM = %d, NAV_BELOK_CMD = %.2f, setpoint = %.1f cm\n",
           FRONT_STOP_CM, NAV_BELOK_CMD, (double)SP);

    // ---------- 1 ----------
    printf("\n-- 1. LORONG DEPAN KOSONG (hantu %.0f cm), ikut dinding KANAN --\n", HANTU_CM);
    isiFilter(-1, SP);
    nav.navMulai(NAV_DINDING_KANAN);
    float t, m; bool stop;
    jalan(-1, SP, 12, t, m, stop);
    printf("   depan dilaporkan : %s\n",
           lidar.getDistance(LIDAR_FRONT) == LIDAR_JAUH ? "jauh (BENAR)" : "ADA HALANGAN (SALAH)");
    printf("   perintah putar   : %+.3f  (harus ~0; +0,60 = berputar ke kiri)\n", t);
    printf("   perintah maju    : %+.3f  (harus penuh %.2f)\n", m, (double)NAV_FWD_SPEED);
    if (lidar.getDistance(LIDAR_FRONT) != LIDAR_JAUH) { printf("   GAGAL: hantu masih dibaca sebagai jarak\n"); ok = false; }
    if (fabsf(t) > 0.10f) { printf("   GAGAL: robot masih membelok di lorong kosong\n"); ok = false; }
    if (m < NAV_FWD_SPEED * 0.95f) { printf("   GAGAL: robot melambat tanpa sebab\n"); ok = false; }
    nav.navBerhenti("selesai.");

    // ---------- 2 ----------
    printf("\n-- 2. HALANGAN SUNGGUHAN 15 cm di depan (harus TETAP membelok) --\n");
    isiFilter(15.0f, SP);
    nav.navMulai(NAV_DINDING_KANAN);
    jalan(15.0f, SP, 12, t, m, stop);
    printf("   depan dilaporkan : %d cm\n", lidar.getDistance(LIDAR_FRONT));
    printf("   perintah putar   : %+.3f  (harus ~%+.2f = menjauhi dinding kanan)\n",
           t, (double)NAV_BELOK_CMD);
    printf("   perintah maju    : %+.3f  (harus 0)\n", m);
    if (lidar.getDistance(LIDAR_FRONT) != 15) { printf("   GAGAL: halangan nyata tidak terbaca 15 cm\n"); ok = false; }
    if (t < NAV_BELOK_CMD * 0.9f) { printf("   GAGAL: tidak menghindar dari halangan nyata\n"); ok = false; }
    if (m > 0.01f) { printf("   GAGAL: masih maju ke arah halangan\n"); ok = false; }
    nav.navBerhenti("selesai.");

    // ---------- 3 ----------
    printf("\n-- 3. SATU BACAAN PENDEK MENYIMPANG saat dinding depan 40 cm --\n");
    printf("   (arah kesalahan paling berbahaya: 'ada dinding' -> 'kosong')\n");
    isiFilter(40.0f, SP);
    printf("   sebelum   : depan = %d cm\n", lidar.getDistance(LIDAR_FRONT));
    // satu sampel pendek, lalu kembali normal
    for (int i = 0; i < 40; i++) { suap(HANTU_CM, SP); lidar.update(); __nowMs += 2; }
    int sesudahSatu = lidar.getDistance(LIDAR_FRONT);
    printf("   1 sampel  : depan = ");
    if (sesudahSatu == LIDAR_JAUH) printf("jauh  <- GAGAL, satu sampel sudah membalik\n");
    else                           printf("%d cm  (bertahan, BENAR)\n", sesudahSatu);
    if (sesudahSatu == LIDAR_JAUH) ok = false;
    // hantu menetap -> baru boleh berubah jadi jauh
    for (int i = 0; i < 600; i++) { suap(-1, SP); lidar.update(); __nowMs += 2; }
    printf("   menetap   : depan = %s\n",
           lidar.getDistance(LIDAR_FRONT) == LIDAR_JAUH ? "jauh (BENAR)" : "MASIH ADA HALANGAN (GAGAL)");
    if (lidar.getDistance(LIDAR_FRONT) != LIDAR_JAUH) ok = false;

    // ---------- 4 ----------
    printf("\n-- 4. BATAS ATAS %d cm (jangkauan akurat terukur ~70 cm) --\n", LIDAR_MAX_CM);
    for (float d : {45.0f, 65.0f, 75.0f, 90.0f}) {
        isiFilter(d, SP);
        int r = lidar.getDistance(LIDAR_FRONT);
        bool harusJauh = (d > LIDAR_MAX_CM);
        bool benar = harusJauh ? (r == LIDAR_JAUH) : (r > 0 && fabsf(r - d) <= 2.0f);
        printf("   dinding %4.0f cm -> %-8s %s\n", d,
               r == LIDAR_JAUH ? "jauh" : (r == LIDAR_MATI ? "MATI" : "terbaca"),
               benar ? "OK" : "GAGAL");
        if (!benar) ok = false;
    }

    printf(ok ? "\nSEMUA UJI LULUS\n" : "\nADA UJI YANG GAGAL\n");
    return ok ? 0 : 1;
}
