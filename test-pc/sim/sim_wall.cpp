// Sweep gain wall-following dengan navUpdate() YANG ASLI (termasuk LidarArray:
// median-3 + EMA + jalur jarakHalus/stempelSampel yang baru).
//
// PERINGATAN JUJUR: model plant-nya kasar -- perintah putar -> laju yaw -> laju
// lateral. Itu memang lingkar umpan balik yang dominan, jadi PERBANDINGAN antar
// gain bermakna; angka mutlaknya tetap harus disetel di robot sungguhan.
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

static const float LEBAR_CM   = 60.0f;
static const float PANJANG_CM = 900.0f;
static const float V_PENUH    = 5.8f;    // cm/s pada maju 1.0
static const float W_PENUH    = 45.5f;   // der/s pada putar 1.0
// 90 der = TEGAK LURUS dinding, dan itulah pemasangan sebenarnya di robot ini
// (ch0/ch1 menghadap kiri, ch3/ch4 menghadap kanan). Dua sudut lain tetap
// disapu sebagai pembanding: hasilnya menunjukkan betapa besar bedanya, jadi
// jangan menyalin gain ini ke robot yang sensornya menyerong.
static float SUDUT_SISI = 90.0f;

static float rx, ry, rth, derauCm;
static uint32_t rngS = 12345;
static float rnd() { rngS = rngS * 1103515245u + 12345u; return ((rngS >> 16) & 0x7FFF) / 32767.0f * 2.0f - 1.0f; }

// jarak perpendicular yang setara dengan setpoint pada sinar 60 der
static float targetRx() { return WALL_SETPOINT_CM * cosf(deg2rad(90.0f - SUDUT_SISI)); }

static uint16_t jarakSisi(bool kiri) {
    float a = deg2rad(kiri ? (SUDUT_SISI - rth) : (SUDUT_SISI + rth));
    float k = cosf(deg2rad(90.0f) - a);
    if (k < 0.15f) return 8190;
    float d = (kiri ? rx : (LEBAR_CM - rx)) / k + derauCm * rnd();
    return (uint16_t)std::max(3.0f, d * 10.0f);
}

// Di VL53L1X, "tak ada target" adalah STATUS, bukan angka jarak. Helper ini
// menerjemahkan penanda 8190 yang dipakai simulasi jadi status yang benar.
static void set1X(int ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}
static void suapLidar() {
    float depan = (PANJANG_CM - ry) / std::max(0.3f, cosf(deg2rad(rth)));
    for (int i = 0; i < 6; i++) set1X(i, 8190);
    set1X(LIDAR_FRONT, (uint16_t)std::max(3.0f, depan * 10.0f));
    set1X(LIDAR_KIRI_D, jarakSisi(true));
    set1X(LIDAR_KANAN_D, jarakSisi(false));
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

struct Hasil { float rms = 0, tembusMin = 0, goyang = 0, jenuhPersen = 0; bool berhenti = false; };

static Hasil jalankan(float mulaiX, float derau, int detik) {
    rx = mulaiX; ry = 0; rth = 0; derauCm = derau;
    for (int i = 0; i < 300; i++) { suapLidar(); lidar.update(); __nowMs += 2; }
    suapYaw(); imu.update();
    nav.navMulai(NAV_DINDING_KIRI);

    Hasil h; h.tembusMin = 1e9f;
    float turnPrev = 0, akum = 0; int nAkhir = 0, nJenuh = 0, n = 0;
    const int TICK_MS = 5, total = detik * 1000 / TICK_MS;

    for (int i = 0; i < total; i++) {
        if (nav.navMode() == NAV_DIAM) { h.berhenti = true; break; }
        __nowMs += TICK_MS;
        float dt = TICK_MS / 1000.0f;
        rth += nav.turnKini() * W_PENUH * dt;
        float v = nav.majuKini() * V_PENUH;
        rx -= v * sinf(deg2rad(rth)) * dt;
        ry += v * cosf(deg2rad(rth)) * dt;
        rx = clampf(rx, 0.5f, LEBAR_CM - 0.5f);

        suapLidar(); suapYaw();
        imu.update(); lidar.update(); nav.navUpdate(); robot.update();

        float t = nav.turnKini();
        h.goyang += fabsf(t - turnPrev); turnPrev = t;
        if (fabsf(t) > 0.99f) nJenuh++;
        h.tembusMin = std::min(h.tembusMin, rx);
        if (i > total / 2) { float e = rx - targetRx(); akum += e * e; nAkhir++; }
        n++;
    }
    h.rms = nAkhir ? sqrtf(akum / nAkhir) : -1;
    h.jenuhPersen = n ? 100.0f * nJenuh / n : 0;
    h.goyang = n ? h.goyang / n * 200.0f : 0;      // per detik
    nav.navBerhenti("selesai.");
    return h;
}

static void baris(float kp, float kd) {
    gParam[K_WALL_KP] = kp;
    gParam[K_WALL_KD] = kd;
    Hasil a = jalankan(30.0f, 0.3f, 40);    // dari tengah lorong
    Hasil b = jalankan(15.0f, 1.0f, 40);    // sudah dekat, sensor SANGAT berderau
    printf("  %.3f  %.3f | %5.1f %5.1f | %5.1f %5.1f | %4.0f%% %4.0f%% | %5.1f %5.1f | %s\n",
           kp, kd, a.rms, b.rms, a.tembusMin, b.tembusMin,
           (double)a.jenuhPersen, (double)b.jenuhPersen, a.goyang, b.goyang,
           (a.berhenti || b.berhenti) ? "BERHENTI SENDIRI" : "");
}

int main() {
    Calib::load();
    robot.begin(); lidar.begin(); robot.arm();
    for (int i = 0; i < 5; i++) { suapYaw(); imu.update(); __nowMs += 10; }

    printf("\n=== SWEEP GAIN IKUT-DINDING ===\n");
    printf("lorong %.0f cm, setpoint %.1f cm\n", LEBAR_CM, (double)WALL_SETPOINT_CM);
    printf("NAV_WALL_TURN_MAX %.2f, 40 detik per uji\n\n", NAV_WALL_TURN_MAX);
    printf("   KP     KD  |  RMS sisa  |  paling dekat |  jenuh 1.0  |  goyang/dtk |\n");
    printf("             | tengah dkt | tengah   dkt  | tengah  dkt | tengah  dkt |\n");
    printf("  ------------+------------+---------------+-------------+-------------+\n");

    for (float sudut : {90.0f, 75.0f, 60.0f}) {
        SUDUT_SISI = sudut;
        printf("\n  >>> sensor samping %.0f der dari depan (setara %.1f cm tegak lurus)%s\n",
               sudut, targetRx(), sudut == 90.0f ? "  <== PEMASANGAN ROBOT INI" : "");
        baris(0.030f, 0.010f);
        printf("  ---- kandidat ----\n");
        for (float kp : {0.006f, 0.008f, 0.010f})
            for (float kd : {0.020f, 0.030f, 0.040f})
                baris(kp, kd);
    }
    return 0;
}
