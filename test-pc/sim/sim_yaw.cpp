// APAKAH YAW IMU MEMBANTU IKUT-DINDING? Perbandingan tiga arsitektur kendali
// pada plant lorong yang SAMA PERSIS, memakai LidarArray asli (median-3 + EMA
// + jalur jarakHalus/stempelSampel) dan Imu asli (frame WIT 0x55 sungguhan).
//
//   A. SEKARANG  -- PD langsung pada jarak terbaca. IMU tidak dipakai sama
//                   sekali di mode 'f'/'F'.
//   B. KOSINUS   -- sama, tapi jarak dikoreksi lebih dulu: d_tegak = d * cos(th).
//                   Sensor samping tegak lurus mengukur d/cos(th) saat robot
//                   menyerong; IMU tahu th, jadi inflasi itu bisa dibuang.
//   C. KASKADE   -- jarak jadi lingkar LUAR yang lambat (menghasilkan sudut
//                   hadap yang diinginkan), heading jadi lingkar DALAM yang
//                   cepat (IMU + giro). Ini yang dipakai kendali kendaraan
//                   pada umumnya.
//   D. GIRO SAJA -- P pada jarak, redaman dari GIRO. Tidak menyentuh yaw sama
//                   sekali: tidak perlu acuan arah lorong, tidak perlu laju
//                   maju, tidak peduli magnetometer. Jauh lebih murah dari C.
//   E. GIRO+D    -- seperti sekarang (P dan D pada jarak) DITAMBAH redaman giro.
//
// Acuan arah lorong TIDAK diambil dari kompas arena (yang perlu kalibrasi
// 'C'+'S'), melainkan DITAKSIR dari LiDAR:
//
//     d_tegak berubah  ->  d/dt(d_tegak) = -v * sin(th)
//     jadi  th_amatan = -asin( (dd/dt) / v )   dan   acuan = yaw - th_amatan
//
// Versi pertama simulasi ini memakai rata-rata bergerak dari yaw saja. Itu
// GAGAL di skenario 3: kalau robot mulai menyerong 20 der, acuannya ikut
// mengunci 20 der yang salah, tidak ada yang mengoreksi, dan kaki menabrak.
// Rata-rata yaw tidak punya acuan luar; penaksir di atas punya -- dinding
// sendiri yang menjadi acuannya.
//
// Yang diuji bukan cuma "mana yang paling rapi di kondisi ideal", tapi juga
// apa yang terjadi kalau kompasnya MELESET -- itu risiko nyata dari memakai
// magnetometer di dekat motor dan rangka besi arena.
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

// --- plant lorong (sama dengan sim_dinding) ---
static const float LEBAR_CM     = 60.0f;
static const float LEBAR_KAKI   = 16.0f;   // ujung kaki tengah dari pusat badan
static const float X_SENSOR     = 5.0f;    // dudukan sensor dari pusat badan
static const float V_PENUH      = 10.7f;   // cm/s pada maju 1.0
static const float W_PENUH      = 45.5f;   // der/detik pada putar 1.0

static float rx, rth;            // jarak pusat badan ke dinding kiri, dan yaw sejati
static float derauCm, biasYaw, derauYaw;
static uint32_t rngS = 97531;
static float rnd() { rngS = rngS * 1103515245u + 12345u; return ((rngS >> 16) & 0x7FFF) / 32767.0f * 2.0f - 1.0f; }

static void set1X(int ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}
static void suapLidar() {
    for (int i = 0; i < 6; i++) set1X(i, 8190);
    set1X(LIDAR_FRONT, 9000);                        // lorong panjang, depan bebas
    // Sensor TEGAK LURUS: lintasan memanjang 1/cos(yaw) saat robot menyerong.
    float k = cosf(deg2rad(rth));
    if (k < 0.15f) { set1X(LIDAR_KIRI_D, 8190); return; }
    float d = (rx - X_SENSOR) / k + derauCm * rnd();
    set1X(LIDAR_KIRI_D, (uint16_t)std::max(3.0f, d * 10.0f));
}
static void suapYaw() {
    auto enc = [](float d) { return (int16_t)lrintf(d / 180.0f * 32768.0f); };
    float y = rth + biasYaw + derauYaw * rnd();
    while (y > 180) y -= 360; while (y < -180) y += 360;
    int16_t v2 = enc(y);
    uint8_t f[11] = { 0x55, 0x53, 0,0, 0,0, (uint8_t)(v2 & 0xFF), (uint8_t)(v2 >> 8), 0,0,0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}
// Frame giro 0x52 (wz di slot ke-3), supaya Imu::gyroZ() terisi seperti aslinya.
static void suapGiro(float wz) {
    auto enc = [](float v) { return (int16_t)lrintf(v / 2000.0f * 32768.0f); };
    int16_t v2 = enc(wz);
    uint8_t f[11] = { 0x55, 0x52, 0,0, 0,0, (uint8_t)(v2 & 0xFF), (uint8_t)(v2 >> 8), 0,0,0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}

// ---- parameter kandidat lingkar luar (hukum C) ----
static float KELUAR_DER_PER_CM = 2.0f;   // simpang jarak -> sudut hadap yang diminta
static float SUDUT_MAKS_DER    = 20.0f;  // batas sudut menyerong
static float ACUAN_TAU         = 4.0f;   // detik, konstanta belajar arah lorong

// wrap180/wrap360 milik Navigation itu privat, jadi disalin lokal di sini.
static float wrap180(float d) { while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f; return d; }
static float wrap360(float d) { while (d >= 360.0f) d -= 360.0f; while (d < 0.0f) d += 360.0f; return d; }

enum Hukum { A_PD, B_KOSINUS, C_KASKADE, D_GIRO, E_GIRO_D };
static const int N_HUKUM = 5;
static float K_GIRO = 0.004f;   // redaman dari giro, satuan putar per (der/detik)

struct Hasil { float rms, celahMin, jenuh, goyang; bool nabrak; };

// Satu percobaan. sisi = +1 (dinding kiri). Mengembalikan ringkasan.
static Hasil jalankan(Hukum hukum, float mulaiRx, float mulaiYaw,
                      float derau, float bias, float derauY, int detik,
                      bool imuMati = false) {
    rx = mulaiRx; rth = mulaiYaw; derauCm = derau; biasYaw = bias; derauYaw = derauY;

    // Isi filter LiDAR & IMU sebelum kendali mulai.
    for (int i = 0; i < 400; i++) { suapLidar(); lidar.update(); __nowMs += 2; }
    for (int i = 0; i < 5; i++) { suapYaw(); suapGiro(0); imu.update(); __nowMs += 10; }

    float acuan = imu.yawDeg();     // tebakan awal; dikoreksi penaksir di bawah
    bool  acuanAda = !imuMati;
    float dPrev = -1.0f, dDot = 0.0f; uint32_t dStempel = 0;
    const float V_NYATA = NAV_FWD_SPEED * V_PENUH;   // cm/s, dari odometri di firmware
    float th = 0.0f;

    float errPrev = 0, errTurunan = 0; uint32_t errStempel = 0; bool errAda = false;
    float turn = 0, turnPrev = 0;

    Hasil h { 0, 1e9f, 0, 0, false };
    double akum = 0; int nAkum = 0, nJenuh = 0, n = 0;
    const int TICK_MS = 5, total = detik * 1000 / TICK_MS;
    const int PULIH = 8000 / TICK_MS;

    for (int i = 0; i < total; i++) {
        __nowMs += TICK_MS;
        float dt = TICK_MS / 1000.0f;

        // --- plant ---
        float wz = turn * W_PENUH;
        rth += wz * dt;
        float v = NAV_FWD_SPEED * V_PENUH;
        rx -= v * sinf(deg2rad(rth)) * dt;
        rx = clampf(rx, LEBAR_KAKI, LEBAR_CM - 0.5f);

        suapLidar(); suapYaw(); suapGiro(wz);
        imu.update(); lidar.update();

        // --- kendali ---
        float jarak = lidar.jarakHalus(LIDAR_KIRI_D);
        if (jarak < 0.0f) jarak = (float)lidar.getDistance(LIDAR_KIRI_D);
        uint32_t stempel = lidar.stempelSampel(LIDAR_KIRI_D);

        // Simpang hadap terhadap arah lorong yang ditaksir.
        th = acuanAda ? wrap180(imu.yawDeg() - acuan) : 0.0f;

        float jarakPakai = jarak;
        if (hukum != A_PD && acuanAda) {
            // Buang inflasi 1/cos: d_tegak = d_terbaca * cos(th).
            float c = cosf(deg2rad(clampf(th, -60.0f, 60.0f)));
            jarakPakai = jarak * c;
        }

        float err = jarakPakai - WALL_SETPOINT_CM;

        if (hukum == C_KASKADE && acuanAda) {
            // LINGKAR LUAR: simpang jarak -> sudut hadap yang diminta.
            float thTarget = clampf(KELUAR_DER_PER_CM * err, -SUDUT_MAKS_DER, SUDUT_MAKS_DER);
            // LINGKAR DALAM: heading (IMU + giro), persis kemudiHeading().
            float errTh = wrap180((acuan + thTarget) - imu.yawDeg());
            turn = HEADING_KP * errTh - HEADING_KD * imu.gyroZ();
        } else {
            if (!errAda) { errPrev = err; errStempel = stempel; errTurunan = 0; errAda = true; }
            else if (stempel != errStempel) {
                float dts = (float)(uint32_t)(stempel - errStempel) / 1000.0f;
                errTurunan = (dts > 0.005f && dts < 0.5f) ? (err - errPrev) / dts : 0.0f;
                errPrev = err; errStempel = stempel;
            }
            turn = WALL_KP * err;
            if (hukum != D_GIRO) turn += WALL_KD * errTurunan;
            // Redaman giro: melawan laju putar badan, bukan laju perubahan
            // jarak. Sinyalnya bersih dan berlaju tinggi, jadi tidak perlu
            // diturunkan dari apa pun. Di firmware suku ini harus ikut
            // dikalikan _pivotSign, sama seperti kemudiHeading().
            if ((hukum == D_GIRO || hukum == E_GIRO_D) && !imuMati)
                turn -= K_GIRO * imu.gyroZ();
        }
        turn = clampf(turn, -NAV_WALL_TURN_MAX, NAV_WALL_TURN_MAX);

        // PENAKSIR ARAH LORONG. Laju perubahan jarak tegak lurus memberi tahu
        // seberapa menyerong robot berjalan, tanpa perlu tahu arah lorong lebih
        // dulu: dd/dt = -v*sin(th). Dihitung pada laju SAMPEL LiDAR, sama
        // seperti suku turunan PD -- pada laju loop pembaginya hampir nol.
        if (acuanAda && stempel != dStempel) {
            if (dPrev >= 0.0f && dStempel != 0) {
                float dts = (float)(uint32_t)(stempel - dStempel) / 1000.0f;
                if (dts > 0.005f && dts < 0.5f) dDot = (jarakPakai - dPrev) / dts;
            }
            dPrev = jarakPakai; dStempel = stempel;

            float sinTh = clampf(-dDot / V_NYATA, -0.9f, 0.9f);
            float thAmatan = rad2deg(asinf(sinTh));
            float acuanAmatan = wrap360(imu.yawDeg() - thAmatan);
            // Tapis lambat: dDot berderau, tapi arah lorong berubah pelan.
            float a = 0.025f / (ACUAN_TAU + 0.025f);
            acuan = wrap360(acuan + a * wrap180(acuanAmatan - acuan));
        }

        // --- ukur ---
        float celah = rx - LEBAR_KAKI;
        if (celah <= 0.05f) h.nabrak = true;
        if (fabsf(turn) > 0.99f * NAV_WALL_TURN_MAX) nJenuh++;
        h.goyang += fabsf(turn - turnPrev); turnPrev = turn;
        if (i > PULIH) {
            h.celahMin = std::min(h.celahMin, celah);
            float e = rx - (X_SENSOR + WALL_SETPOINT_CM);   // simpang posisi SEJATI
            akum += e * e; nAkum++;
        }
        n++;
    }
    h.rms    = nAkum ? sqrtf((float)(akum / nAkum)) : -1;
    h.jenuh  = n ? 100.0f * nJenuh / n : 0;
    h.goyang = n ? h.goyang / n * (1000.0f / TICK_MS) : 0;
    return h;
}

static const char* NAMA[N_HUKUM] = { "A sekarang (PD jarak)", "B + koreksi kosinus  ",
                                     "C kaskade jarak->yaw ", "D P jarak + giro     ",
                                     "E PD jarak + giro     " };

static void blok(const char* judul, float mulaiRx, float mulaiYaw,
                 float derau, float bias, float derauY, int detik) {
    printf("\n  %s\n", judul);
    printf("  hukum                 | RMS sisa | celah kaki min | jenuh | goyang/dtk\n");
    printf("  ----------------------+----------+----------------+-------+-----------\n");
    for (int k = 0; k < N_HUKUM; k++) {
        Hasil h = jalankan((Hukum)k, mulaiRx, mulaiYaw, derau, bias, derauY, detik);
        printf("  %s | %6.2f cm | %9.2f cm  | %4.0f%% | %6.2f %s\n",
               NAMA[k], h.rms, h.celahMin, h.jenuh, h.goyang, h.nabrak ? " KAKI MENYENTUH" : "");
    }
}

int main() {
    Calib::load();
    robot.begin(); lidar.begin(); robot.arm();

    printf("\n=== APAKAH YAW IMU MEMBANTU IKUT-DINDING? ===\n");
    printf("lorong %.0f cm, setpoint %.1f cm, laju maju %.1f cm/s, batas putar %.2f\n",
           LEBAR_CM, (double)WALL_SETPOINT_CM, NAV_FWD_SPEED * V_PENUH, NAV_WALL_TURN_MAX);
    printf("posisi ideal pusat badan = %.1f cm dari dinding\n", X_SENSOR + WALL_SETPOINT_CM);
    printf("lingkar luar C: %.1f der/cm, sudut maks %.0f der, tau acuan %.1f dtk\n",
           KELUAR_DER_PER_CM, SUDUT_MAKS_DER, ACUAN_TAU);

    blok("1. MENJAGA jarak -- mulai pas di setpoint, sejajar, derau +-1 cm",
         X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 0.0f, 0.5f, 40);

    blok("2. MENYUSUL dinding -- mulai dari tengah lorong (30 cm), sejajar",
         30.0f, 0.0f, 1.0f, 0.0f, 0.5f, 40);

    blok("3. MULAI MENYERONG 20 der ke arah dinding",
         X_SENSOR + WALL_SETPOINT_CM, 20.0f, 1.0f, 0.0f, 0.5f, 40);

    blok("4. SENSOR BERDERAU BERAT (+-3 cm)",
         X_SENSOR + WALL_SETPOINT_CM, 0.0f, 3.0f, 0.0f, 0.5f, 40);

    blok("5. KOMPAS MELESET TETAP 15 der (gangguan magnet arena)",
         X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 15.0f, 0.5f, 40);

    blok("6. YAW BERDERAU BERAT (+-5 der)",
         X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 0.0f, 5.0f, 40);

    // Hukum D (giro saja) itu yang PALING murah kalau berhasil: tidak menyentuh
    // yaw, tidak perlu acuan arah lorong, tidak perlu laju maju. Ia menang telak
    // saat sudah dekat setpoint -- tapi gagal MENYUSUL. Sebelum menyimpulkan,
    // redaman gironya disapu: mungkin cuma kekencangan.
    printf("\n  8. BISAKAH HUKUM D DISELAMATKAN DENGAN REDAMAN GIRO LEBIH LEMBUT?\n");
    printf("  k_giro | menjaga RMS | menyusul RMS | menyerong 20: celah min\n");
    printf("  -------+-------------+--------------+------------------------\n");
    for (float kg : {0.000f, 0.001f, 0.002f, 0.004f, 0.008f}) {
        K_GIRO = kg;
        Hasil j = jalankan(D_GIRO, X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 0.0f, 0.5f, 40);
        Hasil m = jalankan(D_GIRO, 30.0f,                       0.0f, 1.0f, 0.0f, 0.5f, 40);
        Hasil y = jalankan(D_GIRO, X_SENSOR + WALL_SETPOINT_CM, 20.0f, 1.0f, 0.0f, 0.5f, 40);
        printf("  %.3f |   %6.2f cm  |   %6.2f cm  |   %6.2f cm %s\n",
               kg, j.rms, m.rms, y.celahMin, y.nabrak ? " KAKI MENYENTUH" : "");
    }
    K_GIRO = 0.004f;
    printf("\n  P saja tidak punya ANTISIPASI: ia baru bereaksi sesudah jaraknya\n");
    printf("  meleset, sedangkan jarak adalah dua integrasi di belakang perintah\n");
    printf("  putar (turn -> laju yaw -> sudut -> laju jarak -> jarak). Giro\n");
    printf("  meredam sudut, bukan jarak -- itu integrasi yang salah.\n");

    // Skenario 6 adalah satu-satunya tempat C kalah: derau yaw masuk langsung
    // ke lingkar dalam lewat HEADING_KP. Berapa derau yang masih tertahan?
    printf("\n  7. BATAS DERAU YAW -- sampai berapa kaskade masih lebih halus?\n");
    printf("  derau yaw | goyang A | goyang C | RMS C   | keterangan\n");
    printf("  ----------+----------+----------+---------+------------------------\n");
    for (float dy : {0.0f, 0.5f, 1.0f, 2.0f, 3.0f, 5.0f}) {
        Hasil a = jalankan(A_PD,      X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 0.0f, dy, 40);
        Hasil c = jalankan(C_KASKADE, X_SENSOR + WALL_SETPOINT_CM, 0.0f, 1.0f, 0.0f, dy, 40);
        const char* ket = (c.goyang > a.goyang)      ? "C lebih kasar dari A" :
                          (c.goyang > GAIT_SLEW_RATE) ? "di atas GAIT_SLEW_RATE -- terbuang" :
                                                        "gait masih sanggup ikut";
        printf("  +-%3.1f der | %8.2f | %8.2f | %5.2f cm | %s\n",
               dy, a.goyang, c.goyang, c.rms, ket);
    }
    printf("\n  GAIT_SLEW_RATE = %.1f satuan/detik: perintah yang bergoyang lebih\n",
           (double)GAIT_SLEW_RATE);
    printf("  cepat dari itu tidak sampai ke kaki, cuma memanaskan lingkar kendali.\n");

    printf("\n  Catatan tentang skenario 5: acuan arah lorong DITAKSIR dari LiDAR\n");
    printf("  lalu dinyatakan relatif terhadap yaw, jadi meleset TETAP ikut\n");
    printf("  terserap acuan dan hampir tak berpengaruh. Yang berbahaya bukan\n");
    printf("  meleset tetap, melainkan meleset yang BERUBAH lebih cepat dari\n");
    printf("  tau acuan %.1f detik.\n", ACUAN_TAU);
    return 0;
}
