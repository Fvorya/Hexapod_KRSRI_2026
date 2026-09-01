// ODOMETRI GAIT: apakah jarak yang dilaporkan sama dengan jarak yang
// BENAR-BENAR ditempuh badan menurut generator gait yang asli?
//
// Kebenaran acuannya bukan angka hafalan, melainkan diukur ulang dari
// legTargets dengan cara yang sama seperti sim_laju: badan bergerak
// berlawanan dengan pergeseran kaki yang sedang MENAPAK, jadi
// laju = -jumlah(delta y kaki stance) / waktu. Kalau odometer dan gait
// berbeda, salah satunya salah -- dan keduanya membaca stepLength serta
// cycleTime yang sama, jadi selisih di luar toleransi berarti rumus
// odometernya yang keliru.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>

#include "Calib.h"
#include "HexaGait.h"

static HexaGait gait;

static const float STEP_LEN   = 60.0f;    // mm
static const float CYCLE_MS   = 900.0f;
static const float TICK_MS    = 10;

static void siapkan(float cmdMaju, float cmdPutar) {
    gParam[K_GAIT_STEP_LENGTH] = STEP_LEN;
    gParam[K_GAIT_CYCLE_TIME]  = CYCLE_MS;
    gParam[K_GAIT_DUTY]        = 0.5f;
    gait = HexaGait();
    gait.begin();
    gait.setProfile({ GAIT_STEP_HEIGHT, STEP_LEN, CYCLE_MS, STAND_HEIGHT, STAND_RADIUS });
    gait.setMoveVector(0.0f, cmdMaju, cmdPutar);
    // Biarkan slew vektor gerak dan ramp profil selesai (2 detik) supaya
    // yang diukur adalah keadaan tunak, bukan percepatan.
    for (int i = 0; i < 200; i++) { __nowMs += (uint32_t)TICK_MS; gait.update(); }
    gait.jarakNol();
}

// Jarak tempuh badan menurut legTargets -- kebenaran acuan.
// Hanya sah untuk maju MURNI (tanpa putar), karena dengan yaw tiap kaki
// bergeser dengan besaran berbeda.
static float ukurAcuanMm(int tick) {
    const float zHome = -STAND_HEIGHT;
    float z0 = gait.legTargets[0].z, z1 = gait.legTargets[1].z;
    float y0 = gait.legTargets[0].y, y1 = gait.legTargets[1].y;
    float maju = 0.0f;
    for (int i = 0; i < tick; i++) {
        __nowMs += (uint32_t)TICK_MS;
        gait.update();
        float nz0 = gait.legTargets[0].z, nz1 = gait.legTargets[1].z;
        float ny0 = gait.legTargets[0].y, ny1 = gait.legTargets[1].y;
        bool st0 = (fabsf(nz0 - zHome) < 0.001f) && (fabsf(z0 - zHome) < 0.001f);
        bool st1 = (fabsf(nz1 - zHome) < 0.001f) && (fabsf(z1 - zHome) < 0.001f);
        if      (st0) maju += -(ny0 - y0);
        else if (st1) maju += -(ny1 - y1);
        z0 = nz0; z1 = nz1; y0 = ny0; y1 = ny1;
    }
    return maju;
}

static int jalanSaja(int tick) {
    for (int i = 0; i < tick; i++) { __nowMs += (uint32_t)TICK_MS; gait.update(); }
    return tick;
}

int main() {
    Calib::load();

    printf("\n== UJI 1: odometer vs jarak nyata menurut gait (maju 1.0, 10 siklus) ==\n");
    {
        siapkan(1.0f, 0.0f);
        int tick = (int)(10.0f * CYCLE_MS / TICK_MS);
        float acuan = ukurAcuanMm(tick);
        float odo   = gait.jarakMm();
        float galat = (acuan > 0.001f) ? fabsf(odo - acuan) / acuan * 100.0f : 999.0f;
        printf("  acuan gait %.1f mm, odometer %.1f mm, galat %.2f%%\n", acuan, odo, galat);
        if (galat > 2.0f) { printf("  GAGAL: galat di luar 2%%\n"); return 1; }
        printf("  OK\n");
    }

    printf("\n== UJI 2: normalisasi langkah ikut terhitung ==\n");
    // Maju sambil berputar membuat gait memangkas panjang langkah
    // (HexaGait.cpp: magMax > stepLength -> f < 1). Kalau odometer
    // mengabaikan f, kedua angka di bawah akan IDENTIK.
    {
        siapkan(0.8f, 0.0f);
        jalanSaja(500);
        float lurus = gait.jarakMm();

        siapkan(0.8f, 0.5f);
        jalanSaja(500);
        float belok = gait.jarakMm();

        printf("  lurus %.1f mm, sambil putar %.1f mm\n", lurus, belok);
        if (belok >= lurus * 0.95f) {
            printf("  GAGAL: jarak saat berputar tidak dipangkas -- faktor f tidak dipakai\n");
            return 1;
        }
        printf("  OK\n");
    }

    printf("\n== UJI 3: skala berlaku pada PENAMBAHAN, bukan pada pembacaan ==\n");
    {
        siapkan(0.8f, 0.0f);
        gait.setSkalaOdo(1.0f);
        gait.jarakNol();
        jalanSaja(500);
        float a = gait.jarakMm();

        gait.setSkalaOdo(0.5f);
        float sesudahSetel = gait.jarakMm();
        if (fabsf(sesudahSetel - a) > 0.001f) {
            printf("  GAGAL: menyetel skala menulis ulang jarak yang sudah terkumpul "
                   "(%.3f -> %.3f)\n", a, sesudahSetel);
            return 1;
        }

        jalanSaja(500);
        float tambahan = gait.jarakMm() - a;
        printf("  tambahan pertama %.1f mm, tambahan kedua %.1f mm (harus ~separuh)\n",
               a, tambahan);
        if (fabsf(tambahan - a * 0.5f) > a * 0.05f) {
            printf("  GAGAL: skala tidak dipakai pada penambahan\n"); return 1;
        }
        gait.setSkalaOdo(1.0f);
        printf("  OK\n");
    }

    printf("\n== UJI 4: skala di-clamp ke 0,5 .. 1,5 ==\n");
    {
        gait.setSkalaOdo(9.0f);
        if (fabsf(gait.skalaOdo() - 1.5f) > 0.001f) {
            printf("  GAGAL: 9.0 tidak di-clamp ke 1.5 (dapat %.3f)\n", gait.skalaOdo());
            return 1;
        }
        gait.setSkalaOdo(0.0f);
        if (fabsf(gait.skalaOdo() - 0.5f) > 0.001f) {
            printf("  GAGAL: 0.0 tidak di-clamp ke 0.5 (dapat %.3f)\n", gait.skalaOdo());
            return 1;
        }
        gait.setSkalaOdo(1.0f);
        printf("  OK\n");
    }

    printf("\nSEMUA UJI LULUS\n");
    return 0;
}
