// Mengukur laju maju robot dari generator gait YANG ASLI.
// Badan bergerak berlawanan dengan pergeseran kaki yang sedang MENAPAK,
// jadi laju = -jumlah(delta y kaki stance) / waktu.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "HexaGait.h"

static HexaGait gait;

static float ukur(float cmd, float stepLen, float cycle, float duty) {
    gParam[K_GAIT_STEP_LENGTH] = stepLen;
    gParam[K_GAIT_CYCLE_TIME]  = cycle;
    gParam[K_GAIT_DUTY]        = duty;
    gait = HexaGait();
    gait.begin();
    gait.setProfile({ GAIT_STEP_HEIGHT, stepLen, cycle, STAND_HEIGHT, STAND_RADIUS });
    gait.setMoveVector(0, cmd, 0);

    // biarkan slew + ramp profil selesai (2 detik)
    for (int i = 0; i < 200; i++) { __nowMs += 10; gait.update(); }

    float z0 = gait.legTargets[0].z, z1 = gait.legTargets[1].z;
    float y0 = gait.legTargets[0].y, y1 = gait.legTargets[1].y;
    const float zHome = -STAND_HEIGHT;
    float maju = 0.0f;
    uint32_t t0 = __nowMs;
    int siklus = 10;
    int tick = (int)(siklus * cycle / 10.0f);
    for (int i = 0; i < tick; i++) {
        __nowMs += 10; gait.update();
        float nz0 = gait.legTargets[0].z, nz1 = gait.legTargets[1].z;
        float ny0 = gait.legTargets[0].y, ny1 = gait.legTargets[1].y;
        bool st0 = (fabsf(nz0 - zHome) < 0.001f) && (fabsf(z0 - zHome) < 0.001f);
        bool st1 = (fabsf(nz1 - zHome) < 0.001f) && (fabsf(z1 - zHome) < 0.001f);
        if      (st0) maju += -(ny0 - y0);
        else if (st1) maju += -(ny1 - y1);
        z0 = nz0; z1 = nz1; y0 = ny0; y1 = ny1;
    }
    float detik = (__nowMs - t0) / 1000.0f;
    printf("  cmd=%.2f  step=%5.1f mm  cycle=%6.0f ms  duty=%.2f  ->  %6.1f mm/siklus  %6.1f mm/s (%.1f cm/s)\n",
           cmd, stepLen, cycle, duty, maju / siklus, maju / detik, maju / detik / 10.0f);
    return maju / detik;
}

int main() {
    Calib::load();
    printf("\n== Laju maju vs parameter gait ==\n");
    printf("\n-- setelan sekarang --\n");
    ukur(0.8f, 60.0f, 900.0f, 0.5f);
    ukur(1.0f, 60.0f, 900.0f, 0.5f);
    printf("\n-- perbesar langkah --\n");
    ukur(1.0f, 80.0f, 900.0f, 0.5f);
    ukur(1.0f, 100.0f, 900.0f, 0.5f);
    printf("\n-- percepat siklus --\n");
    ukur(1.0f, 60.0f, 700.0f, 0.5f);
    ukur(1.0f, 60.0f, 500.0f, 0.5f);
    printf("\n-- gabungan --\n");
    ukur(1.0f, 90.0f, 600.0f, 0.5f);
    printf("\n-- pengaruh duty --\n");
    ukur(1.0f, 60.0f, 900.0f, 0.6f);
    ukur(1.0f, 60.0f, 900.0f, 0.4f);
    return 0;
}
