// Simulasi state machine pivot di PC. Memakai Navigation/Imu/Hexapod YANG ASLI;
// yang dipalsukan hanya jam, bus I2C, EEPROM, dan aliran byte IMU. Yaw disuapkan
// lewat frame WIT 0x53 sungguhan supaya Imu::update() memarsingnya seperti biasa.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>

#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"

static Imu        imu;
static Hexapod    robot;
static LidarArray lidar;
static Navigation nav(imu, robot, lidar);

static float simYaw = 0.0f;    // heading "fisik" robot dalam simulasi
static float simGz  = 0.0f;    // der/detik

// Susun satu frame WIT 0x53 (sudut) dan masukkan ke antrean Serial2.
static void suapYaw(float yaw) {
    auto enc = [](float deg) { return (int16_t)lrintf(deg / 180.0f * 32768.0f); };
    float y = yaw; while (y > 180.0f) y -= 360.0f; while (y < -180.0f) y += 360.0f;
    int16_t v0 = 0, v1 = 0, v2 = enc(y);
    uint8_t f[11] = { 0x55, 0x53,
                      (uint8_t)(v0 & 0xFF), (uint8_t)(v0 >> 8),
                      (uint8_t)(v1 & 0xFF), (uint8_t)(v1 >> 8),
                      (uint8_t)(v2 & 0xFF), (uint8_t)(v2 >> 8),
                      0, 0, 0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}

// Model gerak: perintah putar -> laju yaw. TANDA SENGAJA POSITIF, sama dengan
// _pivotSign default (+1) saat pivot belum dikalibrasi.
static const float DEG_PER_S_PENUH = 45.0f;

// Satu tick loop utama firmware (10 ms).
static void tick() {
    __nowMs += 10;

    // Robot berputar mengikuti perintah putar yang BENAR-BENAR dikeluarkan
    // Navigation pada tick sebelumnya.
    simGz  = nav.turnKini() * DEG_PER_S_PENUH;
    simYaw += simGz * 0.01f;
    while (simYaw >= 360.0f) simYaw -= 360.0f;
    while (simYaw < 0.0f)    simYaw += 360.0f;

    suapYaw(simYaw);
    imu.update();
    nav.navUpdate();
    robot.update();
}

static void lapor(const char* label) {
    printf("  %-22s t=%6u ms  yaw=%7.2f  turn=%+5.2f  mode=%s\n",
           label, __nowMs, simYaw, nav.turnKini(),
           nav.pivotSedangJalan() ? "PIVOT" : "DIAM");
}

int main() {
    Calib::load();
    robot.begin();
    robot.arm();

    // Beri IMU beberapa frame supaya hasData() benar sebelum pivot dimulai.
    for (int i = 0; i < 5; i++) { suapYaw(simYaw); imu.update(); __nowMs += 10; }

    printf("\n== UJI 1: pivot 0 -> 90 derajat ==\n");
    uint32_t t0 = __nowMs;
    nav.pivotKe(90.0f);
    printf("  pivotKe() kembali setelah %u ms (harus 0 = non-blokir)\n", __nowMs - t0);
    if (__nowMs - t0 != 0) { printf("  GAGAL: pivotKe() masih memblokir!\n"); return 1; }

    int n = 0;
    while (nav.pivotSedangJalan() && n < 6000) { tick(); n++; }
    lapor("selesai");
    if (nav.pivotSedangJalan()) { printf("  GAGAL: tidak pernah selesai\n"); return 1; }
    float sisa = fabsf(90.0f - simYaw);
    printf("  simpang akhir %.2f der (toleransi %.1f)\n", sisa, (double)HEADING_TOLERANCE_DEG);
    if (sisa > HEADING_TOLERANCE_DEG) { printf("  GAGAL: di luar toleransi\n"); return 1; }

    printf("\n== UJI 2: dibatalkan di tengah jalan (seperti menekan 'x') ==\n");
    simYaw = 0.0f; simGz = 0.0f;
    for (int i = 0; i < 5; i++) { suapYaw(simYaw); imu.update(); __nowMs += 10; }
    nav.pivotKe(180.0f);
    for (int i = 0; i < 30; i++) tick();          // 300 ms, jelas belum sampai
    lapor("sebelum dibatalkan");
    if (!nav.pivotSedangJalan()) { printf("  GAGAL: pivot sudah berhenti sendiri\n"); return 1; }
    nav.navBerhenti("dibatalkan pengguna.");
    lapor("sesudah navBerhenti");
    if (nav.pivotSedangJalan()) { printf("  GAGAL: pivot tidak mau berhenti\n"); return 1; }
    for (int i = 0; i < 10; i++) tick();
    if (fabsf(nav.turnKini()) > 1e-6f) {
        printf("  GAGAL: masih mengirim perintah putar %+.3f setelah berhenti\n", nav.turnKini());
        return 1;
    }
    printf("  perintah putar kembali ke 0 dan tidak dihidupkan lagi. OK\n");

    printf("\n== UJI 3: target mustahil -> timeout, bukan berputar selamanya ==\n");
    // Robot "macet": model gerak dimatikan supaya yaw tidak pernah berubah.
    simYaw = 0.0f;
    for (int i = 0; i < 5; i++) { suapYaw(simYaw); imu.update(); __nowMs += 10; }
    nav.pivotKe(180.0f);
    uint32_t mulai = __nowMs;
    n = 0;
    while (nav.pivotSedangJalan() && n < 6000) {
        __nowMs += 10;
        suapYaw(simYaw);            // yaw membeku: robot tidak berputar
        imu.update(); nav.navUpdate(); robot.update();
        n++;
    }
    uint32_t lama = __nowMs - mulai;
    printf("  berhenti sendiri setelah %u ms (PIVOT_BATAS_MS = %d)\n", lama, PIVOT_BATAS_MS);
    if (nav.pivotSedangJalan()) { printf("  GAGAL: tidak pernah timeout\n"); return 1; }
    if (lama < PIVOT_BATAS_MS || lama > PIVOT_BATAS_MS + 100) {
        printf("  GAGAL: waktu timeout tidak wajar\n"); return 1;
    }

    printf("\nSEMUA UJI LULUS\n");
    return 0;
}
