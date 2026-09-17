// Ratakan (HNT_SISI) memakai DUA sensor per sisi: dudukan depan dulu, dudukan
// belakang sebagai cadangan. Yang diuji di sini dua sifat yang berlawanan dan
// dua-duanya harus benar:
//
//   1. Satu kanal bisu TIDAK boleh membatalkan ruas selama pasangannya masih
//      membaca -- itu yang membuang misi di arena.
//   2. Dua-duanya bisu HARUS tetap ditolak. Menggeser tanpa penggaris persis
//      yang menabrakkan robot ke dinding 6 Sep 2026, dan "lebih toleran"
//      tidak boleh diam-diam berarti "berjalan buta".
//
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited \
//       ../test-pc/stub/stubdefs.cpp Hexapod_Unlimited/{Navigation,Hexapod,\
//       HexaGait,HexaServos,HexaArm,Imu,LidarArray,Calib,ArmInverse,\
//       LegInverseKinematics}.cpp cek_penggaris_sisi.cpp -o /tmp/cek && /tmp/cek
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cassert>
#include <cstdio>
#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "LidarArray.h"
#include "Navigation.h"

static Imu imu; static Hexapod robot; static LidarArray lidar;
static Navigation nav(imu, robot, lidar);

// mm > 8000 ditandai SignalFail, persis seperti sensor yang tak melihat apa pun.
static void suap(uint8_t ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}

// Isi ulang histori median semua kanal dengan keadaan yang sedang dipasang.
static void putar() {
    for (int i = 0; i < 400; i++) { lidar.update(); __nowMs += 2; }
}

int main() {
    Calib::applyDefaults();
    robot.begin(); lidar.begin(); robot.arm();

    const uint8_t DPN = LIDAR_KANAN_D, BLK = LIDAR_KANAN_B;

    // Lorong 30 cm di kedua dudukan, depan 100 cm supaya tak ada yang lain
    // yang menghentikan perataan.
    for (uint8_t ch = 0; ch < 6; ch++) suap(ch, 1000);
    suap(DPN, 300); suap(BLK, 300);
    putar();
    assert(nav.ratakanMulai(false, 20));          // sehat: harus diterima
    nav.navBerhenti(nullptr);

    // 1) DUDUKAN DEPAN BISU, belakang masih membaca -> ruas TETAP JALAN.
    suap(DPN, 8190);
    putar();
    assert(lidar.getDistance(DPN) == LIDAR_JAUH);
    assert(lidar.getDistance(BLK) == 30);
    assert(nav.ratakanMulai(false, 20));
    nav.navBerhenti(nullptr);
    printf("  satu dudukan bisu  -> perataan JALAN (penggaris pindah)\n");

    // 2) DUA-DUANYA bisu -> tetap DITOLAK, dan tidak ada perintah gerak.
    suap(BLK, 8190);
    putar();
    assert(!nav.ratakanMulai(false, 20));
    assert(nav.navMode() == NAV_DIAM);
    printf("  dua dudukan bisu   -> perataan DITOLAK, robot diam\n");

    printf("cek_penggaris_sisi: LOLOS\n");
    return 0;
}
