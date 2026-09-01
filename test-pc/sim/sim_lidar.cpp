// VL53L1X tidak punya sentinel jarak seperti 8190 mm milik VL53L0X: range_mm
// selalu berisi angka, sah atau tidak. Uji ini memastikan ketiga keadaan
// dipisahkan lewat range_status -- termasuk bahwa pengukuran BURUK jatuh ke
// MATI (fail-safe), bukan ke JAUH (yang berarti "tidak ada halangan").
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "LidarArray.h"
static LidarArray lidar;

static const char* arti(int d) {
    if (d == LIDAR_MATI) return "LIDAR_MATI  <-- tak dipercaya, navigasi berhenti";
    if (d == LIDAR_JAUH) return "LIDAR_JAUH  <-- sehat, tak ada objek";
    return "jarak cm";
}
static void putar(int ms) { for (int i = 0; i < ms; i += 2) { __nowMs += 2; lidar.update(); } }

static void kasus(const char* label, uint16_t mm, uint8_t st) {
    for (int i = 0; i < 8; i++) { __simMm[i] = 600; __simStatus[i] = VL53L1X::RangeValid; }
    putar(400);                                   // mulai dari keadaan sehat
    for (int i = 0; i < 8; i++) { __simMm[i] = mm; __simStatus[i] = st; }
    putar(600);                                   // lewati LIDAR_TIMEOUT_MS
    int d = lidar.getDistance(LIDAR_FRONT);
    printf("  %-46s -> %6d   %s\n", label, d, arti(d));
}

int main() {
    Calib::load();
    for (int i = 0; i < 8; i++) __simStatus[i] = VL53L1X::RangeValid;
    lidar.begin();

    printf("\n=== Tiga keadaan dipisahkan lewat range_status ===\n\n");
    kasus("objek 60 cm (RangeValid)",                 600,  VL53L1X::RangeValid);
    kasus("objek 150 cm > LIDAR_MAX_CM (RangeValid)", 1500, VL53L1X::RangeValid);
    kasus("objek sangat dekat (MinRangeClipped)",       40, VL53L1X::RangeValidMinRangeClipped);
    kasus("tak ada target (SignalFail)",              2200, VL53L1X::SignalFail);
    kasus("tak ada target (OutOfBoundsFail)",         3500, VL53L1X::OutOfBoundsFail);
    kasus("pantulan berputar (WrapTargetFail)",        900, VL53L1X::WrapTargetFail);
    kasus("pengukuran buruk (SigmaFail)",              700, VL53L1X::SigmaFail);
    kasus("pengukuran buruk (HardwareFail)",           700, VL53L1X::HardwareFail);
    kasus("terlalu dekat utk diukur (MinRangeFail)",    20, VL53L1X::MinRangeFail);

    printf("\n  Yang PALING penting dua baris SigmaFail/HardwareFail: angka 700 mm\n");
    printf("  di range_mm itu sampah. Kode gaya VL53L0X akan menerimanya sebagai\n");
    printf("  '70 cm' -- robot mengira ada dinding di tempat yang kosong.\n");

    printf("\n  jumlahHidup() saat semua sensor tak melihat apa pun : ");
    for (int i = 0; i < 8; i++) __simStatus[i] = VL53L1X::SignalFail;
    putar(600);
    printf("%d dari %d (harus %d)\n", lidar.jumlahHidup(), NUM_LIDAR, NUM_LIDAR);

    printf("  jumlahHidup() saat semua sensor menghasilkan sampah  : ");
    for (int i = 0; i < 8; i++) __simStatus[i] = VL53L1X::SigmaFail;
    putar(600);
    printf("%d dari %d (harus 0 -- fail-safe)\n", lidar.jumlahHidup(), NUM_LIDAR);
    return 0;
}
