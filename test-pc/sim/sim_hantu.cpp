// Gejala lapangan: tak ada objek di depan, tapi 'l' sering menunjukkan 5-10 cm.
// Sensor mayoritas melapor "tak ada target", sesekali menyelipkan SATU bacaan
// pendek (crosstalk kaca penutup / pantulan beraliasi / badan robot terserempet).
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "LidarArray.h"
static LidarArray lidar;

// 1 dari tiap 'tiapKe' sampel adalah hantu pendek; sisanya "tak ada target".
static int jalankan(int tiapKe, uint16_t hantuMm, int detik, const char* label) {
    int n = 0, kaliJadiJarak = 0, terpendek = 9999;
    for (int i = 0; i < detik * 1000 / 2; i++) {
        __nowMs += 2;
        if ((i % (LIDAR_PERIOD_MS / 2)) == 0) {   // adegan berganti tiap sampel
            n++;
            bool hantu = (n % tiapKe) == 0;
            for (int c = 0; c < 8; c++) {
                __simMm[c]     = hantu ? hantuMm : 3000;
                __simStatus[c] = hantu ? VL53L1X::RangeValid : VL53L1X::SignalFail;
            }
        }
        lidar.update();
        int d = lidar.getDistance(LIDAR_FRONT);
        if (d != LIDAR_JAUH && d != LIDAR_MATI) {
            kaliJadiJarak++;
            if (d < terpendek) terpendek = d;
        }
    }
    printf("  %-42s -> %s", label,
           kaliJadiJarak ? "MELAPOR JARAK " : "tetap 'jauh' (benar)");
    if (kaliJadiJarak) printf("%d cm, %.0f%% waktu", terpendek,
                              100.0 * kaliJadiJarak / (detik * 500));
    printf("\n");
    return kaliJadiJarak;
}

int main() {
    Calib::load();
    for (int i = 0; i < 8; i++) __simStatus[i] = VL53L1X::RangeValid;
    lidar.begin();

    printf("\n=== Hantu pendek di tengah keadaan 'tak ada target' ===\n\n");
    int a = jalankan(4, 80,  6, "1 dari 4 sampel hantu 8 cm");
    int b = jalankan(3, 60,  6, "1 dari 3 sampel hantu 6 cm");
    int c = jalankan(8, 100, 6, "1 dari 8 sampel hantu 10 cm");

    printf("\n=== Pembanding: objek NYATA yang menetap ===\n\n");
    for (int i = 0; i < 8; i++) { __simMm[i] = 400; __simStatus[i] = VL53L1X::RangeValid; }
    for (int i = 0; i < 400; i++) { __nowMs += 2; lidar.update(); }
    printf("  objek sungguhan 40 cm, tiga sampel berturut-turut  -> %d cm\n",
           lidar.getDistance(LIDAR_FRONT));

    printf("\n  %s\n", (a || b || c)
        ? "GAGAL: hantu masih menembus ke pembacaan."
        : "LULUS: hantu tunggal disaring, objek nyata tetap terbaca.");
    return (a || b || c) ? 1 : 0;
}
