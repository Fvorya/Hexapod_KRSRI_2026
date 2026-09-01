// Menguji perintah 'z' lewat parser serial ASLI, lalu MENGUKUR bentuk pose
// badan yang benar-benar keluar sesudah melewati ramp BODY_SLEW_DEG_S.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "Calib.h"
#include "Hexapod.h"
extern Hexapod robot;
void setup(); void loop();

static void ketik(const char* c, bool diam=false) {
    if (!diam) printf("\n>>> %s\n", c);
    for (const char* p = c; *p; p++) Serial.rx.push_back((uint8_t)*p);
    Serial.rx.push_back('\n');
    for (int i = 0; i < 3; i++) { __nowMs += 10; loop(); }
}

// Ukuran BEBAS FASE. Untuk sinus murni berlaku lajuPuncak = amp*omega; kalau
// ramp memotongnya, laju mentok di BODY_SLEW_DEG_S DAN puncaknya ikut turun.
// Jadi dua rasio ini cukup memisahkan sinus utuh dari segitiga terpotong --
// tanpa perlu tahu kapan persis gelombangnya dimulai.
static void ukur(const char* label, float amp, float per, int detik) {
    float puncak = 0, lajuMaks = 0, prev = robot.bodyRollDeg();
    for (int i = 0; i < detik * 100; i++) {
        __nowMs += 10; loop();
        float r = robot.bodyRollDeg();
        float laju = fabsf(r - prev) / 0.01f;
        prev = r;
        // Lewati satu siklus pertama untuk KEDUANYA. Saat berganti perintah,
        // pose mengejar sasaran barunya pada laju penuh ramp -- itu transisi
        // yang wajar, bukan bentuk gelombangnya.
        if (i > per * 100) {
            puncak   = std::max(puncak, fabsf(r));
            lajuMaks = std::max(lajuMaks, laju);
        }
    }
    float lajuIdeal = amp * 2.0f * (float)M_PI / per;
    float rAmp  = puncak / amp;
    float rLaju = lajuMaks / lajuIdeal;
    bool utuh = rAmp > 0.98f && rAmp < 1.02f && rLaju > 0.97f && rLaju < 1.03f;
    printf("  %-26s puncak %5.2f/%4.1f (%.0f%%) | laju %5.1f/%5.1f (%.0f%%) -> %s\n",
           label, puncak, amp, 100*rAmp, lajuMaks, lajuIdeal, 100*rLaju,
           utuh ? "SINUS UTUH" : "TERPOTONG");
}

int main() {
    __serialDiam = true; setup();
    ketik("b", true);
    for (int i = 0; i < 60; i++) { __nowMs += 20; loop(); }
    __serialDiam = false;

    printf("\n=== bentuk gelombang sesudah melewati ramp (BODY_SLEW_DEG_S = %.0f) ===\n",
           (double)BODY_SLEW_DEG_S);
    ketik("z12 2");            ukur("z12 2  (default)",      12.0f, 2.0f, 6);
    ketik("z");                                        // matikan
    ketik("z8 4");             ukur("z8 4   (pelan, anggun)", 8.0f, 4.0f, 9);
    ketik("z");
    printf("\n  Tanpa penjaga, z12 0.5 menuntut %.0f der/detik -- %.0fx di atas batas ramp,\n",
           12.0 * 2 * M_PI / 0.5, 12.0 * 2 * M_PI / 0.5 / (double)BODY_SLEW_DEG_S);
    printf("  jadi yang keluar akan jadi segitiga, bukan gelombang. Penjaga menaikkan periodenya:\n");
    ketik("z12 0.5");          ukur("z12 0.5 -> dinaikkan",  12.0f, 1.257f, 6);
    ketik("z");
    ketik("z20 3 90");         ukur("z20 3 90 (kerucut)",    20.0f, 3.0f, 7);

    printf("\n=== penghentian oleh perintah lain ===\n");
    ketik("0");
    printf("    roll sesudah '0' + settle: ");
    for (int i = 0; i < 80; i++) { __nowMs += 20; loop(); }
    printf("%.2f der (harus ~0)\n", (double)robot.bodyRollDeg());
    return 0;
}
