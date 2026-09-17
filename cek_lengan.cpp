// Pagar untuk baseline sendi lengan (Hexapod::moveArmTarget).
//
//   export PATH="/c/msys64/ucrt64/bin:$PATH"
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited -o cek_lengan.exe \
//       cek_lengan.cpp Hexapod_Unlimited/ArmInverse.cpp Hexapod_Unlimited/Calib.cpp \
//       ../test-pc/stub/stubdefs.cpp && ./cek_lengan.exe
//
// JEBAKAN: Calib::applyDefaults() WAJIB dipanggil. Tanpanya gParam[] nol
// semua dan angka apa pun yang keluar tidak berarti apa-apa.
//
// Yang dijaga: sudut siku hasil IK adalah sudut DALAM dari acos, jadi selalu
// 0..180 der dan TIDAK PERNAH negatif. Baseline 90 (sama seperti bahu)
// menggesernya ke 90..270; clampf() di HexaArm::angleToPulse() memangkas
// segala yang di atas 180 tanpa memberi tahu siapa pun. Baseline 0 menaruhnya
// pas di dalam rentang servo.
#include <Arduino.h>
#include "Calib.h"
#include "ArmInverse.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

int main() {
    Calib::applyDefaults();

    const float rMin = fabsf(UPPERARM_LENGTH - FOREARM_LENGTH);
    const float rMax = UPPERARM_LENGTH + FOREARM_LENGTH;

    int kena90 = 0, kena0 = 0, n = 0;
    float bahuMin = 1e9f, bahuMaks = -1e9f;

    // Sapu seluruh cincin yang bisa dijangkau, tiap 0,2 mm x 1 der.
    for (int ri = 0; ri <= 200; ri++) {
        for (int ai = -90; ai <= 90; ai++) {
            float r  = rMin + (rMax - rMin) * ri / 200.0f;
            float th = ai * (float)M_PI / 180.0f;
            float bahu = 0.0f, siku = 0.0f;
            if (!ArmInverse::solve(r * cosf(th), r * sinf(th), bahu, siku)) continue;
            n++;

            assert(siku >= 0.0f && siku <= 180.0f);   // sudut dalam, menurut acos

            if (90.0f + siku > 180.0f) kena90++;      // baseline lama
            if (0.0f  + siku > 180.0f) kena0++;       // baseline sekarang

            float sb = 90.0f + bahu;
            if (sb < bahuMin)  bahuMin  = sb;
            if (sb > bahuMaks) bahuMaks = sb;
        }
    }

    printf("titik terjangkau      : %d  (r %.0f..%.0f mm dari bahu)\n", n, (double)rMin, (double)rMax);
    printf("siku baseline 90 (lama): %d ter-clamp\n", kena90);
    printf("siku baseline  0 (kini): %d ter-clamp\n", kena0);
    printf("bahu baseline 90       : %.1f .. %.1f der di seluruh cincin\n",
           (double)bahuMin, (double)bahuMaks);

    assert(n > 1000);            // sapuannya benar-benar jalan
    assert(kena90 > 0);          // baseline lama memang merusak -- ini sebabnya diubah
    assert(kena0 == 0);          // baseline sekarang tidak pernah ter-clamp

    // Bahu TIDAK dijaga: 0..180 der servo memang tak bisa menutupi cincin
    // penuh, dan itu batas fisik, bukan salah baseline. Angkanya dicetak
    // supaya ketahuan sektor mana yang hilang saat horn bahu dipasang.
    fflush(stdout);
    printf("\ncek_lengan: LOLOS\n");
    return 0;
}
