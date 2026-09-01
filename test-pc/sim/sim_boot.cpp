// DEMO OTOMATIS SAAT MENYALA: urutan, waktu, dan jalur pembatalannya.
//
// Dijalankan lewat loop() YANG ASLI, jadi yang diuji benar-benar jalur yang
// dipakai robot -- termasuk handleCmd(), goyangUpdate(), ramp pose badan, dan
// commit servo. Tidak ada satu pun logika demo yang disalin ke sini.
//
// Yang dijaga:
//   1. Robot TETAP LEMAS selama DEMO_BOOT_TUNDA. Ini perlindungan utamanya:
//      kalau ia menyala lalu langsung berdiri, tangan orang masih di sana.
//   2. Sesudah tunda -> berdiri (servo hidup).
//   3. Selama goyang -> badan benar-benar berayun, amplitudo sesuai perintah.
//   4. Sesudah DEMO_BOOT_LAMA -> pose badan kembali NOL, servo tetap hidup.
//   5. Sesudah selesai, demo tidak menghidupkan dirinya lagi.
//   6. Satu ketikan saat hitung mundur MEMBATALKAN -- servo tidak pernah hidup.
//
// Uji 6 butuh proses yang bersih (state machine demo cuma jalan sekali per
// boot), jadi program ini menjalankan dirinya sendiri sekali lagi dengan
// argumen "batal".
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "Calib.h"
#include "Hexapod.h"

extern Hexapod robot;
void setup(); void loop();

static void ketik(const char* c) {
    for (const char* p = c; *p; p++) Serial.rx.push_back((uint8_t)*p);
    Serial.rx.push_back('\n');
}

// Jalankan loop() sampai __nowMs mencapai sampaiMs. Sepanjang jalan, catat
// puncak |roll| supaya bentuk goyangnya bisa diperiksa.
static float jalanSampai(uint32_t sampaiMs) {
    float puncak = 0.0f;
    while (__nowMs < sampaiMs) {
        __nowMs += 10;
        loop();
        puncak = std::max(puncak, fabsf(robot.bodyRollDeg()));
    }
    return puncak;
}

static int ujiNormal() {
    bool ok = true;
    printf("\n=== DEMO MENYALA: URUTAN PENUH ===\n");
    printf("tunda %d ms -> 'b' -> %d ms -> '%s' -> %d ms -> '0'\n\n",
           DEMO_BOOT_TUNDA, DEMO_BOOT_BERDIRI, DEMO_BOOT_PERINTAH, DEMO_BOOT_LAMA);

    setup();

    // --- 1. masih lemas selama tunda ---
    jalanSampai(DEMO_BOOT_TUNDA - 200);
    printf("  t=%5u ms  servo %s\n", __nowMs, robot.isArmed() ? "HIDUP" : "lemas");
    if (robot.isArmed()) {
        printf("  GAGAL: servo hidup SEBELUM tunda habis -- perlindungan boot lemas jebol\n");
        ok = false;
    }

    // --- 2. sesudah tunda: berdiri ---
    jalanSampai(DEMO_BOOT_TUNDA + 300);
    printf("  t=%5u ms  servo %s\n", __nowMs, robot.isArmed() ? "HIDUP" : "lemas");
    if (!robot.isArmed()) { printf("  GAGAL: tidak pernah berdiri\n"); ok = false; }

    // --- 3. goyang benar-benar berayun ---
    uint32_t mulaiGoyang = DEMO_BOOT_TUNDA + DEMO_BOOT_BERDIRI;
    jalanSampai(mulaiGoyang + 3000);          // lewati transisi ramp
    float puncak = jalanSampai(mulaiGoyang + 9000);
    printf("  t=%5u ms  puncak |roll| = %.2f der\n", __nowMs, puncak);
    // 'z10 1' -> amplitudo 10 der. Periodenya dinaikkan sendiri oleh parser
    // supaya ramp tidak memotong sinusnya; amplitudonya tidak berubah.
    if (puncak < 9.0f || puncak > 11.0f) {
        printf("  GAGAL: amplitudo goyang tidak sesuai perintah (harusnya ~10 der)\n");
        ok = false;
    }

    // --- 4. sesudah DEMO_BOOT_LAMA: pose kembali nol, servo tetap hidup ---
    uint32_t selesai = mulaiGoyang + DEMO_BOOT_LAMA;
    jalanSampai(selesai + 2500);              // beri waktu ramp pulang ke nol
    printf("  t=%5u ms  roll = %+.2f der, servo %s\n",
           __nowMs, robot.bodyRollDeg(), robot.isArmed() ? "HIDUP" : "lemas");
    if (fabsf(robot.bodyRollDeg()) > 0.5f) {
        printf("  GAGAL: pose badan tidak kembali ke nol\n"); ok = false;
    }
    if (!robot.isArmed()) {
        printf("  GAGAL: robot tidak lagi berdiri di akhir demo\n"); ok = false;
    }

    // --- 5. tidak menghidupkan dirinya lagi ---
    float puncakSesudah = jalanSampai(__nowMs + 15000);
    printf("  t=%5u ms  puncak |roll| 15 dtk berikutnya = %.2f der\n", __nowMs, puncakSesudah);
    if (puncakSesudah > 0.5f) {
        printf("  GAGAL: demo jalan lagi sesudah selesai\n"); ok = false;
    }

    printf(ok ? "\n  urutan penuh LULUS\n" : "\n  urutan penuh GAGAL\n");
    return ok ? 0 : 1;
}

static int ujiBatal() {
    bool ok = true;
    printf("\n=== DEMO MENYALA: DIBATALKAN SAAT HITUNG MUNDUR ===\n\n");

    setup();
    jalanSampai(1000);
    ketik("d");                                // satu perintah biasa
    jalanSampai(DEMO_BOOT_TUNDA + DEMO_BOOT_BERDIRI + 4000);

    printf("\n  t=%5u ms  servo %s, roll %+.2f der\n",
           __nowMs, robot.isArmed() ? "HIDUP" : "lemas", robot.bodyRollDeg());
    if (robot.isArmed()) {
        printf("  GAGAL: demo tetap menghidupkan servo walau sudah dibatalkan\n");
        ok = false;
    }
    if (fabsf(robot.bodyRollDeg()) > 0.01f) {
        printf("  GAGAL: pose badan ikut bergerak walau demo dibatalkan\n");
        ok = false;
    }
    printf(ok ? "\n  pembatalan LULUS\n" : "\n  pembatalan GAGAL\n");
    return ok ? 0 : 1;
}

int main(int argc, char** argv) {
#if !DEMO_BOOT
    (void)argc; (void)argv;
    printf("\nDEMO_BOOT = 0 -- demo menyala dimatikan di config.h, tidak ada yang diuji.\n");
    return 0;
#else
    if (argc > 1 && strcmp(argv[1], "batal") == 0) return ujiBatal();

    int rc = ujiNormal();

    // Uji pembatalan butuh boot yang bersih: state machine demo hanya berjalan
    // sekali per proses, dan tidak ada jalan meresetnya dari luar -- memang
    // begitu seharusnya di firmware.
    fflush(stdout);          // supaya keluaran anak tidak menyela induknya
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "\"%s\" batal", argv[0]);
    if (system(cmd) != 0) rc = 1;

    printf(rc == 0 ? "\nSEMUA UJI LULUS\n" : "\nADA UJI YANG GAGAL\n");
    return rc;
#endif
}
