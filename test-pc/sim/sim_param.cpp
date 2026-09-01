// Menguji perintah q/Q/W lewat parser serial YANG ASLI: string diumpankan ke
// Serial seperti pengguna mengetiknya, lalu loop() memprosesnya.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "Calib.h"
void setup(); void loop();

static void ketik(const char* perintah) {
    printf("\n>>> %s\n", perintah);
    for (const char* p = perintah; *p; p++) Serial.rx.push_back((uint8_t)*p);
    Serial.rx.push_back('\n');
    for (int i = 0; i < 3; i++) { __nowMs += 10; loop(); }
}

int main() {
    __serialDiam = true;   // diamkan banjir pesan boot
    setup();
    __serialDiam = false;

    ketik("qwall");                 // awalan ambigu -> harus mencetak kandidat
    ketik("qwall.kp");              // satu parameter
    ketik("Qwall.kp 0.012");        // ubah -- LANGSUNG
    ketik("qwall.kp");              // buktikan berubah
    ketik("Qwall.kp 99");           // di luar rentang -> harus lapor clamp
    ketik("Qgait.step_height 55");  // perlu 'b'
    ketik("Qstab.tau 0.2");         // belum dipakai
    ketik("Qtidak.ada 1");          // nama salah
    ketik("Qpulse.min 600");        // servo masih lemas -> boleh
    ketik("b");                     // servo AKTIF
    __serialDiam = true; for (int i=0;i<50;i++){ __nowMs+=20; loop(); } __serialDiam = false;
    ketik("Qpulse.min 700");        // sekarang armed -> harus DITOLAK
    ketik("W");                     // simpan

    printf("\n>>> [reset papan: muat ulang dari EEPROM]\n");
    bool sah = Calib::load();
    printf("    blob EEPROM 0 dimuat  : %s\n", sah ? "VALID" : "DITOLAK -> default (BUG)");
    // 0.100 karena uji clamp di atas meminta 99 dan dipangkas ke batas atas.
    printf("    wall.kp sesudah reset = %.3f  (harus 0.100)\n", (double)gParam[K_WALL_KP]);
    printf("    gait.step_height      = %.3f  (harus 55.000)\n", (double)gParam[K_GAIT_STEP_HEIGHT]);
    bool ok = sah && fabsf(gParam[K_WALL_KP] - 0.100f) < 1e-4f
                  && fabsf(gParam[K_GAIT_STEP_HEIGHT] - 55.0f) < 1e-4f;
    printf("\n%s\n", ok ? "UJI PERSISTENSI LULUS" : "UJI PERSISTENSI GAGAL");
    return ok ? 0 : 1;
}
