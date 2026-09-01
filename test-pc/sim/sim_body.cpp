// Apakah body kinematics punya slew/ramp? Diuji dengan memberi perintah pose
// LANGKAH (seperti mengetik 'r20 0 0') lalu melihat berapa banyak sudut sendi
// berubah pada update() PERTAMA sesudahnya.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "Hexapod.h"

static Hexapod robot;

static const float US_PER_DEG = (2500.0f - 500.0f) / 180.0f;   // rentang pulse baku

struct Sudut { float c[6], f[6], t[6]; };

static Sudut baca() {
    Sudut s{};
    for (int i = 0; i < 6; i++) robot.legAngles(i, s.c[i], s.f[i], s.t[i]);
    return s;
}

static float bedaMaks(const Sudut& a, const Sudut& b, int& legTerburuk, const char*& sendi) {
    float m = 0; legTerburuk = -1; sendi = "?";
    for (int i = 0; i < 6; i++) {
        float d;
        d = fabsf(a.c[i] - b.c[i]); if (d > m) { m = d; legTerburuk = i; sendi = "coxa";  }
        d = fabsf(a.f[i] - b.f[i]); if (d > m) { m = d; legTerburuk = i; sendi = "femur"; }
        d = fabsf(a.t[i] - b.t[i]); if (d > m) { m = d; legTerburuk = i; sendi = "tibia"; }
    }
    return m;
}

static void uji(const char* label, void (*perintah)()) {
    robot.setBodyRotation(0, 0, 0);
    robot.setBodyTranslation(0, 0, 0);
    for (int i = 0; i < 50; i++) { __nowMs += 20; robot.update(); }   // settle
    Sudut sebelum = baca();

    perintah();
    __nowMs += 20;
    robot.update();                       // SATU siklus commit sesudah perintah
    Sudut sesudah = baca();

    int leg; const char* sendi;
    float d = bedaMaks(sebelum, sesudah, leg, sendi);
    printf("  %-26s lonjakan 1 commit: %6.2f der (%6.1f us)  <- kaki %d %s\n",
           label, d, d * US_PER_DEG, leg, sendi);

    // Berapa lama sampai benar-benar diam? Kalau ada ramp, butuh beberapa siklus.
    int siklus = 0; Sudut prev = sesudah;
    for (int i = 0; i < 200; i++) {
        __nowMs += 20; robot.update();
        Sudut kini = baca();
        int l; const char* s2;
        if (bedaMaks(prev, kini, l, s2) < 0.01f) break;
        prev = kini; siklus++;
    }
    printf("  %-26s siklus sampai diam : %d  (%s)\n", "", siklus,
           siklus == 0 ? "LANGSUNG -- tidak ada ramp" : "ada ramp");
}

int main() {
    Calib::load();
    robot.begin();
    robot.arm();

    printf("\n=== BODY KINEMATICS: apakah ada slew? ===\n");
    printf("(commit servo tiap %d ms; %.2f us per derajat)\n\n", SERVO_COMMIT_MS, US_PER_DEG);

    uji("r20 0 0  (roll 20 der)",   []{ robot.setBodyRotation(20, 0, 0); });
    uji("r0 20 0  (pitch 20 der)",  []{ robot.setBodyRotation(0, 20, 0); });
    uji("t0 0 -40 (merunduk 40mm)", []{ robot.setBodyTranslation(0, 0, -40); });

    printf("\n=== PEMBANDING: tinggi badan lewat profil gait ===\n\n");
    robot.setBodyRotation(0, 0, 0); robot.setBodyTranslation(0, 0, 0);
    robot.profileFlat();
    for (int i = 0; i < 100; i++) { __nowMs += 20; robot.update(); }
    Sudut a = baca();
    robot.setGaitProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, 60.0f, STAND_RADIUS });
    __nowMs += 20; robot.update();
    Sudut b = baca();
    int leg; const char* sendi;
    float d = bedaMaks(a, b, leg, sendi);
    printf("  %-26s lonjakan 1 commit: %6.2f der (%6.1f us)  <- kaki %d %s\n",
           "b60 (100mm -> 60mm)", d, d * US_PER_DEG, leg, sendi);
    int siklus = 0; Sudut prev = b;
    for (int i = 0; i < 400; i++) {
        __nowMs += 20; robot.update();
        Sudut kini = baca(); int l; const char* s2;
        if (bedaMaks(prev, kini, l, s2) < 0.01f) break;
        prev = kini; siklus++;
    }
    printf("  %-26s siklus sampai diam : %d  (%s)\n", "", siklus,
           siklus == 0 ? "LANGSUNG -- tidak ada ramp" : "ada ramp");
    return 0;
}
