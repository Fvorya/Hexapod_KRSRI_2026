// Berapa cepat servo diminta berputar pada tiap setelan gait?
// Diukur dari pulse yang BENAR-BENAR dikirim ke driver PCA9685 tiap commit.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "Hexapod.h"
#include <Adafruit_PWMServoDriver.h>

static Hexapod robot;

static void salin(uint16_t d[32]) {
    for (int s = 0; s < 16; s++) { d[s] = __servoUs[0][s]; d[16 + s] = __servoUs[1][s]; }
}

static void ukur(float stepLen, float cycle, float tinggi = -1.0f) {
    if (tinggi > 0) gParam[K_GAIT_STEP_HEIGHT] = tinggi;
    gParam[K_GAIT_STEP_LENGTH] = stepLen;
    gParam[K_GAIT_CYCLE_TIME]  = cycle;
    robot.begin(); robot.arm(); robot.profileFlat();
    robot.walk(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 600; i++) { __nowMs += 5; robot.update(); }  // biarkan mantap

    uint16_t prev[32]; salin(prev);
    float maxDeg = 0.0f; int maxS = -1;
    const float usPerDeg = (float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180.0f;
    uint32_t tKirim = __nowMs;
    for (int i = 0; i < 2000; i++) {
        __nowMs += 5; robot.update();
        if (__nowMs - tKirim < (uint32_t)SERVO_COMMIT_MS) continue;
        tKirim = __nowMs;
        uint16_t now[32]; salin(now);
        for (int s = 0; s < 32; s++) {
            if (!prev[s] || !now[s]) { prev[s] = now[s]; continue; }
            float d = fabsf((float)now[s] - (float)prev[s]) / usPerDeg;
            if (d > maxDeg) { maxDeg = d; maxS = s; }
            prev[s] = now[s];
        }
    }
    const char* sendi = (maxS < 0) ? "?" : ((maxS % 3 == 0) ? "coxa" : (maxS % 3 == 1) ? "femur" : "tibia");
    printf("  step=%5.1f angkat=%4.0f cycle=%5.0f ms -> maks %5.2f der/commit = %5.0f der/detik  (sendi tersibuk: %s, %s)\n",
           stepLen, (float)GAIT_STEP_HEIGHT, cycle, maxDeg, maxDeg * 1000.0f / (float)SERVO_COMMIT_MS,
           sendi, robot.lastPoseInRange() ? "IK OK" : "DI-CLAMP");
}

int main() {
    Calib::load();
    printf("\n== Tuntutan kecepatan servo (commit tiap %d ms) ==\n", SERVO_COMMIT_MS);
    printf("   patokan: servo hobi 0,16 detik/60 der = 375 der/detik TANPA beban\n\n");
    printf("-- sekarang --\n");
    ukur(60, 900, 40);
    printf("\n-- pengaruh step_length (angkat 40) --\n");
    ukur(80, 900, 40); ukur(100, 900, 40);
    printf("\n-- pengaruh cycle_time (step 60, angkat 40) --\n");
    ukur(60, 700, 40); ukur(60, 500, 40);
    printf("\n-- pengaruh step_height (step 60, cycle 900) --\n");
    ukur(60, 900, 30); ukur(60, 900, 25); ukur(60, 900, 20);
    printf("\n-- usulan: langkah lebih besar + angkat lebih rendah --\n");
    ukur(90, 900, 25); ukur(90, 750, 25); ukur(100, 700, 20);
    return 0;
}
