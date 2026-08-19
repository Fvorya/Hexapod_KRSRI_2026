#ifndef TYPES_H
#define TYPES_H

// File ini MURNI (tanpa Arduino.h) supaya matematika bisa diuji di PC (lihat test/).
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    float x, y, z;
};

// --- Helper matematika kecil (inline, tanpa dependensi) ---
inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Selisih sudut terpendek (-180..180), input/output derajat.
inline float angleDiffDeg(float target, float current) {
    float d = target - current;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Kontroler PD (tanpa I: hindari windup pada wall-follow/heading).
// step() butuh dt (detik) dari pemanggil; reset() saat target lompat (ganti state).
// struct Pid {
//     float kp, kd;
//     float prev = 0.0f;
//     bool  has  = false;            // belum ada sampel sebelumnya -> D=0
//     float step(float err, float dt) {
//         float d = (has && dt > 1e-4f) ? (err - prev) / dt : 0.0f;
//         prev = err; has = true;
//         return kp * err + kd * d;
//     }
//     void reset() { has = false; prev = 0.0f; }
// };

// Rotasi titik oleh roll(X), pitch(Y), yaw(Z) dalam radian.
// Urutan: Rz * Ry * Rx (intrinsic). Dipakai untuk body kinematics.
// ponytail: rotasi titik langsung, bukan bangun matriks 4x4 + CMSIS-DSP.
// Untuk 6 titik/loop ini lebih sederhana & cukup cepat di FPU Teensy.
// Invers Rotasi Sejati: Z-Y-X dibalik menjadi X-Y-Z
static inline Vec3 rotatePointInv(Vec3 p, float rollRad, float pitchRad, float yawRad) {
    float c, s;
    
    // 1. Yaw (Rotasi Z dibalik)
    c = cosf(yawRad);  s = sinf(yawRad);
    float x1 =  c * p.x + s * p.y;
    float y1 = -s * p.x + c * p.y;
    float z1 =  p.z;
    
    // 2. Roll (Rotasi Y dibalik)
    c = cosf(pitchRad); s = sinf(pitchRad);
    float x2 =  c * x1 - s * z1;
    float z2 =  s * x1 + c * z1;
    float y2 =  y1;
    
    // 3. Pitch (Rotasi X dibalik)
    c = cosf(rollRad); s = sinf(rollRad);
    
    // Kembalikan langsung ke dalam struktur Vec3
    return { x2, c * y2 + s * z2, -s * y2 + c * z2 };
}

#endif
