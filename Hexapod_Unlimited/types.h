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

// ---------------------------------------------------------------- rotasi
// KONVENSI FRAME (sama dengan config.h & TES_GERAK/kinematics.h):
//   +X = KANAN, +Y = DEPAN, +Z = ATAS, origin = pusat badan.
// Dari situ nama sumbu rotasi mengikuti FISIKA:
//   ROLL  = putar terhadap sumbu DEPAN (+Y) -> badan miring kanan/kiri
//   PITCH = putar terhadap sumbu KANAN (+X) -> badan mendongak/menunduk
//   YAW   = putar terhadap +Z, positif = berlawanan jarum jam (belok KIRI)
//
// KOREKSI (Agustus 2026): versi lama memberi rollRad ke rotasi sumbu X dan
// pitchRad ke sumbu Y -- TERTUKAR untuk frame ini. Perintah "roll" menghasilkan
// gerak PITCH dan sebaliknya. TES_GERAK/motion.h sudah mencatat ketidakcocokan
// ini ("nama di sini mengikuti FISIKA, bukan firmware"); sekarang firmware yang
// disesuaikan supaya keduanya bicara bahasa yang sama.
//
// Yang dihitung adalah INVERS rotasi badan: bila badan berputar R terhadap
// dunia sementara telapak diam di tanah, di frame badan telapak tampak
// berputar R^-1. R = Rz(yaw)*Ry(roll)*Rx(pitch), jadi
// R^-1 = Rx(-pitch)*Ry(-roll)*Rz(-yaw) -- URUTAN DIBALIK, bukan cuma sudut
// dinegatifkan (menegatifkan sudut saja hanya benar untuk sudut kecil).
static inline Vec3 rotatePointInv(Vec3 p, float rollRad, float pitchRad, float yawRad) {
    float c, s;

    // 1. Rz(-yaw)
    c = cosf(yawRad);  s = sinf(yawRad);
    float x1 =  c * p.x + s * p.y;
    float y1 = -s * p.x + c * p.y;
    float z1 =  p.z;

    // 2. Ry(-roll)  -- ROLL berputar terhadap sumbu DEPAN (+Y)
    c = cosf(rollRad); s = sinf(rollRad);
    float x2 =  c * x1 - s * z1;
    float z2 =  s * x1 + c * z1;
    float y2 =  y1;

    // 3. Rx(-pitch) -- PITCH berputar terhadap sumbu KANAN (+X)
    c = cosf(pitchRad); s = sinf(pitchRad);

    // Kembalikan langsung ke dalam struktur Vec3
    return { x2, c * y2 + s * z2, -s * y2 + c * z2 };
}

#endif
