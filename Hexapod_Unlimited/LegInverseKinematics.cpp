#include "LegInverseKinematics.h"

bool LegInverseKinematics::solve(float x, float y, float z,
                  float& coxaDeg, float& femurDeg, float& tibiaDeg,
                  float kunciLututDeg) {
    bool inRange = true;

    // 1) Coxa: putar horizontal menghadap titik target.
    float coxaRad = atan2f(y, x);

    // 2) Pindah ke bidang vertikal kaki.
    float horiz = sqrtf(x * x + y * y);   // jarak horizontal dari pangkal coxa
    float Lr = horiz - COXA_LENGTH;        // jarak horizontal dari sendi femur
    float D = sqrtf(Lr * Lr + z * z);      // jarak lurus femur-joint -> ujung kaki

    float F2 = FEMUR_LENGTH * FEMUR_LENGTH;
    float T2 = TIBIA_LENGTH * TIBIA_LENGTH;

    if (isnan(kunciLututDeg)) {
        // 3) Clamp D ke jangkauan fisik (cegah NaN pada acos).
        const float Dmin = fabsf(FEMUR_LENGTH - TIBIA_LENGTH) + 1.0f;
        const float Dmax = (FEMUR_LENGTH + TIBIA_LENGTH) - 1.0f;
        if (D < Dmin) { D = Dmin; inRange = false; }
        if (D > Dmax) { D = Dmax; inRange = false; }
    } else {
        // 3') LUTUT DIKUNCI: D ditentukan segitiga femur-tibia, bukan target.
        // Cukup menimpa D di sini -- langkah 4 dan 5 di bawah tidak berubah
        // sedikit pun, dan langkah 5 mengembalikan kunciLututDeg itu sendiri.
        //
        // D hasil rumus kosinus ini SELALU di dalam jangkauan fisik, jadi
        // kaki dengan lutut terkunci tidak pernah "di luar jangkauan": ia
        // cuma tidak pergi ke tempat yang diminta. Yang masih bisa mentok
        // sudut servonya, dan itu dijaga angleToPulse().
        D = sqrtf(F2 + T2 - 2.0f * FEMUR_LENGTH * TIBIA_LENGTH *
                  cosf(deg2rad(clampf(kunciLututDeg, 1.0f, 179.0f))));
    }
    float D2 = D * D;

    // 4) Femur dari horizontal: sudut D + sudut segitiga di sendi femur.
    float a1 = atan2f(z, Lr);                                   // arah D (negatif jika ke bawah)
    float a2 = acosf(clampf((F2 + D2 - T2) / (2.0f * FEMUR_LENGTH * D), -1.0f, 1.0f));
    float femurRad = a1 + a2;

    // 5) Sudut interior lutut (tibia).
    float kneeRad = acosf(clampf((F2 + T2 - D2) / (2.0f * FEMUR_LENGTH * TIBIA_LENGTH), -1.0f, 1.0f));

    coxaDeg  = rad2deg(coxaRad);
    femurDeg = rad2deg(femurRad);
    tibiaDeg = rad2deg(kneeRad);
    return inRange;
}
