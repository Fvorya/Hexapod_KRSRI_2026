#include "ArmInverse.h"

// IK 2 link planar (bahu + siku), solusi "siku ke atas".
// return false bila target TIDAK terjangkau -- sudut yang dikembalikan sudah
// di-clamp, jadi jangan dipakai kalau false.
bool ArmInverse::solve(float x, float y, float& shoulderDeg, float& elbowDeg) {
    bool inRange = true;

    // pow() itu double di Teensy -- mahal dan tak perlu untuk kuadrat.
    const float x2 = x * x;
    const float y2 = y * y;
    const float upperArm2 = UPPERARM_LENGTH * UPPERARM_LENGTH;
    const float foreArm2  = FOREARM_LENGTH  * FOREARM_LENGTH;

    const float r = sqrtf(x2 + y2);   // jarak target dari pangkal bahu

    // Batas LUAR: lengan tak cukup panjang.
    // Batas DALAM: titik terlalu dekat ke bahu -- ada lingkaran mati beradius
    //   |UPPERARM - FOREARM| yang tak bisa dicapai betapapun siku ditekuk.
    // Versi lama hanya memeriksa batas luar, sehingga target di dalam
    // lingkaran mati dilaporkan "terjangkau" padahal cosElbow ter-clamp ke -1
    // dan sudut yang keluar ngawur.
    const float rMax = UPPERARM_LENGTH + FOREARM_LENGTH;
    const float rMin = fabsf(UPPERARM_LENGTH - FOREARM_LENGTH);
    if (r > rMax || r < rMin) inRange = false;

    float cosElbow = clampf((x2 + y2 - upperArm2 - foreArm2) /
                            (2.0f * UPPERARM_LENGTH * FOREARM_LENGTH), -1.0f, 1.0f);

    // acos(cosElbow), ditulis lewat atan2 supaya stabil di dekat +-1.
    float elbowRad = atan2f(sqrtf(1.0f - cosElbow * cosElbow), cosElbow);
    elbowDeg = rad2deg(elbowRad);

    float upper1 = atan2f(y, x);
    float upper2 = atan2f(FOREARM_LENGTH * sinf(elbowRad),
                          UPPERARM_LENGTH + FOREARM_LENGTH * cosf(elbowRad));
    shoulderDeg = rad2deg(upper1 - upper2);

    return inRange;
}
