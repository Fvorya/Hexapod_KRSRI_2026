// Apakah tunggu "profil gait tenang" benar-benar menunggu, dan benar-benar
// selesai? Dua-duanya harus dibuktikan: gerbang yang tidak pernah menutup
// tidak menolong apa pun, dan gerbang yang tidak pernah membuka menggantung
// misi di tempat.
//
// Yang diuji Hexapod::gaitProfilTenang() -- predikat yang dipakai MISI_SETEL
// di Misi.cpp. Mesin statusnya sendiri butuh robot; ini menguji satu-satunya
// bagian yang bisa salah diam-diam.
//
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited \
//       ../test-pc/stub/stubdefs.cpp Hexapod_Unlimited/{Hexapod,HexaGait,\
//       HexaServos,HexaArm,Calib,ArmInverse,LegInverseKinematics}.cpp \
//       cek_setel_profil.cpp -o /tmp/cek_setel && /tmp/cek_setel
#include <Arduino.h>
#include <cassert>
#include <cstdio>
#include "Calib.h"
#include "Hexapod.h"

static Hexapod robot;

// Berapa milidetik sampai gerbangnya membuka? -1 = tidak pernah.
static long tunggu(void (*profil)(Hexapod&)) {
    for (int i = 0; i < 200; i++) { __nowMs += 20; robot.update(); }   // diamkan dulu
    assert(robot.gaitProfilTenang());                                  // diam = tenang

    profil(robot);
    assert(!robot.gaitProfilTenang());          // ganti profil = TIDAK tenang lagi

    uint32_t t0 = __nowMs;
    for (int i = 0; i < 500; i++) {             // 10 detik, jauh di atas 5 x tau
        __nowMs += 20; robot.update();
        if (robot.gaitProfilTenang()) return (long)(__nowMs - t0);
    }
    return -1;
}

int main() {
    // TANPA INI GAIT_PROFILE_TAU = 0 dan ramp profil selesai dalam SATU tick --
    // gerbangnya lolos seketika dan ujian ini akan lulus tanpa menguji apa pun.
    // Di robot, setup() yang memanggilnya; di sini tidak ada setup().
    Calib::applyDefaults();

    struct { const char* nama; void (*f)(Hexapod&); } uji[] = {
        { "DATAR -> TANGGA",   [](Hexapod& r) { r.profileStairs(); } },
        { "TANGGA -> MERUNDUK",[](Hexapod& r) { r.profileCrouch(); } },
        { "MERUNDUK -> SEMPIT",[](Hexapod& r) { r.profileNarrow(); } },
        { "SEMPIT -> DATAR",   [](Hexapod& r) { r.profileFlat();   } },
    };

    for (auto& u : uji) {
        long ms = tunggu(u.f);
        printf("  %-22s tenang sesudah %ld ms\n", u.nama, ms);
        assert(ms >= 300);     // benar-benar menunggu, bukan lolos dalam satu tick
        assert(ms < 3000);     // dan benar-benar selesai
    }
    printf("cek_setel_profil: LOLOS\n");
    return 0;
}
