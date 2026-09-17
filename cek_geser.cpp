// Berapa lama perataan sisi menempuh sasarannya, dan sedekat apa berhentinya?
//
// Lup TERTUTUP: dinding tiruan digerakkan oleh odometri geser robot sendiri,
// lalu disuapkan ke LidarArray yang asli -- jadi median-3 dan giliran mux ikut
// terhitung, termasuk keterlambatannya. Yang dipanggil ratakanMulai() dan
// navUpdate() yang asli, bukan tiruan.
//
// Dua hal yang harus benar sekaligus, dan itulah sebabnya laju geser tidak
// bisa cuma "dibesarkan":
//   1. CEPAT -- 0,25 sepanjang jalan menghabiskan 8 detik untuk 27 cm.
//   2. TELITI -- sumbu geser melompat satu langkah gait, jadi laju besar di
//      dekat sasaran berarti terlewat.
//
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited \
//       ../test-pc/stub/stubdefs.cpp Hexapod_Unlimited/{Navigation,Hexapod,\
//       HexaGait,HexaServos,HexaArm,Imu,LidarArray,Calib,ArmInverse,\
//       LegInverseKinematics}.cpp cek_geser.cpp -o /tmp/cek_geser && /tmp/cek_geser
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cassert>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "LidarArray.h"
#include "Navigation.h"

static Imu imu; static Hexapod robot; static LidarArray lidar;
static Navigation nav(imu, robot, lidar);

static void suap(uint8_t ch, float cm) {
    uint16_t mm = (uint16_t)(cm * 10.0f);
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}

// Geser dari `awal` cm ke `sasaran` cm. Mengembalikan detik yang ditempuh;
// `akhir` diisi jarak dinding sebenarnya saat robot berhenti.
static float lari(float awal, int sasaran, float& akhir) {
    for (uint8_t ch = 0; ch < 6; ch++) suap(ch, 100.0f);
    suap(LIDAR_KANAN_D, awal); suap(LIDAR_KANAN_B, awal);
    for (int i = 0; i < 400; i++) { lidar.update(); __nowMs += 2; }

    robot.jarakNol();
    assert(nav.ratakanMulai(false, sasaran));

    uint32_t t0 = __nowMs;
    for (int i = 0; i < 3000 && nav.ratakanSedangJalan(); i++) {
        __nowMs += 20;
        // Geser KANAN (+) mendekatkan dinding kanan.
        float d = awal - robot.geserCm();
        suap(LIDAR_KANAN_D, d); suap(LIDAR_KANAN_B, d);
        lidar.update(); imu.update(); nav.navUpdate(); robot.update();
    }
    akhir = awal - robot.geserCm();
    return (__nowMs - t0) / 1000.0f;
}

// Laju geser terukur (cm/detik) pada perintah tetap -- dipakai menghitung
// berapa lama perataan LAMA (0,25 rata sepanjang jalan) akan menempuh jarak
// yang sama. Bandingannya jadi ikut profil, bukan angka detik yang dihafal.
static float lajuTetap(float g) {
    robot.jarakNol();
    robot.walk(0.0f, g, 0.0f);
    for (int i = 0; i < 500; i++) { __nowMs += 20; robot.update(); }
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 40; i++) { __nowMs += 20; robot.update(); }
    return robot.geserCm() / 10.0f;
}

int main() {
    Calib::applyDefaults();
    robot.begin(); lidar.begin(); robot.arm();
    for (int i = 0; i < 100; i++) { __nowMs += 20; robot.update(); }

    struct { const char* n; void (*f)(Hexapod&); } P[] = {
        { "DATAR",    [](Hexapod& r) { r.profileFlat();   } },
        { "MERUNDUK", [](Hexapod& r) { r.profileCrouch(); } },
    };

    for (auto& p : P) {
        p.f(robot);
        for (int i = 0; i < 150; i++) { __nowMs += 20; robot.update(); }

        float lama = 27.0f / lajuTetap(0.25f);     // cara lama, profil ini
        float akhir = 0.0f;
        float detik = lari(40.0f, 13, akhir);
        float salah = fabsf(akhir - 13.0f);
        printf("  %-9s 40 -> 13 cm : %4.1f detik (cara lama %4.1f), "
               "berhenti di %4.1f cm (meleset %.1f)\n",
               p.n, (double)detik, (double)lama, (double)akhir, (double)salah);
        fflush(stdout);

        assert(nav.ratakanTercapai());
        assert(salah <= 3.5f);      // <= satu lompatan gait pada laju halus
        assert(detik <= 0.65f * lama);    // dan benar-benar lebih cepat
        nav.navBerhenti(nullptr);
    }
    // Perintah 'U' (naik tangga) menyalakan abaikanDepan lalu menyerahkan
    // pemulihannya SEPENUHNYA pada navBerhenti() -- begitu juga pesan yang
    // dicetak 'i1' ("Berhenti apa pun memulihkannya"). Kalau janji itu pernah
    // hilang, robot berjalan buta ke depan di ruas BERIKUTNYA, dan tidak ada
    // apa pun di layar yang mengatakannya. Karena itu ia dijaga di sini.
    for (uint8_t ch = 0; ch < 6; ch++) suap(ch, 100.0f);
    for (int i = 0; i < 10; i++) { __nowMs += 20; lidar.update(); }
    nav.navMulai(NAV_DINDING_KANAN);
    assert(nav.navMode() != NAV_DIAM);
    nav.abaikanDepan(true);
    assert(nav.depanDiabaikan());
    nav.navBerhenti("uji pemulihan");
    assert(!nav.depanDiabaikan());
    printf("  abaikanDepan dipulihkan oleh navBerhenti: ya\n");

    // LORONG SEMPIT, ROBOT DI TENGAH, sensor berderau. Ini keadaan R-9 saat
    // 'U': kedua dinding di dalam pita "terlalu dekat" sekaligus. Dulu
    // dorongan menjauh bang-bang terhadap "dinding mana yang lebih dekat",
    // dan pertanyaan itu dijawab oleh derau -- kemudi menghentak +-MAX
    // bolak-balik justru saat robot sudah benar di tengah. Yang dihitung di
    // sini pembalikan tanda kemudi, bukan besarnya: chattering-lah gejalanya.
    nav.setTengah(true);
    for (uint8_t ch = 0; ch < 6; ch++) suap(ch, 100.0f);
    for (int i = 0; i < 10; i++) { __nowMs += 20; lidar.update(); }
    nav.navMulai(NAV_DINDING_KANAN);
    assert(nav.navMode() != NAV_DIAM);

    const float derau[8] = { 0.2f, -0.2f, 0.0f, 0.3f, -0.1f, 0.1f, -0.3f, 0.0f };
    int balik = 0;
    float turnLalu = 0.0f;
    for (int i = 0; i < 40; i++) {
        const float d = derau[i % 8];                 // lorong 23 cm, robot di tengah
        suap(LIDAR_KIRI_D,  11.5f + d);  suap(LIDAR_KIRI_B,  11.5f + d);
        suap(LIDAR_KANAN_D, 11.5f - d);  suap(LIDAR_KANAN_B, 11.5f - d);
        for (int k = 0; k < 5; k++) { __nowMs += 20; lidar.update(); nav.navUpdate(); robot.update(); }
        const float t = nav.turnKini();
        if (i > 2 && t * turnLalu < 0.0f && fabsf(t - turnLalu) > 0.10f) balik++;
        turnLalu = t;
    }
    printf("  lorong sempit, robot di tengah: kemudi berbalik %dx per 37 sampel\n", balik);
    nav.navBerhenti(nullptr);
    nav.setTengah(false);
    // Sebelum perbaikan angka ini belasan: tiap sampel yang membalik urutan
    // dk/dn membalik kemudi. Robot yang sudah di tengah tidak punya alasan
    // membanting setir sama sekali, jadi ambangnya dibuat ketat.
    assert(balik <= 2);

    // KUNCI HEADING MUTLAK. Ruas menyerong menyebutkan heading-nya sendiri
    // supaya mode arena tidak menebaknya dari yaw -- pada serong 45 der, yaw
    // berjarak sama dari dua mata angin dan tebakan yang salah menarik badan
    // 90 der dari yang dimaksud SEPANJANG RUAS, tanpa gejala lain.
    // Dua hal yang dijaga: kuncinya dibungkus ke 0..360, dan tiap berhenti
    // MELEPASNYA -- kunci yang tertinggal membuat ruas berikutnya, atau 'f'
    // manual, mewarisi serong ruas yang sudah lewat.
    nav.kunciHeading(NAN);
    assert(isnan(nav.headingKunci()));
    nav.kunciHeading(400.0f);
    assert(fabsf(nav.headingKunci() - 40.0f) < 0.01f);
    nav.kunciHeading(-30.0f);
    assert(fabsf(nav.headingKunci() - 330.0f) < 0.01f);
    assert(fabsf(nav.headingTerkunci() - 330.0f) < 0.01f);
    nav.navBerhenti("uji lepas kunci");
    assert(isnan(nav.headingKunci()));
    printf("  kunci heading dibungkus 0..360 dan dilepas navBerhenti: ya\n");

    // simpangHeading harus memberi jalan TERPENDEK, bukan selisih mentah.
    // Tanpa itu penilai odometri membaca serong 2 der sebagai 358 dan
    // menghentikan misi di tengah ruas yang sebenarnya lurus.
    assert(isnan(nav.simpangHeading(NAN)));

    printf("cek_geser: LOLOS\n");
    return 0;
}
