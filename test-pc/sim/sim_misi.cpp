// PIVOT AWAL MISI: robot diletakkan menghadap ke ARAH ACAK, misi harus
// memberangkatkannya ke UTARA.
//
// Ini menguji satu hal yang tidak punya gejala di lapangan sampai terlambat:
// navMulai() mengunci ke mata angin TERDEKAT dari hadap robot saat itu, jadi
// start yang menyimpang >45 der mengunci arah yang SALAH tanpa satu pun pesan
// error -- robot berjalan mantap ke lorong yang keliru. Karena itu yang
// diperiksa di sini bukan "misinya jalan", melainkan "heading akhirnya UTARA".
//
// Memakai Mission/Navigation/Imu/Hexapod YANG ASLI; yang dipalsukan hanya jam,
// bus I2C, EEPROM, dan aliran byte IMU.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include <cmath>
#include <stddef.h>
#include <cstring>

#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"
#include "Mission.h"
#include "EEMap.h"

static Imu        imu;
static Hexapod    robot;
static LidarArray lidar;
static Navigation nav(imu, robot, lidar);
static Mission    misi(robot, nav, lidar);

static float simYaw = 0.0f;

// Jarak robot dari dinding START, seperti yang dilihat sensor BELAKANG.
// Robot diletakkan menempel dinding start, lalu angka ini tumbuh mengikuti
// perintah maju yang BENAR-BENAR dikeluarkan Navigation.
static const float BLK_AWAL_CM = 8.0f;
static const float V_PENUH_CMS = 10.7f;    // cm/detik pada maju 1.0 (sim_laju)
static float blkCm = BLK_AWAL_CM;

// --- IMU palsu: frame WIT 0x53 sungguhan, sama dengan sim_pivot ---
static void suapYaw(float yaw) {
    auto enc = [](float deg) { return (int16_t)lrintf(deg / 180.0f * 32768.0f); };
    float y = yaw; while (y > 180.0f) y -= 360.0f; while (y < -180.0f) y += 360.0f;
    int16_t v0 = 0, v1 = 0, v2 = enc(y);
    uint8_t f[11] = { 0x55, 0x53,
                      (uint8_t)(v0 & 0xFF), (uint8_t)(v0 >> 8),
                      (uint8_t)(v1 & 0xFF), (uint8_t)(v1 >> 8),
                      (uint8_t)(v2 & 0xFF), (uint8_t)(v2 >> 8),
                      0, 0, 0 };
    uint8_t s = 0; for (int i = 0; i < 10; i++) s += f[i];
    f[10] = s;
    for (int i = 0; i < 11; i++) Serial2.rx.push_back(f[i]);
}

// Imu::update() MENOLAK lompatan > IMU_MAX_YAW_JUMP sampai beberapa kali
// berturut-turut. Untuk memindahkan yaw jauh (menyusun kompas, menaruh robot
// di arah acak) frame harus disuapkan berulang sampai penolakannya menyerah.
static void suapLidar();   // didefinisikan di bawah; paksaYaw perlu memanggilnya

static void paksaYaw(float y) {
    simYaw = y;
    // LiDAR ikut disuapi: di firmware sungguhan waktu tidak pernah berjalan
    // tanpa lidar.update(). Kalau di sini hanya IMU yang jalan, 200 ms tanpa
    // sampel baru membuat sensor terlihat MATI dan 'm1' menolak berangkat.
    for (int i = 0; i < 20; i++) {
        suapYaw(simYaw); suapLidar();
        imu.update();    lidar.update();
        __nowMs += 10;
    }
}

// --- LiDAR palsu: lorong terbuka di depan, dinding kanan di jarak setpoint ---
static void suapLidar() {
    for (int c = 0; c < 6; c++) { __simMm[c] = 8000; __simStatus[c] = VL53L1X::SignalFail; }
    __simMm[LIDAR_FRONT]   = 600; __simStatus[LIDAR_FRONT]   = VL53L1X::RangeValid;
    __simMm[LIDAR_KANAN_D] = 200; __simStatus[LIDAR_KANAN_D] = VL53L1X::RangeValid;
    __simMm[LIDAR_KANAN_B] = 200; __simStatus[LIDAR_KANAN_B] = VL53L1X::RangeValid;
    __simMm[LIDAR_BACK]    = (uint16_t)(blkCm * 10.0f);
    __simStatus[LIDAR_BACK] = VL53L1X::RangeValid;
}

static const float DEG_PER_S_PENUH = 45.0f;   // pada perintah putar 1.0

// Satu tick loop utama firmware. URUTANNYA SAMA dengan loop() sungguhan:
// misi.update() SEBELUM nav.navUpdate().
static void tick() {
    __nowMs += 10;
    blkCm  += nav.majuKini() * V_PENUH_CMS * 0.01f;
    simYaw += nav.turnKini() * DEG_PER_S_PENUH * 0.01f;
    while (simYaw >= 360.0f) simYaw -= 360.0f;
    while (simYaw <    0.0f) simYaw += 360.0f;

    suapYaw(simYaw);
    suapLidar();
    imu.update();
    lidar.update();
    misi.update();
    nav.navUpdate();
    robot.update();
}

static float simpangUtara() {
    float d = simYaw; while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f;
    return fabsf(d);
}

// Satu keberangkatan penuh dari heading awal tertentu. return 0 bila lulus.
static int berangkatDari(float yawAwal) {
    printf("\n-- start menghadap %.0f der --\n", yawAwal);
    paksaYaw(yawAwal);
    blkCm = BLK_AWAL_CM;

    misi.mulai();
    if (misi.stat() != MISI_PIVOT_AWAL) {
        printf("  GAGAL: misi tidak masuk MISI_PIVOT_AWAL (stat=%d)\n", (int)misi.stat());
        return 1;
    }
    if (!nav.pivotSedangJalan()) { printf("  GAGAL: pivot tidak jalan\n"); return 1; }

    int n = 0;
    while (misi.stat() == MISI_PIVOT_AWAL && n < 12000) { tick(); n++; }

    printf("  sesudah %d ms: yaw=%.1f  simpang dari UTARA=%.1f der  stat=%d\n",
           n * 10, simYaw, simpangUtara(), (int)misi.stat());

    if (misi.stat() != MISI_KE_KORBAN1) {
        printf("  GAGAL: tidak sampai ke MISI_KE_KORBAN1\n"); return 1;
    }
    // INTI UJI INI. Tanpa pivot awal, navMulai() akan mengunci mata angin
    // terdekat dari yawAwal -- dan dari 200 der itu SELATAN, bukan UTARA.
    if (simpangUtara() > HEADING_TOLERANCE_DEG) {
        printf("  GAGAL: berangkat bukan ke UTARA\n"); return 1;
    }
    if (nav.navMode() != NAV_ARENA_KANAN) {
        printf("  GAGAL: navigasi bukan NAV_ARENA_KANAN\n"); return 1;
    }
    // Sudah menyusuri dinding: harus benar-benar maju, bukan diam berputar.
    for (int i = 0; i < 50; i++) tick();
    if (nav.majuKini() <= 0.0f) {
        printf("  GAGAL: tidak maju sesudah pivot (maju=%.2f)\n", nav.majuKini()); return 1;
    }
    printf("  OK -- berangkat ke UTARA, maju=%.2f\n", nav.majuKini());
    misi.batal("selesai uji.");
    return 0;
}

int main() {
    Calib::load();

    // Kalibrasi pivot palsu di EEPROM 2048: tanpa ini mode arena MENOLAK jalan.
    GerakStore g;
    memset(&g, 0, sizeof(g));
    g.m0 = 0x6E; g.m1 = 0x2C; g.ver = 2;
    g.ccw = 45.0f; g.cw = 45.0f; g.maju = 20.0f; g.sign = +1;
    g.sum = eeSum(&g, offsetof(GerakStore, sum));
    EEPROM.put(EE_GERAK_ADDR, g);

    robot.begin(); lidar.begin(); robot.arm();
    nav.begin();

    // Isi LiDAR sampai tiap channel punya sampel (round-robin 6 channel).
    for (int i = 0; i < 300; i++) { suapLidar(); lidar.update(); __nowMs += 2; }

    printf("\n=== KOMPAS ARENA: U=0 T=90 S=180 B=270 ===\n");
    paksaYaw(0.0f);   nav.kompasCatat(0);
    paksaYaw(90.0f);  nav.kompasCatat(1);
    paksaYaw(180.0f); nav.kompasCatat(2);
    paksaYaw(270.0f); nav.kompasCatat(3);
    if (!nav.kompasLengkap()) { printf("GAGAL: kompas tidak lengkap\n"); return 1; }
    if (!nav.pivotTerkalibrasi()) { printf("GAGAL: pivot tidak terkalibrasi\n"); return 1; }

    printf("\n=== UJI 1: empat heading awal acak, semuanya harus berangkat ke UTARA ===\n");
    // 200 dan 170 sengaja dipilih: keduanya lebih dekat ke SELATAN, jadi
    // navMulai() TANPA pivot awal akan mengunci arah yang berlawanan.
    const float awal[] = { 200.0f, 170.0f, 95.0f, 350.0f };
    for (float a : awal) if (berangkatDari(a)) return 1;

    printf("\n=== UJI 2: dibatalkan saat pivot awal -> GAGAL, bukan jalan diam-diam ===\n");
    paksaYaw(200.0f);
    misi.mulai();
    if (misi.stat() != MISI_PIVOT_AWAL) { printf("  GAGAL: tidak masuk pivot awal\n"); return 1; }
    for (int i = 0; i < 30; i++) tick();          // 300 ms, jelas belum sampai
    nav.navBerhenti("dibatalkan pengguna ('x').");
    tick();
    printf("  stat sesudah dibatalkan = %d  (MISI_GAGAL = %d)\n",
           (int)misi.stat(), (int)MISI_GAGAL);
    if (misi.stat() != MISI_GAGAL) {
        printf("  GAGAL: misi tidak melaporkan kegagalan\n"); return 1;
    }
    if (nav.navMode() != NAV_DIAM) {
        printf("  GAGAL: navigasi malah jalan sesudah pivot dibatalkan\n"); return 1;
    }
    printf("  OK -- berhenti dengan sebab yang tercetak, tidak menyusuri dinding.\n");

    printf("\n=== UJI 3: jarak tempuh menghentikan di K-1, lalu badan menghadap BARAT ===\n");
    // Inilah alasan pemicu belakang ada: K-1 duduk DI SAMPING lintasan, jadi
    // sensor depan tetap membaca lorong terbuka 60 cm sepanjang uji ini.
    // Titik nolnya TIDAK diketik: 'm1' mencatatnya sendiri sesudah pivot ke
    // UTARA, jadi yang diuji juga bahwa 8 + 40 = 48, bukan 40.
    paksaYaw(0.0f);
    blkCm = BLK_AWAL_CM;
    misi.setAmbangBlk(40.0f);   // jarak TEMPUH; eksplisit, jangan bergantung default
    misi.mulai();
    if (misi.stat() != MISI_PIVOT_AWAL) { printf("  GAGAL: tidak masuk pivot awal\n"); return 1; }
    {
        int n = 0;
        while (misi.stat() != MISI_KONFIRM1 && misi.stat() != MISI_GAGAL && n < 20000) { tick(); n++; }
        printf("  berhenti pada bacaan %.1f cm (titik nol %.1f + tempuh 40 = %.1f), yaw=%.1f, stat=%d\n",
               blkCm, BLK_AWAL_CM, BLK_AWAL_CM + 40.0f, simYaw, (int)misi.stat());
        if (misi.stat() != MISI_KONFIRM1) {
            printf("  GAGAL: tidak pernah berhenti di korban 1 -- robot melewatinya\n"); return 1;
        }
        // Titik nol dicatat SENDIRI: kalau salah dianggap sebagai bacaan
        // mutlak, robot akan berhenti di 40 cm, bukan 48.
        if (blkCm < BLK_AWAL_CM + 36.0f || blkCm > BLK_AWAL_CM + 46.0f) {
            printf("  GAGAL: berhenti di jarak yang tidak wajar -- titik nol tidak dipakai?\n");
            return 1;
        }
        float dBarat = fabsf(270.0f - simYaw);
        if (dBarat > HEADING_TOLERANCE_DEG) {
            printf("  GAGAL: badan tidak menghadap BARAT (simpang %.1f der)\n", dBarat); return 1;
        }
        if (nav.navMode() != NAV_DIAM) {
            printf("  GAGAL: navigasi masih jalan saat menunggu konfirmasi\n"); return 1;
        }
        printf("  OK -- berhenti di garis K-1 lalu menghadap BARAT.\n");
    }

    printf("\n=== UJI 3b: 'm3' (bukan korban) memutar badan kembali ke UTARA ===\n");
    {
        float blkSaatItu = blkCm;
        misi.jawab(false);
        if (misi.stat() != MISI_PIVOT_AWAL) {
            printf("  GAGAL: 'm3' tidak memutar badan (stat=%d)\n", (int)misi.stat()); return 1;
        }
        int n = 0;
        while (misi.stat() == MISI_PIVOT_AWAL && n < 12000) { tick(); n++; }
        printf("  sesudah putar balik: yaw=%.1f  simpang UTARA=%.1f  stat=%d\n",
               simYaw, simpangUtara(), (int)misi.stat());
        if (misi.stat() != MISI_KE_KORBAN1) {
            printf("  GAGAL: tidak kembali menyusuri dinding\n"); return 1;
        }
        if (simpangUtara() > HEADING_TOLERANCE_DEG) {
            printf("  GAGAL: berangkat lagi bukan ke UTARA -- terkunci ke arah korban\n");
            return 1;
        }
        // Pemicu jarak SEKALI pakai: tidak boleh langsung berhenti lagi di
        // tempat yang sama walau bacaan belakang masih di atas ambang.
        for (int i = 0; i < 200; i++) tick();
        if (misi.stat() != MISI_KE_KORBAN1) {
            printf("  GAGAL: berhenti lagi di benda yang sama (stat=%d)\n", (int)misi.stat());
            return 1;
        }
        printf("  OK -- jalan lagi ke UTARA, bacaan belakang %.1f -> %.1f cm tanpa memicu ulang.\n",
               blkSaatItu, blkCm);
    }
    misi.batal("selesai uji.");

    printf("\n=== UJI 4: sensor belakang MATI -> 'm1' menolak, bukan jalan buta ===\n");
    __bisu[LIDAR_BACK] = true;
    for (int i = 0; i < 200; i++) { suapLidar(); lidar.update(); __nowMs += 5; }
    if (lidar.getDistance(LIDAR_BACK) != LIDAR_MATI) {
        printf("  LEWAT: stub tidak berhasil mematikan ch2, uji ini dilompati\n");
    } else {
        paksaYaw(0.0f);
        misi.mulai();
        if (misi.stat() != MISI_DIAM) {
            printf("  GAGAL: misi tetap mulai walau sensor belakang mati (stat=%d)\n",
                   (int)misi.stat());
            return 1;
        }
        printf("  OK -- ditolak dengan sebab yang menyebut sensor belakang.\n");
    }

    printf("\nSEMUA UJI LULUS\n");
    return 0;
}
