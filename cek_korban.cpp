// SEKUENS KORBAN TAHAP 1 -- apakah setiap posenya benar-benar TERJANGKAU?
//
// Sekuensnya BUTA: tidak ada sensor yang bisa melaporkan bahwa capit meleset,
// dan angleToPulse() untuk lengan meng-clamp DIAM-DIAM (tidak ada
// _servoClamped seperti kaki). Jadi satu-satunya tempat kesalahan angka bisa
// tertangkap adalah di sini, sebelum diunggah.
//
// Yang diuji fungsi ASLINYA, Hexapod::moveArmGrip() -- bukan salinan rumusnya.
// Amplopnya sempit (beberapa cm), jadi menyalin rumus berarti menguji salinan
// yang suatu saat menyimpang dari yang dipakai robot.
//
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited \
//       ../test-pc/stub/stubdefs.cpp Hexapod_Unlimited/{Hexapod,HexaGait,\
//       HexaServos,HexaArm,Calib,ArmInverse,LegInverseKinematics}.cpp \
//       cek_korban.cpp -o /tmp/cek_korban && /tmp/cek_korban
#include <Arduino.h>
#include <cassert>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "Hexapod.h"

static Hexapod robot;

// Sisa derajat sampai batas terdekat: bahu 0..180, siku 0..180, pergelangan
// +-90. Nol berarti pose itu menempel di batas -- sah menurut rumus, tapi
// satu derajat trim meleset sudah cukup membuatnya ter-clamp di robot.
static float margin(float capitJkn, float capitTgi) {
    float b, s;
    const float t = deg2rad(KORBAN_TAPAK_DER);
    if (!ArmInverse::solve(capitJkn - HAND_LENGTH * cosf(t) - fabsf(ARM_ORIGINS[ARM_DEPAN][1]),
                           capitTgi - HAND_LENGTH * sinf(t) - ARM_ORIGINS[ARM_DEPAN][2],
                           b, s)) return -999.0f;
    const float p = KORBAN_TAPAK_DER - (b + s);
    return fminf(fminf(ARM_BASE_BAHU + b, 180.0f - (ARM_BASE_BAHU + b)),
                 fminf(180.0f - (ARM_BASE_SIKU + s), 90.0f - fabsf(p)));
}

static int buruk = 0;

static void uji(const char* nama, float capitJkn, float tinggiLantai, float badan) {
    const float tgi = -badan + tinggiLantai;
    const bool  ok  = robot.moveArmGrip(ARM_DEPAN, capitJkn, tgi, KORBAN_TAPAK_DER);
    const float m   = margin(capitJkn, tgi);
    printf("    %-8s capit %3.0f mm, %3.0f mm di atas lantai -> a%-4.0f %-4.0f  %s (sisa %5.1f der)\n",
           nama, (double)capitJkn, (double)tinggiLantai,
           (double)(capitJkn - HAND_LENGTH), (double)tgi,
           ok ? "OK    " : "MENTOK", (double)m);
    if (!ok || m < 5.0f) buruk++;
}

int main() {
    Calib::applyDefaults();
    robot.begin(); robot.arm(); robot.armEnable(ARM_DEPAN, true);

    printf("gerbang LiDAR depan %d cm + dudukan %.0f mm -> capit di %.0f mm dari pusat badan\n",
           KORBAN_JARAK_CM, (double)LIDAR_DEPAN_MM, (double)KORBAN_CAPIT_MM);
    printf("bahu di (y %.0f, z %.0f); tapak dijaga MENDATAR, jadi pergelangan\n"
           "selalu %.0f mm di belakang capit.\n",
           (double)ARM_ORIGINS[ARM_DEPAN][1], (double)ARM_ORIGINS[ARM_DEPAN][2],
           (double)HAND_LENGTH);

    // Kedua tinggi badan yang benar-benar dipakai ruas korban: ruas 16 dan 21
    // berprofil TANGGA, sisanya DATAR. Sekuens membaca tingginya dari profil
    // yang sedang berlaku, jadi keduanya harus lolos.
    const float BADAN[2] = { STAND_HEIGHT, STAND_HEIGHT + 15.0f };
    const char* NAMA[2]  = { "DATAR", "TANGGA" };

    for (int i = 0; i < 2; i++) {
        printf("\n  badan %.0f mm (%s):\n", (double)BADAN[i], NAMA[i]);
        uji("dekat",   KORBAN_CAPIT_MM,  KORBAN_TINGGI_MM + KORBAN_DEKAT_MM, BADAN[i]);
        uji("jepit",   KORBAN_CAPIT_MM,  KORBAN_TINGGI_MM,                   BADAN[i]);
        uji("gendong", LENGAN_GENDONG_MM, LENGAN_GENDONG_TGI,                BADAN[i]);
        uji("t-dekat", KORBAN_CAPIT_MM,  KORBAN_TARUH_MM + KORBAN_DEKAT_MM,  BADAN[i]);
        uji("lepas",   KORBAN_CAPIT_MM,  KORBAN_TARUH_MM,                    BADAN[i]);
    }

    // Sampai berapa jauh gerbangnya boleh digeser sebelum pose jepit mentok?
    // Inilah angka yang dicari operator saat menyetel di arena: KORBAN_JARAK_CM
    // ada di tengah jendela ini atau tidak.
    printf("\n  jendela gerbang yang sah pada tinggi jepit %.0f mm:\n", (double)KORBAN_TINGGI_MM);
    for (int i = 0; i < 2; i++) {
        float min1 = -1.0f, maks = -1.0f;
        for (float cm = 15.0f; cm <= 40.0f; cm += 0.5f) {
            const float capit = cm * 10.0f + LIDAR_DEPAN_MM;
            const float tgi   = -BADAN[i] + KORBAN_TINGGI_MM;
            // Pose JEPIT dan DEKAT dua-duanya harus muat: gerbang yang cuma
            // memuat salah satunya membuat sekuens gagal di tengah jalan.
            if (!robot.moveArmGrip(ARM_DEPAN, capit, tgi, KORBAN_TAPAK_DER)) continue;
            if (!robot.moveArmGrip(ARM_DEPAN, capit, tgi + KORBAN_DEKAT_MM,
                                   KORBAN_TAPAK_DER)) continue;
            if (min1 < 0.0f) min1 = cm;
            maks = cm;
        }
        printf("    %-6s : %.1f .. %.1f cm%s\n", NAMA[i], (double)min1, (double)maks,
               (KORBAN_JARAK_CM >= min1 && KORBAN_JARAK_CM <= maks) ? "" : "   << GERBANG DI LUARNYA");
        assert(min1 > 0.0f);
        assert((float)KORBAN_JARAK_CM >= min1 + 1.0f);    // sisakan 1 cm di kedua
        assert((float)KORBAN_JARAK_CM <= maks - 1.0f);    // ujung, bukan menempel
    }

    // Sudut TAPAK melebarkan jendela itu berapa? Pergelangan sekarang ikut
    // ditentukan (moveArmGrip 3 sendi), jadi tapak boleh menunduk -- dan
    // menunduk memindahkan pergelangan MENJAUH dari lantai untuk titik capit
    // yang sama, yang persis melawan batas yang mengikat: bahu tidak boleh
    // turun di bawah 0 der servo.
    printf("\n  jendela gerbang vs sudut tapak (tinggi jepit %.0f mm, badan %.0f):\n",
           (double)KORBAN_TINGGI_MM, (double)BADAN[1]);
    for (float tapak = -60.0f; tapak <= 30.0f; tapak += 15.0f) {
        float min1 = -1.0f, maks = -1.0f;
        for (float cm = 15.0f; cm <= 40.0f; cm += 0.5f) {
            const float capit = cm * 10.0f + LIDAR_DEPAN_MM;
            const float tgi   = -BADAN[1] + KORBAN_TINGGI_MM;
            if (!robot.moveArmGrip(ARM_DEPAN, capit, tgi, tapak)) continue;
            if (!robot.moveArmGrip(ARM_DEPAN, capit, tgi + KORBAN_DEKAT_MM, tapak)) continue;
            if (min1 < 0.0f) min1 = cm;
            maks = cm;
        }
        if (min1 < 0.0f) { printf("    tapak %+4.0f der : (tidak ada)\n", (double)tapak); continue; }
        printf("    tapak %+4.0f der : %4.1f .. %4.1f cm  (lebar %.1f cm)\n",
               (double)tapak, (double)min1, (double)maks, (double)(maks - min1));
    }

    // SLEW. Sekuensnya buta: tiap langkah cuma diberi jatah waktu tetap, dan
    // kalau langkah itu butuh lebih lama, fase berikutnya mulai sementara
    // lengan MASIH BERJALAN -- capit menutup sebelum sampai.
    //
    // Dua laju, jadi dua jatah. Langkah biasa berjalan ARM_SLEW_DEG_S dengan
    // satu LENGAN_JEDA_MS. Lipatan ke REHAT berjalan LENGAN_SLEW_REHAT_DEG_S
    // dan mendapat DUA jatah, karena fase sesudahnya sengaja dikosongkan.
    // Diperiksa di sini supaya menurunkan salah satu laju gagal di PC.
    {
        // Urutannya persis kedua sekuens, disambung: AMBIL lalu TARUH.
        // Keduanya berangkat dari REHAT, karena di situlah lengan ditinggal.
        struct { float bahu, siku, prg; const char* nama; bool lipat; } urut[] = {
            { REHAT_BAHU,        REHAT_SIKU,        REHAT_PERGELANGAN,  "rehat",  false },
            { KORBAN_SIAP_BAHU,  KORBAN_SIAP_SIKU,  KORBAN_SIAP_PRG,    "siap",   false },
            { KORBAN_JEPIT_BAHU, KORBAN_JEPIT_SIKU, KORBAN_JEPIT_PRG,   "jepit",  false },
            { REHAT_BAHU,        REHAT_SIKU,        REHAT_PERGELANGAN,  "lipat1", true  },
            { KORBAN_LEPAS_BAHU, KORBAN_LEPAS_SIKU, KORBAN_LEPAS_PRG,   "lepas",  false },
            { REHAT_BAHU,        REHAT_SIKU,        REHAT_PERGELANGAN,  "lipat2", true  },
        };
        const int N = (int)(sizeof(urut) / sizeof(urut[0]));
        uint16_t pul[16][ARM_N_DEPAN];
        printf("\n  pose sekuens korban (sudut sendi, bukan IK):\n");
        for (int k = 0; k < N; k++) {
            float sv[3];
            const bool sanggup =
                robot.setSudutLengan(ARM_DEPAN, urut[k].bahu, urut[k].siku, urut[k].prg, sv);
            printf("    %-7s bahu %+6.1f siku %+6.1f prg %+6.1f  ->  servo %5.1f %5.1f %5.1f%s\n",
                   urut[k].nama, (double)urut[k].bahu, (double)urut[k].siku,
                   (double)urut[k].prg, (double)sv[0], (double)sv[1], (double)sv[2],
                   sanggup ? "" : "   << DI LUAR 0..180");
            // Pose yang mentok tetap dikirim ke servo, jadi tidak ada gejala
            // lain selain capit yang berhenti di tempat yang salah.
            assert(sanggup);
            for (int id = 0; id < ARM_N_DEPAN; id++)
                pul[k][id] = robot.armDepan()->targetPulse((uint8_t)id);
        }
        const float usPerDer = (float)(SERVO_ARM_PULSE_MAX - SERVO_ARM_PULSE_MIN) / 180.0f;
        printf("\n  tiap langkah terhadap jatahnya:\n");
        float msPerlu = 0.0f;
        for (int k = 1; k < N; k++) {
            // Sendi bergerak BERURUTAN sejak HexaArm diberi tahap: bahu (dengan
            // grip) sampai selesai, lalu siku, lalu pergelangan. Jadi waktu satu
            // langkah adalah JUMLAH ketiga tahap, bukan sendi yang terlama.
            // Memakai max di sini akan melaporkan langkah muat padahal tidak,
            // dan gejalanya di arena cuma capit menutup sebelum sampai.
            float d[ARM_N_DEPAN];
            for (int id = 0; id < ARM_N_DEPAN; id++)
                d[id] = fabsf((float)pul[k][id] - (float)pul[k-1][id]) / usPerDer;
            const float tahapBahu = (d[0] > d[3]) ? d[0] : d[3];   // grip searah bahu
            const float der = tahapBahu + d[1] + d[2];
            const float laju  = urut[k].lipat ? LENGAN_SLEW_REHAT_DEG_S : ARM_SLEW_DEG_S;
            const float jatah = (float)LENGAN_JEDA_MS * (urut[k].lipat ? 2.0f : 1.0f);
            msPerlu = der / laju * 1000.0f;
            printf("    %-7s -> %-7s %5.1f der pada %3.0f der/s = %4.0f ms, jatah %4.0f ms%s\n",
                   urut[k-1].nama, urut[k].nama, (double)der, (double)laju,
                   (double)msPerlu, (double)jatah, (msPerlu <= jatah) ? "" : "   << TIDAK MUAT");
            fflush(stdout);
            assert(msPerlu <= jatah);
        }
    }

    // Gerbangnya juga harus di atas FRONT_STOP_CM, kalau tidak navigasi
    // berhenti sendiri sebelum misi sempat memutuskan. Aturan yang sama
    // dijaga cek_tabel_misi.py untuk seluruh baris HNT_DEPAN; di sini ia
    // diperiksa terhadap angka yang dipakai LENGAN.
    assert(KORBAN_JARAK_CM > FRONT_STOP_CM);
    assert(KORBAN_JARAK_CM < LIDAR_MAX_CM);

    // 'as<bahu> <siku> <prg>' menembak ketiga sendi langsung supaya operator
    // bisa membidik pose dengan tangan lalu MENULISKANNYA KERAS. Janjinya:
    // sudut yang sama menghasilkan pulsa yang sama lewat kedua jalur. Itu
    // benar hanya selama setSudutLengan() memakai id sendi dan ARM_BASE_*
    // yang sama dengan moveArmGrip(); mengubah salah satunya saja membuat
    // pose yang sudah ditulis keras meleset TANPA satu pun pesan galat.
    {
        const float jkn = KORBAN_CAPIT_MM, tgi = -BADAN[0] + KORBAN_TINGGI_MM;
        const float t = deg2rad(KORBAN_TAPAK_DER);
        float bahu = 0.0f, siku = 0.0f;
        const bool adaIK = ArmInverse::solve(
            jkn - HAND_LENGTH * cosf(t) - fabsf(ARM_ORIGINS[ARM_DEPAN][1]),
            tgi - HAND_LENGTH * sinf(t) - ARM_ORIGINS[ARM_DEPAN][2], bahu, siku);
        assert(adaIK);
        const float prg = KORBAN_TAPAK_DER - (bahu + siku);

        assert(robot.moveArmGrip(ARM_DEPAN, jkn, tgi, KORBAN_TAPAK_DER));
        uint16_t lewatIK[3];
        for (int i = 0; i < 3; i++) lewatIK[i] = robot.armDepan()->targetPulse((uint8_t)i);

        float sv[3];
        assert(robot.setSudutLengan(ARM_DEPAN, bahu, siku, prg, sv));
        for (int i = 0; i < 3; i++)
            assert(robot.armDepan()->targetPulse((uint8_t)i) == lewatIK[i]);
        printf("\n  as vs IK di pose jepit: bahu %.1f siku %.1f prg %.1f der -> pulsa SAMA\n",
               (double)bahu, (double)siku, (double)prg);

        // servoOut ada khusus supaya clamp diam-diam angleToPulse() bisa
        // KETAHUAN. Kalau ia berhenti melaporkan, perintahnya diam lagi.
        float sv2[3];
        assert(!robot.setSudutLengan(ARM_DEPAN, -ARM_BASE_BAHU - 10.0f, 0.0f, 0.0f, sv2));
        assert(sv2[0] < 0.0f);
        assert(!robot.setSudutLengan(ARM_BELAKANG, 0.0f, 0.0f, 0.0f));
        printf("  sendi di luar 0..180 dilaporkan: ya (bahu %.1f der servo)\n", (double)sv2[0]);
    }

    printf("\n  pose bermasalah: %d\n", buruk);
    assert(buruk == 0);
#if !LENGAN_KORBAN_AKTIF
    printf("  CATATAN: LENGAN_KORBAN_AKTIF 0 -- geometrinya sah, tapi ruas\n"
           "           AMBIL/TARUH sedang dilewati; tidak ada yang dijalankan.\n");
#endif
    printf("cek_korban: LOLOS\n");
    return 0;
}
