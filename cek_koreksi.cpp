// Uji keputusan "berhenti dulu, baru koreksi" (NavKoreksi.h) memakai ambang
// yang BENAR-BENAR dipakai firmware, diambil dari config.h.
//
// Berkas sendiri, bukan tambahan di cek_geser.cpp: cek_geser me-link seluruh
// Hexapod_Unlimited/*.cpp, dan LidarArray.cpp menuntut pinMode/digitalRead/
// INPUT_PULLUP yang belum ada di ../test-pc/stub. Sampai stub itu dilengkapi,
// cek_geser tidak bisa dibangun di PC Windows ini sama sekali. NavKoreksi.h
// sengaja tidak bergantung apa pun selain math.h supaya bagian yang paling
// mudah salah -- tanda dan histeresis -- tetap bisa diuji tanpa robot.
//
//     g++ -std=gnu++17 -I../test-pc/stub -IHexapod_Unlimited -o cek_koreksi
//         cek_koreksi.cpp && ./cek_koreksi
#include "config.h"
#include "NavKoreksi.h"
#include <cassert>
#include <cmath>
#include <cstdio>

// wall.setpoint; nilainya di Calib.cpp (PARAM_DEFS), bukan config.h, jadi
// disebut tegas di sini -- dan HARUS diikutkan kalau bawaannya disetel.
// 19 -> 16 pada 18 Sep 2026: bawaan Calib sudah 16 sejak 7 Sep (itulah angka
// yang benar-benar dipakai operator di arena), jadi uji ini sempat mengukur
// setpoint yang tidak lagi dipakai robot. Seluruh assert di bawah relatif
// terhadap SP, jadi yang berubah cuma titik acuannya.
static const float SP = 16.0f;

static float bidik(float jarak) {
    return navSudutBidik(jarak, SP, NAV_KOREKSI_JARAK_K, NAV_KOREKSI_BIDIK_MAKS);
}

static bool perlu(float phi, float jarak) {
    return navPerluKoreksi(phi, bidik(jarak), jarak, SP,
                           NAV_KOREKSI_SUDUT_DEG, NAV_KOREKSI_JARAK_CM);
}

int main() {
    printf("ambang: sudut %.0f der, jarak %.0f cm, keluar %.0f der\n",
           (double)NAV_KOREKSI_SUDUT_DEG, (double)NAV_KOREKSI_JARAK_CM,
           (double)NAV_KOREKSI_KELUAR_DEG);
    printf("bidik : %.1f der/cm, batas %.0f der, setpoint %.0f cm\n\n",
           (double)NAV_KOREKSI_JARAK_K, (double)NAV_KOREKSI_BIDIK_MAKS,
           (double)SP);

    // --- SUDUT BIDIK ---
    // Jarak pas -> tidak perlu menyerong sama sekali.
    assert(fabsf(bidik(SP)) < 0.001f);

    // TANDA. sudutDinding() positif = hidung menyerong KE ARAH dinding. Robot
    // yang KEJAUHAN harus mendekat, jadi bidiknya POSITIF. Tanda terbalik di
    // sini membuat robot menjauh dari dinding tiap kali ia kejauhan -- gagal
    // yang tidak pernah terlihat di kompilasi.
    assert(bidik(SP + 4.0f) > 0.0f);
    assert(bidik(SP - 4.0f) < 0.0f);

    // Besarnya linear sebelum ter-clamp.
    assert(fabsf(bidik(SP + 4.0f) - 4.0f * NAV_KOREKSI_JARAK_K) < 0.001f);

    // TER-CLAMP di kedua ujung. Tanpa batas, dinding yang hilang lalu muncul
    // di 60 cm membuat robot membidik 20 der -- menghadap dinding, bukan
    // menyusurinya.
    assert(fabsf(bidik(SP + 100.0f) - NAV_KOREKSI_BIDIK_MAKS) < 0.001f);
    assert(fabsf(bidik(SP - 100.0f) + NAV_KOREKSI_BIDIK_MAKS) < 0.001f);

    // Sensor bisu -> 0, bukan NaN yang menular ke seluruh perbandingan.
    assert(bidik(NAN) == 0.0f);

    // --- PERLU KOREKSI ---
    // Sejajar dan jaraknya pas -> jalan terus.
    assert(!perlu(0.0f, SP));

    // Di dalam kedua pita -> jalan terus. 2 der dan 2 cm dua-duanya di bawah
    // ambang, dan bidik 2 cm = 1 der, jadi sisa sudutnya 1 der.
    assert(!perlu(1.0f, SP + 2.0f));

    // Sudut lewat ambang -> berhenti.
    assert(perlu(NAV_KOREKSI_SUDUT_DEG + 1.0f, SP));
    assert(perlu(-(NAV_KOREKSI_SUDUT_DEG + 1.0f), SP));

    // Jarak lewat ambang -> berhenti, walau badannya sudah membidik benar.
    assert(perlu(bidik(SP + 6.0f), SP + 6.0f));

    // GALAT DIUKUR TERHADAP BIDIK, bukan terhadap nol. Robot yang sengaja
    // menyerong 8 der untuk menutup jarak 16 cm TIDAK sedang salah arah --
    // menghentikannya di situ membuat koreksi jarak tidak pernah selesai.
    // Jaraknya sendiri yang masih memicu, dan itu memang benar.
    {
        const float jauh = SP + 16.0f;
        assert(fabsf(bidik(jauh) - NAV_KOREKSI_BIDIK_MAKS) < 0.001f);
        assert(fabsf(bidik(jauh) - bidik(jauh)) <= NAV_KOREKSI_SUDUT_DEG);
    }

    // Sensor sudut bisu -> JANGAN berhenti. Lup kemudi biasa sudah punya
    // penanganan dinding hilang; berhenti di sini membekukan robot di
    // tanjakan justru saat sensornya sedang bermasalah.
    assert(!perlu(NAN, SP));
    assert(!perlu(NAN, SP + 20.0f));

    // --- SELESAI / HISTERESIS ---
    // Keluar lebih ketat daripada masuk: yang baru saja memicu masuk TIDAK
    // boleh langsung dianggap selesai, kalau tidak robot kedip berhenti-jalan
    // di ambangnya.
    assert(NAV_KOREKSI_KELUAR_DEG < NAV_KOREKSI_SUDUT_DEG);
    assert(!navKoreksiSelesai(NAV_KOREKSI_SUDUT_DEG + 1.0f, 0.0f,
                              NAV_KOREKSI_KELUAR_DEG));
    assert(navKoreksiSelesai(NAV_KOREKSI_KELUAR_DEG - 0.5f, 0.0f,
                             NAV_KOREKSI_KELUAR_DEG));

    // Selesai diukur terhadap BIDIK juga.
    assert(navKoreksiSelesai(8.0f, 8.0f, NAV_KOREKSI_KELUAR_DEG));
    assert(!navKoreksiSelesai(0.0f, 8.0f, NAV_KOREKSI_KELUAR_DEG));

    // Sensor hilang di TENGAH koreksi -> lepaskan, jangan membeku.
    assert(navKoreksiSelesai(NAN, 0.0f, NAV_KOREKSI_KELUAR_DEG));

    // --- JANGKAUAN UKUR ---
    // sudutDinding() menolak |beda| di atas SISI_BEDA_MAKS_CM, jadi sudut yang
    // bisa dibaca terbatas. Ambang masuk HARUS di dalamnya, kalau tidak ia
    // tidak pernah menyala dan koreksinya cuma pajangan.
    {
        const float sudutMaks = atanf(SISI_BEDA_MAKS_CM / WALL_BASE_CM) * 57.2957795f;
        printf("sudut terukur maksimum %.1f der\n", (double)sudutMaks);
        assert(NAV_KOREKSI_SUDUT_DEG < sudutMaks);
        assert(NAV_KOREKSI_BIDIK_MAKS < sudutMaks);
    }

    printf("\nSemua pemeriksaan lolos.\n");
    return 0;
}
