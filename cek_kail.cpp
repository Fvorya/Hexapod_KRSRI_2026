// Profil KAIL memindahkan kaki depan ke DEPAN dan ke ATAS, dan kaki belakang
// ke ATAS. Pertanyaan yang harus dijawab SEBELUM robot mencobanya di tangga:
// apakah kakinya benar-benar SAMPAI ke sana?
//
// Femur 80 + tibia 90 = 170 mm dari pangkal femur; itu batas kerasnya, dan IK
// melaporkan sendiri kalau pose diminta di luar jangkauan
// (Hexapod::lastPoseInRange()). Yang diuji di sini bukan bentuk diamnya saja
// melainkan SATU SIKLUS GAIT PENUH: ayunan menambah stepHeight 75 mm ke atas
// dan stepLength +-70 mm ke depan/belakang di atas offset kail, dan justru
// puncak ayunan itu yang paling dulu mentok.
//
//   g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited \
//       ../test-pc/stub/stubdefs.cpp Hexapod_Unlimited/{Hexapod,HexaGait,\
//       HexaServos,HexaArm,Calib,ArmInverse,LegInverseKinematics}.cpp \
//       cek_kail.cpp -o /tmp/cek_kail && /tmp/cek_kail
#include <Arduino.h>
#include <cassert>
#include <cstdio>
#include <cmath>
#include "Calib.h"
#include "Hexapod.h"

static Hexapod robot;

// Satu siklus gait penuh sambil berjalan maju. true = tidak pernah mentok.
static bool jalanSesiklus() {
    bool aman = true;
    robot.walk(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 120; i++) {              // 2,4 detik > siklus TANGGA 1,1 s
        __nowMs += 20; robot.update();
        if (!robot.lastPoseInRange()) aman = false;
    }
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 40; i++) { __nowMs += 20; robot.update(); }
    return aman;
}

// Bentuk kail dengan angka bebas, supaya batasnya bisa DICARI, bukan ditebak.
// Menyalin isi profileKail() dengan konstanta yang bisa disetel.
// turunBelakang positif = kaki belakang MEMANJANG KE BAWAH.
static bool coba(float badan, float maju, float naikDepan, float turunBelakang) {
    robot.setGaitProfile({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 10.0f,
                           GAIT_CYCLE_TIME + 200.0f, badan, KAIL_RADIUS_KAKI });
    Vec3 off[6] = {};
    off[0] = off[5] = { 0.0f, maju, naikDepan };
    off[2] = off[3] = { 0.0f, 0.0f, -turunBelakang };
    robot.setOffsetKaki(off);
    for (int i = 0; i < 120; i++) { __nowMs += 20; robot.update(); }   // tunggu ramp
    return jalanSesiklus();
}

// Sudut bidang yang masih bisa dilalui dengan badan DATAR, dari selisih
// tinggi kaki depan dan belakang. 156 mm = jarak pangkal kaki depan ke
// pangkal kaki belakang (config.h: y = +78 dan -78).
static float derajat(float naikDepan, float turunBelakang) {
    return atanf((naikDepan + turunBelakang) / 156.0f) * 57.29578f;
}

// Selubung lintasan telapak SATU kaki selama satu siklus, di FRAME KAKI.
// Frame kaki cuma diputar terhadap sumbu z, jadi z-nya sama dengan z frame
// badan: angka "angkat" di bawah langsung terbaca sebagai tinggi angkat.
struct Selubung { float zMin, zMaks, hMin, hMaks, lututMin, lututMaks; };

// Kinematika MAJU, kebalikan persis LegInverseKinematics::solve(). Dipakai
// karena dengan lutut terkunci telapak TIDAK berada di titik yang diminta
// gait -- satu-satunya cara mengetahui letaknya adalah menghitungnya dari
// sudut yang benar-benar dikirim.
static void telapakDariSudut(float coxaDeg, float femurDeg, float lututDeg,
                             float& horiz, float& z) {
    const float F = FEMUR_LENGTH, T = TIBIA_LENGTH;
    float D  = sqrtf(F*F + T*T - 2.0f*F*T*cosf(deg2rad(lututDeg)));
    float a2 = acosf(clampf((F*F + D*D - T*T) / (2.0f*F*D), -1.0f, 1.0f));
    float a1 = deg2rad(femurDeg) - a2;
    horiz = COXA_LENGTH + D * cosf(a1);
    z     = D * sinf(a1);
    (void)coxaDeg;   // arah horizontal tidak dipakai selubung ini
}

// Tinggi telapak kaki depan saat robot BERDIRI DIAM. Inilah pose yang harus
// sama persis dengan atau tanpa kunci -- bukan titik terendah selama siklus.
static float tapakDiam() {
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 60; i++) { __nowMs += 20; robot.update(); }
    float c, f, t, horiz, z;
    robot.legAngles(0, c, f, t);
    telapakDariSudut(c, f, t, horiz, z);
    return z;
}

// Ayunan coxa satu kaki selama satu siklus jalan maju (der), dan sekalian
// jangkauan D terbesar yang dipakainya (mm; batas keras FEMUR+TIBIA-1 = 169).
static float dMaksSiklus = 0.0f;

static float ayunCoxa(int kaki) {
    float lo = 1e9f, hi = -1e9f;
    dMaksSiklus = 0.0f;
    robot.walk(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 150; i++) {
        __nowMs += 20; robot.update();
        float c, f, t;
        robot.legAngles(kaki, c, f, t);
        if (c < lo) lo = c;
        if (c > hi) hi = c;
        const float F = FEMUR_LENGTH, T = TIBIA_LENGTH;
        const float d = sqrtf(F*F + T*T - 2.0f*F*T*cosf(deg2rad(t)));
        if (d > dMaksSiklus) dMaksSiklus = d;
    }
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 40; i++) { __nowMs += 20; robot.update(); }
    return hi - lo;
}

// --- POLIGON TUMPUAN -------------------------------------------------
// Letak telapak di kerangka BADAN, dibaca balik dari sudut servo supaya ia
// benar-benar mengukur apa yang dikirim ke servo, bukan menghitung ulang
// niat profileKail(). legAngles() memberi coxa RELATIF terhadap arah hadap
// kaki, jadi arah mutlaknya BODY_LEG_ANGLE + coxa.
static void telapakXY(int kaki, float& x, float& y) {
    float c, f, t, horiz, z;
    robot.legAngles(kaki, c, f, t);
    telapakDariSudut(c, f, t, horiz, z);
    const float a = deg2rad(BODY_LEG_ANGLE[kaki] + c);
    x = BODY_LEG_ORIGINS[kaki][0] + horiz * cosf(a);
    y = BODY_LEG_ORIGINS[kaki][1] + horiz * sinf(a);
}

// Jarak titik berat ke sisi TERDEKAT segitiga tumpuan, mm. Negatif = CG di
// luar poligon = terguling. Diambil yang terburuk dari kedua tripod.
static float marginGuling(float cgY) {
    const int tripod[2][3] = { {0,2,4}, {1,3,5} };
    float buruk = 1e9f;
    for (int g = 0; g < 2; g++) {
        float px[3], py[3];
        for (int k = 0; k < 3; k++) telapakXY(tripod[g][k], px[k], py[k]);
        float luas = 0.0f;
        for (int k = 0; k < 3; k++) {
            int n = (k + 1) % 3;
            luas += px[k]*py[n] - px[n]*py[k];
        }
        const float arah = (luas > 0.0f) ? 1.0f : -1.0f;
        for (int k = 0; k < 3; k++) {
            int n = (k + 1) % 3;
            const float dx = px[n]-px[k], dy = py[n]-py[k];
            const float d = arah * (dx*(cgY-py[k]) - dy*(0.0f-px[k])) / hypotf(dx, dy);
            if (d < buruk) buruk = d;
        }
    }
    return buruk;
}

static Selubung jejakDepan() {
    Selubung s = { 1e9f, -1e9f, 1e9f, -1e9f, 1e9f, -1e9f };
    robot.walk(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < 120; i++) {
        __nowMs += 20; robot.update();
        float c, f, t;
        robot.legAngles(0, c, f, t);
        float horiz, z;
        telapakDariSudut(c, f, t, horiz, z);
        if (z < s.zMin)         s.zMin = z;
        if (z > s.zMaks)        s.zMaks = z;
        if (horiz < s.hMin)     s.hMin = horiz;
        if (horiz > s.hMaks)    s.hMaks = horiz;
        if (t < s.lututMin)     s.lututMin = t;
        if (t > s.lututMaks)    s.lututMaks = t;
    }
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 40; i++) { __nowMs += 20; robot.update(); }
    return s;
}

int main() {
    Calib::applyDefaults();
    robot.begin(); robot.arm();
    for (int i = 0; i < 100; i++) { __nowMs += 20; robot.update(); }

    // 0) Profil biasa harus lolos -- kalau tidak, ujian ini yang rusak.
    robot.profileStairs();
    for (int i = 0; i < 100; i++) { __nowMs += 20; robot.update(); }
    assert(jalanSesiklus());
    printf("  TANGGA polos            : dalam jangkauan\n");

    // 1) Angka yang benar-benar dipakai profil KAIL.
    robot.profileKail();
    for (int i = 0; i < 120; i++) { __nowMs += 20; robot.update(); }
    bool ok = jalanSesiklus();
    printf("  KAIL badan %.0f: depan +%.0f naik / %.0f maju, belakang -%.0f\n",
           (double)KAIL_TINGGI_BADAN, (double)KAIL_DEPAN_NAIK,
           (double)KAIL_DEPAN_MAJU, (double)KAIL_BELAKANG_TURUN);
    printf("    %s, badan tetap datar sampai bidang %.1f der\n",
           ok ? "dalam jangkauan" : "MENTOK",
           (double)derajat(KAIL_DEPAN_NAIK, KAIL_BELAKANG_TURUN));
    fflush(stdout);
    assert(ok);

    // Bidang R-9 terukur 27,3 der. Kompensasinya tidak harus penuh, tapi
    // harus lebih dari SETENGAHnya -- di bawah itu bentuk ini tidak menjawab
    // soalnya sama sekali, dan lebih baik ketahuan di sini.
    assert(derajat(KAIL_DEPAN_NAIK, KAIL_BELAKANG_TURUN) > 13.7f);

    // 1a) KAKI BELAKANG LURUS KE BELAKANG -- coxa harus BENAR-BENAR diam.
    // Telapak belakang menahan badan yang mendongak; coxa yang mengayuh di
    // bawah beban itu menyeret telapak menyamping, dan seretan itu tidak
    // muncul di mana pun kecuali sebagai selip di tangga. Ia nol hanya kalau
    // telapak duduk tepat di garis x lokal = 0 -- satu saja offset belakang
    // di profileKail() berubah, angka ini langsung naik lagi.
    const float ayunBlk = ayunCoxa(2);
    printf("    coxa belakang mengayuh  : %4.1f der per siklus (TANGGA polos: ", (double)ayunBlk);
    robot.profileStairs();
    for (int i = 0; i < 120; i++) { __nowMs += 20; robot.update(); }
    const float ayunTangga = ayunCoxa(2);
    printf("%.1f)\n", (double)ayunTangga);
    robot.profileKail();
    for (int i = 0; i < 120; i++) { __nowMs += 20; robot.update(); }
    // Diluruskan, coxa belakang benar-benar 0. KAIL_BELAKANG_LEBAR sengaja
    // mengembalikan sebagian demi jejak yang tidak menyatu, jadi yang dijaga
    // bukan lagi nol melainkan bahwa pelurusannya MASIH berarti: kurang dari
    // separuh ayunan profil biasa. Relatif, bukan angka mati, supaya ia tetap
    // bermakna kalau panjang langkah disetel operator lewat calib.
    assert(ayunBlk < 0.5f * ayunTangga);
    assert(ayunTangga > 5.0f);     // pembanding: profil lain memang mengayuh

    // 1b) DUA ONGKOS yang gampang terlanggar sambil menyetel bentuk kail.
    //
    // Kaki TENGAH mengayuh paling jauh: seluruh langkah tegak lurus arah
    // hadapnya. Knopnya KAIL_TENGAH_KELUAR, dan ia murah -- tapi tiap 10 mm
    // melebarkan robot 20 mm, jadi batas sebenarnya ada di lebar lorong R-9,
    // bukan di sini. Yang dijaga di sini cuma bahwa ayunannya tidak diam-diam
    // balik ke 60 der waktu knop lain digeser.
    const float ayunTgh = ayunCoxa(1);
    printf("    coxa tengah mengayuh    : %4.1f der per siklus\n", (double)ayunTgh);
    assert(ayunTgh < 45.0f);

    // Kaki BELAKANG: jangkauan = radius + KAIL_BELAKANG_MUNDUR + setengah
    // langkah. Di 100% IK meng-clamp DIAM-DIAM lewat Dmax, dan kaki yang
    // hampir lurus paling lemah justru ke arah radial -- arah ia mendorong
    // badan naik. Ambang 97% menangkapnya SEBELUM jadi clamp.
    ayunCoxa(2);
    printf("    jangkauan kaki belakang : %.0f mm dari 169 (%.0f%%)\n",
           (double)dMaksSiklus, (double)(100.0f * dMaksSiklus / 169.0f));
    assert(dMaksSiklus < 0.97f * 169.0f);

    // 1c) POLIGON TUMPUAN. Inilah yang dibeli KAIL_BELAKANG_LEBAR dan
    // KAIL_TENGAH_SUDUT, dan satu-satunya hal yang tidak muncul di IK sama
    // sekali: IK dengan senang hati menaruh keenam telapak di satu garis.
    // CG -73 mm adalah geseran akibat lereng 27,7 der (tinggi CG * tan),
    // yaitu keadaan R-9 yang sesungguhnya, bukan robot yang berdiri datar.
    robot.walk(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 60; i++) { __nowMs += 20; robot.update(); }
    printf("    margin guling           : %.0f mm (CG di pusat)  %.0f mm (CG -73 = lereng)\n",
           (double)marginGuling(0.0f), (double)marginGuling(-73.0f));
    //
    // AMBANGNYA LANTAI, BUKAN SASARAN. Syarat fisiknya cuma > 0 (di bawah itu
    // robot terguling), tapi ia berjalan: telapak terangkat bergantian, badan
    // bergoyang, dan geseran CG -73 mm itu sendiri +-20 mm karena TINGGI CG
    // belum pernah ditimbang. 30 mm = seperlima setengah-jejak kaki tengah,
    // dipakai sebagai jaring pengaman terhadap penyetelan yang kebablasan.
    assert(marginGuling(0.0f)   > 55.0f);
    assert(marginGuling(-73.0f) > 30.0f);

    // 1b) LUTUT KAKI DEPAN DIKUNCI -- apa yang sebenarnya dibayar?
    //
    // Operator melaporkan kaki depan terpeleset di tangga dan menduga lutut
    // yang mengayuh penyebabnya: tiap derajat ayunan menggeser titik sentuh
    // ujung kaki. KAIL_LUTUT_KUNCI membekukannya. Tapi lutut beku berarti
    // telapak TIDAK LAGI menuruti lintasan yang diminta gait -- ia jatuh ke
    // busur berjari-jari tetap. Pertanyaan yang harus dijawab SEBELUM tangga:
    // kaki depan masih terangkat berapa? Kalau angkatnya hilang, kunci ini
    // justru MENAMBAH selip, bukan menguranginya.
    //
    // Keduanya memakai bentuk KAIL yang sama. Yang membedakan cuma pintunya:
    // setGaitProfile() menghapus kunci (lihat Hexapod::pasangProfil), jadi ia
    // baseline lutut-bebas; profileKail() yang memasangnya.
    {
        Selubung bebas, kunci;
        float diamBebas, diamKunci;
        {
            robot.setGaitProfile({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 10.0f,
                                   GAIT_CYCLE_TIME + 200.0f, KAIL_TINGGI_BADAN,
                                   KAIL_RADIUS_KAKI });
            Vec3 off[6] = {};
            off[0] = off[5] = { 0.0f, KAIL_DEPAN_MAJU, KAIL_DEPAN_NAIK };
            off[2] = off[3] = { 0.0f, 0.0f,           -KAIL_BELAKANG_TURUN };
            robot.setOffsetKaki(off);
            for (int i = 0; i < 150; i++) { __nowMs += 20; robot.update(); }
            diamBebas = tapakDiam();
            bebas = jejakDepan();
        }
        robot.profileKail();
        for (int i = 0; i < 150; i++) { __nowMs += 20; robot.update(); }
        diamKunci = tapakDiam();
        kunci = jejakDepan();

        printf("\n  kaki depan (kaki 0), satu siklus penuh:\n");
        printf("    lutut bebas  : angkat %5.1f mm, julur %5.1f mm, lutut mengayuh %5.1f der\n",
               (double)(bebas.zMaks - bebas.zMin), (double)(bebas.hMaks - bebas.hMin),
               (double)(bebas.lututMaks - bebas.lututMin));
        printf("    lutut TERKUNCI: angkat %5.1f mm, julur %5.1f mm, lutut mengayuh %5.1f der\n",
               (double)(kunci.zMaks - kunci.zMin), (double)(kunci.hMaks - kunci.hMin),
               (double)(kunci.lututMaks - kunci.lututMin));
        printf("    pose DIAM     : z %.1f mm (bebas) vs %.1f mm (terkunci)\n",
               (double)diamBebas, (double)diamKunci);
        printf("    MENEKAN KE BAWAH selama stance: %.1f mm (bebas) vs %.1f mm (terkunci)\n",
               (double)(diamBebas - bebas.zMin), (double)(diamKunci - kunci.zMin));
        fflush(stdout);

#if KAIL_LUTUT_KUNCI
        // Kuncinya benar-benar terpasang. Inilah yang gagal duluan kalau
        // pasangProfil() atau urutan di profileKail() rusak.
        assert(kunci.lututMaks - kunci.lututMin < 0.5f);
        assert(bebas.lututMaks - bebas.lututMin > 5.0f);   // ada yang dicegah

        // Dan kaki masih terangkat. Ambang 30 mm bukan selera: tinggi anak
        // tangga arena 40 mm, dan telapak yang terangkat kurang dari itu
        // tidak bisa menyeberanginya sama sekali -- persoalan yang lebih
        // besar daripada selip yang hendak diperbaiki kunci ini.
        assert(kunci.zMaks - kunci.zMin > 30.0f);
#else
        assert(kunci.lututMaks - kunci.lututMin > 5.0f);   // kunci memang mati
#endif
        // Pose DIAMnya tidak boleh bergeser: sudut kunci diambil dari pose
        // netral kaki itu sendiri, jadi bentuk berdiri KAIL harus sama persis
        // dengan IK biasa. Kalau ini meleset, sudut kuncinya diambil dari
        // pose yang salah.
        assert(fabsf(diamBebas - diamKunci) < 2.0f);

        // HARGA KUNCI, dan inilah temuan yang paling penting di berkas ini.
        // Langkah stance kaki DEPAN sebagian besar RADIAL: kaki 0 menghadap
        // 60 der, jadi 70 mm ayunan badan ke belakang menuntut julur kaki
        // berubah ~60 mm. Julur itu persis yang TIDAK bisa diberikan lutut
        // beku -- telapak terkunci di satu jari-jari, jadi selisihnya keluar
        // sebagai gerak TEGAK: telapak menekan ke bawah di tengah stance.
        //
        // Artinya ayunan lutut pada IK tiga sendi bukan cacat; itulah yang
        // menjaga telapak tetap di garis lurus. Membekukannya menukar
        // gesekan beberapa mm di ujung kaki dengan tekanan belasan mm --
        // menaikkan badan, meringankan kaki lain, dan itu justru penyebab
        // selip. Angkanya dicetak di atas; putuskan dengan angka itu.
        //
        // Ambangnya menjaga supaya keadaan tidak diam-diam memburuk kalau
        // panjang langkah atau stance KAIL diubah lagi.
        assert(diamKunci - kunci.zMin < 25.0f);
        assert(diamBebas - bebas.zMin < 2.0f);   // IK tiga sendi memang rata
    }

    // 2) Sampai berapa jauh kaki depan boleh membuka?
    //    CATATAN sesudah KAIL_LUTUT_KUNCI: sapuan ini memakai coba(), yang
    //    lewat setGaitProfile() -- jadi ia mengukur batas IK dengan lutut
    //    BEBAS. Dengan lutut terkunci kaki depan tidak pernah "di luar
    //    jangkauan" (lihat LegInverseKinematics), jadi angka di bawah ini
    //    lebih ketat daripada yang sebenarnya berlaku. Itu arah yang aman,
    //    dan ia tetap benar untuk pose BERDIRInya -- yang harus terjangkau
    //    IK biasa, karena dari situlah sudut kunci diambil.
    printf("\n  batas buka kaki depan (belakang turun %.0f mm):\n",
           (double)KAIL_BELAKANG_TURUN);
    for (float naik = 20.0f; naik <= 80.0f; naik += 20.0f) {
        float maksMaju = 0.0f;
        for (float maju = 0.0f; maju <= 120.0f; maju += 10.0f)
            if (coba(KAIL_TINGGI_BADAN, maju, naik, KAIL_BELAKANG_TURUN)) maksMaju = maju;
        printf("    naik %3.0f mm -> maju sampai %3.0f mm\n",
               (double)naik, (double)maksMaju);
    }

    // 3) Sampai berapa jauh kaki BELAKANG boleh memanjang, dan bagaimana ia
    //    ditukar dengan tinggi badan. Inilah yang menentukan sudut bidang
    //    yang masih bisa dilalui dengan badan DATAR -- pertukarannya 1:1,
    //    tiap mm badan naik memakan satu mm jangkauan ke bawah.
    printf("\n  jangkauan kaki belakang ke bawah vs tinggi badan:\n");
    float maksDipakai = -1.0f;
    for (float badan = 90.0f; badan <= 115.0f; badan += 5.0f) {
        float maks = 0.0f;
        for (float turun = 0.0f; turun <= 80.0f; turun += 5.0f)
            if (coba(badan, KAIL_DEPAN_MAJU, KAIL_DEPAN_NAIK, turun)) maks = turun;
        if (badan == KAIL_TINGGI_BADAN) maksDipakai = maks;
        printf("    badan %3.0f mm -> turun sampai %2.0f mm (bidang %4.1f der)%s\n",
               (double)badan, (double)maks, (double)derajat(KAIL_DEPAN_NAIK, maks),
               (badan == KAIL_TINGGI_BADAN) ? "  <-- yang dipakai" : "");
    }

    // Langit-langitnya DIUKUR di sapuan barusan, bukan angka tetap. Versi lama
    // menulis "< 45" -- benar untuk badan 100 mm pada stance 70, lalu diam-diam
    // salah begitu KAIL_RADIUS_KAKI dipersempit ke 60 dan langitnya naik ke 50.
    // Kelegaan 5 mm: sapuannya sendiri berlangkah 5 mm, jadi nilai lolos
    // terakhir sudah menempel di langit-langit.
    printf("    -> badan %.0f mm: langit %.0f mm, terpakai %.0f mm\n",
           (double)KAIL_TINGGI_BADAN, (double)maksDipakai, (double)KAIL_BELAKANG_TURUN);
    fflush(stdout);
    assert(maksDipakai > 0.0f);                          // sapuannya benar jalan
    assert(KAIL_BELAKANG_TURUN <= maksDipakai - 5.0f);   // sisakan kelegaan

    printf("cek_kail: LOLOS\n");
    return 0;
}
