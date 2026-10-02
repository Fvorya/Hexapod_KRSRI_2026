#include "Skor.h"

// Pembukuan poin milik .ino; di sini cuma diberi tahu ruas mana yang selesai.
extern Skor gSkor;
#include "Misi.h"

// ====================================================================
// TABEL LINTASAN -- Guidebook SAR UNLIMITED 2026, halaman 21-30.
//
// Urutan (hal. 21 & 34):
//   HOME -> K-1 -> R-1 -> R-2/R-3 -> R-4/SZ-1 -> K-2 -> R-5/SZ-2 -> R-6
//        -> (K-3, K-4 dimatikan) -> R-9 tangga -> R-10 -> K-5 -> R-11
//        -> SZ-5/FINISH
//
// ATURAN (hal. 21): K-1 harus DICOBA sebelum melewati batas R1-R2; kembali
// mengambilnya sesudah itu tidak sah.
//
// Tabel yang BERLAKU selalu 'm4' (sudah termasuk arah mutlak dan cermin).
// Ruas HNT_ODO diukur dengan 'm6 <idx>' lalu ditulis dengan 'm7 <idx> <cm>'.
// HNT_SISI/HNT_MUNDUR adalah jarak ke dinding, bukan tempuh -- coba 'V<cm>' /
// 'J<cm>' langsung lalu tulis angkanya.
//
// PELAJARAN YANG BERLAKU UNTUK SELURUH TABEL:
//   - MENGGANTI PROFIL MEMBATALKAN PANJANG RUASNYA. Langkah dan selip tiap
//     gait berbeda; angka yang diukur dengan profil lain tidak berlaku.
//   - abaikanDepan HANYA di ruas yang dibatasi ODOMETRI (tabelSiap()
//     menolak selainnya). Di turunan berkas depan menembak lantai; di
//     medan kasar halangan depan membuat mode arena pindah mata angin.
//   - Sensor depan pernah melihat hantu ~8,6 cm yang HILANG saat robot
//     berputar di titik yang sama -- ruas HNT_DEPAN bisa berhenti di tempat
//     yang salah tanpa gejala lain. Penyebabnya belum terjawab.
//   - Arah berangkat dari HOME ditentukan JURI, sedangkan
//     MISI_ARAH_BERANGKAT dipaku 0. Arah lain menggeser seluruh kolom arah;
//     kalibrasi kompas c0 = arah lorong pertama dari HOME.
//
// Ukuran arena: alas 3,6 x 2,4 m, lorong 45 cm, dinding 2 cm tebal / 10 cm
// tinggi (pagar akal sehat MISI_RUAS_MAKS_CM). R-11: jalan yang bisa
// dilalui kaki selebar 30 cm. R-9 tangga 90 horizontal / 50 vertikal
// (miring 103), anak tangga 3,6 cm. M1 80 horizontal / 20 vertikal (14 der).
//
// Skor untuk menimbang risiko: R-9 bernilai 150 tanpa korban, 300 dengan
// korban -- rintangan termahal di arena.
// ====================================================================
// >>> TABEL LINTASAN BAKU: acuan di FLASH, bukan yang dijalankan.
// Yang dijalankan misi adalah salinannya di RAM (RUAS[] di bawah), yang boleh
// diubah dari HUD lewat 'm5s'. 'm5r' memulangkan RAM ke tabel ini.
const Ruas RUAS_BAKU[] = {
//         nama                             belok      kemudi      profil        buta   henti          nilai            aksi           lengan    pivot
/*0*/    { "HOME -> samping K-1",           BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_BELAKANG,  33,              AKS_TIDAK_ADA, 0 },
/*1*/    { "mundur jika bisa",              BLK_KIRI,  KMD_TENGAH, PRF_DATAR,    false, HNT_MUNDUR,     9,              AKS_TIDAK_ADA, 0 },
/*2*/    { "K-1 angkat korban",             BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,  0.0f, false, false, 35.0f, 30.0f },
/*3*/    { "R-1 jalan pecah",               BLK_KANAN, KMD_KIRI,   PRF_TANGGA,   true,  HNT_ODO,       96,              AKS_TIDAK_ADA, 0 },
/*4*/    { "M1 turunan + R-2/R-3",          BLK_LURUS, KMD_KANAN,  PRF_DATAR,    true,  HNT_ODO,       64,              AKS_TIDAK_ADA, 0 },
/*5*/    { "Depan sampai 30 cm",            BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_DEPAN,     30,              AKS_TIDAK_ADA, 0 },
/*6*/    { "SZ-1 taruh korban (dalam R-4)", BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN, -20.0f },
/*7*/    { "pivot kiri ratakan dinding",    BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_DEPAN,     15,              AKS_TIDAK_ADA, 0,         +20.0f },
/*8*/    { "hadap kiri maju",               BLK_KIRI,  KMD_KANAN,  PRF_TANGGA,   false, HNT_ODO,       38,              AKS_TIDAK_ADA, 0 },
/*9*/    { "mundur jika bisa",              BLK_KIRI,  KMD_TENGAH, PRF_TANGGA,   false, HNT_MUNDUR,    12,              AKS_TIDAK_ADA, 0 },
/*10*/   { "K-2 angkat korban",             BLK_LURUS, KMD_TENGAH, PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,   0.0f, false, false, 35.0f, 30.0f }, 
/*11*/   { "R-5 lumpur: BARAT sampai ujung",BLK_KANAN, KMD_KANAN,  PRF_DATAR,    false, HNT_ODO,       18,              AKS_TIDAK_ADA, 0 },
/*12*/   { "SZ-2 taruh korban (kanan 20)",  BLK_LURUS, KMD_KANAN,  PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN, -20.0f },
/*13*/   { "SELATAN keluar R-5 (kiri 20)",  BLK_KIRI,  KMD_KANAN,  PRF_TANGGA,   false, HNT_ODO,       26,              AKS_TIDAK_ADA, 0,         +20.0f },
/*14*/   { "ratakan ke dinding KANAN",      BLK_LURUS, KMD_KANAN,  PRF_MERUNDUK, false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0,           0.0f, false, false },
/*15*/   { "SELATAN sampai tembok K-3",     BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_ODO,       32,              AKS_TIDAK_ADA, 0 },
/*16*/   { "lalu maju mentok",              BLK_LURUS, KMD_KIRI,   PRF_DATAR,    false, HNT_GESER,     30,              AKS_TIDAK_ADA, 0,           0.0f, false, false },
/*17*/   { "jalan sampai bebatuan",         BLK_LURUS, KMD_KIRI,   PRF_TANGGA,   true,  HNT_ODO,       86,              AKS_TIDAK_ADA, 0 },
/*18*/   { "kanan maju",                    BLK_KANAN, KMD_KIRI,   PRF_DATAR,    true,  HNT_ODO,       24,              AKS_TIDAK_ADA, 0 },
/*19*/   { "ratakan 13 cm dinding KANAN",   BLK_KIRI,  KMD_KANAN,  PRF_DATAR,    false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0 },
/*20*/   { "R-9 TANGGA (miring 103)",       BLK_LURUS, KMD_KANAN,  PRF_TANJAK,   true,  HNT_PUNCAK,     0,              AKS_TIDAK_ADA, 0 },
/*21*/   { "Maju sedikit naik tangga",      BLK_LURUS, KMD_KANAN,  PRF_TANJAK,   true,  HNT_ODO,       10,              AKS_TIDAK_ADA, 0 },
/*22*/   { "R-10 puing+lumpur miring",      BLK_LURUS, KMD_KANAN,  PRF_TANGGA,   true,  HNT_ODO,       15,              AKS_TIDAK_ADA, 0 },
/*23*/   { "jalan ke kanan depan K-5",      BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0 },
/*24*/   { "hadap kiri maju buta",          BLK_KIRI,  KMD_TENGAH, PRF_SEMPIT,    true, HNT_ODO,       40,              AKS_TIDAK_ADA, 0 },
/*25*/   { "baca lidar depan [fin]",        BLK_LURUS, KMD_TENGAH, PRF_SEMPIT,   false, HNT_DEPAN,     13,              AKS_TIDAK_ADA, 0 },
};

// <<< TABEL LINTASAN BAKU selesai

// --- TABEL YANG DIJALANKAN, di RAM ---
// Salinan RUAS_BAKU[], diisi tabelBaku() di konstruktor, boleh diubah HUD.
// Semua nama menunjuk ke kolam nama RAM sejak awal, supaya cuma ada satu
// jenis pointer di tabel.
Ruas RUAS[RUAS_MAKS];
static char NAMA_RAM[RUAS_MAKS][RUAS_NAMA_MAKS];

// JUMLAHNYA BOLEH BERUBAH lewat 'm5+' / 'm5-'. Peta poin di Skor.cpp ikut
// digeser di fungsi yang sama -- lihat skorSisip()/skorHapus().
uint8_t RUAS_N = 0;                  // diisi tabelBaku()
const uint8_t RUAS_BAKU_N = sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]);
static_assert(sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]) <= RUAS_MAKS,
              "Tabel lintasan lebih panjang dari _cm[] -- naikkan RUAS_MAKS di Misi.h");

// Plafon kedua, bukan soal RAM: RUAS_N, _i, _iAkhir semuanya uint8_t, dan
// _iAkhir memakai 255 sebagai penanda. Pada 256 baris RUAS_N terpotong jadi
// 0 diam-diam dan misi selesai tanpa menjalankan satu ruas pun.
static_assert(sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]) <= 254,
              "Tabel > 254 baris: RUAS_N/_i/_iAkhir uint8_t, dan 255 dipakai "
              "_iAkhir sebagai penanda. Lebarkan ketiganya ke uint16_t dulu.");

// ====================================================================
// AMBANG
// ====================================================================

// Berapa sampel LiDAR BERTURUT-TURUT sebelum pemicu jarak dipercaya.
// Kecil dengan sengaja: LidarArray sudah menyaring hantu lewat median-3 dan
// LIDAR_MIN_CM, jadi ini cuma lapis terakhir terhadap satu pantulan nyasar.
static const uint8_t MISI_SAMPEL_N = 3;

// Jeda sesudah ganti profil, sebelum LiDAR ruas itu dipercaya. Badan yang
// naik-turun mengayunkan seluruh berkas sensor tanpa robot berpindah, dan
// pemicu yang dibaca di tengah ayunan menghentikan ruas di tempat salah.
// Dua tunggu berurutan: ramp profil selesai (ditanya ke gait), lalu histori
// median LiDAR diisi ulang -- 3 sampel per kanal.
static const uint32_t MISI_LIDAR_SEGAR_MS = 3UL * NUM_LIDAR * LIDAR_PERIOD_MS;

// Pagar kalau ramp profil tidak kunjung selesai -- gait tidak di-update,
// servo lemas, atau profil target diubah dari luar di tengah tunggu.
static const uint32_t MISI_SETEL_BATAS_MS = 5000;

// Batas waktu SATU ruas. Longgar karena ruas kasar dilalui dengan profil
// lambat: TANGGA menaikkan cycleTime +400 ms, MERUNDUK +200 ms.
static const uint32_t MISI_RUAS_BATAS_MS = 90000;

// Pagar jarak satu ruas. Batas waktu saja tidak cukup: 90 detik pada
// ~10 cm/detik = 9 meter, sedangkan arena 3,6 x 2,4 m. Pernah terjadi: ruas
// HNT_DEPAN yang dindingnya tidak pernah terbaca berjalan keluar arena.
// 200 cm jauh di atas ruas terpanjang yang pernah terukur (120 cm).
static const float MISI_RUAS_MAKS_CM = 200.0f;

// Batas waktu SELURUH misi. Kontes memberi 5 menit (300 detik) dan waktu itu
// juga yang dipakai menghitung bonus: skor x 300 / waktu. Lewat dari itu,
// berjalan terus tidak menambah apa pun -- lebih baik berhenti dengan sebab
// yang tercetak daripada mati di tengah arena.
static const uint32_t MISI_TOTAL_BATAS_MS = 300000;

// SEBERAPA SERONG masih boleh selama ruas berjarak. Jauh lebih longgar dari
// HEADING_TOLERANCE_DEG (6 der): angka itu milik pemeriksaan "pivot sudah
// sampai", tempat robot berdiri DIAM. Di sini robot berjalan di atas koral
// dan kelereng, tempat kaki tergelincir tiap langkah.
//
// Ambangnya turunan dari kerugian odometri, bukan selera: simpang theta
// membuat odometer mengukur sepanjang badan sementara ruas diukur sepanjang
// lorong, jadi jarak nyata = terbaca x cos(theta). Pada 25 der itu 9%.
static const float MISI_SERONG_DEG = 25.0f;

// Berapa lama boleh seserong itu. Diukur dalam SIKLUS GAIT, bukan milidetik
// tetap: koreksi heading hanya bisa bekerja selewat langkah, dan profil
// TANGGA memakai cycleTime dua kali lipat DATAR.
static const uint8_t  MISI_SERONG_SIKLUS = 3;
static const uint32_t MISI_SERONG_MIN_MS = 3000;

// ====================================================================
// SEKUENS LENGAN: pose sendi TETAP dari config.h (KORBAN_SIAP_*, JEPIT_*,
// LEPAS_*, ANGKAT_*), tanpa IK dan tanpa umpan balik posisi -- tiap fase
// diberi waktu tetap LENGAN_JEDA_MS.
//
// Harganya: memindahkan tempat robot berhenti menuntut sudut disetel ulang
// dengan tangan. Seluruh baris AMBIL memakai HNT_LANGSUNG, jadi jarak ke
// korban diatur ruas SEBELUMNYA -- kalau capit meleset, setel ruas itu.
//
// setSudutLengan() menandai sudut di luar 0..180 servo tapi tetap mengirimnya.

// Fase dari waktu. Sekuens ini buta, jadi "sudah sampai?" tidak bisa ditanya
// ke siapa pun -- yang bisa dilakukan cuma memberi tiap langkah jatahnya.
// langkah = jumlah fase yang SUDAH dikerjakan, jadi fase 0 jalan seketika dan
// fase yang terlewat (loop tersendat) tetap dikerjakan, tidak dilompati.
static bool faseBaru(uint8_t& langkah, uint32_t sejak, uint8_t& fase) {
    fase = (uint8_t)((millis() - sejak) / LENGAN_JEDA_MS);
    if (fase < langkah) return false;
    langkah = fase + 1;
    return true;
}

// Satu pose sendi, dengan sebabnya kalau mentok. Sudutnya TETAP dikirim --
// sama seperti perintah 'as' -- karena sekuens yang berhenti di tengah jalan
// meninggalkan capit menggantung di atas korban; yang mentok cuma ditandai.
//
// Penandaan ini wajib ada di sini: HexaArm::angleToPulse() meng-clamp DIAM-
// DIAM, tanpa penanda seperti _servoClamped milik kaki, jadi pose yang tidak
// sampai terlihat persis sama dengan pose yang sampai.
static bool poseSendi(Hexapod& robot, uint8_t lengan,
                      float bahu, float siku, float prg, const char* nama) {
    float sv[3];
    if (robot.setSudutLengan(lengan, bahu, siku, prg, sv)) return true;
    Serial.print("!! Pose lengan '"); Serial.print(nama);
    Serial.printf("' MENTOK: servo bahu %.1f, siku %.1f, pergelangan %.1f der",
                  (double)sv[0], (double)sv[1], (double)sv[2]);
    Serial.println(" -- yang di luar 0..180 di-clamp. Setel KORBAN_*_BAHU/SIKU/PRG.");
    return false;
}

// return true bila sekuens ini sudah selesai.
// `condong` = dua fase tambahan (jeda konfirmasi mata, pelurusan yaw) untuk
// korban tertutup reruntuhan. Translasi maju sendiri berlaku di SEMUA AMBIL.
// `koreksiYaw` dihitung pemanggil -- di sini tidak ada akses ke Navigation.
static bool sekuensAmbil(Hexapod& robot, uint8_t lengan, uint8_t& langkah,
                         uint32_t sejak, bool condong, float koreksiYaw,
                         float mundurMm, float condongMm) {
    // Kolom ruas menang atas param global. 0 di kolom berarti "belum diisi",
    // bukan "maju nol" -- lihat Ruas::condongMm.
    const float majuMm = (condongMm > 0.0f) ? condongMm : KORBAN_CONDONG_MM;
    uint8_t fase;
    if (!faseBaru(langkah, sejak, fase)) return false;

    // Fase 2 selalu menggeser badan maju; fase sesudahnya bergeser nomornya
    // (satu switch, bukan dua salinan). Lengan turun ke ketinggian jepit DULU,
    // lalu badan mendorongnya masuk -- capit maju mendatar ke korban.
    //
    //   fase 0  pose SIAP
    //   fase 1  pose JEPIT
    //   fase 2  BADAN MAJU
    //   fase 3  capit MENUTUP
    //   fase 4  ANGKAT + badan mundur
    //   fase 5  lipat ke REHAT
    if (fase >= 2) {
        if (fase == 2) {
            // Badan maju, kaki diam -- memajukan kaki membuat lengan menabrak
            // reruntuhan. Sumbu X dan Z DIPERTAHANKAN: X memegang penengahan vision
            // Raspi ('t<x> 0 0'), dan menolkannya membuang koreksi yang baru dibayar.
            // Pelan, supaya badan tidak goyang tepat di atas korban.
            robot.setBodySlewMm(KORBAN_CONDONG_LAJU_MM_S);
            const Vec3 t0 = robot.bodyTransTarget();
            robot.setBodyTranslation(t0.x, majuMm, t0.z);
            Serial.printf("  badan condong %.0f mm ke depan pada %.0f mm/detik "
                          "(X %.0f mm milik Raspi DIPERTAHANKAN).\n",
                          (double)majuMm,
                          (double)KORBAN_CONDONG_LAJU_MM_S, (double)t0.x);
            return false;
        }

        // SISANYA HANYA UNTUK RUAS `condong`: jeda konfirmasi mata lalu
        // pelurusan yaw. Ruas AMBIL biasa cuma membayar satu fase, yaitu
        // translasi di atas, lalu langsung lanjut ke pose lengan.
        uint8_t nGeser = 1;
        if (condong) {
            // Tiga fase, yang tengah sengaja kosong: jeda konfirmasi mata antara
            // translasi dan rotasi. Fase, bukan delay -- 'm0'/'s' harus tetap bisa
            // menghentikannya. condong.jeda dibulatkan NAIK ke jatah LENGAN_JEDA_MS
            // terdekat (jeda minimum untuk memeriksa).
            const uint8_t nJeda = (uint8_t)((KORBAN_CONDONG_JEDA_MS + LENGAN_JEDA_MS - 1)
                                            / LENGAN_JEDA_MS);
            // Pelurusan yaw hanya memakai fase kalau benar-benar dikerjakan; dengan
            // condong.yaw 0 ruas condong tidak boleh membayar 2100 ms kosong.
            const uint8_t nYaw = KORBAN_CONDONG_YAW ? 1 : 0;
            if (fase >= 3 && fase < (uint8_t)(3 + nJeda)) return false;
            if (nYaw && fase == (uint8_t)(3 + nJeda)) {
                // Sisa simpangan dari mata angin ruas dihabiskan di atas kaki yang diam.
                // Di-clamp ke BODY_MAX_ROT_DEG (diperbaiki sebagian, bukan ditolak). Roll
                // dan pitch dipertahankan -- menolkannya menjatuhkan badan ke datar di
                // atas lantai yang miring.
                const Vec3 r0 = robot.bodyRotTargetDeg();
                robot.setBodyRotation(r0.x, r0.y, koreksiYaw);
                Serial.printf("  badan diluruskan %+.1f der ke heading ruas.\n",
                              (double)koreksiYaw);
                return false;
            }
            // Satu fase dasar (translasi maju) + jeda + pelurusan. Dengan
            // condong.jeda 0 DAN condong.yaw 0 ini jatuh ke 1, yaitu persis
            // ruas AMBIL biasa -- kolom `condong` jadi gratis, bukan cuma
            // tidak berguna.
            nGeser = (uint8_t)(1 + nJeda + nYaw);
        }
        fase -= nGeser;
    }

    switch (fase) {
        case 0:   // servo lengan hidup, capit BUKA, mendekat DARI ATAS korban
            robot.armEnable(lengan, true);
            // 'm0' di tengah lipatan meninggalkan laju separuh terpasang.
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            robot.setGrip(lengan, KORBAN_GRIP_BUKA);
            poseSendi(robot, lengan, KORBAN_SIAP_BAHU, KORBAN_SIAP_SIKU,
                      KORBAN_SIAP_PRG, "siap");
            // Badan mundur menumpang di fase ini -- lengan dan badan aktuator berbeda,
            // jadi jalur turun yang bebas tidak menambah waktu. X dipertahankan (vision).
            if (mundurMm > 0.0f) {
                robot.setBodySlewMm(KORBAN_CONDONG_LAJU_MM_S);
                const Vec3 t0 = robot.bodyTransTarget();
                robot.setBodyTranslation(t0.x, -mundurMm, t0.z);
                Serial.printf("  badan MUNDUR %.0f mm dulu supaya busur turun "
                              "siku tidak memukul korban.\n", (double)mundurMm);
            }
            break;
        case 1:   // TURUN mengelilingi korban -- bukan menjulur mendatar, yang
            //    mendorong boneka menjauh sebelum capit sempat menutup
            poseSendi(robot, lengan, KORBAN_JEPIT_BAHU, KORBAN_JEPIT_SIKU,
                      KORBAN_JEPIT_PRG, "jepit");
            break;
        case 2:
            robot.setGrip(lengan, KORBAN_GRIP_TUTUP);
            break;
        case 3:   // ANGKAT keluar dari kantong, badan pulang.
            // Jalur keluar dari kantong reruntuhan lewat pose ANGKAT, bukan langsung
            // melipat dari JEPIT ke REHAT (jalur yang belum pernah diuji, membawa
            // boneka yang jauh lebih gemuk daripada capit kosong).
            //
            // Badan pulang SATU fase sesudah capit menutup: tidak bersamaan (menarik
            // boneka dari rahang yang belum menggenggam), tidak ditunda ke lipatan
            // (ruas berikutnya akan berjalan dengan badan condong). X dan rotasi
            // dinolkan; Y justru MUNDUR ke -KORBAN_ANGKAT_MUNDUR_MM supaya busur
            // lipatan tidak membentur dinding yang dihadapi -- dilepas di fase 5.
            // Laju lengan diturunkan mulai di sini (LENGAN_SLEW_ANGKAT_DEG_S).
            robot.setSlewLengan(lengan, LENGAN_SLEW_ANGKAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_ANGKAT_BAHU, KORBAN_ANGKAT_SIKU,
                      KORBAN_ANGKAT_PRG, "angkat");
            robot.setBodyTranslation(0.0f, -KORBAN_ANGKAT_MUNDUR_MM, 0.0f);
            robot.setBodyRotation(0.0f, 0.0f, 0.0f);
            break;
        case 4:   // Melipat ke REHAT, dan di sinilah korban DIGENDONG
            //    sepanjang jalan ke safe zone: terlipat di atas badan, bukan
            //    menjulur di depan. Lajunya diturunkan LAGI di sini, dari
            //    laju angkat ke laju lipat: ayunan bahu inilah yang
            //    benar-benar bisa melempar boneka keluar dari capit.
            robot.setSlewLengan(lengan, LENGAN_SLEW_REHAT_DEG_S);
            poseSendi(robot, lengan, REHAT_BAHU, REHAT_SIKU,
                      REHAT_PERGELANGAN, "rehat");
            break;
        case 5:   // Jatah KEDUA lipatan: bahu 90 der pada laju separuh.
            // Badan pulang di sini, melepas mundur fase 4 -- sesudah busur lipatan
            // melewati ketinggian dinding. 20 mm pada laju condong muat dengan longgar.
            // cek_lengan_laju.py memeriksa jatahnya.
            robot.setBodyTranslation(0.0f, 0.0f, 0.0f);
            break;
        default:
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            // Laju geser badan dikembalikan DI SINI, bukan di fase 3: pose
            // badan baru saja dinolkan di sana, dan perjalanan pulang itu pun
            // harus pelan -- korban sudah ada di capit.
            robot.setBodySlewMm(BODY_SLEW_MM_S);
            return true;
    }
    return false;
}

// MENARUH, dicerminkan dari sekuensAmbil(), kecuali: badan tidak condong
// (safe zone kosong dan datar) dan tanpa vision (titik lepas dari tabel).
// Turun lewat SIAP, lalu NAIK lagi sebelum melipat supaya capit lewat di
// atas boneka yang baru berdiri, bukan menyeretnya. Laju mengikuti beban:
// pelan selama menggenggam (fase 0-1), penuh sesudah capit terbuka.
static bool sekuensTaruh(Hexapod& robot, uint8_t lengan, uint8_t& langkah, uint32_t sejak) {
    uint8_t fase;
    if (!faseBaru(langkah, sejak, fase)) return false;
    switch (fase) {
        case 0:   // buka dari REHAT ke SIAP, capit MASIH menggenggam. Ayunan
            //    bahu inilah yang bisa melempar boneka, jadi laju lipat.
            robot.armEnable(lengan, true);
            robot.setSlewLengan(lengan, LENGAN_SLEW_REHAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_SIAP_BAHU, KORBAN_SIAP_SIKU,
                      KORBAN_SIAP_PRG, "siap (menggendong)");
            break;
        case 1:   // KOSONG DENGAN SENGAJA -- jatah kedua untuk ayunan di atas.
            //    REHAT ke SIAP itu bahu 90 der, dan pada laju lipat ia makan
            //    2540 ms, lebih lama daripada satu LENGAN_JEDA_MS. Ketahuan
            //    cek_lengan_laju.py sebelum pernah dijalankan di robot.
            break;
        case 2:   // TURUN ke titik lepas. Siku yang bekerja, jadi boleh laju
            //    angkat -- sama seperti fase 4 AMBIL, arah terbalik.
            robot.setSlewLengan(lengan, LENGAN_SLEW_ANGKAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_LEPAS_BAHU, KORBAN_LEPAS_SIKU,
                      KORBAN_LEPAS_PRG, "lepas");
            break;
        case 3:   // LEPAS HANYA sesudah sampai; melepas di tengah jalan berarti
            //    korban jatuh di luar safe zone dan nilainya hangus.
            robot.setGrip(lengan, KORBAN_GRIP_BUKA);
            break;
        case 4:   // NAIK menjauh dari korban yang baru berdiri, capit KOSONG.
            //    Tidak ada lagi yang bisa terlempar, jadi laju penuh.
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            poseSendi(robot, lengan, KORBAN_ANGKAT_BAHU, KORBAN_ANGKAT_SIKU,
                      KORBAN_ANGKAT_PRG, "angkat (kosong)");
            break;
        case 5:   // melipat ke REHAT untuk ruas berikutnya
            poseSendi(robot, lengan, REHAT_BAHU, REHAT_SIKU,
                      REHAT_PERGELANGAN, "rehat");
            break;
        default:
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            return true;
    }
    return false;
}

// ====================================================================

Misi::Misi(Hexapod& robot, Navigation& nav, LidarArray& lidar)
    : _robot(robot), _nav(nav), _lidar(lidar) {
    tabelBaku();     // isi RUAS[] dari flash, lalu _cm[] + hitungArah()
}

// ====================================================================
// EDITOR TABEL DARI HUD ('m5')
//
// Yang dijaga seluruh bagian ini cuma satu hal: tabel yang dijalankan saat
// percobaan harus sama dengan tabel yang nanti di-flash. Karena itu tiap
// perubahan memanggil tabelBerubah(), tiap perubahan ditolak selama misi
// berjalan, dan tabelCrc() memberi HUD cara memastikannya tanpa menebak.
// ====================================================================

void Misi::salinBaris(uint8_t ke, const Ruas& dari) {
    RUAS[ke] = dari;
    strncpy(NAMA_RAM[ke], dari.nama ? dari.nama : "", RUAS_NAMA_MAKS - 1);
    NAMA_RAM[ke][RUAS_NAMA_MAKS - 1] = '\0';
    RUAS[ke].nama = NAMA_RAM[ke];
}

void Misi::tabelBaku() {
    RUAS_N = RUAS_BAKU_N;
    for (uint8_t i = 0; i < RUAS_N; i++) salinBaris(i, RUAS_BAKU[i]);
    skorBaku();                       // peta poin ikut pulang ke flash
    tabelBerubah();
}

bool Misi::tabelSamaBaku() const {
    // Dibandingkan KOLOM PER KOLOM, bukan memcmp: `nama` sengaja menunjuk ke
    // kolam RAM, jadi pointernya memang berbeda dan memcmp akan selalu gagal.
    if (RUAS_N != RUAS_BAKU_N) return false;
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& a = RUAS[i];
        const Ruas& b = RUAS_BAKU[i];
        if (a.belok != b.belok || a.kemudi != b.kemudi || a.profil != b.profil
            || a.abaikanDepan != b.abaikanDepan || a.henti != b.henti
            || a.nilai != b.nilai || a.aksi != b.aksi || a.lengan != b.lengan
            || a.putar != b.putar || a.condong != b.condong
            || a.jagaBelakang != b.jagaBelakang || a.mundurMm != b.mundurMm
            || a.condongMm != b.condongMm) return false;
        if (strcmp(a.nama ? a.nama : "", b.nama ? b.nama : "") != 0) return false;
    }
    return true;
}

bool Misi::sisipBaris(uint8_t idx) {
    if (!bolehUbahTabel()) return false;
    if (RUAS_N >= RUAS_MAKS) {
        Serial.print("m5+: tabel penuh, batas RUAS_MAKS ");
        Serial.println(RUAS_MAKS);
        return false;
    }
    if (idx > RUAS_N) idx = RUAS_N;
    for (uint8_t i = RUAS_N; i > idx; i--) salinBaris(i, RUAS[i - 1]);
    // Kurung kosong WAJIB: anggota awal struct Ruas (nama..lengan) tidak punya
    // nilai baku di deklarasinya, jadi `Ruas kosong;` berisi sampah stack --
    // dan sampah di kolom `henti` adalah ruas yang berjalan tanpa syarat henti.
    Ruas kosong{};
    kosong.nama = "baris baru";
    kosong.henti = HNT_LANGSUNG;       // baris kosong tidak boleh JALAN
    salinBaris(idx, kosong);
    RUAS_N++;
    skorSisip(idx);                    // peta poin ikut bergeser
    // Struktur berubah -> _cm[] disalin ulang seluruhnya, jadi override 'm7'
    // pada ruas mana pun hilang. Disengaja, dan dicetak supaya tidak senyap.
    tabelBerubah();
    Serial.print("m5+ "); Serial.print(idx);
    Serial.print(" -- "); Serial.print(RUAS_N);
    Serial.println(" baris. Panjang ruas hasil 'm7' dipulangkan ke tabel.");
    Serial.println("  Baris baru: HNT_LANGSUNG, tidak berjalan dan tidak berpoin.");
    return true;
}

bool Misi::hapusBaris(uint8_t idx) {
    if (!bolehUbahTabel()) return false;
    if (idx >= RUAS_N) {
        Serial.print("m5-: indeks "); Serial.print(idx);
        Serial.print(" di luar tabel 0.."); Serial.println(RUAS_N - 1);
        return false;
    }
    if (RUAS_N <= 1) {
        Serial.println("m5-: tabel tinggal satu baris, tidak dihapus.");
        return false;
    }
    for (uint8_t i = idx; i + 1 < RUAS_N; i++) salinBaris(i, RUAS[i + 1]);
    RUAS_N--;
    skorHapus(idx);
    tabelBerubah();
    Serial.print("m5- "); Serial.print(idx);
    Serial.print(" -- "); Serial.print(RUAS_N);
    Serial.println(" baris. Panjang ruas hasil 'm7' dipulangkan ke tabel.");
    return true;
}

void Misi::tabelBerubah() {
    // _cm[] adalah CACHE dari RUAS[].nilai, dan cache yang tidak diperbarui
    // adalah sebab nomor satu "sebelum flash bisa, sesudah flash tidak".
    for (uint8_t i = 0; i < RUAS_N; i++) _cm[i] = RUAS[i].nilai;
    // Arah mutlak tiap ruas turunan dari kolom `belok` + `putar`, jadi ia
    // WAJIB dihitung ulang walau cuma satu baris yang berubah: satu belok
    // yang bergeser memutar seluruh sisa lintasan.
    hitungArah();
}

bool Misi::bolehUbahTabel() {
    if (!berjalan()) return true;
    Serial.println("Tabel TIDAK diubah: misi sedang berjalan. Tekan 'm0' dulu.");
    return false;
}

bool Misi::setBaris(uint8_t idx, const float* v, const char* nama) {
    if (!bolehUbahTabel()) return false;
    if (idx >= RUAS_N) {
        Serial.print("m5s: indeks "); Serial.print(idx);
        Serial.print(" di luar tabel 0.."); Serial.println(RUAS_N - 1);
        return false;
    }
    Ruas& r = RUAS[idx];
    r.belok        = (Belok)  (uint8_t)v[0];
    r.kemudi       = (Kemudi) (uint8_t)v[1];
    r.profil       = (Profil) (uint8_t)v[2];
    r.abaikanDepan = v[3] > 0.5f;
    r.henti        = (Henti)  (uint8_t)v[4];
    r.nilai        = v[5];
    r.aksi         = (Aksi)   (uint8_t)v[6];
    r.lengan        = (uint8_t)v[7];
    r.putar        = v[8];
    r.condong      = v[9]  > 0.5f;
    r.jagaBelakang = v[10] > 0.5f;
    r.mundurMm     = v[11];
    r.condongMm    = v[12];
    if (nama) {
        strncpy(NAMA_RAM[idx], nama, RUAS_NAMA_MAKS - 1);
        NAMA_RAM[idx][RUAS_NAMA_MAKS - 1] = '\0';
    }
    r.nama = NAMA_RAM[idx];
    // MENANG ATAS 'm7'. tabelBerubah() menyalin ulang seluruh _cm[] dari
    // tabel, jadi panjang ruas yang pernah disetel 'm7 <idx> <cm>' dipulangkan
    // ke angka tabel. Disengaja -- tabel RAM adalah satu-satunya sumber -- dan
    // diperingatkan di .ino supaya tidak senyap.
    tabelBerubah();
    return true;
}

uint16_t Misi::tabelCrc() const {
    // BENTUK KANONIK, dan ia harus sama byte per byte dengan
    // moses/lintasan.py::crc_tabel(). Kalau urutan di sini bergeser, CRC HUD
    // tidak akan pernah cocok lagi dan HUD mengirim ulang tabel tanpa henti --
    // gejalanya tidak akan terbaca sebagai "urutan byte berubah".
    uint8_t n = RUAS_N;
    uint16_t crc = Calib::crc16(&n, 1);
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& r = RUAS[i];
        uint8_t buf[25 + RUAS_NAMA_MAKS];
        uint8_t k = 0;
        buf[k++] = (uint8_t)r.belok;
        buf[k++] = (uint8_t)r.kemudi;
        buf[k++] = (uint8_t)r.profil;
        buf[k++] = r.abaikanDepan ? 1 : 0;
        buf[k++] = (uint8_t)r.henti;
        memcpy(buf + k, &r.nilai, 4);     k += 4;
        buf[k++] = (uint8_t)r.aksi;
        buf[k++] = r.lengan;
        memcpy(buf + k, &r.putar, 4);     k += 4;
        buf[k++] = r.condong ? 1 : 0;
        buf[k++] = r.jagaBelakang ? 1 : 0;
        memcpy(buf + k, &r.mundurMm, 4);  k += 4;
        memcpy(buf + k, &r.condongMm, 4); k += 4;
        const char* s = r.nama ? r.nama : "";
        for (uint8_t j = 0; j < RUAS_NAMA_MAKS - 1 && s[j]; j++) buf[k++] = s[j];
        buf[k++] = 0;
        crc = Calib::crc16(buf, k, crc);
    }
    return crc;
}

void Misi::tabelVersi() {
    // Satu baris, dibaca HUD tiap kali ia perlu tahu apakah tabel RAM masih
    // sama dengan draft-nya. Sesudah reset Teensy, CRC kembali ke nilai tabel
    // flash -- dan justru itu yang membuat reset terdeteksi tanpa saklar
    // tambahan.
    Serial.print("#TABV "); Serial.print(tabelCrc());
    Serial.print(' ');      Serial.println(RUAS_N);
}

void Misi::tabelDump() {
    // Kolomnya URUTAN YANG SAMA dengan yang diterima 'm5s', jadi satu baris
    // '#TABR' bisa dikirim balik apa adanya sebagai 'm5s'. Angka pecahan
    // dicetak 2 desimal supaya putar/mundurMm/condongMm tidak dibulatkan.
    //
    // NILAI SUMBER, bukan hasil pencerminan: yang diedit adalah tabelnya, dan
    // belokRuas()/kemudiRuas() yang mencerminkannya saat dibaca. Ini kebalikan
    // dari tabel(), yang sengaja mencetak yang BERLAKU.
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& r = RUAS[i];
        Serial.print("#TABR ");         Serial.print(i);
        Serial.print(' '); Serial.print((uint8_t)r.belok);
        Serial.print(' '); Serial.print((uint8_t)r.kemudi);
        Serial.print(' '); Serial.print((uint8_t)r.profil);
        Serial.print(' '); Serial.print(r.abaikanDepan ? 1 : 0);
        Serial.print(' '); Serial.print((uint8_t)r.henti);
        Serial.print(' '); Serial.print(r.nilai, 2);
        Serial.print(' '); Serial.print((uint8_t)r.aksi);
        Serial.print(' '); Serial.print(r.lengan);
        Serial.print(' '); Serial.print(r.putar, 2);
        Serial.print(' '); Serial.print(r.condong ? 1 : 0);
        Serial.print(' '); Serial.print(r.jagaBelakang ? 1 : 0);
        Serial.print(' '); Serial.print(r.mundurMm, 2);
        Serial.print(' '); Serial.print(r.condongMm, 2);
        Serial.print(" | ");
        Serial.println(r.nama ? r.nama : "");
    }
    tabelVersi();
}

// Belok relatif -> mata angin mutlak. Dulu rumus ini punya kembaran di
// Navigation::arahGeser(); kembarannya dihapus 6 Sep 2026 bersama
// belok-otomatis mode arena, jadi INILAH satu-satunya tempat arah mutlak
// dihitung di seluruh firmware.
// Serong yang terkumpul sejak _yawUjung dicatat, dibungkus ke +-180.
// Positif = badan berputar searah yaw naik; tandanya dicocokkan di arena
// sekali, lalu berlaku untuk semua cetakan ini.
float Misi::selisihYaw() const {
    float d = _nav.yawKini() - _yawUjung;
    while (d >= 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// --- ARENA CERMIN ---
// Saklar dibaca dari gParam tiap pemakaian, bukan disalin: tombol boleh
// membaliknya kapan saja (selama misi tidak berjalan).
// _arah[] dan _serong[] adalah CACHE -- segarkanArah() menghitungnya ulang
// saat saklar berubah. Tanpa itu kolom kemudi tercermin tapi sasaran pivot
// tidak, dan robot tetap berbelok ke arah tabel asli.
bool Misi::arenaCermin() { return gParam[K_ARENA_MIRROR] >= 0.5f; }

Belok Misi::belokRuas(uint8_t i) const {
    const Belok b = RUAS[i].belok;
    if (!arenaCermin()) return b;
    // (4 - b) % 4: LURUS 0 tetap 0, KANAN 1 jadi 3 (KIRI), BALIK 2 tetap 2,
    // KIRI 3 jadi 1 (KANAN). Rumus, bukan tabel -- tabel kecil yang benar
    // hari ini adalah tabel yang salah begitu enumnya disisipi nilai baru.
    return (Belok)((4u - (uint8_t)b) % 4u);
}

Kemudi Misi::kemudiRuas(uint8_t i) const {
    const Kemudi k = RUAS[i].kemudi;
    if (!arenaCermin() || k == KMD_TENGAH) return k;   // TENGAH tidak berarah
    return (k == KMD_KIRI) ? KMD_KANAN : KMD_KIRI;
}

float Misi::putarRuas(uint8_t i) const {
    return arenaCermin() ? -RUAS[i].putar : RUAS[i].putar;
}

void Misi::segarkanArah() {
    if (arenaCermin() != _arahCermin) hitungArah();
}

void Misi::hitungArah() {
    _arahCermin = arenaCermin();
    uint8_t a = MISI_ARAH_BERANGKAT;
    float   s = 0.0f;
    for (uint8_t i = 0; i < RUAS_N; i++) {
        a = (uint8_t)((a + (uint8_t)belokRuas(i)) % 4);
        // Serong menumpuk seperti belok menumpuk: keduanya "belok masuk ruas
        // ini", cuma satuannya berbeda. Yang satu dibungkus modulo 4, yang
        // satu dijumlahkan lurus.
        s += putarRuas(i);
        _arah[i]   = a;
        _serong[i] = s;
    }
}

// Heading MUTLAK ruas ini: mata angin arena + serong yang menumpuk.
// NAN bila mata anginnya belum dicatat kompas -- pemanggil harus memeriksa,
// karena seluruh misi mustahil tanpa itu.
float Misi::headingRuas(uint8_t i) const {
    const float dasar = _nav.headingArah(_arah[i]);
    if (isnan(dasar)) return NAN;
    float h = dasar + _serong[i];
    while (h >= 360.0f) h -= 360.0f;
    while (h < 0.0f)    h += 360.0f;
    return h;
}

// true bila ruas ini memang menyerong dari mata angin. Dipakai cetakan supaya
// "UTARA" tidak berbohong saat robot sebenarnya menghadap 45 der dari UTARA.
bool Misi::ruasSerong(uint8_t i) const { return fabsf(_serong[i]) > 0.5f; }

// Berapa derajat badan harus diputar supaya menghadap heading ruas ini.
// Positif/negatif mengikuti wrap180: hasilnya simpangan yang HARUS DIHABISKAN,
// bukan heading sasarannya.
//
// NAN dari headingRuas() atau IMU yang bisu -> 0, artinya "jangan luruskan".
// Menebak arah dengan angka yang tidak diketahui lebih buruk daripada tidak
// meluruskan sama sekali: yang pertama memutar badan ke arah yang salah tepat
// sebelum capit turun.
float Misi::koreksiYawRuas() const {
    const float sasar = headingRuas(_i);
    if (isnan(sasar)) return 0.0f;
    const float kini = _nav.yawKini();
    if (isnan(kini)) return 0.0f;
    float d = sasar - kini;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void Misi::masuk(StatMisi s) {
    _stat = s;
    _t0 = millis();
    _n = 0;
    _stempel = 0;
    _langkah = 0;
    _tenangT0 = 0;
}

void Misi::gagal(const char* sebab) {
    _sebab = sebab;
    _stat  = MISI_GAGAL;
    lepasVisi();       // kamera tidak boleh ikut menggantung bersama misinya
    // Apa pun sebabnya, jangan tinggalkan robot berjalan buta ke depan.
    // navBerhenti() mengembalikan abaikanDepan sendiri, tapi ia menolak
    // bekerja kalau navigasi memang sudah diam -- dan justru itu jalur
    // kegagalan yang paling sering (navigasi berhenti duluan, misi menyusul).
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    Serial.print("\n!! MISI GAGAL di ruas "); Serial.print(_i);
    Serial.print(" ("); Serial.print(RUAS[_i].nama); Serial.println(")");
    Serial.print("   sebab: "); Serial.println(sebab);
    Serial.println("   Robot berhenti di tempat, servo TETAP HIDUP.");
    Serial.println("   'm' status  |  'm1' ulang dari awal  |  'm4 <idx>' lanjut dari ruas");
    Serial.println("   'x' melemaskan servo.");
}

// ====================================================================
// PEMERIKSAAN TABEL -- dijalankan sekali di 'm1', selagi robot masih diam.
//
// Semua yang diperiksa di sini adalah hal yang KALAU LOLOS akan merusak
// sesuatu di tengah arena, tempat tidak ada lagi yang bisa dilakukan.
// ====================================================================
bool Misi::tabelSiap(uint8_t dari, uint8_t sampai) {
    bool ok = true;

    for (uint8_t i = dari; i <= sampai; i++) {
        // 1) Ruas berjalan yang panjangnya belum diukur.
        if (RUAS[i].henti != HNT_LANGSUNG && _cm[i] < 0.0f) {
            if (ok) Serial.println("Gagal: masih ada ruas yang panjangnya BELUM DIUKUR.");
            Serial.print("  m7 "); Serial.print(i); Serial.print(" <cm>   -- ");
            Serial.println(RUAS[i].nama);
            ok = false;
        }

        // 2) Ruas buta ke depan yang tidak dibatasi odometri: tidak ada yang
        //    menghentikannya. HNT_PUNCAK dikecualikan -- 'buta' hanya mematikan
        //    aturan kemudi Navigation; ruasSelesai() tetap membaca sensor depan
        //    langsung. Buta untuk menyetir, melihat untuk berhenti.
        if (RUAS[i].abaikanDepan && RUAS[i].henti != HNT_ODO &&
            RUAS[i].henti != HNT_PUNCAK) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.println(" berjalan buta ke depan tapi tidak dibatasi odometri.");
            Serial.println("  Perbaiki TABEL di Misi.cpp -- ini salah tulis, bukan salah setel.");
            ok = false;
        }

        // 3) Ambang sensor depan harus DI ATAS FRONT_STOP_CM. Di bawah angka
        //    itu navigasi keburu BERHENTI sendiri karena halangan depan, dan
        //    ruasnya tidak pernah sampai ke aksi ujungnya. (Dulu ia berbelok
        //    90 der alih-alih berhenti; belok-otomatisnya dihapus 6 Sep 2026,
        //    tapi ambangnya tetap harus di atas FRONT_STOP_CM.)
        // PUNCAK dengan nilai 0 SAH: itu artinya "gyro sendirian", bukan ambang
        // yang lupa diisi. HNT_DEPAN tidak punya arti itu -- di sana 0 memang
        // salah tulis.
        if ((RUAS[i].henti == HNT_DEPAN ||
             (RUAS[i].henti == HNT_PUNCAK && _cm[i] > 0.0f)) &&
            _cm[i] <= (float)FRONT_STOP_CM) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" ambang depan "); Serial.print(_cm[i], 0);
            Serial.print(" cm <= FRONT_STOP_CM "); Serial.println(FRONT_STOP_CM);
            Serial.println("  Navigasi akan berbelok sebelum misi sempat berhenti.");
            ok = false;
        }

        // 3b) Ruas GESER: sisi mana yang jadi penggaris dibaca dari kolom
        //    kemudi, jadi KMD_TENGAH tidak punya arti di sini. Dan sasarannya
        //    harus DI ATAS pita "terlalu dekat": penjaga arah geser memakai
        //    pita yang sama, sehingga sasaran di bawahnya akan ditolak tepat
        //    sebelum tercapai -- gagal di tengah arena, bukan di sini.
        if (RUAS[i].henti == HNT_SISI) {
            if (kemudiRuas(i) == KMD_TENGAH) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.println(" bergeser tapi kemudinya MENENGAH -- pilih KANAN atau KIRI.");
                Serial.println("  Perbaiki TABEL di Misi.cpp -- ini salah tulis, bukan salah setel.");
                ok = false;
            }
            if (_cm[i] >= 0.0f && _cm[i] <= (float)WALL_MIN_CM) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" sasaran geser "); Serial.print(_cm[i], 0);
                Serial.print(" cm <= wall.min "); Serial.println(WALL_MIN_CM, 0);
                Serial.println("  Penjaga arah geser akan menolaknya sebelum sasaran tercapai.");
                ok = false;
            }
        }

        // 3c) Ruas MUNDUR: sasarannya harus DI ATAS pita "terlalu dekat",
        //    alasan yang sama persis dengan ruas GESER di atas.
        //    setelBelakangMulai() menolak sasaran <= WALL_MIN_CM, dan tanpa
        //    pemeriksaan ini penolakan itu datang di arena -- sesudah ruas
        //    sebelumnya selesai dan robot sudah berdiri di tempatnya.
        if (RUAS[i].henti == HNT_GESER && RUAS[i].nilai >= 0.0f &&
            (RUAS[i].nilai < 1.0f || RUAS[i].nilai > 200.0f)) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" geser "); Serial.print(RUAS[i].nilai, 0);
            Serial.println(" cm di luar 1..200.");
            ok = false;
        }

        if (RUAS[i].henti == HNT_MUNDUR &&
            _cm[i] >= 0.0f && _cm[i] <= (float)MUNDUR_MIN_CM) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" sasaran mundur "); Serial.print(_cm[i], 0);
            Serial.print(" cm <= lantai mundur "); Serial.println(MUNDUR_MIN_CM, 0);
            Serial.println("  setelBelakangMulai() akan menolaknya sebelum berangkat.");
            ok = false;
        }

        // 3d) Ruas AMBIL: mundurMm tidak boleh lewat BODY_MAX_TRANS_MM (di-clamp
        //    diam-diam), dan mundurMm + condong harus muat dalam SATU jatah
        //    LENGAN_JEDA_MS -- kalau tidak, capit menutup sebelum badan sampai.
        //    Semua baris AMBIL diperiksa: yang kolomnya kosong tetap menempuh
        //    KORBAN_CONDONG_MM.
        if (RUAS[i].aksi == AKS_AMBIL) {
            // Kolom ruas menang atas param global, sama seperti di
            // sekuensAmbil(). Memeriksa yang global di sini sedangkan sekuens
            // memakai yang per-ruas berarti memeriksa angka yang salah.
            const float maju = (RUAS[i].condongMm > 0.0f) ? RUAS[i].condongMm
                                                          : KORBAN_CONDONG_MM;
            if (RUAS[i].mundurMm > BODY_MAX_TRANS_MM ||
                maju > BODY_MAX_TRANS_MM) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" mundur "); Serial.print(RUAS[i].mundurMm, 0);
                Serial.print(" / maju "); Serial.print(maju, 0);
                Serial.print(" mm > BODY_MAX_TRANS_MM "); Serial.println(BODY_MAX_TRANS_MM, 0);
                ok = false;
            }
            // DUA perjalanan badan, masing-masing satu jatah fase, dan yang
            // TERPANJANG yang menentukan:
            //   BADAN MAJU   dari -mundurMm ke +maju
            //   ANGKAT       dari +maju ke -KORBAN_ANGKAT_MUNDUR_MM
            // Memeriksa yang pertama saja pernah cukup; sejak mundur-saat-
            // mengangkat dipasang 17 Sep 2026, yang kedua bisa lebih panjang
            // kalau mundurMm kecil sementara maju besar.
            const float tempuhMaju   = RUAS[i].mundurMm + maju;
            const float tempuhAngkat = maju + KORBAN_ANGKAT_MUNDUR_MM;
            const float tempuh = (tempuhAngkat > tempuhMaju) ? tempuhAngkat
                                                             : tempuhMaju;
            const float muat   = KORBAN_CONDONG_LAJU_MM_S * (LENGAN_JEDA_MS / 1000.0f);
            if (tempuh > muat) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" badan menempuh "); Serial.print(tempuh, 0);
                Serial.print(" mm, hanya muat "); Serial.print(muat, 0);
                Serial.println(" mm dalam satu fase.");
                Serial.print("  maju "); Serial.print(tempuhMaju, 0);
                Serial.print(" mm, angkat "); Serial.print(tempuhAngkat, 0);
                Serial.println(" mm.");
                Serial.println("  Turunkan mundurMm/condong.mm, atau naikkan condong laju.");
                ok = false;
            }
        }

        // 3e) PUNCAK di ruas DATAR hampir pasti salah ketik. Bukan kegagalan:
        //    tanjakan bisa saja dilewati dengan profil lain suatu hari, dan
        //    menolaknya berarti menebak arena. Tapi HNT_PUNCAK menunggu badan
        //    MIRING dulu, dan di ruas datar itu tidak pernah terjadi -- ruasnya
        //    berjalan sampai batas waktu, tanpa satu pun pesan yang menyebut
        //    sebabnya.
        if (RUAS[i].henti == HNT_PUNCAK &&
            RUAS[i].profil != PRF_TANJAK && RUAS[i].profil != PRF_KAIL) {
            Serial.print("Awas: ruas "); Serial.print(i);
            Serial.println(" HNT_PUNCAK tapi profilnya bukan TANJAK/KAIL.");
            Serial.println("  PUNCAK menunggu badan MIRING dulu. Di ruas datar itu");
            Serial.println("  tidak pernah terjadi, dan ruasnya habis di batas waktu.");
        }

        // 3f) PUNCAK dengan nilai 0 tidak punya jalan keluar kalau IMU bisu.
        //    Peringatan, bukan penolakan: IMU yang sehat membuat kombinasi ini
        //    benar dan memang diminta di R-9. Yang dijaga cuma supaya orang
        //    tahu bahwa satu-satunya penahan tersisa adalah batas waktu ruas.
        if (RUAS[i].henti == HNT_PUNCAK && _cm[i] <= 0.0f) {
            Serial.print("Awas: ruas "); Serial.print(i);
            Serial.println(" HNT_PUNCAK nilai 0 -- dinding depan dimatikan, gyro sendirian.");
            Serial.println("  Kalau IMU bisu, TIDAK ADA yang mengakhiri ruas ini selain");
            Serial.println("  batas waktu. Periksa 'k' dan IMU sebelum berangkat.");
        }

        // 4) Pemicu sensor belakang harus jatuh di dalam jangkauan sensor.
        //    Di luar itu bacaannya jatuh ke LIDAR_JAUH dan pemicunya tidak
        //    pernah menyala -- robot melewati sasarannya lalu mati kehabisan
        //    waktu, tanpa satu pun petunjuk sebabnya.
        if (RUAS[i].henti == HNT_BELAKANG && _cm[i] >= (float)LIDAR_MAX_CM - 15.0f) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" jarak tempuh "); Serial.print(_cm[i], 0);
            Serial.print(" cm terlalu jauh untuk sensor belakang (maks ");
            Serial.print(LIDAR_MAX_CM); Serial.println(" cm).");
            ok = false;
        }
    }

    // 5) Tiap capit muat SATU korban. Disimulasikan sepanjang ruas yang akan
    //    dijalani, dari capit kosong -- cacat ini lahir dari URUTAN, tidak
    //    kelihatan dari satu baris.
    bool isi[2] = { false, false };
    for (uint8_t i = dari; i <= sampai; i++) {
        if (RUAS[i].aksi == AKS_TIDAK_ADA) continue;
        const uint8_t a = RUAS[i].lengan & 1;
        const char* nama = a == ARM_DEPAN ? "DEPAN" : "BELAKANG";
        if (RUAS[i].aksi == AKS_AMBIL) {
            // Sekuensnya buta (sudut tetap), jadi sesuatu harus menaruh korban di
            // jangkauan lengan. Dua cara sah:
            //   HNT_DEPAN     ruas ini berjalan sampai LiDAR depan membaca `nilai`.
            //   HNT_LANGSUNG  ruas SEBELUMNYA yang menempatkan robot (mis. HNT_MUNDUR
            //                 ke tembok belakang -- lebih jujur daripada LiDAR depan
            //                 di ceruk sempit).
            if (RUAS[i].henti != HNT_DEPAN && RUAS[i].henti != HNT_LANGSUNG) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.println(" mengambil korban tapi hentinya bukan HNT_DEPAN atau HNT_LANGSUNG.");
                Serial.println("  HNT_DEPAN <cm> = ruas ini yang mendekat.");
                Serial.println("  HNT_LANGSUNG   = ruas sebelumnya yang sudah menempatkan robot.");
                ok = false;
            }
            // Peringatan, bukan penolakan: jarak boleh saja diatur ruas yang lebih
            // jauh atau oleh penempatan tangan sebelum start. Tetap dicetak karena
            // akibatnya diam -- capit menutup di udara.
            if (RUAS[i].henti == HNT_LANGSUNG) {
                const bool adaPendekat = (i > 0)
                    && (RUAS[i - 1].henti == HNT_MUNDUR
                        || RUAS[i - 1].henti == HNT_DEPAN);
                if (!adaPendekat) {
                    Serial.print("Awas: ruas "); Serial.print(i);
                    Serial.println(" AMBIL HNT_LANGSUNG tanpa ruas pendekat di depannya.");
                    Serial.println("  Jaraknya diatur dari luar tabel. Kalau capit menutup di");
                    Serial.println("  udara, ini tempat pertama yang harus diperiksa: HNT_SISI");
                    Serial.println("  cuma menggeser menyamping, baris TARUH tidak bergerak.");
                }
            }
            if (isi[a]) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" mengambil korban dengan capit "); Serial.print(nama);
                Serial.println(" yang MASIH TERISI.");
                Serial.println("  Taruh dulu, atau pindahkan ruas ini ke capit sebelahnya.");
                ok = false;
            }
            isi[a] = true;
        } else {
            if (!isi[a]) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" menaruh korban dari capit "); Serial.print(nama);
                Serial.println(" yang KOSONG.");
                ok = false;
            }
            isi[a] = false;
        }
    }
    return ok;
}

// ====================================================================

void Misi::mulai() { mulaiDari(0); }

// Syarat yang WAJIB benar sebelum robot boleh bergerak sama sekali. Dipakai
// bersama oleh 'm1'/'m4' dan mode ukur 'm6' -- satu pemeriksaan, satu pesan.
bool Misi::siapJalan(uint8_t idx, uint8_t sampai) {
    if (berjalan()) {
        Serial.println("Misi sedang berjalan. 'm' status, 'm0' batalkan dulu.");
        return false;
    }
    if (idx >= RUAS_N) {
        Serial.print("Ruas "); Serial.print(idx); Serial.print(" tidak ada -- hanya 0..");
        Serial.println(RUAS_N - 1);
        return false;
    }

    // navMulai() SUDAH memeriksa servo, mux, sensor depan, sensor samping,
    // IMU, kelengkapan kompas, dan kalibrasi pivot -- lalu mencetak persis
    // mana yang gagal. Yang diperiksa DI SINI hanya dua hal yang kalau
    // dibiarkan akan gagal SESUDAH robot terlanjur berputar 20 detik:
    // pivotKe() cuma MEMPERINGATKAN kalau arah putar belum dikalibrasi,
    // sedangkan mode arena MENOLAK.
    if (!_nav.kompasLengkap()) {
        Serial.println("Gagal: kompas arena belum lengkap -- 'c0'..'c3' lalu 'e', atau 'E'.");
        Serial.println("       Periksa dengan 'k'.");
        return false;
    }
    if (!_nav.pivotTerkalibrasi()) {
        Serial.println("Gagal: pivot belum dikalibrasi -- 'C' lalu 'S' sekali saja.");
        Serial.println("       Periksa dengan 'K'. Tanpa itu arah putar masih tebakan.");
        return false;
    }

    // Sensor yang dipakai ruas tapi tidak diperiksa navMulai(): belakang
    // (HNT_BELAKANG/HNT_MUNDUR) dan sisi penggaris ruas geser. Ditolak di sini,
    // bukan di tengah arena. Hanya sampai `sampai` -- latihan satu ruas tidak
    // boleh ditolak karena sensor ruas lain.
    for (uint8_t i = idx; i <= sampai && i < RUAS_N; i++) {
        if (RUAS[i].henti != HNT_SISI) continue;
        const uint8_t ch = (kemudiRuas(i) == KMD_KIRI) ? LIDAR_KIRI_D : LIDAR_KANAN_D;
        if (_lidar.getDistance(ch) != LIDAR_MATI) continue;
        Serial.print("Gagal: sensor "); Serial.print(LidarArray::nama(ch));
        Serial.print(" mati, padahal ruas "); Serial.print(i);
        Serial.println(" memakainya sebagai penggaris geser.");
        Serial.println("       'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return false;
    }

    bool perluBelakang = false;
    for (uint8_t i = idx; i <= sampai && i < RUAS_N; i++)
        if (RUAS[i].henti == HNT_BELAKANG ||
            RUAS[i].henti == HNT_MUNDUR) perluBelakang = true;
    if (perluBelakang && _lidar.getDistance(LIDAR_BACK) == LIDAR_MATI) {
        Serial.print("Gagal: sensor BELAKANG (channel "); Serial.print(LIDAR_BACK);
        Serial.println(") tidak merespons.");
        Serial.println("       Dialah yang mengukur jarak tempuh dari dinding START.");
        Serial.println("       'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return false;
    }
    return true;
}

void Misi::mulaiDari(uint8_t idx, uint8_t sampai) {
    if (sampai >= RUAS_N) sampai = RUAS_N - 1;
    if (!siapJalan(idx, sampai)) return;
    if (sampai < idx) {
        Serial.print("Ruas akhir "); Serial.print(sampai);
        Serial.print(" ada SEBELUM ruas awal "); Serial.println(idx);
        return;
    }
    // Saklar cermin boleh dibalik kapan saja SELAMA misi tidak berjalan, dan
    // arah tiap ruas turunan dari saklar itu. Disegarkan di sini, sebelum
    // tabelSiap() -- pemeriksa itu sendiri membaca arah.
    segarkanArah();
    if (!tabelSiap(idx, sampai)) return;
    _iAkhir = sampai;

    // Mulai dari keadaan yang DIKETAHUI. Kalau lari sebelumnya gagal di
    // tengah ruas kasar, profil gait masih TANGGA dan kemudi masih MENENGAH;
    // tanpa baris ini lari berikutnya menyusuri lorong datar dengan siklus
    // dua kali lipat dan kehabisan waktu tanpa sebab yang kelihatan.
    _robot.profileFlat();
    _nav.setTengah(false);
    _nav.abaikanDepan(false);

    _i       = idx;
    _blkAwal = -1.0f;
    _korban[0] = _korban[1] = false;
    _parkirVision = false;
    _visiJalan    = false;
    _sebab   = nullptr;
    _serongT0 = 0;
    _lidarUlang = 0;

    Serial.println("\n=== MISI MULAI ===");
    Serial.print("  ruas "); Serial.print(idx); Serial.print(" dari 0..");
    Serial.print(RUAS_N - 1); Serial.print("  --  "); Serial.println(RUAS[idx].nama);
    Serial.print("  berangkat   : "); Serial.print(_nav.namaArah(MISI_ARAH_BERANGKAT));
    Serial.println("  <- c0, arah LORONG PERTAMA dari HOME.");
    Serial.println("                Salah catat di sini menggeser SELURUH kolom arah.");
    Serial.print("  Batas waktu kontes "); Serial.print(MISI_TOTAL_BATAS_MS / 1000);
    Serial.println(" detik. 's', 'x', Enter, atau 'm0' membatalkan kapan saja.");
    Serial.println("  Capit belum terpasang: tiap ruas korban hanya BERHENTI sejenak.");

    _tMisi = millis();
    ruasMasuk();
}

// ====================================================================
// MODE UKUR: satu ruas dengan profil, kemudi, dan arahnya sendiri, berjalan
// sampai operator menghentikannya, lalu mencetak jarak tempuh odometri.
// Profilnya ikut dipakai -- panjang langkah tiap gait berbeda.
// ====================================================================
void Misi::ukur(uint8_t idx) {
    if (!siapJalan(idx, idx)) return;

    const Ruas& x = RUAS[idx];
    if (x.henti == HNT_LANGSUNG) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" tidak berjalan (hanya aksi) -- tidak ada yang bisa diukur.");
        return;
    }
    if (x.henti == HNT_SISI) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" bergeser menyamping -- yang disetel bukan jarak tempuh.");
        Serial.println("  Sasarannya jarak ke dinding SAMPING: baca dari 'l', lalu 'm7'.");
        Serial.println("  Cobanya dengan 'V<cm>' langsung, tanpa lewat misi.");
        return;
    }
    if (x.henti == HNT_MUNDUR) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" berjalan MUNDUR -- yang disetel bukan jarak tempuh.");
        Serial.println("  Sasarannya jarak MUTLAK ke dinding belakang: baca dari 'l', lalu 'm7'.");
        Serial.println("  Cobanya dengan 'J<cm>' langsung, tanpa lewat misi.");
        return;
    }
    if (x.henti == HNT_DEPAN) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" berhenti pada DINDING, bukan pada jarak tempuh.");
        Serial.println("  Yang disetel di sana bukan panjang ruas melainkan seberapa dekat");
        Serial.println("  robot boleh mendekat. Bacanya dari 'l' (LiDAR depan), bukan dari sini.");
        return;
    }

    _robot.profileFlat();
    _nav.setTengah(false);
    _nav.abaikanDepan(false);

    _i        = idx;
    _blkAwal  = -1.0f;
    _sebab    = nullptr;
    _serongT0 = 0;
    _tMisi    = millis();

    Serial.print("\n=== MODE UKUR -- ruas "); Serial.print(idx);
    Serial.print(": "); Serial.println(x.nama);
    Serial.print("  berangkat "); Serial.print(_nav.namaArah(MISI_ARAH_BERANGKAT));
    Serial.println(" = c0 = arah lorong pertama dari HOME");
    Serial.print("  arah "); Serial.print(_nav.namaArah(_arah[idx]));
    if (ruasSerong(idx)) { Serial.print(" serong "); Serial.print(_serong[idx], 0);
                           Serial.print(" der (heading "); Serial.print(headingRuas(idx), 0);
                           Serial.print(")"); }
    const Kemudi kmd = kemudiRuas(_i);
    Serial.print(", kemudi "); Serial.print(kmd == KMD_TENGAH ? "MENENGAH" :
                                            kmd == KMD_KIRI   ? "dinding KIRI" : "dinding KANAN");
    Serial.print(", profil ");
    Serial.println(x.profil == PRF_TANGGA   ? "TANGGA" :
                   x.profil == PRF_MERUNDUK ? "MERUNDUK" :
                   x.profil == PRF_SEMPIT   ? "SEMPIT" :
                   x.profil == PRF_KAIL     ? "KAIL" :
                   x.profil == PRF_TANJAK   ? "TANJAK" : "DATAR");
    Serial.println("  TIDAK ada syarat henti -- robot berjalan sampai ANDA menghentikannya.");
    Serial.println("  Hentikan tepat di ujung ruas: 's', Enter, atau 'm0'.");
    if (x.abaikanDepan)
        Serial.println("  ! Sensor depan DIABAIKAN di ruas ini -- robot buta ke depan, awasi.");

    _ukurMulai = MISI_UKUR;   // ditandai supaya ruasMasuk() berhenti di MISI_UKUR
    ruasMasuk();
}

void Misi::ujiLengan(bool ambil) {
    if (berjalan()) {
        Serial.println("Misi/sekuens lain sedang berjalan -- 'm0' dulu.");
        return;
    }
    if (!_robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); return; }

    _uji      = true;
    _ujiAmbil = ambil;
    masuk(MISI_LENGAN);          // masuk() yang menyetel _t0 dan _langkah

    Serial.print("\n=== SEKUENS LENGAN ");
    Serial.print(ambil ? "AMBIL" : "TARUH");
    Serial.println(" (uji, lepas dari tabel) ===");
    Serial.print("  tiap pose ");
    Serial.print(LENGAN_JEDA_MS);
    Serial.println(" ms, servo merayap 120 der/detik. 'm0' atau 's' untuk berhenti.");
}

// TUTUP JENDELA DETEKSI DI RASPI. '#LEPAS' adalah satu-satunya token yang
// sudah dikenal sisi Pi sebagai "selesai, berhentilah" -- memakainya berarti
// pembatalan bekerja tanpa Pi perlu diperbarui lebih dulu.
//
// Dipanggil dari batal() DAN gagal(). Sebelum ini keduanya meninggalkan
// kamera menyala: '#KORBAN' sudah terkirim, '#LEPAS' hanya dikirim di jalur
// SUKSES, jadi misi yang dihentikan di depan korban membuat Raspi mendeteksi
// terus tanpa ada yang akan menjawabnya.
void Misi::lepasVisi() {
    if (!_visiJalan) return;
    _visiJalan    = false;
    _parkirVision = false;
    KORBAN_SERIAL.print("#LEPAS ");
    KORBAN_SERIAL.println(_i);
}

void Misi::batal(const char* alasan) {
    _uji = false;
    lepasVisi();
    // Pola sama dengan navBerhenti(): kalau tidak ada yang berjalan, jangan
    // mencetak apa pun -- tanpa ini tiap 'x' dan Enter meninggalkan baris
    // "Misi DIBATALKAN" walau tak ada misi sama sekali.
    if (!berjalan()) { _stat = MISI_DIAM; return; }

    // Mode ukur: inilah momennya -- operator berhenti di ujung ruas, dan
    // jarak tempuh sejak ruas dimulai ADALAH panjang ruas itu. Dicetak dengan
    // penanda tetap supaya alat di PC bisa memungutnya tanpa menebak.
    bool sedangUkur = (_stat == MISI_UKUR);
    float tempuh = _robot.jarakCm() - _ruasAwal;

    _stat = MISI_DIAM;
    _ukurMulai = MISI_DIAM;
    _sebab = nullptr;
    _nav.navBerhenti(alasan);
    // navBerhenti() memulihkan abaikanDepan tapi TIDAK tengah, dan ia diam
    // saja kalau navigasi memang sudah berhenti duluan. Bereskan sendiri.
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    if (sedangUkur) {
        Serial.print("\nHASIL UKUR ruas "); Serial.print(_i);
        Serial.print(" = "); Serial.print(tempuh, 1); Serial.println(" cm");
        Serial.print("UKUR "); Serial.print(_i); Serial.print(" ");
        Serial.println(tempuh, 1);          // baris tetap untuk alat di PC
        Serial.print("  Simpan dengan: m7 "); Serial.print(_i); Serial.print(" ");
        Serial.println((int)(tempuh + 0.5f));
        return;
    }

    Serial.print("Misi DIBATALKAN di ruas "); Serial.print(_i);
    Serial.print(" ("); Serial.print(RUAS[_i].nama); Serial.print(")");
    if (alasan) { Serial.print(": "); Serial.println(alasan); } else Serial.println(".");
}

// ====================================================================
// MASUK RUAS: pivot dulu bila arahnya berbeda, baru berjalan.
// ====================================================================
void Misi::ruasMasuk() {
    const Ruas& x = RUAS[_i];

    Serial.print("\n--- RUAS "); Serial.print(_i); Serial.print(": ");
    Serial.print(x.nama); Serial.print("  ["); Serial.print(_nav.namaArah(_arah[_i]));
    if (ruasSerong(_i)) { Serial.print(" serong "); Serial.print(_serong[_i], 0); Serial.print(" der"); }
    Serial.println("]");

    // Badan sudah menghadap arah ruas ini? Kalau belum, putar dulu. Ini juga
    // yang menangani ketentuan mulai kontes: robot boleh diletakkan menghadap
    // ke mana saja, dan ruas 0 memaksanya menghadap arah berangkat sebelum
    // mode arena sempat mengunci mata angin yang salah.
    const float sasar = headingRuas(_i);
    if (isnan(sasar)) {
        lewati("mata angin ruas ini belum dicatat kompas -- 'c0'..'c3' lalu 'e', atau 'E'.");
        return;
    }
    if (!_nav.diHeading(sasar)) {
        _pivotUlang = 0;            // jatah ulang, dihitung per RUAS bukan per pivot
        _nav.pivotKe(sasar);
        if (!_nav.pivotSedangJalan()) {
            lewati("pivot masuk ruas tidak mau jalan -- sebabnya tercetak di atas ('b' bila servo lemas).");
            return;
        }
        masuk(MISI_PIVOT);
        return;
    }

    ruasBerangkat();
}

// Pasang profil gait ruas ini. true = badan harus tenang dulu sebelum
// berangkat. Hanya ruas yang berhenti pada SENSOR yang menunggu: pemicu
// jarak membaca satu angka lalu mengakhiri ruas, tanpa kesempatan kedua.
// Ruas HNT_ODO disambung tanpa berhenti.
bool Misi::pasangProfil() {
    const Ruas& x = RUAS[_i];

    switch (x.profil) {
        case PRF_TANGGA:   _robot.profileStairs(); break;
        case PRF_MERUNDUK: _robot.profileCrouch(); break;
        case PRF_SEMPIT:   _robot.profileNarrow(); break;
        case PRF_KAIL:     _robot.profileKail();   break;
        case PRF_TANJAK:   _robot.profileTanjak(); break;
        default:           _robot.profileFlat();   break;
    }

    // Keluar dari TANJAK di misi: lengan ke pose REHAT ('R'). Manual 'T0'..'T4'
    // tidak menyentuh lengan, supaya penyetelan tangan tidak terhapus.
    // PRF_SEMPIT dikecualikan -- sisa dari saat SEMPIT memakai bentuk TANJAK.
    // Sekarang SEMPIT tidak melipat lengan, jadi TANJAK -> SEMPIT langsung akan
    // meninggalkan lengan terlipat. Tabel sekarang tidak punya urutan itu.
    if (_i > 0 && RUAS[_i - 1].profil == PRF_TANJAK &&
        x.profil != PRF_TANJAK && x.profil != PRF_SEMPIT &&
        _robot.isArmed()) {
        _robot.armEnable(ARM_DEPAN, true);
        _robot.setSudutLengan(ARM_DEPAN, REHAT_BAHU, REHAT_SIKU,
                                         REHAT_PERGELANGAN, nullptr);
        Serial.println("  lengan dipulangkan ke pose REHAT (keluar dari TANJAK).");
    }

    bool pakaiSensor = (x.henti == HNT_DEPAN ||
                        x.henti == HNT_BELAKANG ||
                        x.henti == HNT_SISI ||
                        x.henti == HNT_MUNDUR ||
                        x.henti == HNT_PUNCAK);
    if (!pakaiSensor) { _pivotBaru = false; return false; }

    // Sesudah pivot, tunggu juga: sampel LiDAR yang tersimpan lahir saat badan
    // masih menghadap arah lain, dan satu sampel basi cukup untuk mengakhiri
    // ruas di tempat salah. Tunggu MISI_LIDAR_SEGAR_MS, sama dengan profil.
    if (_pivotBaru) {
        _pivotBaru = false;
        _nav.navBerhenti("menunggu sampel LiDAR yang lahir SESUDAH pivot.");
        return true;
    }
    if (_robot.gaitProfilTenang()) return false;

    // BERHENTI DULU. Ruas beruntun disambung tanpa menghentikan navigasi,
    // jadi tanpa baris ini robot menunggu sambil TETAP BERJALAN -- 1,2 detik
    // dengan pemicu jarak yang belum boleh dipercaya berarti ~15 cm terlewat,
    // persis kesalahan yang mau dicegah.
    _nav.navBerhenti("menunggu profil gait tenang sebelum membaca LiDAR.");
    return true;
}

// Berangkat menjalani RUAS[_i]: pasang profil, tunggu badan tenang bila ruas
// ini membaca sensor, baru jalan. Dipanggil dari tiga tempat -- masuk ruas
// tanpa pivot, sesudah pivot selesai, dan sesudah tunggu selesai. Yang
// terakhir aman berulang: profilnya sudah terpasang, jadi pasangProfil()
// menjawab "tidak perlu menunggu" dan jalur ini lewat sekali jalan.
void Misi::ruasBerangkat() {
    if (pasangProfil()) {
        masuk(MISI_SETEL);
        // Sebabnya sudah dicetak navBerhenti() di dalam pasangProfil(), jadi
        // di sini cukup menyebut SENSOR MANA yang ditunggu.
        Serial.print("  menunggu sebelum ");
        Serial.println(RUAS[_i].henti == HNT_SISI ? "membaca dinding samping."
                                                  : "membaca LiDAR.");
        return;
    }
    if (!ruasJalan()) return;
    masuk(_ukurMulai == MISI_UKUR ? MISI_UKUR : MISI_JALAN);
}

bool Misi::ruasJalan() {
    const Ruas& x = RUAS[_i];

    // BENDERA LANJUT DIPAKAI SEKALI, dan dihabiskan DI SINI -- bukan di cabang
    // yang kebetulan mengambilnya. Fungsi ini punya enam jalan keluar, dan
    // empat di antaranya (LANGSUNG, GESER, SISI, MUNDUR) dulu keluar tanpa
    // membersihkan benderanya. Akibatnya bukan ruas ini yang salah, melainkan
    // ruas BERIKUTNYA: ia ikut melewati penulisan titik nol, lalu mengukur
    // jarak tempuhnya dari titik nol milik ruas sebelumnya.
    const bool lanjut = _lanjutRuas;
    _lanjutRuas = false;

    _nav.setTengah(kemudiRuas(_i) == KMD_TENGAH);
    _nav.abaikanDepan(x.abaikanDepan);

    // Ruas yang cuma aksi: tidak ada yang perlu dijalankan.
    if (x.henti == HNT_LANGSUNG) return true;

    // Ruas GESER: bukan mode arena sama sekali. Navigation yang menutup
    // lupnya sendiri (syarat henti dibaca tiap tick), jadi di sini cukup
    // menyalakannya dan menunggu -- ruasSehat()/ruasSelesai() punya cabang
    // sendiri untuk membaca hasilnya.
    if (x.henti == HNT_GESER) {
        // SISA, bukan jarak penuh. Mengulang 20 cm geser berarti menempuh 40 cm
        // ke arah yang sama, dan di R-11 arah itu jurang.
        int gsr = (int)_cm[_i];
        if (lanjut) {
            const int tempuh = (int)_nav.geserTempuhCm();
            gsr -= tempuh;
            Serial.print("  geser dilanjutkan: sudah "); Serial.print(tempuh);
            Serial.print(" cm, sisa "); Serial.print(gsr); Serial.println(" cm.");
            if (gsr < 1) {     // praktis sudah sampai
                _serongT0 = 0;
                return true;
            }
        }
        if (!_nav.geserMulai(kemudiRuas(_i) == KMD_KIRI, gsr, x.jagaBelakang)) {
            lewati("geser ditolak -- sebabnya tercetak di atas.");
            return false;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        return true;
    }

    if (x.henti == HNT_SISI) {
        if (!_nav.ratakanMulai(kemudiRuas(_i) == KMD_KIRI, (int)_cm[_i], x.jagaBelakang)) {
            lewati("perataan sisi ditolak -- sebabnya tercetak di atas.");
            return false;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        return true;
    }

    // Ruas MUNDUR: sama polanya dengan GESER di atas. Navigation menutup
    // lupnya sendiri lewat setelBelakangUpdate(), jadi di sini cukup
    // menyalakannya. Mode arena TIDAK dinyalakan -- robot berjalan mundur,
    // dan kemudi dinding yang menghadap ke depan tidak punya arti di sana.
    if (x.henti == HNT_MUNDUR) {
        // TIDAK ADA DINDING DI BELAKANG: ruas ini dilewati, bukan dijalankan.
        // Sasaran mundur adalah BACAAN sensor belakang, jadi tanpa dinding di
        // dalam jangkauan tidak ada yang bisa dituju -- robot akan mundur
        // sampai batas waktu, menjauhi lintasan. Diminta R2C 18 Sep 2026 untuk
        // baris "mundur jika bisa": ruas ini memang oportunistik.
        //
        // LIDAR_MATI TIDAK ikut ke sini. Sensor yang tidak menjawab bukan
        // "tidak ada dinding", dan lewati() memang membatalkan misi untuk itu.
        const int blk = _lidar.getDistance(LIDAR_BACK);
        if (blk == LIDAR_JAUH || blk >= LIDAR_MAX_CM) {
            Serial.print("  sensor belakang JAUH (");
            Serial.print(blk); Serial.println(" cm) -- tidak ada dinding untuk dituju.");
            lewati("mundur dilewati -- tidak ada dinding di belakang.");
            return false;
        }
        if (!_nav.setelBelakangMulai((int)_cm[_i])) {
            lewati("mundur ditolak -- sebabnya tercetak di atas.");
            return false;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        return true;
    }

    ModeNav m = (kemudiRuas(_i) == KMD_KIRI) ? NAV_ARENA_KIRI : NAV_ARENA_KANAN;
    _nav.navMulai(m);
    if (_nav.navMode() != m) {
        lewati("gagal memulai navigasi untuk ruas ini -- sebabnya tercetak di atas.");
        return false;
    }

    // Dipasang LAGI sesudah navMulai(): ruas beruntun disambung tanpa
    // menghentikan navigasi, jadi navMulai() memanggil navBerhenti() yang
    // mengembalikan abaikanDepan(false). Tanpa baris ini ruas turunan berjalan
    // dengan sensor depan hidup -- berselang-seling, tergantung apakah pivot
    // sempat membuat navigasi mampir ke NAV_DIAM. Yang di atas tetap perlu:
    // ia membersihkan bendera untuk ruas yang keluar lebih dulu.
    _nav.abaikanDepan(x.abaikanDepan);

    // Sasaran kemudi fase jalan. Tanpa ini mode arena mengunci ke mata angin
    // TERDEKAT dari yaw sekarang, dan pada ruas menyerong 45 der jarak ke dua
    // mata angin sama besar: pilihannya lemparan koin, dan yang salah menarik
    // badan 90 der dari yang dimaksud sepanjang ruas. Dipasang SESUDAH
    // navMulai() karena navBerhenti() yang dipanggilnya melepas kunci ini,
    // persis seperti yang terjadi pada abaikanDepan di atas.
    _nav.kunciHeading(headingRuas(_i));

    // MELANJUTKAN, bukan mengulang: titik nol dan penanda gyro DIBIARKAN apa
    // adanya. Tidak ada perhitungan tambahan di sini -- yang dikerjakan justru
    // MELEWATI penulisan ulang. Jarak yang sudah ditempuh tetap terhitung.
    //
    // Yang tidak ikut: HNT_SISI, HNT_DEPAN, HNT_BELAKANG dan HNT_MUNDUR
    // sasarannya BACAAN MUTLAK sensor, jadi mereka melanjutkan sendiri tanpa
    // perlu diberi tahu. Yang benar-benar butuh ini cuma yang menghitung
    // TEMPUH: HNT_ODO lewat _ruasAwal, dan HNT_GESER lewat sisa jaraknya.
    if (lanjut) {
        Serial.print("  ruas DILANJUTKAN dari ");
        Serial.print(_robot.jarakCm() - _ruasAwal, 0);
        Serial.println(" cm yang sudah ditempuh.");
        _serongT0 = 0;
        return true;
    }

    _ruasAwal = _robot.jarakCm();
    _serongT0 = 0;

    // ACUAN PITCH HNT_PUNCAK, dicatat di sini bersama titik nol yang lain.
    // Badan sudah menghadap arah ruas dan profil gaitnya sudah terpasang, jadi
    // kemiringan yang diperintahkan TANJAK_PITCH_DEG sudah ikut terbaca --
    // itulah yang membuat selisih terhadap angka ini bersih dari kemiringan
    // yang kita sendiri minta.
    _pitchAwal    = _nav.imuBicara() ? _nav.pitchKini() : NAN;
    _naikTerlihat = false;
    _datarT0      = 0;

    // Titik nol jarak tempuh, dicatat SEKARANG: badan sudah lurus menghadap
    // arah ruas, jadi berkas sensor belakang tegak lurus dinding START dan
    // angkanya jarak yang sebenarnya. Badan yang masih menyerong membuat
    // berkasnya memanjang 1/cos(sudut) -- titik nol yang salah.
    if (x.henti == HNT_BELAKANG) {
        int b0 = _lidar.getDistance(LIDAR_BACK);
        if (b0 == LIDAR_MATI || b0 == LIDAR_JAUH) {
            lewati("dinding START tidak terbaca sensor belakang -- titik nol tak bisa dicatat.");
            return false;
        }
        _blkAwal = (float)b0;
        Serial.print("  titik nol "); Serial.print(_blkAwal, 1);
        Serial.print(" cm -> berhenti di bacaan "); Serial.print(_blkAwal + _cm[_i], 1);
        Serial.println(" cm");
    }
    return true;
}

// ====================================================================
// PENJAGA SELAMA RUAS BERJALAN
// ====================================================================
bool Misi::ruasSehat() {
    // Ruas GESER punya penjaganya sendiri DI DALAM Navigation (sensor buta,
    // halangan di arah geser, batas waktu), dan tidak satu pun penjaga di
    // bawah berlaku untuknya: ia tidak memakai mode arena, tidak menempuh
    // jarak maju, dan memang tidak menghadap arah ruas selama bergeser.
    if (RUAS[_i].henti == HNT_SISI || RUAS[_i].henti == HNT_GESER) {
        if (_nav.ratakanSedangJalan()) return true;
        if (!_nav.ratakanTercapai()) {
            lewati("perataan sisi berhenti sebelum sasaran -- sebabnya tercetak di atas.");
            return false;
        }
        return true;
    }

    // Ruas MUNDUR: alasannya sama persis dengan GESER di atas. Penjaganya ada
    // di dalam NAV_SETEL_BLK (sensor belakang mati di tengah jalan, halangan,
    // MUNDUR_BATAS_MS), dan tidak satu pun penjaga mode arena di bawah
    // berlaku -- robot memang tidak menghadap arah jalannya saat mundur.
    if (RUAS[_i].henti == HNT_MUNDUR) {
        if (_nav.setelBelakangSedangJalan()) return true;
        if (!_nav.setelBelakangTercapai()) {
            lewati("mundur berhenti sebelum sasaran -- sebabnya tercetak di atas.");
            return false;
        }
        return true;
    }

    // 1) Navigasi masih milik kita? Menangkap navigasi yang berhenti sendiri
    //    DAN yang diambil alih dari serial, tanpa kait di parser. Wajib
    //    kemudiRuas(), bukan kolom mentah -- di mode cermin keduanya berbeda.
    ModeNav m = (kemudiRuas(_i) == KMD_KIRI) ? NAV_ARENA_KIRI : NAV_ARENA_KANAN;
    if (_nav.navMode() != m) {
        lewati("navigasi berhenti atau diambil alih -- sebabnya tercetak di atas.");
        return false;
    }

    if (lewat() > MISI_RUAS_BATAS_MS) {
        _nav.navBerhenti("batas waktu ruas.");
        lewati("ruas tidak selesai dalam batas waktunya.");
        return false;
    }

    // Pagar jarak -- lihat MISI_RUAS_MAKS_CM. Berjalan lebih jauh dari ini
    // berarti syarat hentinya tidak akan pernah terpenuhi, apa pun sebabnya.
    const float tempuhRuas = _robot.jarakCm() - _ruasAwal;
    if (tempuhRuas > MISI_RUAS_MAKS_CM) {
        _nav.navBerhenti("pagar jarak ruas.");
        Serial.print("  sudah menempuh "); Serial.print(tempuhRuas, 1);
        Serial.print(" cm, pagar "); Serial.print(MISI_RUAS_MAKS_CM, 0);
        Serial.println(" cm.");
        lewati("ruas berjalan jauh melampaui ukuran arena -- syarat hentinya tidak tercapai.");
        return false;
    }

    // 2) Navigasi memilih mata angin LAIN. Itu 90 der sekaligus, bukan
    //    goyangan, dan tiap langkah sesudahnya dihitung ke arah yang salah.
    //    Tidak ada gunanya menunggu.
    if (_nav.arahDituju() != (int8_t)_arah[_i]) {
        _nav.navBerhenti("navigasi berbelok ke mata angin lain di tengah ruas.");
        lewati("navigasi berpindah arah -- sisa ruas akan diukur ke arah yang salah.");
        return false;
    }

    // 3) Badan terserong sementara sambil tetap MENUJU arah yang sama. Di
    //    lantai pecah itu normal: kaki tergelincir, mode arena menariknya
    //    kembali selewat beberapa langkah. Yang ditangkap hanya serong BESAR
    //    yang BERTAHAN -- misalnya robot menyangkut sehingga koreksinya tidak
    //    pernah menang.
    float simpang = _nav.simpangHeading(headingRuas(_i));
    if (isnan(simpang)) {
        _nav.navBerhenti("IMU tidak memberi data di tengah ruas.");
        lewati("acuan heading hilang -- jarak tempuh tidak bisa dipercaya.");
        return false;
    }
    if (fabsf(simpang) <= MISI_SERONG_DEG) {
        _serongT0 = 0;
    } else {
        uint32_t batas = (uint32_t)(MISI_SERONG_SIKLUS * _robot.gaitProfile().cycleTime);
        if (batas < MISI_SERONG_MIN_MS) batas = MISI_SERONG_MIN_MS;
        if (_serongT0 == 0) _serongT0 = millis();
        else if (millis() - _serongT0 > batas) {
            _nav.navBerhenti("heading menyimpang di tengah ruas.");
            Serial.print("  serong "); Serial.print(simpang, 1);
            Serial.print(" der bertahan lebih dari "); Serial.print(batas);
            Serial.println(" ms.");
            lewati("robot tidak lagi menghadap arah ruas -- jarak tempuh tidak bisa dipercaya.");
            return false;
        }
    }
    return true;
}

// ====================================================================
// SYARAT HENTI
// ====================================================================
bool Misi::ruasSelesai() {
    const Ruas& x = RUAS[_i];

    if (x.henti == HNT_LANGSUNG) return true;
    if (x.henti == HNT_ODO)      return (_robot.jarakCm() - _ruasAwal) >= _cm[_i];

    // Navigation sudah berhenti sendiri begitu sasaran tercapai; kalau ia
    // berhenti karena GAGAL, ruasSehat() sudah membatalkan misi lebih dulu.
    if (x.henti == HNT_SISI ||
        x.henti == HNT_GESER)    return !_nav.ratakanSedangJalan();
    if (x.henti == HNT_MUNDUR)   return !_nav.setelBelakangSedangJalan();

    if (x.henti == HNT_PUNCAK) {
        // Dinding depan hanya berlaku kalau `nilai` > 0. Di R-9 dimatikan (berkas
        // mengenai muka anak tangga); dibiarkan sebagai pilihan untuk tanjakan
        // landai, sebagai penjaring kalau gyro meleset.
        const int dP = _lidar.getDistance(LIDAR_FRONT);
        // MATI dan JAUH bukan "sudah sampai" -- yang satu sensor putus, yang
        // lain lorong masih terbuka.
        const bool dindingDekat = (_cm[_i] > 0.0f &&
                                   dP != LIDAR_MATI && dP != LIDAR_JAUH &&
                                   (float)dP <= _cm[_i]);

        // IMU BISU: gerbang mendaki tidak bisa dipakai, jadi tinggal dinding.
        // Menunggu gyro yang tidak pernah datang berarti berjalan sampai batas
        // waktu ruas, di tangga.
        if (isnan(_pitchAwal) || !_nav.imuBicara()) {
            if (dindingDekat) {
                Serial.print("  PUNCAK: IMU bisu, berhenti dari dinding depan ");
                Serial.print(dP); Serial.println(" cm.");
            }
            return dindingDekat;
        }

        const float simpang = fabsf(_nav.pitchKini() - _pitchAwal);

        // GERBANG MENDAKI. Selama robot belum pernah benar-benar miring, ruas
        // ini TIDAK BOLEH berakhir -- di kaki tangga badan memang datar, dan
        // berkas depan yang menyentuh muka anak tangga pertama akan
        // mengakhirinya juga.
        if (!_naikTerlihat) {
            if (simpang >= PUNCAK_NAIK_DEG) {
                _naikTerlihat = true;
                Serial.print("  PUNCAK: mendaki terlihat, simpang ");
                Serial.print(simpang, 1); Serial.println(" der.");
            }
            return false;
        }

        if (dindingDekat) {
            Serial.print("  PUNCAK: dinding depan "); Serial.print(dP);
            Serial.print(" cm <= "); Serial.print(_cm[_i], 0);
            Serial.println(" cm -- ruas selesai.");
            return true;
        }

        // DATAR LAGI, dan harus BERTAHAN. Satu sampel datar muncul juga di
        // tengah tanjakan tiap kali kaki depan menapak anak tangga berikutnya.
        if (simpang > PUNCAK_DATAR_DEG) { _datarT0 = 0; return false; }
        if (_datarT0 == 0) { _datarT0 = millis(); return false; }
        if (millis() - _datarT0 < PUNCAK_DATAR_MS) return false;

        Serial.print("  PUNCAK: datar lagi ("); Serial.print(simpang, 1);
        Serial.print(" der) selama "); Serial.print(PUNCAK_DATAR_MS);
        Serial.println(" ms -- ruas selesai.");
        return true;
    }

    uint8_t kanal = (x.henti == HNT_DEPAN) ? LIDAR_FRONT : LIDAR_BACK;
    int     d     = _lidar.getDistance(kanal);

    // Tiga keadaan, tiga perlakuan:
    //   MATI  jangan hitung apa pun; ruasSehat() yang menangkapnya.
    //   JAUH  depan: lorong masih terbuka. Belakang: dinding START hilang --
    //         memicu dari situ berarti berhenti di tempat acak.
    if (d == LIDAR_MATI || d == LIDAR_JAUH) { _n = 0; return false; }

    // Hitung HANYA saat ada sampel BARU. update() dipanggil ribuan kali per
    // detik, sedangkan tiap sensor cuma menghasilkan satu pengukuran tiap
    // ~150 ms (round-robin enam channel). Tanpa gerbang ini, "3 sampel
    // berturut-turut" berarti tiga iterasi loop yang membaca angka yang sama.
    uint32_t st = _lidar.stempelSampel(kanal);
    if (st == _stempel) return false;
    _stempel = st;

    bool kena = (x.henti == HNT_DEPAN) ? ((float)d <= _cm[_i])
                                       : ((float)d >= _blkAwal + _cm[_i]);
    if (!kena) { _n = 0; return false; }
    return (++_n >= MISI_SAMPEL_N);
}

// ====================================================================

bool Misi::lidarMati() {
    // NUM_LIDAR, bukan cuma kanal yang dipakai ruas ini. Sensor yang berhenti
    // menjawab hampir tidak pernah sendirian -- mux, catu, atau kabel bersama.
    // Ruas berikutnya yang memakainya akan menemuinya juga, dan menemukannya
    // di tengah tanjakan lebih mahal daripada berhenti sekarang.
    return _lidar.jumlahHidup() < NUM_LIDAR;
}

bool Misi::pulihkanLidar() {
    if (_lidarUlang >= LIDAR_ULANG_MAKS) {
        Serial.print("  sudah "); Serial.print(_lidarUlang);
        Serial.println(" kali dipindai ulang dan tetap putus -- menyerah.");
        return false;
    }
    _lidarUlang++;

    // BERHENTI DULU. pindaiI2C() memblokir beberapa ratus milidetik sambil
    // mematikan dan menghidupkan tiap kanal mux. Robot yang masih melangkah
    // selama itu berjalan buta, dan yang menghentikannya cuma ruas berikutnya
    // -- terlambat.
    _nav.navBerhenti("LiDAR dipindai ulang.");

    Serial.print("\n!! LiDAR PUTUS di tengah misi -- pindai ulang (");
    Serial.print(_lidarUlang); Serial.print(" dari ");
    Serial.print(LIDAR_ULANG_MAKS); Serial.println(")");

    // false = jangan memulai ulang peripheral I2C walau mux bisu. Lihat
    // LidarArray::pindaiI2C(): pemulihan bus adalah langkah diagnostik, dan
    // di tengah lari ia memutus bus yang sedang dipakai sambil menahan lup
    // utama lebih lama lagi.
    _lidar.pindaiI2C(false);

    const bool pulih = !lidarMati();
    _lanjutRuas = pulih;
    Serial.println(pulih ? "  PULIH."
                         : "  masih ada yang tidak menjawab.");
    return pulih;
}

void Misi::lewati(const char* sebab) {
    // LiDAR MATI MENANG atas 'lewati'. Sebab lunak apa pun yang terjadi
    // bersamaan dengan sensor yang berhenti menjawab bukan lagi sebab lunak:
    // ruas berikutnya akan gagal dengan cara yang sama, dan meneruskan cuma
    // memindahkan kegagalan ke tempat yang lebih sulit dibaca.
    if (lidarMati()) {
        // Robot tidak boleh disentuh saat lomba, jadi ia mencoba sendiri dulu.
        // Berhasil: ruas ini DIULANG dari awal, bukan dilewati -- ia berhenti
        // karena sensornya berkedip, bukan karena ruasnya tidak bisa dikerjakan.
        if (pulihkanLidar()) {
            Serial.println("  CATATAN: odometri ruas ini dihitung dari nol lagi,");
            Serial.println("  jadi jarak yang sudah ditempuh akan ditempuh dua kali.");
            ruasMasuk();
            return;
        }
        gagal("LiDAR tidak merespons -- periksa 'l' lalu 'I'.");
        return;
    }
    // Mode UKUR ('m7') dipakai operator untuk membaca satu ruas, bukan untuk
    // menyelesaikan lintasan. Maju diam-diam ke ruas berikutnya di sana
    // menghilangkan justru yang sedang diamati.
    if (_stat == MISI_UKUR) { gagal(sebab); return; }
    Serial.print("\n== RUAS "); Serial.print(_i);
    Serial.print(" DILEWATI ("); Serial.print(RUAS[_i].nama); Serial.println(")");
    Serial.print("   sebab: "); Serial.println(sebab);
    Serial.println("   Misi DITERUSKAN ke ruas berikutnya. Ruas ini TIDAK berpoin.");

    _nav.navBerhenti("ruas dilewati.");
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    ruasBerikut(false);
}

void Misi::ruasBerikut(bool berpoin) {
    // POIN DICATAT DI SINI, dan cuma di sini. Ini satu-satunya tempat sebuah
    // ruas dinyatakan SELESAI. Ruas yang gagal atau dibatalkan tidak pernah
    // lewat sini; ruas yang DILEWATI lewat sini dengan berpoin=false.
    if (berpoin) gSkor.ruasSelesai(_i);

    _i++;
    if (_i >= RUAS_N || _i > _iAkhir) {
        _nav.navBerhenti("lintasan selesai.");
        _nav.abaikanDepan(false);
        _nav.setTengah(false);
        _robot.profileFlat();
        masuk(MISI_SELESAI);
        Serial.println(_iAkhir >= RUAS_N - 1
                       ? "\n=== FINISH -- seluruh lintasan selesai ==="
                       : "\n=== BERHENTI di ruas akhir yang diminta ===");
        Serial.print("  waktu total: "); Serial.print(lewat() / 1000);
        Serial.println(" detik.");
        return;
    }
    ruasMasuk();
}

void Misi::update() {
    if (!berjalan()) return;

    switch (_stat) {

    case MISI_PIVOT: {
        // Navigation punya timeout PIVOT_BATAS_MS sendiri, jadi tinggal
        // menunggu ia berhenti lalu MEMERIKSA hasilnya: pivot yang selesai
        // dan pivot yang timeout/dibatalkan sama-sama berakhir di NAV_DIAM,
        // jadi heading akhir yang membedakannya.
        if (_nav.pivotSedangJalan()) return;
        const float sasar = headingRuas(_i);
        if (!_nav.diHeading(sasar)) {
            // SEKALI ULANG sebelum menyerah. Percobaan kedua berangkat dari
            // heading yang sudah lebih dekat dan mendapat jatah
            // PIVOT_BATAS_MS yang baru, jadi pivot yang cuma KEHABISAN WAKTU
            // -- kaki terganjal, lantai licin, badan mentok dinding sebentar
            // -- sering selesai di percobaan itu. Yang TIDAK ditolong:
            // kompas belum tercatat, IMU lepas, atau c0..c3 yang memang
            // salah; ketiganya meleset dengan simpangan yang sama besar dua
            // kali, dan angka yang tercetak di bawah yang membedakannya.
            float simpang = _nav.simpangHeading(sasar);
            if (_pivotUlang == 0) {
                _pivotUlang = 1;
                Serial.print("  pivot meleset "); Serial.print(simpang, 1);
                Serial.println(" der -- MENGULANG sekali.");
                _nav.pivotKe(sasar);
                if (!_nav.pivotSedangJalan()) {
                    lewati("pivot ulangan tidak mau jalan -- sebabnya tercetak di atas.");
                    return;
                }
                masuk(MISI_PIVOT);
                return;
            }
            Serial.print("  ulangan pun meleset "); Serial.print(simpang, 1);
            Serial.println(" der.");
            lewati("pivot tidak sampai walau sudah diulang -- periksa 'k' (kompas c0..c3) dan IMU.");
            return;
        }
        // Badan baru saja berputar. Sampel LiDAR yang ada sekarang lahir dari
        // heading lama, jadi ruas yang berhenti pada sensor harus menunggu
        // berkasnya berganti isi dulu -- lihat pasangProfil().
        _pivotBaru = true;
        ruasBerangkat();
        break;
    }

    case MISI_SETEL: {
        // Navigasi sudah diam (pasangProfil() yang menghentikannya), jadi
        // penjaga servo milik Navigation tidak berlaku -- jaga sendiri, sama
        // seperti MISI_LENGAN dan MISI_KONFIRM.
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu profil gait."); return; }

        if (!_robot.gaitProfilTenang()) {
            if (lewat() > MISI_SETEL_BATAS_MS) {
                gagal("ramp profil gait tidak kunjung selesai -- gait tidak di-update?");
                return;
            }
            _tenangT0 = 0;      // sempat tenang lalu bergerak lagi -> hitung dari nol
            return;
        }
        if (_tenangT0 == 0) _tenangT0 = millis();
        if (millis() - _tenangT0 < MISI_LIDAR_SEGAR_MS) return;

        ruasBerangkat();
        break;
    }

    case MISI_JALAN: {
        if (RUAS[_i].henti != HNT_LANGSUNG && !ruasSehat()) return;
        if (!ruasSelesai()) return;

        const Ruas& x = RUAS[_i];

        // Berhenti HANYA kalau ada yang harus dikerjakan di tempat. Ruas
        // beruntun yang searah disambung tanpa berhenti: menghentikan lalu
        // menjalankan lagi tiap ganti profil membuat robot tersendat di
        // perbatasan, dan perbatasan itulah bibir rintangan.
        if (x.aksi == AKS_TIDAK_ADA) { ruasBerikut(); return; }

        _nav.navBerhenti("ujung ruas tercapai.");
        _nav.abaikanDepan(false);

        if (x.aksi == AKS_KONFIRM) {
            masuk(MISI_KONFIRM);
            Serial.println("\n=== BERHENTI -- MENUNGGU KEPUTUSAN OPERATOR ===");
            Serial.println("  'm2' = lanjut  |  'm3' = ulangi ruas ini  |  'm0' = batal");
            return;
        }

        // Pemicu ke Raspi 5: robot sudah diam di depan korban, silakan
        // mulai deteksi. Tempatnya di sini dan bukan di MISI_LENGAN karena
        // cabang ini cuma dilewati SEKALI per ruas -- MISI_LENGAN dipanggil
        // tiap loop. Kirim saja, tidak menunggu jawaban: TAHAP 1 memang tidak
        // memakai balasannya -- pelurusan kiri-kanan dikerjakan kamera SEBELUM
        // ruas ini, dan jarak datang dari LiDAR depan lewat HNT_DEPAN.
        KORBAN_SERIAL.print("#KORBAN ");
        KORBAN_SERIAL.print(x.aksi == AKS_AMBIL ? "AMBIL " : "TARUH ");
        KORBAN_SERIAL.print(_i);
        // Arah mata angin ruas (0..3) dan saklar cermin, kolom tambahan untuk fase
        // CARI di Raspi. _arah[] sudah hasil pencerminan, jadi aturan cermin tidak
        // perlu disalin ke sisi lain. Pembaca lama yang cuma mengambil dua kolom
        // pertama tetap bekerja.
        KORBAN_SERIAL.print(' '); KORBAN_SERIAL.print(_arah[_i]);
        KORBAN_SERIAL.print(' '); KORBAN_SERIAL.println(arenaCermin() ? 1 : 0);
        _visiJalan = true;      // ditutup '#LEPAS', termasuk lewat lepasVisi()

        // PARKIR, bukan langsung menurunkan capit. Ini yang dikembalikan
        // 15 Sep 2026 sesudah v1.15 membuangnya: tanpa parkir, Raspi melihat
        // korban tapi tidak boleh menggerakkan badan -- dua penguasa untuk
        // satu robot -- jadi ia hanya menonton sampai nama ruasnya berganti.
        // Dari luar: robot melihat korban lalu pergi.
        //
        // Hanya untuk AKS_AMBIL: menaruh tidak butuh kamera, titik lepasnya
        // ditentukan tabel.
        if (_tungguVision && x.aksi == AKS_AMBIL) {
            _parkirVision = true;
            _yawUjung = _nav.yawKini();
            masuk(MISI_KONFIRM);
            Serial.println("\n=== PARKIR UNTUK VISION -- menunggu 'm2' dari Raspi ===");
            Serial.println("  Kendali kaki MILIK RASPI sampai 'm2'. Ia menengahkan");
            Serial.println("  badan ke korban, menahannya, lalu menjawab.");
            Serial.println("  'm2' = lanjut ke sekuens capit  |  'm3' = ulangi ruas");
            Serial.println("  'm0' = batal  |  'm8 0' = matikan mode tunggu ini");
            return;
        }

        // Titik nol pengukuran serong. Diambil SESUDAH navBerhenti(), jadi
        // yang terukur sesudahnya murni apa yang terjadi saat gait berhenti
        // dan saat lengan bergerak -- bukan kemudi ruas yang baru saja usai.
        _yawUjung = _nav.yawKini();

        masuk(MISI_LENGAN);
        Serial.print("\n=== "); Serial.print(x.aksi == AKS_AMBIL ? "ANGKAT" : "TARUH");
        Serial.print(" KORBAN -- ruas "); Serial.print(_i); Serial.println(" ===");
        if (LENGAN_KORBAN_AKTIF)
            Serial.printf("  pose sendi tetap: bahu %.0f, siku %.0f, pergelangan %.0f der\n",
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_BAHU : KORBAN_LEPAS_BAHU),
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_SIKU : KORBAN_LEPAS_SIKU),
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_PRG  : KORBAN_LEPAS_PRG));
        else
            Serial.println("  DILEWATI -- LENGAN_KORBAN_AKTIF 0 di config.h");
        break;
    }

    case MISI_LENGAN: {
        // Navigasi sudah diam di sini (kita yang menghentikannya), jadi
        // penjaga servo milik Navigation tidak lagi berlaku. Jaga sendiri.
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat sekuens lengan."); return; }

        // Uji operator: pose yang sama, tapi tidak ada ruas yang menunggu di
        // ujungnya dan tidak ada capit yang perlu dicatat isinya.
        if (_uji) {
            const bool selesai = _ujiAmbil
                                     // mundurMm 0: uji operator dijalankan di
                                     // meja tanpa korban, dan menarik badan
                                     // mundur di sana cuma menyembunyikan
                                     // bentuk busur turun yang mau dilihat.
                                     ? sekuensAmbil(_robot, ARM_DEPAN, _langkah, _t0,
                                                    false, 0.0f, 0.0f, 0.0f)
                                     : sekuensTaruh(_robot, ARM_DEPAN, _langkah, _t0);
            if (!selesai) return;
            _uji  = false;
            _stat = MISI_DIAM;
            Serial.println("Sekuens lengan selesai.");
            return;
        }

        // Sebelum satu servo lengan pun bergerak: berapa derajat badan sudah
        // berpindah sejak ruas berhenti? Itu sumbangan GAIT, bukan lengan.
        if (_langkah == 0 && !isnan(_yawUjung))
            Serial.printf("  serong sejak ruas berhenti, SEBELUM lengan: %+.1f der\n",
                          (double)selisihYaw());

        const Ruas& x = RUAS[_i];
        // Saklar config.h. Hubung-singkat, jadi dengan saklar mati sekuensnya
        // tidak pernah dipanggil -- tapi tetap DIRUJUK, sehingga kompilator
        // tidak mengeluh soal fungsi tak terpakai dan isi sekuens tetap
        // ikut diperiksa saat kompilasi.
        const bool usai = !LENGAN_KORBAN_AKTIF ||
                          ((x.aksi == AKS_AMBIL)
                               ? sekuensAmbil(_robot, x.lengan, _langkah, _t0,
                                              x.condong, koreksiYawRuas(),
                                              x.mundurMm, x.condongMm)
                               : sekuensTaruh(_robot, x.lengan, _langkah, _t0));
        if (!usai) return;

        // Tidak diperbarui saat dilewati: capit benar-benar kosong, dan
        // ringkasan 'm1' tidak boleh berbohong soal itu.
        if (!isnan(_yawUjung)) {
            Serial.printf("  serong sejak ruas berhenti, SESUDAH lengan: %+.1f der\n",
                          (double)selisihYaw());
            _yawUjung = NAN;
        }

        if (LENGAN_KORBAN_AKTIF) _korban[x.lengan & 1] = (x.aksi == AKS_AMBIL);

        // SERAH-TERIMA BALIK. Capit selesai, tapi Raspi bisa MASIH memegang
        // kaki: ia menahan pose badan yang dipakai menengahkan tadi.
        // Melangkah sekarang berarti Teensy meng-override penguasa yang belum
        // melepas, dan pose yang masih berlaku menyentak badan di langkah
        // pertama. Jadi: beri tahu, lalu TUNGGU jawabannya.
        //
        // Hanya untuk ruas yang tadi memang diparkir ke vision. Ruas TARUH
        // tidak pernah menyerahkan kaki.
        if (_parkirVision) {
            _parkirVision = false;
            _visiJalan    = false;
            KORBAN_SERIAL.print("#LEPAS ");
            KORBAN_SERIAL.println(_i);
            masuk(MISI_LEPAS);
            Serial.println("\n=== CAPIT SELESAI -- menunggu 'm9' dari Raspi ===");
            Serial.println("  'm9' = Raspi sudah tidak memegang kaki, misi boleh lanjut.");
            return;
        }

        ruasBerikut();
        break;
    }

    case MISI_LEPAS: {
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu 'm9'."); return; }
        if (lewat() <= MISI_LEPAS_BATAS_MS) return;   // masih menunggu Raspi

        // Habis waktu TIDAK menggagalkan misi. Raspi yang mati atau kabel
        // serial yang lepas tidak boleh membuat robot berdiri diam sampai jam
        // kontes habis sambil menggenggam korban yang sudah berhasil diangkat.
        Serial.print("!! 'm9' tidak datang dalam ");
        Serial.print(MISI_LEPAS_BATAS_MS / 1000); Serial.println(" detik.");
        Serial.println("   Misi DILANJUTKAN. Kalau Raspi ternyata masih menahan pose");
        Serial.println("   badan, langkah pertama akan menyentak -- periksa HUD.");
        ruasBerikut();
        break;
    }

    case MISI_UKUR:
        // Penjaganya tetap berlaku -- arah, serong, navigasi diambil alih.
        // Yang TIDAK ada cuma syarat hentinya; itu tugas operator.
        ruasSehat();
        break;

    case MISI_KONFIRM:
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu konfirmasi."); return; }

        // Batas waktu HANYA untuk parkir vision. Ruas AKS_KONFIRM biasa
        // menunggu keputusan MANUSIA, dan manusia tidak boleh di-timeout --
        // ia mungkin sedang mengukur sesuatu.
        if (_parkirVision && lewat() > MISI_VISI_BATAS_MS) {
            _parkirVision = false;
            Serial.print("!! Raspi tidak menjawab dalam ");
            Serial.print(MISI_VISI_BATAS_MS / 1000); Serial.println(" detik.");
            Serial.println("   Sekuens capit DIJALANKAN memakai sudut tetap -- sama");
            Serial.println("   dengan perilaku tanpa vision. Korban tidak hilang karena ini.");
            masuk(MISI_LENGAN);
        }
        break;

    default:
        break;
    }

    // Jam kontes, diperiksa paling akhir supaya ruas yang baru saja selesai
    // tetap tercatat sebelum misinya ditutup. _t0 tidak bisa dipakai untuk
    // ini -- ia di-reset tiap masuk state.
    if (berjalan() && millis() - _tMisi > MISI_TOTAL_BATAS_MS) {
        _nav.navBerhenti("batas waktu kontes 5 menit.");
        gagal("waktu kontes habis (300 detik).");
    }
}

void Misi::jawab(bool lanjut) {
    if (_stat != MISI_KONFIRM) {
        Serial.println("Tidak sedang menunggu konfirmasi. 'm' untuk status.");
        return;
    }
    // DUA UJUNG, dan membedakannya WAJIB. Parkir vision harus lanjut ke
    // sekuens CAPIT; ruas AKS_KONFIRM biasa harus lanjut ke ruas BERIKUTNYA.
    // Menyamakan keduanya berarti 'm2' melompati pengambilannya dan robot
    // berjalan ke ruas berikutnya dengan capit kosong, tanpa satu pun pesan.
    if (_parkirVision) {
        if (lanjut) {
            Serial.println("Dicatat: LANJUT -- kendali kaki kembali ke Teensy.");
            masuk(MISI_LENGAN);       // _parkirVision tetap: dipakai MISI_LENGAN
            return;                   // untuk tahu ia harus mengirim '#LEPAS'
        }
        _parkirVision = false;
        Serial.println("Dicatat: ULANGI ruas ini.");
        ruasMasuk();
        return;
    }

    if (lanjut) { Serial.println("Dicatat: LANJUT."); ruasBerikut(); return; }
    Serial.println("Dicatat: ULANGI ruas ini.");
    ruasMasuk();
}

void Misi::lepasKendali() {
    if (_stat != MISI_LEPAS) {
        Serial.println("Tidak sedang menunggu 'm9'. 'm' untuk status.");
        return;
    }
    Serial.println("Dicatat: Raspi melepas kaki. Misi dilanjutkan.");
    ruasBerikut();
}

void Misi::setTungguVision(int mode) {
    _tungguVision = (mode < 0) ? !_tungguVision : (mode != 0);
    Serial.print("Tunggu vision: ");
    Serial.println(_tungguVision ? "NYALA -- ruas AMBIL parkir menunggu 'm2' dari Raspi"
                                 : "MATI -- capit turun tanpa menunggu kamera");
    if (!_tungguVision && _parkirVision && _stat == MISI_KONFIRM) {
        // Dimatikan SELAGI parkir. Melanjutkan parkir yang sudah tidak diakui
        // siapa pun berarti robot berdiri diam sampai batas waktunya habis.
        _parkirVision = false;
        Serial.println("  Sedang parkir -- dilepas sekarang, capit turun.");
        masuk(MISI_LENGAN);
    }
}

void Misi::setRuasCm(uint8_t idx, float cm) {
    if (idx >= RUAS_N) {
        Serial.print("Ruas "); Serial.print(idx); Serial.print(" tidak ada -- hanya 0..");
        Serial.println(RUAS_N - 1);
        return;
    }
    if (RUAS[idx].henti == HNT_LANGSUNG) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" tidak berjalan (hanya aksi) -- tidak ada panjang untuk disetel.");
        return;
    }

    // Batas atas & bawah menurut CARA ruas itu berhenti, bukan satu rentang
    // untuk semuanya: ambang sensor punya batas fisik yang berbeda dari
    // panjang odometri.
    float lo = 5.0f, hi = 400.0f;
    if (RUAS[idx].henti == HNT_DEPAN)    { lo = (float)FRONT_STOP_CM + 1.0f; hi = 200.0f; }
    // PUNCAK memakai sensor DEPAN, jadi batasnya sama persis dengan HNT_DEPAN.
    if (RUAS[idx].henti == HNT_PUNCAK)   { lo = (float)FRONT_STOP_CM + 1.0f; hi = 200.0f; }
    if (RUAS[idx].henti == HNT_BELAKANG) { hi = (float)LIDAR_MAX_CM - 15.0f; }
    if (RUAS[idx].henti == HNT_SISI)     { lo = WALL_MIN_CM + 1.0f; hi = (float)LIDAR_MAX_CM; }
    // GESER: jarak tempuh menyamping, bukan bacaan dinding. Batas atasnya
    // lebar arena, bukan jangkauan LiDAR.
    if (RUAS[idx].henti == HNT_GESER)    { lo = 1.0f; hi = 200.0f; }
    // MUNDUR: batas bawahnya sama dengan yang dipakai setelBelakangMulai()
    // untuk menolak sasaran -- kalau berbeda, 'm7' akan menerima angka yang
    // nanti ditolak lagi di arena, sesudah robot terlanjur berangkat.
    if (RUAS[idx].henti == HNT_MUNDUR)   { lo = WALL_MIN_CM + 1.0f; hi = (float)LIDAR_MAX_CM; }

    float v = clampf(cm, lo, hi);
    Serial.print("ruas "); Serial.print(idx); Serial.print(" ("); Serial.print(RUAS[idx].nama);
    Serial.print("): ");
    if (_cm[idx] < 0.0f) Serial.print("belum diukur"); else Serial.print(_cm[idx], 1);
    Serial.print(" -> "); Serial.print(v, 1); Serial.println(" cm");
    if (fabsf(v - cm) > 1e-3f) {
        Serial.print("  (diminta "); Serial.print(cm, 1); Serial.print(", DI-CLAMP ke ");
        Serial.print(lo, 0); Serial.print(" .. "); Serial.print(hi, 0); Serial.println(")");
    }
    _cm[idx] = v;
    Serial.println("  Hanya di RAM -- belum ada slot EEPROM untuk parameter misi.");
}

void Misi::cetakRuas(uint8_t idx) {
    const Ruas& x = RUAS[idx];
    Serial.print(idx == _i && berjalan() ? " >" : "  ");
    if (idx < 10) Serial.print(' ');
    Serial.print(idx); Serial.print("  ");
    Serial.print(_nav.namaArah(_arah[idx]));
    for (uint8_t k = strlen(_nav.namaArah(_arah[idx])); k < 8; k++) Serial.print(' ');
    // YANG DICETAK ADALAH YANG BERLAKU, bukan yang tertulis di tabel. Di mode
    // cermin keduanya berbeda, dan tabel yang menampilkan sumber sementara
    // robot menjalankan cerminnya adalah tabel yang menyesatkan justru saat
    // paling dibutuhkan.
    switch (belokRuas(idx)) {
        case BLK_KANAN: Serial.print("kanan"); break;
        case BLK_KIRI:  Serial.print("kiri "); break;
        case BLK_BALIK: Serial.print("balik"); break;
        default:        Serial.print("lurus"); break;
    }
    // Serong dicetak MENEMPEL pada beloknya, bukan di kolom sendiri: ia
    // memang bagian dari belok itu, cuma satuannya derajat.
    const float ptr = putarRuas(idx);
    if (fabsf(ptr) > 0.5f)     { Serial.print(ptr > 0 ? '+' : '-');
                                 Serial.print(fabsf(ptr), 0); }
    else                       { Serial.print("  "); }
    Serial.print(' ');

    switch (kemudiRuas(idx)) {
        case KMD_KIRI:   Serial.print("kiri   "); break;
        case KMD_TENGAH: Serial.print("tengah "); break;
        default:         Serial.print("kanan  "); break;
    }
    switch (x.profil) {
        case PRF_TANGGA:   Serial.print("TANGGA   "); break;
        case PRF_MERUNDUK: Serial.print("MERUNDUK "); break;
        case PRF_SEMPIT:   Serial.print("SEMPIT   "); break;
        case PRF_KAIL:     Serial.print("KAIL     "); break;
        case PRF_TANJAK:   Serial.print("TANJAK   "); break;
        default:           Serial.print("datar    "); break;
    }
    switch (x.henti) {
        case HNT_ODO:       Serial.print("odo  "); break;
        case HNT_DEPAN:     Serial.print("dpn  "); break;
        case HNT_BELAKANG:  Serial.print("blk  "); break;
        case HNT_SISI:      Serial.print("sisi "); break;
        case HNT_MUNDUR:    Serial.print("mdr  "); break;
        case HNT_PUNCAK:    Serial.print("pck  "); break;
        case HNT_GESER:     Serial.print("gsr  "); break;
        default:            Serial.print("--   "); break;
    }
    if (x.henti == HNT_LANGSUNG)   Serial.print("     ");
    else if (_cm[idx] < 0.0f)      Serial.print("  ?  ");
    else { if (_cm[idx] < 100.0f) Serial.print(' '); Serial.print(_cm[idx], 0); Serial.print(" "); }

    Serial.print(x.abaikanDepan ? " buta " : "      ");
    if      (x.aksi == AKS_AMBIL)   Serial.print("ANGKAT  ");
    else if (x.aksi == AKS_TARUH)   Serial.print("TARUH   ");
    else if (x.aksi == AKS_KONFIRM) Serial.print("konfirm ");
    else                            Serial.print("        ");
    Serial.println(x.nama);
}

void Misi::tabel() {
    // Kolom belok/kemudi dibaca hidup, kolom arah datang dari cache. Tanpa
    // baris ini keduanya bisa bercerita berbeda di layar yang sama.
    segarkanArah();
    Serial.println("\n--- TABEL LINTASAN ---");
    // MODE CERMIN DIUMUMKAN DI KEPALA TABEL. Tanpa baris ini, satu-satunya
    // tanda bahwa robot menjalankan cerminnya adalah kolom belok/kemudi yang
    // "terasa terbalik" -- dan itu baru disadari sesudah robot berjalan ke
    // arah yang salah.
    if (arenaCermin()) {
        Serial.println("  ARENA CERMIN AKTIF -- kiri dan kanan DITUKAR.");
        Serial.println("  Yang tercetak di bawah sudah hasil pencerminan, bukan isi tabel.");
        Serial.println("  Matikan dengan 'Qarena.mirror 0' atau tombol D3.");
    }
    Serial.println("  #  arah    belok  kemudi profil   henti  cm  buta  aksi    nama");
    for (uint8_t i = 0; i < RUAS_N; i++) cetakRuas(i);
    Serial.println("  'm7 <idx> <cm>' menyetel panjang ruas.  'm4 <idx>' mulai dari satu ruas.");
    Serial.println("  '?' pada kolom cm = BELUM DIUKUR, misi menolak berangkat.");
}

void Misi::status() {
    Serial.println("\n--- STATUS MISI ---");
    Serial.print("  state       : ");
    switch (_stat) {
        case MISI_DIAM:    Serial.println("DIAM ('m1' mulai, 'm4' lihat tabel)"); break;
        case MISI_PIVOT:   Serial.print("PIVOT -- menghadapkan badan ke ");
                           Serial.print(_nav.namaArah(_arah[_i]));
                           if (ruasSerong(_i)) { Serial.print(" serong ");
                                                 Serial.print(_serong[_i], 0);
                                                 Serial.print(" der"); }
                           Serial.println(); break;
        case MISI_JALAN:   Serial.println("BERJALAN"); break;
        case MISI_SETEL:   Serial.println("MENUNGGU PROFIL GAIT TENANG (badan sedang turun/naik)"); break;
        case MISI_LENGAN:  Serial.println("SEKUENS LENGAN (pose tetap, buta)"); break;
        case MISI_UKUR:    Serial.print("MODE UKUR -- sudah ");
                           Serial.print(_robot.jarakCm() - _ruasAwal, 1);
                           Serial.println(" cm, hentikan di ujung ruas ('s')"); break;
        case MISI_KONFIRM: Serial.println("MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)"); break;
        case MISI_SELESAI: Serial.println("SELESAI -- FINISH"); break;
        case MISI_GAGAL:   Serial.println("GAGAL"); break;
        default:           Serial.println("?"); break;
    }
    if (_stat == MISI_GAGAL && _sebab) { Serial.print("  sebab       : "); Serial.println(_sebab); }

    Serial.print("  ruas        : "); Serial.print(_i); Serial.print(" dari 0..");
    Serial.print(RUAS_N - 1); Serial.print("  --  "); Serial.println(RUAS[_i].nama);

    // Sidik tabel ikut di SETIAP status, bukan cuma saat diminta: HUD menarik
    // status berkala, jadi tabel RAM yang tidak lagi sama dengan draft editor
    // (misalnya sesudah Teensy reset) ketahuan tanpa perintah tambahan.
    tabelVersi();

    if (_stat == MISI_JALAN) {
        const Ruas& x = RUAS[_i];
        Serial.print("  di ruas ini : ");
        if (x.henti == HNT_ODO) {
            Serial.print(_robot.jarakCm() - _ruasAwal, 1); Serial.print(" dari ");
            Serial.print(_cm[_i], 0); Serial.println(" cm (odometri gait)");
        } else if (x.henti == HNT_DEPAN) {
            Serial.print("menunggu depan <= "); Serial.print(_cm[_i], 0);
            Serial.print(" cm, sampel "); Serial.print(_n);
            Serial.print("/"); Serial.println(MISI_SAMPEL_N);
        } else if (x.henti == HNT_BELAKANG) {
            Serial.print("menunggu belakang >= "); Serial.print(_blkAwal + _cm[_i], 1);
            Serial.print(" cm, sampel "); Serial.print(_n);
            Serial.print("/"); Serial.println(MISI_SAMPEL_N);
        } else Serial.println("hanya aksi");
        Serial.print("  waktu ruas  : "); Serial.print(lewat() / 1000);
        Serial.print(" dari "); Serial.print(MISI_RUAS_BATAS_MS / 1000); Serial.println(" detik");
    }

    Serial.print("  membawa     : depan ");
    Serial.print(_korban[ARM_DEPAN] ? "KORBAN" : "kosong");
    Serial.print(", belakang ");
    Serial.println(_korban[ARM_BELAKANG] ? "KORBAN" : "kosong");
    Serial.println("  capit       : BELUM TERPASANG -- ruas korban hanya berhenti kosong");

    int depan = _lidar.getDistance(LIDAR_FRONT);
    Serial.print("  depan       : ");
    if      (depan == LIDAR_MATI) Serial.println("MATI -- sensor tidak merespons");
    else if (depan == LIDAR_JAUH) Serial.println("jauh");
    else { Serial.print(depan); Serial.println(" cm"); }

    int blk = _lidar.getDistance(LIDAR_BACK);
    Serial.print("  belakang    : ");
    if      (blk == LIDAR_MATI) Serial.println("MATI");
    else if (blk == LIDAR_JAUH) Serial.println("jauh -- dinding START di luar jangkauan");
    else { Serial.print(blk); Serial.println(" cm dari dinding START"); }

    Serial.print("  navigasi    : ");
    Serial.println(_nav.navMode() == NAV_DIAM ? "DIAM" : "sedang dipegang misi");
    Serial.println("  ('m4' tabel lintasan, 'v' rincian navigasi, 'l' tabel LiDAR)");
}
