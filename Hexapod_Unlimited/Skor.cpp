#include "Skor.h"
#include "Misi.h"

// ====================================================================
// PETA RUAS -> JENIS POIN
//
// DISUSUN ULANG 18 Sep 2026 untuk tabel lintasan 34 baris. Dicocokkan lewat
// NAMA ruas, bukan nomornya.
//
// INI SUDAH SALAH DUA KALI, dan dua-duanya DIAM: JenisSkor mana pun sah di
// indeks mana pun, jadi peta yang bergeser memberi poin ke ruas yang salah
// tanpa satu pun keluhan dari kompiler. Yang terakhir (peta 30 baris melawan
// tabel 34 baris) membuat ruas 16 "K-3 angkat korban" bernilai NOL, dan ruas
// 30..33 -- termasuk "SZ-5 / FINISH" -- tidak dinilai sama sekali karena
// ruasSelesai() keluar lebih dulu di `idx >= SKOR_RUAS_N`. Tanpa FINISH,
// _finis tidak pernah true dan bonus waktu guidebook tidak pernah dihitung.
//
// ATURANNYA: tabel ini WAJIB sepanjang RUAS_BAKU[] di Misi.cpp, dan komentar
// tiap baris WAJIB nama ruas pada indeks itu. Tampilan::begin() memeriksa
// panjangnya saat menyala dan mengeluh ke Serial kalau tidak cocok -- baca
// keluhan itu, jangan lewati.
//
//
// Disusun dari NAMA ruas di tabel lintasan Misi.cpp, dan itu satu-satunya
// sumber yang firmware punya. Dua tempat yang HARUS dicocokkan dengan peta
// arena sebelum dipercaya:
//
//   * ruas 3 "M1 turunan + R-2/R-3" melewati DUA rintangan dalam satu ruas.
//     Di sini ia dihitung SATU. Kalau juri menilai R-2 dan R-3 terpisah,
//     angka OLED lebih kecil 100..150 daripada yang sebenarnya. Itu arah galat
//     yang benar untuk angka yang dipakai mengambil keputusan di lapangan.
//
// DUA YANG SENGAJA TIDAK ADA, keputusan R2C 15 Sep 2026: rintangan R-8 dan
// pembersihan koral R-7 dilewati -- ongkos waktunya tidak sepadan dengan
// poinnya. Jadi ketiadaannya di tabel ini BUKAN celah yang perlu ditambal.
// Perintah 'X1'/'X2' tetap ada kalau keputusan itu berubah di lapangan.
// ====================================================================
const JenisSkor SKOR_RUAS[] = {
/* 0*/ SK_HOME,        // HOME -> samping K-1
/* 1*/ SK_ANGKAT,      // K-1 angkat korban
/* 2*/ SK_RINTANG,     // R-1 jalan pecah
/* 3*/ SK_RINTANG,     // M1 turunan + R-2/R-3  -- lihat catatan di atas
/* 4*/ SK_NIHIL,       // Depan sampai 20 cm
/* 5*/ SK_TARUH,       // SZ-1 taruh korban (dalam R-4)
/* 6*/ SK_RINTANG_SZ,  // hadap kiri maju -- lihat SK_RINTANG_SZ di Skor.h.
       //          R-4 dinilai DI SINI karena ruas 6 selesai tepat saat robot
       //          sampai di depan K-2; sebelum itu robot masih di dalam R-4,
       //          tempat SZ-1 berada.
/* 7*/ SK_ANGKAT,      // K-2 angkat korban
/* 8*/ SK_RINTANG,     // R-5 lumpur: BARAT sampai ujung
/* 9*/ SK_TARUH,       // SZ-2 taruh korban (kanan 20)
/*10*/ SK_NIHIL,       // SELATAN keluar R-5 (kiri 20)
/*11*/ SK_NIHIL,       // ratakan ke dinding KANAN
/*12*/ SK_NIHIL,       // SELATAN sampai tembok K-3
/*13*/ SK_NIHIL,       // putar kiri lalu maju
/*14*/ SK_NIHIL,       // kiri ke depan K-3
/*15*/ SK_NIHIL,       // maju sedikit (CARI yang halus)
/*16*/ SK_ANGKAT,      // K-3 angkat korban
/*17*/ SK_NIHIL,       // hadap SELATAN (rotasi saja)
/*18*/ SK_RINTANG,     // R-6 pecah: SELATAN sampai 28
/*19*/ SK_TARUH,       // SZ-3 taruh korban
/*20*/ SK_NIHIL,       // hadap BARAT (rotasi saja)
/*21*/ SK_NIHIL,       // K-4: pendekatan milik CARI
/*22*/ SK_ANGKAT,      // K-4 angkat korban
/*23*/ SK_NIHIL,       // jalan ke depan tangga
/*24*/ SK_NIHIL,       // ratakan 13 cm dinding KANAN
/*25*/ SK_RINTANG_R9,  // R-9 TANGGA (miring 103)
/*26*/ SK_NIHIL,       // Maju sedikit naik tangga
/*27*/ SK_RINTANG,     // R-10 puing+lumpur miring
/*28*/ SK_NIHIL,       // jalan ke kiri ke depan SZ-4
/*29*/ SK_TARUH,       // SZ-4 taruh korban (dalam R-10)
/*30*/ SK_NIHIL,       // jalan ke kanan depan K-5
/*31*/ SK_ANGKAT,      // K-5 angkat korban
/*32*/ SK_RINTANG,     // R-11 longsor (lebar jalan 30)
/*33*/ SK_TARUH_SZ5,   // SZ-5 / FINISH
};

// RUAS_N adalah `const uint8_t` yang diisi sizeof di Misi.cpp, jadi ia bukan
// constant expression di unit terjemahan ini -- static_assert tidak bisa
// memakainya. Panjangnya dipakai saat jalan, dan ketidakcocokannya dilaporkan
// sekali di begin() alih-alih diam.
const uint8_t SKOR_RUAS_N = (uint8_t)(sizeof(SKOR_RUAS) / sizeof(SKOR_RUAS[0]));

// --- SALINAN RAM, dan INILAH yang dibaca ruasSelesai() -------------------
//
// Panjangnya RUAS_MAKS, bukan SKOR_RUAS_N: sesudah 'm5+' peta ini ikut tumbuh
// sampai sepanjang tabel lintasan, dan tabel lintasan dibatasi RUAS_MAKS.
JenisSkor SKOR_RAM[RUAS_MAKS];
uint8_t   SKOR_RAM_N = 0;

void skorBaku() {
    SKOR_RAM_N = SKOR_RUAS_N < RUAS_MAKS ? SKOR_RUAS_N : RUAS_MAKS;
    for (uint8_t i = 0; i < SKOR_RAM_N; i++) SKOR_RAM[i] = SKOR_RUAS[i];
}

void skorSisip(uint8_t idx) {
    if (SKOR_RAM_N >= RUAS_MAKS) return;
    if (idx > SKOR_RAM_N) idx = SKOR_RAM_N;
    for (uint8_t i = SKOR_RAM_N; i > idx; i--) SKOR_RAM[i] = SKOR_RAM[i - 1];
    // SK_NIHIL, bukan menebak: ruas yang baru disisipkan belum punya arti di
    // guidebook, dan poin yang ditebak lebih buruk daripada poin yang nol --
    // yang nol kelihatan di papan skor, yang ditebak tidak.
    SKOR_RAM[idx] = SK_NIHIL;
    SKOR_RAM_N++;
}

void skorHapus(uint8_t idx) {
    if (idx >= SKOR_RAM_N || SKOR_RAM_N == 0) return;
    for (uint8_t i = idx; i + 1 < SKOR_RAM_N; i++) SKOR_RAM[i] = SKOR_RAM[i + 1];
    SKOR_RAM_N--;
}

bool Skor::tandai(uint8_t idx) {
    if (idx >= 64) return false;
    const uint8_t w = idx >> 5, b = idx & 31;
    if (_sudah[w] & (1UL << b)) return false;
    _sudah[w] |= (1UL << b);
    return true;
}

void Skor::reset() {
    _total = 0;
    _nAngkat = _nTaruh = _bersih = 0;
    _bawa = _finis = false;
    _sudah[0] = _sudah[1] = 0;
}

void Skor::ruasSelesai(uint8_t idx) {
    // Dijaga terhadap KEDUA panjang: tabel poin yang lebih pendek daripada
    // tabel lintasan akan membaca di luar batas, dan itu jenis kesalahan yang
    // muncul sebagai poin acak, bukan sebagai crash.
    if (idx >= RUAS_N || idx >= SKOR_RAM_N) return;
    const JenisSkor j = SKOR_RAM[idx];

    // MUATAN DIPERBARUI WALAU RUASNYA DIULANG. Bit "sudah dinilai" menjaga
    // POIN dari dihitung dua kali; ia tidak boleh ikut membekukan keadaan
    // capit, karena ruas AMBIL yang diulang memang benar-benar mengangkat
    // korban lagi dan rintangan sesudahnya dinilai menurut muatan itu.
    if (j == SK_ANGKAT)                          _bawa = true;
    else if (j == SK_TARUH || j == SK_TARUH_SZ5) _bawa = false;

    if (!tandai(idx)) return;                    // sudah pernah dinilai

    switch (j) {
        case SK_HOME:   _total += POIN_HOME; break;
        case SK_ANGKAT: _total += POIN_ANGKAT; _nAngkat++; break;
        case SK_RINTANG:
            // Dinilai menurut muatan SAAT menyeberang, dan _bawa sudah
            // diperbarui di atas oleh ruas AMBIL/TARUH sebelumnya.
            _total += _bawa ? POIN_RINTANG_BAWA : POIN_RINTANG_KOSONG;
            break;
        case SK_RINTANG_SZ:
            // SELALU nilai "membawa", tanpa melihat _bawa. Korbannya memang
            // sudah dilepas -- di safe zone yang ada DI DALAM rintangan ini,
            // dan justru itu yang membuat guidebook tetap menilainya membawa.
            _total += POIN_RINTANG_BAWA;
            break;
        case SK_RINTANG_R9:
            _total += _bawa ? POIN_R9_BAWA : POIN_R9_KOSONG;
            break;
        case SK_TARUH:     _total += POIN_TARUH;     _nTaruh++; break;
        case SK_TARUH_SZ5: _total += POIN_TARUH_SZ5; _nTaruh++; _finis = true; break;
        default: break;
    }
}

void Skor::setBersihR7(uint8_t tingkat) {
    if (tingkat > 2) tingkat = 2;
    // Diambil yang TERBAIK, sama seperti aturan rintangan: laporan "sebagian"
    // sesudah "seluruh" tidak boleh menurunkan angkanya.
    if (tingkat <= _bersih) return;
    const uint32_t lama = (_bersih == 2) ? POIN_BERSIH_PENUH
                        : (_bersih == 1) ? POIN_BERSIH_SEBAGIAN : 0;
    const uint32_t baru = (tingkat == 2) ? POIN_BERSIH_PENUH
                        : (tingkat == 1) ? POIN_BERSIH_SEBAGIAN : 0;
    _total += (baru - lama);
    _bersih = tingkat;
}

uint32_t Skor::bonus(uint32_t detik) const {
    if (!bonusSah() || detik == 0) return 0;
    return ((uint32_t)_total * BONUS_FAKTOR) / detik;
}

void Skor::cetak(uint32_t detik) const {
    Serial.println("\n--- PERKIRAAN POIN (bukan penilaian juri) ---");
    Serial.print("  korban terangkat : "); Serial.print(_nAngkat);
    Serial.print(" x "); Serial.print(POIN_ANGKAT); Serial.println(" poin");
    Serial.print("  korban tertaruh  : "); Serial.println(_nTaruh);
    Serial.print("  sedang membawa   : "); Serial.println(_bawa ? "YA" : "tidak");
    Serial.print("  bersih R-7       : ");
    Serial.println(_bersih == 2 ? "seluruh area (200)"
                 : _bersih == 1 ? "sebagian (100)" : "belum dilaporkan");
    Serial.print("  TOTAL            : "); Serial.println(_total);
    if (bonusSah()) {
        Serial.print("  bonus waktu      : "); Serial.print(_total);
        Serial.print(" x "); Serial.print(BONUS_FAKTOR);
        Serial.print(" / "); Serial.print(detik);
        Serial.print(" detik = "); Serial.println(bonus(detik));
        Serial.print("  TOTAL + BONUS    : "); Serial.println(totalDenganBonus(detik));
    } else {
        Serial.println("  bonus waktu      : BELUM SAH -- butuh 5 korban tertaruh");
        Serial.println("                     DAN finish di SZ-5.");
    }
    Serial.println("  Angka ini dihitung dari RUAS YANG SELESAI, bukan dari");
    Serial.println("  penglihatan. Firmware tidak tahu korban benar-benar terjepit");
    Serial.println("  atau seluruh badannya masuk safe zone -- juri yang menilai.");
}
