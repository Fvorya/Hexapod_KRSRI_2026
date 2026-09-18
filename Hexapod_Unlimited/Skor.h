#ifndef SKOR_H
#define SKOR_H

#include <Arduino.h>

// ====================================================================
// PEMBUKUAN POIN, menurut tabel Penilaian guidebook (moses/ARENA_GUIDEBOOK.md
// bagian 6). Dihitung di Teensy supaya angkanya ada di OLED tanpa bergantung
// pada Raspi yang boot-nya lebih lambat dan bisa mati sendiri.
//
// YANG DIHITUNG HANYA YANG BISA DIKETAHUI FIRMWARE: ruas mana yang SELESAI.
// Firmware tidak punya mata; ia tidak tahu apakah korban benar-benar terjepit,
// apakah seluruh badan korban masuk safe zone, atau seberapa bersih R-7. Jadi
// angka di sini adalah "poin yang SEHARUSNYA didapat kalau tiap ruas berhasil
// seperti rancangannya", bukan poin yang dinilai juri.
//
// Perbedaan itu penting dan sengaja tidak disembunyikan: OLED menandainya
// dengan '~' supaya tidak ada yang membacanya sebagai skor resmi.
// ====================================================================

enum JenisSkor : uint8_t {
    SK_NIHIL = 0,     // ruas perpindahan biasa, tidak berpoin
    SK_HOME,          // keluar Home                                   50
    SK_ANGKAT,        // korban terangkat keluar area korban           50
    SK_RINTANG,       // R1..R11 selain R7 & R9   100 kosong / 150 bawa
    SK_RINTANG_SZ,    // rintangan yang safe zone-nya ADA DI DALAMNYA: selalu
                      //   dinilai "membawa korban". Guidebook: "Untuk R4, R5,
                      //   R8, R10: walau robot belum sepenuhnya keluar
                      //   rintangan, kalau berhasil menaruh korban di Safety
                      //   Zone-nya, robot tetap mendapat nilai membawa korban
                      //   -- asal akhirnya berhasil keluar rintangan."
                      //   Dipakai R-4, yang ruas penyeberangannya jatuh
                      //   SESUDAH peletakan di SZ-1 sehingga muatan capit
                      //   sudah kosong saat dihitung.
    SK_RINTANG_R9,    // R9 (tangga)              150 kosong / 300 bawa
    SK_TARUH,         // korban ditaruh di SZ-1..SZ-4                  50
    SK_TARUH_SZ5      // korban ditaruh di SZ-5                       100
};

// Nilai mentah tabel guidebook. Dipisah dari kodenya supaya menyesuaikan
// guidebook edisi berikutnya tidak menuntut membaca logika apa pun.
#define POIN_HOME            50
#define POIN_ANGKAT          50
#define POIN_RINTANG_KOSONG 100
#define POIN_RINTANG_BAWA   150
#define POIN_R9_KOSONG      150
#define POIN_R9_BAWA        300
#define POIN_TARUH           50
#define POIN_TARUH_SZ5      100
#define POIN_BERSIH_SEBAGIAN 100   // R-7, dilaporkan operator -- bukan sensor
#define POIN_BERSIH_PENUH    200

// Bonus guidebook: total x 300 / waktu_detik, HANYA bila 5 misi + finish.
#define BONUS_FAKTOR        300

// Jenis poin tiap ruas. Definisinya di Skor.cpp, panjangnya WAJIB sama dengan
// RUAS_N. Tidak bisa dijaga static_assert -- RUAS_N diisi sizeof di unit
// terjemahan lain -- jadi Tampilan::begin() yang memeriksanya dan mengeluh
// ke Serial kalau tidak cocok.
extern const JenisSkor SKOR_RUAS[];
extern const uint8_t    SKOR_RUAS_N;

// PETA POIN YANG BENAR-BENAR DIPAKAI, di RAM.
//
// Kembaran RUAS[] milik Misi: sama seperti tabel lintasan, peta poin harus
// bisa bergeser saat operator menyisipkan atau menghapus ruas dari HUD.
// Tanpa ini 'm5+' menggeser lintasan tanpa menggeser poinnya, dan tiap ruas
// sesudah titik sisip dinilai sebagai ruas yang salah -- diam-diam, karena
// JenisSkor mana pun sah di indeks mana pun.
//
// Diisi skorBaku() saat menyala, dari SKOR_RUAS[] di flash.
extern JenisSkor SKOR_RAM[];
extern uint8_t   SKOR_RAM_N;

void skorBaku();                    // RAM <- flash
void skorSisip(uint8_t idx);        // sisip SK_NIHIL di idx
void skorHapus(uint8_t idx);

class Skor {
public:
    // Dipanggil sekali tiap ruas SELESAI. Idempoten terhadap ruas yang sama:
    // ruas yang diulang ('m3') tidak menambah poin dua kali, sesuai aturan
    // "tiap rintangan hanya dinilai 1x, diambil nilai terbaik".
    void ruasSelesai(uint8_t idx);

    void reset();

    // Poin yang sudah terkumpul, tanpa bonus.
    uint32_t total() const { return _total; }
    uint8_t  korbanTerangkat() const { return _nAngkat; }
    uint8_t  korbanTertaruh()  const { return _nTaruh; }
    bool     membawa() const { return _bawa; }

    // Kebersihan R-7 dilaporkan OPERATOR, bukan sensor: tidak ada cara
    // firmware tahu seberapa bersih koralnya. 0 = belum, 1 = sebagian,
    // 2 = seluruh area.
    void setBersihR7(uint8_t tingkat);
    uint8_t bersihR7() const { return _bersih; }

    // Bonus hanya sah bila kelima korban tertaruh DAN finish tercapai.
    bool bonusSah() const { return _nTaruh >= 5 && _finis; }
    uint32_t bonus(uint32_t detik) const;
    uint32_t totalDenganBonus(uint32_t detik) const {
        return _total + bonus(detik);
    }

    void cetak(uint32_t detik) const;   // rincian ke Serial

private:
    uint32_t _total   = 0;
    uint8_t  _nAngkat = 0;
    uint8_t  _nTaruh  = 0;
    uint8_t  _bersih  = 0;
    bool     _bawa    = false;
    bool     _finis   = false;
    // Satu bit per ruas, sampai 64 ruas. Yang menjaga "dinilai 1x".
    uint32_t _sudah[2] = { 0, 0 };

    bool tandai(uint8_t idx);           // true bila BARU ditandai
};

#endif
