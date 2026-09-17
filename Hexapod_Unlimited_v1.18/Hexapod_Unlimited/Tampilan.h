#ifndef TAMPILAN_H
#define TAMPILAN_H

#include <Arduino.h>
#include "Skor.h"

class Misi;

// ====================================================================
// OLED + EMPAT TOMBOL (D2..D5)
//
// Gunanya satu: menjalankan robot di arena TANPA laptop. Aturan lomba
// (guidebook bagian 7) menuntut robot dijalankan dengan satu tombol ditekan
// satu kali, dan sesudah itu tidak boleh ada interaksi peserta sama sekali.
// Menyeret laptop ke garis start untuk mengetik 'm1' bukan cuma repot -- ia
// juga membuat "satu tombol" jadi tidak jujur.
//
// TOMBOL MENGIRIM PERINTAH SERIAL YANG SAMA dengan yang diketik operator.
// Tidak ada jalur kedua ke dalam Misi atau Navigation. Itu yang membuat
// tombol tidak bisa menyimpang dari konsol: keduanya masuk lewat handleCmd().
//
// Kabel: OLED menumpang Wire (SDA 18 / SCL 19) bersama mux LiDAR, alamat
// 0x3C. Tombol ke GND, dengan INPUT_PULLUP -- ditekan = LOW.
// ====================================================================

// 128x32, BUKAN 128x64. Panel yang terpasang setengah tinggi, dan Adafruit
// mengambil geometrinya dari konstruktor -- salah tinggi berarti separuh
// perintah gambar jatuh di luar buffer dan hilang tanpa pesan apa pun.
//
// Jatah tulisan pada tinggi 32, font 6x8: ukuran 1 = 21 kolom x 4 baris
// (y 0, 8, 16, 24). Ukuran 2 = 10 kolom x 2 baris. Ukuran 3 = 7 kolom x 1
// baris. Tiap layar di gambar() dipatok ke petak itu; kursor y di atas 24
// tidak akan terlihat.
// LAYAR ADA DI Wire2, BUKAN Wire. Teensy 4.1: SCL2 pin 24, SDA2 pin 25.
// Bus LiDAR (mux + enam sensor) tetap di Wire, SCL 19 / SDA 18.
//
// Ini bukan rincian sepele. Selama konstruktornya menunjuk &Wire, layar
// dicari di bus yang memang tidak memuatnya, jadi 'I' tidak pernah melihat
// 0x3C -- dan tiap gagal memicu coba-lagi 3 detik yang menjalankan
// Wire.begin() di bus LiDAR. Satu pointer salah, dua gejala: layar mati DAN
// mux TCA9548 hilang.
#define OLED_I2C_BUS  Wire2
#define OLED_I2C_CLOCK 400000

#define OLED_LEBAR    128
#define OLED_TINGGI    32
#define OLED_ALAMAT  0x3C

#define TOMBOL_N        4
#define PIN_TOMBOL_A    2     // D2
#define PIN_TOMBOL_B    3     // D3
#define PIN_TOMBOL_C    4     // D4
#define PIN_TOMBOL_D    5     // D5

// Debounce dan ambang tahan. 2 detik sesuai permintaan R2C; cukup lama
// sehingga tidak ada yang tak sengaja memicu aksi tahan, cukup pendek
// sehingga tidak terasa seperti robot tidak merespons.
#define TOMBOL_DEBOUNCE_MS   25
#define TOMBOL_TAHAN_MS    2000

// Hitung mundur sebelum berjalan, milidetik.
//
// 1 detik, BUKAN 3. Aturan guidebook: robot dijalankan dengan satu tekan, dan
// kalau 3 detik belum bergerak boleh ditekan sekali lagi -- tekan ketiga
// dianggap gagal dijalankan. Hitung mundur 3 detik membakar persis jendela
// itu, jadi tekan kedua yang sah justru lahir dari rancangan kita sendiri.
// Satu detik cukup untuk menjauhkan tangan dan robot sudah bergerak jauh
// sebelum jendela aturan habis.
#define JALAN_MUNDUR_MS    1000

enum LayarId : uint8_t {
    LAYAR_DIAM = 0,   // "R2C UKSW" -- tekan apa saja untuk masuk menu
    LAYAR_MENU,       // ruas, state, skor
    LAYAR_MUNDUR,     // hitung mundur sebelum m1/m4
    LAYAR_IMU,        // catat 4 arah kompas
    LAYAR_PIVOT,      // kalibrasi pivot berjalan
    LAYAR_SKOR        // rincian poin
};

class Navigation;

class Tampilan {
public:
    // `kirim` = handleCmd() milik .ino. Disuntikkan, bukan dipanggil langsung,
    // supaya modul ini tidak ikut bergantung pada seluruh isi .ino.
    void begin(Misi* misi, Skor* skor, void (*kirim)(char*), Navigation* nav = nullptr);
    void update();                    // tiap loop, non-blokir

    bool ada() const { return _ada; }
    void pesan(const char* teks);     // tampilkan sebentar, lalu kembali

private:
    struct Tombol {
        uint8_t  pin;
        bool     turun;               // sedang ditekan, SESUDAH debounce
        bool     tahanTerpakai;       // aksi tahan sudah dijalankan?
        uint32_t tSejak;              // kapan tekanan yang sah itu mulai
        bool     mentahAkhir;         // level pin terakhir, BELUM disaring
        uint32_t tUbah;               // kapan level mentah itu terakhir berubah
    };

    // Satu baris ke serial tiap tombol dipicu. Bukan sekadar jejak: HUD di
    // Pi adalah satu-satunya layar operator selama OLED belum menyala, dan
    // tombol yang bekerja tanpa mengumumkan diri tidak bisa dibedakan dari
    // tombol yang tidak terbaca sama sekali.
    void lapor(uint8_t i, bool tahanan, const char* fungsi);
    void bacaTombol();
    void tekan(uint8_t i);
    void tahan(uint8_t i);
    void gambar();
    void kirimCmd(const char* teks);

    Misi* _misi = nullptr;
    Navigation* _nav = nullptr;
    Skor* _skor = nullptr;
    void (*_kirim)(char*) = nullptr;

    Tombol   _t[TOMBOL_N];
    LayarId  _layar = LAYAR_DIAM;
    bool     _ada   = false;          // OLED terdeteksi?
    uint32_t _tGambar = 0;
    uint32_t _tCoba   = 0;            // kapan OLED terakhir dicoba lagi
    uint32_t _tMundur = 0;            // kapan hitung mundur mulai
    uint32_t _tPesan  = 0;            // sampai kapan pesan sementara tampil
    char     _pesan[22] = { 0 };
    uint8_t  _arahBerikut = 0;        // 0..3 -- utara, timur, selatan, barat
    uint8_t  _ruasSimpan  = 0;        // ruas saat 'stop', untuk 'continue'
};

#endif
