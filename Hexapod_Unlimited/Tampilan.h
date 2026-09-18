#ifndef TAMPILAN_H
#define TAMPILAN_H

#include <Arduino.h>
#include "Skor.h"

class Misi;

// ====================================================================
// OLED + EMPAT TOMBOL (D6..D3)
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
//
// ALUR TOMBOL, sejak v2.0:
//
//   panel MATI  --tekan apa pun-->  MENU  --tekan/tahan-->  TANYA
//                                     ^                       |
//                                     +--TEKAN tombol sama----+
//                                                             |
//   panel MATI  <--tombol lain, atau 5 detik tanpa jawaban-----+
//
// Jawabannya selalu TEKAN, walau yang ditanyakan aksi tahan. Menahan di layar
// TANYA tidak menjawab dan tidak membatalkan; ia habis waktu seperti biasa.
//
// DUA REM TIDAK IKUT ALUR INI: TOMBOL_STOP tekan dan TOMBOL_RESET tahan jalan
// seketika dari layar mana pun, juga saat panel mati.
//
// Dua hal yang berubah dan keduanya saling menopang. Panel MATI adalah
// keadaan default, jadi layar tidak lagi menghabiskan giliran I2C dan waktu
// loop untuk tulisan yang tidak dibaca siapa pun. Dan karena panel mati,
// operator tidak bisa lagi memastikan keadaan robot dari layar sebelum
// menekan -- maka tidak satu pun tombol boleh langsung menjalankan
// perintahnya. Tombol menanyakan dulu, layar menjawab apa yang akan terjadi,
// dan diam selama 5 detik berarti TIDAK JADI.
//
// Satu akibat yang harus disadari: menu ikut mati sesudah 5 detik, juga saat
// misi sedang berjalan. Selama robot jalan, HUD di Pi yang jadi layar status,
// bukan OLED.
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

// URUT TERBALIK terhadap v1.18: D6, D5, D4, D3 -- bukan D2..D5. Tata letak PCB
// menaruh tombol pertama di D6 dan turun ke D3.
//
// Yang berubah hanya peta pin dan nama yang dicetak di layar. Seluruh logika
// memakai INDEKS 0..3, tidak pernah nomor pin, jadi fungsi tiap tombol tetap
// menempel pada tombol fisik yang sama dan tidak ada tabel lain yang bergeser.
// Pin 3 dan 6 tidak dipakai modul mana pun: LiDAR di Wire (18/19), OLED dan
// driver servo di Wire2 (24/25).
#define PIN_TOMBOL_A    6     // D6
#define PIN_TOMBOL_B    5     // D5
#define PIN_TOMBOL_C    4     // D4
#define PIN_TOMBOL_D    3     // D3

// Indeks tombol kompas. Dipakai di LAYAR_IMU, yang memperlakukan tombol ini
// berbeda dari tiga lainnya.
#define TOMBOL_KOMPAS   2

// REM. Keduanya mengirim 'm0' dan keduanya MELEWATI konfirmasi, di layar mana
// pun, termasuk saat panel mati. Rem yang minta konfirmasi bukan rem: dua
// gerakan untuk menghentikan robot yang sedang salah jalan adalah satu gerakan
// terlalu banyak, dan menekan rem tidak pernah punya akibat yang perlu
// ditanyakan dulu. Alasan yang sama sudah berlaku untuk pembatalan hitung
// mundur sejak awal.
//
// TOMBOL_STOP tekan  = berhenti di ruas kini, ruasnya disimpan untuk 'lanjut'.
// TOMBOL_RESET tahan = berhenti DAN penunjuk ruas serta poin kembali ke 0.
//
// Yang TIDAK ikut dikecualikan: TOMBOL_STOP tahan ('lanjut dari ruas
// tersimpan') dan TOMBOL_RESET tekan (mirror). Yang pertama membuat robot
// berjalan lagi -- itu gas, bukan rem.
#define TOMBOL_STOP     1
// RESET PINDAH KE D5 TAHAN, 18 Sep 2026, diminta R2C. 'lanjut dari ruas
// tersimpan' yang tadinya di sana dibuang -- fitur itu tidak dipakai.
//
// Menaruhnya di tombol yang sama dengan STOP membuat D5 jadi tombol rem
// seluruhnya: tekan berhenti, tahan berhenti dan menolkan. Sebelumnya D5 tahan
// justru MENJALANKAN robot lagi, dan satu tombol yang remnya di satu gerakan
// dan gasnya di gerakan lain adalah tombol yang salah ditekan saat panik.
#define TOMBOL_RESET    1

// MISI. Sejak 18 Sep 2026 ia diperlakukan SAMA seperti TOMBOL_STOP: sekali
// tekan, seketika, dari layar mana pun termasuk panel mati, tanpa konfirmasi
// dan tanpa hitung mundur. Diminta R2C.
//
// Alasannya waktu lomba. Panel mati sesudah 5 detik, jadi start misi lewat
// menu menuntut bangunkan layar, pilih, konfirmasi, lalu tunggu hitung mundur
// -- empat langkah di detik yang paling sibuk. Yang dibayar: satu sentuhan
// salah memberangkatkan robot. Itu diterima karena robot di garis start
// memang sedang menunggu diberangkatkan, dan REM-nya satu sentuhan juga.
#define TOMBOL_MISI     0

// KONFIRMASI HANYA UNTUK INI. Tombol lain jalan seketika. Kompas dikecualikan
// karena ia satu-satunya yang MENULIS kalibrasi: mencatat arah yang salah
// merusak seluruh mode arena, dan salahnya baru terlihat beberapa ruas
// kemudian saat robot berbelok ke arah yang tidak masuk akal.
#define TOMBOL_PERLU_TANYA(i)  ((i) == TOMBOL_KOMPAS)

// Debounce dan ambang tahan. 2 detik sesuai permintaan R2C; cukup lama
// sehingga tidak ada yang tak sengaja memicu aksi tahan, cukup pendek
// sehingga tidak terasa seperti robot tidak merespons.
#define TOMBOL_DEBOUNCE_MS   25
#define TOMBOL_TAHAN_MS    2000

// Berapa lama layar yang menunggu jawaban manusia boleh menyala tanpa disentuh.
// Habis waktu = TIDAK JADI, dan layar kembali mati. Layar mati adalah keadaan
// default, jadi tidak menjawab selalu berarti tidak terjadi apa-apa.
//
// LAYAR_IMU dan LAYAR_PIVOT TIDAK memakai batas ini: keduanya dimasuki sesudah
// konfirmasi, dan operator di sana sedang memutar robot dengan dua tangan --
// layar yang mati sendiri di tengah pencatatan empat arah kompas membuang
// pekerjaan yang sudah dikerjakan.
#define LAYAR_TIMEOUT_MS   5000

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
    LAYAR_DIAM = 0,   // panel MATI -- tekan apa saja untuk menyalakan
    LAYAR_MENU,       // ruas, state, skor
    LAYAR_TANYA,      // "mau jalankan ini?" -- tekan lagi = ya
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
    uint16_t _gagalN = 0;        // percobaan OLED yang gagal berturut-turut
    void bacaTombol();
    void tekan(uint8_t i);
    void tahan(uint8_t i);

    // minta() memasang LAYAR_TANYA; aksi*() menjalankan perintahnya. Dipisah
    // supaya hanya ada SATU tempat yang bisa menjalankan fungsi tombol, dan
    // tempat itu cuma bisa dicapai lewat jawaban YA.
    void minta(uint8_t i, bool tahanan);
    void aksiTekan(uint8_t i);
    void aksiTahan(uint8_t i);

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
    uint32_t _tSentuh = 0;            // kapan tombol terakhir ditekan
    bool     _nyala   = false;        // panel sedang menyala? (DISPLAYON)
    uint8_t  _tanyaIdx   = 0;         // tombol yang sedang ditanyakan
    bool     _tanyaTahan = false;     // yang ditanyakan: tekan atau tahan
    uint32_t _tPesan  = 0;            // sampai kapan pesan sementara tampil
    char     _pesan[22] = { 0 };
    uint8_t  _arahBerikut = 0;        // 0..3 -- utara, timur, selatan, barat
    // Ruas saat 'stop'. SEKARANG HANYA UNTUK PESAN DI LAYAR -- 'lanjut dari
    // ruas tersimpan' dibuang 18 Sep 2026, jadi tidak ada lagi yang membacanya
    // untuk melanjutkan. Dibiarkan karena "STOP di ruas 12" jauh lebih berguna
    // daripada "STOP" saat mencoba ruas yang sama berulang kali.
    uint8_t  _ruasSimpan  = 0;
};

#endif
