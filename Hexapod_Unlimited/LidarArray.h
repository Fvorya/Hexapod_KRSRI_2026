#ifndef LIDARARRAY_H
#define LIDARARRAY_H

// 6x VL53L1X via mux TCA9548A, NON-BLOCKING (state machine round-robin).
// update() memajukan SATU sensor per panggilan tanpa busy-wait -> loop utama tetap kencang.
// Filter: Register Interrupt -> median-3 (buang outlier) -> EMA.
//
// PENTING -- tiga keadaan, bukan dua:
//   jarak cm    : ada objek dalam jangkauan
//   LIDAR_JAUH  : sensor SEHAT tapi tak ada objek di dalam LIDAR_MAX_CM
//   LIDAR_MATI  : sensor tidak merespons / hanya menghasilkan pengukuran buruk
// Versi lama menyamakan "jauh" dengan "mati" (keduanya -1). Untuk wall-follow
// itu berbahaya: "lorong terbuka" dan "sensor putus" butuh reaksi berlawanan.
//
// VL53L1X memisahkan ketiganya lewat range_status, BUKAN lewat ambang jarak.
// Berbeda dari VL53L0X yang mengembalikan 8190 mm untuk "tak ada target",
// range_mm di sini SELALU berisi angka hasil hitungan -- sah atau tidak. Jadi
// menyaring dengan ambang jarak akan menerima angka sampah sebagai jarak sah.
//
// SATU PENGECUALIAN, dari uji fisik robot ini: sensornya TIDAK selalu memakai
// SignalFail untuk "tak ada target". Sering kali ia melaporkan ~5 cm dengan
// status RangeValid. Karena itu ada LIDAR_MIN_CM (config.h): bacaan di bawah
// batas geometri tiap arah dipetakan ke LIDAR_JAUH, bukan diterima sebagai
// jarak. Ini bukan "menyaring pakai ambang jarak" seperti kode VL53L0X lama --
// ambangnya diturunkan dari jangkauan KAKI robot, jadi ia menyatakan sesuatu
// yang mustahil secara fisik, bukan sekadar sesuatu yang tampak aneh.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h> // Pustaka "VL53L1X by Pololu" -- BUKAN "VL53L0X by Pololu";
                     // keduanya pustaka terpisah, bukan versi baru dari yang sama.
#include "config.h"

class LidarArray {
public:
    LidarArray();
    bool begin();                  // true bila mux + KEENAM sensor sukses init
    void update();                 // non-blocking (dipanggil di loop utama)

    // cm, atau LIDAR_JAUH / LIDAR_MATI (lihat catatan di atas)
    int  getDistance(uint8_t id);
    bool sensorHidup(uint8_t id);  // masih merespons dalam LIDAR_TIMEOUT_MS?

    // Jarak EMA TANPA pembulatan ke cm, plus stempel waktu sampel terakhir.
    // Keduanya untuk suku turunan PD. getDistance() membulatkan ke integer,
    // jadi selisih antar sampel selalu kelipatan 1 cm dan turunannya berbentuk
    // tangga -- nol terus, lalu melonjak. Stempelnya membuat turunan bisa
    // dihitung pada laju SAMPEL (~30 ms), bukan laju loop (bisa < 1 ms).
    // jarakHalus() mengembalikan < 0 bila keadaannya bukan "ada objek dalam
    // jangkauan"; pakai getDistance() untuk membedakan JAUH dari MATI.
    float    jarakHalus(uint8_t id);
    uint32_t stempelSampel(uint8_t id) const { return (id < NUM_LIDAR) ? _lastOk[id] : 0; }

    bool    muxTerdeteksi() const { return _muxOk; }
    uint8_t jumlahHidup();         // berapa sensor yang masih merespons

    void cetakTabel();             // laporan enam baris (perintah 'l')
    void cetakBaris();             // satu baris ringkas (aliran 'L')

    // Perintah 'I'. Memindai mux + tiap channel, DAN meng-init ulang sensor
    // yang ada di bus tapi belum aktif. begin() hanya jalan sekali saat boot,
    // jadi tanpa ini sensor yang gagal init (mis. modul belum siap saat papan
    // menyala, atau kabel yang baru dibetulkan) tetap mati sampai di-reset --
    // padahal pindaiannya sendiri melaporkan modulnya "ADA".
    void pindaiI2C();

    // JEJAK: kumpulkan statistik mentah satu/semua channel selama beberapa
    // detik, lalu ringkas. Satu cuplikan 'l' tidak bisa membedakan hantu yang
    // MENETAP (crosstalk / benda terpasang di depan sensor) dari hantu yang
    // BERUBAH-UBAH (aliasing, pantulan sesaat) -- sebaran angkanya yang bisa.
    // Non-blokir: pengumpulan berjalan di update(), hasilnya dicetak sendiri.
    void jejakMulai(int8_t ch, uint16_t detik);
    bool jejakJalan() const { return _jejakSampai != 0; }

    // ISOLASI: ukur SATU sensor dua kali -- sekali saat kelima sensor lain
    // ikut memancar, sekali saat kelimanya DIMATIKAN -- lalu bandingkan.
    // Ini satu-satunya cara memisahkan crosstalk ANTAR-SENSOR (enam VL53L1X
    // memancar bersamaan terus-menerus di mode kontinu) dari pantulan kaca
    // penutup / bagian robot, karena keduanya sama-sama menghasilkan angka
    // pendek yang menetap. Tidak bisa dilakukan dengan tangan.
    // ch < 0 = sapu SEMUA sensor berurutan lalu cetak tabel ringkas.
    void isolasiMulai(int8_t ch, uint16_t detik);
    bool isolasiJalan() const { return _isoFase != 0; }

    static const char* nama(uint8_t id);

private:
    VL53L1X _sensor[NUM_LIDAR];       // Wajib: 1 Objek per channel mux
    bool _isReady[NUM_LIDAR];         // Penanda jika sensor sukses di-init

    float _dist[NUM_LIDAR];           // hasil EMA (cm)
    int  _hist[NUM_LIDAR][3];         // 3 sampel terakhir (untuk filter median)
    uint8_t _histN[NUM_LIDAR];        // jumlah sampel terkumpul (<=3)
    uint8_t _pendekN[NUM_LIDAR];      // sampel berturut-turut di bawah LIDAR_MIN_CM
    uint32_t _lastOk[NUM_LIDAR];      // waktu data DALAM JANGKAUAN terakhir
    uint32_t _lastResp[NUM_LIDAR];    // waktu sensor terakhir MENJAWAB (walau jauh)
    bool _jauh[NUM_LIDAR];            // jawaban terakhir: sah tapi di luar LIDAR_MAX_CM

    // Jawaban MENTAH terakhir, sebelum disaring. Dicetak 'l' supaya bacaan
    // pendek yang tak masuk akal bisa dilacak ke sumbernya: status RangeValid
    // berarti sensornya memang melihat sesuatu (crosstalk / badan robot),
    // sedangkan WrapTargetFail berarti pantulan dari objek yang terlalu jauh.
    uint16_t _mmAkhir[NUM_LIDAR];
    uint8_t  _statusAkhir[NUM_LIDAR];

    // --- akumulator jejak ---
    uint32_t _jejakSampai = 0;        // millis() kapan jendela berakhir (0 = mati)
    int8_t   _jejakCh = -1;           // -1 = semua channel
    uint16_t _jHist[NUM_LIDAR][12];   // cacah per range_status
    uint16_t _jMin[NUM_LIDAR], _jMax[NUM_LIDAR];
    uint32_t _jJumlah[NUM_LIDAR];     // jumlah mm yang sah
    uint16_t _jN[NUM_LIDAR];          // cacah mm yang sah
    void jejakCatat(uint8_t ch, uint16_t mm, uint8_t status);
    void jejakCetak();

    // --- uji isolasi ---
    uint8_t  _isoFase = 0;            // 0 mati, 1 semua nyala, 2 lainnya mati
    uint8_t  _isoCh = 0;
    uint16_t _isoDetik = 0;
    uint32_t _isoSampai = 0;
    uint16_t _isoMin[2], _isoMaks[2], _isoN[2], _isoTotal[2];
    uint32_t _isoJumlah[2];
    bool     _isoSemua = false;       // menyapu seluruh channel?
    int16_t  _isoRata[NUM_LIDAR][2];  // rata mm per fase; -1 = tak ada bacaan sah
    void isolasiCatat(uint8_t ch, uint16_t mm, uint8_t status);
    void isolasiCetak();
    void isolasiTabel();
    bool isolasiLanjut();
    void isolasiNolkan(uint8_t fase);
    void isolasiLainnya(bool nyalakan);

    uint8_t _cur;                     // sensor yang sedang diproses di state machine
    bool _muxOk;                      // mux menjawab saat begin()?

    bool selectMux(uint8_t ch);       // false bila mux tak mengakui alamat

    // Init + konfigurasi + mulai mode kontinu untuk SATU channel, plus
    // membersihkan sisa filter/stempel waktunya. Dipakai bersama begin() dan
    // pindaiI2C() supaya sensor yang di-init ulang mendapat konfigurasi yang
    // persis sama dengan saat boot, bukan salinan yang bisa menyimpang.
    bool initSensor(uint8_t ch);
};

#endif
