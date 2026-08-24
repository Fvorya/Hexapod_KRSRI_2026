#ifndef LIDARARRAY_H
#define LIDARARRAY_H

// 6x VL53L0X (TOF200C) via mux TCA9548A, NON-BLOCKING (state machine round-robin).
// update() memajukan SATU sensor per panggilan tanpa busy-wait -> loop utama tetap kencang.
// Filter: Register Interrupt -> median-3 (buang outlier) -> EMA.
//
// PENTING -- tiga keadaan, bukan dua:
//   jarak cm    : ada objek dalam jangkauan
//   LIDAR_JAUH  : sensor SEHAT tapi tak ada objek di dalam LIDAR_MAX_CM
//   LIDAR_MATI  : sensor tidak merespons / belum pernah ada data
// Versi lama menyamakan "jauh" dengan "mati" (keduanya -1). Untuk wall-follow
// itu berbahaya: "lorong terbuka" dan "sensor putus" butuh reaksi berlawanan.

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h> // Pastikan Anda sudah menginstal library "VL53L0X by Pololu"
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

    static const char* nama(uint8_t id);

private:
    VL53L0X _sensor[NUM_LIDAR];       // Wajib: 1 Objek per channel mux
    bool _isReady[NUM_LIDAR];         // Penanda jika sensor sukses di-init

    float _dist[NUM_LIDAR];           // hasil EMA (cm)
    int  _hist[NUM_LIDAR][3];         // 3 sampel terakhir (untuk filter median)
    uint8_t _histN[NUM_LIDAR];        // jumlah sampel terkumpul (<=3)
    uint32_t _lastOk[NUM_LIDAR];      // waktu data DALAM JANGKAUAN terakhir
    uint32_t _lastResp[NUM_LIDAR];    // waktu sensor terakhir MENJAWAB (walau jauh)
    bool _jauh[NUM_LIDAR];            // jawaban terakhir: sah tapi di luar LIDAR_MAX_CM

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
