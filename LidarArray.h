#ifndef LIDARARRAY_H
#define LIDARARRAY_H

// 6x VL53L0X (TOF200C) via mux TCA9548A, NON-BLOCKING (state machine round-robin).
// update() memajukan SATU sensor per panggilan tanpa busy-wait -> loop utama tetap kencang.
// Filter: Register Interrupt -> median-3 (buang outlier) -> EMA. 
// getDistance() = nilai terakhir valid, atau -1 bila status buruk / sensor mati (timeout).

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h> // Pastikan Anda sudah menginstal library "VL53L0X by Pololu"
#include "config.h"

class LidarArray {
public:
    LidarArray();
    bool begin();
    void update();                 // non-blocking (dipanggil di loop utama)
    int  getDistance(uint8_t id);  // cm (terakhir valid), -1 jika error

private:
    VL53L0X _sensor[NUM_LIDAR];       // Wajib: 1 Objek per channel mux
    bool _isReady[NUM_LIDAR];         // Penanda jika sensor sukses di-init

    float _dist[NUM_LIDAR];           // hasil EMA (cm)
    int  _hist[NUM_LIDAR][3];         // 3 sampel terakhir (untuk filter median)
    uint8_t _histN[NUM_LIDAR];        // jumlah sampel terkumpul (<=3)
    uint32_t _lastOk[NUM_LIDAR];      // waktu data valid terakhir (untuk fail-safe)
    
    uint8_t _cur;                     // sensor yang sedang diproses di state machine
    
    void selectMux(uint8_t ch);
};

#endif