#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "Imu.h"
#include "Hexapod.h"

class Navigation {
public:
    // Menerima referensi dari objek IMU dan Hexapod secara langsung
    Navigation(Imu& imuRef, Hexapod& robotRef);

    void begin();

    // --- 1. Kompas Arena ---
    void kompasCatat(uint8_t arah); // 0=U, 1=T, 2=S, 3=B
    void kompasSimpan();
    bool kompasMuat(bool cerewet = true);
    void kompasTabel();

    // --- 2. Pivot Tertutup (PD) ---
    void pivotKe(float targetYaw);
    void pivotKompas(uint8_t arah);
    void pivotRelatif(float der);

    // --- 3. Kalibrasi Kecepatan Putar ---
    void kalibrasiPivot(uint8_t siklus);

private:
    Imu& _imu;
    Hexapod& _robot; // Pointer ke sistem robot utama

    float _headArah[4] = { -1, -1, -1, -1 };
    const char* _arahNama[4] = { "UTARA", "TIMUR", "SELATAN", "BARAT" };
    
    float _degCCW = 0, _degCW = 0;
    int8_t _pivotSign = 1;

    // Fungsi utilitas internal
    float wrap180(float d);
    uint8_t kompasSum(const void* buf, size_t n);
    void gaitPutar(float turn);
    
    // Fungsi simulasi pemblokiran dengan update konstan
    bool tungguYaw(uint32_t ms, float& yawAkum);
    bool tunggu(uint32_t ms);
    void updateSistem();
};

#endif