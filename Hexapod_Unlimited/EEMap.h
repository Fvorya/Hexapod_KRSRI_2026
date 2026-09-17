#ifndef EEMAP_H
#define EEMAP_H

/* =====================================================================
   EEMap.h -- SATU-SATUNYA tempat tata letak EEPROM didefinisikan.

   Empat blok di EEPROM ditulis oleh EMPAT program yang tidak pernah
   dikompilasi bersama, jadi satu-satunya "kontrak" di antara mereka adalah
   alamat yang dipatok + tata letak struct yang harus sama persis.

   Dulu struct-nya disalin diam-diam ke beberapa file (GerakStore & ServoMap
   di Hexapod.cpp, KompasStore di Navigation.cpp). Kalau satu salinan berubah
   dan yang lain tidak, checksum tetap lolos tapi field-nya salah baca --
   kerusakan senyap yang paling sulit dilacak. File ini menghapus salinan itu
   dan menggantinya dengan satu definisi + penjaga static_assert.

   BILA MENGUBAH STRUCT DI SINI: ubah juga di program penulisnya
   (TES_SERVO/SET_HOME, TES_IMU, TES_GERAK), lalu perbarui angka ukuran di
   static_assert paling bawah supaya kompilasi menolak kalau lupa.
   ===================================================================== */

#include <Arduino.h>
#include <EEPROM.h>
#include <stddef.h>
#include "config.h"
#include "Calib.h"
#include "HexaGait.h"

struct ProfilStore {
    uint16_t magic;
    uint8_t version, mask;
    GaitProfile nilai[6];
    uint16_t crc;
};
static_assert(sizeof(ProfilStore) == 128, "Layout profil EEPROM berubah");
static_assert(EE_GERAK_ADDR + 80 <= EE_PROFIL_ADDR, "Profil menimpa kalibrasi gerak");
static_assert(EE_PROFIL_ADDR + sizeof(ProfilStore) <= EE_TOTAL_BYTES, "Profil melebihi EEPROM");

// ------------------------------------------------------------ 1024
// Ditulis TES_SERVO / SET_HOME. Layout = servo_map.h.
// Firmware hanya memakai invert[] & trim[]; drv[]/ch[] diabaikan karena
// penomoran driver di sana kebalikan dari HexaServos (lihat Hexapod.cpp).
#define EE_SM_SLOTS 24
struct ServoMap {
    char     magic[2];              // 'S','M'
    uint8_t  version;               // 1
    int8_t   drv[EE_SM_SLOTS];      // 0/1, -1 = belum dipetakan
    int8_t   ch[EE_SM_SLOTS];       // 0..15
    uint8_t  invert[EE_SM_SLOTS];   // 1 = arah sendi dibalik
    int16_t  trim[EE_SM_SLOTS];     // us, koreksi netral per servo
    uint16_t crc;                   // CRC16-CCITT (Calib::crc16)
};

// NAMA SLOT, urutan sama dengan blob di atas. Disalin dari
// legacy-2026/KALIBRASI/servo_map.h pada 18 Sep 2026, waktu penyetelan trim
// pindah ke firmware utama: sesudah sketsa legacy pensiun untuk trim, nama
// slot butuh SATU pemilik, dan pemiliknya di sebelah struct yang ia namai.
//
// Dipakai 'Yt' dan tab trim di HUD. Nomor slotnya yang dikirim lewat serial,
// bukan namanya -- nama ini untuk mata manusia.
static const char* const SLOT_NAMA[EE_SM_SLOTS] = {
    "K0_COXA",  "K0_FEMUR",  "K0_TIBIA",     // Ka-Depan
    "K1_COXA",  "K1_FEMUR",  "K1_TIBIA",     // Ka-Tengah
    "K2_COXA",  "K2_FEMUR",  "K2_TIBIA",     // Ka-Belakang
    "K3_COXA",  "K3_FEMUR",  "K3_TIBIA",     // Ki-Belakang
    "K4_COXA",  "K4_FEMUR",  "K4_TIBIA",     // Ki-Tengah
    "K5_COXA",  "K5_FEMUR",  "K5_TIBIA",     // Ki-Depan
    "ARMR_BASE", "ARMR_SHOULDER", "ARMR_GRIP",
    "ARML_BASE", "ARML_SHOULDER", "ARML_GRIP"
};

// BATAS TRIM, mikrodetik. Trim itu koreksi netral, bukan pemetaan: pada
// 500..2500 us untuk 180 der, 200 us sudah 18 der. Yang menuntut lebih dari
// itu bukan trim yang kurang, melainkan horn terpasang di gigi yang salah
// atau invert yang terbalik -- dan menutupinya dengan trim besar membuang
// setengah jangkauan servo di satu ujung.
#define TRIM_MAKS_US  200

// ------------------------------------------------------------ 1792
// Ditulis TES_IMU dan perintah 'e' di firmware.
struct KompasStore {
    uint8_t m0, m1, ver;            // 0xC0, 0x3A, 1
    float   head[4];                // heading IMU utk U/T/S/B arena
    uint8_t sum;                    // eeSum()
};

// ------------------------------------------------------------ 2048
// Ditulis TES_GERAK. Layout WAJIB identik dengan struct di TES_GERAK.ino.
struct GerakStore {
    uint8_t m0, m1, ver;            // 0x6E, 0x2C, 2
    float   ccw, cw, maju;          // derajat/siklus CCW & CW, mm/siklus maju
    int8_t  sign;                   // tanda putar hasil kalibrasi
    float   lvlR, lvlP;             // trim rata badan (derajat)
    float   refR, refP;             // pembacaan IMU saat badan dinyatakan RATA
    float   jac[4];                 // d(imu roll,pitch)/d(perintah roll,pitch)
    float   zoff[6];                // offset tinggi telapak per kaki (mm)
    uint8_t sum;                    // eeSum()
};

// Checksum 8-bit milik TES_GERAK / TES_IMU. Lemah (1 dari 256 data rusak
// bisa lolos) tapi formatnya ditentukan program penulis -- jangan diubah
// tanpa ikut mengubah program itu. Blok firmware sendiri pakai CRC16.
static inline uint8_t eeSum(const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    uint8_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = (uint8_t)(acc + p[i] * 31 + 7);
    return acc;
}

// =====================================================================
// PENJAGA TATA LETAK -- dicek saat KOMPILASI, bukan saat robot berjalan.
// =====================================================================

// 1) Ukuran struct dipatok. Kalau ada field ditambah/dihapus/diubah tipenya,
//    kompilasi GAGAL di sini -- bukan diam-diam salah baca di lapangan.
static_assert(sizeof(ServoMap)    == 126, "ServoMap berubah -- samakan dgn servo_map.h di TES_SERVO");
static_assert(sizeof(KompasStore) ==  24, "KompasStore berubah -- samakan dgn TES_IMU");
static_assert(sizeof(GerakStore)  ==  80, "GerakStore berubah -- samakan dgn TES_GERAK.ino");

// 2) Tidak boleh ada blok yang tumbuh sampai menabrak tetangganya.
//    Inilah penjaga yang selama ini tidak ada: EEPROM.put() menulis persis
//    sizeof(struct) tanpa memeriksa apa pun.
static_assert(EE_CALIB_ADDR    + sizeof(CalibBlob)   <= EE_SERVOMAP_ADDR,
              "CalibBlob tumbuh melewati ServoMap (1024) -- geser EE_SERVOMAP_ADDR");
static_assert(EE_SERVOMAP_ADDR + sizeof(ServoMap)    <= EE_KOMPAS_ADDR,
              "ServoMap tumbuh melewati KompasStore (1792)");
static_assert(EE_KOMPAS_ADDR   + sizeof(KompasStore) <= EE_GERAK_ADDR,
              "KompasStore tumbuh melewati GerakStore (2048)");
static_assert(EE_GERAK_ADDR    + sizeof(GerakStore)  <= EE_TOTAL_BYTES,
              "GerakStore melewati ujung EEPROM Teensy");

// Alamat teratas yang benar-benar dipakai -- untuk cek runtime di bawah.
#define EE_TERPAKAI_TERATAS (EE_PROFIL_ADDR + (uint32_t)sizeof(ProfilStore))

// Cetak peta + pastikan chip-nya memang sebesar yang diasumsikan config.h.
// EE_TOTAL_BYTES hanya dugaan saat kompilasi; EEPROM.length() adalah
// kenyataan. Kalau firmware dipindah ke Teensy lain, ini yang menangkap.
static inline bool eeMapPeriksa(bool cerewet = true) {
    uint32_t nyata = (uint32_t)EEPROM.length();
    bool ok = (nyata >= EE_TERPAKAI_TERATAS);

    if (cerewet) {
        Serial.println("\n--- PETA EEPROM ---");
        Serial.printf("  %-12s %5d .. %-5u  (%u B)  firmware sendiri\n",
                      "CalibBlob",   EE_CALIB_ADDR,
                      (unsigned)(EE_CALIB_ADDR + sizeof(CalibBlob) - 1), (unsigned)sizeof(CalibBlob));
        Serial.printf("  %-12s %5d .. %-5u  (%u B)  TES_SERVO/SET_HOME\n",
                      "ServoMap",    EE_SERVOMAP_ADDR,
                      (unsigned)(EE_SERVOMAP_ADDR + sizeof(ServoMap) - 1), (unsigned)sizeof(ServoMap));
        Serial.printf("  %-12s %5d .. %-5u  (%u B)  TES_IMU\n",
                      "KompasStore", EE_KOMPAS_ADDR,
                      (unsigned)(EE_KOMPAS_ADDR + sizeof(KompasStore) - 1), (unsigned)sizeof(KompasStore));
        Serial.printf("  %-12s %5d .. %-5u  (%u B)  TES_GERAK\n",
                      "GerakStore",  EE_GERAK_ADDR,
                      (unsigned)(EE_GERAK_ADDR + sizeof(GerakStore) - 1), (unsigned)sizeof(GerakStore));
        Serial.printf("  kapasitas chip: %u B, terpakai s/d %u B\n",
                      (unsigned)nyata, (unsigned)EE_TERPAKAI_TERATAS);
        Serial.printf("  ProfilStore  %d .. %u (%u B), enam profil medan\n", EE_PROFIL_ADDR,
                      (unsigned)(EE_PROFIL_ADDR + sizeof(ProfilStore) - 1), (unsigned)sizeof(ProfilStore));
    }

    if (!ok) {
        Serial.println("!! EEPROM chip LEBIH KECIL dari peta -- blok teratas akan gagal ditulis!");
        Serial.println("!! Periksa EE_TOTAL_BYTES di config.h terhadap papan yang dipakai.");
    }
    return ok;
}

#endif
