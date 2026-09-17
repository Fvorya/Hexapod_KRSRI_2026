#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>
#include "config.h"

// ARM_SLOT_N, bukan ARM_NUM_SERVOS: yang menentukan ukuran blob adalah slot
// yang DIALOKASIKAN (3 per lengan, sejak dulu), bukan jumlah servo yang
// benar-benar terpasang (4 depan + 1 belakang). Memakai ARM_NUM_SERVOS di sini
// akan mengubah ukuran CalibBlob dan membuang seluruh kalibrasi tersimpan.
#define TOTAL_SERVOS (NUM_SERVOS + 2*ARM_SLOT_N)

enum ParamId {
    K_PULSE_MIN, K_PULSE_MAX, K_ARM_PULSE_MIN, K_ARM_PULSE_MAX,
    K_GAIT_STEP_HEIGHT, K_GAIT_STEP_LENGTH, K_GAIT_CYCLE_TIME, K_GAIT_DUTY,
    K_GAIT_SLEW_RATE, K_GAIT_PROFILE_TAU, K_GAIT_SETTLE_TAU,
    K_STAB_TAU, K_STAB_SIGN_ROLL, K_STAB_SIGN_PITCH,
    K_HEADING_KP, K_HEADING_KD, K_WALL_KP, K_WALL_KD, K_WALL_SETPOINT,
    K_WALL_MIN,
    K_HEAD_UTARA, K_HEAD_TIMUR, K_HEAD_SELATAN, K_HEAD_BARAT,
    K_ARENA_MIRROR,   // 0 = arena hadap kanan (default), 1 = cermin (hadap kiri)
    // SEKUENS CONDONG K-3/K-4. Di tabel parameter, bukan #define, supaya bisa
    // disetel lewat 'Q<nama> <nilai>' lalu disimpan 'W' -- menyetelnya di
    // arena tidak lagi menuntut flash ulang. Itu permintaan R2C 15 Sep 2026,
    // dan alasannya praktis: satu putaran flash memakan menit yang tidak ada
    // saat sesi latihan berjalan.
    K_CONDONG_MM,     // geser badan maju sebelum lengan turun, mm
    K_CONDONG_JEDA,   // jeda konfirmasi antara geser dan meluruskan, ms
    K_CONDONG_YAW,    // 1 = luruskan badan ke heading ruas, 0 = jangan
    N_PARAMS
};

// KAPAN sebuah parameter benar-benar berpengaruh sesudah diubah. Ini bukan
// hiasan: tanpa keterangan ini, mengubah gait.step_height lalu melihat robot
// tidak berubah apa-apa terlihat seperti perintahnya gagal, padahal nilainya
// memang baru masuk saat profil gait di-set ulang.
enum ParamBerlaku : uint8_t {
    P_LANGSUNG = 0,     // dibaca tiap loop -> efeknya seketika
    P_PERLU_B,          // baru masuk lewat profileFlat(), yaitu saat 'b'
    P_SERVO_LEMAS,      // mengubah pemetaan sudut->pulse SELURUH servo sekaligus
    P_BELUM_DIPAKAI     // slotnya ada, tapi belum ada kode yang membacanya
};

// PARAM_DEFS ada di flash dan TIDAK ikut CalibBlob, jadi menambah field di sini
// tidak mengubah tata letak EEPROM dan tidak menuntut kenaikan CALIB_VERSION.
struct ParamDef {const char* name; float def, lo, hi; ParamBerlaku berlaku;};

struct CalibBlob {
    char     magic[2];
    uint8_t  version;
    float    param[N_PARAMS];
    float    offset[TOTAL_SERVOS];
    int16_t  trim[TOTAL_SERVOS];
    uint8_t  invert[TOTAL_SERVOS];
    uint16_t crc;
};

extern const ParamDef PARAM_DEFS[N_PARAMS];
extern CalibBlob gCalib;

#define gParam  (gCalib.param)
#define gOffset (gCalib.offset)
#define gTrim   (gCalib.trim)
#define gInvert (gCalib.invert)

#define SERVO_PULSE_MIN   ((uint16_t)gParam[K_PULSE_MIN])
#define SERVO_PULSE_MAX   ((uint16_t)gParam[K_PULSE_MAX])

#define SERVO_ARM_PULSE_MIN ((uint16_t)gParam[K_ARM_PULSE_MIN])
#define SERVO_ARM_PULSE_MAX ((uint16_t)gParam[K_ARM_PULSE_MAX])

#define SERVO_OFFSET      gOffset
#define SERVO_TRIM_US     gTrim
#define SERVO_INVERT      gInvert

#define GAIT_STEP_HEIGHT  gParam[K_GAIT_STEP_HEIGHT]
#define GAIT_STEP_LENGTH  gParam[K_GAIT_STEP_LENGTH]
#define GAIT_CYCLE_TIME   gParam[K_GAIT_CYCLE_TIME]
#define GAIT_DUTY         gParam[K_GAIT_DUTY]
#define GAIT_SLEW_RATE    gParam[K_GAIT_SLEW_RATE]
#define GAIT_PROFILE_TAU  gParam[K_GAIT_PROFILE_TAU]
#define GAIT_SETTLE_TAU   gParam[K_GAIT_SETTLE_TAU]
#define STAB_TAU          gParam[K_STAB_TAU]

#define HEADING_KP        gParam[K_HEADING_KP]
#define HEADING_KD        gParam[K_HEADING_KD]
#define WALL_KP           gParam[K_WALL_KP]
#define WALL_KD           gParam[K_WALL_KD]
// Setpoint TIDAK lagi dibulatkan ke int. Dulu (int) memaksa jarak dinding
// hanya bisa disetel per 1 cm, padahal celah ujung kaki ke dinding cuma
// beberapa cm -- setengah centimeter benar-benar terasa di situ.
#define WALL_SETPOINT_CM  gParam[K_WALL_SETPOINT]
#define WALL_MIN_CM       gParam[K_WALL_MIN]
#define HEAD_UTARA        gParam[K_HEAD_UTARA]
#define HEAD_TIMUR        gParam[K_HEAD_TIMUR]
#define HEAD_SELATAN      gParam[K_HEAD_SELATAN]
#define HEAD_BARAT        gParam[K_HEAD_BARAT]

#define KORBAN_CONDONG_MM      gParam[K_CONDONG_MM]
#define KORBAN_CONDONG_JEDA_MS ((uint32_t)gParam[K_CONDONG_JEDA])
#define KORBAN_CONDONG_YAW     (gParam[K_CONDONG_YAW] > 0.5f)

namespace Calib {
    void  begin();                              // applyDefaults + load (panggil di setup awal)
    void  applyDefaults();                      // isi gCalib dari PARAM_DEFS + default servo
    bool  load();                               // dari EEPROM; false bila invalid (lalu default)
    void  save();                               // ke EEPROM (+ hitung crc)
    int   findParam(const char* name);          // -1 bila tak ada
    bool  setParam(const char* name, float v);  // clamp ke [lo,hi]; false bila nama tak ada
    uint16_t crc16(const uint8_t* p, uint32_t n);
}

#endif
