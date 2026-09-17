#ifndef HEXAARM_H
#define HEXAARM_H

#include "HexaServos.h"
#include "types.h"

class HexaArm {
public:
    // calibBase = index slot kalibrasi milik lengan ini.
    //   lengan kanan = 18 (NUM_SERVOS), lengan kiri = 21.
    // Sebelumnya KEDUA lengan memakai NUM_SERVOS + id, jadi lengan kiri diam-diam
    // memakai offset/trim/invert milik lengan kanan.
    // n = jumlah servo lengan INI (ARM_N_DEPAN 4 / ARM_N_BELAKANG 1). Kedua
    // lengan tidak lagi sama panjang, jadi ia tidak bisa disimpulkan.
    HexaArm(HexaServos* servosDriver, const uint8_t pinMap[][2],
            uint8_t n, uint8_t calibBase);

    void begin();
    void setArmPulse(uint8_t id, uint16_t pulseUs);
    bool commit();

    // Lengan belum terpasang fisik -> default NONAKTIF, tidak pernah diberi
    // pulsa. Hidupkan lewat enable() kalau lengan sudah ada.
    void enable()  { _enabled = true;  }
    void disable() { _enabled = false; }
    bool isEnabled() const { return _enabled; }

    // SASARAN sendi ini, dan pulsa yang BENAR-BENAR sedang dikirim. Keduanya
    // berbeda selama slew berjalan; debugDump() mencetak keduanya, karena
    // "yang diminta" dan "yang sedang dikirim" adalah dua pertanyaan berbeda
    // saat lengan tidak sampai ke tempat yang diminta.
    uint16_t targetPulse(uint8_t id) const { return (id < _n) ? _target[id] : 0; }
    uint16_t pulseKini(uint8_t id)   const { return (id < _n) ? _kini[id]   : 0; }

    // Tahap gerak yang sedang berjalan: 0 bahu+grip, 1 siku+pergelangan.
    // Dipakai debugDump() dan uji PC untuk membedakan "lengan masih merayap"
    // dari "lengan menunggu giliran sendi berikutnya".
    uint8_t tahap() const { return _tahap; }

    // Sasaran sudah tercapai? Sekuens buta memakai jeda tetap, tapi perintah
    // manual dan uji PC butuh cara menanyakannya.
    bool sampai() const {
        for (uint8_t i = 0; i < _n; i++) if (_kini[i] != _target[i]) return false;
        return true;
    }

    // LAJU SLEW lengan, derajat/detik. Baku ARM_SLEW_DEG_S; sekuens korban
    // memperlambatnya untuk lipatan ke REHAT, yang dikerjakan sambil capit
    // menggenggam. Disetel per gerakan, bukan per lengan selamanya -- yang
    // menyetelnya WAJIB mengembalikannya, kalau tidak perintah 'as' berikutnya
    // ikut lambat tanpa sebab yang kelihatan.
    void  setSlew(float degS) { _slewDegS = (degS > 1.0f) ? degS : 1.0f; }
    float slew() const        { return _slewDegS; }

    uint16_t angleToPulse(uint8_t id, float geoAngleDeg, float baseline);

    // Sudut servo yang AKAN dipakai angleToPulse, SEBELUM clamp 0..180.
    // angleToPulse() meng-clamp tanpa penanda apa pun -- tidak seperti kaki
    // yang menyalakan _servoClamped -- jadi pemanggil yang perlu tahu posenya
    // mentok harus bisa menanyakannya lebih dulu. Indeks slot kalibrasi
    // (_calibBase + id) cuma diketahui di sini, jadi hitungannya juga.
    float sudutServo(uint8_t id, float geoAngleDeg, float baseline) const;

private:
    bool tahapSampai(uint8_t tahap) const;

    HexaServos* _driver;
    const uint8_t (*_pinMap)[2];
    uint8_t _calibBase;
    uint8_t _n;
    bool _enabled;

    uint16_t _target[ARM_NUM_SERVOS];
    uint16_t _kini[ARM_NUM_SERVOS];    // pulsa yang sedang dikirim (hasil slew)
    unsigned long _lastUpdate;
    float _slewDegS = ARM_SLEW_DEG_S;
    // LAJU SESAAT tiap sendi, der/detik -- bukan sasaran, melainkan yang
    // sedang berlaku setelah dibatasi ARM_ACCEL_DEG_S2. Per sendi, bukan satu
    // untuk seluruh tahap: satu laju bersama berarti rem dihitung dari sendi
    // yang paling jauh, dan sendi yang lebih dekat berhenti MENDADAK saat
    // sampai -- persis hentakan yang sedang dihilangkan. Sendi di luar tahap
    // yang sedang berjalan dinolkan, jadi ia selalu mulai dari diam.
    float _laju[ARM_NUM_SERVOS] = {0};
    uint8_t _tahap = 0;          // sendi mana yang sedang boleh bergerak
};

#endif
