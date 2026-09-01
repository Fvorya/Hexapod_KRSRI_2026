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
    HexaArm(HexaServos* servosDriver, const uint8_t pinMap[ARM_NUM_SERVOS][2],
            uint8_t calibBase);

    void begin();
    void setArmPulse(uint8_t id, uint16_t pulseUs);
    bool commit();

    // Lengan belum terpasang fisik -> default NONAKTIF, tidak pernah diberi
    // pulsa. Hidupkan lewat enable() kalau lengan sudah ada.
    void enable()  { _enabled = true;  }
    void disable() { _enabled = false; }
    bool isEnabled() const { return _enabled; }

    uint16_t angleToPulse(uint8_t id, float geoAngleDeg, float baseline);

private:
    HexaServos* _driver;
    const uint8_t (*_pinMap)[2];
    uint8_t _calibBase;
    bool _enabled;

    uint16_t _target[ARM_NUM_SERVOS];
    unsigned long _lastUpdate;
};

#endif
