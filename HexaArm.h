#ifndef HEXAARM_H
#define HEXAARM_H

#include "HexaServos.h"
#include "types.h"

class HexaArm {
public:
    HexaArm(HexaServos* servosDriver, const uint8_t pinMap[ARM_NUM_SERVOS][2]);

    void begin();
    void setArmPulse(uint8_t id, uint16_t pulseUs);
    bool commit();
    
    uint16_t angleToPulse(uint8_t id, float geoAngleDeg, float baseline);

private:
    HexaServos* _driver;
    const uint8_t (*_pinMap)[2];

    uint16_t _target[ARM_NUM_SERVOS];
    unsigned long _lastUpdate;
};

#endif
