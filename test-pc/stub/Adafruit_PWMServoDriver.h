#ifndef PCA_H_STUB
#define PCA_H_STUB
#include <stdint.h>
#include <Wire.h>

// Perekam pulse terakhir per (driver, channel). Dipakai sim_servolaju untuk
// mengukur tuntutan kecepatan servo langsung dari yang BENAR-BENAR dikirim.
extern uint16_t __servoUs[2][16];

class Adafruit_PWMServoDriver {
public:
    Adafruit_PWMServoDriver(uint8_t addr, __WireStub&) : _slot(addr == 0x41 ? 1 : 0) {}
    void begin() {}
    void setPWMFreq(float) {}
    void setPWM(uint8_t ch, uint16_t, uint16_t off) { if (ch < 16 && off == 0) __servoUs[_slot][ch] = 0; }
    void writeMicroseconds(uint8_t ch, uint16_t us) { if (ch < 16) __servoUs[_slot][ch] = us; }
private:
    uint8_t _slot;
};
#endif
