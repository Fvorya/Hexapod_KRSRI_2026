#ifndef VL_H_STUB
#define VL_H_STUB
#include <stdint.h>
#include <Wire.h>
extern uint16_t __simMm[8];
extern bool __initGagal[8];   // init() gagal di channel ini
extern bool __bisu[8];       // init OK tapi tak pernah 'data siap'     // jarak per channel mux, mm
class VL53L0X {
public:
    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
    void setBus(__WireStub*) {}
    void setTimeout(uint16_t) {}
    bool init() { return __muxAda && !__initGagal[__muxCh & 7]; }
    void setSignalRateLimit(float) {}
    void setVcselPulsePeriod(vcselPeriodType, uint8_t) {}
    bool setMeasurementTimingBudget(uint32_t) { return true; }
    void startContinuous(uint32_t) {}
    uint8_t readReg(uint8_t) { return __bisu[__muxCh & 7] ? 0x00 : 0x07; }              // selalu "data siap"
    uint16_t readRangeContinuousMillimeters() { return __simMm[__muxCh & 7]; }
    bool timeoutOccurred() { return false; }
};
#endif
