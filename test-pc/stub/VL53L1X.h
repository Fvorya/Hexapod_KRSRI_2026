// Stub VL53L1X -- HANYA untuk uji di PC. Bentuk API mengikuti dokumentasi
// pustaka "VL53L1X by Pololu" (README + VL53L1X.cpp). Nilai numerik enum
// RangeStatus di sini SEMBARANG: firmware hanya memakai namanya.
#ifndef VL_H_STUB
#define VL_H_STUB
#include <stdint.h>
#include <Wire.h>
extern uint16_t __simMm[8];      // jarak per channel mux
extern uint8_t  __simStatus[8];  // range_status per channel (lihat enum)
extern bool __initGagal[8];      // init() gagal di channel ini
extern bool __bisu[8];           // init OK tapi dataReady() tak pernah true
extern uint32_t __simPeriodMs;   // jeda antar pengukuran yang ditiru (ms)
extern uint32_t __simLastMeas[8];
extern bool __simJalan[8];        // mode kontinu aktif di channel ini?
extern bool __simCrosstalkAntar;  // tiru crosstalk antar-sensor
extern uint16_t __simHantuMm[8];  // hantu yang muncul bila tetangga memancar

class VL53L1X {
public:
    enum DistanceMode { Short, Medium, Long };
    enum RangeStatus : uint8_t {
        RangeValid = 0, SigmaFail, SignalFail, RangeValidMinRangeClipped,
        OutOfBoundsFail, HardwareFail, RangeValidNoWrapCheckFail,
        WrapTargetFail, XtalkSignalFail, SynchronizationInt, MinRangeFail, None
    };
    struct RangingData {
        uint16_t    range_mm;
        RangeStatus range_status;
        float peak_signal_count_rate_MCPS;
        float ambient_count_rate_MCPS;
    } ranging_data;

    void setBus(__WireStub*) {}
    void setTimeout(uint16_t) {}
    bool init(bool = true) { return __muxAda && !__initGagal[__muxCh & 7]; }
    bool setDistanceMode(DistanceMode) { return true; }
    bool setMeasurementTimingBudget(uint32_t us) { return us >= 20000; }
    void setROISize(uint8_t, uint8_t) {}
    void startContinuous(uint32_t) { __simJalan[__muxCh & 7] = true; }
    void stopContinuous()           { __simJalan[__muxCh & 7] = false; }
    // Sensor nyata hanya menghasilkan SATU pengukuran per jeda antar
    // pengukuran. Stub yang selalu bilang "siap" membuat uji filter menipu:
    // satu keadaan fisik terhitung sebagai banyak sampel berturut-turut.
    bool dataReady() {
        int c = __muxCh & 7;
        if (__bisu[c]) return false;
        return (uint32_t)(__nowMs - __simLastMeas[c]) >= __simPeriodMs;
    }
    uint16_t read(bool = true) {
        int c = __muxCh & 7;
        __simLastMeas[c] = __nowMs;
        ranging_data.range_mm     = __simMm[c];
        ranging_data.range_status = (RangeStatus)__simStatus[c];
        // Crosstalk antar-sensor: hantu pendek HANYA muncul kalau ada sensor
        // LAIN yang ikut memancar. Inilah yang dipisahkan uji isolasi.
        if (__simCrosstalkAntar) {
            int tetangga = 0;
            for (int k = 0; k < 8; k++) if (k != c && __simJalan[k]) tetangga++;
            if (tetangga > 0) {
                ranging_data.range_mm     = __simHantuMm[c];
                ranging_data.range_status = RangeValid;
            }
        }
        return ranging_data.range_mm;
    }
    uint16_t readRangeContinuousMillimeters(bool b = true) { return read(b); }
    bool timeoutOccurred() { return false; }
    static const char* rangeStatusToString(RangeStatus st) {
        switch (st) {
            case RangeValid:                return "range valid";
            case SigmaFail:                 return "sigma fail";
            case SignalFail:                return "signal fail";
            case RangeValidMinRangeClipped: return "range valid, min range clipped";
            case OutOfBoundsFail:           return "out of bounds fail";
            case HardwareFail:              return "hardware fail";
            case RangeValidNoWrapCheckFail: return "range valid, no wrap check fail";
            case WrapTargetFail:            return "wrap target fail";
            case XtalkSignalFail:           return "xtalk signal fail";
            case SynchronizationInt:        return "synchronization int";
            case MinRangeFail:              return "min range fail";
            default:                        return "unknown";
        }
    }
    uint8_t readReg(uint16_t) { return 0; }
};
#endif
