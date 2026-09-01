#ifndef WIRE_H_STUB
#define WIRE_H_STUB
#include <stdint.h>
// Stub mencatat channel mux yang terakhir dipilih, supaya stub VL53L0X bisa
// mengembalikan jarak yang berbeda per channel (seperti mux sungguhan).
extern int __muxCh;
extern bool __muxAda;
struct __WireStub {
    uint8_t _addr = 0;
    void begin() {}
    void setClock(uint32_t) {}
    void beginTransmission(uint8_t a) { _addr = a; }
    void write(uint8_t v) {
        if (_addr == 0x70) { for (int i = 0; i < 8; i++) if (v & (1 << i)) __muxCh = i; }
    }
    uint8_t endTransmission() {
        if (_addr == 0x70 && !__muxAda) return 2;   // mux tidak menjawab
        return 0;
    }
};
extern __WireStub Wire, Wire1, Wire2;
#endif
