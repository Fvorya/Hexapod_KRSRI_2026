#ifndef EEPROM_H_STUB
#define EEPROM_H_STUB
#include <stdint.h>
#include <string.h>
static uint8_t __EE[8192];
struct __EEStub {
    template <class T> T& get(int a, T& t) { memcpy(&t, __EE + a, sizeof(T)); return t; }
    template <class T> const T& put(int a, const T& t) { memcpy(__EE + a, &t, sizeof(T)); return t; }
    unsigned length() const { return 4284; }
};
extern __EEStub EEPROM;
#endif
