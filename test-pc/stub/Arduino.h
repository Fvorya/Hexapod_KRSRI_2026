// Stub Arduino.h -- HANYA untuk pemeriksaan sintaks & simulasi di PC.
// Bukan bagian dari sketsa; tidak pernah ikut dikompilasi Teensyduino.
#ifndef ARDUINO_H_STUB
#define ARDUINO_H_STUB

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <deque>

#define HEX 16
#define DEC 10

// Jam palsu yang bisa dimajukan simulasi.
extern uint32_t __nowMs;
static inline uint32_t millis() { return __nowMs; }
static inline void delay(uint32_t ms) { __nowMs += ms; }

template <class T, class L, class H>
static inline T constrain(T v, L lo, H hi) {
    return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v);
}

// Diam secara default supaya keluaran simulasi bersih; __serialDiam=false
// untuk melihat pesan firmware apa adanya.
extern bool __serialDiam;

struct __SerialStub {
    std::deque<uint8_t> rx;          // byte yang "masuk" ke Teensy

    void begin(unsigned long) {}
    void addMemoryForRead(void*, size_t) {}
    int  available() { return (int)rx.size(); }
    int  read() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
    operator bool() const { return true; }

    void p(const char* f, ...) {
        if (__serialDiam) return;
        va_list a; va_start(a, f); vprintf(f, a); va_end(a);
    }
    void print(const char* s) { p("%s", s); }
    void print(char c)        { p("%c", c); }
    void print(int v)         { p("%d", v); }
    void print(unsigned v)    { p("%u", v); }
    void print(long v)        { p("%ld", v); }
    void print(unsigned long v) { p("%lu", v); }
    void print(double v)      { p("%.2f", v); }
    void print(int v, int)    { p("%X", v); }
    void print(unsigned v, int) { p("%X", v); }
    void print(double v, int d) { p("%.*f", d, v); }

    void println()                 { p("\n"); }
    void println(const char* s)    { p("%s\n", s); }
    void println(char c)           { p("%c\n", c); }
    void println(int v)            { p("%d\n", v); }
    void println(unsigned v)       { p("%u\n", v); }
    void println(long v)           { p("%ld\n", v); }
    void println(unsigned long v)  { p("%lu\n", v); }
    void println(double v)         { p("%.2f\n", v); }
    void println(int v, int)       { p("%X\n", v); }
    void println(unsigned v, int)  { p("%X\n", v); }
    void println(double v, int d)  { p("%.*f\n", d, v); }

    __attribute__((format(printf, 2, 3)))
    int printf(const char* f, ...) {
        if (__serialDiam) return 0;
        va_list a; va_start(a, f); int n = vprintf(f, a); va_end(a); return n;
    }
};

extern __SerialStub Serial;
extern __SerialStub Serial2;

#endif
