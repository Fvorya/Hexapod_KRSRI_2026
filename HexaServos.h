#ifndef HEXASERVOS_H
#define HEXASERVOS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Calib.h"

#define PCA9685_0_ADDR 0x41
#define PCA9685_1_ADDR 0x40

#define SERVO_FREQ SERVO_PWM_FREQ   // Dari config.h (50 Hz)

// Jeda antar servo saat pengaktifan pertama (posisi awal belum diketahui).
// Sama dengan staggerKe() di TES_GERAK: menyebar lonjakan arus 18 servo
// supaya BEC tidak drop dan Teensy tidak brownout.
#define SERVO_STAGGER_MS  60
#define SERVO_RAMP_MS    400        // ramp halus bila posisi awal sudah diketahui

class HexaServos {
public:
    HexaServos();
    void begin();

    // Set target pulse (us) satu servo kaki (0..NUM_SERVOS-1). Disimpan, dikirim saat commit().
    void setLegPulse(uint8_t legServoID, uint16_t pulseUs);
    uint16_t targetPulse(uint8_t legServoID) const;

    // Kirim 18 pulse kaki bila sudah waktunya (tiap 20ms). Panggil di loop.
    // return true bila benar-benar mengirim siklus ini.
    // TIDAK mengirim apa pun selama belum enable() -- lihat catatan di bawah.
    bool commit();

    // --- KESELAMATAN: robot boot dalam keadaan PWM MATI ---
    // begin() TIDAK lagi menulis pulse tengah 1500 us ke 18 servo. Dulu itulah
    // hentakan pertama saat Teensy reset (termasuk reset otomatis sesudah
    // upload). Sekarang semua channel di-setPWM(0,0) = output mati, servo bebas,
    // dan baru hidup ketika enable() dipanggil dari perintah serial.
    void allOff();                 // matikan 16 channel di KEDUA driver
    void enable();                 // aktifkan output (stagger/ramp ke _target)
    void disable();                // = allOff() + tandai nonaktif
    bool isEnabled() const { return _enabled; }

    // Tulis langsung ke channel mana pun (untuk lengan/aux). Bukan via timer.
    void writeRaw(uint8_t driver, uint8_t channel, uint16_t pulseUs);

private:
    Adafruit_PWMServoDriver _kit0 = Adafruit_PWMServoDriver(PCA9685_0_ADDR, SERVO_0_I2C_BUS);
    Adafruit_PWMServoDriver _kit1 = Adafruit_PWMServoDriver(PCA9685_1_ADDR, SERVO_1_I2C_BUS);
    uint16_t _target[NUM_SERVOS];
    uint16_t _last[NUM_SERVOS];    // pulse yang benar-benar terkirim terakhir
    bool _enabled;                 // output PWM hidup?
    bool _driven;                  // servo pernah diberi pulsa sejak enable?
    unsigned long _lastUpdate;
    static const unsigned int UPDATE_MS = SERVO_COMMIT_MS;

    void staggerTo();              // satu per satu, jeda SERVO_STAGGER_MS
    void rampTo();                 // interpolasi _last -> _target, SERVO_RAMP_MS
};

#endif
