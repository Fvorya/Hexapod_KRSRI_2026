#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "config.h"
#include "types.h"

class Imu {
public:
    Imu();
    void begin();
    void update();                 // Non-blocking, panggil tiap loop (Berisi logika witPump)

    // roll/pitch relatif tare (0 saat datar). yaw absolut (kompas).
    float rollDeg()  { return _roll  - _roll0; }
    float pitchDeg() { return _pitch - _pitch0; }
    float yawDeg()   { return _yaw; }

    void tare() { _roll0 = _roll; _pitch0 = _pitch; }  // panggil saat robot datar & diam
    bool hasData() const { return _have; }

    float gyroZ() const { return _gz; }
    float accelZ() const { return _az; }
    float magMagnitude() const { return sqrtf(_mhx * _mhx + _mhy * _mhy + _mhz * _mhz); }

private:
    static const int WIT_LEN = 11;
    static const int RX_EXTRA = 2048;

    uint8_t _rxExtra[RX_EXTRA]; // Injeksi buffer ekstra 2KB
    uint8_t _rxBuf[64];         // Buffer antrean pembacaan (pengganti array _rx lama)
    uint8_t _rxN;               // Counter jumlah byte saat ini di dalam _rxBuf

    bool _have;
    float _roll, _pitch, _yaw;
    float _roll0, _pitch0;
    float _gz;                  // Penampung Gyro Z
    uint8_t _tolakYaw;          // penolakan lonjakan yaw berturut-turut
    float _ax, _ay, _az;        // Percepatan X, Y, Z (Accel)
    float _mhx, _mhy, _mhz;     // Medan magnet X, Y, Z (Magnet)

    void parseFrame(const uint8_t* f); // Pengganti fungsi witFrame()
};

#endif