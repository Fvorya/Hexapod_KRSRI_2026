#include "HexaArm.h"

HexaArm::HexaArm(HexaServos* servosDriver, const uint8_t pinMap[ARM_NUM_SERVOS][2],
                 uint8_t calibBase) {
    _driver    = servosDriver;
    _pinMap    = pinMap;
    _calibBase = calibBase;
    _enabled   = false;      // lengan belum terpasang -> jangan digerakkan
    _lastUpdate = 0;
    // Konstruktor jalan sebelum Calib::load(), jadi jangan hitung center di sini.
    for (int i = 0; i < ARM_NUM_SERVOS; i++) _target[i] = 0;
}

void HexaArm::begin() {
    // Sama seperti kaki: isi target, TAPI jangan kirim apa pun. PWM lengan
    // tetap mati sampai enable() dipanggil.
    uint16_t centerPulse = (SERVO_ARM_PULSE_MIN + SERVO_ARM_PULSE_MAX) / 2;
    for (int i = 0; i < ARM_NUM_SERVOS; i++) _target[i] = centerPulse;
    _lastUpdate = millis();
}

void HexaArm::setArmPulse(uint8_t id, uint16_t pulseUs) {
    if (id >= ARM_NUM_SERVOS) return; // Proteksi khusus lengan (0-2)
    _target[id] = constrain((int)pulseUs, SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
}

uint16_t HexaArm::angleToPulse(uint8_t id, float geoAngleDeg, float baseline) {
    // 1. Index slot kalibrasi milik LENGAN INI (18-20 kanan, 21-23 kiri)
    uint8_t k = _calibBase + id;

    // 2. Terapkan offset dari memori, dan cek apakah perlu di-invert
    float s = SERVO_INVERT[k] ? -geoAngleDeg : geoAngleDeg;
    float servoAngle = baseline + SERVO_OFFSET[k] + s;
    servoAngle = clampf(servoAngle, 0.0f, 180.0f);

    // 3. Gunakan rentang pulse khusus lengan
    int pulse = SERVO_ARM_PULSE_MIN +
                (int)((servoAngle / 180.0f) * (SERVO_ARM_PULSE_MAX - SERVO_ARM_PULSE_MIN));

    pulse += SERVO_TRIM_US[k]; // Terapkan trim

    return (uint16_t)constrain(pulse, SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
}

bool HexaArm::commit() {
    if (!_enabled) return false;                        // gerbang keselamatan
    if (!_driver->isEnabled()) return false;            // ikut status PWM global
    if (millis() - _lastUpdate < 20) return false;
    _lastUpdate = millis();

    for (int i = 0; i < ARM_NUM_SERVOS; i++) {
        _driver->writeRaw(_pinMap[i][0], _pinMap[i][1], _target[i]);
    }
    return true;
}
