#include "HexaArm.h"

HexaArm::HexaArm(HexaServos* servosDriver, const uint8_t pinMap[ARM_NUM_SERVOS][2]) {
    uint16_t centerPulse = (SERVO_ARM_PULSE_MIN + SERVO_ARM_PULSE_MAX) / 2;
    
    _driver = servosDriver;
    _pinMap = pinMap; 
    _lastUpdate = 0;
    for (int i = 0; i < ARM_NUM_SERVOS; i++) {
        _target[i] = centerPulse;
    }
}

void HexaArm::begin() {
    uint16_t centerPulse = (SERVO_ARM_PULSE_MIN + SERVO_ARM_PULSE_MAX) / 2;

    for (int i = 0; i < ARM_NUM_SERVOS; i++) {
        _target[i] = centerPulse;
        // Gunakan _driver untuk mengirim sinyal
        _driver->writeRaw(_pinMap[i][0], _pinMap[i][1], centerPulse); 
    }
    _lastUpdate = millis();
}

void HexaArm::setArmPulse(uint8_t id, uint16_t pulseUs) {
    if (id >= ARM_NUM_SERVOS) return; // Proteksi khusus lengan (0-2)
    _target[id] = constrain((int)pulseUs, SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
}

uint16_t HexaArm::angleToPulse(uint8_t id, float geoAngleDeg, float baseline) {
    // 1. Dapatkan index aktual di EEPROM
    int eepromIndex = NUM_SERVOS + id; // Jika id 0 (Bahu), maka indexnya 18
    
    // 2. Terapkan offset dari memori, dan cek apakah perlu di-invert
    float s = SERVO_INVERT[eepromIndex] ? -geoAngleDeg : geoAngleDeg;
    float servoAngle = baseline + SERVO_OFFSET[eepromIndex] + s;
    servoAngle = clampf(servoAngle, 0.0f, 180.0f);
    
    // 3. Gunakan konstanta EEPROM yang baru kita buat
    int pulse = SERVO_ARM_PULSE_MIN + 
                (int)((servoAngle / 180.0f) * (SERVO_ARM_PULSE_MAX - SERVO_ARM_PULSE_MIN));
    
    pulse += SERVO_TRIM_US[eepromIndex]; // Terapkan trim
    
    return (uint16_t)constrain(pulse, SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
}

bool HexaArm::commit() {
    if (millis() - _lastUpdate < 20) return false; 
    _lastUpdate = millis();
    
    for (int i = 0; i < ARM_NUM_SERVOS; i++) {
        // Gunakan _driver->writeRaw, bukan _kit0
        _driver->writeRaw(_pinMap[i][0], _pinMap[i][1], _target[i]);
    }
    return true;
}
