#include "HexaServos.h"

HexaServos::HexaServos() {
    _lastUpdate = 0;
    uint16_t center = (SERVO_PULSE_MIN + SERVO_PULSE_MAX) / 2;
    for (int i = 0; i < NUM_SERVOS; i++) _target[i] = center;
}

void HexaServos::begin() {
    _kit0.begin();
    // _kit1.begin();
    SERVO_I2C_BUS.setClock(SERVO_I2C_CLOCK);   // bus servo terpisah, kencang
    _kit0.setPWMFreq(SERVO_FREQ);
    // _kit1.setPWMFreq(SERVO_FREQ);
    uint16_t center = (SERVO_PULSE_MIN + SERVO_PULSE_MAX) / 2;
    for (int i = 0; i < NUM_SERVOS; i++) {
        _target[i] = center;
        writeRaw(SERVO_PIN_MAP[i][0], SERVO_PIN_MAP[i][1], center);
    }
    _lastUpdate = millis();
}

void HexaServos::setLegPulse(uint8_t id, uint16_t pulseUs) {
    if (id >= NUM_SERVOS) return;
    _target[id] = constrain((int)pulseUs, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

bool HexaServos::commit() {
    if (millis() - _lastUpdate < UPDATE_MS) return false;
    _lastUpdate = millis();
    for (int i = 0; i < NUM_SERVOS; i++) {
        writeRaw(SERVO_PIN_MAP[i][0], SERVO_PIN_MAP[i][1], _target[i]);
    }
    return true;
}

void HexaServos::writeRaw(uint8_t driver, uint8_t channel, uint16_t pulseUs) {
    if (driver == 0) _kit0.writeMicroseconds(channel, pulseUs);
    else             _kit1.writeMicroseconds(channel, pulseUs);
}
