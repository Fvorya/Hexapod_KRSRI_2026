#include "HexaServos.h"

HexaServos::HexaServos() {
    _lastUpdate = 0;
    uint16_t center = (SERVO_PULSE_MIN + SERVO_PULSE_MAX) / 2;
    for (int i = 0; i < NUM_SERVOS; i++) _target[i] = center;
}

void HexaServos::begin() {
    // 1. Inisialisasi kedua modul (ini otomatis memanggil .begin() pada bus masing-masing)
    _kit0.begin();
    _kit1.begin();

    // 2. Set frekuensi PWM untuk motor servo (biasanya 50 Hz atau 330 Hz)
    _kit0.setPWMFreq(SERVO_PWM_FREQ);
    _kit1.setPWMFreq(SERVO_PWM_FREQ);

    // 3. Set kecepatan komunikasi I2C agar pengiriman data ke 21 servo tidak lag
    // Ganti SERVO_I2C_BUS yang lama dengan kedua bus ini:
    SERVO_0_I2C_BUS.setClock(SERVO_I2C_CLOCK);
    SERVO_1_I2C_BUS.setClock(SERVO_I2C_CLOCK);
    
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
