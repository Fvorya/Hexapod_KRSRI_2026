#include "HexaServos.h"

HexaServos::HexaServos() {
    _lastUpdate = 0;
    _enabled = false;
    _driven  = false;
    // Catatan: SERVO_PULSE_MIN/MAX di sini masih 0 karena konstruktor objek
    // global jalan SEBELUM Calib::load(). Nilainya diisi ulang di begin().
    for (int i = 0; i < NUM_SERVOS; i++) { _target[i] = 0; _last[i] = 0; }
}

void HexaServos::begin() {
    // 1. Inisialisasi kedua modul (ini otomatis memanggil .begin() pada bus masing-masing)
    _kit0.begin();
    _kit1.begin();

    // 2. Set frekuensi PWM untuk motor servo (biasanya 50 Hz atau 330 Hz)
    _kit0.setPWMFreq(SERVO_PWM_FREQ);
    _kit1.setPWMFreq(SERVO_PWM_FREQ);

    // 3. Set kecepatan komunikasi I2C agar pengiriman data ke 24 servo tidak lag
    SERVO_0_I2C_BUS.setClock(SERVO_I2C_CLOCK);
    SERVO_1_I2C_BUS.setClock(SERVO_I2C_CLOCK);

    // 4. PWM MATI. Isi _target dengan posisi tengah sebagai nilai awal yang
    //    masuk akal, tapi JANGAN dikirim -- robot harus tetap lemas sampai
    //    ada perintah eksplisit. Ini yang membuat upload aman tanpa mencabut
    //    Teensy: sesudah reset otomatis, servo tidak bergerak sama sekali.
    uint16_t center = (SERVO_PULSE_MIN + SERVO_PULSE_MAX) / 2;
    for (int i = 0; i < NUM_SERVOS; i++) { _target[i] = center; _last[i] = center; }

    allOff();
    _lastUpdate = millis();
}

void HexaServos::allOff() {
    // setPWM(ch, 0, 0) = output benar-benar mati (bukan pulse 0 us).
    for (uint8_t c = 0; c < 16; c++) { _kit0.setPWM(c, 0, 0); _kit1.setPWM(c, 0, 0); }
    _enabled = false;
    _driven  = false;
}

void HexaServos::disable() {
    allOff();
    Serial.println("Servo NONAKTIF (PWM mati, servo bebas).");
}

void HexaServos::enable() {
    if (_enabled) return;
    _enabled = true;

    if (_driven) rampTo();     // posisi terakhir diketahui -> ramp mulus
    else         staggerTo();  // baru bangun -> satu per satu

    _lastUpdate = millis();
}

// Posisi fisik servo belum diketahui (baru dinyalakan). Kirim satu per satu
// supaya arus start 18 servo tidak menumpuk jadi satu lonjakan.
void HexaServos::staggerTo() {
    Serial.println("Mengaktifkan servo satu per satu -- TOPANG ROBOT.");
    for (int i = 0; i < NUM_SERVOS; i++) {
        writeRaw(SERVO_PIN_MAP[i][0], SERVO_PIN_MAP[i][1], _target[i]);
        _last[i] = _target[i];
        delay(SERVO_STAGGER_MS);
    }
    _driven = true;
}

// Posisi terakhir diketahui -> interpolasi supaya tidak menyentak.
void HexaServos::rampTo() {
    uint16_t from[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; i++) from[i] = _last[i];

    const uint16_t n = SERVO_RAMP_MS / 20;
    for (uint16_t k = 1; k <= n; k++) {
        float a = (float)k / (float)n;
        for (int i = 0; i < NUM_SERVOS; i++) {
            uint16_t p = (uint16_t)(from[i] + (int)((float)((int)_target[i] - (int)from[i]) * a));
            writeRaw(SERVO_PIN_MAP[i][0], SERVO_PIN_MAP[i][1], p);
            _last[i] = p;
        }
        delay(20);
    }
    _driven = true;
}

void HexaServos::setLegPulse(uint8_t id, uint16_t pulseUs) {
    if (id >= NUM_SERVOS) return;
    _target[id] = constrain((int)pulseUs, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

uint16_t HexaServos::targetPulse(uint8_t id) const {
    return (id < NUM_SERVOS) ? _target[id] : 0;
}

bool HexaServos::commit() {
    if (!_enabled) return false;                       // <-- gerbang keselamatan
    if (millis() - _lastUpdate < UPDATE_MS) return false;
    _lastUpdate = millis();
    for (int i = 0; i < NUM_SERVOS; i++) {
        writeRaw(SERVO_PIN_MAP[i][0], SERVO_PIN_MAP[i][1], _target[i]);
        _last[i] = _target[i];
    }
    return true;
}

void HexaServos::writeRaw(uint8_t driver, uint8_t channel, uint16_t pulseUs) {
    if (driver == 0) _kit0.writeMicroseconds(channel, pulseUs);
    else             _kit1.writeMicroseconds(channel, pulseUs);
}
