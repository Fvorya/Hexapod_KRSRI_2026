#include "HexaArm.h"

// URUTAN GERAK SENDI. Satu tahap bergerak sampai selesai sebelum tahap
// berikutnya boleh mulai: BAHU dulu, lalu SIKU+PERGELANGAN. Grip ikut tahap
// bahu -- ia tidak mengubah letak capit, jadi menjalankannya bersamaan tidak
// menambah jalur yang harus dihindari.
//
// Sebabnya bukan kerapian. Semua sendi berangkat serentak berarti capit
// menempuh lengkung yang tidak pernah dihitung siapa pun: IK cuma menjamin
// titik AKHIR-nya, bukan jalan yang dilewati. Menahan bahu sampai selesai
// membuat jalur itu bisa ditebak dari sudut yang sedang berjalan.
//
// SIKU DAN PERGELANGAN SATU TAHAP, bukan dua. Dari arena: keduanya terbaca
// sebagai satu gerakan, jadi memisahnya cuma menambah jeda tanpa menambah
// kendali.
//
// Harganya waktu: total = jumlah waktu tiap TAHAP, bukan yang terlama. Pada
// ARM_SLEW_DEG_S 150 der/detik, pose 90 der di ketiga sendi makan 2 x 0,6 =
// 1,2 detik, bukan 0,6. Sekuens korban di Misi.cpp memakai jeda TETAP, dan
// arah perubahan ini AMAN terhadapnya: dulu 3 tahap = 1,8 detik, jadi margin
// di dalam LENGAN_JEDA_MS 2100 naik dari 300 ms ke 900 ms. Menggabungkan
// tahap tidak pernah memperlambat.
//
// Harga yang kedua: pergelangan tidak lagi menahan sudutnya selagi siku
// mengayun, jadi sudut hadap capit BERUBAH sepanjang jalan. Titik akhirnya
// tetap dijamin IK. Yang bisa gigit cuma halangan di TENGAH jalan -- kalau
// capit menyenggol sesuatu saat turun ke korban, pisahkan lagi tahap ini
// sebelum mencari sebab lain.
//
// Indeks sendi (ARM_PIN_MAP_DEPAN): 0 bahu, 1 siku, 2 pergelangan, 3 grip.
// Lengan BELAKANG cuma punya id 0 (grip), dan id 0 ada di tahap pertama, jadi
// ia tidak pernah menunggu giliran.
static const uint8_t TAHAP_MASK[] = {
    (1 << 0) | (1 << 3),   // bahu + grip
    (1 << 1) | (1 << 2)    // siku + pergelangan
};
static const uint8_t N_TAHAP = sizeof(TAHAP_MASK) / sizeof(TAHAP_MASK[0]);

HexaArm::HexaArm(HexaServos* servosDriver, const uint8_t pinMap[][2],
                 uint8_t n, uint8_t calibBase) {
    _driver    = servosDriver;
    _pinMap    = pinMap;
    _calibBase = calibBase;
    _n         = (n > ARM_NUM_SERVOS) ? ARM_NUM_SERVOS : n;
    _enabled   = false;      // lengan belum terpasang -> jangan digerakkan
    _lastUpdate = 0;
    // Konstruktor jalan sebelum Calib::load(), jadi jangan hitung center di sini.
    for (int i = 0; i < ARM_NUM_SERVOS; i++) _target[i] = _kini[i] = 0;  // seluruh array
}

void HexaArm::begin() {
    // Sama seperti kaki: isi target, TAPI jangan kirim apa pun. PWM lengan
    // tetap mati sampai enable() dipanggil.
    uint16_t centerPulse = (SERVO_ARM_PULSE_MIN + SERVO_ARM_PULSE_MAX) / 2;
    for (int i = 0; i < _n; i++) _target[i] = _kini[i] = centerPulse;
    _lastUpdate = millis();
    _tahap = 0;
}

void HexaArm::setArmPulse(uint8_t id, uint16_t pulseUs) {
    // _n, bukan ARM_NUM_SERVOS: lengan belakang cuma punya id 0. Tanpa ini,
    // menulis id 1..3 padanya akan membaca _pinMap di luar batas dan menembak
    // kanal PCA yang dipegang KAKI.
    if (id >= _n) return;
    const uint16_t baru = (uint16_t)constrain((int)pulseUs,
                                              SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
    // Sasaran yang BERUBAH memulai urutan dari tahap pertama lagi. Perintah
    // seperti 'as' menulis ketiga sendi berturut-turut dalam satu baris, jadi
    // penyetelan ulang ini terjadi sebelum commit() berikutnya sempat jalan --
    // satu urutan untuk satu perintah, bukan satu per sendi.
    if (baru != _target[id]) _tahap = 0;
    _target[id] = baru;
}

// Semua sendi milik tahap ini sudah sampai sasaran? Sendi yang tidak ada pada
// lengan ini (_n lebih kecil dari bit maskenya) dihitung SUDAH sampai, kalau
// tidak lengan belakang akan menunggu siku yang tidak pernah ada.
bool HexaArm::tahapSampai(uint8_t tahap) const {
    const uint8_t mask = TAHAP_MASK[tahap];
    for (uint8_t i = 0; i < _n; i++)
        if ((mask & (1 << i)) && _kini[i] != _target[i]) return false;
    return true;
}

float HexaArm::sudutServo(uint8_t id, float geoAngleDeg, float baseline) const {
    const uint8_t k = _calibBase + id;
    const float s = SERVO_INVERT[k] ? -geoAngleDeg : geoAngleDeg;
    return baseline + SERVO_OFFSET[k] + s;
}

uint16_t HexaArm::angleToPulse(uint8_t id, float geoAngleDeg, float baseline) {
    // 1. Index slot kalibrasi milik LENGAN INI (depan 18-21, belakang 22)
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
    unsigned long now = millis();
    unsigned long dt  = now - _lastUpdate;
    if (dt < 20) return false;
    _lastUpdate = now;

    // SLEW. Perintah lengan menetapkan _target; di sinilah _kini merayap
    // menujunya, dan HANYA _kini yang pernah sampai ke servo.
    //
    // Laju dinyatakan derajat/detik dan diubah ke mikrodetik di sini, bukan
    // ditulis langsung dalam us: rentang pulsa lengan itu parameter Calib yang
    // bisa berubah ('Qarm.pulse.max'), dan laju yang ditulis dalam us akan
    // diam-diam berganti arti setiap kali rentangnya disetel.
    const float usPerDer = (float)(SERVO_ARM_PULSE_MAX - SERVO_ARM_PULSE_MIN) / 180.0f;
    const float dtS      = (float)dt / 1000.0f;

    // Maju ke tahap berikutnya selama sendi tahap ini sudah sampai. Memakai
    // while, bukan if: tahap yang sasarannya memang tidak berubah harus
    // dilewati dalam siklus yang sama, kalau tidak tiap perintah membayar
    // 20 ms per sendi yang diam.
    while (_tahap + 1 < N_TAHAP && tahapSampai(_tahap)) _tahap++;

    const uint8_t mask = TAHAP_MASK[_tahap];

    for (int i = 0; i < _n; i++) {
        // Sendi di luar tahap ini TIDAK merayap -- tapi tetap ditulis, supaya
        // servonya terus ditahan di tempat, bukan dilepas.
        if (mask & (1 << i)) {
            int d = (int)_target[i] - (int)_kini[i];
            const float sisaDer = fabsf((float)d) / usPerDer;

            // NAIK dibatasi percepatan; TURUN dibatasi sisa jarak. Rem memakai
            // v = sqrt(2 a s), yaitu laju tercepat yang masih bisa berhenti
            // TEPAT di sasaran dengan percepatan yang sama. Keduanya batas
            // ATAS, jadi gerakan pendek tidak pernah sampai laju penuh dan
            // melandai sendiri tanpa cabang khusus.
            _laju[i] += ARM_ACCEL_DEG_S2 * dtS;
            if (_laju[i] > _slewDegS) _laju[i] = _slewDegS;
            const float vRem = sqrtf(2.0f * ARM_ACCEL_DEG_S2 * sisaDer);
            if (_laju[i] > vRem) _laju[i] = vRem;

            int langkah = (int)(_laju[i] * usPerDer * dtS);
            // Lantai 1 us WAJIB. Rem membuat langkah menuju nol di ujung, dan
            // tahapSampai() menuntut _kini == _target PERSIS -- tanpa lantai
            // ini sendi berhenti satu mikrodetik sebelum sasaran dan tahap
            // berikutnya tidak pernah mendapat giliran.
            if (langkah < 1) langkah = 1;
            if (d >  langkah) d =  langkah;
            if (d < -langkah) d = -langkah;
            _kini[i] = (uint16_t)((int)_kini[i] + d);
        } else {
            _laju[i] = 0.0f;     // tahap berikutnya berangkat dari diam
        }
        _driver->writeRaw(_pinMap[i][0], _pinMap[i][1], _kini[i]);
    }
    return true;
}
