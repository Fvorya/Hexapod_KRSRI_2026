#include "Navigation.h"
#include "Calib.h" // Memanggil ini agar bisa membaca GAIT_CYCLE_TIME dari sistem

struct KompasStore { uint8_t m0, m1, ver; float head[4]; uint8_t sum; };

// Konstruktor disambungkan ke Hexapod
Navigation::Navigation(Imu& imuRef, Hexapod& robotRef) : _imu(imuRef), _robot(robotRef) {}

void Navigation::begin() {
    kompasMuat(false);
}

// ====================================================================
// UTILITAS INTERNAL
// ====================================================================

float Navigation::wrap180(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

uint8_t Navigation::kompasSum(const void* buf, size_t n) {
    const uint8_t* p = (const uint8_t*)buf;
    uint8_t acc = 0;
    for (size_t i = 0; i < n; i++) acc = (uint8_t)(acc + p[i] * 31 + 7);
    return acc;
}

// Meneruskan perintah putar langsung ke fungsi walk milik Hexapod
void Navigation::gaitPutar(float turn) {
    // walk(maju, geser, putar)
    _robot.walk(0.0f, 0.0f, turn); 
}

// Sangat Krusial: Selama fungsi delay tunggu(), robot harus terus di-update!
void Navigation::updateSistem() {
    _imu.update();
    _robot.update(); // Menggerakkan gait, Kinematika, dan servo secara mandiri
}

bool Navigation::tunggu(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        updateSistem();
        // Cek interupsi serial atau tombol di sini jika diperlukan
    }
    return true;
}

bool Navigation::tungguYaw(uint32_t ms, float& yawAkum) {
    uint32_t t0 = millis();
    float yawPrev = _imu.yawDeg();
    yawAkum = 0;
    
    while (millis() - t0 < ms) {
        updateSistem();
        float yKini = _imu.yawDeg();
        yawAkum += wrap180(yKini - yawPrev);
        yawPrev = yKini;
    }
    return true;
}

// ====================================================================
// 1. KOMPAS ARENA
// ====================================================================

void Navigation::kompasCatat(uint8_t arah) {
    if (arah > 3) return;
    if (!_imu.hasData()) { Serial.println("Navigation: Tidak ada data sudut IMU."); return; }
    
    _headArah[arah] = _imu.yawDeg();
    Serial.print("Tercatat "); Serial.print(_arahNama[arah]);
    Serial.print(" = "); Serial.print(_headArah[arah], 1); Serial.println(" derajat");
}

void Navigation::kompasSimpan() {
    KompasStore s;
    memset(&s, 0, sizeof(s));
    s.m0 = 0xC0; s.m1 = 0x3A; s.ver = 1;
    for (uint8_t i = 0; i < 4; i++) s.head[i] = _headArah[i];
    s.sum = kompasSum(&s, offsetof(KompasStore, sum));
    EEPROM.put(EE_KOMPAS_ADDR, s);
    Serial.println("Kompas Arena disimpan ke EEPROM 1792.");
}

bool Navigation::kompasMuat(bool cerewet) {
    KompasStore s;
    EEPROM.get(EE_KOMPAS_ADDR, s);
    if (s.m0 != 0xC0 || s.m1 != 0x3A || s.ver != 1 ||
        s.sum != kompasSum(&s, offsetof(KompasStore, sum))) {
        if (cerewet) Serial.println("EEPROM kompas kosong/rusak.");
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) _headArah[i] = s.head[i];
    if (cerewet) Serial.println("4 arah dimuat dari EEPROM.");
    return true;
}

void Navigation::kompasTabel() {
    Serial.println("\n--- KOMPAS ARENA ---");
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(i); Serial.print(" "); Serial.print(_arahNama[i]);
        if (_headArah[i] < 0) { Serial.println("\t: belum dicatat"); continue; }
        Serial.print("\t: "); Serial.print(_headArah[i], 1); Serial.println(" der");
    }
}

// ====================================================================
// 2. PIVOT TERTUTUP (PD Controller)
// ====================================================================

void Navigation::pivotKe(float targetYaw) {
    if (!_imu.hasData()) { Serial.println("Gagal: Tidak ada data IMU."); return; }

    uint32_t t0 = millis(), lapor = 0, masukSejak = 0;
    bool selesai = false;

    while (millis() - t0 < PIVOT_BATAS_MS) {
        updateSistem();
        
        float yawKini = _imu.yawDeg();
        float gzKini = _imu.gyroZ();
        
        float err = wrap180(targetYaw - yawKini);
        
        // Suku D diambil murni dari Gyro Z untuk membuang kebisingan turunan
        float turn = PIVOT_KP * err - PIVOT_KD * gzKini;
        
        if (turn >  1.0f) turn =  1.0f;
        if (turn < -1.0f) turn = -1.0f;
        
        if (fabsf(err) > HEADING_TOLERANCE_DEG && fabsf(turn) < PIVOT_MIN_CMD) {
            turn = (turn >= 0 ? PIVOT_MIN_CMD : -PIVOT_MIN_CMD);
        }

        gaitPutar(_pivotSign * turn);

        // Histeresis masuk ke dalam toleransi target
        if (fabsf(err) <= HEADING_TOLERANCE_DEG) {
            if (!masukSejak) masukSejak = millis();
            if (millis() - masukSejak >= PIVOT_DIAM_MS) { selesai = true; break; }
        } else {
            masukSejak = 0;
        }
    }
    
    gaitPutar(0); // Matikan putaran setelah sampai atau timeout
    tunggu(800);  // Biarkan kaki merespons dan kembali *settle* ke *home*
}

void Navigation::pivotKompas(uint8_t arah) {
    if (arah > 3 || _headArah[arah] < 0) {
        Serial.println("Arah belum dicatat atau tidak valid.");
        return;
    }
    Serial.print("Navigation: Pivot menuju "); Serial.println(_arahNama[arah]);
    pivotKe(_headArah[arah]);
}

void Navigation::pivotRelatif(float der) {
    if (!_imu.hasData()) return;
    float t = _imu.yawDeg() + der;
    while (t >= 360.0f) t -= 360.0f;
    while (t < 0)       t += 360.0f;
    pivotKe(t);
}

// ====================================================================
// 3. KALIBRASI PIVOT
// ====================================================================

void Navigation::kalibrasiPivot(uint8_t siklus) {
    if (!_imu.hasData()) return;

    uint32_t lama = (uint32_t)(siklus * GAIT_CYCLE_TIME);
    float hasil[2], yawAkum;

    for (uint8_t arah = 0; arah < 2; arah++) {
        float cmd = arah ? -1.0f : 1.0f;
        gaitPutar(cmd);
        tunggu(500); // Tunggu *slew* naik mulus
        
        tungguYaw(lama, yawAkum);
        
        gaitPutar(0);
        tungguYaw(1000, yawAkum); // Sisa *settle* pengereman
        
        hasil[arah] = yawAkum;
    }

    _degCCW = hasil[0] / siklus;
    _degCW  = hasil[1] / siklus;
    _pivotSign = (_degCCW >= 0) ? +1 : -1;

    Serial.print("Kalibrasi selesai. Tanda putar: "); Serial.println(_pivotSign);
}
