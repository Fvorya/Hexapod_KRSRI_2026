#include "Hexapod.h"
#include <Arduino.h>

struct GerakStore {
    uint8_t m0, m1, ver;
    float   ccw, cw, maju;
    int8_t  sign;
    float   lvlR, lvlP;        
    float   refR, refP;        
    float   jac[4];            
    float   zoff[6];  // Ini yang kita butuhkan!
    uint8_t sum;
};

Hexapod::Hexapod() : _armR(&_servos, ARM_PIN_MAP_R), _armL(&_servos, ARM_PIN_MAP_L) {
    _roll = _pitch = _yaw = 0.0f;
    _trans = {0, 0, 0};
    _lastStabT = 0;
}

void Hexapod::begin() {
    _servos.begin();
    _gait.begin();
    _armR.begin();
    _armL.begin();
    profileFlat();

    GerakStore s;
    EEPROM.get(2048, s); 
    
    // Cek apakah data valid (magic number 0x6E 0x2C dan versi 2)
    if (s.m0 == 0x6E && s.m1 == 0x2C && s.ver == 2) {
        for (uint8_t i = 0; i < 6; i++) {
            _zOff[i] = s.zoff[i]; // Salin offset telapak ke sistem
        }
        Serial.println("Hexapod: Offset kaki rata dimuat dari EEPROM 2048.");
    } else {
        Serial.println("Hexapod: Peringatan, kalibrasi kaki tidak valid/belum ada.");
    }
}

void Hexapod::update() {
    _gait.update();
    solvePose();
    _servos.commit();
    _armR.commit();
    _armL.commit();
}

void Hexapod::walk(float forward, float strafe, float turn) {
    // gait: vx=strafe, vy=forward, vyaw=turn
    _gait.setMoveVector(strafe, forward, turn);
}

void Hexapod::stop() { _gait.setMoveVector(0, 0, 0); }

// void Hexapod::setStabilization(float rollDeg, float pitchDeg) {
//     // deadband
//     if (fabsf(rollDeg)  < STAB_DEADBAND_DEG) rollDeg  = 0;
//     if (fabsf(pitchDeg) < STAB_DEADBAND_DEG) pitchDeg = 0;
//     // clamp
//     rollDeg  = clampf(rollDeg,  -STAB_MAX_DEG, STAB_MAX_DEG);
//     pitchDeg = clampf(pitchDeg, -STAB_MAX_DEG, STAB_MAX_DEG);
//     // low-pass berbasis dt (konstan tau -> kehalusan tak tergantung kecepatan loop)
//     uint32_t now = millis();
//     float dt = _lastStabT ? (now - _lastStabT) / 1000.0f : 0.02f;
//     _lastStabT = now;
//     dt = clampf(dt, 0.0f, 0.05f);
//     float a = dt / (STAB_TAU + dt);
//     _roll  = lerpf(_roll,  deg2rad(rollDeg),  a);
//     _pitch = lerpf(_pitch, deg2rad(pitchDeg), a);
// }

void Hexapod::setBodyTranslation(float x, float y, float z) { _trans = {x, y, z}; }

void Hexapod::setBodyRotation(float rollDeg, float pitchDeg, float yawDeg) {
    _roll  = deg2rad(rollDeg);
    _pitch = deg2rad(pitchDeg);
    _yaw   = deg2rad(yawDeg);
}

void Hexapod::jog(uint8_t tuneId, uint16_t pulseUs) {
    if (tuneId >= NUM_TUNE_SERVOS) return;
    _servos.writeRaw(TUNE_PIN_MAP[tuneId][0], TUNE_PIN_MAP[tuneId][1], pulseUs);
}

void Hexapod::profileFlat() {
    // 0 DATAR: { 40, 60, 900, 100, 70 } -> Sesuai konstanta dasar
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, STAND_HEIGHT, STAND_RADIUS });
}

void Hexapod::profileStairs() {
    // 1 TANGGA: { 75, 70, 1800, 110, 70 }
    // Perubahan: Tinggi(+35), Langkah(+10), Siklus(+900), Tinggi Badan(+10)
    _gait.setProfile({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 10.0f,
                       GAIT_CYCLE_TIME + 900.0f, STAND_HEIGHT + 10.0f, STAND_RADIUS });
}

void Hexapod::profileCrouch() {
    // 2 MERUNDUK: { 40, 55, 1100, 80, 70 }
    // Perubahan: Langkah(-5), Siklus(+200), Tinggi Badan(-20)
    _gait.setProfile({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH - 5.0f, 
                       GAIT_CYCLE_TIME + 200.0f, STAND_HEIGHT - 20.0f, STAND_RADIUS });
}

void Hexapod::profileNarrow() {
    // 3 SEMPIT: { 30, 45, 1000, 100, 45 }
    // Perubahan: Tinggi(-10), Langkah(-15), Siklus(+100), Lebar Kaki(-25)
    _gait.setProfile({ GAIT_STEP_HEIGHT - 10.0f, GAIT_STEP_LENGTH - 15.0f, 
                       GAIT_CYCLE_TIME + 100.0f, STAND_HEIGHT, STAND_RADIUS - 25.0f });
}

// geoAngle (derajat) -> pulse, dengan kalibrasi per-servo.
// servoAngle = baseline + offset + (invert? -geo : geo); lalu map ke pulse + trim.
uint16_t Hexapod::angleToPulse(uint8_t id, float geoAngleDeg, float baseline) {
    float s = SERVO_INVERT[id] ? -geoAngleDeg : geoAngleDeg;
    float servoAngle = baseline + SERVO_OFFSET[id] + s;
    servoAngle = clampf(servoAngle, 0.0f, 180.0f);
    int pulse = SERVO_PULSE_MIN +
        (int)((servoAngle / 180.0f) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    pulse += SERVO_TRIM_US[id];
    return (uint16_t)constrain(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

void Hexapod::moveArmTarget(float x, float y) {
    float shoulderDeg = 0.0f;
    float elbowDeg = 0.0f;

    // Kurangi target koordinat dengan posisi fisik pangkal bahu dari pusat robot
    float armLocalX = x - ARM_ORIGINS[0][0]; 
    float armLocalY = y - ARM_ORIGINS[0][2]; // Menggunakan indeks [2] jika Y di IK lengan mewakili tinggi (Z di robot)

    // Masukkan koordinat lokal yang sudah dikoreksi ke rumus IK Lengan
    if (ArmInverse::solve(armLocalX, armLocalY, shoulderDeg, elbowDeg)) {
        
        uint16_t pulseShoulder = _armR.angleToPulse(0, shoulderDeg, 90.0f);
        uint16_t pulseElbow    = _armR.angleToPulse(1, elbowDeg, 90.0f);

        _armR.setArmPulse(0, pulseShoulder);
        _armR.setArmPulse(1, pulseElbow);
    }
}

void Hexapod::solvePose() {
    for (int leg = 0; leg < 6; leg++) {
        Vec3 foot = _gait.legTargets[leg];

        // =================================================================
        // 1) Body Kinematics (Metode Aljabar Langsung - Super Cepat)
        // =================================================================
        Vec3 p = { 
            foot.x - _trans.x, 
            foot.y - _trans.y, 
            (foot.z + _zOff[leg]) - _trans.z  // <--- Z Offset dimasukkan di sini
        };
        
        Vec3 pb = rotatePointInv(p, _roll, _pitch, _yaw);

        // =================================================================
        // 2) Relatif pangkal coxa
        // =================================================================
        float vx = pb.x - BODY_LEG_ORIGINS[leg][0];
        float vy = pb.y - BODY_LEG_ORIGINS[leg][1];
        float vz = pb.z - BODY_LEG_ORIGINS[leg][2];

        // 3) Rotasi ke frame kaki (neutral menghadap +X)
        float a = -deg2rad(BODY_LEG_ANGLE[leg]);
        float lx = cosf(a) * vx - sinf(a) * vy;
        float ly = sinf(a) * vx + cosf(a) * vy;
        float lz = vz;

        // 4) Inverse Kinematics Kaki
        float coxa, femur, tibia;
        LegInverseKinematics::solve(lx, ly, lz, coxa, femur, tibia);

        // 5) Konversi ke pulse servo
        uint8_t c = leg * 3 + 0, f = leg * 3 + 1, t = leg * 3 + 2;
        _servos.setLegPulse(c, angleToPulse(c, coxa,          90.0f));
        _servos.setLegPulse(f, angleToPulse(f, femur,         90.0f));
        _servos.setLegPulse(t, angleToPulse(t, tibia - 90.0f, 90.0f));
    }
}
