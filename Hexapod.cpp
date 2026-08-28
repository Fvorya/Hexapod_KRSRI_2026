#include "Hexapod.h"
#include <Arduino.h>

// GerakStore, ServoMap, KompasStore dan eeSum() sekarang tinggal di EEMap.h
// (lihat Hexapod.h). Dulu struct-nya disalin ke sini dan ke Navigation.cpp --
// dua salinan yang bisa menyimpang diam-diam dari program penulisnya.

Hexapod::Hexapod()
    : _armF(&_servos, ARM_PIN_MAP_DEPAN,    NUM_SERVOS),                      // slot 18,19,20
      _armB(&_servos, ARM_PIN_MAP_BELAKANG, NUM_SERVOS + ARM_NUM_SERVOS) {   // slot 21,22,23
    _roll  = _pitch  = _yaw  = 0.0f;
    _rollT = _pitchT = _yawT = 0.0f;
    _trans = _transT = {0, 0, 0};
    _lastPoseT = 0;
}

void Hexapod::begin() {
    // Impor kalibrasi fisik SEBELUM servo dipakai (menimpa default Calib).
    loadServoMap();

    _servos.begin();     // PWM dimatikan di sini -- robot tetap lemas
    _gait.begin();
    _armF.begin();
    _armB.begin();
    profileFlat();

    loadZOff();

    Serial.println("Hexapod: PWM MATI. Ketik 'b' untuk berdiri (topang robot dulu).");
}

// Ambil invert[] & trim[] hasil uji fisik dari EEPROM 1024.
// Ini melengkapi Calib (alamat 0): kalau blob di 1024 sah, ia yang menang,
// karena itulah data yang benar-benar diukur di robot.
void Hexapod::loadServoMap() {
    ServoMap m;
    EEPROM.get(EE_SERVOMAP_ADDR, m);

    // offsetof, bukan sizeof - sizeof(crc): rumus kedua ikut menelan field crc
    // sendiri begitu struct punya padding di ekor. Di ServoMap keduanya
    // kebetulan bernilai sama (124), tapi di CalibBlob tidak -- dan di sana ia
    // membuat blob EEPROM tidak pernah bisa dimuat. Samakan bentuknya supaya
    // kekeliruan itu tidak menular saat struct berubah.
    uint16_t want = Calib::crc16((const uint8_t*)&m, offsetof(ServoMap, crc));
    if (m.magic[0] != 'S' || m.magic[1] != 'M' || m.version != 1 || m.crc != want) {
        Serial.println("Hexapod: ServoMap EEPROM 1024 kosong/rusak -> pakai default Calib.");
        _mapLoaded = false;
        return;
    }

    for (uint8_t i = 0; i < TOTAL_SERVOS && i < EE_SM_SLOTS; i++) {
        gInvert[i] = m.invert[i];
        gTrim[i]   = m.trim[i];
    }
    _mapLoaded = true;
    Serial.println("Hexapod: invert & trim dimuat dari ServoMap EEPROM 1024.");
}

void Hexapod::loadZOff() {
    GerakStore s;
    EEPROM.get(EE_GERAK_ADDR, s);

    // Magic + versi + CHECKSUM. Tanpa cek checksum, EEPROM yang separuh
    // tertulis tetap lolos dan _zOff terisi sampah -> kaki melipat.
    bool ok = (s.m0 == 0x6E && s.m1 == 0x2C && s.ver == 2 &&
               s.sum == eeSum(&s, offsetof(GerakStore, sum)));

    if (!ok) {
        for (uint8_t i = 0; i < 6; i++) _zOff[i] = 0.0f;
        _zOffValid = false;
        Serial.println("Hexapod: Peringatan, kalibrasi kaki EEPROM 2048 tidak valid -> zOff = 0.");
        return;
    }

    for (uint8_t i = 0; i < 6; i++) _zOff[i] = s.zoff[i];
    _zOffValid = true;
    Serial.print("Hexapod: Offset kaki rata dimuat dari EEPROM 2048:");
    for (uint8_t i = 0; i < 6; i++) { Serial.print(' '); Serial.print(_zOff[i], 1); }
    Serial.println(" mm");
}

// Hitung dulu pose berdiri (tanpa mengirim), baru hidupkan PWM. Dengan begitu
// servo langsung menuju pose yang benar, bukan lewat 1500 us dulu.
void Hexapod::arm() {
    if (_servos.isEnabled()) return;
    _gait.setMoveVector(0, 0, 0);
    solvePose();          // isi _target[] dengan pose berdiri saat ini
    _servos.enable();     // stagger 60 ms/servo (pertama kali) atau ramp 400 ms
    Serial.println("Servo AKTIF.");
}

void Hexapod::disarm() {
    _servos.disable();
    _armF.disable();
    _armB.disable();
}

void Hexapod::update() {
    slewBodyPose();      // pose badan merayap ke target SEBELUM dipakai IK
    _gait.update();
    solvePose();
    _servos.commit();
    _armF.commit();
    _armB.commit();
}

void Hexapod::walk(float forward, float strafe, float turn) {
    // gait: vx=strafe, vy=forward, vyaw=turn
    _gait.setMoveVector(strafe, forward, turn);
}

void Hexapod::stop() { _gait.setMoveVector(0, 0, 0); }

// SAAT MENYAMBUNGKAN INI NANTI: jangan menulis _roll/_pitch langsung seperti
// draf di bawah. Sekarang ada _rollT/_pitchT + slewBodyPose(), jadi cukup
// panggil setBodyRotation() -- perataannya sudah ditangani ramp, dan low-pass
// STAB_TAU di draf ini jadi peredam kedua yang tidak perlu. Yang masih relevan
// dari draf ini hanya deadband dan clamp STAB_MAX_DEG.
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

// ---------------------------------------------------------------- pose badan
// setBody*() hanya MENETAPKAN SASARAN. Yang menggerakkan adalah slewBodyPose()
// di awal update(), dengan laju terbatas.
//
// Kenapa perlu: pose badan diterapkan SESUDAH gait, jadi ia melewati kedua
// peredam yang sudah ada -- GAIT_SLEW_RATE (vektor gerak) dan GAIT_PROFILE_TAU
// (profil medan). Diukur di simulasi: 'r20 0 0' tanpa ramp mengubah femur
// ~49 der dalam SATU commit 20 ms, setara ~2400 der/detik. Servo tidak bisa
// menurutinya; yang terjadi adalah hentakan arus dan robot melonjak.
void Hexapod::setBodyTranslation(float x, float y, float z) {
    _transT = { clampf(x, -BODY_MAX_TRANS_MM, BODY_MAX_TRANS_MM),
                clampf(y, -BODY_MAX_TRANS_MM, BODY_MAX_TRANS_MM),
                clampf(z, -BODY_MAX_TRANS_MM, BODY_MAX_TRANS_MM) };
}

// roll  + = miring KANAN  (sumbu depan +Y)
// pitch + = MENDONGAK     (sumbu kanan +X)
// yaw   + = belok KIRI    (sumbu atas  +Z)
void Hexapod::setBodyRotation(float rollDeg, float pitchDeg, float yawDeg) {
    _rollT  = deg2rad(clampf(rollDeg,  -BODY_MAX_ROT_DEG, BODY_MAX_ROT_DEG));
    _pitchT = deg2rad(clampf(pitchDeg, -BODY_MAX_ROT_DEG, BODY_MAX_ROT_DEG));
    _yawT   = deg2rad(clampf(yawDeg,   -BODY_MAX_ROT_DEG, BODY_MAX_ROT_DEG));
}

// Ramp berlaju tetap (bukan low-pass): waktu tempuhnya bisa diprediksi dan
// tidak pernah ada ekor panjang yang membuat pose "hampir sampai" selamanya.
// Pola sama dengan slew() di HexaGait.
static float rayap(float cur, float tgt, float langkah) {
    if (tgt > cur) { cur += langkah; return (cur > tgt) ? tgt : cur; }
    if (tgt < cur) { cur -= langkah; return (cur < tgt) ? tgt : cur; }
    return tgt;
}

void Hexapod::slewBodyPose() {
    uint32_t now = millis();
    // dt dijepit seperti di HexaGait: jeda besar (mis. sesudah blocking
    // kalibrasi) tidak boleh berubah jadi satu lompatan besar.
    float dt = _lastPoseT ? (now - _lastPoseT) / 1000.0f : 0.02f;
    _lastPoseT = now;
    dt = clampf(dt, 0.0f, 0.05f);

    const float dRot = deg2rad(BODY_SLEW_DEG_S) * dt;
    const float dMm  = BODY_SLEW_MM_S * dt;

    _roll    = rayap(_roll,    _rollT,    dRot);
    _pitch   = rayap(_pitch,   _pitchT,   dRot);
    _yaw     = rayap(_yaw,     _yawT,     dRot);
    _trans.x = rayap(_trans.x, _transT.x, dMm);
    _trans.y = rayap(_trans.y, _transT.y, dMm);
    _trans.z = rayap(_trans.z, _transT.z, dMm);
}

bool Hexapod::bodyPoseSampai() const {
    const float eR = deg2rad(0.05f), eM = 0.05f;
    return fabsf(_roll  - _rollT)  < eR && fabsf(_pitch - _pitchT) < eR &&
           fabsf(_yaw   - _yawT)   < eR &&
           fabsf(_trans.x - _transT.x) < eM && fabsf(_trans.y - _transT.y) < eM &&
           fabsf(_trans.z - _transT.z) < eM;
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

    // Sudut di luar 0..180 berarti servo TIDAK bisa menuruti perintah.
    // Dulu ini di-clamp diam-diam: IK melaporkan "dalam jangkauan" (karena
    // hanya memeriksa jarak D), padahal coxa diminta memutar 120 der dan
    // servo mentok di 90. Sekarang ikut ditandai.
    if (servoAngle < 0.0f || servoAngle > 180.0f) _servoClamped = true;
    servoAngle = clampf(servoAngle, 0.0f, 180.0f);

    int pulse = SERVO_PULSE_MIN +
        (int)((servoAngle / 180.0f) * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    pulse += SERVO_TRIM_US[id];
    if (pulse < SERVO_PULSE_MIN || pulse > SERVO_PULSE_MAX) _servoClamped = true;

    return (uint16_t)constrain(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

// Lengan = BAHU, SIKU, GRIP (2 DOF planar + penjepit), dipasang DEPAN dan
// BELAKANG badan. Keduanya menyapu bidang VERTIKAL yang menghadap keluar
// dari badan; tidak ada sendi pemutar di pangkal, jadi untuk membidik objek
// yang tidak segaris, BADAN robot yang harus diarahkan. Karena itu IK 2 link
// di ArmInverse memang bentuk yang tepat.
//
// Slot kalibrasi: depan 18=bahu 19=siku 20=grip, belakang 21/22/23.
// (SLOT_NAME[] di legacy TES_GERAK/servo_map.h masih menulis "BASE" untuk
//  slot pertama dan menamai lengan "R"/"L" -- label itu USANG.)
//
// jangkauan/tinggi diukur dari PUSAT BADAN; offset pangkal bahu dikurangkan
// di sini. Versi lama mengurangkan ARM_ORIGINS[0][0] -- offset ke SAMPING --
// dari koordinat jangkauan, dua sumbu yang berbeda.
bool Hexapod::moveArmTarget(uint8_t arm, float jangkauan, float tinggi) {
    if (arm > 1) return false;
    HexaArm* a = (arm == ARM_DEPAN) ? &_armF : &_armB;

    // fabsf: lengan belakang punya origin -Y, tapi "jangkauan" untuk kedua
    // lengan sama-sama positif ke arah hadapnya masing-masing.
    float lokalX = jangkauan - fabsf(ARM_ORIGINS[arm][1]);
    float lokalY = tinggi    - ARM_ORIGINS[arm][2];

    float bahuDeg = 0.0f, sikuDeg = 0.0f;
    if (!ArmInverse::solve(lokalX, lokalY, bahuDeg, sikuDeg)) return false;

    a->setArmPulse(0, a->angleToPulse(0, bahuDeg, 90.0f));
    a->setArmPulse(1, a->angleToPulse(1, sikuDeg, 90.0f));
    return true;
}

// Penjepit: 0 = menutup penuh, 100 = membuka penuh. Sudut servo 0..180 der
// lewat jalur kalibrasi yang sama (offset/trim/invert slot grip berlaku).
bool Hexapod::setGrip(uint8_t arm, float persen) {
    if (arm > 1) return false;
    HexaArm* a = (arm == ARM_DEPAN) ? &_armF : &_armB;
    persen = clampf(persen, 0.0f, 100.0f);
    float geo = persen / 100.0f * 180.0f - 90.0f;   // baseline 90 der
    a->setArmPulse(2, a->angleToPulse(2, geo, 90.0f));
    return true;
}

void Hexapod::armEnable(uint8_t arm, bool on) {
    HexaArm* a = (arm == ARM_DEPAN) ? &_armF : &_armB;
    if (arm > 1) return;
    if (on) a->enable(); else a->disable();
}

bool Hexapod::armEnabled(uint8_t arm) {
    if (arm > 1) return false;
    return (arm == ARM_DEPAN) ? _armF.isEnabled() : _armB.isEnabled();
}

// Cetak seluruh rantai perhitungan per kaki -- untuk membedakan masalah
// kalibrasi, geometri, dan arah servo tanpa menebak.
void Hexapod::debugDump() {
    Serial.println("\n================ DEBUG HEXAPOD ================");
    Serial.print("PWM        : "); Serial.println(_servos.isEnabled() ? "AKTIF" : "MATI");
    Serial.print("ServoMap   : "); Serial.println(_mapLoaded ? "EEPROM 1024" : "default Calib");
    Serial.print("zOff       : "); Serial.println(_zOffValid ? "EEPROM 2048" : "TIDAK VALID (0)");
    Serial.print("Lengan     : depan "); Serial.print(_armF.isEnabled() ? "aktif" : "mati");
    Serial.print(", belakang ");        Serial.println(_armB.isEnabled() ? "aktif" : "mati");
    Serial.print("Kaki (mm)  : coxa "); Serial.print(COXA_LENGTH, 1);
    Serial.print("  femur ");          Serial.print(FEMUR_LENGTH, 1);
    Serial.print("  tibia ");          Serial.println(TIBIA_LENGTH, 1);
    Serial.print("Pulse (us) : ");     Serial.print(SERVO_PULSE_MIN);
    Serial.print(" .. ");              Serial.println(SERVO_PULSE_MAX);

    GaitProfile p = _gait.profile();
    Serial.print("Profil     : standH "); Serial.print(p.standHeight, 1);
    Serial.print("  standR ");            Serial.print(p.standRadius, 1);
    Serial.print("  stepL ");             Serial.print(p.stepLength, 1);
    Serial.print("  cycle ");             Serial.println(p.cycleTime, 0);

    // --- body kinematics ---
    Serial.print("Pose badan : roll "); Serial.print(bodyRollDeg(), 2);
    Serial.print("  pitch ");           Serial.print(bodyPitchDeg(), 2);
    Serial.print("  yaw ");             Serial.print(bodyYawDeg(), 2);
    Serial.println(" der");
    Vec3 tr = bodyTranslation();
    Serial.print("Geser badan: x "); Serial.print(tr.x, 1);
    Serial.print("  y ");            Serial.print(tr.y, 1);
    Serial.print("  z ");            Serial.print(tr.z, 1);
    Serial.println(" mm");
    Serial.println("             (roll+ = miring KANAN, pitch+ = MENDONGAK, yaw+ = belok KIRI)");
    Serial.print("Jangkauan  : ");
    Serial.println(lastPoseInRange() ? "semua kaki OK" : "!! ADA YANG DI-CLAMP !!");

    Serial.println("\nkaki  zOff |    lx     ly     lz |   coxa   femur   tibia | inv c/f/t | sdeg c/f/t | pulse c/f/t | rng");
    for (int leg = 0; leg < 6; leg++) {
        float lx, ly, lz, coxa, femur, tibia;
        bool ok = legSolve(leg, lx, ly, lz, coxa, femur, tibia);

        uint8_t c = leg * 3 + 0, f = leg * 3 + 1, t = leg * 3 + 2;
        Serial.printf(" %d  %+6.1f | %6.1f %6.1f %6.1f | %6.2f %7.2f %7.2f | %d %d %d | %5.1f %5.1f %5.1f | %4u %4u %4u | %s\n",
            leg, _zOff[leg], lx, ly, lz, coxa, femur, tibia,
            SERVO_INVERT[c], SERVO_INVERT[f], SERVO_INVERT[t],
            (double)(SERVO_INVERT[c] ? 90.0f - coxa  : 90.0f + coxa),
            (double)(SERVO_INVERT[f] ? 90.0f - femur : 90.0f + femur),
            (double)(SERVO_INVERT[t] ? 180.0f - tibia : tibia),
            _servos.targetPulse(c), _servos.targetPulse(f), _servos.targetPulse(t),
            ok ? "ok" : "CLAMP");
    }
    Serial.println("===============================================\n");
}

// Satu kaki: titik gait -> transform badan -> frame kaki -> IK.
// Dipakai bersama oleh solvePose() dan debugDump() supaya angka yang dicetak
// dijamin sama dengan yang benar-benar dikirim ke servo.
bool Hexapod::legSolve(int leg, float& lx, float& ly, float& lz,
                       float& coxa, float& femur, float& tibia) {
    Vec3 foot = _gait.legTargets[leg];

    Vec3 p = {
        foot.x - _trans.x,
        foot.y - _trans.y,
        (foot.z + _zOff[leg]) - _trans.z
    };
    Vec3 pb = rotatePointInv(p, _roll, _pitch, _yaw);

    float vx = pb.x - BODY_LEG_ORIGINS[leg][0];
    float vy = pb.y - BODY_LEG_ORIGINS[leg][1];
    float vz = pb.z - BODY_LEG_ORIGINS[leg][2];

    float a = -deg2rad(BODY_LEG_ANGLE[leg]);
    lx = cosf(a) * vx - sinf(a) * vy;
    ly = sinf(a) * vx + cosf(a) * vy;
    lz = vz;

    return LegInverseKinematics::solve(lx, ly, lz, coxa, femur, tibia);
}

bool Hexapod::legAngles(int leg, float& coxaDeg, float& femurDeg, float& tibiaDeg) {
    if (leg < 0 || leg > 5) return false;
    float lx, ly, lz;
    return legSolve(leg, lx, ly, lz, coxaDeg, femurDeg, tibiaDeg);
}

void Hexapod::solvePose() {
    bool clamped = false;
    _servoClamped = false;          // diisi ulang oleh angleToPulse() di bawah
    for (int leg = 0; leg < 6; leg++) {
        float lx, ly, lz, coxa, femur, tibia;

        // 1-4) Body kinematics -> frame kaki -> IK (lihat legSolve()).
        if (!legSolve(leg, lx, ly, lz, coxa, femur, tibia)) clamped = true;

        // 5) Konversi ke pulse servo. Baseline 90 der; invert & trim per slot
        //    diambil dari ServoMap EEPROM 1024 bila ada.
        uint8_t c = leg * 3 + 0, f = leg * 3 + 1, t = leg * 3 + 2;
        _servos.setLegPulse(c, angleToPulse(c, coxa,          90.0f));
        _servos.setLegPulse(f, angleToPulse(f, femur,         90.0f));
        _servos.setLegPulse(t, angleToPulse(t, tibia - 90.0f, 90.0f));
    }
    // "tidak dalam jangkauan" = IK mentok ATAU servo mentok. Keduanya sama
    // artinya bagi pengguna: robot tidak menuruti pose yang diminta.
    _ikClamped = clamped || _servoClamped;
}
