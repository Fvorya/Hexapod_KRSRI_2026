#include "Hexapod.h"
#include <Arduino.h>

// GerakStore, ServoMap, KompasStore dan eeSum() sekarang tinggal di EEMap.h
// (lihat Hexapod.h). Dulu struct-nya disalin ke sini dan ke Navigation.cpp --
// dua salinan yang bisa menyimpang diam-diam dari program penulisnya.

Hexapod::Hexapod()
    : _armF(&_servos, ARM_PIN_MAP_DEPAN,    ARM_N_DEPAN,    NUM_SERVOS),                 // slot 18,19,20,21
      _armB(&_servos, ARM_PIN_MAP_BELAKANG, ARM_N_BELAKANG, NUM_SERVOS + ARM_N_DEPAN) { // slot 22
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
    muatProfil();
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

// --- TRIM SERVO ------------------------------------------------------------
//
// Ditulis ke blob yang SAMA dengan yang dibaca loadServoMap(), alamat 1024.
// Bukan ke CalibBlob alamat 0: blob itu juga punya trim[] dan invert[], tapi
// keduanya MATI -- loadServoMap() menimpanya tiap boot, jadi 'W' yang menyimpan
// trim ke alamat 0 tidak pernah berpengaruh. Menulis ke dua tempat yang salah
// satunya diabaikan adalah cara membuat dua angka menyimpang diam-diam.

void Hexapod::setTrim(uint8_t slot, int16_t us) {
    if (slot >= TOTAL_SERVOS || slot >= EE_SM_SLOTS) {
        Serial.print("Trim DITOLAK: slot "); Serial.print(slot);
        Serial.print(" di luar 0.."); Serial.println(EE_SM_SLOTS - 1);
        return;
    }
    if (us > TRIM_MAKS_US)  us =  TRIM_MAKS_US;
    if (us < -TRIM_MAKS_US) us = -TRIM_MAKS_US;
    gTrim[slot] = us;
    Serial.print("Trim "); Serial.print(SLOT_NAMA[slot]);
    Serial.print(" (slot "); Serial.print(slot);
    Serial.print(") = "); Serial.print(us);
    Serial.println(" us -- RAM saja, 'YtW' untuk menyimpan.");
}

void Hexapod::nolkanTrim() {
    for (uint8_t i = 0; i < TOTAL_SERVOS && i < EE_SM_SLOTS; i++) gTrim[i] = 0;
    Serial.println("Seluruh trim DINOLKAN -- RAM saja, 'YtW' untuk menyimpan.");
}

void Hexapod::cetakTrim() {
    // AWALAN '#TRIM' TETAP. HUD mencocokkan awalan ini, bukan kalimat penuh,
    // supaya menyunting teks di bawah tidak diam-diam mematikan tab trimnya.
    // Pola yang sama dengan '#KORBAN' dan '#LEPAS'.
    Serial.println("--- TRIM SERVO (us) ---");
    for (uint8_t i = 0; i < TOTAL_SERVOS && i < EE_SM_SLOTS; i++) {
        Serial.printf("#TRIM %u %s %u %+d\n",
                      i, SLOT_NAMA[i], SERVO_INVERT[i], gTrim[i]);
    }
    Serial.print("  sumber: ");
    Serial.println(_mapLoaded ? "EEPROM 1024" : "bawaan Calib (EEPROM 1024 kosong/rusak)");
    Serial.println("  'Yt<slot> <us>' setel, 'YtW' simpan, 'Yt!' nolkan semua.");
}

bool Hexapod::simpanServoMap() {
    // BACA DULU, baru timpa. drv[] dan ch[] adalah hasil pemetaan TES_SERVO --
    // firmware ini tidak pernah memakainya (penomoran drivernya kebalikan dari
    // HexaServos) tapi sketsa legacy masih, dan menulis nol ke sana membuang
    // pemetaan yang cuma bisa didapat dengan menelusuri servo satu per satu.
    ServoMap m;
    EEPROM.get(EE_SERVOMAP_ADDR, m);

    uint16_t want = Calib::crc16((const uint8_t*)&m, offsetof(ServoMap, crc));
    if (m.magic[0] != 'S' || m.magic[1] != 'M' || m.version != 1 || m.crc != want) {
        // Blob belum ada: mulai dari kosong, dan tandai drv/ch BELUM DIPETAKAN
        // (-1) alih-alih 0 -- 0 adalah driver yang sah, dan menulisnya berarti
        // berbohong kepada sketsa legacy bahwa pemetaan sudah dikerjakan.
        Serial.println("ServoMap EEPROM 1024 kosong/rusak -> blob BARU dibuat.");
        Serial.println("  drv/ch ditandai BELUM DIPETAKAN -- jalankan TES_SERVO kalau perlu.");
        m.magic[0] = 'S'; m.magic[1] = 'M'; m.version = 1;
        for (uint8_t i = 0; i < EE_SM_SLOTS; i++) { m.drv[i] = -1; m.ch[i] = -1; }
    }

    for (uint8_t i = 0; i < TOTAL_SERVOS && i < EE_SM_SLOTS; i++) {
        m.trim[i]   = gTrim[i];
        m.invert[i] = gInvert[i];
    }
    m.crc = Calib::crc16((const uint8_t*)&m, offsetof(ServoMap, crc));
    EEPROM.put(EE_SERVOMAP_ADDR, m);

    // BACA BALIK. EEPROM Teensy 4.1 itu emulasi di flash, dan tulisan yang
    // gagal tidak melapor. Tanpa baca balik, "tersimpan" cuma berarti
    // "perintahnya dikirim" -- dan yang dipertaruhkan di sini kalibrasi yang
    // baru saja dikerjakan dengan tangan.
    ServoMap cek;
    EEPROM.get(EE_SERVOMAP_ADDR, cek);
    uint16_t cekCrc = Calib::crc16((const uint8_t*)&cek, offsetof(ServoMap, crc));
    if (cek.crc != cekCrc || cek.crc != m.crc) {
        Serial.println("GAGAL menyimpan ServoMap: baca balik tidak cocok.");
        return false;
    }
    _mapLoaded = true;
    Serial.println("Trim TERSIMPAN ke EEPROM 1024 -- bertahan sesudah reset.");
    return true;
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
    const float dMm  = _bodySlewMm * dt;

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
    pasangProfil({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, STAND_HEIGHT, STAND_RADIUS }, 0);
}

void Hexapod::profileStairs() {
    // 1 TANGGA: { 75, 70, 1100, 115, 70 }
    // Perubahan: Tinggi(+35), Langkah(+10), Siklus(+200), Tinggi Badan(+15)
    //
    // Badan +10 -> +25 sesudah trial di lantai pecah (sasis mengandas), lalu
    // DITURUNKAN ke +15 di robot: 125 mm terlalu tinggi untuk dipakai.
    // 125 mm masih jauh di dalam jangkauan IK -- femur 80 + tibia 90 = 170 mm
    // dari pangkal femur, dengan jangkauan mendatar 70 - 20 = 50 mm, batas
    // tegaknya sqrt(170^2 - 50^2) = 162 mm. Perintah 'b<mm>' pun membolehkan
    // sampai 160.
    //
    // Siklus +900 -> +400 -> +200 sesudah trial berturut-turut. Badan maju
    // 2 x stepLength = 140 mm per siklus, jadi angkanya langsung jadi laju:
    // 1800 ms = 7,8 cm/detik (hampir separuh profil DATAR, terasa sangat
    // lambat di arena), 1300 ms = 10,8, dan 1100 ms yang berlaku sekarang
    // = 12,7 cm/detik.
    //
    // Kalau ternyata terlalu cepat sehingga kaki menyangkut bibir ubin,
    // setel dasarnya: 'Qgait.cycle_time 1300' lalu 'T1' -> 1500 ms.
    pasangProfil({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH + 10.0f,
                       GAIT_CYCLE_TIME + 200.0f, STAND_HEIGHT + 15.0f, STAND_RADIUS }, 1);
}

void Hexapod::profileCrouch() {
    // 2 MERUNDUK / TURUNAN: { 40, 45, 1100, 80, 70 }
    // Perubahan: Langkah(-15), Siklus(+200), Tinggi Badan(-20)
    //
    // Langkah 45 mm dipilih untuk turunan 1:4 (14,04 der) di arena. Tiap
    // langkah, tanah di bawah kaki yang MENAPAK turun sebesar
    // panjang_langkah x tan(14,04 der) = panjang_langkah x 0,25 -- 15 mm pada
    // langkah baku 60 mm, 11 mm pada 45 mm. Trayektori ayun kembali ke
    // z = -standHeight di kerangka BADAN, jadi selisih itu persis seberapa
    // jauh kaki menggantung di udara sebelum badan jatuh menimpanya. Ia
    // LINEAR terhadap panjang langkah, jadi memendekkan langkah menyerang
    // tepat di sumbernya. Angka -5 sebelumnya tidak pernah diuji terhadap
    // kemiringan apa pun.
    //
    // Badan -20 mm menolong dua kali. Titik berat lebih rendah mengurangi
    // kecenderungan terjungkal ke depan, yang merupakan mode jatuh di
    // turunan. Dan kaki jadi lebih TERLIPAT, sehingga sisa jangkauan ke
    // bawahnya bertambah -- itu yang dibutuhkan di PUNCAK turunan, saat kaki
    // depan melangkah ke tanah yang jatuh sementara kaki belakang masih di
    // lantai datar.
    pasangProfil({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH - 15.0f, 
                       GAIT_CYCLE_TIME + 200.0f, STAND_HEIGHT - 20.0f, STAND_RADIUS }, 2);
}

// 4 KAIL -- SEMENTARA DISEDERHANAKAN (15 Sep 2026, permintaan Vincent).
// Dasarnya sekarang DATAR (T0) + satu kemiringan badan, titik. Yang dibuang:
// stance dipersempit ke 60, kaki depan maju, kaki belakang diputar lurus ke
// belakang, kaki tengah didorong keluar, tinggi badan 100 milik KAIL, dan
// lutut depan dibekukan.
//
// Yang tersisa cuma yang diminta: badan BELAKANG NAIK, badan DEPAN TURUN.
// Lewat offset z per kaki, karena computeHome() menulis satu z = -standHeight
// untuk keenam kaki dan satu angka tidak bisa berarti dua tinggi sekaligus.
//
//   +z = kaki NAIK                -> badan di sisi itu TURUN
//   -z = kaki MEMANJANG KE BAWAH  -> badan di sisi itu NAIK
//
// Kaki tengah sengaja tidak diubah: merekalah sumbu miringnya.
//
// Jangkauan aman pada STAND_RADIUS 70 dan STAND_HEIGHT 100: kaki belakang
// boleh memanjang 47 mm (dipakai 42) dan kaki depan boleh mengait 75 mm
// (dipakai 40). Kedua batas itu diukur cek_kail.cpp. Naikkan salah satu knop
// melewati batasnya dan IK mentok -- ukur ulang di robot dulu.
//
// Sisa konstanta KAIL_* di config.h SENGAJA dibiarkan di tempatnya. Nilainya
// hasil ukur, dan memulihkan profil lama nanti cuma perlu menulis ulang
// fungsi ini.
void Hexapod::profileKail() {
    // DASAR T0, tapi DITULIS SENDIRI, bukan lewat profileFlat(). Bedanya cuma
    // satu kolom: siklusnya milik KAIL, supaya laju R-9 bisa disetel tanpa
    // ikut mengubah seluruh misi. Empat kolom lain memakai konstanta yang
    // sama persis dengan profileFlat(), jadi keduanya tetap bergerak bersama
    // kalau dasarnya disetel.
    pasangProfil({ GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH,
                   GAIT_CYCLE_TIME + KAIL_CYCLE_TAMBAH_MS,
                   STAND_HEIGHT, STAND_RADIUS }, 4);

    Vec3 off[6] = {};
    off[0] = off[5] = { 0.0f, 0.0f,  KAIL_DEPAN_NAIK     };  // depan: badan TURUN
    off[2] = off[3] = { 0.0f, 0.0f, -KAIL_BELAKANG_TURUN };  // belakang: badan NAIK
    _gait.setOffsetKaki(off);        // WAJIB sesudah pasangProfil():
                                     // setProfile() menghapus offset.
}

// PROFIL TANJAK -- R-9, versi lambat yang kaki depannya BENAR-BENAR maju.
//
// Ia menjawab satu laporan arena: kaki depan berhasil naik ke anak tangga
// berikutnya KURANG DARI 50% per langkah, sementara robot yang berdiri DIAM
// di tanjakan itu tidak merosot sesenti pun. Berdiri diam yang mantap berarti
// gesekannya cukup dan margin jungkirnya cukup -- yang gagal AYUNANnya, bukan
// tumpuannya. Karena itu ketiga perubahan di bawah semuanya soal ayunan.
//
// 1. TINGGI LANGKAH 40 -> 75 mm. profileKail() memakai tinggi langkah DATAR
//    (dulu lewat profileFlat(), sejak 17 Sep 2026 lewat GAIT_STEP_HEIGHT
//    langsung -- yang berubah cuma siklusnya) sebagai
//    dasar, jadi selama ini kaki depan cuma terangkat setinggi langkah DATAR
//    -- di bawah tinggi banyak anak tangga. Telapak yang tidak melewati muka
//    anak tangga menendang mukanya, dan "kadang naik kadang tidak" adalah
//    bentuk yang diharapkan dari kaki yang tingginya pas-pasan. 75 mm angka
//    milik TANGGA, satu-satunya tinggi langkah di firmware ini yang sudah
//    terbukti menaiki anak tangga.
//
// 2. SIKLUS 900 -> 1300 ms, langkah 60 -> 45 mm. Badan yang masih berayun
//    saat telapak mendarat memindahkan momentumnya ke kaki yang tumpuannya
//    paling tipis, dan di tanjakan itu selalu kaki depan.
//
// 3. GEOMETRI KAKI PENUH, DIAMBIL DARI program-krsri-misi. profileKail() di
//    pohon ini sengaja disederhanakan jadi offset z saja (lihat catatan di
//    atasnya), jadi tujuh konstanta KAIL_* berdiri di config.h tanpa ada yang
//    membacanya. Yang paling mahal hilangnya:
//
//    - KAKI BELAKANG DIPUTAR LURUS KE BELAKANG. Langkah gait searah +-y, jadi
//      begitu telapak belakang duduk di garis x lokal = 0 seluruh stroke jadi
//      RADIAL dan coxa berhenti mengayuh: 41,5 -> 0,0 der per siklus, diukur.
//      Coxa yang mengayuh sambil telapak menahan badan yang mendongak berarti
//      telapak DISERET MENYAMPING. IK tidak pernah melaporkannya, karena IK
//      cuma tahu posisi, bukan gesekan.
//    - KAKI DEPAN MAJU 60 mm, menaruh telapak lebih dalam di TAPAK anak tangga
//      alih-alih di bibirnya.
//    - STANCE DIPERSEMPIT 70 -> 60 mm (KAIL_RADIUS_KAKI), yang justru
//      MEMBEBASKAN jangkauan: belakang boleh memanjang sampai 52 mm (dari 47)
//      dan depan mengait sampai 85 mm (dari 75) sebelum IK mentok.
//
//    Harga stance sempit: poligon tumpuan menyempit, jadi lebih mudah oleng
//    MENYAMPING di bidang 27,3 der. Kalau robot mulai goyang kiri-kanan di
//    tangga, KAIL_RADIUS_KAKI tersangka pertamanya.
//
// Pitch-nya SENGAJA sama dengan KAIL: depan +40 dan belakang -42 memberi
// atan(82/156) = 27,7 der. config.h:547 memperingatkan bahwa melewati 27,3 der
// membuat badan MENUNDUK di tanjakan sehingga telapak depan menekan tegak
// lurus ke muka anak tangga alih-alih mengait tapaknya -- persis gejala yang
// sedang diperbaiki di sini.
//
// LUTUT DEPAN TIDAK DIKUNCI di sini, walau program-krsri-misi menguncinya.
// Kunci lutut membuang komponen radial lintasan ayun, dan yang sedang dibeli
// di profil ini justru tinggi ayunan itu. Kalau telapak depan ternyata
// MENGGARUK tapak (bukan gagal naik), itu knop berikutnya: pasang
// _lututKunci = (1 << 0) | (1 << 5) tepat sebelum kurung tutup.
void Hexapod::profileTanjak() {
    pasangProfil({ GAIT_STEP_HEIGHT + 35.0f, GAIT_STEP_LENGTH - 15.0f,
                   GAIT_CYCLE_TIME + 400.0f, KAIL_TINGGI_BADAN, KAIL_RADIUS_KAKI }, 5);

    // Telapak belakang diputar ke -y pada radius yang SAMA, jadi jangkauan D
    // tidak berubah sedikit pun -- yang berubah cuma arahnya.
    const float aBlk = deg2rad(BODY_LEG_ANGLE[2]);
    const float blkX = KAIL_RADIUS_KAKI * (0.0f - cosf(aBlk)) + KAIL_BELAKANG_LEBAR;
    const float blkY = KAIL_RADIUS_KAKI * (-1.0f - sinf(aBlk)) - KAIL_BELAKANG_MUNDUR;

    // Kaki tengah: radius dan SUDUT, bukan geseran x. Memutar pada radius
    // tetap membuat jangkauan IK-nya juga tidak berubah.
    const float rTgh = KAIL_RADIUS_KAKI + KAIL_TENGAH_KELUAR;
    const float tghX = rTgh * cosf(deg2rad(KAIL_TENGAH_SUDUT)) - KAIL_RADIUS_KAKI;
    const float tghY = rTgh * sinf(deg2rad(KAIL_TENGAH_SUDUT));

    Vec3 off[6] = {};
    off[0] = off[5] = { 0.0f, KAIL_DEPAN_MAJU, KAIL_DEPAN_NAIK };
    off[1] = {  tghX, tghY, 0.0f };
    off[4] = { -tghX, tghY, 0.0f };                   // cermin kaki 1
    off[2] = {  blkX, blkY, -KAIL_BELAKANG_TURUN };
    off[3] = { -blkX, blkY, -KAIL_BELAKANG_TURUN };   // cermin kaki 2
    _gait.setOffsetKaki(off);        // WAJIB sesudah pasangProfil():
                                     // setProfile() menghapus offset.
}

void Hexapod::profileNarrow() {
    // 3 SEMPIT: { 30, 45, 1000, 100, 45 }
    // Perubahan: Tinggi(-10), Langkah(-15), Siklus(+100), Lebar Kaki(-25)
    pasangProfil({ GAIT_STEP_HEIGHT - 10.0f, GAIT_STEP_LENGTH - 15.0f, 
                       GAIT_CYCLE_TIME + 100.0f, STAND_HEIGHT, STAND_RADIUS - 25.0f }, 3);
}

static bool profilSah(const GaitProfile& p) {
    return isfinite(p.stepHeight) && p.stepHeight >= 0 && p.stepHeight <= 120 &&
           isfinite(p.stepLength) && p.stepLength >= 0 && p.stepLength <= 150 &&
           isfinite(p.cycleTime) && p.cycleTime >= 300 && p.cycleTime <= 3000 &&
           isfinite(p.standHeight) && p.standHeight >= 40 && p.standHeight <= 160 &&
           isfinite(p.standRadius) && p.standRadius >= 30 && p.standRadius <= 120;
}

void Hexapod::pilihProfil(uint8_t id) {
    switch (id) {
        case 0: profileFlat(); break;
        case 1: profileStairs(); break;
        case 2: profileCrouch(); break;
        case 3: profileNarrow(); break;
        case 4: profileKail(); break;
        case 5: profileTanjak(); break;
    }
}

bool Hexapod::ubahProfil(uint8_t id, const GaitProfile& p) {
    if (id >= 6 || !profilSah(p)) return false;
    _profil.nilai[id] = p;
    _profil.mask |= 1 << id;
    pilihProfil(id);
    return true;
}

void Hexapod::resetProfil(uint8_t id) {
    if (id >= 6) return;
    _profil.mask &= ~(1 << id);
    pilihProfil(id);
}

bool Hexapod::muatProfil() {
    ProfilStore p;
    EEPROM.get(EE_PROFIL_ADDR, p);
    if (p.magic != 0x4850 || p.version != 1 || p.mask > 63 ||
        p.crc != Calib::crc16((const uint8_t*)&p, offsetof(ProfilStore, crc))) return false;
    for (int i = 0; i < 6; ++i)
        if ((p.mask & (1 << i)) && !profilSah(p.nilai[i])) return false;
    _profil = p;
    if (_profilId >= 0) pilihProfil(_profilId);
    return true;
}

bool Hexapod::simpanProfil() {
    if (_profilId >= 0) {
        const GaitProfile p = _gait.targetProfile();
        if (!profilSah(p)) return false;
        _profil.nilai[_profilId] = p;
        _profil.mask |= 1 << _profilId;
    }
    _profil.magic = 0x4850;
    _profil.version = 1;
    _profil.crc = Calib::crc16((const uint8_t*)&_profil, offsetof(ProfilStore, crc));
    EEPROM.put(EE_PROFIL_ADDR, _profil);
    ProfilStore cek;
    EEPROM.get(EE_PROFIL_ADDR, cek);
    return memcmp(&cek, &_profil, sizeof cek) == 0;
}

void Hexapod::cetakProfil() const {
    const GaitProfile p = _gait.targetProfile();
    Serial.printf("#PROFIL %d %.2f %.2f %.2f %.2f %.2f %u\n", _profilId,
                  p.stepHeight, p.stepLength, p.cycleTime, p.standHeight, p.standRadius,
                  (unsigned)_profil.mask);
    Serial.printf("#SERVO %d\n", isArmed() ? 1 : 0);
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

// HANYA LENGAN DEPAN yang bisa menjangkau. Ia BAHU, SIKU, PERGELANGAN, GRIP:
// bahu+siku menyapu bidang VERTIKAL yang menghadap keluar badan, dan karena
// tidak ada sendi pemutar di pangkal, untuk membidik objek yang tidak segaris
// BADAN robot yang harus diarahkan. Karena itu IK 2 link di ArmInverse memang
// bentuk yang tepat -- PERGELANGAN sengaja di luar IK, ia sudut ketiga yang
// disetel sendiri lewat setPergelangan().
//
// LENGAN BELAKANG cuma grip. Tidak ada yang bisa dijangkau, jadi fungsi ini
// MENOLAK -- letak capit belakang ditentukan letak badan, bukan sudut sendi.
//
// Slot kalibrasi: depan 18=bahu 19=siku 20=pergelangan 21=grip, belakang
// 22=grip. Slot 23 dialokasikan tapi tidak terpakai.
// (SLOT_NAME[] di legacy TES_GERAK/servo_map.h masih menulis "BASE" untuk
//  slot pertama dan menamai lengan "R"/"L" -- label itu USANG.)
//
// jangkauan/tinggi diukur dari PUSAT BADAN; offset pangkal bahu dikurangkan
// di sini. Versi lama mengurangkan ARM_ORIGINS[0][0] -- offset ke SAMPING --
// dari koordinat jangkauan, dua sumbu yang berbeda.
bool Hexapod::moveArmTarget(uint8_t arm, float jangkauan, float tinggi) {
    if (arm != ARM_DEPAN) return false;   // belakang tidak punya sendi
    HexaArm* a = (arm == ARM_DEPAN) ? &_armF : &_armB;

    // fabsf: lengan belakang punya origin -Y, tapi "jangkauan" untuk kedua
    // lengan sama-sama positif ke arah hadapnya masing-masing.
    float lokalX = jangkauan - fabsf(ARM_ORIGINS[arm][1]);
    float lokalY = tinggi    - ARM_ORIGINS[arm][2];

    float bahuDeg = 0.0f, sikuDeg = 0.0f;
    if (!ArmInverse::solve(lokalX, lokalY, bahuDeg, sikuDeg)) return false;

    a->setArmPulse(ARM_ID_BAHU, a->angleToPulse(ARM_ID_BAHU, bahuDeg, ARM_BASE_BAHU));
    // BASELINE SIKU 0, BUKAN 90 SEPERTI BAHU. Keduanya bukan sudut sejenis:
    // bahuDeg bertanda, berayun di sekitar 0, jadi baseline 90 menaruhnya di
    // tengah rentang servo. sikuDeg adalah sudut DALAM dari acos -- selalu
    // 0..180, tidak pernah negatif. Dengan baseline 90 ia jadi 90..270 dan
    // clampf() di angleToPulse() memakan segala yang di atas 180 -- DIAM-DIAM,
    // tanpa penanda seperti _servoClamped milik kaki. Diukur di cek_lengan:
    // dua pertiga jangkauan lengan (r < 31 mm dari bahu) hilang begitu saja.
    //
    // Kalau siku ternyata berputar ke arah yang salah, JANGAN membalik ke
    // baseline 90 + invert: itu cuma mencerminkan clamp-nya ke ujung bawah.
    // Pasangannya yang benar adalah baseline 180 + invert slot 19 -- 180-siku
    // juga menempati 0..180 penuh, cuma berlawanan arah.
    a->setArmPulse(ARM_ID_SIKU, a->angleToPulse(ARM_ID_SIKU, sikuDeg, ARM_BASE_SIKU));
    return true;
}

// Target di TITIK CAPIT dengan tapak dijaga MENDATAR. Lihat Hexapod.h.
bool Hexapod::moveArmGrip(uint8_t arm, float jangkauan, float tinggi, float tapakDeg) {
    if (arm != ARM_DEPAN) return false;   // belakang tidak punya sendi

    // Titik PERGELANGAN = titik capit dikurangi satu tapak, searah tapak.
    const float t   = deg2rad(tapakDeg);
    const float pjk = jangkauan - HAND_LENGTH * cosf(t);
    const float ptg = tinggi    - HAND_LENGTH * sinf(t);

    float bahuDeg = 0.0f, sikuDeg = 0.0f;
    if (!ArmInverse::solve(pjk - fabsf(ARM_ORIGINS[arm][1]),
                           ptg - ARM_ORIGINS[arm][2], bahuDeg, sikuDeg)) return false;

    // Sudut tapak = jumlah SEMUA putaran sendi dari badan. bahuDeg diukur dari
    // mendatar-ke-depan dan sikuDeg adalah putaran lengan bawah terhadap
    // lengan atas (0 = lurus), jadi sisanya milik pergelangan.
    const float prgDeg = tapakDeg - (bahuDeg + sikuDeg);
    if (fabsf(prgDeg) > 90.0f) return false;

    // Sudut servo di luar 0..180 = ter-clamp diam-diam di angleToPulse().
    const float bahuServo = ARM_BASE_BAHU + bahuDeg;
    const float sikuServo = ARM_BASE_SIKU + sikuDeg;
    if (bahuServo < 0.0f || bahuServo > 180.0f) return false;
    if (sikuServo < 0.0f || sikuServo > 180.0f) return false;

    moveArmTarget(arm, pjk, ptg);   // menghitung ulang IK yang sama; murah,
    setPergelangan(arm, prgDeg);    // dan pulsa tetap lahir di satu tempat
    return true;
}

// PERGELANGAN: sudut ketiga lengan depan, disetel sendiri dan TIDAK ikut IK.
// Sudut geometris terhadap baseline 90 der, sama seperti bahu/siku, jadi
// offset/trim/invert slot 20 berlaku lewat jalur yang sama.
bool Hexapod::setSudutLengan(uint8_t arm, float bahuDeg, float sikuDeg,
                             float prgDeg, float* servoOut) {
    if (arm != ARM_DEPAN) return false;    // belakang cuma grip

    const uint8_t id[3]   = { ARM_ID_BAHU, ARM_ID_SIKU, ARM_ID_PERGELANGAN };
    const float   base[3] = { ARM_BASE_BAHU, ARM_BASE_SIKU, ARM_BASE_PERGELANGAN };
    const float   geo[3]  = { bahuDeg, sikuDeg, prgDeg };

    bool sanggup = true;
    for (uint8_t i = 0; i < 3; i++) {
        const float sv = _armF.sudutServo(id[i], geo[i], base[i]);
        if (servoOut) servoOut[i] = sv;
        if (sv < 0.0f || sv > 180.0f) sanggup = false;
        _armF.setArmPulse(id[i], _armF.angleToPulse(id[i], geo[i], base[i]));
    }
    return sanggup;
}

void Hexapod::setSlewLengan(uint8_t arm, float degS) {
    if (arm > 1) return;
    ((arm == ARM_DEPAN) ? _armF : _armB).setSlew(degS);
}

bool Hexapod::setPergelangan(uint8_t arm, float deg) {
    if (arm != ARM_DEPAN) return false;   // belakang tidak punya pergelangan
    deg = clampf(deg, -90.0f, 90.0f);
    _armF.setArmPulse(ARM_ID_PERGELANGAN,
                      _armF.angleToPulse(ARM_ID_PERGELANGAN, deg, ARM_BASE_PERGELANGAN));
    return true;
}

// Penjepit: 0 = menutup penuh, 100 = membuka penuh. Sudut servo 0..180 der
// lewat jalur kalibrasi yang sama (offset/trim/invert slot grip berlaku).
bool Hexapod::setGrip(uint8_t arm, float persen) {
    if (arm > 1) return false;
    HexaArm* a = (arm == ARM_DEPAN) ? &_armF : &_armB;
    persen = clampf(persen, GRIP_PERSEN_MIN, GRIP_PERSEN_MAKS);   // MG90S, lihat config.h
    float geo = persen / 100.0f * 180.0f - 90.0f;   // baseline 90 der
    const uint8_t id = gripId(arm);                 // depan 3, belakang 0
    a->setArmPulse(id, a->angleToPulse(id, geo, ARM_BASE_GRIP));
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

    // --- LENGAN ---------------------------------------------------
    // Satu-satunya cara melihat invert & trim lengan dari serial: keduanya
    // datang dari ServoMap EEPROM 1024, bukan dari parameter 'Q' yang bisa
    // dibaca 'q'. "basis" adalah baseline yang dipakai angleToPulse() --
    // dicetak karena dialah yang menentukan bagian mana dari 0..180 der
    // servo yang terpakai, dan dia tidak muncul di mana pun selain sini.
    Serial.printf("\nlengan pulse (us) : %u .. %u\n",
                  SERVO_ARM_PULSE_MIN, SERVO_ARM_PULSE_MAX);
    Serial.printf("slew lengan       : %.0f der/detik\n", (double)ARM_SLEW_DEG_S);
    Serial.println("sendi    slot  inv    off   trim  basis  target   kini  PWM");
    const struct { const char* nama; HexaArm* a; uint8_t id; float basis; } LNG[] = {
        { "bahu",   &_armF, ARM_ID_BAHU,          ARM_BASE_BAHU        },
        { "siku",   &_armF, ARM_ID_SIKU,          ARM_BASE_SIKU        },
        { "prglng", &_armF, ARM_ID_PERGELANGAN,   ARM_BASE_PERGELANGAN },
        { "gripD",  &_armF, ARM_ID_GRIP_DEPAN,    ARM_BASE_GRIP        },
        { "gripB",  &_armB, ARM_ID_GRIP_BELAKANG, ARM_BASE_GRIP        },
    };
    for (uint8_t i = 0; i < 5; i++) {
        // Slot kalibrasi dihitung sama seperti di konstruktor HexaArm:
        // depan mulai NUM_SERVOS, belakang NUM_SERVOS + ARM_N_DEPAN.
        uint8_t slot = (i < 4) ? (uint8_t)(NUM_SERVOS + LNG[i].id)
                               : (uint8_t)(NUM_SERVOS + ARM_N_DEPAN + LNG[i].id);
        // target DAN kini: dengan slew keduanya berbeda selama lengan masih
        // berjalan, dan "diminta" vs "sedang dikirim" adalah dua pertanyaan
        // yang berbeda saat lengan tidak sampai ke tempat yang diminta.
        Serial.printf("%-8s  %2u   %u  %+5.1f  %+5d  %5.1f   %4u   %4u  %s\n",
            LNG[i].nama, slot, SERVO_INVERT[slot], (double)SERVO_OFFSET[slot],
            SERVO_TRIM_US[slot], (double)LNG[i].basis,
            LNG[i].a->targetPulse(LNG[i].id), LNG[i].a->pulseKini(LNG[i].id),
            LNG[i].a->isEnabled() ? "aktif" : "mati");
    }

    // --- UJI ARAH SERVO LENGAN -------------------------------------
    // Perintah 'a' menggerakkan bahu DAN siku sekaligus, jadi arah satu
    // sendi tidak bisa dibaca dari sembarang pose. Dua sasaran di bawah
    // memisahkannya, dan keduanya DIHITUNG dari panjang link + ARM_ORIGINS
    // yang berlaku sekarang -- bukan angka tetap yang basi tiap kali lengan
    // diukur ulang.
    //   sasaran-1: radius sama dengan netral, arah diputar -30 der
    //              -> sudut siku tetap, cuma bahu yang berubah
    //   sasaran-2: radius maksimum (lengan wajib lurus) tepat lurus ke
    //              depan -> bahu tetap nol, cuma siku yang berubah
    const float oy = fabsf(ARM_ORIGINS[ARM_DEPAN][1]);
    const float oz = ARM_ORIGINS[ARM_DEPAN][2];
    const float r0 = hypotf(UPPERARM_LENGTH, FOREARM_LENGTH);
    const float t1 = atan2f(FOREARM_LENGTH, UPPERARM_LENGTH) - deg2rad(30.0f);
    Serial.println("\nuji arah (servo lengan hidup, mulai dari netral):");
    Serial.printf("  a%-4.0f %-4.0f      NETRAL: bentuk L -- atas mendatar, bawah TEGAK\n",
                  (double)(oy + UPPERARM_LENGTH), (double)(oz + FOREARM_LENGTH));
    Serial.printf("  a%-4.0f %-4.0f      lengan ATAS turun 30 der, siku DIAM   -> slot 18\n",
                  (double)(oy + r0 * cosf(t1)), (double)(oz + r0 * sinf(t1)));
    Serial.printf("  a%-4.0f %-4.0f      lengan LURUS MENDATAR (siku 90->0)    -> slot 19\n",
                  (double)(oy + UPPERARM_LENGTH + FOREARM_LENGTH), (double)oz);
    Serial.printf("  a%-4.0f %-4.0f 30   hanya pergelangan yang miring         -> slot 20\n",
                  (double)(oy + UPPERARM_LENGTH), (double)(oz + FOREARM_LENGTH));
    Serial.println("  g20 / G20      capit MENUTUP dari 50%                 -> slot 21 / 22");
    Serial.println("  (bergerak ke arah sebaliknya = slot itu yang ter-invert)");
    // Sesudah slew dipasang, perintah lengan tidak lagi seketika. Langkah
    // terbesar uji ini 90 der (siku 90 -> 0), jadi mengetik perintah
    // berikutnya terlalu cepat berarti yang diamati gerakan yang BELUM selesai
    // -- dan arah gerakan setengah jalan bisa terbaca terbalik.
    Serial.printf("  beri jeda ~%.0f ms tiap perintah: servo merayap %.0f der/detik\n",
                  (double)(90.0f / ARM_SLEW_DEG_S * 1000.0f), (double)ARM_SLEW_DEG_S);
    Serial.println("===============================================\n");
}

// Sudut lutut yang dipakai kaki ini kalau dibekukan: sudut lutut pada pose
// NETRALnya sendiri, dihitung ulang tiap update karena offset KAIL di-ramp.
// Akibatnya bentuk BERDIRI sama persis dengan IK biasa -- yang berubah hanya
// ayunan. NAN = kaki ini tidak dikunci.
float Hexapod::kunciLutut(int leg) {
    if (!(_lututKunci & (1 << leg))) return NAN;
    float lx, ly, lz, c, f, t;
    legSolveAt(leg, _gait.footHome(leg), lx, ly, lz, c, f, t, NAN);
    return t;
}

bool Hexapod::legSolve(int leg, float& lx, float& ly, float& lz,
                       float& coxa, float& femur, float& tibia) {
    return legSolveAt(leg, _gait.legTargets[leg], lx, ly, lz,
                      coxa, femur, tibia, kunciLutut(leg));
}

// Satu kaki: titik telapak -> transform badan -> frame kaki -> IK.
// Dipakai bersama oleh solvePose() dan debugDump() supaya angka yang dicetak
// dijamin sama dengan yang benar-benar dikirim ke servo.
bool Hexapod::legSolveAt(int leg, const Vec3& foot, float& lx, float& ly, float& lz,
                         float& coxa, float& femur, float& tibia, float kunciLututDeg) {
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

    return LegInverseKinematics::solve(lx, ly, lz, coxa, femur, tibia, kunciLututDeg);
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
