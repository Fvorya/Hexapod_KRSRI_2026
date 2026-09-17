#include "HexaGait.h"
#include <Arduino.h>

HexaGait::HexaGait() {
    // 1. Menambahkan STAND_RADIUS ke nilai inisiasi
    _prof = _tgtProf = { GAIT_STEP_HEIGHT, GAIT_STEP_LENGTH, GAIT_CYCLE_TIME, STAND_HEIGHT, STAND_RADIUS };
    for (int i = 0; i < 6; i++) _off[i] = _tgtOff[i] = { 0.0f, 0.0f, 0.0f };
    _tgtX = _tgtY = _tgtYaw = 0;
    _curX = _curY = _curYaw = 0;
    _running = false;
    _phase = 0.0f; // Menggantikan _cycleStart
    _lastUpdate = 0;
}

void HexaGait::computeHome() {
    // Kaki netral = pangkal coxa + radius dinamis searah hadap kaki.
    for (int i = 0; i < 6; i++) {
        float a = deg2rad(BODY_LEG_ANGLE[i]);
        // 2. Menggunakan _prof.standRadius, bukan nilai statis
        _footHome[i].x = BODY_LEG_ORIGINS[i][0] + _prof.standRadius * cosf(a);
        _footHome[i].y = BODY_LEG_ORIGINS[i][1] + _prof.standRadius * sinf(a);
        _footHome[i].z = -_prof.standHeight;

        // Offset per kaki DI ATAS bentuk seragam. Nol untuk semua profil
        // kecuali KAIL, jadi jalur ini tidak mengubah apa pun sampai ada
        // yang benar-benar memintanya.
        _footHome[i].x += _off[i].x;
        _footHome[i].y += _off[i].y;
        _footHome[i].z += _off[i].z;
    }
}

void HexaGait::begin() {
    computeHome();
    for (int i = 0; i < 6; i++) legTargets[i] = _footHome[i];
    _lastUpdate = millis();
}

void HexaGait::setMoveVector(float vx, float vy, float vyaw) {
    _tgtX = vx; _tgtY = vy; _tgtYaw = vyaw;   // di-slew di update()
}

// dt detik sejak update terakhir, di-clamp agar aman saat jeda besar.
float HexaGait::dtSeconds() {
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    _lastUpdate = now;
    return clampf(dt, 0.0f, 0.05f);
}

// ramp 'cur' menuju 'tgt' dengan laju maks (unit/detik).
static float slew(float cur, float tgt, float rate, float dt) {
    float step = rate * dt;
    if (tgt > cur) return (cur + step < tgt) ? cur + step : tgt;
    if (tgt < cur) return (cur - step > tgt) ? cur - step : tgt;
    return tgt;
}

void HexaGait::update() {
    float dt = dtSeconds();

    // 1) Ramp vektor gerak (start/stop/belok mulus + ease-in otomatis).
    _curX   = slew(_curX,   _tgtX,   GAIT_SLEW_RATE, dt);
    _curY   = slew(_curY,   _tgtY,   GAIT_SLEW_RATE, dt);
    _curYaw = slew(_curYaw, _tgtYaw, GAIT_SLEW_RATE, dt);

    // 2) Ramp profil medan (termasuk standRadius sekarang).
    float ap = dt / (GAIT_PROFILE_TAU + dt);
    _prof.stepHeight  = lerpf(_prof.stepHeight,  _tgtProf.stepHeight,  ap);
    _prof.stepLength  = lerpf(_prof.stepLength,  _tgtProf.stepLength,  ap);
    _prof.cycleTime   = lerpf(_prof.cycleTime,   _tgtProf.cycleTime,   ap);
    _prof.standHeight = lerpf(_prof.standHeight, _tgtProf.standHeight, ap);
    _prof.standRadius = lerpf(_prof.standRadius, _tgtProf.standRadius, ap);
    for (int i = 0; i < 6; i++) {
        _off[i].x = lerpf(_off[i].x, _tgtOff[i].x, ap);
        _off[i].y = lerpf(_off[i].y, _tgtOff[i].y, ap);
        _off[i].z = lerpf(_off[i].z, _tgtOff[i].z, ap);
    }

    computeHome();

    bool moving = (fabsf(_curX) + fabsf(_curY) + fabsf(_curYaw)) > 0.002f;
    if (moving && !_running) { 
        _running = true; 
        _phase = 0.0f; // Reset fase saat mulai melangkah
    }
    if (!moving) {
        _running = false;
        _phase = 0.0f;
        // Settle ke home, berbasis waktu (konstan tau, tak tergantung loop).
        float as = dt / (GAIT_SETTLE_TAU + dt);
        for (int i = 0; i < 6; i++) {
            legTargets[i].x = lerpf(legTargets[i].x, _footHome[i].x, as);
            legTargets[i].y = lerpf(legTargets[i].y, _footHome[i].y, as);
            legTargets[i].z = lerpf(legTargets[i].z, _footHome[i].z, as);
        }
        return;
    }

    // 3. Fase Diakumulasi (kebal terhadap transisi cycleTime)
    if (_prof.cycleTime < 100.0f) _prof.cycleTime = 100.0f;
    float dPhase = dt * 1000.0f / _prof.cycleTime;
    _phase += dPhase;
    while (_phase >= 1.0f) {
        _phase -= 1.0f;
    }

    // 4. Normalisasi Vektor Langkah (Mencegah servo terbakar saat maju sambil berputar)
    float sxa[6], sya[6], magMax = 0.0f;
    
    // Hitung seluruh vektor target awal
    for (int leg = 0; leg < 6; leg++) {
        float rx = _footHome[leg].x, ry = _footHome[leg].y;
        sxa[leg] = (_curX + (-_curYaw * ry / 100.0f)) * _prof.stepLength;
        sya[leg] = (_curY + ( _curYaw * rx / 100.0f)) * _prof.stepLength;
        
        float m = sqrtf(sxa[leg] * sxa[leg] + sya[leg] * sya[leg]);
        if (m > magMax) magMax = m;
    }

    // Pangkas (Normalisasi) bersama-sama jika ada kaki yang melampaui stepLength
    float f = 1.0f;
    if (magMax > _prof.stepLength && magMax > 0.001f) {
        f = _prof.stepLength / magMax;
        for (int leg = 0; leg < 6; leg++) {
            sxa[leg] *= f;
            sya[leg] *= f;
        }
    }

    // ODOMETRI. Dihitung DI SINI, sesudah normalisasi, karena hanya di sini
    // 'f' diketahui -- dan tanpa 'f' jarak saat menyusuri dinding akan
    // terhitung lebih jauh daripada saat lurus karena bug rumus, lalu
    // disalahartikan sebagai slip mekanis.
    //
    // Suku yaw batal sendiri saat dirata-rata enam kaki (rx simetris
    // kiri-kanan), jadi cukup _curY; tidak perlu merata-rata sya[].
    // Memakai dPhase, bukan dt: bukan supaya kebal loop tersendat (dt sudah
    // di-clamp sama seperti dPhase), melainkan supaya odometer tak pernah
    // berselisih dengan seberapa jauh kaki sungguh maju -- dPhase dihitung
    // dari dt yang sama dan cycleTime yang sama (dengan floor yang sama)
    // yang dipakai fase gait itu sendiri untuk melangkah.
    //
    // Koefisien 2.0 mengasumsikan jendela tumpu (stance) kedua tripod pas
    // menutupi satu siklus penuh tanpa celah maupun tindih -- itu hanya
    // benar saat GAIT_DUTY = 0.5. Untuk duty > 0.5 jendela tumpu kedua
    // tripod saling tindih, sehingga total waktu tumpu per siklus jadi
    // SATU siklus (bukan 2 x duty x T), dan badan sungguh maju sejauh
    // sy / duty per siklus, bukan 2*sy. fminf(2.0, 1/duty) mengoreksi ini;
    // pada duty <= 0.5 hasilnya tetap 2.0 seperti semula.
    float majuMm = _skalaOdo * fminf(2.0f, 1.0f / GAIT_DUTY) * _curY * _prof.stepLength * f * dPhase;
    _jarakMm += majuMm;

    // Sumbu geser diakumulasi dengan rumus yang SAMA PERSIS -- ia melewati
    // normalisasi 'f' yang sama dan skala slip yang sama, jadi memisahkannya
    // hanya akan membuat dua rumus yang bisa menyimpang diam-diam.
    float geserMm = _skalaOdo * fminf(2.0f, 1.0f / GAIT_DUTY) * _curX * _prof.stepLength * f * dPhase;
    _geserMm += geserMm;

    // Panjang lintasan: besar perpindahan, bukan komponennya. Dipakai rem
    // jarak supaya satu pagar berlaku untuk maju, mundur, dan kepiting.
    _lintasMm += sqrtf(majuMm * majuMm + geserMm * geserMm);
    // Laju sesaat dari PERTAMBAHAN yang sama, bukan dari menyelisihkan
    // _jarakMm belakangan: rumusnya sudah memuat ramp _curY, penormalan
    // langkah f, dan skala slip -- tiga hal yang mustahil ditebak dari luar.
    if (dt > 1e-5f) _lajuMmS = majuMm / dt;

    // 5. Eksekusi Gerakan
    for (int leg = 0; leg < 6; leg++) {
        // Tripod: grup {0,2,4} fase 0, grup {1,3,5} geser 0.5.
        float legPhase = _phase + ((leg % 2 == 0) ? 0.0f : 0.5f);
        if (legPhase >= 1.0f) legPhase -= 1.0f;

        float jumlahTitik = 100.0f; // Ini buat ngebagi kurva sikloid jadi beberapa titik
        float stepPhase = floor(legPhase * jumlahTitik) / jumlahTitik;

        // Ambil nilai yang sudah dinormalisasi
        float sx = sxa[leg];
        float sy = sya[leg];

        float dx, dy, dz;
        if (stepPhase < GAIT_DUTY) {
            // STANCE: geser lurus +1/2 -> -1/2 (dorong badan maju), kecepatan konstan.
            float s = stepPhase / GAIT_DUTY;          // 0..1
            float k = 0.5f - s;
            dx = sx * k; dy = sy * k; dz = 0.0f;
        } else {
            // SWING SIKLOID: kecepatan nol di liftoff & touchdown -> mendarat lembut.
            float s = (stepPhase - GAIT_DUTY) / (1.0f - GAIT_DUTY); // 0..1
            float twoPiS = 2.0f * (float)M_PI * s;
            float k = -0.5f + (s - sinf(twoPiS) / (2.0f * (float)M_PI)); // -0.5 -> +0.5
            dx = sx * k; dy = sy * k;
            dz = _prof.stepHeight * (1.0f - cosf(twoPiS)) * 0.5f;        // 0 -> peak -> 0
        }

        legTargets[leg].x = _footHome[leg].x + dx;
        legTargets[leg].y = _footHome[leg].y + dy;
        legTargets[leg].z = _footHome[leg].z + dz;
    }

    // Telemetri fase. Dulu dicetak tanpa syarat tiap 200 ms -- termasuk saat
    // robot berdiri diam -- sehingga menenggelamkan output perintah lain.
    // Sekarang hanya saat benar-benar melangkah, dan bisa dimatikan lewat
    // GAIT_DEBUG di config.h.
#if GAIT_DEBUG
    static uint32_t tPrint = 0;
    if (millis() - tPrint > 200) {
        tPrint = millis();
        Serial.printf("phase=%.3f dt=%.4f cyc=%.0f\n", (float)_phase, dt, _prof.cycleTime);
    }
#endif
}
