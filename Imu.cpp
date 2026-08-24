#include "Imu.h"
#include <string.h> // Dibutuhkan untuk fungsi memmove()

Imu::Imu() {
    _rxN = 0;
    _have = false;
    _roll = _pitch = _yaw = 0;
    _roll0 = _pitch0 = 0;
    _gz = 0;
    _tolakYaw = 0;
    _ax = _ay = _az = 0;
    _mhx = _mhy = _mhz = 0;
}

void Imu::begin() {
    // 1. Suntikkan buffer ekstra SEBELUM komunikasi serial dimulai
    IMU_SERIAL.addMemoryForRead(_rxExtra, sizeof(_rxExtra));
    
    // 2. Mulai komunikasi (Pastikan IMU_BAUD di config.h sudah diubah ke 230400)
    IMU_SERIAL.begin(IMU_BAUD); 
}

void Imu::update() {
    // Logika Resinkronisasi Sejati (membuang byte satu per satu jika gagal).
    // Dibatasi IMU_MAX_BYTE_UPDATE byte per panggilan supaya loop utama tidak
    // bisa kelaparan kalau IMU membanjiri serial.
    uint16_t jatah = IMU_MAX_BYTE_UPDATE;
    while (IMU_SERIAL.available() && jatah--) {
        
        // Jika buffer penuh, paksa geser 1 byte ke kiri
        if (_rxN >= sizeof(_rxBuf)) {               
            memmove(_rxBuf, _rxBuf + 1, --_rxN);
        }
        
        // Baca data baru masuk
        _rxBuf[_rxN++] = (uint8_t)IMU_SERIAL.read();

        while (_rxN >= WIT_LEN) {
            
            // Validasi Header (0x55)
            if (_rxBuf[0] != 0x55) {
                memmove(_rxBuf, _rxBuf + 1, --_rxN); // Buang 1 byte
                continue;
            }
            
            // Validasi Checksum
            uint8_t sum = 0;
            for (uint8_t i = 0; i < WIT_LEN - 1; i++) sum += _rxBuf[i];
            if (sum != _rxBuf[WIT_LEN - 1]) {
                memmove(_rxBuf, _rxBuf + 1, --_rxN); // Buang 1 byte
                continue;
            }
            
            // Frame valid, terjemahkan nilainya
            parseFrame(_rxBuf);
            
            // Buang 11 byte yang sudah sukses diproses dari buffer
            _rxN -= WIT_LEN;
            memmove(_rxBuf, _rxBuf + WIT_LEN, _rxN);
        }
    }
}

void Imu::parseFrame(const uint8_t* f) {
    uint8_t t = f[1]; // Tipe paket (0x52 untuk Gyro, 0x53 untuk Sudut)
    
    int16_t v0 = (int16_t)(f[3] << 8 | f[2]);
    int16_t v1 = (int16_t)(f[5] << 8 | f[4]);
    int16_t v2 = (int16_t)(f[7] << 8 | f[6]);

    switch (t) {
        case 0x51: // Accelerometer (+- 16g)
            _ax = v0 / 32768.0f * 16.0f;
            _ay = v1 / 32768.0f * 16.0f;
            _az = v2 / 32768.0f * 16.0f;
            break;
            
        case 0x52: // Gyro / Angular Velocity (+- 2000 derajat/detik)
            // Di sini kita hanya menyimpan Gyro Z karena paling relevan untuk Pivot
            _gz = v2 / 32768.0f * 2000.0f; 
            break;
            
        case 0x53: { // Sudut Euler (+- 180 derajat)
            _roll  = v0 / 32768.0f * 180.0f;
            _pitch = v1 / 32768.0f * 180.0f;
            float y = v2 / 32768.0f * 180.0f;
            
            if (y < 0) y += 360.0f; 
            if (!_have || fabsf(angleDiffDeg(y, _yaw)) <= IMU_MAX_YAW_JUMP) {
                _yaw = y;
                _tolakYaw = 0;
            } else if (++_tolakYaw >= IMU_MAX_YAW_TOLAK) {
                // Sudah sekian sampel berturut-turut jauh dari nilai lama ->
                // ini bukan spike sesaat, tapi heading yang benar-benar
                // berpindah. Terima, jangan biarkan _yaw membeku selamanya.
                _yaw = y;
                _tolakYaw = 0;
                Serial.println("Imu: yaw resinkron (lonjakan menetap diterima).");
            }
            _have = true;
            break;
        }
        
        case 0x54: // Data Magnetik Mentah
            _mhx = (float)v0; 
            _mhy = (float)v1; 
            _mhz = (float)v2;
            break;
            
        default: break; 
    }
}