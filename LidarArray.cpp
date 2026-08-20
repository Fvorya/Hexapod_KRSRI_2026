#include "LidarArray.h"

LidarArray::LidarArray() {
    for (int i = 0; i < NUM_LIDAR; i++) { 
        _dist[i] = -1; 
        _histN[i] = 0; 
        _lastOk[i] = 0; 
        _isReady[i] = false;
    }
    _cur = 0;
}

void LidarArray::selectMux(uint8_t ch) {
    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    LIDAR_I2C_BUS.write(1 << ch);
    LIDAR_I2C_BUS.endTransmission();
}

bool LidarArray::begin() {
    LIDAR_I2C_BUS.begin();
    LIDAR_I2C_BUS.setClock(LIDAR_I2C_CLOCK);
    bool ok = true;
    
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        selectMux(i);
        _sensor[i].setBus(&LIDAR_I2C_BUS);
        _sensor[i].setTimeout(200);

        if (!_sensor[i].init()) {
            Serial.print("LidarArray: VL53L0X GAGAL init di channel "); Serial.println(i);
            ok = false;
            _isReady[i] = false;
        } else {
            // Konfigurasi Long Range (~2 meter)
            _sensor[i].setSignalRateLimit(0.1);
            _sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
            _sensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
            _sensor[i].setMeasurementTimingBudget(20000); // 20 ms
            _sensor[i].startContinuous(0); // Mode non-stop
            _isReady[i] = true;
            Serial.print("LidarArray: Channel "); Serial.print(i); Serial.println(" OK");
        }
    }
    return ok;
}

// Filter median 3 nilai
static int median3(int a, int b, int c) {
    if (a > b) { int t=a; a=b; b=t; }
    if (b > c) { int t=b; b=c; c=t; }
    if (a > b) { int t=a; a=b; b=t; }
    return b;
}

// State machine non-blocking menggunakan polling interupsi
void LidarArray::update() {
    // Lewati channel yang sensornya memang mati/rusak sejak awal
    if (!_isReady[_cur]) {
        _cur = (_cur + 1) % NUM_LIDAR;
        return;
    }

    selectMux(_cur);

    // Polling bit status interupsi (Register 0x13)
    // Jika nilainya bukan 0, berarti data jarak sudah selesai diukur.
    uint8_t status = _sensor[_cur].readReg(0x13);

    if ((status & 0x07) != 0) {
        // Data siap! Fungsi ini akan membaca jarak sekaligus melakukan 'Clear Interrupt' otomatis
        uint16_t mm = _sensor[_cur].readRangeContinuousMillimeters();
        bool timeout = _sensor[_cur].timeoutOccurred();

        int cm = mm / 10;
        
        // Validasi: Tidak timeout, bukan 0xFFFF, dan angka masuk akal VL53L0X (< 8000)
        if (!timeout && mm != 0xFFFF && mm < 8000 && cm <= LIDAR_MAX_CM) {
            
            // 1) Histori 3-tap untuk median (buang spike/outlier sesaat)
            _hist[_cur][2] = _hist[_cur][1];
            _hist[_cur][1] = _hist[_cur][0];
            _hist[_cur][0] = cm;
            
            if (_histN[_cur] < 3) _histN[_cur]++;
            int m = (_histN[_cur] < 3) ? cm : median3(_hist[_cur][0], _hist[_cur][1], _hist[_cur][2]);
            
            // 2) EMA (Exponential Moving Average) untuk menghaluskan data
            _dist[_cur] = (_dist[_cur] < 0) ? m : (1.0f - LIDAR_EMA_ALPHA) * _dist[_cur] + LIDAR_EMA_ALPHA * m;
            _lastOk[_cur] = millis();
        }
        
        // Lanjut ke sensor berikutnya karena sensor ini sudah dibaca
        _cur = (_cur + 1) % NUM_LIDAR;
        
    } else if (millis() - _lastOk[_cur] > LIDAR_TIMEOUT_MS) {
        // Fail-safe: Jika sensor macet (tidak pernah memicu interupsi terlalu lama), 
        // jangan biarkan sistem terjebak. Paksa geser ke sensor berikutnya.
        _cur = (_cur + 1) % NUM_LIDAR;
    }
}

int LidarArray::getDistance(uint8_t id) {
    if (id >= NUM_LIDAR) return -1;
    if (_dist[id] < 0) return -1;
    if (millis() - _lastOk[id] > LIDAR_TIMEOUT_MS) return -1;  // Sensor diam -> kembalikan error
    return (int)(_dist[id] + 0.5f);
}