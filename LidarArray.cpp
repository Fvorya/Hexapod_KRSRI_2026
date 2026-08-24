#include "LidarArray.h"

#define VL53L0X_ADDR              0x29
#define VL53L0X_REG_INT_STATUS    0x13   // RESULT_INTERRUPT_STATUS

// Diurutkan menurut CHANNEL MUX, dan isinya arah FISIK sebenarnya -- bukan
// urutan ideal. Tabel ini harus selalu sepakat dengan LIDAR_* di config.h;
// kalau tidak, perintah 'l' akan melabeli sensor dengan arah yang keliru dan
// pencarian kesalahan jadi menyesatkan.
static const char* const LIDAR_NAMA[NUM_LIDAR] = {
    "DEPAN-KI", "BLKG-KI", "BELAKANG", "BLKG-KA", "DEPAN-KA", "DEPAN"
};

// Enam LIDAR_* wajib menunjuk enam channel BERBEDA di 0..NUM_LIDAR-1. Salah
// ketik saat memetakan ulang (mis. dua arah menunjuk channel yang sama) dulu
// hanya ketahuan sebagai perilaku navigasi yang aneh di lapangan.
static_assert(NUM_LIDAR == 6, "Tabel LIDAR_NAMA & penjaga di bawah menganggap 6 sensor");
static_assert(((1u << LIDAR_FRONT) | (1u << LIDAR_FRONT_R) | (1u << LIDAR_BACK_R) |
               (1u << LIDAR_BACK)  | (1u << LIDAR_BACK_L)  | (1u << LIDAR_FRONT_L)) == 0x3Fu,
              "LIDAR_* di config.h harus enam channel BERBEDA dalam 0..5");

const char* LidarArray::nama(uint8_t id) {
    return (id < NUM_LIDAR) ? LIDAR_NAMA[id] : "?";
}

LidarArray::LidarArray() {
    for (int i = 0; i < NUM_LIDAR; i++) {
        _dist[i] = -1;
        _histN[i] = 0;
        _lastOk[i] = 0;
        _lastResp[i] = 0;
        _jauh[i] = false;
        _isReady[i] = false;
        _hist[i][0] = _hist[i][1] = _hist[i][2] = 0;
    }
    _cur = 0;
    _muxOk = false;
}

// false bila mux tidak mengakui alamatnya -- dulu hasil endTransmission()
// dibuang, sehingga mux yang kabelnya lepas terlihat sama dengan mux sehat.
bool LidarArray::selectMux(uint8_t ch) {
    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    LIDAR_I2C_BUS.write(1 << ch);
    return (LIDAR_I2C_BUS.endTransmission() == 0);
}

bool LidarArray::begin() {
    LIDAR_I2C_BUS.begin();
    LIDAR_I2C_BUS.setClock(LIDAR_I2C_CLOCK);

    // Pastikan mux-nya ada dulu. Tanpa ini, keenam init akan gagal satu per
    // satu dan pesannya menyesatkan ("sensor rusak") padahal muxnya yang mati.
    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    _muxOk = (LIDAR_I2C_BUS.endTransmission() == 0);
    if (!_muxOk) {
        Serial.print("LidarArray: mux TCA9548A @0x");
        Serial.print(I2C_MUX_ADDR, HEX);
        Serial.println(" TIDAK MENJAWAB -- LiDAR dinonaktifkan.");
        Serial.println("            Cek SDA 18 / SCL 19, catu daya mux, pull-up. Ketik 'I' untuk memindai.");
        return false;
    }

    bool ok = true;
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        if (initSensor(i)) {
            Serial.print("LidarArray: Channel "); Serial.print(i);
            Serial.print(" ("); Serial.print(LIDAR_NAMA[i]); Serial.println(") OK");
        } else {
            Serial.print("LidarArray: VL53L0X GAGAL init di channel "); Serial.println(i);
            ok = false;
        }
    }
    if (!ok)
        Serial.println("LidarArray: sesudah memperbaiki kabel/daya, ketik 'I' untuk init ulang tanpa reset.");
    return ok;
}

// Satu channel: pilih mux -> init -> konfigurasi long range -> mulai kontinu.
// Sengaja dipakai bersama begin() dan pindaiI2C(): kalau init ulang memakai
// urutan yang disalin terpisah, sensor yang dipulihkan bisa diam-diam berjalan
// dengan konfigurasi yang berbeda dari saudaranya.
bool LidarArray::initSensor(uint8_t ch) {
    if (ch >= NUM_LIDAR) return false;
    if (!selectMux(ch)) { _isReady[ch] = false; return false; }

    _sensor[ch].setBus(&LIDAR_I2C_BUS);
    _sensor[ch].setTimeout(200);

    if (!_sensor[ch].init()) { _isReady[ch] = false; return false; }

    // Konfigurasi Long Range (~2 meter)
    _sensor[ch].setSignalRateLimit(0.1);
    _sensor[ch].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    _sensor[ch].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

    // Nilai baliknya SEKARANG diperiksa. Dengan periode VCSEL yang diperpanjang
    // di atas, kebutuhan waktu minimum sensor naik dan permintaan 20 ms bisa
    // ditolak -- dulu diam-diam, sehingga kita mengira konfigurasinya terpasang
    // padahal sensor tetap memakai anggaran bawaannya. Bukan kesalahan fatal.
    if (!_sensor[ch].setMeasurementTimingBudget(20000)) {
        Serial.print("LidarArray: anggaran waktu 20 ms ditolak di channel ");
        Serial.print(ch); Serial.println(" -- sensor pakai bawaannya (tidak fatal).");
    }

    _sensor[ch].startContinuous(0); // Mode non-stop

    // Buang sisa kehidupan sebelumnya: sensor yang di-init ulang tidak boleh
    // mewarisi EMA, histori median, atau stempel waktu yang lama -- kalau tidak,
    // getDistance() sempat mengembalikan jarak basi sebagai data baru.
    _dist[ch]     = -1;
    _histN[ch]    = 0;
    _hist[ch][0]  = _hist[ch][1] = _hist[ch][2] = 0;
    _lastOk[ch]   = 0;
    _jauh[ch]     = false;

    // init() barusan melakukan puluhan baca-tulis I2C yang semuanya dijawab --
    // itu memang "sensor merespons", jadi mencatatnya di sini bukan curang.
    // Gunanya: sensor yang baru dipulihkan tidak langsung dihitung MATI pada
    // detik yang sama, sehingga menekan 'I' dua kali berturut-turut tidak
    // meng-init ulang sensor yang sebenarnya baik-baik saja. Kalau sampai
    // LIDAR_TIMEOUT_MS tak ada pengukuran juga, barulah ia jatuh ke MATI.
    _lastResp[ch] = millis();

    _isReady[ch]  = true;
    return true;
}

// Filter median 3 nilai
static int median3(int a, int b, int c) {
    if (a > b) { int t=a; a=b; b=t; }
    if (b > c) { int t=b; b=c; c=t; }
    if (a > b) { int t=a; a=b; b=t; }
    return b;
}

// State machine non-blocking menggunakan polling interupsi.
// SELALU memajukan _cur. Versi lama hanya maju bila data siap atau sudah
// timeout 300 ms, sehingga satu sensor macet menahan kelima sensor lain.
void LidarArray::update() {
    if (!_muxOk) return;

    // Cari sensor hidup berikutnya (maksimal satu putaran penuh).
    uint8_t coba = 0;
    do {
        _cur = (uint8_t)((_cur + 1) % NUM_LIDAR);
        coba++;
    } while (!_isReady[_cur] && coba < NUM_LIDAR);
    if (!_isReady[_cur]) return;      // tak ada sensor hidup sama sekali

    if (!selectMux(_cur)) return;

    // Polling bit status interupsi. Belum siap -> tinggalkan, putaran
    // berikutnya akan mampir lagi. Tidak ada busy-wait di sini.
    uint8_t status = _sensor[_cur].readReg(VL53L0X_REG_INT_STATUS);
    if ((status & 0x07) == 0) return;

    // Data siap -- readRange... sekaligus melakukan 'Clear Interrupt'
    uint16_t mm = _sensor[_cur].readRangeContinuousMillimeters();
    bool timeout = _sensor[_cur].timeoutOccurred();

    // HANYA ini yang berarti "tidak menjawab". Pustaka Pololu mengembalikan
    // 8190 mm untuk "tak ada target dalam jangkauan" -- itu jawaban SAH dari
    // sensor yang SEHAT, bukan omong kosong.
    //
    // Versi sebelumnya membuang mm >= 8000 di baris yang sama dengan timeout,
    // yaitu SEBELUM _lastResp sempat disegarkan. Akibatnya sensor yang
    // menghadap ruang terbuka dinyatakan MATI begitu lewat LIDAR_TIMEOUT_MS,
    // dan navigasi berhenti dengan "sensor tidak merespons" -- justru
    // kebingungan yang mau dihapus oleh pembedaan tiga keadaan. Cabang
    // cm > LIDAR_MAX_CM pun praktis tak pernah tercapai, karena VL53L0X
    // melompat langsung dari jarak terukur ke 8190.
    if (timeout || mm == 0xFFFF) return;

    // Sensor MENJAWAB -> jelas SEHAT, berapa pun angkanya.
    _lastResp[_cur] = millis();

    int cm = (int)((mm + 5) / 10);      // dibulatkan, bukan dipotong

    if (mm >= 8000 || cm > LIDAR_MAX_CM) {
        // Sah, tapi di luar jangkauan pakai. Bedakan dari sensor mati.
        _jauh[_cur] = true;
        _histN[_cur] = 0;               // jangan campur histori dekat & jauh
        return;
    }

    _jauh[_cur] = false;

    // 1) Histori 3-tap untuk median (buang spike/outlier sesaat)
    _hist[_cur][2] = _hist[_cur][1];
    _hist[_cur][1] = _hist[_cur][0];
    _hist[_cur][0] = cm;
    if (_histN[_cur] < 3) _histN[_cur]++;
    int m = (_histN[_cur] < 3) ? cm : median3(_hist[_cur][0], _hist[_cur][1], _hist[_cur][2]);

    // 2) EMA (Exponential Moving Average) untuk menghaluskan data
    _dist[_cur] = (_dist[_cur] < 0) ? m
                : (1.0f - LIDAR_EMA_ALPHA) * _dist[_cur] + LIDAR_EMA_ALPHA * m;
    _lastOk[_cur] = millis();
}

bool LidarArray::sensorHidup(uint8_t id) {
    if (id >= NUM_LIDAR || !_isReady[id]) return false;
    if (_lastResp[id] == 0) return false;                       // belum pernah menjawab
    return (millis() - _lastResp[id] <= LIDAR_TIMEOUT_MS);
}

int LidarArray::getDistance(uint8_t id) {
    if (id >= NUM_LIDAR) return LIDAR_MATI;
    if (!sensorHidup(id)) return LIDAR_MATI;    // benar-benar tidak merespons
    if (_jauh[id]) return LIDAR_JAUH;           // sehat, tapi tak ada objek dekat
    if (_dist[id] < 0) return LIDAR_JAUH;       // menjawab tapi belum pernah dekat
    if (millis() - _lastOk[id] > LIDAR_TIMEOUT_MS) return LIDAR_JAUH;
    return (int)(_dist[id] + 0.5f);
}

// Nilai EMA apa adanya (cm), tanpa pembulatan. Syarat keabsahannya sama persis
// dengan getDistance() supaya keduanya tidak pernah berbeda pendapat.
float LidarArray::jarakHalus(uint8_t id) {
    if (id >= NUM_LIDAR)   return -1.0f;
    if (!sensorHidup(id))  return -1.0f;
    if (_jauh[id])         return -1.0f;
    if (_dist[id] < 0)     return -1.0f;
    if (millis() - _lastOk[id] > LIDAR_TIMEOUT_MS) return -1.0f;
    return _dist[id];
}

uint8_t LidarArray::jumlahHidup() {
    uint8_t n = 0;
    for (uint8_t i = 0; i < NUM_LIDAR; i++) if (sensorHidup(i)) n++;
    return n;
}

void LidarArray::cetakTabel() {
    Serial.println("\n--- LIDAR ---");
    if (!_muxOk) { Serial.println("  mux TIDAK terdeteksi -- ketik 'I' untuk memindai."); return; }
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        Serial.print("  "); Serial.print(i); Serial.print(' ');
        Serial.print(LIDAR_NAMA[i]);
        for (uint8_t k = strlen(LIDAR_NAMA[i]); k < 10; k++) Serial.print(' ');
        Serial.print(": ");
        if (!_isReady[i]) { Serial.println("tidak di-init saat begin()"); continue; }
        int d = getDistance(i);
        if (d == LIDAR_MATI) {
            Serial.println("MATI -- tidak merespons");
        } else if (d == LIDAR_JAUH) {
            Serial.print("jauh (di atas "); Serial.print(LIDAR_MAX_CM); Serial.println(" cm)");
        } else {
            Serial.print(d); Serial.println(" cm");
        }
    }
    Serial.print("  sensor hidup: "); Serial.print(jumlahHidup());
    Serial.print(" dari "); Serial.println(NUM_LIDAR);
}

void LidarArray::cetakBaris() {
    if (!_muxOk) { Serial.println("lidar: mux tidak terdeteksi"); return; }
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        Serial.print(LIDAR_NAMA[i]); Serial.print(' ');
        int d = getDistance(i);
        if      (d == LIDAR_MATI) Serial.print("---");
        else if (d == LIDAR_JAUH) Serial.print(">MAX");
        else                    { Serial.print(d); Serial.print("cm"); }
        if (i < NUM_LIDAR - 1) Serial.print(" | ");
    }
    Serial.println();
}

// Pemindai I2C: memastikan mux dan keenam sensor benar-benar ada SEBELUM
// menyalahkan program -- SEKALIGUS meng-init ulang yang ada tapi belum aktif.
// Dijalankan lewat perintah 'I'.
void LidarArray::pindaiI2C() {
    Serial.println("\n--- PINDAI I2C (bus LiDAR, SDA 18 / SCL 19) ---");

    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    bool mux = (LIDAR_I2C_BUS.endTransmission() == 0);
    Serial.print("  TCA9548A @0x"); Serial.print(I2C_MUX_ADDR, HEX);
    Serial.println(mux ? " : TERDETEKSI" : " : TIDAK MENJAWAB");

    // Pindaian ini lebih baru daripada catatan begin(). Dulu _muxOk hanya
    // pernah diisi saat boot, jadi mux yang baru disambungkan tetap dianggap
    // hilang selamanya -- dan navMulai() menolak jalan tanpa sebab yang terlihat.
    _muxOk = mux;

    if (!mux) {
        Serial.println("  -> cek SDA 18 / SCL 19, catu daya mux, resistor pull-up.");
        Serial.println("  -> JANGAN satukan bus ini dengan PCA9685: alamat ALL-CALL-nya");
        Serial.println("     juga 0x70 dan akan bentrok dengan mux.");
        return;
    }

    uint8_t ada = 0, pulih = 0, gagal = 0;
    for (uint8_t ch = 0; ch < NUM_LIDAR; ch++) {
        Serial.print("  ch"); Serial.print(ch); Serial.print(" (");
        Serial.print(LIDAR_NAMA[ch]);
        for (uint8_t k = strlen(LIDAR_NAMA[ch]); k < 9; k++) Serial.print(' ');
        Serial.print(") : ");

        if (!selectMux(ch)) { Serial.println("gagal memilih channel"); continue; }

        LIDAR_I2C_BUS.beginTransmission(VL53L0X_ADDR);
        if (LIDAR_I2C_BUS.endTransmission() != 0) {
            Serial.println("KOSONG (tidak ada VL53L0X)");
            // Tidak mengakui alamatnya = benar-benar tidak ada di bus. Sensor
            // yang sedang mengukur pun tetap meng-ACK, jadi ini bukan "sibuk".
            // Ditandai mati supaya round-robin berhenti membuang giliran padanya;
            // 'I' berikutnya akan memulihkannya kalau kabelnya sudah benar.
            _isReady[ch] = false;
            continue;
        }
        ada++;

        // Sehat = sudah di-init DAN masih mengirim data. Dua-duanya harus
        // diperiksa: sensor yang lolos init lalu berhenti mengirim (baris
        // "MATI" di 'l') tetap ber-_isReady true, jadi kalau hanya _isReady
        // yang dilihat, justru kasus itu yang tidak pernah ikut dipulihkan.
        if (_isReady[ch] && sensorHidup(ch)) {
            Serial.println("VL53L0X @0x29 ADA, sudah aktif");
            continue;
        }

        // Ada di bus tapi tidak bekerja -> inilah yang dulu butuh reset papan.
        bool tadinyaIniet = _isReady[ch];
        if (initSensor(ch)) {
            Serial.print("VL53L0X @0x29 ADA -> INIT ULANG BERHASIL");
            Serial.println(tadinyaIniet ? " (tadinya MATI)" : " (tadinya gagal init)");
            pulih++;
        } else {
            Serial.println("VL53L0X @0x29 ADA tapi INIT ULANG GAGAL");
            gagal++;
        }
    }

    Serial.print("  total terdeteksi: "); Serial.print(ada);
    Serial.print(" dari "); Serial.println(NUM_LIDAR);

    if (pulih) {
        Serial.print("  "); Serial.print(pulih);
        Serial.println(" sensor dipulihkan tanpa reset papan. Ketik 'l' untuk memastikan.");
    }
    if (gagal) {
        Serial.print("  "); Serial.print(gagal);
        Serial.println(" sensor menjawab di bus tapi menolak init -- curigai daya/kabel, bukan program.");
    }
}
