#include "LidarArray.h"

#define VL53L1X_ADDR              0x29   // sama dengan VL53L0X -- wiring mux tak berubah

// Anggaran waktu minimum ditentukan mode: 20 ms hanya sah di Short, Medium dan
// Long menuntut 33 ms. Dijaga saat kompilasi supaya salah pasang tidak berakhir
// sebagai setMeasurementTimingBudget() yang ditolak diam-diam di lapangan.
#if LIDAR_MODE == LIDAR_MODE_SHORT
  static_assert(LIDAR_BUDGET_US >= 20000, "Mode Short butuh anggaran waktu >= 20000 us");
#else
  static_assert(LIDAR_BUDGET_US >= 33000, "Mode Medium/Long butuh anggaran waktu >= 33000 us");
#endif
static_assert(LIDAR_PERIOD_MS * 1000L >= (long)LIDAR_BUDGET_US,
              "LIDAR_PERIOD_MS harus >= anggaran waktu -- sensor tak bisa mengukur lebih cepat dari itu");

// Diurutkan menurut CHANNEL MUX, dan isinya arah FISIK sebenarnya -- bukan
// urutan ideal. Tabel ini harus selalu sepakat dengan LIDAR_* di config.h;
// kalau tidak, perintah 'l' akan melabeli sensor dengan arah yang keliru dan
// pencarian kesalahan jadi menyesatkan.
static const char* const LIDAR_NAMA[NUM_LIDAR] = {
    "KIRI-DPN", "KIRI-BLK", "BELAKANG", "KANAN-BLK", "KANAN-DPN", "DEPAN"
};

// Enam LIDAR_* wajib menunjuk enam channel BERBEDA di 0..NUM_LIDAR-1. Salah
// ketik saat memetakan ulang (mis. dua arah menunjuk channel yang sama) dulu
// hanya ketahuan sebagai perilaku navigasi yang aneh di lapangan.
static_assert(NUM_LIDAR == 6, "Tabel LIDAR_NAMA & penjaga di bawah menganggap 6 sensor");
static_assert(((1u << LIDAR_FRONT) | (1u << LIDAR_KANAN_D) | (1u << LIDAR_KANAN_B) |
               (1u << LIDAR_BACK)  | (1u << LIDAR_KIRI_B)  | (1u << LIDAR_KIRI_D)) == 0x3Fu,
              "LIDAR_* di config.h harus enam channel BERBEDA dalam 0..5");

const char* LidarArray::nama(uint8_t id) {
    return (id < NUM_LIDAR) ? LIDAR_NAMA[id] : "?";
}

LidarArray::LidarArray() {
    for (int i = 0; i < NUM_LIDAR; i++) {
        _dist[i] = -1;
        _histN[i] = 0;
        _pendekN[i] = 0;
        _lastOk[i] = 0;
        _lastResp[i] = 0;
        _jauh[i] = false;
        _isReady[i] = false;
        _mmAkhir[i] = 0;
        _statusAkhir[i] = (uint8_t)VL53L1X::None;
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

// PEMULIHAN BUS I2C, sembilan pulsa clock.
//
// Slave yang tersela di TENGAH transaksi -- master di-reset, Teensy di-flash,
// catu kedip -- boleh menahan SDA RENDAH sambil menunggu clock yang tidak
// pernah datang. Sesudah itu tidak satu alamat pun menjawab, termasuk mux,
// dan gejalanya persis "kemarin jalan, hari ini tidak" tanpa ada yang
// disentuh.
//
// YANG MEMBUATNYA SULIT DIKENALI: reset Teensy tidak menyembuhkannya. Yang
// menahan SDA itu SLAVE, dan slave tidak ikut mati saat Teensy di-flash. Jadi
// flash ulang berkali-kali memberi hasil yang sama dan menunjuk ke arah yang
// salah -- seolah program atau solderan yang rusak.
//
// Yang melepaskannya cuma dua: cabut catu slave, atau beri sembilan pulsa
// clock supaya ia menyelesaikan byte yang tergantung lalu melepas SDA.
// Sembilan karena satu byte itu 8 bit + 1 ACK.
// BACA GARIS HANYA DI SINI, sebelum Wire.begin(). Sesudah Wire.begin() pin
// 18/19 dipindah ke peripheral LPI2C, dan digitalRead() membaca register GPIO
// yang tidak lagi mengikuti keadaan pad -- hasilnya bisa RENDAH terus tanpa
// ada yang menarik apa pun. Bacaan di bawah dipercaya; bacaan sesudah
// Wire.begin() tidak.
static bool bebaskanBus(uint8_t sda, uint8_t scl) {
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(10);
    const bool sdaAwal = digitalRead(sda);
    const bool sclAwal = digitalRead(scl);

    Serial.print("LidarArray: garis (mode GPIO, sebelum Wire.begin) -- SDA ");
    Serial.print(sdaAwal ? "TINGGI" : "RENDAH");
    Serial.print(", SCL ");
    Serial.println(sclAwal ? "TINGGI" : "RENDAH");

    // SCL RENDAH memisahkan dua dunia, dan pemisahan itu yang paling berharga
    // di sini. Slave I2C boleh menahan SDA selamanya, tapi ia TIDAK PERNAH
    // menahan SCL selain sebagai clock-stretch sesaat -- dan tidak ada clock
    // untuk di-stretch sebelum master menyala. Jadi SCL rendah di titik ini
    // bukan perangkat yang menggantung: itu hubung singkat, pull-up yang
    // hilang, atau pin yang rusak. Sembilan pulsa clock tidak akan menolong.
    if (!sclAwal) {
        Serial.println("            SCL rendah TANPA master aktif -- ini BUKAN slave");
        Serial.println("            menggantung. Cabut catu tidak akan menyembuhkannya.");
        Serial.println("            Ukur ohm SDA-GND dan SCL-GND dengan catu MATI.");
    }

    if (sdaAwal) return true;      // tidak tersangkut, tidak usah diapa-apakan

    for (uint8_t i = 0; i < 9 && !digitalRead(sda); i++) {
        // Open-drain ditiru dengan bertukar mode: OUTPUT LOW menarik, dan
        // INPUT_PULLUP melepas. JANGAN pakai OUTPUT HIGH -- kalau slave masih
        // menahan garisnya, itu hubung singkat.
        pinMode(scl, OUTPUT);
        digitalWrite(scl, LOW);
        delayMicroseconds(5);
        pinMode(scl, INPUT_PULLUP);
        delayMicroseconds(5);
    }

    // STOP: SDA naik selagi SCL tinggi. Tanpa ini slave berhenti di tengah
    // bingkai dan byte berikutnya dibaca sebagai lanjutan, bukan alamat baru.
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(5);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(sda, INPUT_PULLUP);
    delayMicroseconds(5);

    return digitalRead(sda);
}

// UJI PIN dengan pull-up lalu pull-down, tanpa menyentuh peripheral I2C.
//
// Gunanya memisahkan "pin ini tertarik ke GND" dari "pin ini tidak menjawab".
// Pull-up internal 22k melawan apa pun yang menarik garis; pull-down internal
// melakukan kebalikannya. Pin yang bebas menuruti keduanya:
//
//     pull-up TINGGI, pull-down RENDAH   pin SEHAT, mengambang bebas
//     pull-up RENDAH, pull-down RENDAH   TERTARIK KE GND, di bawah ~9,5k
//     pull-up TINGGI, pull-down TINGGI   TERTARIK KE 3V3
//
// SCL dipakai sebagai pembanding, bukan pin bebas yang ditebak: ia di papan
// yang sama, jalur yang sama panjangnya, dan sudah terbukti terbaca TINGGI.
// Kalau SCL lulus dan SDA tidak, selisihnya bukan soal metode.
//
// TIDAK ada pin yang di-drive di sini. Pin yang mungkin terhubung GND tidak
// boleh disuruh mengeluarkan arus -- itu menambah kerusakan pada pin yang
// justru sedang diperiksa.
void LidarArray::periksaPinBus() {
    Serial.println("\n--- UJI PIN BUS (GPIO murni, peripheral I2C tidak dipakai) ---");
    LIDAR_I2C_BUS.end();      // lepas pin dari LPI2C supaya GPIO benar-benar berlaku

    const uint8_t pin[2]  = { LIDAR_I2C_SDA, LIDAR_I2C_SCL };
    const char*   nama[2] = { "SDA", "SCL" };

    for (uint8_t i = 0; i < 2; i++) {
        pinMode(pin[i], INPUT_PULLUP);
        delayMicroseconds(200);         // longgar: kapasitansi jalur + 22k
        const bool naik = digitalRead(pin[i]);

        pinMode(pin[i], INPUT_PULLDOWN);
        delayMicroseconds(200);
        const bool turun = digitalRead(pin[i]);

        pinMode(pin[i], INPUT_PULLUP);  // tinggalkan dalam keadaan aman

        Serial.print("  "); Serial.print(nama[i]);
        Serial.print(" (pin "); Serial.print(pin[i]); Serial.print(")  pull-up ");
        Serial.print(naik ? "TINGGI" : "RENDAH ");
        Serial.print("  pull-down ");
        Serial.print(turun ? "TINGGI" : "RENDAH ");
        Serial.print("   -> ");
        if (naik && !turun)       Serial.println("SEHAT, mengambang bebas");
        else if (!naik && !turun) Serial.println("TERTARIK KE GND (di bawah ~9,5k)");
        else if (naik && turun)   Serial.println("TERTARIK KE 3V3");
        else                      Serial.println("aneh -- ulangi uji ini");
    }

    Serial.println("  Kalau SCL SEHAT dan SDA TERTARIK KE GND sementara kabel SDA");
    Serial.println("  sudah dicabut, yang tersisa cuma pin 18 Teensy itu sendiri.");
    Serial.println("  Bukti terakhirnya: jalankan Teensy SENDIRIAN, lepas dari board,");
    Serial.println("  hanya dengan USB, lalu baca baris garis saat boot.");

    LIDAR_I2C_BUS.begin();
    LIDAR_I2C_BUS.setClock(LIDAR_I2C_CLOCK);
}

bool LidarArray::begin() {
    // SEBELUM Wire.begin(). Sesudahnya pin sudah dipegang peripheral I2C dan
    // tidak bisa digoyang sebagai GPIO.
    if (!bebaskanBus(LIDAR_I2C_SDA, LIDAR_I2C_SCL)) {
        Serial.println("LidarArray: SDA MASIH TERTAHAN RENDAH sesudah 9 pulsa clock.");
        Serial.println("            Bukan program: ada yang menarik garisnya terus.");
        Serial.println("            Cabut catu mux+LiDAR sebentar, lalu nyalakan lagi.");
    }

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
            Serial.print("LidarArray: VL53L1X GAGAL init di channel "); Serial.println(i);
            ok = false;
        }
    }
    if (!ok)
        Serial.println("LidarArray: sesudah memperbaiki kabel/daya, ketik 'I' untuk init ulang tanpa reset.");
    return ok;
}

// Satu channel: pilih mux -> init -> mode jarak -> anggaran waktu -> kontinu.
// Sengaja dipakai bersama begin() dan pindaiI2C(): kalau init ulang memakai
// urutan yang disalin terpisah, sensor yang dipulihkan bisa diam-diam berjalan
// dengan konfigurasi yang berbeda dari saudaranya.
bool LidarArray::initSensor(uint8_t ch) {
    if (ch >= NUM_LIDAR) return false;
    if (!selectMux(ch)) { _isReady[ch] = false; return false; }

    _sensor[ch].setBus(&LIDAR_I2C_BUS);
    _sensor[ch].setTimeout(200);

    if (!_sensor[ch].init()) { _isReady[ch] = false; return false; }

    // Mode jarak menggantikan setSignalRateLimit + setVcselPulsePeriod milik
    // VL53L0X -- ketiganya TIDAK ADA di pustaka VL53L1X.
#if   LIDAR_MODE == LIDAR_MODE_SHORT
    _sensor[ch].setDistanceMode(VL53L1X::Short);
#elif LIDAR_MODE == LIDAR_MODE_MEDIUM
    _sensor[ch].setDistanceMode(VL53L1X::Medium);
#else
    _sensor[ch].setDistanceMode(VL53L1X::Long);
#endif

#if LIDAR_ROI_SEMPIT
    _sensor[ch].setROISize(4, 4);   // ~27 der -> ~15 der, mengurangi crosstalk
#endif

    // Nilai baliknya diperiksa: permintaan di bawah minimum mode ditolak
    // pustaka, dan dulu ditolak DIAM-DIAM sehingga kita mengira konfigurasinya
    // terpasang padahal sensor tetap memakai bawaannya. Bukan kesalahan fatal.
    if (!_sensor[ch].setMeasurementTimingBudget(LIDAR_BUDGET_US)) {
        Serial.print("LidarArray: anggaran waktu ");
        Serial.print(LIDAR_BUDGET_US / 1000);
        Serial.print(" ms ditolak di channel "); Serial.print(ch);
        Serial.println(" -- sensor pakai bawaannya (tidak fatal).");
    }

    // BEDA PENTING dari VL53L0X: argumennya bukan "0 = beruntun secepatnya",
    // melainkan JEDA ANTAR PENGUKURAN dalam ms, ditulis langsung ke register.
    // Nilai 0 tidak divalidasi pustaka dan bukan yang kita inginkan.
    _sensor[ch].startContinuous(LIDAR_PERIOD_MS);

    // Buang sisa kehidupan sebelumnya: sensor yang di-init ulang tidak boleh
    // mewarisi EMA, histori median, atau stempel waktu yang lama -- kalau tidak,
    // getDistance() sempat mengembalikan jarak basi sebagai data baru.
    _dist[ch]     = -1;
    _histN[ch]    = 0;
    _pendekN[ch]  = 0;
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
    // Jendela jejak habis -> cetak ringkasannya. Ditaruh di sini supaya
    // pengumpulannya benar-benar non-blokir: loop utama tetap jalan penuh.
    if (_jejakSampai && (int32_t)(millis() - _jejakSampai) >= 0) {
        _jejakSampai = 0;
        jejakCetak();
    }

    // Peralihan fase uji isolasi.
    if (_isoFase && (int32_t)(millis() - _isoSampai) >= 0) {
        if (_isoFase == 1) {
            isolasiLainnya(false);          // matikan kelima sensor lain
            isolasiNolkan(1);
            _isoFase = 2;
            _isoSampai = millis() + (uint32_t)_isoDetik * 1000UL;
            Serial.println("  ... kelima sensor lain DIMATIKAN, mengukur lagi.");
        } else {
            isolasiLainnya(true);           // hidupkan lagi
            _isoFase = 0;
            if (_isoSemua) {
                _isoRata[_isoCh][0] = _isoN[0] ? (int16_t)(_isoJumlah[0] / _isoN[0]) : -1;
                _isoRata[_isoCh][1] = _isoN[1] ? (int16_t)(_isoJumlah[1] / _isoN[1]) : -1;
                if (!isolasiLanjut()) isolasiTabel();
            } else {
                isolasiCetak();
            }
        }
    }

    if (!_muxOk) return;

    // Cari sensor hidup berikutnya (maksimal satu putaran penuh).
    uint8_t coba = 0;
    do {
        _cur = (uint8_t)((_cur + 1) % NUM_LIDAR);
        coba++;
    } while (!_isReady[_cur] && coba < NUM_LIDAR);
    if (!_isReady[_cur]) return;      // tak ada sensor hidup sama sekali

    if (!selectMux(_cur)) return;

    // Pustaka VL53L1X menyediakan dataReady() publik, jadi tidak perlu lagi
    // mengintip register mentah -- register 0x13 milik VL53L0X tak ada artinya
    // di sini (peta registernya beralamat 16-bit dan berbeda total).
    // Cek ini WAJIB: read(false) yang dipanggil saat data belum siap akan
    // mengembalikan hasil pengukuran SEBELUMNYA, bukan menunggu.
    if (!_sensor[_cur].dataReady()) return;

    // false = jangan memblokir. read() juga membersihkan interupsi, sama
    // seperti readRangeContinuousMillimeters() di VL53L0X.
    uint16_t mm = _sensor[_cur].read(false);
    if (_sensor[_cur].timeoutOccurred()) return;   // benar-benar tidak menjawab

    // Rekam jawaban MENTAH sebelum disaring apa pun. Inilah yang membedakan
    // "sensor melihat hantu" dari "kode salah menyaring" saat mencari sebab
    // bacaan pendek yang tak masuk akal -- dicetak oleh perintah 'l'.
    _mmAkhir[_cur]     = mm;
    _statusAkhir[_cur] = (uint8_t)_sensor[_cur].ranging_data.range_status;
    if (_jejakSampai) jejakCatat(_cur, mm, _statusAkhir[_cur]);
    if (_isoFase)     isolasiCatat(_cur, mm, _statusAkhir[_cur]);

    // ---------------------------------------------------------------------
    // VL53L1X TIDAK punya sentinel seperti 8190 mm milik VL53L0X: range_mm
    // SELALU berisi angka hasil hitungan, sah atau tidak. Satu-satunya sumber
    // kebenaran adalah range_status. Menyaring lewat ambang jarak seperti kode
    // VL53L0X akan menerima angka sampah sebagai jarak sah -- robot mengira
    // ada dinding di tempat yang kosong.
    //
    // Statusnya dipilah jadi TIGA golongan, dan _lastResp sengaja hanya
    // disegarkan untuk dua yang pertama:
    //   sah          -> pakai jaraknya
    //   tak ada target -> sensor SEHAT, tandai jauh  (segarkan _lastResp)
    //   pengukuran buruk -> JANGAN disegarkan, supaya sensor yang terus-menerus
    //                     menghasilkan sampah jatuh ke MATI sesudah
    //                     LIDAR_TIMEOUT_MS dan navigasi berhenti. Kalau ia
    //                     disegarkan juga, ia akan tampak "jauh" selamanya --
    //                     yaitu "tidak ada halangan" -- justru saat sensornya
    //                     tidak bisa dipercaya.
    // ---------------------------------------------------------------------
    switch (_sensor[_cur].ranging_data.range_status) {
        case VL53L1X::RangeValid:
        case VL53L1X::RangeValidNoWrapCheckFail:
        case VL53L1X::RangeValidMinRangeClipped:
            // MinRangeClipped = objek SANGAT dekat dan angkanya dipangkas ke
            // batas bawah. Wajib diterima: menolaknya membuat objek yang
            // paling dekat justru dilaporkan "jauh".
            _lastResp[_cur] = millis();
            break;

        case VL53L1X::SignalFail:
        case VL53L1X::OutOfBoundsFail:
        case VL53L1X::WrapTargetFail:
            // Sensor sehat, memang tak ada target dalam jangkauan.
            _lastResp[_cur] = millis();
            _jauh[_cur]  = true;
            _histN[_cur] = 0;           // jangan campur histori dekat & jauh
            return;

        default:
            // SigmaFail, XtalkSignalFail, MinRangeFail, HardwareFail, dll.
            return;
    }

    int cm = (int)((mm + 5) / 10);      // dibulatkan, bukan dipotong

    if (cm > LIDAR_MAX_CM) {
        // Sah, tapi di luar jangkauan pakai. Bedakan dari sensor mati.
        _jauh[_cur] = true;
        _histN[_cur] = 0;
        return;
    }

    // BATAS BAWAH GEOMETRI. Sensor robot ini melaporkan "tak ada objek dalam
    // jangkauan" sebagai bacaan pendek ~5 cm berstatus RangeValid, BUKAN
    // sebagai SignalFail. Padahal benda sungguhan tidak bisa berada sedekat
    // itu -- kaki sudah menabraknya lebih dulu (lihat LIDAR_MIN_CM di
    // config.h). Jadi bacaan di bawah batas ini berarti KOSONG, bukan
    // "halangan mepet". Keduanya menuntut reaksi yang berlawanan.
    //
    // Ini memperbaiki gejala nyata: saat lorong di depan terbuka, sensor depan
    // melaporkan 5 cm, navigasi membacanya sebagai "halangan <= FRONT_STOP_CM",
    // lalu memutar menjauhi dinding yang sedang diikuti -- robot berbelok
    // sendiri di lorong yang justru kosong.
    //
    // Gerbangnya sama dengan gerbang median di bawah: butuh TIGA sampel
    // berturut-turut sebelum keadaan "jauh" diumumkan, supaya satu bacaan
    // pendek yang menyimpang saat dinding sungguhan sedang terlihat tidak
    // membalik sensor jadi "kosong" -- arah kesalahan yang paling berbahaya
    // untuk sensor depan.
    if (cm < (int)LIDAR_MIN_CM[_cur]) {
        _histN[_cur] = 0;
        if (_pendekN[_cur] < 3) _pendekN[_cur]++;
        if (_pendekN[_cur] >= 3) _jauh[_cur] = true;
        return;
    }
    _pendekN[_cur] = 0;

    // 1) Histori 3-tap untuk median (buang spike/outlier sesaat)
    _hist[_cur][2] = _hist[_cur][1];
    _hist[_cur][1] = _hist[_cur][0];
    _hist[_cur][0] = cm;
    if (_histN[_cur] < 3) _histN[_cur]++;

    // Median-3 baru berarti kalau histori benar-benar berisi tiga sampel, dan
    // histori itu DINOLKAN tiap kali sensor melapor "jauh" (supaya jarak dekat
    // dan jauh tidak tercampur). Akibatnya, sampel pertama sesudah keadaan
    // "jauh" dulu melewati median MENTAH-MENTAH -- dan barisnya juga langsung
    // menulis _jauh = false. Jadi SATU pembacaan pendek yang menyimpang
    // (crosstalk dari kaca penutup, pantulan beraliasi, badan robot terserempet
    // bidang pandang) cukup untuk membalik sensor dari "tak ada objek" menjadi
    // "ada dinding 8 cm" -- persis gejala "sering membaca 5-10 cm padahal
    // kosong". Filter yang dipasang untuk membuang spike justru dilucuti tepat
    // pada saat ia paling dibutuhkan.
    //
    // Sekarang keadaan "jauh" hanya ditinggalkan sesudah TIGA sampel dalam
    // jangkauan berturut-turut. Ongkosnya 3 x LIDAR_PERIOD_MS (~75 ms); pada
    // 5,8 cm/detik robot hanya maju 0,4 mm, jadi tidak ada bedanya untuk
    // penghindaran halangan.
    if (_histN[_cur] < 3) return;

    int m = median3(_hist[_cur][0], _hist[_cur][1], _hist[_cur][2]);

    _jauh[_cur] = false;

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

void LidarArray::jejakMulai(int8_t ch, uint16_t detik) {
    _jejakCh = (ch >= 0 && ch < NUM_LIDAR) ? ch : -1;
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        for (uint8_t k = 0; k < 12; k++) _jHist[i][k] = 0;
        _jMin[i] = 0xFFFF; _jMax[i] = 0; _jJumlah[i] = 0; _jN[i] = 0;
    }
    _jejakSampai = millis() + (uint32_t)detik * 1000UL;
    if (_jejakSampai == 0) _jejakSampai = 1;      // 0 dipakai sbg "mati"

    Serial.print("Jejak LiDAR ");
    if (_jejakCh < 0) Serial.print("SEMUA channel");
    else { Serial.print("channel "); Serial.print(_jejakCh);
           Serial.print(" ("); Serial.print(LIDAR_NAMA[_jejakCh]); Serial.print(")"); }
    Serial.print(" selama "); Serial.print(detik);
    Serial.println(" detik. Arahkan ke ruang KOSONG dan jangan disentuh.");
}

void LidarArray::jejakCatat(uint8_t ch, uint16_t mm, uint8_t status) {
    if (_jejakCh >= 0 && ch != (uint8_t)_jejakCh) return;
    if (status < 12) _jHist[ch][status]++;
    // Hanya golongan "sah" yang masuk statistik jarak -- angka pada status
    // lain tidak berarti apa-apa (lihat catatan range_status di atas).
    if (status == VL53L1X::RangeValid ||
        status == VL53L1X::RangeValidNoWrapCheckFail ||
        status == VL53L1X::RangeValidMinRangeClipped) {
        if (mm < _jMin[ch]) _jMin[ch] = mm;
        if (mm > _jMax[ch]) _jMax[ch] = mm;
        _jJumlah[ch] += mm; _jN[ch]++;
    }
}

void LidarArray::jejakCetak() {
    Serial.println("\n--- JEJAK LIDAR ---");
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        if (_jejakCh >= 0 && i != (uint8_t)_jejakCh) continue;
        uint16_t total = 0;
        for (uint8_t k = 0; k < 12; k++) total += _jHist[i][k];
        Serial.print("  ch"); Serial.print(i); Serial.print(" ");
        Serial.print(LIDAR_NAMA[i]);
        for (uint8_t k = strlen(LIDAR_NAMA[i]); k < 10; k++) Serial.print(' ');
        if (!total) { Serial.println(": tidak ada sampel (sensor mati?)"); continue; }
        Serial.print(": "); Serial.print(total); Serial.println(" sampel");

        for (uint8_t k = 0; k < 12; k++) {
            if (!_jHist[i][k]) continue;
            Serial.print("      ");
            Serial.print(VL53L1X::rangeStatusToString((VL53L1X::RangeStatus)k));
            Serial.print(" : "); Serial.print(_jHist[i][k]);
            Serial.print(" ("); Serial.print(100UL * _jHist[i][k] / total);
            Serial.println("%)");
        }
        if (_jN[i]) {
            uint16_t rata = (uint16_t)(_jJumlah[i] / _jN[i]);
            uint16_t sebar = _jMax[i] - _jMin[i];
            Serial.print("      jarak sah : "); Serial.print(_jMin[i]);
            Serial.print(" .. ");              Serial.print(_jMax[i]);
            Serial.print(" mm, rata "); Serial.print(rata);
            Serial.print(" mm, sebaran "); Serial.print(sebar); Serial.println(" mm");
            // Inilah kesimpulan yang dicari: hantu yang MENETAP hampir tak
            // bersebaran, hantu dari pantulan sesaat bersebaran lebar.
            if (sebar <= 20)
                Serial.println("      -> nyaris TIDAK bersebaran: benda TETAP di depan sensor"
                               " (crosstalk kaca / bagian robot), bukan pantulan sesaat.");
            else if (sebar >= 100)
                Serial.println("      -> sebaran LEBAR: pantulan tak menentu, bukan benda tetap.");
        }
    }
    Serial.println("  Di ruang kosong yang benar, yang WAJAR adalah 100% 'signal fail'.");
}

void LidarArray::isolasiNolkan(uint8_t fase) {
    _isoMin[fase] = 0xFFFF; _isoMaks[fase] = 0;
    _isoJumlah[fase] = 0; _isoN[fase] = 0; _isoTotal[fase] = 0;
}

// Hentikan / jalankan lagi mode kontinu SELAIN channel yang diuji. Sensor yang
// dihentikan berhenti memancar IR -- itulah inti percobaannya.
void LidarArray::isolasiLainnya(bool nyalakan) {
    for (uint8_t c = 0; c < NUM_LIDAR; c++) {
        if (c == _isoCh || !_isReady[c]) continue;
        if (!selectMux(c)) continue;
        if (nyalakan) {
            _sensor[c].startContinuous(LIDAR_PERIOD_MS);
            // Riwayat filternya sudah tidak berlaku sesudah dijeda.
            _dist[c] = -1; _histN[c] = 0; _lastOk[c] = 0;
            _jauh[c] = false; _lastResp[c] = millis();
        } else {
            _sensor[c].stopContinuous();
        }
    }
}

void LidarArray::isolasiMulai(int8_t ch, uint16_t detik) {
    _isoDetik = detik;
    _isoSemua = (ch < 0);

    if (_isoSemua) {
        for (uint8_t i = 0; i < NUM_LIDAR; i++) _isoRata[i][0] = _isoRata[i][1] = -1;
        // mulai dari sensor aktif pertama
        int8_t p = -1;
        for (uint8_t i = 0; i < NUM_LIDAR; i++) if (_isReady[i]) { p = (int8_t)i; break; }
        if (p < 0) { Serial.println("Uji isolasi: tak ada sensor aktif."); return; }
        ch = p;
        Serial.print("\nUji isolasi SEMUA sensor, ");
        Serial.print(2 * detik); Serial.println(" detik per sensor.");
        Serial.println("Arahkan robot ke ruang KOSONG dan JANGAN disentuh sampai selesai.");
    } else if (ch >= (int8_t)NUM_LIDAR || !_isReady[ch]) {
        Serial.println("Uji isolasi: channel tidak sah atau sensornya tidak aktif.");
        return;
    }

    _isoCh = (uint8_t)ch;
    isolasiNolkan(0); isolasiNolkan(1);
    _isoFase = 1;
    _isoSampai = millis() + (uint32_t)detik * 1000UL;

    Serial.print("\n  channel "); Serial.print(_isoCh);
    Serial.print(" ("); Serial.print(LIDAR_NAMA[_isoCh]);
    Serial.println(") -- fase A: semua sensor memancar");
}

void LidarArray::isolasiCatat(uint8_t ch, uint16_t mm, uint8_t status) {
    if (ch != _isoCh) return;
    uint8_t f = _isoFase - 1;
    if (f > 1) return;
    _isoTotal[f]++;
    if (status != VL53L1X::RangeValid &&
        status != VL53L1X::RangeValidNoWrapCheckFail &&
        status != VL53L1X::RangeValidMinRangeClipped) return;
    if (mm < _isoMin[f]) _isoMin[f] = mm;
    if (mm > _isoMaks[f]) _isoMaks[f] = mm;
    _isoJumlah[f] += mm; _isoN[f]++;
}

// Pindah ke sensor aktif berikutnya. false bila sudah habis.
bool LidarArray::isolasiLanjut() {
    for (uint8_t i = _isoCh + 1; i < NUM_LIDAR; i++) {
        if (!_isReady[i]) continue;
        _isoCh = i;
        isolasiNolkan(0); isolasiNolkan(1);
        _isoFase = 1;
        _isoSampai = millis() + (uint32_t)_isoDetik * 1000UL;
        Serial.print("\n  channel "); Serial.print(i);
        Serial.print(" ("); Serial.print(LIDAR_NAMA[i]);
        Serial.println(") -- fase A: semua sensor memancar");
        return true;
    }
    return false;
}

// Bacaan menetap yang JAUH bukan hantu -- itu memang ada bendanya. Hantu yang
// kita buru selalu pendek (hasil lapangan: 25-50 mm). Tanpa batas ini, sensor
// yang kebetulan menghadap dinding akan ikut dituduh bermasalah.
static const uint16_t HANTU_BATAS_MM = 150;

void LidarArray::isolasiTabel() {
    Serial.println("\n--- TABEL UJI ISOLASI ---");
    Serial.println("  channel        A: semua   B: sendiri   kesimpulan");
    uint8_t nCross = 0, nMelekat = 0;
    for (uint8_t i = 0; i < NUM_LIDAR; i++) {
        if (_isoRata[i][0] < 0 && _isoRata[i][1] < 0) continue;
        Serial.print("  ch"); Serial.print(i); Serial.print(" ");
        Serial.print(LIDAR_NAMA[i]);
        for (uint8_t k = strlen(LIDAR_NAMA[i]); k < 10; k++) Serial.print(' ');

        for (uint8_t f = 0; f < 2; f++) {
            if (_isoRata[i][f] < 0) Serial.print("   -kosong-");
            else { Serial.print("   "); Serial.print(_isoRata[i][f]); Serial.print(" mm  "); }
        }
        if (_isoRata[i][0] >= 0 && _isoRata[i][1] < 0) {
            Serial.println("  hantu HILANG -> crosstalk antar-sensor"); nCross++;
        } else if (_isoRata[i][0] >= 0 && _isoRata[i][1] >= _isoRata[i][0] + 100) {
            Serial.println("  hantu MUNDUR -> sebagian besar crosstalk"); nCross++;
        } else if (_isoRata[i][0] < 0 && _isoRata[i][1] < 0) {
            Serial.println("  bersih (tak ada target)");
        } else if (_isoRata[i][1] >= 0 && _isoRata[i][1] >= (int16_t)HANTU_BATAS_MM) {
            Serial.println("  target NYATA, bukan hantu");
        } else {
            Serial.println("  TIDAK berubah -> melekat pada sensor"); nMelekat++;
        }
    }
    Serial.println();
    if (nCross && !nMelekat) {
        Serial.println("  KESIMPULAN: crosstalk ANTAR-SENSOR di semua yang bermasalah.");
        Serial.println("  Bisa diperbaiki di perangkat lunak: ukur BERGILIRAN, bukan serentak.");
    } else if (nMelekat && !nCross) {
        Serial.println("  KESIMPULAN: semuanya MELEKAT pada sensor masing-masing.");
        Serial.println("  Perangkat lunak tidak bisa memperbaikinya -- periksa kaca penutup/braket.");
    } else if (nCross && nMelekat) {
        Serial.println("  KESIMPULAN: campuran -- sebagian crosstalk, sebagian melekat.");
    } else {
        Serial.println("  KESIMPULAN: tidak ada hantu terdeteksi.");
    }
    Serial.print("  (bacaan menetap di atas "); Serial.print(HANTU_BATAS_MM);
    Serial.println(" mm dianggap benda sungguhan, bukan hantu)");
}

void LidarArray::isolasiCetak() {
    Serial.println("\n--- HASIL UJI ISOLASI ---");
    const char* label[2] = { "A. semua sensor memancar", "B. hanya sensor ini  " };
    uint16_t rata[2] = {0, 0};
    for (uint8_t f = 0; f < 2; f++) {
        Serial.print("  "); Serial.print(label[f]); Serial.print(" : ");
        if (!_isoTotal[f]) { Serial.println("tidak ada sampel"); continue; }
        Serial.print(_isoN[f]); Serial.print("/"); Serial.print(_isoTotal[f]);
        Serial.print(" sampel sah");
        if (_isoN[f]) {
            rata[f] = (uint16_t)(_isoJumlah[f] / _isoN[f]);
            Serial.print(", "); Serial.print(_isoMin[f]);
            Serial.print(" .. ");  Serial.print(_isoMaks[f]);
            Serial.print(" mm, rata "); Serial.print(rata[f]); Serial.print(" mm");
        }
        Serial.println();
    }

    Serial.print("  KESIMPULAN: ");
    if (!_isoN[0] && !_isoN[1]) {
        Serial.println("tidak ada bacaan sah sama sekali -- ulangi menghadap ruang kosong.");
    } else if (_isoN[0] && !_isoN[1]) {
        Serial.println("hantu HILANG saat sensor lain dimatikan.");
        Serial.println("  -> CROSSTALK ANTAR-SENSOR. Keenam sensor memancar bersamaan;");
        Serial.println("     pancaran tetangga masuk ke penerima sensor ini.");
        Serial.println("     Obatnya di perangkat lunak: bergiliran, bukan serentak.");
    } else if (_isoN[0] && _isoN[1] && rata[1] > rata[0] + 100) {
        Serial.println("hantu MUNDUR JAUH saat sensor lain dimatikan.");
        Serial.println("  -> sebagian besar CROSSTALK ANTAR-SENSOR.");
    } else {
        Serial.println("hantu TIDAK berubah saat sensor lain dimatikan.");
        Serial.println("  -> bukan crosstalk antar-sensor. Sumbernya melekat pada sensor ini:");
        Serial.println("     kaca penutup, lubang braket, atau bagian robot di depannya.");
        Serial.println("     Perangkat lunak tidak bisa memperbaikinya -- periksa fisiknya.");
    }
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
            Serial.print("MATI -- tidak merespons");
        } else if (d == LIDAR_JAUH) {
            Serial.print("jauh (di atas "); Serial.print(LIDAR_MAX_CM); Serial.print(" cm)");
        } else {
            Serial.print(d); Serial.print(" cm");
        }
        // Jawaban mentah + status: ini yang memisahkan "sensor melihat hantu"
        // dari "kode salah menyaring".
        Serial.print("   [mentah ");
        Serial.print(_mmAkhir[i]); Serial.print(" mm, ");
        Serial.print(VL53L1X::rangeStatusToString((VL53L1X::RangeStatus)_statusAkhir[i]));
        Serial.println("]");
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
    // Nomor pin dicetak dari konstanta, bukan ditulis tangan: bus ini sudah
    // sekali pindah, dan judul yang berbohong soal pin mengirim orang
    // mengukur jalur yang salah.
    Serial.print("\n--- PINDAI I2C (bus LiDAR, SDA ");
    Serial.print(LIDAR_I2C_SDA); Serial.print(" / SCL ");
    Serial.print(LIDAR_I2C_SCL); Serial.println(") ---");

    LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
    bool mux = (LIDAR_I2C_BUS.endTransmission() == 0);
    Serial.print("  TCA9548A @0x"); Serial.print(I2C_MUX_ADDR, HEX);
    Serial.println(mux ? " : TERDETEKSI" : " : TIDAK MENJAWAB");

    // Pindaian ini lebih baru daripada catatan begin(). Dulu _muxOk hanya
    // pernah diisi saat boot, jadi mux yang baru disambungkan tetap dianggap
    // hilang selamanya -- dan navMulai() menolak jalan tanpa sebab yang terlihat.
    _muxOk = mux;

    if (!mux) {
        // KEADAAN GARIS DULU, baru tuduhan. Tapi pin 18/19 sedang dipegang
        // peripheral LPI2C, dan digitalRead() di situ membaca register GPIO
        // yang TIDAK mengikuti pad -- ia balas RENDAH terus walau garisnya
        // sehat. Versi pertama pemindai ini memakai bacaan itu apa adanya dan
        // melaporkan "SDA RENDAH, SCL RENDAH" pada bus yang belum tentu rusak.
        //
        // Jadi busnya dilepas dulu, dibaca sebagai GPIO biasa, baru dipasang
        // lagi. 'I' memang perintah diagnostik manual -- memulai ulang bus di
        // sini boleh, di tengah misi tidak.
        LIDAR_I2C_BUS.end();
        const bool sehat = bebaskanBus(LIDAR_I2C_SDA, LIDAR_I2C_SCL);
        LIDAR_I2C_BUS.begin();
        LIDAR_I2C_BUS.setClock(LIDAR_I2C_CLOCK);

        if (!sehat) {
            Serial.println("  -> SDA MASIH RENDAH sesudah 9 pulsa clock. Bukan program.");
            Serial.println("     Ukur ohm SDA-GND dan SCL-GND dengan catu MATI.");
            return;
        }

        // Garis sudah sehat sesudah dibebaskan -- coba mux sekali lagi
        // sebelum menyerah.
        LIDAR_I2C_BUS.beginTransmission(I2C_MUX_ADDR);
        if (LIDAR_I2C_BUS.endTransmission() == 0) {
            Serial.println("  -> mux MENJAWAB sesudah bus dibebaskan. Bus tadi tersangkut.");
            _muxOk = true;
            mux = true;
        }
        if (mux) return;

        // Garis sehat tapi 0x70 bisu. Sapu seluruh alamat: satu pun yang
        // menjawab berarti bus hidup dan muxnya sendiri yang bermasalah.
        Serial.print("  sapuan 0x08..0x77:");
        uint8_t ketemu = 0;
        for (uint8_t a = 0x08; a < 0x78; a++) {
            LIDAR_I2C_BUS.beginTransmission(a);
            if (LIDAR_I2C_BUS.endTransmission() == 0) {
                Serial.print(" 0x"); Serial.print(a, HEX);
                ketemu++;
            }
        }
        Serial.println(ketemu ? "" : " (kosong)");
        if (!ketemu)
            Serial.println("  -> bus hidup tapi SUNYI: curigai catu mux, bukan alamatnya.");

        Serial.println("  -> cek kabel SDA/SCL, catu daya mux, resistor pull-up.");
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

        LIDAR_I2C_BUS.beginTransmission(VL53L1X_ADDR);
        if (LIDAR_I2C_BUS.endTransmission() != 0) {
            Serial.println("KOSONG (tidak ada VL53L1X)");
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
            Serial.println("VL53L1X @0x29 ADA, sudah aktif");
            continue;
        }

        // Ada di bus tapi tidak bekerja -> inilah yang dulu butuh reset papan.
        bool tadinyaIniet = _isReady[ch];
        if (initSensor(ch)) {
            Serial.print("VL53L1X @0x29 ADA -> INIT ULANG BERHASIL");
            Serial.println(tadinyaIniet ? " (tadinya MATI)" : " (tadinya gagal init)");
            pulih++;
        } else {
            Serial.println("VL53L1X @0x29 ADA tapi INIT ULANG GAGAL");
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
