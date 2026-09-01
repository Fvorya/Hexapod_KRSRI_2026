#include "Calib.h"   // huruf besar: aman di filesystem case-sensitive (CI/Linux)
#include <string.h>
#include <stddef.h>   // offsetof -- lihat catatan CRC di save()/load()
#include <EEPROM.h>

// Gunakan ini apabila ingin mengetes di PC
// #ifdef ARDUINO
//   #include <EEPROM.h>
// #else
//   // Stub host (untuk test/ tanpa Teensyduino). Simpan di RAM.
//   static uint8_t HOST_EE[1024];
//   struct HostEE {
//       template <class T> T&       get(int a, T& t)       { memcpy(&t, HOST_EE + a, sizeof(T)); return t; }
//       template <class T> const T& put(int a, const T& t) { memcpy(HOST_EE + a, &t, sizeof(T)); return t; }
//   } EEPROM;
// #endif

// v6: tabel invert diperbaiki (v5 memakai rumus "balik semua kaki kiri" yang
//     salah di 9 dari 18 sendi). Versi WAJIB dinaikkan -- kalau tidak, blob v5
//     yang sudah terlanjur tersimpan di EEPROM tetap lolos CRC dan invert lama
//     akan dimuat kembali, sehingga perbaikan di bawah tidak berefek apa-apa.
// v7: default wall.kp & wall.kd diganti (lihat catatan di PARAM_DEFS).
//     Sama seperti v6, versi WAJIB dinaikkan: gain lama sudah terlanjur ada di
//     EEPROM alamat 0 dan blob v6 tetap lolos CRC, jadi tanpa kenaikan versi
//     robot akan memuat kembali 0,030/0,010 dan perubahan ini tak berefek.
// v8: wall.setpoint 13 -> 19 cm, plus dua parameter BARU (wall.min, wall.hantu).
// v9: wall.hantu DIBUANG lagi. Pekerjaannya pindah ke LIDAR_MIN_CM di config.h,
//     yang menolak bacaan mustahil di SEMUA arah -- termasuk sensor DEPAN, yang
//     tidak pernah terlindungi oleh wall.hantu. Itulah sebab robot berbelok
//     sendiri di lorong kosong: hantu 5 cm sensor depan dibaca sebagai halangan.
//     Menyaring di LidarArray membuat satu aturan berlaku untuk semua pemakai
//     jaraknya, bukan cuma jalur ikut-dinding.
#define CALIB_VERSION 9     // naikkan bila layout CalibBlob/urutan param berubah
#define CALIB_ADDR    EE_CALIB_ADDR   // satu sumber alamat: config.h / EEMap.h

const ParamDef PARAM_DEFS[N_PARAMS] = {
    { "pulse.min",        500.0f,  400.0f, 1200.0f, P_SERVO_LEMAS },
    { "pulse.max",       2500.0f, 1800.0f, 2600.0f, P_SERVO_LEMAS },
    { "arm.pulse.min",   1000.0f,  500.0f, 1500.0f, P_SERVO_LEMAS },
    { "arm.pulse.max",   2000.0f, 1500.0f, 2500.0f, P_SERVO_LEMAS },
    { "gait.step_height",  40.0f,    0.0f,  120.0f, P_PERLU_B },
    { "gait.step_length",  60.0f,    0.0f,  150.0f, P_PERLU_B },
    { "gait.cycle_time",  900.0f,  300.0f, 2000.0f, P_PERLU_B },
    { "gait.duty",          0.5f,    0.3f,    0.7f, P_LANGSUNG },
    { "gait.slew_rate",     3.0f,    0.5f,   10.0f, P_LANGSUNG },
    { "gait.profile_tau",   0.25f,   0.05f,   1.0f, P_LANGSUNG },
    { "gait.settle_tau",    0.10f,   0.02f,   0.5f, P_LANGSUNG },
    { "stab.tau",           0.08f,   0.02f,   0.5f, P_BELUM_DIPAKAI },
    { "stab.sign_roll",    -1.0f,   -1.0f,    1.0f, P_BELUM_DIPAKAI },
    { "stab.sign_pitch",   -1.0f,   -1.0f,    1.0f, P_BELUM_DIPAKAI },
    { "heading.kp",         0.020f,  0.0f,    0.1f, P_LANGSUNG },
    { "heading.kd",         0.004f,  0.0f,    0.05f, P_LANGSUNG },
    // wall.kp 0,030 -> 0,008 : yang lama menjenuhkan kemudi ke +-1,00 begitu
    //   dinding lebih jauh dari 46 cm, jadi robot memutar PENUH menghadap
    //   dinding alih-alih menggeser mendekat.
    // wall.kd 0,010 -> 0,030 : yang lama bekerja pada jarak yang dibulatkan ke
    //   cm dengan pembagi dt loop, jadi ia hanya menyuntikkan impuls yang lalu
    //   disaring GAIT_SLEW_RATE. Sesudah turunan dihitung pada laju sampel
    //   LiDAR, angka yang lebih besar barulah berarti sebagai redaman.
    // Dipilih lewat sweep di simulasi; tetap perlu disetel di robot sungguhan.
    { "wall.kp",            0.008f,  0.0f,    0.1f, P_LANGSUNG },
    { "wall.kd",            0.030f,  0.0f,    0.05f, P_LANGSUNG },
    // GEOMETRI -- kenapa 13 cm membuat kaki menggesek dinding.
    //
    // Titik TERLEBAR robot adalah ujung kaki TENGAH: pangkal coxa x = +-90 mm
    // ditambah STAND_RADIUS 70 mm = 160 mm dari pusat badan. Diukur di
    // sim_dinding, angka itu TIDAK bertambah saat berjalan maupun saat memutar
    // di batas NAV_WALL_TURN_MAX -- kaki tengah bergeser di sumbu Y, bukan X.
    //
    // Keempat LiDAR samping (ch0/ch1 kiri, ch3/ch4 kanan) menghadap TEGAK LURUS
    // dinding; hanya ch5 (depan) dan ch2 (belakang) yang searah sumbu panjang.
    // Jadi saat robot sejajar lorong:
    //
    //     x_dinding = x_sensor + jarak_terbaca
    //
    // x_sensor = jarak pasang sensor dari pusat badan, diukur ~50 mm dari
    // gambar tata letak (ch0/ch1 di 49,8 mm; ch3/ch4 di 52,0 mm).
    //
    // Konsekuensinya: kaki menyentuh dinding saat sensor masih membaca
    // 160 - 50 = 110 mm. Setpoint lama 13 cm hanya menyisakan celah 2 cm --
    // satu ayunan kemudi saja sudah cukup untuk menyentuh.
    //
    // 19 cm memberi 50 + 190 - 160 = 80 mm celah. Batas atas dinaikkan ke 45 cm
    // supaya lorong lebar juga terlayani.
    //
    // CARA MENYETEL YANG PALING TEPAT (tidak perlu menebak x_sensor):
    // berdirikan robot di samping dinding, atur dengan tangan sampai celah
    // ujung kaki tengah ke dinding sesuai selera (mis. 8 cm), lalu baca 'L'.
    // Angka sensor samping SAAT ITU adalah wall.setpoint yang benar.
    { "wall.setpoint",     19.0f,    5.0f,   45.0f, P_LANGSUNG },
    // AMBANG "TERLALU DEKAT". Di bawah ini kendali PD dilewati dan robot
    // memutar menjauh dengan kekuatan tetap. Perlu karena PD proporsional
    // dengan wall.kp 0,008 memang SENGAJA lembut supaya tidak menjenuh saat
    // dinding jauh -- konsekuensinya, saat terlalu dekat 10 cm pun koreksinya
    // cuma 0,08 dari 1,00. Satu gain tidak bisa memenuhi kedua kebutuhan itu,
    // jadi respons dekat dipisahkan jadi aturannya sendiri.
    // 15 cm = kaki masih 4 cm dari dinding saat penjaga ini mulai bekerja.
    { "wall.min",          15.0f,    4.0f,   35.0f, P_LANGSUNG },
    { "head.utara",         0.0f,    0.0f,  360.0f, P_BELUM_DIPAKAI },
    { "head.timur",        90.0f,    0.0f,  360.0f, P_BELUM_DIPAKAI },
    { "head.selatan",     180.0f,    0.0f,  360.0f, P_BELUM_DIPAKAI },
    { "head.barat",       270.0f,    0.0f,  360.0f, P_BELUM_DIPAKAI },
    { "arena.mirror",       0.0f,    0.0f,    1.0f, P_BELUM_DIPAKAI },   // 0=hadap kanan, 1=cermin (hadap kiri)
};

CalibBlob gCalib;

// Arah putar tiap sendi -- HASIL UJI FISIK, disalin dari
// legacy-2026/TES_GERAK/servo_map.h (smDefaults, Agustus 2026).
// Polanya TIDAK seragam per kaki, jadi JANGAN disederhanakan jadi rumus:
//   coxa  : dibalik di KEENAM kaki (arah coxa memang berlawanan dgn pemasangan)
//   femur : dibalik hanya di sisi KANAN (kaki 0,1,2)
//   tibia : dibalik hanya di sisi KIRI  (kaki 3,4,5)
// Femur & tibia saling melengkapi antar sisi -- konsisten dengan modul kaki
// kanan/kiri yang terpasang bercermin.
// Urutan slot: 0-17 kaki (coxa,femur,tibia per kaki), 18-20 lengan kanan,
// 21-23 lengan kiri. Sama dengan SLOT_NAME[] di servo_map.h.
static const uint8_t INVERT_DEF[TOTAL_SERVOS] = {
    1, 1, 0,    // K0 kanan-depan
    1, 1, 0,    // K1 kanan-tengah
    1, 1, 0,    // K2 kanan-belakang
    1, 0, 1,    // K3 kiri-belakang
    1, 0, 1,    // K4 kiri-tengah
    1, 0, 1,    // K5 kiri-depan
    0, 0, 0,    // lengan kanan (base, shoulder, grip)
    0, 0, 0     // lengan kiri
};

uint16_t Calib::crc16(const uint8_t* p, uint32_t n) {
    uint16_t crc = 0xFFFF;                        // CRC16-CCITT
    for (uint32_t i = 0; i < n; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void Calib::applyDefaults() {
    for (int i = 0; i < N_PARAMS; i++) gCalib.param[i] = PARAM_DEFS[i].def;
    
    // <--- LOOPING SAMPAI TOTAL_SERVOS (24 slot: 18 kaki + 2x3 lengan)
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        gCalib.offset[i] = 0.0f;
        gCalib.trim[i]   = 0;
        gCalib.invert[i] = INVERT_DEF[i];   // lihat tabel di atas
    }
    
    gCalib.magic[0] = 'H'; gCalib.magic[1] = 'X';
    gCalib.version  = CALIB_VERSION;
}

// CRC dihitung sampai offsetof(crc), BUKAN sizeof - sizeof(crc).
//
// Keduanya terlihat setara, dan memang setara kalau struct-nya tidak punya
// padding di ekor. CalibBlob punya: sizeof 272, tapi field crc ada di byte
// 268-269, jadi 272-2 = 270 berarti hash ikut MENELAN field crc itu sendiri.
// Akibatnya save() mem-hash crc LAMA lalu menimpanya dengan crc BARU, sehingga
// load() -- yang mem-hash rentang sama, kini berisi crc baru -- selalu
// mendapat angka berbeda dan SELALU menolak blob-nya.
//
// Jadi blok kalibrasi di EEPROM 0 tidak pernah sekali pun berhasil dimuat:
// tiap boot jatuh ke applyDefaults() lalu menulis ulang. Gejalanya cuma satu
// baris "EEPROM kosong/rusak/versi beda" yang gampang dikira normal.
//
// ServoMap kebetulan lolos (sizeof 126, crc di 124) -- kebetulan, bukan
// karena rumusnya benar. Karena itu di sini pun dipakai offsetof.
void Calib::save() {
    gCalib.magic[0] = 'H'; gCalib.magic[1] = 'X';
    gCalib.version  = CALIB_VERSION;
    gCalib.crc = crc16((const uint8_t*)&gCalib, offsetof(CalibBlob, crc));
    EEPROM.put(CALIB_ADDR, gCalib);
}

bool Calib::load() {
    CalibBlob tmp;
    EEPROM.get(CALIB_ADDR, tmp);
    uint16_t want = crc16((const uint8_t*)&tmp, offsetof(CalibBlob, crc));
    if (tmp.magic[0] == 'H' && tmp.magic[1] == 'X' &&
        tmp.version == CALIB_VERSION && tmp.crc == want) {
        gCalib = tmp;
        return true;
    }
    applyDefaults();   // EEPROM kosong/rusak/versi beda -> default + persist
    save();
    return false;
}

void Calib::begin() { load(); }

int Calib::findParam(const char* name) {
    for (int i = 0; i < N_PARAMS; i++)
        if (strcmp(name, PARAM_DEFS[i].name) == 0) return i;
    return -1;
}

bool Calib::setParam(const char* name, float v) {
    int i = findParam(name);
    if (i < 0) return false;
    if (v < PARAM_DEFS[i].lo) v = PARAM_DEFS[i].lo;
    if (v > PARAM_DEFS[i].hi) v = PARAM_DEFS[i].hi;
    gCalib.param[i] = v;
    return true;
}
