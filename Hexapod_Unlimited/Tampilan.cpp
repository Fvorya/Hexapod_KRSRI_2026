#include "Tampilan.h"
#include "Misi.h"
#include "Navigation.h"
#include "Calib.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 oled(OLED_LEBAR, OLED_TINGGI, &OLED_I2C_BUS, -1);

// Nama arah, urut sesuai perintah 'c0'..'c3'.
static const char* const ARAH_NAMA[4] = { "UTARA", "TIMUR", "SELATAN", "BARAT" };

// Nama pin FISIK, urut indeks tombol. Yang muncul di HUD harus sama dengan
// yang tercetak di PCB -- operator di arena memegang papan, bukan kode.
static const char* const TOMBOL_PIN[TOMBOL_N] = { "D2", "D3", "D4", "D5" };

// Fungsi tiap tombol, [indeks][0 = tekan, 1 = tahan]. HANYA untuk laporan;
// yang menjalankan tetap switch di tekan() dan tahan(). Dua tempat yang harus
// dijaga sinkron memang harga yang dibayar, dan itu disengaja: laporan yang
// ikut memilih perintah akan menjadi jalur kedua ke dalam robot, dan seluruh
// file ini berdiri di atas janji bahwa jalur itu cuma satu.
//
// Layar DIAM, MUNDUR dan SKOR menimpa fungsi ini -- di sana tombol berarti
// "buka menu", "batalkan", "tutup". Cabang itu melapor sendiri.
static const char* const TOMBOL_FUNGSI[TOMBOL_N][2] = {
    { "siapkan: I lalu R lalu b", "JALAN (hitung mundur 1 detik)" },
    { "STOP di ruas kini",        "lanjut dari ruas tersimpan"    },
    { "catat 1 arah kompas",      "kalibrasi pivot"               },
    { "mirror -- BELUM ADA",      "reset ruas & poin ke 0"        },
};

void Tampilan::begin(Misi* misi, Skor* skor, void (*kirim)(char*), Navigation* nav) {
    _misi  = misi;
    _nav   = nav;
    _skor  = skor;
    _kirim = kirim;

    const uint8_t pin[TOMBOL_N] = { PIN_TOMBOL_A, PIN_TOMBOL_B,
                                    PIN_TOMBOL_C, PIN_TOMBOL_D };
    for (uint8_t i = 0; i < TOMBOL_N; i++) {
        _t[i].pin = pin[i];
        _t[i].turun = false;
        _t[i].tahanTerpakai = false;
        _t[i].tSejak = 0;
        // Level awal dianggap LEPAS, bukan dibaca: pinMode baru saja menyalakan
        // pull-up, dan pin yang belum sempat naik akan terbaca LOW -- satu
        // tekanan hantu di detik pertama.
        _t[i].mentahAkhir = false;
        _t[i].tUbah = millis();
        pinMode(pin[i], INPUT_PULLUP);
    }

    // LAYAR PUNYA BUSNYA SENDIRI (Wire2), jadi ia yang harus mem-begin()-nya.
    // LidarArray hanya mengurus Wire. Dipanggil SEKALI di sini, bukan di
    // dalam oled.begin(): argumen periphBegin di bawah sengaja false supaya
    // coba-lagi tiap 3 detik tidak me-reset peripheral berulang kali.
    OLED_I2C_BUS.begin();
    OLED_I2C_BUS.setClock(OLED_I2C_CLOCK);

    // Panjang tabel poin harus sama dengan tabel lintasan. DI ATAS pemeriksaan
    // OLED, bukan di bawahnya: dulu ia ada di dalam cabang "OLED ada", jadi
    // penjaga ini hilang justru saat menguji di meja tanpa layar -- tempat
    // tabelnya paling sering disunting. Poin untuk ruas yang salah tidak
    // pernah terlihat seperti kesalahan tabel.
    if (SKOR_RUAS_N != RUAS_N) {
        Serial.print("!! Tabel poin "); Serial.print(SKOR_RUAS_N);
        Serial.print(" baris, tabel lintasan "); Serial.print(RUAS_N);
        Serial.println(" ruas. Samakan SKOR_RUAS[] di Skor.cpp.");
    }

    _tCoba = millis();
    _ada = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ALAMAT,
                      false,   // reset: pin RST tidak dipakai (-1 di konstruktor)
                      false);  // periphBegin: Wire2 sudah di-begin() sendiri di begin()
    if (!_ada) {
        Serial.println("OLED tidak terdeteksi di 0x3C pada Wire2 (SCL2 24 / SDA2 25)");
        Serial.println("  -- tombol tetap jalan, layarnya saja yang tidak ada. Dicoba");
        Serial.println("  lagi tiap 3 detik. 'I' memindai bus LiDAR, BUKAN bus ini.");
        return;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    gambar();
}

void Tampilan::lapor(uint8_t i, bool tahanan, const char* fungsi) {
    // AWALAN '#', sama seperti '#KORBAN' dan '#LEPAS'. Di mission_hud.py baris
    // berawalan '#' berarti firmware berbicara DULUAN -- bukan jawaban atas
    // poll -- jadi HUD tidak pernah menyembunyikannya.
    //
    // Itu sebabnya bukan '[TOMBOL]'. Baris '[TOMBOL]' lama lewat jalur log
    // biasa, dan poll otomatis HUD menahan log 0,8 detik tiap kali ia mengirim
    // 'm' dan 'l'. Dengan poll sekali sedetik, sebagian besar baris tombol
    // jatuh di dalam jendela diam itu: perintahnya jalan, gemanya hilang.
    // Persis gejala yang dilaporkan R2C 16 Sep 2026.
    Serial.print("#TOMBOL ");
    Serial.print(TOMBOL_PIN[i]);
    Serial.print(tahanan ? " TAHAN " : " TEKAN ");
    Serial.println(fungsi);
}

void Tampilan::pesan(const char* teks) {
    strncpy(_pesan, teks, sizeof(_pesan) - 1);
    _pesan[sizeof(_pesan) - 1] = 0;
    _tPesan = millis() + 1500;
}

void Tampilan::kirimCmd(const char* teks) {
    if (!_kirim) return;
    // handleCmd() menulis ke dalam buffernya, jadi ia butuh salinan yang bisa
    // diubah. Literal string di flash tidak boleh diserahkan langsung.
    char buf[24];
    strncpy(buf, teks, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    Serial.print("[TOMBOL] "); Serial.println(buf);
    _kirim(buf);
}

void Tampilan::update() {
    bacaTombol();

    // OLED DICOBA LAGI selama belum ada. Layar yang dicolok sesudah Teensy
    // menyala -- atau yang catu dayanya menyusul -- tidak lagi menuntut reboot.
    // Tiap 3 detik, bukan tiap loop: probe I2C berbagi bus dengan mux LiDAR,
    // dan sensor lebih penting daripada layar yang mungkin tidak ada.
    if (!_ada && millis() - _tCoba >= 3000) {
        _tCoba = millis();
        _ada = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ALAMAT,
                      false,   // reset: pin RST tidak dipakai (-1 di konstruktor)
                      false);  // periphBegin: Wire2 sudah di-begin() sendiri di begin()
        if (_ada) {
            oled.clearDisplay();
            oled.setTextColor(SSD1306_WHITE);
            Serial.println("OLED terdeteksi -- layar menyala.");
        }
    }

    // Hitung mundur: satu-satunya layar yang bertindak sendiri.
    if (_layar == LAYAR_MUNDUR && millis() - _tMundur >= JALAN_MUNDUR_MS) {
        _layar = LAYAR_MENU;
        const uint8_t i = _misi ? _misi->ruasKini() : 0;
        if (i == 0) kirimCmd("m1");
        else {
            char buf[16];
            snprintf(buf, sizeof(buf), "m4 %u", (unsigned)i);
            kirimCmd(buf);
        }
    }

    // 8 Hz cukup: layar ini dibaca manusia, dan menggambar OLED lewat I2C
    // berbagi bus dengan mux LiDAR. Menggambar tiap loop mencuri giliran
    // sensor yang jauh lebih penting.
    if (millis() - _tGambar >= 125) {
        _tGambar = millis();
        gambar();
    }
}

void Tampilan::bacaTombol() {
    const uint32_t now = millis();
    for (uint8_t i = 0; i < TOMBOL_N; i++) {
        const bool mentah = (digitalRead(_t[i].pin) == LOW);

        // DEBOUNCE BERBASIS KESTABILAN LEVEL, bukan lama tekan.
        //
        // Versi sebelumnya menerima tepi TURUN seketika lalu membuang tekanan
        // yang lebih pendek dari TOMBOL_DEBOUNCE_MS saat dilepas. Itu menutup
        // pantulan di awal tekan, tapi TIDAK menutup pantulan saat DILEPAS:
        // tekan() sudah terlanjur jalan pada tepi naik pertama, dan pantulan
        // sesudahnya memulai tekanan baru. Saklar taktil yang mulai aus
        // menghasilkan dua sampai tiga 'm1' dari satu tekanan -- dan di layar
        // menu, dua tekanan berarti dua perintah yang berbeda.
        //
        // Sekarang level mentah harus DIAM selama TOMBOL_DEBOUNCE_MS sebelum
        // ia dianggap keadaan tombol. Pantulan, ke arah mana pun, cuma
        // menyetel ulang penghitungnya dan tidak pernah sampai ke tekan().
        if (mentah != _t[i].mentahAkhir) {
            _t[i].mentahAkhir = mentah;
            _t[i].tUbah = now;
            continue;
        }
        if (now - _t[i].tUbah < TOMBOL_DEBOUNCE_MS) continue;

        if (mentah && !_t[i].turun) {
            _t[i].turun = true;
            _t[i].tSejak = now;
            _t[i].tahanTerpakai = false;
        } else if (mentah && _t[i].turun) {
            // AKSI TAHAN DIJALANKAN SAAT AMBANG TERCAPAI, bukan saat dilepas.
            // Operator perlu tahu tahanannya sudah kena tanpa harus melepas
            // dan melihat -- di arena tangan sedang sibuk.
            if (!_t[i].tahanTerpakai && now - _t[i].tSejak >= TOMBOL_TAHAN_MS) {
                _t[i].tahanTerpakai = true;
                tahan(i);
            }
        } else if (!mentah && _t[i].turun) {
            _t[i].turun = false;
            if (!_t[i].tahanTerpakai) tekan(i);
        }
    }
}

void Tampilan::tekan(uint8_t i) {
    // DARI LAYAR DIAM, tekan apa pun HANYA membuka menu. Perintahnya TIDAK
    // ikut jalan: robot yang berdiri di garis start tidak boleh bergerak
    // karena seseorang menyentuh tombol untuk melihat layarnya.
    if (_layar == LAYAR_DIAM) {
        lapor(i, false, "buka menu (dari layar diam)");
        _layar = LAYAR_MENU;
        return;
    }

    if (_layar == LAYAR_MUNDUR) {
        // Tombol apa pun membatalkan hitung mundur. Ini rem, dan rem harus
        // bisa dijangkau tanpa mengingat tombol mana.
        lapor(i, false, "BATALKAN hitung mundur");
        _layar = LAYAR_MENU;
        pesan("mundur DIBATALKAN");
        return;
    }

    if (_layar == LAYAR_SKOR) {
        lapor(i, false, "tutup layar skor");
        _layar = LAYAR_MENU;
        return;
    }

    lapor(i, false, TOMBOL_FUNGSI[i][0]);

    switch (i) {
        case 0:   // D2 -- siapkan robot
            kirimCmd("I");
            kirimCmd("R");
            kirimCmd("b");
            pesan("init, capit nol, berdiri");
            break;

        case 1:   // D3 -- STOP di ruas saat ini
            _ruasSimpan = _misi ? _misi->ruasKini() : 0;
            kirimCmd("m0");
            {
                char buf[22];
                snprintf(buf, sizeof(buf), "STOP di ruas %u", (unsigned)_ruasSimpan);
                pesan(buf);
            }
            break;

        case 2:   // D4 -- catat satu arah kompas, urut utara..barat
            if (_layar != LAYAR_IMU) { _layar = LAYAR_IMU; _arahBerikut = 0; }
            {
                // DIPANGGIL LANGSUNG, bukan lewat string "c<n>". Bukan karena
                // parser-nya lambat, tapi karena parser tidak mengembalikan
                // apa-apa: tombol tidak bisa tahu pencatatannya DITOLAK.
                // Fungsinya sama persis dengan yang dipanggil perintah 'c'.
                //
                // Maju ke arah berikutnya HANYA kalau berhasil. Versi
                // sebelumnya selalu maju, jadi satu penolakan -- heading yang
                // belum tenang, IMU bisu -- menggeser seluruh sisa tabel satu
                // slot: TIMUR tersimpan di slot SELATAN, dan seterusnya. Dari
                // luar itu terlihat persis seperti kompas yang mencatat angka
                // acak, laporan R2C 16 Sep 2026.
                const bool ok = _nav ? _nav->kompasCatat(_arahBerikut) : false;
                char p[22];
                if (!ok) {
                    snprintf(p, sizeof(p), "%s GAGAL", ARAH_NAMA[_arahBerikut]);
                    pesan(p);
                    break;
                }
                snprintf(p, sizeof(p), "%s dicatat", ARAH_NAMA[_arahBerikut]);
                pesan(p);
                _arahBerikut++;
                if (_arahBerikut >= 4) {
                    // Keempatnya lengkap -> SIMPAN. Tanpa 'e' angkanya hilang
                    // saat Teensy reset, dan itu ketahuan baru saat 'm1'
                    // ditolak di garis start.
                    kirimCmd("e");
                    pesan("4 arah disimpan");
                    _arahBerikut = 0;
                    _layar = LAYAR_MENU;
                }
            }
            break;

        case 3:   // D5 -- lapangan cermin: BELUM ADA
            // Tidak mengirim apa pun. 'arena.mirror' memang ada di tabel
            // kalibrasi, tapi bertanda P_BELUM_DIPAKAI -- tidak satu baris pun
            // membacanya. Menukarnya akan mengubah angka di layar tanpa
            // mengubah robot, dan tombol yang berbohong lebih buruk daripada
            // tombol yang berkata belum ada.
            pesan("Mirror Belum Ada");
            break;
    }
}

void Tampilan::tahan(uint8_t i) {
    if (_layar == LAYAR_DIAM) {
        lapor(i, true, "buka menu (dari layar diam)");
        _layar = LAYAR_MENU;
        return;
    }

    lapor(i, true, TOMBOL_FUNGSI[i][1]);

    switch (i) {
        case 0:   // D2 tahan -- JALAN, sesudah hitung mundur pendek
            _layar   = LAYAR_MUNDUR;
            _tMundur = millis();
            break;

        case 1:   // D3 tahan -- lanjutkan dari ruas tempat berhenti
            {
                const uint8_t idx = _ruasSimpan;
                char buf[16];
                snprintf(buf, sizeof(buf), "m4 %u", (unsigned)idx);
                kirimCmd(buf);
                char p[22];
                snprintf(p, sizeof(p), "lanjut ruas %u", (unsigned)idx);
                pesan(p);
            }
            break;

        case 2:   // D4 tahan -- kalibrasi pivot
            _layar = LAYAR_PIVOT;
            kirimCmd("C");
            pesan("kalibrasi pivot...");
            break;

        case 3:   // D5 tahan -- kembalikan penunjuk ruas ke 0
            //
            // SLOT INI TADINYA KOSONG. Diisi reset ruas karena 'continue'
            // pindah ke D3 tahan dan meninggalkan reset tanpa tombol. Katakan
            // kalau ia lebih baik di tempat lain.
            _ruasSimpan = 0;
            kirimCmd("m0");
            // Poin ikut di-nol: lari berikutnya dari ruas 0 adalah percobaan
            // BARU, dan membawa poin lari sebelumnya membuat angkanya menipu.
            if (_skor) _skor->reset();
            pesan("ruas & poin ke 0");
            break;
    }
}

void Tampilan::gambar() {
    if (!_ada) return;
    oled.clearDisplay();

    // Pesan sementara menimpa apa pun. Ia jawaban atas tombol yang baru
    // ditekan, dan jawaban yang datang terlambat tidak menjawab apa-apa.
    if (_pesan[0] && millis() < _tPesan) {
        oled.setTextSize(1);
        oled.setCursor(0, 8);      // baris 2 dari 4: sisakan ruang kalau membungkus
        oled.println(_pesan);
        oled.display();
        return;
    }
    _pesan[0] = 0;

    switch (_layar) {
        case LAYAR_DIAM:
            // Dua baris ukuran 2 mengisi tinggi 32 tepat (16 + 16). Kolomnya
            // dihitung, bukan ditebak: ukuran 2 lebarnya 12 px per huruf.
            oled.setTextSize(2);
            oled.setCursor((OLED_LEBAR - 3 * 12) / 2, 0);
            oled.println("R2C");
            oled.setCursor((OLED_LEBAR - 4 * 12) / 2, 16);
            oled.println("UKSW");
            break;

        case LAYAR_MUNDUR: {
            const uint32_t sisa = (millis() - _tMundur >= JALAN_MUNDUR_MS)
                                      ? 0
                                      : (JALAN_MUNDUR_MS - (millis() - _tMundur));
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.println("JALAN dalam");
            // Ukuran 2, bukan 3. Angka ukuran 3 tingginya 24 px dan menyisakan
            // 8 px -- tidak cukup untuk baris batal, dan hitung mundur tanpa
            // cara membatalkannya lebih buruk daripada angka yang lebih kecil.
            oled.setTextSize(2);
            oled.setCursor(0, 8);
            oled.println((int)(sisa / 100));       // per sepersepuluh detik
            oled.setTextSize(1);
            oled.setCursor(0, 24);
            oled.println("tombol = batal");
            break;
        }

        case LAYAR_IMU:
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.println("SETUP KOMPAS");
            oled.setCursor(0, 8);
            oled.print("berikut: ");
            oled.println(ARAH_NAMA[_arahBerikut]);
            oled.setCursor(0, 16);
            oled.print("sudah: "); oled.print(_arahBerikut); oled.println("/4");
            oled.setCursor(0, 24);
            oled.println("D4 = catat arah ini");
            break;

        case LAYAR_PIVOT:
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.println("KALIBRASI PIVOT");
            oled.setCursor(0, 8);
            oled.println("robot berputar...");
            oled.setCursor(0, 16);
            oled.println("selesai: ketik 'S'");
            oled.setCursor(0, 24);
            oled.println("D3 = kembali");
            break;

        case LAYAR_SKOR:
        case LAYAR_MENU:
        default: {
            const uint8_t  ruas  = _misi ? _misi->ruasKini() : 0;
            const uint32_t detik = _misi ? _misi->waktuMisiDetik() : 0;

            // Enam baris milik layar 64 px dipadatkan jadi empat. Yang dibuang
            // baris "Mirror Belum Ada": ia tidak melaporkan keadaan apa pun,
            // dan satu baris dari empat terlalu mahal untuk catatan kerja.
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.print("ruas ");
            oled.print(ruas);
            oled.print("/");
            oled.print(RUAS_N - 1);
            oled.print(" ");
            oled.print(detik);
            oled.print("s ");
            oled.println(_misi && _misi->berjalan() ? "JALAN" : "diam");

            oled.setCursor(0, 8);
            oled.print("poin ~");
            oled.print(_skor ? _skor->total() : 0);
            if (_skor && _skor->bonusSah()) {
                oled.print("+");
                oled.print(_skor->bonus(detik));
            }
            oled.println();

            oled.setCursor(0, 16);
            oled.print("korban ");
            oled.print(_skor ? _skor->korbanTertaruh() : 0);
            oled.print("/5 ");
            oled.println((_skor && _skor->membawa()) ? "BAWA" : "");

            // 18 kolom dari jatah 21. D5 tidak disebut: fungsi tekannya masih
            // mirror yang belum ada, dan legenda yang menjanjikan tombol mati
            // lebih menyesatkan daripada legenda yang diam.
            oled.setCursor(0, 24);
            oled.println("D2jln D3stop D4imu");
            break;
        }
    }
    oled.display();
}
