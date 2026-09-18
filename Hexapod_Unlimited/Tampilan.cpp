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
static const char* const TOMBOL_PIN[TOMBOL_N] = { "D6", "D5", "D4", "D3" };

// Fungsi tiap tombol, [indeks][0 = tekan, 1 = tahan]. HANYA untuk laporan;
// yang menjalankan tetap switch di tekan() dan tahan(). Dua tempat yang harus
// dijaga sinkron memang harga yang dibayar, dan itu disengaja: laporan yang
// ikut memilih perintah akan menjadi jalur kedua ke dalam robot, dan seluruh
// file ini berdiri di atas janji bahwa jalur itu cuma satu.
//
// Layar DIAM, TANYA, MUNDUR, IMU, PIVOT dan SKOR menimpa fungsi ini -- di sana
// tombol berarti "nyalakan layar", "ya/batal", "batalkan", "catat/keluar",
// "kembali", "tutup". Cabang itu melapor sendiri.
//
// Teks ini juga yang dibaca operator di LAYAR_TANYA, jadi ia harus menjelaskan
// akibatnya, bukan namanya. Muat 2 baris x 21 kolom = 42 karakter.
static const char* const TOMBOL_FUNGSI[TOMBOL_N][2] = {
    { "siapkan: I lalu R lalu b", "JALAN (hitung mundur 1 detik)" },
    { "STOP di ruas kini",        "lanjut dari ruas tersimpan"    },
    { "catat 1 arah kompas",      "kalibrasi pivot"               },
    { "mirror -- BELUM ADA",      "reset ruas & poin ke 0"        },
};

// Peta pin dibalik sekali (D2..D5 menjadi D6..D3) dan bisa dibalik lagi. Dua
// tombol di pin yang sama tidak memberi gejala apa pun selain "satu tombol
// menjalankan dua fungsi", jadi penjaganya di waktu kompilasi.
static_assert(PIN_TOMBOL_A != PIN_TOMBOL_B && PIN_TOMBOL_A != PIN_TOMBOL_C &&
              PIN_TOMBOL_A != PIN_TOMBOL_D && PIN_TOMBOL_B != PIN_TOMBOL_C &&
              PIN_TOMBOL_B != PIN_TOMBOL_D && PIN_TOMBOL_C != PIN_TOMBOL_D,
              "dua tombol memakai pin yang sama");
static_assert(TOMBOL_KOMPAS < TOMBOL_N, "TOMBOL_KOMPAS di luar tabel");

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

    // LAYAR_TANYA memberi teks fungsi dua baris dari empat: 2 x 21 = 42 kolom.
    // Kelebihannya tidak membungkus ke baris ketiga -- ia menimpa baris
    // "lagi=YA lain=batal", yaitu satu-satunya petunjuk cara menjawab. Di sini,
    // bukan di dalam cabang "OLED ada", dengan alasan yang sama dengan
    // pemeriksaan di atas: tabelnya paling sering disunting di meja tanpa layar.
    for (uint8_t i = 0; i < TOMBOL_N; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            if (strlen(TOMBOL_FUNGSI[i][j]) > 42) {
                Serial.print("!! TOMBOL_FUNGSI["); Serial.print(i);
                Serial.print("]["); Serial.print(j);
                Serial.println("] lebih dari 42 kolom, layar TANYA terpotong.");
            }
        }
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
    oled.display();
    // Panel menyala sesudah begin(). update() yang pertama melihat LAYAR_DIAM
    // dan langsung mematikannya -- keadaan default adalah layar mati.
    _nyala = true;
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
            _nyala = true;
            Serial.println("OLED terdeteksi.");
        }
    }

    // HABIS WAKTU = TIDAK JADI. Layar yang menunggu jawaban manusia mati
    // sendiri sesudah LAYAR_TIMEOUT_MS tanpa sentuhan, dan kembali ke keadaan
    // default: panel mati. Tombol yang sudah ditanyakan tapi tidak dijawab
    // TIDAK dijalankan -- diam bukan persetujuan.
    //
    // LAYAR_IMU dan LAYAR_PIVOT sengaja tidak ada di daftar ini: keduanya
    // dimasuki sesudah konfirmasi dan dikerjakan dengan dua tangan di robot.
    if ((_layar == LAYAR_MENU || _layar == LAYAR_TANYA || _layar == LAYAR_SKOR)
        && millis() - _tSentuh >= LAYAR_TIMEOUT_MS) {
        if (_layar == LAYAR_TANYA) {
            Serial.print("#TOMBOL "); Serial.print(TOMBOL_PIN[_tanyaIdx]);
            Serial.println(" HABIS WAKTU -- tidak jadi");
        }
        _layar = LAYAR_DIAM;
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

    // PANEL MATI ADALAH KEADAAN DEFAULT, bukan tulisan "R2C UKSW".
    //
    // Panel yang menyala menuntut oled.display() tiap 125 ms, dan tiap
    // display() mendorong 512 byte framebuffer lewat I2C -- bus yang sama
    // dengan driver servo (config.h: SERVO_1_I2C_BUS Wire2). Selama tidak ada
    // yang membacanya, giliran bus itu dibayar untuk apa-apa. DISPLAYOFF
    // mematikan panel di sisi SSD1306; isi RAM-nya tetap, jadi menyalakan
    // kembali cuma satu perintah, tanpa begin() ulang.
    if (_layar == LAYAR_DIAM) {
        if (_ada && _nyala) {
            oled.clearDisplay();
            oled.display();
            oled.ssd1306_command(SSD1306_DISPLAYOFF);
            _nyala = false;
        }
        return;
    }
    if (_ada && !_nyala) {
        oled.ssd1306_command(SSD1306_DISPLAYON);
        _nyala = true;
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

// TOMBOL TIDAK LAGI MENJALANKAN APA PUN SECARA LANGSUNG.
//
// Dari menu, tekan dan tahan hanya MENANYAKAN: layar memperlihatkan apa yang
// akan terjadi, dan perintahnya baru jalan kalau tombol yang sama DITEKAN
// sekali lagi. Menahan tidak menjawab -- jawaban cuma satu gerakan, supaya
// tidak ada tombol yang menjalankan perintah lewat dua gerakan berbeda. Sebelumnya satu sentuhan tak sengaja di garis start langsung
// mengirim 'm1'. Yang dijaga bukan cuma salah tekan: dengan panel mati sebagai
// keadaan default, operator tidak selalu tahu layar mana yang sedang aktif
// sebelum ia menekan.
//
// Pengecualian, semuanya disengaja:
//   TOMBOL_STOP  tekan = REM. Jalan seketika dari layar mana pun, juga saat
//                panel mati. Tidak pernah ditanyakan.
//   LAYAR_DIAM   tekan apa pun cuma menyalakan layar.
//   LAYAR_MUNDUR tekan apa pun membatalkan -- rem tidak minta konfirmasi.
//   LAYAR_IMU    tombol kompas mencatat langsung; empat arah lewat delapan
//                konfirmasi tidak bisa dikerjakan sambil memutar robot.
//   LAYAR_PIVOT  tombol apa pun kembali ke menu.
//   LAYAR_SKOR   tombol apa pun menutup.
void Tampilan::tekan(uint8_t i) {
    _tSentuh = millis();

    // HITUNG MUNDUR DIPERIKSA PALING DULU, mendahului rem di bawahnya. Di sini
    // robot BELUM berjalan, jadi rem yang benar adalah membatalkan hitung
    // mundurnya. Mengirim 'm0' saja tidak cukup: aksiTekan() tidak menyentuh
    // _layar, layarnya tetap LAYAR_MUNDUR, dan hitungannya tetap habis lalu
    // menjalankan 'm1' -- rem yang justru memberangkatkan robot.
    if (_layar == LAYAR_MUNDUR) {
        lapor(i, false, "BATALKAN hitung mundur");
        _layar = LAYAR_MENU;
        pesan("mundur DIBATALKAN");
        return;
    }

    // JAWABAN "YA" MENDAHULUI REM, dan hanya untuk tombol yang sedang
    // ditanyakan. Tanpa urutan ini, TOMBOL_STOP tidak bisa lagi menjawab
    // pertanyaannya sendiri -- pertanyaan "lanjut dari ruas tersimpan" lahir
    // dari MENAHAN tombol yang sama, dan cabang rem di bawah akan menelan
    // tekanan jawabannya. Fungsi lanjut jadi tidak punya tombol sama sekali.
    //
    // Yang dibayar: selama layar TANYA untuk TOMBOL_STOP terbuka, menekannya
    // berarti "ya, lanjut", bukan rem. Jendelanya paling lama 5 detik, dan
    // hanya ada kalau operator sendiri yang baru saja menahan tombol itu untuk
    // bertanya. Rem yang tidak pernah kalah tetap ada di tombol lain:
    // TOMBOL_RESET tahan berhenti dari layar mana pun, tanpa pengecualian.
    if (_layar == LAYAR_TANYA && i == _tanyaIdx) {
        lapor(i, _tanyaTahan, "YA -- jalankan");
        // Kembali ke MENU DULU, baru jalankan: aksi yang punya layarnya
        // sendiri (MUNDUR, IMU, PIVOT) menimpanya, dan yang tidak punya
        // mendarat di menu -- yang lalu mati sendiri sesudah 5 detik.
        _layar = LAYAR_MENU;
        if (_tanyaTahan) aksiTahan(i); else aksiTekan(i);
        return;
    }

    // REM, DARI LAYAR MANA PUN, TERMASUK SAAT PANEL MATI.
    //
    // Sengaja mendahului cabang LAYAR_DIAM. Panel mati sesudah 5 detik, jadi
    // selama misi berjalan panel HAMPIR SELALU mati -- dan kalau rem harus
    // membangunkan layar dulu, menghentikan robot yang sedang menabrak dinding
    // butuh dua tekanan. Yang dijaga cabang LAYAR_DIAM adalah robot yang
    // BERGERAK karena disentuh; 'm0' bergerak ke arah sebaliknya.
    if (i == TOMBOL_STOP) {
        lapor(i, false, TOMBOL_FUNGSI[i][0]);
        _layar = LAYAR_MENU;     // supaya pesannya terlihat, panel ikut menyala
        aksiTekan(i);
        return;
    }

    // DARI LAYAR MATI, tekan apa pun HANYA menyalakan layar. Perintahnya TIDAK
    // ikut jalan: robot yang berdiri di garis start tidak boleh bergerak
    // karena seseorang menyentuh tombol untuk melihat layarnya.
    if (_layar == LAYAR_DIAM) {
        lapor(i, false, "nyalakan layar (dari panel mati)");
        _layar = LAYAR_MENU;
        return;
    }

    if (_layar == LAYAR_SKOR) {
        lapor(i, false, "tutup layar skor");
        _layar = LAYAR_MENU;
        return;
    }

    // SETUP KOMPAS: masuknya sudah dikonfirmasi, dan di dalamnya TIDAK ada
    // habis waktu. Tombol kompas mencatat arah berikutnya langsung; tombol lain
    // keluar tanpa menjalankan fungsinya sendiri.
    if (_layar == LAYAR_IMU) {
        if (i == TOMBOL_KOMPAS) {
            lapor(i, false, TOMBOL_FUNGSI[i][0]);
            aksiTekan(i);
        } else {
            lapor(i, false, "keluar dari setup kompas");
            _layar = LAYAR_DIAM;
        }
        return;
    }

    // KALIBRASI PIVOT berjalan sampai operator mengetik 'S'. Tombol di sini
    // hanya kembali ke menu. Dulu cabang ini tidak ada: tekanan apa pun jatuh
    // ke switch di bawah dan menjalankan fungsi tombol saat robot berputar.
    if (_layar == LAYAR_PIVOT) {
        lapor(i, false, "kembali ke menu");
        _layar = LAYAR_MENU;
        return;
    }

    // Sampai di sini, layar TANYA berarti tombol LAIN yang ditekan: batal.
    // Jawaban "ya" sudah ditangani jauh di atas.
    if (_layar == LAYAR_TANYA) {
        lapor(i, false, "BATAL -- panel dimatikan");
        _layar = LAYAR_DIAM;
        return;
    }

    minta(i, false);   // LAYAR_MENU: tanya dulu
}

void Tampilan::minta(uint8_t i, bool tahanan) {
    _tanyaIdx   = i;
    _tanyaTahan = tahanan;
    _layar      = LAYAR_TANYA;

    // Tanda tanya, bukan spasi: di HUD baris ini harus bisa dibedakan dari
    // baris tombol yang benar-benar menjalankan sesuatu.
    Serial.print("#TOMBOL ");
    Serial.print(TOMBOL_PIN[i]);
    Serial.print(tahanan ? " TAHAN? " : " TEKAN? ");
    Serial.print(TOMBOL_FUNGSI[i][tahanan ? 1 : 0]);
    Serial.println(" -- TEKAN tombol sama = YA, lain = batal, 5 detik = batal");
}

void Tampilan::aksiTekan(uint8_t i) {
    switch (i) {
        case 0:   // D6 -- siapkan robot
            kirimCmd("I");
            kirimCmd("R");
            kirimCmd("b");
            pesan("init, capit nol, berdiri");
            break;

        case 1:   // D5 -- STOP di ruas saat ini
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

        case 3:   // D3 -- lapangan cermin: BELUM ADA
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
    _tSentuh = millis();

    // REM KEDUA: berhenti dan nolkan penunjuk ruas serta poin. Alasannya sama
    // dengan TOMBOL_STOP, dan sama-sama mendahului cabang panel mati. Ia sudah
    // dijaga ambang tahan 2 detik, jadi salah picu bukan yang dikhawatirkan.
    if (i == TOMBOL_RESET) {
        lapor(i, true, TOMBOL_FUNGSI[i][1]);
        _layar = LAYAR_MENU;
        aksiTahan(i);
        return;
    }

    if (_layar == LAYAR_DIAM) {
        lapor(i, true, "nyalakan layar (dari panel mati)");
        _layar = LAYAR_MENU;
        return;
    }

    // JAWABAN HANYA LEWAT TEKAN. Menahan di layar TANYA tidak menjawab apa pun
    // -- juga tidak membatalkan. Tahan adalah gerakan yang sudah punya arti
    // lain di menu, dan membiarkannya ikut menjawab berarti tombol yang sama
    // menjalankan perintah lewat dua gerakan berbeda. Yang ditahan di sini
    // akan habis waktunya seperti layar TANYA yang dibiarkan saja.
    if (_layar == LAYAR_TANYA || _layar == LAYAR_MUNDUR || _layar == LAYAR_IMU ||
        _layar == LAYAR_PIVOT  || _layar == LAYAR_SKOR) {
        lapor(i, true, "tahan tidak berarti di layar ini");
        return;
    }

    minta(i, true);    // LAYAR_MENU: tanya dulu
}

void Tampilan::aksiTahan(uint8_t i) {
    switch (i) {
        case 0:   // D6 tahan -- JALAN, sesudah hitung mundur pendek
            _layar   = LAYAR_MUNDUR;
            _tMundur = millis();
            break;

        case 1:   // D5 tahan -- lanjutkan dari ruas tempat berhenti
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

        case 3:   // D3 tahan -- kembalikan penunjuk ruas ke 0
            //
            // SLOT INI TADINYA KOSONG. Diisi reset ruas karena 'continue'
            // pindah ke D5 tahan dan meninggalkan reset tanpa tombol. Katakan
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
            // KOSONG, bukan "R2C UKSW". Keadaan default sekarang panel MATI,
            // dan update() sudah mengirim DISPLAYOFF sebelum sampai ke sini --
            // cabang ini hanya dilewati sekali, pada gambar terakhir sebelum
            // panel padam. Menggambar nama tim tiap 125 ms berarti membayar
            // giliran I2C untuk tulisan yang tidak melaporkan keadaan apa pun.
            break;

        case LAYAR_TANYA: {
            // Sisa waktu dibulatkan KE ATAS: "1s" harus masih terlihat selama
            // detik terakhir, dan hitungan yang melompat dari 1 ke 0 lalu
            // padam membuat operator mengira ia masih punya waktu.
            const uint32_t lewat = millis() - _tSentuh;
            const uint32_t sisa  = (lewat >= LAYAR_TIMEOUT_MS)
                                       ? 0 : (LAYAR_TIMEOUT_MS - lewat);
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.print(TOMBOL_PIN[_tanyaIdx]);
            oled.print(_tanyaTahan ? " TAHAN" : " TEKAN");
            oled.print("     ");
            oled.print((int)((sisa + 999) / 1000));
            oled.println("s");

            // Teks fungsi membungkus sendiri ke baris berikutnya: jatah 21
            // kolom per baris, dan dua baris (y 8 dan y 16) menampung 42.
            oled.setCursor(0, 8);
            oled.println(TOMBOL_FUNGSI[_tanyaIdx][_tanyaTahan ? 1 : 0]);

            oled.setCursor(0, 24);
            oled.println("tekan lagi=YA  batal");
            break;
        }

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
            // Nama pin diambil dari TOMBOL_PIN[], bukan diketik: peta pin
            // sudah pernah berubah sekali (D2..D5 menjadi D6..D3), dan
            // legenda yang menyebut pin yang salah lebih buruk daripada tidak
            // ada legenda. Tepat 21 kolom.
            oled.setCursor(0, 24);
            oled.print(TOMBOL_PIN[TOMBOL_KOMPAS]);
            oled.println("=catat, lain=keluar");
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
            oled.println("tombol = kembali");
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

            // 18 kolom dari jatah 21. Tombol keempat tidak disebut: fungsi
            // tekannya masih mirror yang belum ada, dan legenda yang
            // menjanjikan tombol mati lebih menyesatkan daripada legenda yang
            // diam. Nama pin dari TOMBOL_PIN[], supaya peta pin cukup diubah
            // di satu tempat.
            oled.setCursor(0, 24);
            oled.print(TOMBOL_PIN[0]); oled.print("jln ");
            oled.print(TOMBOL_PIN[1]); oled.print("stop ");
            oled.print(TOMBOL_PIN[2]); oled.println("imu");
            break;
        }
    }
    oled.display();
}
