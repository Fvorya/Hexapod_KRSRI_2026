#pragma once
#include <math.h>

// KEPUTUSAN "berhenti dulu, baru koreksi" -- dipisah dari Navigation.cpp
// SUPAYA BISA DIUJI DI PC. Keduanya murni: tidak menyentuh sensor, tidak
// menyentuh state, tidak memanggil millis(). Pemanggilnya yang memegang
// histeresis, batas waktu dan pencetakan.
//
// Alasan berkas sendiri, bukan static di Navigation.cpp: Navigation.cpp
// menarik seluruh Arduino, LidarArray dan Hexapod, dan stub di ../test-pc
// belum punya pinMode/digitalRead sehingga berkas itu tidak bisa dilink di
// PC. Header ini tidak bergantung apa pun selain math.h, jadi cek_koreksi.cpp
// menguji angka yang BENAR-BENAR dipakai firmware, bukan salinannya.
//
// Lihat NAV_KOREKSI_* di config.h untuk arti tiap ambang.

// Sudut yang DITUJU, derajat, dengan tanda yang sama seperti sudutDinding():
// positif = hidung menyerong KE ARAH dinding yang diikuti.
//
// Bukan selalu nol. Memutar di tempat tidak memindahkan badan, jadi galat
// JARAK hanya bisa ditutup dengan menyerong sedikit ke arah dinding lalu
// berjalan. Robot yang kejauhan membidik positif (mendekat), yang kedekatan
// membidik negatif (menjauh).
//
// jarak NaN (sensor sisi bisu) -> 0, artinya "cukup sejajar saja". Menebak
// arah serong dengan jarak yang tidak diketahui lebih buruk daripada tidak
// menyerong sama sekali.
static inline float navSudutBidik(float jarak, float setpoint,
                                  float k, float maks) {
    if (isnan(jarak)) return 0.0f;
    float bidik = k * (jarak - setpoint);
    if (bidik >  maks) bidik =  maks;
    if (bidik < -maks) bidik = -maks;
    return bidik;
}

// true bila robot harus BERHENTI untuk mengoreksi.
//
// Dua sebab, sesuai permintaan R2C: sudutnya sudah melenceng, ATAU jaraknya
// keluar pita. Keduanya diukur terhadap SASARAN, bukan terhadap nol -- kalau
// robot memang sedang sengaja menyerong untuk menutup jarak, serong itu bukan
// kesalahan yang perlu dihentikan.
//
// phi NaN -> false. Sensor sisi yang bisu bukan alasan berhenti; lup kemudi
// biasa sudah punya penanganan dinding hilang sendiri.
static inline bool navPerluKoreksi(float phi, float bidik,
                                   float jarak, float setpoint,
                                   float ambangDeg, float ambangCm) {
    if (isnan(phi)) return false;
    if (fabsf(phi - bidik) > ambangDeg) return true;
    if (!isnan(jarak) && fabsf(jarak - setpoint) > ambangCm) return true;
    return false;
}

// true bila koreksi yang sedang berjalan sudah boleh dilepas.
//
// HANYA sudut yang diperiksa, walau pemicunya boleh jarak. Berdiri di tempat
// tidak pernah mengubah jarak, jadi menunggu jarak membaik di sini berarti
// menunggu sampai batas waktu, tiap kali. Yang menutup jarak langkah
// sesudahnya, dengan badan yang sudah membidik ke arah yang benar.
static inline bool navKoreksiSelesai(float phi, float bidik, float keluarDeg) {
    if (isnan(phi)) return true;      // sensor hilang di tengah koreksi
    return fabsf(phi - bidik) <= keluarDeg;
}
