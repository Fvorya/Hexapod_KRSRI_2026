#include <Arduino.h>
#include "config.h"
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"

// ====================================================================
// DEKLARASI OBJEK GLOBAL
// ====================================================================
Imu imu;
Hexapod robot;                // Digunakan oleh Navigation
Navigation nav(imu, robot);    // Menyuntikkan referensi IMU dan Motion

// ====================================================================
// PARSER PERINTAH SERIAL
// ====================================================================
static void handleCmd(char* s) {
    while (*s == ' ') s++;
    char c = *s;
    
    // Alat bantu ekstrak angka dari perintah
    bool hasNum = (s[1] >= '0' && s[1] <= '9') || s[1] == '-';
    int  v      = atoi(s + 1);
    uint8_t d1  = (uint8_t)(s[1] - '0'); // Ekstrak digit pertama (0-9)
    
    switch (c) {
        // --- 1. NAVIGASI: KOMPAS ARENA ---
        case 'c': 
            if (d1 > 3) { Serial.println("c0=UTARA c1=TIMUR c2=SELATAN c3=BARAT"); break; }
            nav.kompasCatat(d1); 
            break;
            
        case 'k': 
            nav.kompasTabel(); 
            break;
            
        case 'e': 
            nav.kompasSimpan(); 
            break;
            
        case 'E': 
            if (nav.kompasMuat(true)) nav.kompasTabel(); 
            break;

        // --- 2. NAVIGASI: PIVOT OTOMATIS (PD) ---
        case 'o': 
            if (d1 > 3) { Serial.println("o0=UTARA o1=TIMUR o2=SELATAN o3=BARAT"); break; }
            // robot.stop(); // Opsional: Pastikan robot berhenti sebelum pivot
            nav.pivotKompas(d1); 
            break;
            
        case 'O': 
            if (!hasNum) { Serial.println("Format: O<derajat>, misal O90 atau O-90"); break; }
            nav.pivotRelatif((float)v); 
            break;

        // --- 3. NAVIGASI: KALIBRASI ---
        case 'C': 
            nav.kalibrasiPivot((uint8_t)(hasNum ? v : 4)); 
            break;

        // --- 4. KONTROL ROBOT DASAR ---
        case 's': // Stop
            robot.stop();
            Serial.println("Robot berhenti.");
            break;
            
        case 'w': // Walk (Maju manual)
            robot.walk(NAV_FWD_SPEED, 0.0f, 0.0f);
            Serial.println("Robot maju.");
            break;

        case 'h': // Bantuan
            Serial.println("\n--- BANTUAN PERINTAH SERIAL ---");
            Serial.println("KOMPAS:");
            Serial.println("  c[0-3] : Catat arah (0=U, 1=T, 2=S, 3=B)");
            Serial.println("  k      : Cetak tabel kompas");
            Serial.println("  e / E  : Simpan / Muat dari EEPROM");
            Serial.println("PIVOT:");
            Serial.println("  o[0-3] : Pivot menuju arah arena");
            Serial.println("  O[der] : Pivot relatif (misal O90)");
            Serial.println("  C[sik] : Kalibrasi pivot (derajat/siklus)");
            Serial.println("GERAK DASAR:");
            Serial.println("  w      : Jalan Maju");
            Serial.println("  s      : Stop");
            Serial.println("  Enter  : Rem Darurat");
            break;
            
        default: 
            Serial.println("Perintah tidak dikenal. Ketik 'h' untuk bantuan.");
    }
}

// ====================================================================
// FUNGSI SETUP
// ====================================================================
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {} // Tunggu serial maksimal 3 detik

    Serial.println("\n\nMemulai Hexapod Unlimited...");

    // 1. Inisialisasi Perangkat Keras
    imu.begin();
    robot.begin();
    
    // 2. Muat Data Memori
    nav.begin(); 
    
    Serial.println("Sistem siap! Ketik 'h' untuk daftar perintah.");
}

// ====================================================================
// FUNGSI LOOP (Non-Blokir)
// ====================================================================
void loop() {
    // 1. BACA SENSOR (Prioritas Tinggi - Bebas Waktu)
    imu.update(); 
    
    // 2. KENDALI GERAK & SERVO (Diatur internal oleh Hexapod)
    robot.update(); 
    
    // 3. PARSER SERIAL MONITOR
    static char buf[40];
    static uint8_t len = 0;

    while (Serial.available()) {
        char ch = Serial.read();
        
        // Eksekusi jika ditekan Enter
        if (ch == '\n' || ch == '\r') {
            buf[len] = 0; // Kunci string
            
            if (len) {
                handleCmd(buf); // Masuk ke parser
            } else {
                // Fitur Keselamatan: Tekan Enter kosong untuk rem darurat
                robot.stop(); 
                Serial.println("!! REM DARURAT (Vektor = 0) !!");
            }
            len = 0; // Bersihkan buffer untuk perintah berikutnya
            
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = ch; // Tampung karakter ke buffer
        }
    }
}