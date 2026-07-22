#include <Arduino.h>
#include "Hexapod.h"
#include "Calib.h"

// Membuat objek utama robot
Hexapod robot;

void setup() {
    // 1. Inisialisasi Komunikasi PC
    Serial.begin(115200);
    
    // 2. Tunggu sebentar agar Serial Monitor siap (opsional, berguna untuk Teensy)
    delay(2000); 
    
    // 3. Mulai sistem memori dan sistem utama robot
    Calib::begin(); 
    robot.begin();  

    // 4. Tampilkan menu panduan di Serial Monitor
    Serial.println("==========================================");
    Serial.println("   TES BODY KINEMATICS HEXAPOD AKTIF!   ");
    Serial.println("==========================================");
    Serial.println("Gunakan format berikut untuk menggerakkan bodi:");
    Serial.println("1. Translasi -> T <X> <Y> <Z> (Contoh: T 0 0 -20)");
    Serial.println("2. Rotasi    -> R <Roll> <Pitch> <Yaw> (Contoh: R 15 -5 0)");
    Serial.println("3. Reset     -> Ketik: RESET");
    Serial.println("==========================================");
}

void loop() {
    // WAJIB ADA: Menjaga mesin kalkulator dan servo tetap berjalan
    robot.update(); 

    // Cek apakah ada pesan masuk dari laptop
    if (Serial.available() > 0) {
        // Baca teks sampai tombol Enter ditekan
        String input = Serial.readStringUntil('\n');
        input.trim(); // Membersihkan spasi kosong atau karakter 'enter' tersembunyi

        // Jika perintah adalah TRANSLASI (diawali huruf T dan spasi)
        if (input.startsWith("T ")) {
            float tx = 0, ty = 0, tz = 0;
            // sscanf digunakan untuk menyedot 3 angka desimal dari teks
            if (sscanf(input.c_str(), "T %f %f %f", &tx, &ty, &tz) == 3) {
                robot.setBodyTranslation(tx, ty, tz);
                Serial.printf("Eksekusi Translasi: X=%.1f, Y=%.1f, Z=%.1f\n", tx, ty, tz);
            } else {
                Serial.println("Format salah! Gunakan: T <X> <Y> <Z>");
            }
        }
        
        // Jika perintah adalah ROTASI (diawali huruf R dan spasi)
        else if (input.startsWith("R ")) {
            float r = 0, p = 0, y = 0;
            if (sscanf(input.c_str(), "R %f %f %f", &r, &p, &y) == 3) {
                robot.setBodyRotation(r, p, y);
                Serial.printf("Eksekusi Rotasi: Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", r, p, y);
            } else {
                Serial.println("Format salah! Gunakan: R <Roll> <Pitch> <Yaw>");
            }
        }
        
        // Jika perintah adalah RESET
        else if (input == "RESET") {
            robot.setBodyTranslation(0, 0, 0);
            robot.setBodyRotation(0, 0, 0);
            Serial.println("Kembali ke posisi netral (0, 0, 0).");
        }
        
        // Jika teks tidak dikenali
        else if (input.length() > 0) {
            Serial.println("Perintah tidak dikenali!");
        }
    }
}

// Kode sebelumnya untuk ngetes jalan maju
// | | | | | | | | |
// V V V V V V V V V

// #include <Arduino.h>
// #include "Calib.h"
// #include "Hexapod.h"

// Hexapod robot;

// void setup() {
//     Serial.begin(115200);
//     delay(2000);
    
//     // 1. Inisialisasi kalibrasi dan sistem robot
//     Calib::begin();
//     robot.begin();

//     // 2. MENGGUNAKAN setProfile()
//     // Urutan: { stepHeight, stepLength, cycleTime, standHeight }
//     GaitProfile profilUjiCoba = { 
//         50.0f,   // Tinggi angkatan kaki saat melangkah (50 mm)
//         80.0f,   // Jauh ayunan langkah (80 mm)
//         1200.0f, // Waktu 1 siklus langkah (1.2 detik)
//         100.0f   // Tinggi perut / pangkal kaki dari lantai (100 mm)
//     };
//     robot.setGaitProfile(profilUjiCoba);

//     Serial.println("=== TEST TRAJEKTORI 1 KAKI ===");
//     Serial.println("Ketik 'W' lalu Enter untuk mulai melangkah maju");
//     Serial.println("Ketik 'S' lalu Enter untuk berhenti (kembali ke posisi netral)");
// }

// void loop() {
//     // 3. Baca perintah dari Serial PC
//     if (Serial.available()) {
//         char cmd = Serial.read();
        
//         if (cmd == 'W' || cmd == 'w') {
//             // Perintah jalan: maju 100%, geser 0%, belok 0%
//             robot.walk(1.0f, 0.0f, 0.0f); 
//             Serial.println("Mengeksekusi langkah berjalan...");
//         } 
//         else if (cmd == 'S' || cmd == 's') {
//             // Perintah berhenti
//             robot.stop();
//             Serial.println("Berhenti (kembali ke pose berdiri)...");
//         }
//     }

//     // 4. Jantung sistem (Wajib dipanggil terus-menerus tanpa delay)
//     // Ini akan menjalankan HexaGait -> IK -> Servo secara otomatis
//     robot.update(); 
// }

