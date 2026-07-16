#include <Arduino.h>
#include "Calib.h"
#include "Hexapod.h"

Hexapod robot;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    // 1. Inisialisasi kalibrasi dan sistem robot
    Calib::begin();
    robot.begin();

    // 2. MENGGUNAKAN setProfile()
    // Urutan: { stepHeight, stepLength, cycleTime, standHeight }
    GaitProfile profilUjiCoba = { 
        50.0f,   // Tinggi angkatan kaki saat melangkah (50 mm)
        80.0f,   // Jauh ayunan langkah (80 mm)
        1200.0f, // Waktu 1 siklus langkah (1.2 detik)
        100.0f   // Tinggi perut / pangkal kaki dari lantai (100 mm)
    };
    robot.setGaitProfile(profilUjiCoba);

    Serial.println("=== TEST TRAJEKTORI 1 KAKI ===");
    Serial.println("Ketik 'W' lalu Enter untuk mulai melangkah maju");
    Serial.println("Ketik 'S' lalu Enter untuk berhenti (kembali ke posisi netral)");
}

void loop() {
    // 3. Baca perintah dari Serial PC
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 'W' || cmd == 'w') {
            // Perintah jalan: maju 100%, geser 0%, belok 0%
            robot.walk(1.0f, 0.0f, 0.0f); 
            Serial.println("Mengeksekusi langkah berjalan...");
        } 
        else if (cmd == 'S' || cmd == 's') {
            // Perintah berhenti
            robot.stop();
            Serial.println("Berhenti (kembali ke pose berdiri)...");
        }
    }

    // 4. Jantung sistem (Wajib dipanggil terus-menerus tanpa delay)
    // Ini akan menjalankan HexaGait -> IK -> Servo secara otomatis
    robot.update(); 
}

// Kode sebelumnya untuk IK manual pake komunikasi serial (x, y, z)
// | | | | | | | | |
// V V V V V V V V V

// #include <Arduino.h>
// #include "Calib.h"
// #include "HexaServos.h"
// #include "InverseKinematics.h"

// HexaServos servos;

// // Fungsi bantuan untuk mengubah derajat IK menjadi mikrodetik PWM
// uint16_t angleToPulse(float degree) {
//     // Asumsi: 0 derajat = PULSE_MIN (500us), 180 derajat = PULSE_MAX (2500us)
//     float pulse = gCalib.param[K_PULSE_MIN] + 
//                   (degree / 180.0f) * (gCalib.param[K_PULSE_MAX] - gCalib.param[K_PULSE_MIN]);
//     return (uint16_t)constrain(pulse, gCalib.param[K_PULSE_MIN], gCalib.param[K_PULSE_MAX]);
// }

// void setup() {
//     Serial.begin(115200);
//     delay(2000); // Tunggu serial siap
    
//     Calib::begin();
//     servos.begin();
    
//     Serial.println("=== TEST 1 KAKI HEXAPOD ===");
//     Serial.println("Format input: X Y Z");
//     Serial.println("Contoh ketik: 100 0 -80");
// }

// void loop() {
//     // Cek apakah ada data masuk dari PC
//     if (Serial.available()) {
//         float x = Serial.parseFloat();
//         float y = Serial.parseFloat();
//         float z = Serial.parseFloat();
        
//         // Bersihkan sisa karakter (seperti Enter/Newline)
//         while(Serial.read() != -1); 

//         float coxaDeg, femurDeg, tibiaDeg;
//         bool inRange = InverseKinematics::solve(x, y, z, coxaDeg, femurDeg, tibiaDeg);

//         Serial.print("Target X:"); Serial.print(x);
//         Serial.print(" Y:"); Serial.print(y);
//         Serial.print(" Z:"); Serial.println(z);

//         if (inRange) {
//             Serial.print(">> IK Sukses! Coxa: "); Serial.print(coxaDeg);
//             Serial.print(" | Femur: "); Serial.print(femurDeg);
//             Serial.print(" | Tibia: "); Serial.println(tibiaDeg);

//             // Geser +90 derajat agar nilai tengah (0 derajat IK) menjadi posisi 90 derajat Servo fisik
//             servos.setLegPulse(0, angleToPulse(coxaDeg + 90.0f));
//             servos.setLegPulse(1, angleToPulse(femurDeg + 90.0f));
//             servos.setLegPulse(2, angleToPulse(tibiaDeg)); // Lutut biasanya sudah absolut
//         } else {
//             Serial.println(">> GAGAL: Titik target di luar jangkauan (Out of Range)!");
//         }
//     }

//     // Eksekusi pergerakan servo secara non-blocking
//     servos.commit();
// }
