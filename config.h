#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define NUM_SERVOS 18 // 18
#define NUM_TUNE_SERVOS 18 //24
// #define ARM_NUM_SERVOS 3

// --- Dimensi Kaki Hexapod --- //
#define COXA_LENGTH 23.0f
#define FEMUR_LENGTH 54.0f
#define TIBIA_LENGTH 69.0f

#define STAND_HEIGHT  100.0f // Tinggi badan dari tanah
#define STAND_RADIUS   70.0f // Jauh kaki ke pangkal coxa

// Posisi pangkal coxa tiap kaki dari pusat (mm).
const float BODY_LEG_ORIGINS[6][3] = {
    { 45.0f,  78.0f, 0.0f},  // 0 (Kaki kanan depan)
    { 90.0f,   0.0f, 0.0f},  // 1 (Kaki kanan)
    { 45.0f, -78.0f, 0.0f},  // 2 (Kaki kanan belakang)
    {-45.0f, -78.0f, 0.0f},  // 3 (Kaki kiri belakang)
    {-90.0f,   0.0f, 0.0f},  // 4 (Kaki kiri)
    {-45.0f,  78.0f, 0.0f}   // 5 (Kaki kiri depan)
};

// Arah hadap kaki (derajat, dari +X, CCW positif).
const float BODY_LEG_ANGLE[6] = {
     60.0f,    // 0 (Kaki kanan depan)
      0.0f,    // 1 (Kaki kanan)
    -60.0f,    // 2 (Kaki kanan belakang)
   -120.0f,    // 3 (Kaki kiri belakang)
    180.0f,    // 4 (Kaki kiri)
    120.0f     // 5 (Kaki kiri depan)
};

// Peta pin servo kaki (driver 0 = PCA9685 address (0x40)/driver 1 = PCA9685 address (0x41))
const uint8_t SERVO_PIN_MAP[NUM_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0 (coxa,femur,tibia)
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2}   // Kaki 5
};

const uint8_t TUNE_PIN_MAP[NUM_TUNE_SERVOS][2] = {
    {0, 8},  {0, 9},  {0, 10}, // Kaki 0
    {0, 4},  {0, 5},  {0, 6},  // Kaki 1
    {0, 0},  {0, 1},  {0, 2},  // Kaki 2
    {1, 8},  {1, 9},  {1, 10}, // Kaki 3
    {1, 4},  {1, 5},  {1, 6},  // Kaki 4
    {1, 0},  {1, 1},  {1, 2}  // Kaki 5
    // {0, 12}, {0, 13}, {0, 14}, // Lengan kanan (base,shoulder,gripper)
    // {1, 12}, {1, 13}, {1, 14}  // Lengan kiri
};

// const uint8_t ARM_PIN_MAP_R[ARM_NUM_SERVOS][2] = { {0, 12}, {0, 13}, {0, 14} }; // Lengkan kanan
// const uint8_t ARM_PIN_MAP_L[ARM_NUM_SERVOS][2] = { {1, 12}, {1, 13}, {1, 14} }; // Lengan kiri

// BUS I2C (Wire SDA 18 / SCL 19 Teensy 4.1)
#define SERVO_I2C_BUS    Wire
#define SERVO_I2C_CLOCK  100000 // 400000 kHz
#define LIDAR_I2C_BUS    Wire
#define LIDAR_I2C_CLOCK  100000 // 400000 kHz

// --- LiDAR --- //
// #define NUM_LIDAR        6
// #define I2C_MUX_ADDR     0x70
// #define LIDAR_EMA_ALPHA  0.4f    // bobot sampel baru (median dulu, lalu EMA)
// #define LIDAR_TIMEOUT_MS 300     // sensor dianggap mati bila tak ada data valid
// #define LIDAR_MAX_CM     400     // di atas ini dianggap tak valid

// --- IMU --- //
// #define IMU_MAX_YAW_JUMP 30.0f   // derajat/sample; lonjakan > ini ditolak (gangguan magnet)
// // Indeks lidar -> arti (sesuaikan pemasangan fisik)
// #define LIDAR_FRONT      0
// #define LIDAR_FRONT_R    1
// #define LIDAR_RIGHT      2
// #define LIDAR_BACK       3
// #define LIDAR_LEFT       4
// #define LIDAR_FRONT_L    5

// #define IMU_SERIAL       Serial1
// #define IMU_BAUD         921600   // Yahboom 10-axis (protokol WIT, frame 0x55)

// --- Navigasi --- //
// #define HEADING_TOLERANCE_DEG   3.0f     // Toleransi heading dianggap "lurus" (const)
// #define FRONT_STOP_CM     20       // Berhenti/belok bila depan < ini (const)
// #define NAV_FWD_SPEED     0.8f     // Kecepatan maju normal (0..1) (const)

// --- Lain-Lain --- //
// #define PIN_BUTTON_START  30   // Tombol mulai (INPUT_PULLUP)
// #define PIN_LED_FOUND     13   // LED tanda korban ditemukan (cek aturan lomba)

// Stabilisasi badan (IMU)
// #define STAB_MAX_DEG      15.0f   // Clamp koreksi roll/pitch (const)
// #define STAB_DEADBAND_DEG 1.0f    // Abaikan getaran kecil (const)

#define SERVO_PWM_FREQ    50      // Hz, frekuensi sinyal PCA9685 (50-330 Hz)
#define SERVO_COMMIT_MS   20      // ms, periode kirim 18 pulse (20=50Hz, 10=100Hz)

// Loop kontrol laju-tetap (dt deterministik untuk gait/PID/stabilisasi).
#define CONTROL_HZ        100     // Hz, tick loop utama (servo commit tetap di SERVO_COMMIT_MS)
#define PROFILE_LOOP      1       // 1 = cetak "PROF avg/max/util" tiap detik (saat tak tuning)

#endif
