# Dokumentasi Hexapod Unlimited Kocak-Kocakan Dikit 🎃

Firmware ini adalah sistem kontrol terpadu untuk robot hexapod (kaki enam) berkemampuan navigasi otonom berbasis IMU, Kinematika Invers (IK) kaki dan lengan, serta manajemen kalibrasi EEPROM.

## 🛠️ Arsitektur Perangkat Keras & Konfigurasi Bus I2C

Sistem menggunakan tiga bus I2C terpisah pada Teensy 4.1 untuk menghindari tabrakan data:
* **`Wire` (SDA 18 / SCL 19):** Jalur khusus untuk sensor LiDAR (TCA9548A / VL53L0X).
* **`Wire1` (SDA 17 / SCL 16):** Menghubungkan Driver Servo 1 (PCA9685 pada alamat `0x41`)[cite: 1, 18, 19].
* **`Wire2` (SDA 25 / SCL 24):** Menghubungkan Driver Servo 0 (PCA9685 pada alamat `0x40`)[cite: 1, 18, 19].
* **Serial2 (RX 7 / TX 8):** Jalur komunikasi IMU (Yahboom 10-axis, protokol WIT, 230400 baud)[cite: 1, 21].

---

## 📋 Daftar Pengujian Fisik & Kalibrasi (Tahapan Eksekusi)

Sebelum menjalankan misi otonom atau navigasi penuh, lakukan pengujian dan kalibrasi dengan urutan berikut menggunakan Serial Monitor (115200 baud, *Newline*):

### 1. Kalibrasi Kaki & Telapak Rata (`TES_GERAK`)
Karena beban gravitasi dan *sagging* pada servo, posisi berdiri awal biasanya membuat ada kaki yang menggantung.
* **Gunakan alat bantu eksternal `TES_GERAK.ino`** untuk mencari offset tinggi telapak per kaki (`zOff`).
* Simpan hasilnya secara permanen ke **EEPROM alamat 2048** dengan mengetik **`W`**. 
* *Catatan:* Program utama `Hexapod` Anda akan otomatis membaca data dari EEPROM 2048 ini saat pertama kali dinyalakan untuk mengoreksi posisi Z tiap telapak kaki[cite: 40].

### 2. Kalibrasi Kompas Arena & Pivot
Untuk memastikan robot dapat berputar secara akurat menuju arah mata angin:
* Letakkan robot menghadap Utara, lalu ketik **`c0`** (0=Utara, 1=Timur, 2=Selatan, 3=Barat)[cite: 16].
* Ketik **`e`** untuk menyimpan data orientasi arena ke **EEPROM alamat 1792**[cite: 16].
* Ketik **`C`** untuk melakukan kalibrasi putar otomatis guna menentukan rasio derajat per siklus langkah[cite: 16].

---

## 🕹️ Panduan Perintah Serial Monitor (`Hexapod_Unlimited`)

Setelah firmware utama diunggah ke Teensy 4.1, Anda dapat mengontrol robot menggunakan perintah teks berikut:

### Kontrol Dasar & Gerak
* **`s`** : Memerintahkan robot untuk berdiri tegak (mengaktifkan siklus Kinematika Invers dan *gait*)[cite: 40].
* **`w`** : Memerintahkan robot berjalan maju dengan kecepatan normal (`NAV_FWD_SPEED`)[cite: 25].
**`Enter Kosong`** : Berhenti darurat (mengatur vektor gerak ke nol / rem mendadak)[cite: 35].

### Navigasi & Kompas Arena (Modul `Navigation`)
* **`c[0-3]`** : Mencatat sudut heading IMU saat ini sebagai arah arena (0=Utara, 1=Timur, 2=Selatan, 3=Barat)[cite: 16].
* **`k`** : Mencetak tabel arah kompas arena yang tersimpan ke Serial Monitor[cite: 16].
* **`e` / `E`** : Menyimpan / Memuat ulang data kompas dari EEPROM 1792[cite: 16].
* **`o[0-3]`** : Memerintahkan robot berputar otomatis (*pivot closed-loop* dengan kendali PD) menuju arah kompas arena yang dipilih[cite: 16].
* **`O<derajat>`** : Memerintahkan robot berputar relatif dari posisi saat ini (misal: `O90` untuk belok 90° searah jarum jam)[cite: 16].

### Manipulator Lengan (Inverse Kinematics)
* Lengan robot dikendalikan menggunakan Kinematika Invers 2D berbasis koordinat Cartesian ($X, Y$)[cite: 18, 41].
* Pemanggilan fungsi `robot.moveArmTarget(x, y)` di dalam program akan otomatis menghitung sudut bahu dan siku secara presisi tanpa perlu menebak-nebak nilai pulsa servo[cite: 41].

---

## Kesimpulan

Programnya masih sama fungsinya kayak TES_GERAK sama TES_IMU, buat kalibrasi kaki masih pakek TES_GERAK disimpen di EEPROM terus nanti dibaca di program ini.

Karna lengan blom jadi blom bisa ngetes IK lengan. Sama buat LiDAR sama PID masih belom diimplementasiin.