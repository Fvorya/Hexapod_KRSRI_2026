# Penjelasan Kocak-Kocakan 😹

•	servo.ino
Masih belum buat 6 kaki, baru coba jalanin satu kaki (kanan depan) pakai komunikasi serial ke Teensy buat merintah WALK sama STOP.

•	config.h
ngeset pin-pin apa saja yang kepake, ngukur spesifkasi badan robot, dll.

•	calib.cpp / .h
Buat kalibrasi rentang atas bawah parameter dan nilai defaultnya terus disimpen ke EEPROM.

•	HexaServos.cpp / .h
Buat setup servonya batas aman dan ngatur saat awal robot nyala servonya ada di nilai tengah. Terus tinggal di-HexaServos::commit() jalanin servo.

•	HexaGait.cpp / .h
Ini nentuin cara tiap kaki ngelangkah. Kayak tingginya, jauhnya, berapa lama buat nyelesain satu langkah, posisi kaki.

•	LegInverseKinematics.cpp / .h
Buat ngesolve inverse kinematik pakai nilai-nilai yang sudah di-define di config.h

•	Hexapod.cpp / .h
Semuanya bakal masuk ke sini. Bakal nerima perintah buat jalan/stop/pose robot terus nanti dikasih tiap kelas bawahnya (HexaServos, HexaGait, LegInverseKinematics.h).

# Hexapod KRSRI 2026 🕷️ <- Versi Lebih Jelas

Repository ini berisi *source code* sistem kendali untuk robot Hexapod 3-DOF yang dirancang khusus untuk misi penyelamatan pada ajang **Kontes Robot SAR Indonesia (KRSRI) 2026**. 

Sistem diprogram menggunakan C++ berorientasi objek (OOP) dan dirancang untuk mikrokontroler dengan komputasi tinggi (seperti Teensy 4.1), dengan fokus pada pergerakan yang mulus (*smooth trajecotry*) dan arsitektur kode yang *modular*.

## ✨ Fitur Utama

- **Inverse Kinematics (IK):** Perhitungan sudut sendi secara *real-time* (Coxa, Femur, Tibia) menggunakan trigonometri aturan cosinus untuk mencegah pergerakan di luar batas fisik.
- **Tripod Gait Generator:** Animasi langkah berjalan menggunakan kurva trajektori **Sikloid**. Memastikan kecepatan ujung kaki berada di titik nol saat *liftoff* dan *touchdown* untuk menghilangkan hentakan mekanis.
- **Slew Rate & Ramping:** Transisi mulus berbasis waktu (*dt*) saat robot berakselerasi, berhenti, atau berganti profil gaya berjalan (misal: dari mode merayap ke memanjat tangga).
- **Fasad Architecture:** Kelas utama yang membungkus kerumitan perhitungan IK dan sinkronisasi kaki, sehingga perintah navigasi dari otak utama (*Mission*) menjadi sangat sederhana.

## 📂 Struktur Arsitektur Kode

* **`servo.ino`** : Sandbox buat eksperimen. Fokus ke uji coba satu kaki (kanan depan) via komunikasi serial biar servo tidak langsung 'mengamuk' kalau logika kita salah.

Kode dipecah menjadi beberapa "spesialis" untuk memudahkan *debugging* dan *unit testing*:

* **`Hexapod.cpp / .h`** : Dirigen orkestra (CEO). Dia yang menerima perintah 'Jalan', lalu membagi tugas ke HexaGait (buat pola), LegInverseKinematics (buat sudut), dan HexaServos (eksekusi motor).
* **`HexaGait.cpp / .h`** : Otak koreografer. Dia yang merancang pola langkah Tripod Gait. Dia yang menentukan kapan kaki harus melayang (fase Swing) dan kapan harus menapak (fase Stance) supaya robot jalan mulus, bukan loncat-loncat.
* **`LegInverseKinematics.cpp / .h`** : Mesin kalkulator 3D. Mengubah keinginan kita (misal: 'kaki harus di titik X') menjadi sudut derajat yang dimengerti oleh servo.
* **`HexaServos.cpp / .h`** : Driver otot. Ini jembatan antara kode matematika dan motor fisik. Tugasnya memastikan servo bergerak dengan halus dan tidak menyentak saat diperintah.
* **`types.h` & `config.h`** : Fondasi tipe data (Vektor 3D), rumus rotasi matriks. Tempat menyimpan ukuran tulang kaki (Coxa, Femur, Tibia) dan peta pin PCA9685 supaya robot tahu anggota tubuhnya sendiri.

## 🚀 Cara Penggunaan (Testing 1 Kaki)

Untuk melakukan kalibrasi dan pengujian *Inverse Kinematics* pada satu kaki (3 servo), Anda dapat mengunggah file `servo/servo.ino`. 

ketik "W" pada serial monitor Arduino IDE buat jalan ke depan dan "S" buat stop. Ada versi bedanya (di bagian bawah servo.ino) buat masukkin manual koordinat x, y, z buat gerakin kaki

---
