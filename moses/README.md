# Panel operator Raspberry Pi

UI gelap untuk laptop, disajikan oleh `mission_hud.py`. Tidak memerlukan internet, npm, atau font eksternal.

## Menjalankan versi ini

Salin ke folder aplikasi Raspi, bersama `detect.py` dan model yang sudah dipakai:

- `mission_hud.py`
- `operator_control.py`
- `operator.css`
- `operator.js`

Jalankan aplikasi dengan argumen/service yang biasa digunakan. Buka alamat Raspi dari laptop dan muat ulang halaman. Firmware Teensy dalam repo ini juga perlu dikompilasi dan diunggah agar kontrol putar, watchdog, dan profil EEPROM tersedia. Tidak ada flash/deploy otomatis dari perubahan ini.

## Kontrol manual

1. Baca robot. UI juga meminta pembacaan awal setelah serial tersambung.
2. Lepas STOP jika terkunci, lalu Aktifkan manual. Mode ini menghentikan misi dan navigasi yang sedang berjalan.
3. Topang robot sesuai prosedur pengujian, lalu tekan Berdiri untuk menyalakan servo.
4. Tekan-tahan salah satu dari delapan arah atau tombol putar. Lepaskan untuk berhenti. Speed adalah pengali 5–100%, bukan pengukuran cm/detik.
5. Keyboard bisa diaktifkan secara terpisah: WASD/panah untuk arah, Q/E untuk putar, Esc untuk STOP. Mengisi angka atau berpindah tab menghentikan gerak keyboard.

Tombol STOP mempertahankan daya servo. Lemaskan servo ada di bagian Kendali robot; tindakan ini mematikan PWM. Gerak manual baru memiliki batas perintah 450 ms tanpa pembaruan, kemudian vektor gerak dinolkan; penghentian mekanis tetap mengikuti settling gait. Pengaman jarak memakai sensor arah gerak yang tersedia, bukan jaminan menghindari semua benturan.

## Profil dan EEPROM

Pilih Datar, Tangga, Merunduk, Sempit, Kail, atau Tanjak. Setel tinggi langkah, panjang langkah, waktu siklus, tinggi badan, dan radius kaki melalui slider atau angka.

- **Terapkan perubahan**: mengubah profil di RAM dengan transisi gait yang sudah tersedia.
- **Simpan profil ke EEPROM**: menyimpan profil aktif dan setelan profil lain yang telah diedit; keberhasilan tampil setelah firmware membaca balik hasil penulisan.
- **Muat EEPROM**: mengganti setelan RAM dengan simpanan terakhir.
- **Bawaan profil ini**: membuang override profil terpilih di RAM. Tekan Simpan untuk mempertahankan hasilnya setelah reset.

Enam profil memakai blok baru **2304–2431**, versi 1 dan CRC16. Blok kalibrasi lama di 0, trim di 1024, kompas di 1792, serta kalibrasi gerak di 2048 tidak dipindahkan. Profil sah dimuat saat boot dan digunakan juga ketika misi memilih medan. Robot tidak otomatis bergerak karena memuat profil.

Tinggi badan adalah target model. Bentuk khusus kaki pada Kail/Tanjak tetap mengikuti firmware; rentang input tidak menjamin setiap kombinasi geometrinya terjangkau. Periksa peringatan IK dan hasil pengukuran robot.

## Kalibrasi

Tab Kalibrasi menyediakan pencarian, kelompok parameter Teensy beserta rentang serta waktu berlakunya, trim servo, kompas, dan setelan vision. Edit parameter Teensy dengan tombol Terapkan pada barisnya, lalu Simpan kalibrasi ke EEPROM. Trim dan kompas memakai tombol simpan masing-masing. Nilai vision tersimpan sebagai JSON di Raspi, bukan EEPROM Teensy.

Reconnect USB tidak lagi otomatis mengganti `condong.jeda` dan `condong.yaw`: nilai firmware yang telah dikalibrasi dihormati. Tombol Kirim + simpan condong tetap tersedia jika ingin memakai nilai dari pengaturan Raspi.

## Uji tanpa robot

```powershell
python moses/test_mission_hud.py
python moses/test_operator.py
node --check moses/operator.js
arduino-cli compile -b teensy:avr:teensy41 --warnings all Hexapod_Unlimited
```

Pratinjau lokal opsional:

```powershell
python moses/test_operator.py --preview
```

Buka `http://127.0.0.1:8765`. Banner SIMULASI selalu tampil; server hanya mendengarkan loopback, tidak membuka kamera/serial, dan tidak menulis EEPROM. Tutup dengan Ctrl+C. Simulasi menguji interaksi aplikasi, bukan stabilitas fisik robot.
