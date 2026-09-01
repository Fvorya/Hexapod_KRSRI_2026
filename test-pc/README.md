# Harness simulasi PC

Menjalankan `Navigation`, `Hexapod`, `Imu`, dan `LidarArray` **yang asli** dari
`../Hexapod_Unlimited/` di komputer, tanpa papan Teensy. Tidak ada logika yang
disalin ke sini — yang dipalsukan hanya lapisan perangkat kerasnya.

## Cara pakai

Perlu `g++` (WSL, MinGW-w64, atau Linux):

```
./build.sh              # sintaks + semua simulasi
./build.sh sim_wall     # satu saja
```

## Isi

| | |
|---|---|
| `stub/` | `Arduino.h`, `EEPROM.h`, `Wire.h`, `Adafruit_PWMServoDriver.h`, `VL53L0X.h` |
| `sim/` | Program simulasi |
| `bin/` | Hasil kompilasi (boleh dihapus) |

## Yang dipalsukan

* **Jam** — `millis()` dibaca dari `__nowMs` yang bisa dimajukan sesuka hati,
  jadi timeout 20 detik bisa diuji dalam sepersekian detik.
* **Serial2** — antrean byte. Yaw disuapkan sebagai frame WIT `0x55 0x53`
  sungguhan, sehingga parser IMU ikut teruji, bukan dilewati.
* **I2C** — stub `Wire` mencatat channel mux yang dipilih; stub `VL53L0X`
  mengembalikan jarak per channel dari `__simMm[]`, dan `__initGagal[]` /
  `__bisu[]` bisa meniru sensor yang gagal init atau berhenti mengirim data.
* **EEPROM** — array RAM.

## Batas yang perlu diingat

Model gerak robot di `sim_wall` kasar: perintah putar → laju yaw → laju
lateral, dengan sensor samping dimodelkan sebagai sinar bersudut. Itu memang
lingkar umpan balik yang dominan, jadi **perbandingan** antar gain bermakna —
tapi angka mutlaknya tetap harus disetel di robot sungguhan.

Yang TIDAK dimodelkan: dinamika servo, slip kaki, crosstalk IR antar sensor,
dan getaran badan.
