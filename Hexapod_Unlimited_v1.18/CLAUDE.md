# CLAUDE.md — Handoff

Repo: `program-krsri-misi`, branch `misi-tabel`. Hexapod KRSRI / SAR UNLIMITED
2026, Teensy 4.1.

Catatan ini merangkum **keputusan arsitektur dan status fitur** dari sesi
12–14 September 2026. Ia tidak mengulang apa yang sudah dijelaskan kode atau
riwayat git — untuk alasan sebuah angka, bacalah komentar di `config.h`, yang
hampir selalu memuat tabel ukurannya.

---

## Cara memverifikasi sebelum commit

Setiap perubahan firmware diuji di PC lebih dulu. Stub Arduino ada di
`../test-pc/stub/` (`Arduino.h` hanya punya `Serial` dan `Serial2`, tidak ada
`Serial1`). `Calib::applyDefaults()` **wajib** dipanggil di tiap program uji —
tanpanya `gParam[]` nol semua dan angka apa pun yang keluar tidak berarti.

```sh
export PATH="/c/msys64/ucrt64/bin:$PATH"
cp Hexapod_Unlimited/Hexapod_Unlimited.ino "$CLAUDE_JOB_DIR/tmp/_ino.cpp"
g++ -std=gnu++17 -O1 -I../test-pc/stub -IHexapod_Unlimited -fsyntax-only \
    -Wall -Wextra -Wno-unused-parameter Hexapod_Unlimited/*.cpp "$CLAUDE_JOB_DIR/tmp/_ino.cpp"

# tiap cek_*.cpp dilink dengan seluruh Hexapod_Unlimited/*.cpp + stubdefs.cpp
python cek_tabel_misi.py
```

Program uji: `cek_kail`, `cek_korban`, `cek_lengan`, `cek_geser`,
`cek_setel_profil`, `cek_penggaris_sisi`, dan `cek_tabel_misi.py`.

Dua jebakan yang berulang di sesi ini:

- Beberapa berkas `Hexapod_Unlimited/` memakai **CRLF** (`Navigation.cpp`,
  `Misi.cpp`), sementara `config.h` memakai LF. Penyuntingan berbasis jangkar
  multi-baris harus menyesuaikan akhir barisnya, kalau tidak jangkarnya tidak
  ketemu.
- Heredoc shell memakan satu garis miring terbalik walaupun dikutip. Untuk
  menyisipkan `\n` literal ke dalam kode, pakai penampung (`B = chr(92)`) lalu
  ganti.

---

## Keputusan arsitektur

### 1. Sekuens korban dimatikan lewat saklar, bukan dihapus

`LENGAN_KORBAN_AKTIF 0` di `config.h`. Ruas `AKS_AMBIL`/`AKS_TARUH` cuma
dilewati: robot **tetap** berhenti di gerbang 24 cm dan **tetap** memicu Raspi,
tapi tidak satu servo lengan pun digerakkan.

Saklarnya dipasang sebagai **hubung-singkat di sisi pemanggil**, bukan `#if`.
Akibatnya sekuens dan `moveArmGrip()` tetap ikut dikompilasi, jadi salah ketik
di dalamnya tetap ketahuan, dan menghidupkannya kembali benar-benar cuma
mengubah 0 jadi 1. `_korban[]` sengaja **tidak** diperbarui saat dilewati —
ringkasan `m1` harus jujur bahwa capit kosong.

### 2. Pergelangan masuk IK: `moveArmGrip()` membidik titik capit

Lengan depan = 3 sendi di bidang 2 dimensi, dan pose bidang itu memang 3 angka
(x, y, sudut tapak). Jadi ketiganya dipenuhi persis, tanpa sisa kebebasan:
mundur `HAND_LENGTH` dari titik capit searah tapak untuk mendapat titik
pergelangan, pecahkan 2 link ke situ, lalu `prg = tapak − (bahu + siku)`.

`moveArmTarget()` menaruh **pergelangan** di titik yang diminta; memakainya
untuk membidik korban berarti tiap pemanggil harus mengurangkan `HAND_LENGTH`
sendiri — 120 mm yang cuma perlu lupa sekali.

`moveArmGrip()` lebih ketat: ia menolak pose yang sudut servonya keluar 0..180.
Ini penting karena **`HexaArm::angleToPulse()` meng-clamp diam-diam**, tanpa
penanda seperti `_servoClamped` milik kaki. Asimetri itu berlaku di seluruh
kode lengan; tiap fitur lengan baru harus memeriksa jangkauannya sendiri.

### 3. Profil KAIL: tiap kelompok kaki punya knopnya sendiri

Menaikkan `KAIL_RADIUS_KAKI` memperbaiki tiga keluhan sekaligus, tapi radius
global ikut menarik kaki belakang keluar — padahal telapak belakang menggantung
42 mm di bawah badan dan sudah memakai 95% jangkauan. Radius 80 dengan langkah
70 langsung mentok. Jadi radius tetap 60, dan tiap kelompok kaki dapat knop
sendiri: `KAIL_TENGAH_KELUAR`, `KAIL_TENGAH_SUDUT`, `KAIL_BELAKANG_MUNDUR`,
`KAIL_BELAKANG_LEBAR`.

Kaki belakang **diluruskan ke belakang** (telapak di garis `x` lokal = 0) supaya
seluruh langkah jadi radial dan coxa berhenti mengayuh — tanpa mekanisme kunci
baru, murni lewat `atan2(y, x)`. `KAIL_BELAKANG_LEBAR` sengaja mengembalikan
sebagian ayunan demi jejak yang tidak menyatu.

Dua hasil pengukuran yang berlawanan dengan dugaan, dan keduanya tercatat di
`config.h`:

- **Sudut kaki tengah tidak berpengaruh apa pun** diukur dari pusat badan.
  Di tiap tripod, sisi pengikatnya garis depan–belakang di sisi yang *sama*,
  dan kaki tengah ada di seberang. Ia baru bicara saat CG bergeser.
- **Melebarkan kaki belakang memperburuk margin di lereng.** Saat CG mundur,
  sisi pengikat berpindah ke garis telapak-belakang → kaki-tengah-seberang.

### 4. Kemudi MENENGAH: dorongan proporsional, bukan bang-bang

Dulu dorongan menjauh memakai `-sisiDekat * NAV_WALL_TURN_MAX`, yaitu bang-bang
terhadap "dinding mana yang lebih dekat". Di lorong sempit yang robotnya sudah
**di tengah**, pertanyaan itu dijawab oleh derau: kemudi berbalik 7× per 10
sampel dengan amplitudo penuh, padahal PD-nya sendiri cuma ±0,001.

Sekarang dorongan memakai `errTengah` sebagai tanda sekaligus besar, penuh di
`NAV_TENGAH_PITA_CM`. Ia hilang sendiri tepat di tempat pertanyaannya jadi tidak
berarti. **Ikut-dinding satu sisi tidak disentuh** — di sana `sisiDekat` tidak
punya apa pun untuk dibalik-balik.

### 5. Ruas bisa belok pecahan derajat

Kolom `float putar = 0.0f` di `Ruas`, terakhir, boleh tidak ditulis. **Mutlak,
bukan relatif**: acuannya mata angin arena hasil `belok` ditambah `putar`. Pivot
relatif menumpuk galatnya sepanjang 33 ruas. **Menumpuk antar ruas**: sekali
satu ruas menyerong 45 der, ruas `BLK_LURUS` sesudahnya tetap 45 der.

Bagian yang tidak jelas dari luar: pivot menyerong saja tidak cukup. Mode arena
mengunci heading ke slot mata angin, dan slot itu dipilih `navMulai()` dari yaw.
Pada serong 45 der, yaw berjarak **sama** dari dua mata angin — pilihannya
lemparan koin, dan yang salah menarik badan 90 der sepanjang ruas tanpa gejala
lain. Karena itu ada `Navigation::kunciHeading(float)`; `navBerhenti()`
melepasnya, alasan yang sama dengan `abaikanDepan`.

### 6. Perintah operator dirangkai, bukan diserahkan ke urutan ketikan

- **`U<cm>`** — naik tangga: `T4` + `Z1` + `F` + `D<cm>` + `i1` sekaligus.
  `i1` **harus** paling akhir, karena `navMulai()` memeriksa sensor depan hidup
  dan pemeriksaan itu sengaja dilewati kalau depan sudah diabaikan. Rem jarak
  wajib: dengan depan buta, tidak ada lagi yang menghentikan robot. Kalau
  `navMulai()` menolak, `U` membatalkan seluruhnya dan tidak jadi membutakan
  sensor depan — hal yang tidak bisa ditiru dengan mengetik manual.
- **`as<bahu> <siku> <pergelangan>`** — tembak ketiga sendi langsung, derajat
  **geometris**, tanpa IK. Untuk membidik pose dengan tangan lalu menuliskannya
  keras. Sudut tetap dikirim walau mentok, dan yang mentok ditandai.

---

## Status fitur

| Fitur | Status |
|---|---|
| Sekuens korban tahap 1 | Selesai, **dimatikan** (`LENGAN_KORBAN_AKTIF 0`) |
| Pergelangan di IK lengan | Jalan, terverifikasi di `cek_korban` |
| Slew lengan 120 der/detik | Jalan, `LENGAN_JEDA_MS 900` sudah menampungnya |
| Lutut kaki depan dikunci di KAIL | Jalan, harga 15,3 mm tekanan stance tercatat |
| Profil KAIL (bentuk kaki) | Jalan, sedang disetel di robot |
| Kemudi MENENGAH tanpa hentakan | Jalan, terverifikasi di `cek_geser` |
| Perintah `U<cm>` | Jalan, sudah dicoba di arena |
| Perintah `as` | Jalan, belum dicoba di robot |
| Belok pecahan derajat | Jalan, **belum ada baris tabel yang memakainya** |

---

## Yang menunggu

### `cek_kail` MERAH, sengaja

```
assert marginGuling(-73) > 30   ->  29,4 mm
```

Gagal 0,6 mm dengan `KAIL_TENGAH_SUDUT 0` yang sekarang dipasang. Ambang 30 itu
pilihan yang tidak pernah diukur, dan geseran CG −73 mm-nya sendiri ±20 mm
karena tinggi titik berat belum ditimbang. Assert yang menyala pada selisih 2%
dari masukan yang cuma diketahui ±27% tidak sedang mengukur apa pun — tapi
menurunkannya supaya hijau juga bukan jawaban selama yang benar belum diketahui.

Tabel lengkapnya, lereng CG −73 mm:
`-20 der → 37,3 | -10 → 33,5 | -5 → 31,5 | 0 → 29,4`

Penutupnya: timbang tinggi CG, **atau** putuskan dari robot bahwa 0 der memang
lebih stabil di tangga. Data itu mengalahkan ambang mana pun.

### Geometri kaki salah, terukur 14 Sep 2026, BELUM diperbaiki

Tabel lengkap dan alasannya ada di `config.h`, tepat di atas `COXA_LENGTH`.
Ringkasnya: pada `b100` robot berdiri 75 mm, bukan 100, dan telapaknya 80 mm
dari sumbu coxa, bukan 70. Itu dua kesalahan yang menumpuk — panjang link
(55/65 terukur versus 80/90 tertulis) dan datum sudut lutut yang meleset
sekitar +24 der. Membetulkan salah satunya sendirian memindahkan robot ke
tempat ketiga yang juga salah.

Penutupnya satu ukuran: sudut femur dan sudut dalam lutut dengan busur
derajat pada `b100`, dibandingkan dengan kolom `sdeg c/f/t` milik `d`.

Yang sudah berlaku sekarang, dan dipakai kerja lengan: **tinggi badan nyata
75 mm**. Setiap pose lengan yang beracuan lantai meleset 25 mm dari model.

### Ukuran lapangan, urut

1. **Tinggi poros coxa dari tanah di T0.** `BODY_LEG_ORIGINS` z = 0, jadi poros
   coxa *adalah* bidang pusat badan. ~100 mm berarti `ARM_ORIGINS` z harus 68
   bukan 45; ~123 mm berarti robot berdiri 23 mm terlalu tinggi — dan itu
   menggeser setiap pose korban yang beracuan lantai.
2. **Tegangan catu servo lengan** (DS3225 butuh 6,8 V), plus uji tekan-lepas di
   capit. Capit melendut 70 mm saat terjulur mendatar sementara lengan lurus
   saat tegak: itu lendutan gravitasi, bukan horn atau trim. Perbaiki catunya
   **sebelum** mengompensasi, atau penyetelan dikerjakan dua kali.
3. **Pose jepit sebenarnya** dengan `a182 -60 15` (poros grip seharusnya 302 mm
   dari pusat badan, 40 mm dari tanah), lalu biaskan `KORBAN_TINGGI_MM`.
4. **Lebar lorong R-9.** `KAIL_TENGAH_KELUAR 30` membuat robot lebih lebar lagi
   di kaki tengah, dan batas knop itu bukan IK melainkan lorong. `WALL_KAKI_CM`
   sendiri sudah tidak lagi menebak: diukur 14 Sep 2026 dengan mendorong kaki
   kanan tengah sampai menyentuh dinding, kedua LiDAR kanan membaca ~7 cm, dan
   angkanya diturunkan 11,0 — 7,0. Yang belum diukur: bacaan yang sama pada
   profil KAIL, yang kaki tengahnya sengaja dikeluarkan 30 mm lagi.

### Lain-lain

- **Panjang rahang capit belum diukur.** `moveArmGrip()` menaruh **poros servo
  grip** di sasaran. Perbaikannya: masukkan panjang rahang ke `HAND_LENGTH` dan
  naikkan `KORBAN_JARAK_CM` sebesar itu dalam cm — jendelanya bergeser 1:1,
  lebarnya tetap.
- **`KORBAN_TINGGI_MM 40` masih tebakan.** Tinggi jepit boneka belum diukur.
- **SZ-2 dan SZ-4 sama-sama peron 4 cm** tapi memakai `KORBAN_TARUH_MM` yang
  sama. Membedakannya perlu kolom baru di `Ruas`.
- **Korban digendong di pose REHAT**, terlipat di atas badan, sejak kedua
  sekuens berakhir di sana (15 Sep 2026). `LENGAN_GENDONG_MM 280` yang lama
  membuat korban menonjol ~14 cm di depan ujung kaki depan dan berpotensi
  menyenggol dinding saat pivot di R-11; itu tidak berlaku lagi. Gantinya satu
  pertanyaan baru yang BELUM diuji: REHAT disetel dengan capit KOSONG, jadi
  belum diketahui apakah boneka yang tergantung di sana membentur LiDAR depan
  atau kabel. Jalankan `aa` dengan boneka di capit dan lihat.
- **Stabilitas menyamping di `KAIL_RADIUS_KAKI 60`** belum diuji di tangga.
- **`RUAS_MAKS 36`** biarkan; tabel 33 baris. Naikkan saat baris ke-37 ditulis.
  Biayanya 9 byte per slot dan tidak menyentuh EEPROM. Plafon kerasnya 254 baris
  (`RUAS_N`/`_i`/`_iAkhir` semuanya `uint8_t`, dan 255 dipakai `_iAkhir` sebagai
  penanda) — dijaga `static_assert` di `Misi.cpp`.
- **`Hexapod_Unlimited.zip`** sengaja tidak di-commit dan belum masuk
  `.gitignore`.
