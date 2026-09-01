# Odometri per-ruas — desain

**Tanggal:** 2026-09-01
**Status:** disetujui, siap dibuat rencana implementasi
**Firmware:** `Hexapod_Unlimited`

---

## 1. Masalah

Misi sekarang menghentikan robot di korban 1 dengan membaca jarak ke dinding START lewat LiDAR belakang (ch2). Cara itu bekerja untuk K-1, tetapi punya dua batas keras:

- **Jangkauan 70 cm.** Di atas `LIDAR_MAX_CM` bacaan jatuh ke `LIDAR_JAUH` dan pemicunya tidak pernah menyala.
- **Butuh dinding di belakang.** Begitu robot melewati tikungan pertama, dinding START hilang dan acuannya habis.

Robot butuh cara menjawab "sudah berapa jauh sejak titik acuan terakhir" yang berlaku di ruas mana pun, tanpa dinding, tanpa batas 70 cm.

## 2. Ruang lingkup

**Yang dibuat:** satu bilangan skalar — jarak tempuh sejak terakhir dinolkan — plus perintah serial untuk membacanya, menolkannya, dan memasang "rem jarak" yang menghentikan robot pada jarak tertentu.

**Yang TIDAK dibuat:**

- Koordinat x,y di kerangka arena. Galatnya menumpuk dan tidak ada yang membutuhkannya sekarang.
- Koreksi slip **otomatis** dari sensor. Belum diketahui apakah perlu — itu yang mau diukur. Yang ada hanyalah satu faktor skala manual (§6) yang diisi orang sesudah mengukur.
- Perubahan pada misi. `Mission` tidak disentuh di iterasi ini; pemicu ch2 untuk K-1 tetap seperti adanya.
- Penghitungan geser samping (`_curX`) dan putar. Navigasi tidak pernah memakai geser samping, dan putar bukan jarak tempuh.

**Tujuan langsung:** menjadi alat ukur. Robot dijalankan sejauh N cm menurut odometri, lalu jarak sebenarnya diukur meteran di arena. Selisihnya menentukan apakah koreksi slip diperlukan sama sekali.

## 3. Pendekatan

Odometer duduk di `HexaGait`, mengintegrasikan dari fase gait dan vektor gerak yang **sudah di-slew**, bukan dari perintah yang diminta.

Dua alternatif ditolak:

**Di `Navigation`, dari `_majuKini`.** Diff-nya paling pendek, tapi (a) `w` (jalan manual) memanggil `robot.walk()` langsung tanpa lewat `navUpdate()`, sehingga pengukuran lurus di lantai terbuka — kasus paling bersih — tidak tercakup; dan (b) `HexaGait` mem-*slew* vektor gerak, jadi mengintegrasikan perintah menghitung jarak lebih jauh dari kenyataan di kedua ujung tiap ruas. Pada ruas 40 cm yang isinya hampir hanya percepatan dan perlambatan, porsi galat itu besar.

**Kelas odometri berdiri sendiri.** Satu file dan satu antarmuka baru untuk menyimpan satu `float` yang datanya sudah ada di `HexaGait`. Tidak ada yang didapat.

## 4. Rumus

README §7 mencatat rumus jarak yang sudah diverifikasi `sim_laju`:

```
jarak = 2 × step_length × perintah_maju × (waktu / cycle_time)
```

Rumus itu **hanya benar saat jalan lurus**. `HexaGait::update()` memangkas panjang langkah saat maju sambil berputar (`HexaGait.cpp:107`):

```c
if (magMax > _prof.stepLength && magMax > 0.001f) {
    float f = _prof.stepLength / magMax;
```

Saat menyusuri dinding, kemudi bekerja terus, jadi `f < 1` hampir sepanjang waktu. Mengabaikannya membuat jarak saat menyusur-dinding terhitung lebih jauh daripada saat lurus **karena bug rumus**, dan selisih itu akan disalahartikan sebagai slip mekanis. Maka rumus yang dipakai:

```
Δjarak_mm = skala × 2 × _curY × _prof.stepLength × f × Δfase
```

dengan `f = min(1, stepLength / magMax)` — faktor yang sama yang sudah dihitung gait — dan `skala` faktor slip dari §6, default `1.0`.

`skala` dikalikan pada **setiap penambahan**, bukan saat dibaca. Konsekuensinya disengaja: mengubah skala di tengah perjalanan tidak menulis ulang jarak yang sudah terkumpul. Riwayat tetap riwayat.

Komponen yaw batal sendiri saat dirata-rata enam kaki, karena `rx` simetris kiri-kanan. Jadi cukup suku `_curY`; tidak perlu merata-rata `sya[]`.

Integrasi memakai `Δfase`, bukan `dt` loop, sehingga kebal terhadap loop yang tersendat — alasan yang sama dengan turunan PD dinding yang memakai stempel sampel LiDAR.

## 5. Komponen

Tidak ada file baru. Tiga sentuhan.

### 5.1 `HexaGait`

- Field `float _jarakMm`.
- Akumulasi ditempatkan **sesudah blok normalisasi**, karena di situlah `f` sudah diketahui. `Δfase` disimpan ke variabel lokal saat dihitung, sebelum `while (_phase >= 1.0f)` membungkusnya.
- Cabang `!moving` sudah `return` lebih dulu, jadi saat robot diam tidak ada yang bertambah.
- Field `float _skalaOdo = 1.0f` — faktor slip dari §6. Hidup di sini, bukan di `.ino`, supaya ia ikut pada saat penambahan dan bukan pada saat pembacaan.
- Antarmuka: `float jarakMm() const`, `void jarakNol()`, `void setSkalaOdo(float)`, `float skalaOdo() const`.

### 5.2 `Hexapod`

Teruskan saja, dalam cm: `float jarakCm() const`, `void jarakNol()`, `void setSkalaOdo(float)`, `float skalaOdo() const`. Fasad ini sudah satu-satunya pintu yang dipakai `.ino` dan misi; `HexaGait` tidak perlu dibocorkan.

### 5.3 `Hexapod_Unlimited.ino`

Perintah `D`:

| Perintah | Arti |
|---|---|
| `D` | cetak jarak terkumpul, keadaan rem, dan faktor skala |
| `D<cm>` | nolkan jarak, lalu pasang rem di `<cm>` (`<cm>` harus > 0) |
| `D0` | lepas rem **dan** nolkan jarak |
| `Ds<faktor>` | setel faktor skala slip, RAM saja (rentang sah 0,5 .. 1,5) |

`D0` sengaja berarti dua-duanya, bukan salah satu: memasang rem di 0 cm tidak punya arti, jadi tidak ada yang hilang, dan "lepas rem" hampir selalu diikuti keinginan menolkan jarak.

**Kenapa huruf `D`, walau `d` sudah dipakai.** Firmware ini memakai huruf besar-kecil untuk pasangan **simetris** (`f`/`F`, `g`/`G`), dan `d` sudah menjadi dump diagnostik. Konvensi itu tidak bisa dipenuhi di sini: setiap huruf besar yang masih bebas pasangan kecilnya sudah terpakai — tidak ada satu pun pasangan yang benar-benar kosong. Yang tersisa adalah memilih tabrakan yang paling tidak berbahaya, dan `D` memenuhi syarat itu: `D` tanpa angka hanya **mencetak**, dan `D<cm>` hanya memasang rem — tidak satu pun perintah `D` yang **menggerakkan** robot. Salah ketik `D` saat memaksudkan `d` tidak melakukan apa-apa yang berbahaya, yang persis alasan konvensi itu ada.

Rem jarak adalah **satu pemeriksaan di `loop()`**, dijalankan sesudah `nav.navUpdate()`: kalau rem terpasang dan `robot.jarakCm()` sudah melewati sasaran, panggil `nav.navBerhenti()` lalu `robot.stop()`.

Keduanya jalur berhenti yang sudah ada — `navBerhenti()` mengurus semua mode navigasi dan `return` sendiri bila sudah `NAV_DIAM`; `robot.stop()` mengurus `w` manual. Rem **hanya menolkan dan tidak pernah menulis vektor gerak**, sehingga doktrin "satu penulis" yang dijaga `navBerhenti()` di seluruh firmware tetap utuh.

Rem bersifat mode-agnostik dengan sengaja: `F` lalu `D80` mengukur slip saat menyusuri dinding, `D80` sendirian mengukur slip jalan lurus. Satu alat, dua pengukuran.

## 6. Faktor slip disimpan di RAM, bukan `Calib`

Odometer menghitung jarak geometris; selisihnya dengan jarak nyata adalah slip, dan itu butuh satu faktor skala. Tempat yang secara arsitektur benar adalah tabel `Calib`, tetapi tidak di iterasi ini:

**Menambah baris parameter memaksa kenaikan `CALIB_VERSION`.** `param[N_PARAMS]` berada di dalam `CalibBlob` (`Calib.h:36`), jadi menambah baris mengubah tata letak EEPROM, dan `Calib::load()` membuang blob yang versinya tidak cocok (`Calib.cpp:33`, sekarang versi 9). Seluruh gain yang sudah disetel — `wall.kp`, `wall.setpoint`, dan lainnya — hilang. Harga yang tidak sepadan untuk satu `float`, apalagi menjelang trial.

**Mendaur ulang slot `P_BELUM_DIPAKAI` adalah jebakan.** Lima slot mati tersedia (`head.utara`..`head.barat`, `arena.mirror`) dan mengganti namanya tidak mengubah tata letak, karena `PARAM_DEFS` ada di flash dan bukan bagian dari blob. Tetapi nilai tersimpan dibaca **per indeks**: blob lama menyimpan `0.0` di slot itu, sehingga slot yang diganti nama jadi `odo.skala` akan dimuat sebagai skala **nol** dan odometer diam-diam selalu membaca 0 cm. Ini persis kelas kegagalan senyap yang `EEMap.h` dibuat untuk mencegah.

Maka faktor skala hidup di RAM, disetel lewat `Ds<faktor>`, default `1.0` — mengikuti preseden `m8`/`m9` yang juga menyatakan "hanya di RAM". Begitu angkanya diketahui dari pengukuran lapangan, ia ditulis sebagai konstanta di `config.h`, di mana ia ikut flash dan tidak bisa hilang. Pemindahan ke `Calib` dilakukan hanya bila terbukti perlu berbeda per medan, sekalian dengan kenaikan versi yang direncanakan.

## 7. Penanganan galat

| Keadaan | Perilaku |
|---|---|
| Profil medan berubah (`profileCrouch`/`profileStairs`) | Rumus membaca `_prof.stepLength` tiap tick, jadi ramp-nya terikuti sendiri tanpa kode tambahan. |
| Servo lemas | Gait tidak `moving`, cabang `!moving` `return` sebelum akumulasi. Tidak ada yang bertambah. |
| Rem tertinggal terpasang | Rem dilepas otomatis saat menyala, dan ikut dilepas oleh `s`, `x`, Enter, dan `m0`. |
| Robot tersangkut | Odometer terus menghitung dan rem menyala terlalu cepat. Ini batas mati semua *dead reckoning* dan tidak bisa diperbaiki tanpa sensor luar; batas waktu misi tetap jadi jaring terakhir. |

## 8. Uji

`test-pc/sim/sim_odo.cpp`, didaftarkan di `build.sh`. Tiga bagian:

1. **Ikat ke angka yang sudah terverifikasi.** `sim_laju` menetapkan 10,7 cm/detik pada perintah maju 1,0. Jalankan gait 10 detik pada 1,0; odometer harus membaca 107 cm ±2%. Keduanya membaca `stepLength`/`cycleTime` yang sama, jadi selisih di luar itu berarti rumus barunya keliru. Uji dijalankan dengan `skala` = 1,0 supaya yang diuji rumusnya, bukan knob-nya.
2. **Faktor normalisasi benar-benar terpakai.** Maju 0,8 lurus, catat jaraknya; ulangi maju 0,8 sambil putar 0,5. Jarak kedua harus lebih pendek. Tanpa `f` kedua angka identik dan uji ini gagal — memang itu tujuannya.
3. **Rem berhenti di tempat.** Pasang rem 80 cm, jalankan sampai berhenti, periksa jarak akhir berada di 80..81 cm — batas atasnya satu tick loop pada laju penuh (10,7 cm/detik × 10 ms ≈ 0,1 cm, dibulatkan longgar) — lalu periksa vektor gerak benar-benar nol dan tetap nol 10 tick sesudahnya.
4. **Skala berlaku pada penambahan.** Jalankan 5 detik pada skala 1,0, catat; setel skala 0,5 lalu jalankan 5 detik lagi. Tambahan kedua harus separuh tambahan pertama, dan angka yang sudah terkumpul tidak boleh berubah saat skala disetel.

Pass-through `Hexapod::jarakCm()` tidak diuji: satu baris yang tidak bisa gagal diam-diam.

Rem jarak diuji di `sim_odo` pada tingkat `Hexapod`/`Navigation`, bukan lewat parser `.ino`, mengikuti pola `sim_pivot` — parser sudah punya cakupannya sendiri di `sim_param`.

## 9. Kriteria selesai

- `./build.sh` bersih, termasuk `sim_odo`.
- `D` mencetak jarak yang bertambah saat robot berjalan dan berhenti bertambah saat robot diam.
- `F` lalu `D80` menghentikan robot yang sedang menyusuri dinding; `D80` sendirian menghentikan robot yang berjalan lurus.
- Jarak menurut odometri bisa dibandingkan dengan meteran di arena, dan hasil perbandingan itu menjawab pertanyaan "perlu koreksi slip atau tidak".

## 10. Langkah sesudah ini

Hasil pengukuran menentukan iterasi berikutnya:

- Selisih kecil → tulis faktor skala ke `config.h`, selesai.
- Selisih besar atau berbeda antara jalan lurus dan menyusur dinding → rancang koreksi, dan barulah pertimbangkan memindahkan knob ke `Calib`.

Penyambungan ke `Mission` (mengganti pemicu ch2, atau menyediakan acuan untuk ruas sesudah tikungan pertama) adalah spec terpisah, sesudah ketelitiannya diketahui.
