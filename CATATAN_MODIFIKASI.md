# Catatan arsitektur modifikasi — v1.18

Hasil inspeksi 18 Sep 2026. Berkas ini untuk didiskusikan dulu, baru dikerjakan.

Temuan pokoknya satu kalimat: **hampir semua yang "belum ada" sebenarnya sudah
ada mesinnya, yang hilang saklarnya.** Enam fungsi/field sudah ditulis,
dikompilasi, dan tidak pernah dipanggil siapa pun. Jadi bagian terbesar dari
"melengkapi firmware" bukan menulis fitur baru, melainkan menyambungkan yang
sudah terlanjur dibayar.

---

## STATUS — PROMPT 1 SUDAH DIKERJAKAN (18 Sep 2026)

Seluruh teks di bawah ditulis **sebelum** ada perubahan kode. Sesudahnya
PROMPT 1 dari `PROMPT_KERJA.md` dikerjakan; statusnya di sini, dan yang
ternyata berbeda dari dugaan ditulis apa adanya, bukan dirapikan.

| Butir PROMPT 1 | Status | Keterangan |
|---|---|---|
| 1. Profil waktu loop | **SELESAI** | Baris `PROF` nyata: `n/avg/max/min ms`, `util` terhadap `CONTROL_HZ`, dan `lambat50`. Komentar `CONTROL_HZ` diperbaiki supaya jujur: ia BUKAN pembatas laju. |
| 2. Konsol `Y` (`Yo`/`Yi`/`Yj`/`Yz`) | **SELESAI** | Tanpa menyentuh tata letak EEPROM. Rinciannya di bawah. |
| 3. Laju gambar OLED | **DILEWATI** | Butuh robot menyala untuk membaca baris `PROF`; hipotesis A2 tidak boleh dijawab dengan tebakan. Tampilan.cpp **tidak disentuh**. |
| 4. Deteksi terguling | **SELESAI** | Tiga ambang di `config.h` + saklar, ditandai BELUM DIUKUR. |
| 5. Offset LiDAR `Yd` | **SELESAI** | RAM saja, dikurangkan di satu tempat sesudah median+EMA. |

### Butir 2 — apa yang nyatanya berbeda dari dugaan

- **`Yo` dan `Yi` punya tombol simpan yang BERBEDA, dan itu bukan pilihan
  gaya.** `gOffset[]` duduk di `CalibBlob` (EEPROM 0, disimpan `W`),
  sedangkan `gInvert[]` duduk di `ServoMap` (EEPROM 1024, disimpan `YtW`).
  Menulis invert ke alamat 0 memang **tidak berpengaruh apa pun**, karena
  `loadServoMap()` menimpanya tiap boot. Jadi teks bantuan menyebutkan
  keduanya terpisah, bukan menyatukannya supaya enak dibaca.
- **`Yj` tidak bisa menjangkau grip depan, dan itu bukan bug yang bisa
  diperbaiki dari sisi perintah.** `TUNE_PIN_MAP` hanya memuat 3 servo per
  lengan; `{0,15}` (grip depan) tidak ada di sana. Dibiarkan apa adanya dan
  ditulis di komentar + teks bantuan, karena menambahkannya berarti mengubah
  tabel pin yang dipakai `jog()` — di luar lingkup batch ini.
- **`Yz` menerima nilai dalam keadaan servo HIDUP** (tidak ditolak seperti
  `Yi`). Sebabnya praktis: `zOff` justru disetel untuk membetulkan kaki yang
  menggantung, dan itu cuma terlihat saat robot berdiri. Harganya: perubahannya
  terasa seketika tanpa ramp.
- **`cetakOffset()` memakai awalan `#OFFSET` yang baru, bukan menumpang
  `cetakTrim()`.** Pilihan yang disebut pertama di usul ternyata yang berisiko:
  menambah kolom di tabel `#TRIM` akan menggeser jumlah kolom yang dicocokkan
  HUD Raspi. Jadi yang diambil opsi kedua, dan `cetakTrim()` tidak disentuh
  sama sekali.

### Butir 5 — catatan yang perlu dibaca sebelum angka itu dipercaya

Pencatatan **ditolak** kalau sensornya `MATI` atau `JAUH`. Itu disengaja dan
termasuk lingkup, bukan penambahan: offset yang dicatat dari bacaan tak sah
akan menggeser **semua** jarak sensor itu tanpa satu pun gejala di layar.

Ia **RAM saja**, jadi ia hilang tiap robot menyala. Yang menutupnya adalah
batch `CALIB_VERSION` (usul B, prompt 2) — 6 baris `PARAM_DEFS` tambahan,
kesempatan gratis yang sama.

---

## 0. Yang sudah ada tapi MATI (tidak ada pemanggil)

Diverifikasi dengan grep di seluruh pohon **sebelum** PROMPT 1:

| Yang mati | Ada di | Dipanggil dari |
|---|---|---|
| `Hexapod::jog(tuneId, pulseUs)` | `Hexapod.cpp:280` | **tidak ada** |
| `Imu::tare()` | `Imu.h:19` | **tidak ada** |
| `gOffset[24]` (offset sudut per servo) | `Calib.h`, dipakai di jalur servo | **tidak ada penulis** — nol selamanya |
| `GerakStore::lvlR/lvlP/refR/refP/jac[4]` | `EEMap.h:78` | **tidak ada pembaca** |
| `Hexapod::setStabilization()` | `Hexapod.h:25` | dikomentari |
| `stab.tau`, `stab.sign_*` | `PARAM_DEFS` | `P_BELUM_DIPAKAI` |
| `head.utara/timur/selatan/barat`, `arena.mirror` | `PARAM_DEFS` | `P_BELUM_DIPAKAI` |

**Tiga di antaranya sudah hidup sejak 18 Sep 2026**, dan itu menghapus tiga
baris dari tabel di atas: `jog()` kini dipanggil `Yj`; `gOffset[]` akhirnya punya
penulis, perintah `Yo`; dan `Imu::accelZ()` — yang tidak masuk tabel ini tapi
sama-sama hanya dicetak — sekarang bertindak lewat deteksi terguling.
`Imu::tare()` masih mati dan akan tetap begitu sampai `Yl` (usul C) diputuskan.
`GerakStore::lvlR/lvlP/refR/refP/jac[4]` masih belum dibaca siapa pun, dan
`Yz` justru **diwajibkan** mempertahankannya (baca-ubah-tulis).

`gOffset[]` yang paling mahal hilangnya — lihat bagian 3.

---

## 1. Tinggi langkah

### Keadaan sekarang

`GaitProfile::stepHeight` ada, di-ramp `GAIT_PROFILE_TAU`, dan berlaku sambil
berjalan. Yang tidak ada: cara menyetelnya sebagai **satu knop**.

- `Qgait.step_height <mm>` mengubah **dasar** yang dipakai keenam profil, dan
  bertanda `P_PERLU_B` — baru masuk saat `b`, dan `b` sekaligus menolkan pose
  badan serta mengembalikan profil ke DATAR.
- `T1` (+35) dan `T5` (75) menuliskannya keras.
- `b<mm>` sudah membuktikan polanya bisa: ia menyalin profil DATAR lalu
  mengganti **satu kolom** (`standHeight`). Tidak ada padanannya untuk empat
  kolom yang lain.

### Usul

Perluas `T` jadi keluarga berawalan, meniru `b<mm>` dan `Yt`:

```
Th<mm>   tinggi langkah      Tl<mm>   panjang langkah
Tc<ms>   waktu siklus        Tr<mm>   radius kaki      Tb<mm>  tinggi badan
```

Isinya: ambil `robot.gaitProfile()`, ganti satu kolom, pasang lagi. ~5 baris
per huruf, memakai `setGaitProfile()` yang sudah publik.

**Satu jebakan yang harus diputuskan.** `setGaitProfile()` → `pasangProfil()`
→ `HexaGait::setProfile()` **menghapus offset kaki**. Jadi mengetik `Th60`
saat T4/T5 aktif akan meratakan bentuk KAIL diam-diam — persis kelas bug yang
selama ini dijaga di firmware ini. Dua jalan:

- **(a)** tambah `HexaGait::setKolomProfil()` yang tidak menyentuh `_tgtOff`
  (2 baris), dan `T?<x>` memakai itu. Invarian "satu pintu pemasangan profil"
  tetap utuh karena ini bukan pemasangan profil, melainkan penyetelan kolom.
- **(b)** `T?<x>` menolak jalan kalau ada offset kaki aktif, suruh `T0` dulu.

Saya condong ke **(a)** — penyetelan tinggi langkah paling dibutuhkan justru
di R-9, tempat offset kaki memang harus tetap hidup.

---

## 2. Kontrol manual & gerak tiap arah

### Keadaan sekarang, diverifikasi

| Sumbu | Perintah | Catatan |
|---|---|---|
| maju / mundur | `w<maju> …` | terbuka, −1..1 |
| geser kiri/kanan | `w… <geser>` | terbuka, tapi **terkuantisasi satu siklus** |
| geser satu siklus | `H<amp>` | kuantisasi dijadikan satuan |
| geser berumpan-balik | `V<cm>` / `V-<cm>` | acuan LiDAR sisi |
| maju/mundur berumpan-balik | `J<cm>` | acuan LiDAR belakang |
| putar di tempat | `o<0-3>` / `O<der>` | PD, non-blokir |
| **putar SAMBIL jalan (serong/membusur)** | **tidak ada** | — |

Sumbu putar sudah ada di `Hexapod::walk(maju, geser, putar)` dan di
`HexaGait::setMoveVector()` lengkap dengan normalisasi vektor langkah
(`HexaGait.cpp` butir 4, yang justru ditulis untuk melindungi servo saat maju
sambil berputar). Navigasi memakainya di `Navigation.cpp:1626`. Yang tidak
punya jalan masuk hanya **operator**: ketiga pemanggil `walk()` di `.ino`
mengisi sumbu putar `0.0f`.

Jadi "gerak serong/membusur" bukan fitur yang belum ada — ia sumbu yang belum
ada tombolnya.

### Usul

`w` sekarang: `w <maju> <geser> [detik]`. Sumbu putar ditaruh di argumen
**keempat**, bukan ketiga:

```
w <maju> <geser> [detik] [putar]
```

Jelek dibaca, tapi `w 0.5 0 3` yang sudah ada di README dan di kepala orang
tetap berarti sama. Menukar urutannya lebih rapi tapi mengubah arti perintah
yang sudah dipakai — bukan harga yang sepadan. Perubahannya 1 baris.

### Yang SENGAJA tidak saya usulkan

- **Mode kemudi menerus (tahan-jalan).** Semua gerak manual sekarang dibatasi
  0,5–30 detik, dan batas itu satu-satunya penjaga saat `jarakArahJalan()`
  mengembalikan −1 (penjaga buta). Membuka mode menerus membuang penjaga
  terakhir demi kenyamanan mengetik. `s`/Enter sudah menghentikan kapan saja.
- **Gait wave/ripple.** Tripod sudah terbukti di arena. Wave menaruh 4–5 kaki
  di tanah dan hampir separuh lebih lambat; kasus sulitnya (tangga) sudah
  dijawab T5.
- **Gamepad/PS2.** Serial + HUD Raspi sudah jadi jalur operator.

---

## 3. Kalibrasi — ini gap terbesarnya

### Inventaris jujur

| Besaran | Bisa disetel saat firmware jalan? | Tersimpan? |
|---|---|---|
| pulse min/max global | ya, `Qpulse.*` | EEPROM 0 |
| trim per servo (us) | ya, `Yt<slot> <us>` | EEPROM 1024 |
| gain PD, gait, condong | ya, `Q…` + `W` | EEPROM 0 |
| kompas arena | ya, `c0..3` + `e` | EEPROM 1792 |
| pivot der/siklus | ya, `C` — **tapi MEMBLOKIR** | EEPROM 2048 |
| **offset sudut per servo (der)** | **tidak** | field ada, nol selamanya |
| **invert per servo** | **tidak** | baca saja dari EEPROM 1024 |
| **zoff telapak per kaki** | **tidak** | baca saja dari EEPROM 2048 |
| **skala odometri** | `Ds`, **RAM saja** | tidak |
| **bias sudut dinding** | `Y0`, **RAM saja** | tidak — "ulangi tiap menyala" |
| **odometri geser (mm/siklus)** | **tidak** | `H` menyuruh ukur sendiri lalu simpan di Raspi |
| **panjang link kaki** | **tidak**, `#define` | butuh flash |
| **datum sudut lutut** | **tidak ada knopnya sama sekali** | — |
| **rata badan dari IMU** | **tidak**, `tare()` mati | field EEPROM ada, tak dibaca |
| **HAND_LENGTH, KORBAN_TINGGI_MM** | **tidak**, `#define` | keduanya diakui tebakan di CLAUDE.md |

Tiga yang paling menyakitkan bergabung jadi satu: **geometri kaki salah
terukur** (CLAUDE.md: berdiri 75 mm bukan 100, telapak 80 mm bukan 70 — dua
kesalahan menumpuk, panjang link dan datum lutut ±24 der), **tidak ada satu
pun knopnya di runtime**, dan penutupnya menuntut membandingkan busur derajat
dengan kolom `sdeg` milik `d`. Artinya satu putaran ukur–koreksi = satu flash.

### Usul A — konsol kalibrasi `Y`, TANPA menyentuh tata letak EEPROM

`Yt` (18 Sep) sudah membangun polanya: huruf kedua memilih subperintah, dan
`Y` dipilih justru karena salah ketik di situ tidak membuat robot berjalan.
Teruskan pola itu — dan keempat yang di bawah **tidak** mengubah `CalibBlob`
sama sekali, jadi tidak ada kalibrasi yang terbuang:

```
Yo<slot> <der>   offset SUDUT per servo   -> gOffset[], sudah di jalur servo,
                                             sudah ikut disimpan 'W'
Yi<slot> <0|1>   invert per servo         -> gInvert[], tolak saat servo hidup
Yj<slot> <us>    jog pulse MENTAH         -> Hexapod::jog(), sudah ada
Yz<kaki> <mm>    offset tinggi telapak    -> baca-ubah-tulis EEPROM 2048,
                                             pola sama dengan gerakSimpan()
```

`Yo` itu jawaban untuk **datum lutut**: `gOffset[]` memang persis "koreksi
datum dalam derajat", sudah dipakai di `Hexapod.cpp:468` dan `HexaArm.cpp:88`,
sudah ikut `Calib::save()`, dan tidak pernah punya penulis. Bedanya dengan
`Yt`: trim itu mikrodetik (gigi horn), offset itu derajat (datum sudut). Dua
pekerjaan berbeda, satu sudah bertombol satu belum.

`Yj` menghidupkan kembali tuner servo mentah tanpa flash sketsa lain — persis
keluhan yang melahirkan `Yt`.

### Usul B — satu kali kenaikan `CALIB_VERSION`, dirangkai

Sisanya menuntut baris baru di `PARAM_DEFS`, dan itu menaikkan
`CALIB_VERSION` → **seluruh kalibrasi tersimpan terbuang**. Karena itu
kerjakan **sekali, dalam satu batch**, jangan satu per satu.

Dibuang (5 baris, semuanya `P_BELUM_DIPAKAI`):
`head.utara`, `head.timur`, `head.selatan`, `head.barat`, `arena.mirror`

Ditambah (11):

| nama | dari | kelas |
|---|---|---|
| `leg.coxa`, `leg.femur`, `leg.tibia` | `#define` di config.h | `P_SERVO_LEMAS` |
| `stand.height`, `stand.radius` | `#define` | `P_PERLU_B` |
| `odo.skala` | `Ds`, RAM | `P_LANGSUNG` |
| `odo.geser` | belum ada; mm/siklus untuk `H` | `P_LANGSUNG` |
| `wall.bias_kiri`, `wall.bias_kanan` | `Y0`, RAM | `P_LANGSUNG` |
| `arm.hand_len` | `HAND_LENGTH` | `P_LANGSUNG` |
| `korban.tinggi` | `KORBAN_TINGGI_MM` | `P_LANGSUNG` |

Net +6 baris, `param[]` tumbuh 24 byte; EEPROM 0..1023 masih longgar.

**Biayanya nyaris nol di sisi kode.** `Calib.h` sudah memakai pola
`#define GAIT_STEP_HEIGHT gParam[K_…]`, jadi memindahkan `FEMUR_LENGTH` cukup
memindahkan barisnya dari `config.h` ke `Calib.h`. **Tidak satu pun tempat
pemakaian berubah** — sudah saya periksa: tak ada yang memakainya di
inisialisasi statis.

**Prosedur migrasinya sudah ada dan tidak perlu ditulis:** `q` mencetak semua
parameter dan menandai `*` yang berbeda dari default. Foto layarnya sebelum
flash, ketik ulang yang bertanda `*` sesudahnya. Hanya itu yang hilang.

### Usul C — rata badan dari IMU (perlu keputusan)

`setStabilization()` dikomentari, `stab.*` menganggur, `tare()` mati, dan
`GerakStore::jac[4]` — yaitu d(roll,pitch IMU)/d(perintah roll,pitch), persis
kalibrasi yang dibutuhkan lingkar rata badan — ditulis TES_GERAK dan tidak
pernah dibaca siapa pun. Jadi lingkarnya pernah dirancang utuh lalu tidak
disambung.

Versi paling malas yang berguna: `Yl` = `tare()` + satu saklar yang meneruskan
−roll/−pitch IMU ke `setBodyRotation()` dengan `STAB_TAU` dan `STAB_MAX_DEG`
yang sudah ada. Tanpa `jac[]` dulu — anggap gain 1:1, biarkan `stab.sign_*`
yang membalik tandanya.

**Ini satu-satunya usul di berkas ini yang benar-benar fitur baru, dan
satu-satunya yang bisa menjatuhkan robot kalau tandanya terbalik.** Karena itu
ia tidak saya masukkan ke urutan kerja sampai kamu memutuskan. Pertanyaan
sebenarnya: di tangga, apakah badan yang ikut rata itu menolong, atau justru
mencuri jangkauan IK kaki depan yang sudah 95% terpakai?

---

## 4. Yang TIDAK saya usulkan, dan alasannya

- **Kerangka kalibrasi baru.** `PARAM_DEFS` + `Q`/`W` sudah kerangkanya. Tiap
  knop yang hilang = satu baris tabel, bukan satu subsistem.
- **Sensor sentuh telapak / adaptasi medan.** Robot ini tidak punya sensornya.
  Tanpa itu, "adaptasi medan" cuma tebakan berumpan-balik dari IMU.
- **Perencana lintasan / keseimbangan dinamis.** Tabel `Ruas[]` 33 baris sudah
  jadi perencananya, dan ia bisa dibaca orang.
- **Kalibrasi `C` dijadikan non-blokir.** Ia memblokir, dan itu satu-satunya
  yang tersisa — tapi ia dijalankan sekali di meja, bukan di arena. Biarkan.
- **Offset LiDAR per sensor.** `WALL_BIAS_*` global kiri/kanan sudah cukup
  sampai ada bukti tiap sensor menyimpang sendiri-sendiri.

Catatan atas permintaan "super duper lengkap": tiap knop baru adalah satu
angka lagi yang bisa salah disetel di arena, dan tiap baris `PARAM_DEFS`
adalah satu risiko lagi `CALIB_VERSION` naik. Daftar di atas saya batasi pada
yang **sudah terbukti dibutuhkan** oleh catatan pengukuranmu sendiri di
`CLAUDE.md` dan `config.h`.

---

## 5. Urutan kerja yang saya usulkan

Status per 18 Sep 2026 ditulis di ujung tiap butir.

1. **`Yo` / `Yi` / `Yj` / `Yz`** — tidak mengubah EEPROM, tidak membuang apa
   pun, langsung membuka jalur ukur–koreksi tanpa flash. Kerjakan lebih dulu
   karena butir 2 akan LEBIH MUDAH sesudahnya.
   → **SELESAI.** `Yd` menyusul di batch yang sama (RAM saja, jadi sekaligus).
2. **Ukur geometri kaki** dengan busur derajat pada `b100`, pakai `Yo` untuk
   datum lutut. Ini penutup yang sudah diminta `CLAUDE.md`.
   → **MENUNGGU PENGUKURAN.** Knop-nya sudah ada; angkanya belum.
3. **Batch `CALIB_VERSION`** (usul B) — sekali, sesudah angka geometri
   diketahui, supaya defaultnya langsung benar dan yang perlu diketik ulang
   sesudah wipe tinggal sedikit.
   → **BELUM**, dan tetap begitu sampai butir 2 selesai — ia membuang
   kalibrasi tersimpan. Enam baris `PARAM_DEFS` untuk offset LiDAR ikut di sini.
4. **`Th`/`Tl`/`Tc`/`Tr`/`Tb`** + argumen putar di `w`. Dua-duanya kecil,
   taruh terakhir karena tidak memblokir apa pun.
   → **BELUM.** Argumen putar di `w` sudah lebih dulu ada di pohon (bukan dari
   batch ini); `Th`/`Tl`/… belum.
5. **`Yl` rata badan** — hanya kalau kamu memutuskan iya di bagian 3C.
   → **BELUM**, dan keputusannya belum diambil. Satu-satunya usul di berkas ini
   yang bisa menjatuhkan robot kalau tandanya terbalik.

Uji: butir 1 dan 4 tidak punya program uji di repo ini (`test-pc/` tertinggal
di `program-krsri-misi`). Butir 3 menyentuh `Calib` — `cek_koreksi.cpp` tidak
me-link firmware, jadi ia tidak menangkapnya. Yang menjaga keempatnya cuma
`arduino-cli compile`. Kalau mau lebih dari itu, satu program uji di PC untuk
`Calib::setParam` + rentangnya layak ditulis bersama butir 3.

---

## 6. Catatan tambahan: huruf perintah sudah habis

Ke-52 huruf besar-kecil terpakai. Yang tersisa di switch tingkat atas hanya
digit `1`–`9` (digit `0` sudah dipakai "nolkan pose badan"). Karena itu semua
usul di atas berbentuk **awalan** di huruf yang sudah ada (`Y…`, `T…`), bukan
huruf baru — meneruskan keputusan yang sudah diambil saat `Yt` dibuat, dan
bukan menambah konvensi kedua.
