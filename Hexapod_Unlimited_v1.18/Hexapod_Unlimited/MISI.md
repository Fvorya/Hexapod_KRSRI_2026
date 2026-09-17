# Misi — lintasan sebagai tabel

Lapisan misi kontes KRSRI/SAR UNLIMITED 2026. Ia **menyetir** `Navigation`,
tidak menggantikannya: vektor gerak tetap milik `navUpdate()` seorang diri.

Yang berbeda dari versi sebelumnya: **lintasan adalah data, bukan kode.**
Satu baris `RUAS[]` per potongan lintasan; mesin statusnya cuma lima
(JALAN, PIVOT, SETEL, LENGAN, KONFIRM). Menyetel arena = mengubah angka, bukan
mengubah alur.

> **Capit belum terpasang.** Tiap ruas korban sekarang hanya **berhenti kosong**
> 1,5 detik lalu lanjut. Sekuens lengan lengkapnya sudah ditulis utuh di
> `Misi.cpp` dalam blok komentar — tinggal ditukar begitu capit ada.

---

## 1. Kenapa tabel, bukan satu state per potongan

Versi sebelumnya menulis satu `case` per potongan lintasan: 12 state untuk
**seperempat** rute (HOME → K-1 → lantai pecah → turunan). Rute penuh
guidebook 2026 punya 22 potongan. Diteruskan dengan pola yang sama, itu jadi
~60 state yang masing-masing **menyalin** logika penjaganya sendiri.

Penjaga yang disalin adalah penjaga yang suatu saat lupa disalin, dan itu
sudah terbukti sekali di versi lama: `berjalan()` menyebut empat state dan
lupa lima state ruas berjarak, sehingga **`m0` di tengah lantai pecah
diam-diam tidak melakukan apa pun** — navigasi tidak dihentikan, tidak ada
yang tercetak, seluruh pengawasan `ruasSehat()` mati, dan `abaikanDepan`
tetap menyala. Robot berjalan buta tanpa ada yang mengawasi.

Di sini `berjalan()` ditulis sekali sebagai "state apa pun selain
DIAM/SELESAI/GAGAL", jadi state baru ikut terhitung tanpa ada yang perlu
ingat.

---

## 2. Isi satu baris tabel

```c
{ "R-6 jalan pecah 45x55", BLK_LURUS, KMD_TENGAH, PRF_TANGGA, true, HNT_ODO, 55, AKS_TIDAK_ADA, 0 },
//  nama                   ^          ^           ^           ^     ^        ^   ^              ^
//                      belok         kemudi      profil      buta  henti    cm  aksi ujung     lengan
```

| Kolom | Arti |
|---|---|
| `belok` | **belok masuk, relatif terhadap ruas sebelumnya**: `BLK_LURUS` / `BLK_KANAN` / `BLK_BALIK` / `BLK_KIRI`. Mata angin mutlaknya dihitung, lihat §2b. Mesin **memutar badan sendiri** — pivot bukan baris tabel |
| `kemudi` | `KMD_KANAN` / `KMD_KIRI` ikut dinding satu sisi; `KMD_TENGAH` garis tengah lorong (selisih kiri−kanan) |
| `profil` | `PRF_DATAR` / `PRF_TANGGA` / `PRF_MERUNDUK` / `PRF_SEMPIT` |
| `buta` | `true` = sensor depan **diabaikan** sepanjang ruas ini |
| `henti` | `HNT_ODO` odometri gait · `HNT_DEPAN` LiDAR depan ≤ nilai · `HNT_BELAKANG` LiDAR belakang ≥ titik nol + nilai · `HNT_LANGSUNG` ruas ini hanya aksi · `HNT_SISI` **geser menyamping** sampai dinding sisi = nilai; sisinya dibaca dari kolom `kemudi` |
| `cm` | arti tergantung `henti`. **`-1` = belum diukur, misi menolak berangkat** |
| `aksi` | `AKS_TIDAK_ADA` · `AKS_AMBIL` · `AKS_TARUH` · `AKS_KONFIRM` |

## 2b. Kompas: belok relatif, mata angin dihitung

Arah tiap ruas **tidak** ditulis sebagai mata angin mutlak. Yang ditulis
adalah beloknya, lalu mutlaknya dihitung dengan rumus yang sama persis dengan
`Navigation::arahGeser()`:

```
arah[i] = (arah[i-1] + belok[i]) % 4,   arah[0] = MISI_ARAH_BERANGKAT = 0
```

Dua alasan, keduanya soal mapping:

1. **Yang salah saat mapping selalu satu tikungan, bukan satu ruas.** Kalau
   ternyata belokan di ujung lorong itu ke kiri dan bukan ke kanan, dengan
   mata angin mutlak semua ruas sesudahnya ikut salah dan harus diketik ulang.
   Dengan belok relatif, mengubah satu baris memperbaiki sisa lintasan sendiri.
2. **Yang bisa dilihat orang di arena adalah tikungan, bukan mata angin.**
   "di ujung R-4 belok kanan" bisa dicocokkan sambil berdiri di sana.

Jangkarnya diambil dari misi adik tingkat (`Mission.cpp` @ `Ver1_8`), jadi
tiga ruas yang sudah pernah dia jalankan keluar dengan angka yang sama persis
— bukan ditulis ulang, tapi **hasil hitungan**:

| Ruas | belok | arah hasil | konstanta miliknya |
|---|---|---|---|
| 0 HOME → samping K-1 | `BLK_LURUS` | UTARA (0) | `MISI_ARAH_AWAL = 0` |
| 1 K-1 | `BLK_KIRI` | BARAT (3) | `MISI_ARAH_KORBAN1 = 3` |
| 2 R-1 jalan pecah | `BLK_KANAN` | UTARA (0) | pivot balik ke lorong |

Dijaga oleh `cek_tabel_misi.py` di akar repo — jalankan tiap kali tabelnya
disentuh:

```
python cek_tabel_misi.py
```

Ia mencetak mata angin hasil hitungan untuk seluruh ruas, menandai mana
yang belum diukur, lalu memeriksa ketiga jangkar di atas plus invarian tabel
(ruas buta wajib odometri, ambang depan di atas `FRONT_STOP_CM`, pemicu
belakang di dalam jangkauan sensor). Tidak butuh robot maupun compiler.

### Profil KAIL — bentuk yang tidak seragam (ruas 24, R-9)

Empat profil lama (`DATAR`/`TANGGA`/`MERUNDUK`/`SEMPIT`) semuanya **seragam**:
`computeHome()` menulis `z = -standHeight` untuk keenam kaki. Karena itu
"kaki depan naik ke tapak di atas, kaki belakang memanjang ke tapak di bawah"
**tidak bisa** ditulis dengan `STAND_HEIGHT` berapa pun — bukan karena
angkanya kurang, tapi karena satu angka tidak bisa berarti dua tinggi
sekaligus. Yang dibutuhkan **offset per kaki**: `HexaGait::setOffsetKaki()`,
di-ramp dengan `GAIT_PROFILE_TAU` yang sama dan ikut dihitung `profilTenang()`.

`setProfile()` **menghapus** offset itu, jadi tidak ada profil biasa yang bisa
lupa membersihkannya; `profileKail()` memasangnya lagi sesudahnya.

| Konstanta (`config.h`) | Arti | Nilai |
|---|---|---|
| `KAIL_TINGGI_BADAN` | tinggi badan profil ini — **bukan** 115 milik TANGGA | 100 mm |
| `KAIL_DEPAN_MAJU` | kaki 0 & 5 membuka ke depan (mengait) | 40 mm |
| `KAIL_DEPAN_NAIK` | kaki 0 & 5 **naik** ke tapak di atas | 40 mm |
| `KAIL_BELAKANG_TURUN` | kaki 2 & 3 **memanjang ke bawah** | 35 mm |

Kaki tengah tidak disentuh: ia yang menanggung badan.

**Selisih depan-belakang itulah yang mendatarkan badan**, bukan kaki belakang
yang ikut naik — yang justru menjungkitkan hidung. Sudut yang dikompensasi =
`atan((40 + 35) / 156)` = **25,7°**, dengan 156 mm jarak pangkal kaki depan ke
belakang. Bidang R-9 terukur 27,3°, jadi hampir penuh.

**Batasnya jangkauan IK, dan ia ditukar 1:1 dengan tinggi badan** — inilah
sebabnya KAIL tidak memakai tinggi badan TANGGA. Diukur `cek_kail.cpp`, tiap
baris satu siklus gait penuh sambil berjalan:

```
badan  90 mm -> belakang turun sampai 55 mm (bidang 31,3 der)
badan 100 mm -> turun sampai 45 mm (bidang 28,6 der)   <-- yang dipakai
badan 110 mm -> turun sampai 35 mm (bidang 25,7 der)
badan 115 mm -> turun sampai 30 mm (bidang 24,2 der)   <-- TANGGA, mentok
```

Pada 115 mm jatahnya habis sebelum sudut bidangnya tercapai. Pada 100 mm sisa
jatahnya 45 dan yang dipakai 35 — **10 mm kelegaan**, karena kemiringan
berubah sepanjang tanjakan (terukur 5,9 → 13,0 → 24,9 → 27,3°) dan kaki yang
mentok tidak sampai ke titik yang diperintahkan.

> 100 mm bukan penurunan kelegaan: sampai 8 Sep ruas ini berprofil `DATAR`
> yang tinggi badannya memang 100, dan dengan itu robot sempat menempuh 62
> dari ~103 cm.

`T4` memasangnya sambil robot berdiri — **lihat bentuknya di meja dulu**.

**Yang belum dibuktikan sama sekali:** apakah bentuk ini benar-benar menaiki
R-9. Profil mengubah **bentuk**, bukan **urutan langkah** — gait tripod tetap
berjalan, jadi kaki depan yang sudah mengait **tetap terangkat** pada
gilirannya. Kalau yang dibutuhkan kail yang *menahan* sementara kaki belakang
mendorong, itu sekuens gerak (`AKS_*`), bukan profil, dan harus disetel di
robot.

### Laju geser: kasar selagi jauh, halus saat mendekat

Sumbu geser **terkuantisasi satu langkah gait** — badan tidak merayap, ia
melompat `2 × laju × stepLength` tiap siklus. Itu sebabnya laju geser dulu
dipatok 0,25: lompatan 30 mm (DATAR) adalah resolusi yang membuat sasaran
13 cm bisa dikenai. Harganya terukur, dan mahal:

| profil | 0.25 | 0.50 | 0.80 | 1.00 |
|---|---|---|---|---|
| DATAR | 3,3 | 6,7 | 10,7 | 13,3 cm/s |
| TANGGA | 3,2 | 6,4 | 10,2 | 12,7 cm/s |
| MERUNDUK | 2,0 | 4,1 | 6,5 | 8,2 cm/s |
| SEMPIT | 2,2 | 4,5 | 7,2 | 9,0 cm/s |

Linear sempurna — **kakinya bukan batasnya**, angka perintahnya yang menahan.

Sekarang lajunya dipilih dari **sisa jarak ke sasaran**, bentuknya sama persis
dengan rem maju terhadap `NAV_PELAN_CM`: `RATA_LAJU_JAUH` 0,80 selagi sisa ≥
`RATA_PELAN_CM` 15 cm, turun linear, dan **kembali ke 0,25 begitu dekat** —
jadi ketelitian akhirnya tidak berubah sama sekali. Penggaris bisu (sisa tidak
terukur) juga memakai yang pelan: jangan melempar badan cepat ke arah yang
tidak terukur.

Diukur lup-tertutup dengan LidarArray asli, termasuk keterlambatan median-3
(`cek_geser.cpp`), geser 40 → 13 cm:

```
DATAR     4,0 detik (cara lama  8,1), berhenti di 12,2 cm
MERUNDUK  6,4 detik (cara lama 13,2), berhenti di 12,9 cm
```

Dua kali lebih cepat, dan melesetnya **di bawah satu lompatan gait**.

> Geser MANUAL tidak pernah terbatas: `w <maju> <geser> [detik]` memakai angka
> Anda sendiri. `w0 0.8 2` = geser kanan ~10,7 cm/detik selama 2 detik.

### Ganti profil → tunggu, tapi hanya kalau ruasnya membaca sensor

Mengganti profil gait **menggerakkan badan**: `standHeight` beda 35 mm antara
DATAR dan TANGGA, `standRadius` 25 mm ke SEMPIT. Selama badan turun atau naik,
seluruh berkas LiDAR ikut berayun — yang depan menyapu naik-turun di dinding,
yang samping menjauh lalu mendekat tanpa robot berpindah sesenti pun. Pemicu
jarak yang dibaca di tengah ayunan itu mengakhiri ruas di tempat yang salah.

Karena itu ruas yang berhenti pada **sensor** (`HNT_DEPAN`, `HNT_BELAKANG`,
`HNT_SISI`) dan datang dengan profil **berbeda** dari ruas sebelumnya berhenti
dulu di state `SETEL`: navigasi dihentikan, robot berdiri diam, dan ruasnya
baru berangkat sesudah dua syarat berurutan terpenuhi.

1. **Ramp profil selesai** — ditanyakan ke gait (`gaitProfilTenang()`), bukan
   ditebak dari jam, jadi ia ikut kalau `gait.profile_tau` disetel. Diukur
   dengan `cek_setel_profil.cpp`: **840–1140 ms** untuk keempat peralihan
   profil pada `tau` baku 0,25 detik.
2. **Histori median LiDAR terisi ulang** — 3 sampel penuh sesudah badan diam,
   `3 × NUM_LIDAR × LIDAR_PERIOD_MS` = 450 ms.

Jadi ongkosnya **~1,3–1,6 detik**, dan cuma di ruas yang benar-benar
membutuhkannya. Pada tabel sekarang itu **dua ruas**: ruas 12 (MERUNDUK,
meratakan diri ke dinding kanan) dan ruas 31 (DATAR sesudah R-11 yang SEMPIT).

Ruas `HNT_ODO` **tidak** menunggu, dan itu disengaja: aturan "ruas beruntun
disambung tanpa berhenti" ada untuk menjaga robot tidak tersendat di bibir
rintangan, dan odometri tidak peduli berkas sensor sedang berayun. Kemudi
dinding samping memang ikut membaca LiDAR selama tunggu, tapi ia mengoreksi
terus-menerus — satu sampel miring diperbaiki sampel berikutnya. Pemicu jarak
tidak punya kesempatan kedua.

### Acuan posisi dipilih per ruas

Ini gagasan terpenting yang dibawa dari versi sebelumnya. Tidak ada satu cara
mengukur posisi yang benar di seluruh arena:

- **Sensor belakang** untuk K-1. Korbannya duduk di ceruk **di samping**
  lintasan (guidebook hal. 25: lebar 40 cm, kedalaman 15 cm), jadi sensor
  depan tidak akan pernah melihatnya. Yang bisa diukur adalah jarak ke
  dinding START. Titik nolnya dicatat sendiri **sesudah** pivot, karena badan
  yang masih menyerong membuat berkasnya memanjang 1/cos(sudut).
- **Odometri gait** untuk ruas kasar. Di jalan pecah, tangga, dan bidang
  miring tidak ada acuan mutlak yang searah jalan; LiDAR depan mendatar tidak
  melihat lantai pecah maupun bibir turunan. Slip odometri diukur di lantai
  arena 2026-09-02 dan hasilnya 1,0 — tidak ada koreksi yang perlu dipasang.
- **LiDAR depan** untuk ruas yang berakhir di dinding atau safe zone.

### `buta` selalu berpasangan dengan odometri

Ruas yang mengabaikan sensor depan berjalan **buta**. Itu perlu di dua tempat:
di ruas kasar, halangan depan membuat mode arena **pindah mata angin** dan
sisa jaraknya akan diukur ke arah yang salah; di turunan, berkas sensor depan
menembak lantai (pada dudukan ~10 cm dan kemiringan 14°, terbaca ~40 cm lalu
mengecil — navigasi membacanya sebagai halangan yang mendekat).

Karena itu ruas buta **wajib** dibatasi `HNT_ODO`. `tabelSiap()` menolak
berangkat kalau ada baris yang melanggar — itu salah tulis, bukan salah setel.

---

## 3. Rute — dirombak 6 September 2026 (malam)

Tabelnya **33 baris**. Yang menentukan bentuknya sekarang bukan lagi gambar
guidebook melainkan operator yang berdiri di arena; guidebook sudah meleset
lima kali dan kalah tiap kali arena membantahnya.

```
HOME → K-1 → R-1 → M1/R-2/R-3 → maju ke tembok → R-4/SZ-1
     → BARAT lewat K-2, TEMBUS R-5 → SZ-2
     → SELATAN sedikit → GESER KANAN → SELATAN sampai tembok K-3
     → GESER KIRI → maju ke samping K-3 → pivot BARAT → angkat K-3
     → TIMUR sampai tembok → SELATAN lewat R-6 → SZ-3
     → geser + maju ke K-4 → angkat K-4 → jalan ke depan tangga
     → ratakan 20 cm ke dinding kanan → R-9 (TANGGA)
     → R-10 → geser kiri → SZ-4 → geser kanan → K-5 → R-11 → SZ-5/FINISH
```

Cek silang yang membuat rantai belok ini bisa dipercaya: tangga tetap jatuh
di **SELATAN**, arah yang sudah diverifikasi di arena. Ada tujuh belokan di
antara R-4 dan tangga; satu saja keliru, tangganya tidak lagi jatuh di sana.

**Diukur di arena** (`m6 <idx>` lalu `m7 <idx> <cm>`): ruas 0, 2, 3, 5, 9,
11, 13, 15, 20, 25.
**Dari geometri guidebook:** ruas 24 (103 = √(90²+50²)).
**Masih `-1`, dan `m1` penuh menolak berangkat sampai diisi:** ruas 30 (R-11).
**Bukan panjang, jadi `m6` tidak berlaku:** ruas `HNT_SISI` (7, 12, 17, 23,
26, 28) adalah jarak ke dinding samping — coba dengan `V<cm>`; ruas
`HNT_DEPAN` (4, 14, 18, 22, 31, 32) adalah jarak berhenti yang dipilih, bukan
diukur.

> ⚠ **Nomor ruas di blok komentar `Misi.cpp` sudah tidak sinkron** dengan
> tabelnya — tabel dirombak dari arena beberapa kali dan catatannya tidak ikut
> dinomori ulang. Isinya masih benar; cari berdasarkan **nama** ruas.

> ⚠ **Enam ruas berhenti pada sensor depan** dan sensor itu punya hantu yang
> belum terjelaskan: di satu titik yang sama, menghadap SELATAN ia membaca
> "kosong" 340 kali berturut-turut, menghadap BARAT ia membaca benda padat
> yang sangat mantap di 8,6 cm — di tempat yang operator pastikan kosong.
> Kalau salah satunya berhenti seketika tanpa ada tembok, itu hantunya: ganti
> ke `HNT_ODO` dengan `buta`.

> ⚠ `FRONT_STOP_CM` **turun 20 → 12** pada 9 September 2026: operator
> memastikan bacaan depan 10-15 cm masih aman. Ambang `HNT_DEPAN` wajib **di
> atas** konstanta itu, jadi batas bawahnya sekarang **13 cm**, bukan 21.
> Jangan turunkan `FRONT_STOP_CM` lebih jauh tanpa memindahkan
> `LIDAR_MIN_CM[depan]` (7 cm) — di bawah 7 cm bacaan diumumkan "kosong",
> bukan "mepet", dan pita 7..12 cm itulah satu-satunya tempat halangan depan
> masih bisa dikenali.

Catatan lengkap per ruas — dari mana tiap angka datang, apa yang sudah dicoba
dan gagal — ada di **blok komentar kepala `Misi.cpp`**. Di situ, bukan di sini,
angka baru ditulis.

---

## 4. Menyambung ke robot

Flash lewat Arduino IDE seperti biasa, lalu jalankan **`jalankan.bat`** di akar
repo — ia mencari sendiri port Teensy-nya dan menyambung di 115200 baud.
Kalau nomornya sudah tahu: `jalankan.bat COM5`.

Arduino IDE dan `jalankan.bat` **tidak bisa memegang port yang sama**. Tutup
Serial Monitor Arduino IDE sebelum menjalankannya, dan tutup jendelanya
sebelum upload berikutnya.

> Firmware menerima CR, LF, maupun CRLF sebagai satu akhir baris. Ini bukan
> sekadar kerapian: sebelum diperbaiki, terminal ber-CRLF membuat **tiap
> perintah langsung disusul rem darurat** — CR menjalankan perintahnya, LF
> masuk sebagai Enter kosong. Yang terlihat adalah robot yang menerima
> perintah lalu berhenti sendiri tanpa sebab. Enter kosong yang sungguhan
> tetap jadi rem darurat.

---

## 5. Yang perlu dipanggil di arena

Berurutan. Tiap langkah punya angka yang harus benar sebelum lanjut.

### a. Sekali saat robot menyala

```
b                 berdiri (servo hidup) — boot-nya lemas dengan sengaja
l                 tabel LiDAR: KEENAM channel harus memberi angka
                  ch2 (BELAKANG) WAJIB hidup — dia yang mengukur K-1
```

### b. Kalibrasi yang tanpa itu misi MENOLAK jalan

```
C                 kalibrasi arah putar (sekali saja)
S                 simpan hasilnya ke EEPROM 2048
K                 periksa: pivot sudah terkalibrasi?

c0 c1 c2 c3       catat kompas arena: hadapkan badan ke tiap arah lalu ketik
e                 simpan kompas ke EEPROM 1792
k                 periksa: keempat arah sudah tercatat?
```

`c0` adalah **arah berangkat dari HOME**, bukan utara magnet. Sisanya
searah jarum jam dari situ.

### c. Mapping — mengisi panjang ruas

```
m4                cetak TABEL LINTASAN. Kolom cm bertanda '?' = belum diukur
m7 <idx> <cm>     setel panjang satu ruas, misal:  m7 2 55
```

Ulangi `m7` sampai `m4` tidak menyisakan satu pun `?`. Semua hanya di RAM —
belum ada slot EEPROM untuk parameter misi, jadi **diulang tiap robot
menyala**.

### d. Menguji per rintangan (sebelum lari penuh)

```
m4 <idx>          MULAI dari satu ruas saja, bukan dari HOME
```

Ini yang dipakai untuk menyetel satu rintangan tanpa mengulang seluruh
lintasan. Letakkan robot di mulut ruas itu, hadapkan kira-kira ke arahnya
(misi memutar sendiri sisanya), lalu jalankan.

### e. Lari penuh

```
m1                MULAI dari ruas 0 (HOME)
m                 status: ruas ke berapa, sudah berapa cm, sensor apa yang ditunggu
m0                batalkan
```

### f. Rem darurat — semuanya menghentikan misi DAN navigasi

```
Enter (kosong)    rem darurat
s                 stop
x                 LEMAS — PWM mati, servo bebas
```

---

## 6. Daftar perintah misi

| Perintah | Arti |
|---|---|
| `m` | status misi |
| `m4` | cetak tabel lintasan (arah hasil hitungan + beloknya) |
| `m1` | mulai dari ruas 0 |
| `m4 <idx>` | mulai dari ruas `<idx>` |
| `m0` | batalkan |
| `m7 <idx> <cm>` | setel panjang ruas |
| `m2` / `m3` | saat `AKS_KONFIRM`: lanjut / ulangi ruas ini |

---

## 7. Penjaga yang berjalan sepanjang ruas

Semuanya berhenti dengan **sebab yang tercetak**, bukan diam-diam:

| Penjaga | Kapan berbunyi |
|---|---|
| navigasi diambil alih | mode navigasi bukan lagi milik misi — menangkap `f`/`F`/`p`/`o`/`C` **dan** navigasi yang berhenti sendiri, sekaligus |
| batas waktu ruas | 90 detik satu ruas |
| batas waktu kontes | 300 detik total (hal. 35: bonus = skor × 300 / waktu) |
| pindah mata angin | navigasi memilih arah lain — 90° sekaligus, bukan goyangan; sisa ruas akan diukur ke arah yang salah |
| serong bertahan | > 25° selama 3 siklus gait. Ambangnya turunan dari kerugian odometri (jarak nyata = terbaca × cos θ; pada 25° itu 9%), bukan selera |
| IMU hilang | `simpangArah()` mengembalikan NaN |
| servo dilemaskan | saat sekuens lengan, menunggu konfirmasi, atau menunggu profil gait |
| ramp profil macet | 5 detik di state `SETEL` tanpa badan pernah tenang — gait tidak di-update |

### Dua hal yang TIDAK lagi langsung menggagalkan misi

**Pivot masuk ruas yang tidak sampai diulang sekali** sebelum menyerah.
Percobaan kedua berangkat dari heading yang sudah lebih dekat dan mendapat
jatah `PIVOT_BATAS_MS` yang baru, jadi pivot yang cuma **kehabisan waktu**
(kaki terganjal, lantai licin, badan sempat mentok dinding) sering selesai di
situ. Yang tidak ditolong: kompas belum tercatat, IMU lepas, atau `c0..c3`
yang memang salah — ketiganya meleset sebesar yang sama dua kali. Simpangan
kedua percobaan **dicetak dalam derajat**, jadi log-nya yang membedakan mana
yang terjadi. Jatahnya per **ruas**, bukan per pivot.

**Sensor sisi yang bisu memakai pasangannya.** Tiap sisi punya dua LiDAR
(dudukan depan dan belakang), dan `ratakanMulai()` dulu menolak berangkat
kalau dudukan depan tidak memberi jarak — satu kanal bisu membuang seluruh
misi. Sekarang ia pindah ke dudukan belakang dan mencetak peralihannya.
`jarakGeser()` memang sudah lama memperlakukan sepasang itu sebagai satu sisi.

> ⚠ Harganya ketelitian: kedua dudukan ada di titik yang berbeda sepanjang
> badan, jadi kalau robot menyerong terhadap dinding, sasaran cm itu diukur
> dari dudukan belakang. Dan **fail-closed tetap berlaku** — kalau
> dua-duanya bisu, perataan tetap ditolak dan robot tidak bergerak
> sesenti pun. Kedua sifat itu dikunci `cek_penggaris_sisi.cpp`.

Yang **belum** ikut: kalau penggaris mati di TENGAH perataan, ia tidak pindah
sendiri — perataan berjalan sampai penjaga "pita terlalu dekat" atau
`RATA_BATAS_MS` menghentikannya. Dibiarkan begitu dengan sengaja: pindah
penggaris di tengah gerak bisa berarti mengejar dinding yang lain.

`gagal()` selalu mengembalikan `abaikanDepan(false)` dan `setTengah(false)` —
termasuk di jalur tempat navigasi sudah berhenti duluan, yang justru jalur
kegagalan tersering dan tempat `navBerhenti()` memilih diam.

---

## 8. Kalau capit sudah terpasang

> **Kedua lengan TIDAK sama** (dikonfirmasi 7 Sep 2026). **DEPAN 4 servo** —
> bahu, siku, pergelangan, grip; pergelangan adalah sudut ketiga yang disetel
> sendiri, di luar IK (`a<jkn> <tgi> [prg]`). **BELAKANG 1 servo** — grip saja.
> `moveArmTarget()` menolak lengan belakang: letak capitnya ditentukan letak
> **badan**, jadi ruas yang memakainya (K-3 di ruas 16, SZ-3 di ruas 19) -- kini keduanya memakai lengan DEPAN harus
> berhenti tepat di posisi angkat — tidak ada sendi yang bisa mengoreksi.

1. Ukur ulang `ARM_ORIGINS` di `config.h` — sekarang masih ditandai
   *"ANGKA 50 MASIH PERKIRAAN"*.
2. Di `Misi.cpp`, tukar `sekuensAmbil()` / `sekuensTaruh()` yang kosong
   dengan versi lengkap di blok komentar tepat di bawahnya.
3. Hidupkan servo lengan: `robot.armEnable(ARM_DEPAN, true)` — servo lengan
   **default mati** karena lengan belum tentu terpasang.
4. Perhatikan tinggi lepas yang berbeda: SZ-1/SZ-3 sejajar lantai, tapi
   **SZ-2 dan SZ-4 tingginya 4 cm** dari lantai (hal. 25, 28).

### Konsekuensi skor selama capit belum ada

Menurut hal. 34–35: rintangan tetap dinilai **100** per rintangan alih-alih
**150** "membawa korban", dan penempatan di safe zone (50, atau **100** di
SZ-5) hangus. Melewati seluruh lintasan tanpa korban **tetap sah dan tetap
berskor**.

Yang **tidak** boleh (hal. 21): kembali mengambil korban sesudah rintangan
berikutnya dilewati — pengangkatannya tidak sah. Karena itu ruas korban tetap
duduk di tempatnya di tabel walau isinya masih kosong; urutannya yang dijaga,
bukan cuma isinya.
