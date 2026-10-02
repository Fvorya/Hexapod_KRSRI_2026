# CLAUDE.md — Handoff

Repo `Hexapod_KRSRI_2026`, branch `hexapod-v1.18`. Hexapod KRSRI / SAR
UNLIMITED 2026, Teensy 4.1 + Raspberry Pi 5.

- Firmware: `Hexapod_Unlimited/` — satu-satunya pohon yang di-flash. Folder
  `Hexapod_Unlimited_v1.18`, `_v2.2`, `Hexapod_KRSRI_2026-hexapod-v2.0*` dan
  `program-krsri-misi/` adalah salinan lama di luar git; jangan disunting.
- HUD Raspi: `moses/` (lihat `moses/README_HUD.md`).
- Alasan tiap angka ada di komentar `config.h`; arti kolom tabel lintasan di
  `Misi.h`. Catatan ini cuma keputusan yang tidak terlihat dari kode.

---

## Verifikasi sebelum commit

Lima pemeriksaan, semuanya tanpa robot:

```sh
arduino-cli compile -b teensy:avr:teensy41 --warnings all Hexapod_Unlimited
python cek_tabel_misi.py          # tabel lintasan, mata angin, invarian
python cek_lengan_laju.py         # jatah waktu sekuens lengan
export PATH="/c/msys64/ucrt64/bin:$PATH"
g++ -std=gnu++17 -Wall -Wextra -IHexapod_Unlimited -o /tmp/cek_koreksi \
    cek_koreksi.cpp && /tmp/cek_koreksi
python moses/test_mission_hud.py  # 1151 pemeriksaan HUD
```

`arduino-cli` ada di `C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\`.

**Perubahan yang mestinya tidak mengubah perilaku** (komentar, rename) bisa
dibuktikan: firmware dibangun deterministik dan tidak ada `__LINE__`/`__DATE__`
di sumbernya, jadi `--output-dir` dua kali lalu bandingkan md5 berkas `.hex`.
Identik = tidak satu instruksi pun berubah.

`cek_geser.cpp`, `cek_lengan.cpp`, `cek_penggaris_sisi.cpp`,
`cek_setel_profil.cpp` masih ter-track tapi TIDAK bisa dibangun di repo ini —
stub-nya (`test-pc/stub/`) tertinggal di `program-krsri-misi`.

Jebakan yang berulang:

- **Akhir baris**: LF di repo, CRLF di disk (`core.autocrlf=true`). Suntingan
  berbasis jangkar multi-baris harus memakai akhir baris yang ada di disk.
- **Heredoc** memakan satu garis miring terbalik walau dikutip. Pakai
  `chr(92)` di Python.
- **Git Bash mengubah argumen berawalan `//`** jadi path (`/...`). Untuk
  argumen yang memang berisi komentar C++, pakai `MSYS_NO_PATHCONV=1`.
- **Serial Monitor Arduino IDE memegang port.** Upload dari skrip gagal dengan
  `Access is denied` selama ia terbuka.

---

## Keputusan arsitektur

1. **Misi adalah tabel data**, bukan state per potongan. `RUAS_BAKU[]` di
   flash, `RUAS[]` di RAM (disunting HUD lewat `m5*`). `SKOR_RUAS` ikut
   bergeser saat baris disisip/dihapus.

2. **Kegagalan misi**: sebab lunak (pivot meleset, batas waktu, heading hilang,
   perataan ditolak) membuat ruas **dilewati** tanpa poin. Yang membatalkan
   cuma servo lemas, waktu kontes habis, dan LiDAR mati. LiDAR putus dipindai
   ulang sekali tanpa reset bus (`pindaiI2C(false)`); kalau pulih, ruas
   **dilanjutkan** — `_lanjutRuas` dihabiskan sekali di kepala `ruasJalan()`.

3. **Arena cermin** lewat tiga pengakses `belokRuas()`/`kemudiRuas()`/
   `putarRuas()`, bukan tabel kedua. `_arah[]`/`_serong[]` adalah cache;
   `segarkanArah()` menghitungnya ulang saat saklar berubah (sebelum itu
   pivot tetap memakai tabel asli). Tombol cermin dikunci selama misi.

4. **Jenis henti yang mirip tapi berbeda**: `HNT_BELAKANG` (jarak tempuh dari
   titik nol) vs `HNT_MUNDUR` (jarak mutlak ke dinding belakang, lantai
   `MUNDUR_MIN_CM`, dilewati kalau belakang JAUH); `HNT_SISI` (berhenti pada
   bacaan dinding) vs `HNT_GESER` (berhenti pada odometri geser, tanpa
   dinding); `HNT_PUNCAK` (gyro: mendaki lalu datar, relatif ke pitch awal).

5. **`pasangProfil()` satu-satunya pintu profil**, dan ia menolkan pitch badan
   serta kunci lutut. TANJAK memasang pitch-nya sesudahnya.

6. **Lengan memakai sudut sendi tetap** (`KORBAN_*` di `config.h`, satuan
   `as`), bukan IK — datum bahu masih diragukan. `HexaArm::angleToPulse()`
   meng-clamp **diam-diam**; fitur lengan baru harus memeriksa jangkauannya
   sendiri. Fase sekuens berganti murni dari jam (`LENGAN_JEDA_MS`), jadi
   menaikkan slew tidak mempercepat misi.

7. **Kemudi MENENGAH proporsional**, bukan bang-bang — di tengah lorong derau
   membalik "dinding mana yang lebih dekat".

8. **`putar` mutlak dan menumpuk**: acuannya mata angin hasil `belok`; sesudah
   satu ruas menyerong, ruas `BLK_LURUS` berikutnya tetap menyerong.
   `kunciHeading()` mencegah mode arena memilih mata angin yang salah pada
   serong 45 der.

9. **Perintah dirangkai** supaya urutannya tidak bergantung ketikan: `U<cm>`
   (naik tangga) dan tombol D3 tahan (`I`, `b`, `R` — `R` ditolak selama servo
   lemas, jadi `b` harus duluan).

---

## Yang menunggu

### Geometri kaki salah, terukur 14 Sep 2026, belum diperbaiki

Pada `b100` robot berdiri 75 mm (bukan 100), telapak 80 mm dari sumbu coxa
(bukan 70). Dua kesalahan menumpuk: panjang link (55/65 terukur vs 80/90
tertulis) dan datum lutut ~+24 der. Membetulkan satu saja memindahkan robot ke
tempat ketiga yang juga salah. Penutupnya: busur derajat pada `b100`
dibandingkan kolom `sdeg c/f/t` milik `d`. Tabelnya di `config.h` di atas
`COXA_LENGTH`.

### Belum diukur

- `PUNCAK_DATAR_MS` (puncak tangga), `TERGULING_*`.
- `MUNDUR_MIN_CM` untuk kaki **belakang** (angkanya dari kaki tengah).
- Skala odometri **geser** — `HNT_GESER` memakai skala yang dikalibrasi maju.
- Panjang rahang capit (di luar poros grip, belum masuk `HAND_LENGTH`).
- Tinggi poros coxa dari tanah di T0; tegangan catu servo lengan (DS3225 6,8 V).
- Arah fisik LiDAR ch0 dan ch2 (`l`).

### Ketidakcocokan yang diketahui

- **Bukaan capit**: HUD `capit_buka_persen` 20, firmware `KORBAN_GRIP_BUKA` 30.
  Keduanya menggerakkan capit yang sama.
- **Bantuan `h` di firmware basi**: baris `U` masih menyebut T4+Z1, "Capit
  belum terpasang" masih tercetak, dan `m5*`/`m8`/`m9` tidak tercantum.
  Mengubahnya mengubah biner — perlu flash.
- **Docstring `mission_hud.py`** masih menyebut mode BACA/KENDALI yang sudah
  dihapus.
- **Ruas K-5 (AMBIL `HNT_LANGSUNG`)** tidak didahului ruas pendekat;
  `cek_tabel_misi.py` memperingatkannya.
- **Keluar TANJAK → SEMPIT langsung** tidak merehatkan lengan (sisa dari saat
  SEMPIT memakai bentuk TANJAK). Tabel sekarang tidak punya urutan itu.

### Kode yang hampir mati

- Kunci lutut KAIL (`Hexapod::kunciLutut`, `_lututKunci`, parameter
  `kunciLututDeg` di IK): `_lututKunci` tidak pernah diisi selain 0, jadi
  jalurnya selalu NAN. Saklarnya (`KAIL_LUTUT_KUNCI`) sudah dihapus.
- `SEMPIT_PITCH_DEG/RADIUS_KAKI/TINGGI_BADAN` di `config.h` tidak dibaca
  firmware sejak `profileNarrow()` disetel ulang; disimpan sebagai catatan.
- Slot Calib `K_HEAD_*` dan `K_STAB_TAU` tidak dipakai, tapi jangan dihapus:
  menggeser enum mengubah tata letak blob EEPROM 0.

### Lain-lain

- `RUAS_MAKS 40`, tabel 26 baris. Plafon keras 254 (`uint8_t`, 255 penanda).
- Pi 5 mencatat under-voltage saat boot; kamera menyala sepanjang misi.
