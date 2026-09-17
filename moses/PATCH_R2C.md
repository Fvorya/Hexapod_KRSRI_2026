# Patch R2C atas v1.12 — 12 Sep 2026, disetel ulang 13 Sep 2026

Tiga perubahan di firmware Vincent. Semuanya kecil dan berdiri sendiri.

---

## 1. `sekuensAmbil` fase 4 → pose NETRAL (`R`), bukan GENDONG

```c
// sebelum
pose(robot, lengan, LENGAN_GENDONG_MM, lt + LENGAN_GENDONG_TGI, "gendong");

// sesudah
if (pose(robot, lengan, REHAT_JANGKAUAN, REHAT_TINGGI, "netral (R)"))
    robot.setPergelangan(lengan, REHAT_PERGELANGAN);
```

**Jebakan kerangka acuan yang hampir kena.** `REHAT_TINGGI` diukur dari
**bidang pusat badan**; fase 0–3 memakai tinggi dari **lantai** (lewat
`lt = -standHeight`). Jadi fase 4 **tanpa `lt`**. Menambahkannya menggeser pose
sebesar tinggi badan (100 mm DATAR / 115 mm TANGGA), melempar lengan ke luar
amplop jangkauan, dan `pose()` akan menolaknya — korban tertinggal menggantung
di pose jepit.

Bukan tebakan: `sekuensTaruh` fase 4 milik Vincent sendiri sudah memakai
`REHAT_JANGKAUAN, REHAT_TINGGI` **tanpa `lt`**. Pola yang sama diikuti.

Dipakai `pose()` (melaporkan kalau di luar jangkauan) alih-alih
`moveArmTarget()` langsung — pose yang gagal diam-diam di sini berarti korban
tertinggal menggantung.

---

## 2. `m8` — tunggu Raspi sebelum sekuens AMBIL. **BAWAANNYA NYALA.**

### Masalah yang ditutupnya

Diuji 12 Sep di arena: pemicu `#KORBAN AMBIL <ruas>` sampai, vision mendeteksi
(kotak abu-abu → hijau/merah), **robot tidak bergerak**, 1–2 detik kemudian
kotak abu-abu lagi, lalu **capit turun sendiri**.

Tiap bagiannya bekerja sesuai rancangan, dan hasil gabungannya tidak berguna:

1. v1.12 mengirim pemicu lalu **langsung** `masuk(MISI_LENGAN)` —
   kirim-lalu-lanjut, seperti komentarnya sendiri menyebut.
2. HUD menolak menggerakkan badan selagi sekuens lengan berjalan (dua penguasa
   untuk satu robot), jadi ia hanya **menonton**.
3. Vision mati 1–2 detik kemudian karena nama ruas firmware sudah berganti.

### Yang dikerjakan

Ruas `AKS_AMBIL` berhenti di `MISI_KONFIRM` **sesudah** mengirim pemicu. Raspi
meluruskan badan dengan kamera, menjawab `m2`, dan **baru** sekuens lengan
jalan. `m3` mengulang ruasnya. `m8` / `m8 1` / `m8 0` membalik atau memaksa.

### Kenapa bawaannya NYALA, padahal mula-mula MATI

Karena parkirnya sekarang punya **batas waktu 20 detik**
(`MISI_VISI_BATAS_MS`). Habisnya waktu **tidak menggagalkan apa pun**: sekuens
lengan jalan memakai posisi meja yang sudah diukur — persis perilaku v1.12
tanpa `m8`. Jadi `m8` NYALA berarti *"coba pakai vision; kalau tidak ada
jawaban, kerjakan cara lama"*, dan itu **tidak pernah lebih buruk daripada
MATI**. Korban tidak bisa hilang karena mode ini.

### 20 detik itu diturunkan dari anggaran HUD, bukan dipilih enak

Sisi HUD memakai **dua** state untuk pekerjaan ini, bukan satu:

| state HUD | jatah | isinya |
|---|---|---|
| `AMBIL_TENGAH` | 8 s | pivot kaki `O`, penengahan kasar sampai ≤ 8° |
| `AMBIL_HALUS` | 8 s | putar badan `r0 0 <yaw>` + dwell 2 detik |
| **jumlah** | **16 s** | |

Angka di firmware harus **lebih besar** dari jumlah itu. Kalau keduanya sama,
keduanya habis pada detik yang sama dan yang terjadi jadi lomba: capit turun
tanpa HUD pernah sempat mengatakan apa pun. Dengan 20 di sini HUD **selalu
bicara lebih dulu** — `GAGAL: batas waktu di state AMBIL_HALUS` — dan capit
baru turun 4 detik kemudian. Urutan itu yang membuat kejadiannya bisa dibaca.

⚠️ **Kalau salah satunya diubah, ubah keduanya.** Menurunkan angka firmware
sendirian mengembalikan lomba itu; menaikkan anggaran HUD sendirian juga.
Penjaganya ada di **bagian 69** `test_mission_hud.py`, jadi ketidakcocokannya
gagal di laptop, bukan di arena.

### Tabel `RUAS[]` TIDAK disentuh

Menyisipkan baris `AKS_KONFIRM` akan menggeser seluruh indeks sesudahnya, dan
bersamanya panjang ruas hasil `m7`, acuan "ruas 30", dan rentang `m4 0 29`.
Satu flag runtime tidak menggeser apa pun.

### Bagian yang paling mudah salah

`MISI_KONFIRM` sekarang dipakai **dua** hal dengan ujung berbeda: ruas
`AKS_KONFIRM` (→ `ruasBerikut()`) dan parkir vision (→ `MISI_LENGAN`).
`_parkirVision` membedakannya. Tanpa pembeda itu `m2` akan **melompati
pengambilannya** dan robot berjalan ke ruas berikutnya dengan capit kosong,
tanpa satu pun pesan yang menyebut kenapa. Logikanya ditransliterasi ke Python
dan diuji 19 kasus, termasuk kasus capit-kosong itu, `m2` yang datang telat,
dan habisnya batas waktu.

Batas waktunya **hanya** berlaku untuk parkir vision. Ruas `AKS_KONFIRM` biasa
menunggu keputusan MANUSIA, dan manusia tidak boleh di-timeout — ia mungkin
sedang mengukur sesuatu.

---

## 3. `GRIP_PERSEN_MIN` 5 → 10 — capit menutup ke 10%, bukan 0

Laporan R2C: pada 0–5% capit **menabrak dirinya sendiri**. Servo terus mendorong
ke posisi yang sudah tertahan mekanis, arus stall mengalir terus, dan
**motornya panas**. Servo yang stall lama akan rusak, dan selama itu ia menarik
arus besar di robot yang tegangannya sudah sempit (4,9 V sudah cukup membuat USB
macet dan bit throttle Pi melatch).

Angka itu dipakai **dua** tempat, dan itu yang membuatnya tempat yang benar
untuk diperbaiki:

1. `sekuensAmbil` fase 2 menutup capit ke `GRIP_PERSEN_MIN` → urutan otomatis
   ikut terlindungi.
2. perintah `g` di-clamp ke `GRIP_PERSEN_MIN..MAKS` → **`g0` yang diketik
   manual pun berhenti di 10%**.

Memperbaikinya hanya di sisi Raspi akan meninggalkan jalur kedua terbuka — dan
jalur kedua itu justru yang dipakai saat trial-and-error.

---

## Berkas yang disentuh

| berkas | perubahan |
|---|---|
| `config.h` | `GRIP_PERSEN_MIN` 5 → 10 |
| `Misi.h` | `setTungguVision()`, `tungguVision()`, `_tungguVision = true`, `_parkirVision` |
| `Misi.cpp` | fase 4 → REHAT; parkir + batas 20 s; `jawab()` dua ujung; reset di `mulaiDari`; baris status `tunggu visi` |
| `Hexapod_Unlimited.ino` | `case '8'` + dua baris bantuan |

## Cara memakai

Sesudah flash, **tidak ada yang perlu diketik** — `m8` sudah NYALA dan HUD
sudah bawaan `pemicu_korban = ambil_alih`. Jalankan misi seperti biasa.

Untuk kembali ke perilaku v1.12: `m8 0`.

Periksa keadaannya dengan `m` — ada baris baru:

```
  tunggu visi : NYALA  (SEDANG PARKIR -- menunggu 'm2')
```

## Yang BELUM diuji

**Belum pernah dikompilasi** — tidak ada `arduino-cli` + core Teensy di tempat
patch ini dibuat. Yang sudah diperiksa:

- Keseimbangan kurung **identik** dengan aslinya (selisih `(` vs `)` di
  `Misi.cpp` memang −9 **sebelum dan sesudah** — warisan komentar Vincent,
  bukan dari patch ini).
- Logika serah-terima `m2` + batas waktu: 19 kasus, ditransliterasi ke Python.
- Sisi Raspi: 767 tes lulus.

**Compile dulu sebelum flash:**

```
python3 ~/M/flash_teensy.py --versi R2C --hanya-compile
```

Itu berhenti sebelum menyentuh Teensy.
