# Arena & aturan KRSRI/Unlimited SAR 2026 — ringkasan kerja

Diambil dari **`Guidebook SAR.pdf`** (folder `LEGACY2026-main`). Ini bukan
pengganti guidebook — ini yang **dipakai kode**, supaya angka di
`mission_hud.py` dan `config.h` bisa dilacak ke sumbernya.

> ⚠️ **Hasil technical meeting terakhir mengalahkan dokumen ini.** Guidebook
> sendiri menulis bahwa TM terakhir jadi acuan terakhir aturan. Cek ulang
> sebelum lomba.

---

## 1. Angka yang langsung dipakai kode

| Besaran | Nilai | Dipakai di |
|---|---|---|
| Alas arena | 3,6 × 2,4 m | — |
| **Lebar lorong** | **45 cm** | `wall.setpoint 17`, `wall.min 13` (v1.7+) |
| Tebal dinding | 2 cm | — |
| Tinggi dinding | 10 cm | gerbang tinggi bbox `bbox_h_min/max` |
| Waktu | **300 detik** | anggaran seluruh FSM |
| Korban asli | oranye, ≥ 33 g | kelas `korban` di YOLO |

Model 3D korban: `https://kontesrobotindonesia.id/data/2021/korban.stl`

**Lorong 45 cm itu penting.** Vincent menurunkan `wall.setpoint` 19 → 17 dan
`wall.min` 15 → 13 justru setelah lebar sebenarnya diukur — sim semula memakai
60 cm. Di lorong 45 cm **kedua sisi mengikat sekaligus**, jadi yang dicari bukan
"sejauh mungkin dari dinding yang diikuti" tapi setpoint yang celah tersempitnya
paling besar.

---

## 2. Ruang korban — ini yang menentukan desain vision

### K-1
- Ruang **40 × 15 cm**
- **2 dummy + 1 asli**, di tiga tanda silang
- Korban asli **menyerong 45°** kiri/kanan
- Posisi **diundi** setelah robot di Home dan power menyala

Dari sinilah seluruh konsep "slot" lahir: tiga boneka berjajar dalam 40 cm, dan
kamera hanya akurat pada satu boneka sekali lihat.

### K-2
- Area **15 × 28 cm**, setinggi lantai dasar
- **1 dummy + 1 asli**, asli menyerong 45°
- Titik tengah boneka **7,5 cm** ke tanggul / dinding belakang
- Dua titik penempatan dari samping masing-masing **8 cm**
- Posisi diundi

### K-3 dan K-4
- Ditimpa **dua papan 14 × 17 cm, tebal 2 cm** (ada coakan)
- Korban serong **45°**
- Titik tengah korban ke sisi R6: **9 cm**
- Titik tengah **K-4** ke sisi R8: **13 cm**
- Titik tengah **K-3** ke dinding ruang kosong/kanan: **10 cm**
- Ketinggian K-3/K-4 = tinggi tanggul R6 = **1 cm** dari lantai
- **Ada dummy tambahan di antara K-3 dan K-4**

Tertimpa papan + serong 45° = kasus terburuk untuk vision. Prior "tepat satu
asli per ruang" paling menolong di sini.

### K-5
- Diundi **khusus Nasional**

---

## 3. Safety Zone

| Zona | Ukuran | Ketinggian | Catatan |
|---|---|---|---|
| SZ-1 | 20 × 20 cm | lantai | di ruang R4 (45 × 60 cm), R4 = jalan berpuing |
| SZ-2 | 20 × 20 cm | **4 cm** dari lantai | |
| SZ-3 | 20 × 20 cm | sejajar lantai | **harus dibersihkan dulu** (R7) |
| SZ-4 | 20 × 20 cm | **4 cm** | |
| SZ-5 | lubang **12 × 7 cm** | di bidang miring | dinding samping & belakang 10 cm; belakang 7 cm dari lantai |

**SZ-3 punya aturan khusus:** ruang harus bersih dari koral sebagai syarat
penyelamatan K-3 sah. **Robot dilarang memakai boneka untuk mendorong batu** —
pembersihan harus oleh bagian tubuh robot sendiri (kaki, badan, lengan).
Kalau tidak bersih seluruhnya, **penyelamatan di SZ-3 tidak dinilai sama sekali**.

---

## 4. Rintangan

| Kode | Jenis | Ukuran |
|---|---|---|
| M1 | jalan miring | horizontal 80 cm, vertikal 20 cm |
| M2 | **tangga** | horizontal 90 cm, vertikal 50 cm; anak tangga tinggi 2 cm, lebar 3,6 cm |
| M3 | jalan miring | horizontal 50 cm, vertikal 5 cm |
| R4 | berpuing (koral putih 3–5 cm) | ruang 45 × 60 cm, diapit tanggul 2 × 2 cm |
| R5 | berlumpur (kelereng 15–17 mm) | 45 × 45 cm, 2 lapis, diapit tanggul 2 × 2 cm |
| R6 | jalan pecah | 45 × 55 cm, diapit tanggul lebar 2 cm tinggi 1 cm |
| R8 | ruang | 73 × 51 cm (toleransi 1–2 cm), tidak termasuk SZ-3 |
| R10 | miring + puing + lumpur | lebar 45 cm, panjang 50,2 cm miring / 50 cm horizontal |
| R11 | jalan longsor | lebar jalan yang bisa dilalui **30 cm** (50−10−10) |

Jalan pecah: penyangga tiang diameter 3–4 cm, tebal 10–12 mm; bawahnya tumpukan
circle tebal 2 cm diameter 3–4 cm.

Jarak sisi luar tanggul R6 ke sisi luar tanggul R5: **52 cm**.
Lantai dicat aquaproof supaya tidak licin.

---

## 5. Urutan wajib

> Robot harus melalui lintasan **berurutan**. Contoh guidebook: robot harus
> mencoba mengangkat K-1 **sebelum** melewati R1 menuju R2. Kalau sudah
> melewati batas R1/R2 lalu kembali mengambil K-1, **pengangkatan tidak sah**.

Ini alasan tabel `MISI` di `mission_hud.py` berurutan dan tidak boleh diacak.

⚠️ **BELUM DIVERIFIKASI:** pasangan **K-4/SZ-4** dan **K-5/SZ-5** di tabel `MISI`
masih bisa dibaca dua cara dari panah peta guidebook. Salah pasang =
penyelamatan **tidak sah**. Cocokkan ke gambar peta sebelum lomba.

---

## 6. Penilaian

| Capaian | Nilai |
|---|---|
| Keluar Home | 50 |
| Mengangkat korban keluar area korban (seluruh badan di jalur utama, **diangkat**) | 50 |
| Melewati rintangan R1–R11 (selain R7, R9) **tanpa** korban | 100 |
| Melewati rintangan R1–R11 (selain R7, R9) **membawa** korban | 150 |
| Melewati **R9** | 150 tanpa korban, **300** membawa korban |
| Menaruh seluruh badan korban di SZ-1..SZ-4 | 50 |
| Menaruh seluruh badan korban di **SZ-5** | **100** |
| Bersihkan SZ-3 (R7) — area sekitar peletakan | 100 |
| Bersihkan SZ-3 (R7) — **seluruh** area | 200 |

Tiap rintangan hanya dinilai **1×**, diambil nilai terbaik.

Untuk R4, R5, R8, R10: walau robot belum sepenuhnya keluar rintangan, kalau
berhasil menaruh korban di Safety Zone-nya, robot tetap mendapat nilai "membawa
korban" — asal akhirnya berhasil keluar rintangan.

**Bonus** (5 misi + finish): `total × 300 / waktu_detik`.
Membawa korban bernilai **1,5×** tanpa korban, jadi menyeberangi rintangan
sambil menggenggam korban selalu lebih berharga daripada bolak-balik.

---

## 7. Aturan yang mengikat cara kita menjalankan robot

**Aktivasi.** Dua tahap: nyalakan power (saat diminta juri, **sebelum** ditaruh
di Home), lalu jalankan robot dengan **satu tombol/switch start/run ditekan 1×**.
Kalau 3 detik robot belum bergerak, boleh tekan **tombol yang sama** 1× lagi.
Lebih dari itu = robot dianggap gagal dijalankan. **Sound activation dilarang.**

→ Boot Raspi **tidak** di jalur kritis: power menyala sebelum robot ditaruh.
→ Tombol start **jangan toggle** — tekan kedua harus MENGULANG start, bukan stop.

**Otonom.** Robot harus berjalan otonom, **tidak boleh ada interaksi peserta**
selama misi. Jadi tombol stop hanya untuk latihan dan keselamatan, bukan untuk
dipencet saat bertanding.

**Orientasi awal.** Robot menghadap **salah satu dinding sesuai permintaan juri**.
→ Inilah sebabnya perlu saklar SET mirroring, dan kenapa arahnya tidak boleh
di-hardcode di firmware.

**Diam 10 detik.** Di Finish, stopwatch berhenti ketika robot berhenti dan diam,
divalidasi **10 detik**. Berhentinya harus dari logika robot sendiri.

**Retry.** Hanya 1×, syaratnya robot sudah keluar Home dan diam/bergerak di satu
tempat **lebih dari 10 detik**. Waktu total tetap 300 detik; waktu untuk bonus
dihitung sejak retry berjalan ulang dari Home. Wajib angkat tangan dan **dapat
persetujuan juri** sebelum menyentuh robot.

**Hitungan mundur juri.** Kalau robot diam/bergerak di tempat karena error lebih
dari 10 hitungan, pertandingan dihentikan dan peserta diberi kesempatan retry
kalau belum terpakai. Aturan hitung mundur ini **tidak berlaku mulai R5** dan
seterusnya.

**Persiapan.** Peserta merapikan jalan pecah/korban/puing, lalu **mengacak
rintangan, koral dan kelereng 2–3 kali** sesuai arahan juri, dan memastikan
safety zone tertutup puing/koral. Satu sesi 5 menit, persiapan ~2 menit,
**tidak ada arena latihan**.

**Format.** Pertandingan antara 2 tim. **K-3, K-4 dan K-5 diperebutkan** kedua
tim.

**Robot harus** bergerak sendiri tanpa operator dan tanpa garis penuntun,
membuat keputusan sendiri, dan **tidak terpengaruh parameter pengganggu ruangan
seperti warna arena dan sorotan cahaya**.

→ Kalimat terakhir itu argumen terkuat untuk pencahayaan sendiri di robot
(LED putih), bukan mengandalkan lampu hall.

---

## 8. Yang masih harus diukur sendiri di arena

Guidebook tidak menyebutkannya, dan semuanya menyentuh kode kita:

- **`m8`** — jarak tempuh Home → korban 1, diukur sensor BELAKANG
- **`m7`** — lebar lantai pecah (odometri gait)
- **`m6`** — panjang turunan (odometri gait)
- **`capit_cm`** — jarak berhenti untuk mencapit; default 12 cm masih **tebakan**
- **Batas jarak vision** — seluruh arsitektur slot berdiri di atas asumsi
  "akurat hanya di 20 cm". Kalau ternyata masih benar di 40 cm, seluruh ceruk
  K-1 muat satu frame dan geser 8 cm tidak perlu lagi. Uji di 20/30/40/50/60 cm
  × asli/dummy × serong kiri/kanan.
