# Deployment YOLOv8n Deteksi Korban/Dummy pada Raspberry Pi 5

Catatan eksperimen — optimasi inferensi *real-time* untuk KRSRI.

**Tanggal:** 10 Agustus 2026
**Perangkat:** Raspberry Pi 5, 8 GB RAM, Debian 13 (trixie), kernel 6.12.47, Python 3.13.5
**Kamera:** Logitech C922 Pro Stream (`/dev/video0`, UVC)
**Model:** YOLOv8n, 2 kelas (`0: dummy`, `1: korban`), dilatih pada dataset Roboflow
`korban-dummy-detection-ejors` (4098 latih / 171 validasi)
**Runtime:** onnxruntime 1.28.0 (CPUExecutionProvider), OpenCV 5.0.0, tanpa PyTorch

---

## 1. Ringkasan Hasil

Seluruh angka berikut diukur pada **konfigurasi akhir** (CPU 2.7 GHz, akuisisi 60 FPS,
3 *thread* inferensi, pemilihan mode akuisisi otomatis):

Seluruh model diukur pada kedua frekuensi CPU dengan kode dan parameter identik
(pemilihan mode akuisisi dan jumlah *thread* otomatis), dengan *reboot* di antara
kedua rangkaian pengukuran:

| Konfigurasi | Ukuran | Mode | *Thr* | FPS @2.4 GHz | FPS @2.7 GHz | F1 |
|---|---|---|---|---|---|---|
| PyTorch `.pt` (tidak dijalankan di Pi) | 6.2 MB | — | — | — | — | — |
| ONNX FP32, 640 px | 12.3 MB | serial | 4 | 6.7 | 7.1 | 0.999 |
| ONNX INT8, 640 px | 3.5 MB | serial | 4 | 16.3 | 17.2 | 0.999 |
| ONNX INT8, 416 px | 3.4 MB | threaded | 3 | 37.0 | 40.3 | 0.993 |
| ONNX INT8, 320 px | 3.4 MB | threaded | 3 | 58.2 | **60.2** | 0.990 |

Peningkatan total: **6.7 → 60.2 FPS (9.0×)** dengan penurunan F1 dari 0.999 ke 0.990
(kehilangan 4 deteksi dari 417 pada dataset validasi).

Rincian kontribusi tiap tahap optimasi terhadap konfigurasi produksi (320 px):

| Tahap optimasi | FPS | Kenaikan | Jenis |
|---|---|---|---|
| Baseline FP32 640 px | 6.7 | — | — |
| Kuantisasi INT8 (Bagian 4) | 16.3 | 2.4× | perangkat lunak |
| Reduksi resolusi 640 → 320 px (Bagian 7) | ~30 | 1.9× | perangkat lunak |
| Perbaikan akuisisi kamera (Bagian 5) | ~52 | 1.7× | konfigurasi |
| Optimasi jumlah *thread* (Bagian 6.2) | 58.2 | 1.12× | konfigurasi |
| *Overclock* CPU 2.4 → 2.7 GHz (Bagian 6.3) | **60.2** | 1.034× | perangkat keras |

Perlu dicatat bahwa *overclock* — satu-satunya optimasi yang mengubah pengaturan
perangkat keras — memberi kontribusi **terkecil (+3.4%)** meskipun menaikkan clock
sebesar 12.5%, karena pada tahap tersebut penghambat telah berpindah ke kamera.

> **Catatan pengukuran:** baseline FP32 640 px pada konfigurasi akhir menghasilkan
> 6.7 FPS, praktis sama dengan 6.6 FPS yang terukur pada konfigurasi awal. Kebetulan
> ini bersifat semu: pada konfigurasi awal model tersebut dibatasi kamera (68 ms),
> sedangkan pada konfigurasi akhir dibatasi komputasinya sendiri (136 ms). Kesamaan
> angka **tidak** berarti perbaikan kamera tidak berpengaruh — pengaruhnya baru
> terlihat pada model yang cukup cepat untuk memanfaatkannya.

Tiga hal yang perlu ditegaskan untuk penulisan ilmiah:

1. **Percepatan tidak seluruhnya berasal dari model.** Kuantisasi INT8 memberi 2.7×;
   sisanya berasal dari perbaikan konfigurasi kamera (4.2×, Bagian 5) dan paralelisasi
   pipeline (Bagian 6). Tanpa kedua perbaikan tersebut, model 320 px maupun 416 px
   sama-sama mentok di 15 FPS meskipun waktu inferensinya berbeda hampir 4×.
2. **Penghambat berpindah dua kali selama optimasi.** Awalnya model (140 ms), lalu
   kamera (68 ms, kemudian 32 ms), lalu kembali ke model setelah akuisisi mencapai
   62 FPS. Setiap perpindahan mengubah optimasi mana yang bermanfaat.
3. **Angka FPS pada Bagian 5–7 mencerminkan tahapan optimasi yang berbeda.** Nilai
   30.2 FPS untuk 416 px diukur ketika akuisisi masih terbatas 31 FPS; setelah akuisisi
   mencapai 62 FPS, model yang sama menghasilkan 33.6 FPS.

---

## 2. Metodologi Pengukuran

Agar angka dapat direproduksi, dua besaran diukur terpisah:

**Kecepatan.** Median latensi `session.run()` atas 171 citra validasi. Median dipilih
alih-alih rata-rata karena kekebalannya terhadap *outlier* pada iterasi awal (cache dingin).

**Akurasi.** Deteksi dicocokkan dengan anotasi ground-truth menggunakan kriteria
IoU ≥ 0.5 **dan** kelas harus sama. Dilaporkan sebagai precision, recall, F1, beserta
jumlah TP/FP/FN absolut. Ambang deteksi conf = 0.35, NMS IoU = 0.45.

Skrip: `bench.py` (pengukuran), `test_bench.py` + `test_detect.py` (verifikasi
kebenaran kode pengukuran itu sendiri — lihat Bagian 8).

> **Catatan keterbatasan:** metrik yang dilaporkan adalah F1 pada IoU tunggal (0.5),
> bukan mAP@[.5:.95] seperti pada laporan pelatihan Ultralytics. Angka F1 di sini
> **tidak sebanding langsung** dengan mAP hasil training. Metrik ini dipilih karena
> tujuannya membandingkan model terkuantisasi terhadap model FP32 pada pipeline
> yang sama, bukan mengklaim performa absolut.

---

## 3. Konversi PyTorch → ONNX

Ekspor dilakukan **di Raspberry Pi**, bukan di laptop, karena ruang disk laptop tidak
mencukupi untuk dependensi PyTorch (~2.5 GB, tersisa 1.4 GB).

```bash
yolo export model=best.pt format=onnx imgsz=640 opset=12 simplify=True dynamic=False
```

Hasil: 12.3 MB, input `images` (1,3,640,640), output `output0` (1,6,8400)
= 4 koordinat kotak + 2 skor kelas × 8400 kandidat.

Metadata `names` tersimpan di dalam berkas ONNX, sehingga nama kelas dibaca otomatis
saat runtime dan tidak perlu ditulis ulang di kode.

**Catatan penting:** ekspor menggunakan `nms=False`, sehingga *Non-Maximum Suppression*
diimplementasikan manual di sisi Python (`detect.py`). Ini disengaja: NMS bawaan ONNX
menambah node yang tidak semua *runtime* dukung, dan implementasi manual memungkinkan
*class-aware NMS* (Bagian 8).

---

## 4. Kuantisasi INT8 — Empat Kegagalan dan Penyebab Sebenarnya

Bagian ini didokumentasikan lengkap karena merupakan temuan teknis utama, dan
kegagalannya tidak terdokumentasi baik di literatur populer.

### 4.1 Prosedur

Kuantisasi statis (*post-training static quantization*) dengan
`onnxruntime.quantization.quantize_static`, format QDQ, kalibrasi menggunakan
**171 citra validasi asli** — bukan data sintetis — dengan praproses identik dengan
kondisi inferensi (letterbox, RGB, normalisasi /255).

### 4.2 Kronologi kegagalan

| # | Konfigurasi | Hasil |
|---|---|---|
| 1 | per-channel, opset 12 | **Gagal dimuat**: `INVALID_GRAPH` |
| 2 | per-tensor, opset 12 | Termuat, **semua skor kelas = 0.000000** |
| 3 | per-channel, opset 13 | Termuat, skor kelas = 0.000000 |
| 4 | per-channel + head FP32 | Termuat, skor kelas = 0.000000 |
| 5 | signed INT8 activations | Termuat, skor kelas = 0.000000 |
| 6 | **ekor graf dipertahankan FP32** | **Berhasil, F1 identik** |

**Kegagalan #1** disebabkan `per_channel=True` menuliskan atribut `axis` pada node
`DequantizeLinear`, yang baru sah pada opset ≥ 13. Diatasi dengan konversi opset
menggunakan `onnx.version_converter` — diverifikasi *bit-exact* terhadap model asal
(selisih maksimum 0.0), sehingga tidak memerlukan ekspor ulang dari PyTorch.

**Kegagalan #2–#5** memiliki gejala identik: model termuat dan berjalan, tetapi
seluruh 8400 skor kelas bernilai **nol eksak**. Nol eksak merupakan petunjuk penting:
fungsi sigmoid tidak pernah menghasilkan nol dari masukan berhingga, sehingga ini
kerusakan numerik, bukan degradasi akurasi.

### 4.3 Dua hipotesis yang terbukti salah

Didokumentasikan karena keduanya adalah dugaan yang wajar dan sering disarankan:

**Hipotesis A — granularitas skala kuantisasi.** Dugaan bahwa per-tensor terlalu kasar
untuk cabang klasifikasi. *Terbantah:* per-channel (percobaan #3) menghasilkan kegagalan
yang sama persis.

**Hipotesis B — rentang logit negatif.** Pengukuran menunjukkan logit pra-sigmoid
berkisar **−109 hingga +3.4 dengan 99.8% bernilai negatif**. Diduga kuantisasi *unsigned*
(UInt8) menghancurkan pita sempit di sekitar nol yang menentukan deteksi. *Terbantah:*
mengganti ke *signed* INT8 (percobaan #5) tidak mengubah hasil sama sekali.

Kesalahan metodologis pada kedua hipotesis: keduanya menduga penyebab dari pengamatan
tidak langsung, tanpa menelusuri di titik mana sinyal hilang.

### 4.4 Penyebab sebenarnya

Penelusuran aktivasi lapis-demi-lapis pada model FP32 dan INT8 menunjukkan keduanya
**sehat di seluruh kedalaman jaringan**, termasuk keluaran kepala klasifikasi
(`cv3.0.2`: −31.5 pada INT8 vs −32.2 pada FP32). Bahkan keluaran sigmoid akhir
bernilai **0.9703 (INT8) vs 0.9685 (FP32)** — praktis identik.

Artinya deteksi tetap dihasilkan dengan benar di dalam graf, namun rusak sebelum
mencapai keluaran akhir. Penelusuran mundur dari node keluaran menemukan penyebabnya:

> Kuantizer menyisipkan pasangan `QuantizeLinear`/`DequantizeLinear` pada **tensor
> keluaran akhir**. Tensor tersebut merupakan hasil `Concat` antara koordinat kotak
> (rentang 0–640) dan skor kelas (rentang 0–1). Karena satu skala kuantisasi berlaku
> untuk seluruh tensor, skala ditentukan oleh rentang terlebar (640), menghasilkan
> langkah kuantisasi ≈2.5. Seluruh skor kelas yang bernilai < 1 dibulatkan menjadi
> **nol**.

Ini adalah kasus khas *dynamic range mismatch* pada tensor gabungan — kegagalan yang
tidak terdeteksi oleh pemeriksaan "model berhasil dimuat" maupun oleh inspeksi bobot.

### 4.5 Solusi

Mempertahankan lima node ekor graf dalam presisi FP32:
`Concat_3`, `Sigmoid`, `Mul_2`, `Concat_2`, `Div_1`.

Kelimanya merupakan operasi *elementwise* murni, sehingga **biaya komputasinya dapat
diabaikan** — terbukti dari waktu inferensi 51 ms yang tetap 2.7× lebih cepat dari FP32.

Pemilihan node dilakukan secara **struktural** (penelusuran mundur dari node keluaran),
bukan berdasarkan nama node yang di-*hardcode*. Ini penting karena nama node merupakan
detail implementasi eksportir dan berubah antar `imgsz`; daftar nama yang usang akan
mengembalikan kegagalan secara diam-diam. Pemilihan struktural terbukti menghasilkan
himpunan node yang benar pada ketiga ukuran (640, 416, 320).

### 4.6 Hasil kuantisasi

| Model | Ukuran | Inferensi | F1 | TP | FP | FN |
|---|---|---|---|---|---|---|
| FP32 640 | 12.3 MB | 139.8 ms | 0.999 | 417 | 1 | 0 |
| INT8 640 | 3.5 MB | 51.2 ms | 0.999 | 417 | 1 | 0 |

**Percepatan 2.73× tanpa kehilangan satu deteksi pun** (TP/FP/FN identik) pada seluruh
171 citra validasi.

---

## 5. Temuan: Konfigurasi Kamera sebagai Penghambat Tersembunyi

Setelah model 320 px dan 416 px selesai, pengujian webcam menunjukkan **ketiga ukuran
model menghasilkan 15.0 FPS yang sama persis**, meskipun waktu inferensinya berbeda
jauh (51 / 21 / 14 ms). Keseragaman yang mencurigakan ini menandakan penghambat di
luar model.

Pengukuran `cap.read()` terisolasi memberi **67.9 ms** (≈14.7 FPS), dan nilainya
**identik pada seluruh kombinasi** resolusi (1280×720, 800×600, 640×480), format
(MJPG, YUYV), dan permintaan FPS (30, 60). Konstansi sempurna semacam ini menunjukkan
pembatas tetap, bukan keterbatasan bandwidth.

Penyebabnya ditemukan pada kontrol V4L2:

```
exposure_dynamic_framerate  default=0  value=1
```

Kontrol ini mengizinkan kamera **menurunkan frame rate** demi waktu pencahayaan lebih
panjang pada kondisi cahaya rendah. Menonaktifkannya:

| Kondisi | Latensi akuisisi | FPS |
|---|---|---|
| `exposure_dynamic_framerate=1` (bawaan) | 67.9 ms | 14.7 |
| `exposure_dynamic_framerate=0` | 32.0 ms | 31.2 |
| ditambah urutan `CAP_PROP_FPS` yang benar (Bagian 7) | **16.1 ms** | **62.1** |

**Peningkatan 2.1× pada tahap ini, dan 4.2× setelah digabung dengan perbaikan urutan
pemanggilan API — tanpa mengubah model sama sekali.** Pengujian lanjutan menunjukkan
pencahayaan manual (exposure 157 dan 78) tidak menambah kecepatan lebih jauh, sehingga
auto-exposure dipertahankan aktif demi adaptasi terhadap perubahan cahaya arena.

Perbaikan ini diterapkan otomatis di `detect.py` pada setiap inisialisasi kamera,
sehingga tidak hilang saat kamera dicabut-pasang.

---

## 6. Temuan: Paralelisasi Akuisisi dan Inferensi

Setelah kamera diperbaiki, akuisisi (32 ms) dan inferensi (14–21 ms) berjalan
**berurutan**, sehingga total ≈46–53 ms per frame. Karena `cap.read()` bersifat
*blocking*, CPU menganggur selama menunggu frame.

Akuisisi dipindahkan ke *thread* terpisah dengan kebijakan **selalu mengambil frame
terbaru dan membuang frame usang** — untuk deteksi real-time, latensi lebih penting
daripada kelengkapan frame.

Biaya per frame menjadi `max(32, 21)` = 32 ms alih-alih `32 + 21` = 53 ms.

| Konfigurasi | FPS (416 px) |
|---|---|
| Akuisisi serial | ~19 |
| Akuisisi ter-*thread* | **30.2** (mode GUI: 36.0) |

### 6.0 Catatan: antarmuka preview pada sesi Wayland

Sistem menggunakan **wayvnc** dengan kompositor **labwc** (Wayland), bukan X11 murni.
VNC menampilkan sesi desktop yang sama dengan keluaran HDMI, sehingga program preview
harus dijalankan dalam konteks sesi tersebut.

Bila dijalankan langsung melalui SSH tanpa variabel lingkungan yang sesuai, OpenCV
gagal membuka window. `run.sh` karena itu menyetel `XDG_RUNTIME_DIR`,
`WAYLAND_DISPLAY`, dan `DISPLAY` secara otomatis bila belum tersedia, sehingga program
dapat dijalankan baik dari terminal desktop maupun dari SSH.

Verifikasi dilakukan dengan menangkap layar desktop (`grim`) selama program berjalan,
mengonfirmasi window preview benar-benar ter-*render* pada kompositor — bukan sekadar
program berjalan tanpa galat.

Laju frame pada mode preview terukur **48–57 FPS**, lebih rendah daripada mode
*headless* (60.2 FPS) karena penggambaran window dan enkode VNC turut menggunakan CPU.
Untuk pengukuran, mode `--headless` digunakan agar angka tidak tercemar biaya
penampilan.

### 6.1 Paralelisasi tidak selalu menguntungkan

Setelah akuisisi mencapai 62 FPS, pengukuran ulang menunjukkan bahwa *threading*
justru **merugikan** model yang lambat:

| Model | Inferensi | Serial | Ter-*thread* | Selisih |
|---|---|---|---|---|
| FP32 640 px | 136 ms | **6.7** | 5.2 | −22% |
| INT8 640 px | 50 ms | **15.9** | 14.1 | −11% |
| INT8 320 px | 13 ms | 47.2 | **53.4** | +13% |

Penyebabnya: bila waktu inferensi melebihi interval frame (16.7 ms pada 60 FPS),
*thread* akuisisi terus-menerus mengambil frame yang langsung dibuang tanpa pernah
diproses. Pekerjaan tersebut sia-sia, dan CPU yang digunakannya diambil langsung dari
proses inferensi — pada Pi 5 yang hanya memiliki 4 inti, dampaknya terukur.

Ini juga menjelaskan mengapa waktu inferensi FP32 640 px terukur 202 ms dalam mode
ter-*thread* namun 136 ms dalam mode serial: selisihnya adalah CPU yang direbut oleh
*thread* akuisisi.

**Solusi:** pemilihan mode dilakukan otomatis saat inisialisasi. Program mengukur
waktu inferensi nyata (3 iterasi setelah 2 iterasi pemanasan) dan memilih mode
ter-*thread* hanya bila inferensi < 1.5 × interval frame. Ambang 1.5× diverifikasi
terhadap keempat model dan menghasilkan pilihan yang benar pada seluruhnya:

```
FP32 640 : inferensi 136 ms vs interval 17 ms -> serial
INT8 640 : inferensi  50 ms vs interval 17 ms -> serial
INT8 416 : inferensi  21 ms vs interval 17 ms -> threaded
INT8 320 : inferensi  13 ms vs interval 17 ms -> threaded
```

Mode dapat dipaksa melalui `--capture threaded|serial` untuk keperluan pengukuran.

### 6.2 Jumlah *thread* inferensi bergantung pada mode akuisisi

Pengukuran terisolasi menunjukkan anomali: inferensi 320 px sanggup **75.5 FPS** dan
akuisisi sanggup **62.4 FPS**, namun pipeline gabungan hanya menghasilkan 49 FPS.
Selisih ~13 FPS tersebut merupakan biaya kompetisi CPU, bukan keterbatasan salah satu
komponen.

Menyisakan satu inti untuk *thread* akuisisi dan dekode MJPG:

| `intra_op_num_threads` | FPS (2.4 GHz) | FPS (2.7 GHz) |
|---|---|---|
| 2 | 50.0 | — |
| **3** | **59.5** | **60.4** |
| 4 | 53.8 | 52.0 |

**Peningkatan 11% hanya dengan mengurangi satu *thread*.** Pemberian seluruh 4 inti
kepada onnxruntime justru merugikan karena *thread* akuisisi dan dekode MJPG terpaksa
berebut inti dengan inferensi. Efek ini tetap konsisten setelah *overclock*, yang
mengonfirmasi bahwa penyebabnya struktural (jumlah inti), bukan kekurangan kapasitas
komputasi.

**Namun keunggulan 3 *thread* tidak berlaku universal.** Pengukuran seluruh model
menunjukkan pembalikan:

| Model | Mode akuisisi | 3 *thread* | 4 *thread* | Optimal |
|---|---|---|---|---|
| FP32 640 px | serial | 6.7 | **7.0** | 4 |
| INT8 640 px | serial | 16.4 | **17.3** | 4 |
| INT8 416 px | threaded | **40.0** | 35.5 | 3 |
| INT8 320 px | threaded | **60.4** | 52.0 | 3 |

Polanya konsisten dengan penjelasan di atas: model lambat berjalan pada mode akuisisi
**serial**, sehingga tidak ada *thread* latar yang perlu diberi inti — memberikan
seluruh 4 inti kepada inferensi adalah pilihan terbaik. Model cepat berjalan
*threaded* dan memerlukan satu inti tersisa.

**Solusi:** jumlah *thread* ditentukan otomatis mengikuti mode akuisisi yang terpilih
(`ncores - 1` bila *threaded*, `ncores` bila serial). Verifikasi menunjukkan pemilihan
otomatis menghasilkan konfigurasi optimal pada keempat model. Dapat ditimpa melalui
`--threads N`.

### 6.3 *Overclock* CPU

Konfigurasi: `arm_freq=2700` pada `/boot/firmware/config.txt` (2.4 → 2.7 GHz, +12.5%),
tanpa `over_voltage` manual — firmware menaikkan tegangan inti secara otomatis dari
0.76 V ke 0.91 V.

Untuk memisahkan efek *overclock* dari efek optimasi lain, seluruh konfigurasi diukur
ulang pada kedua frekuensi menggunakan **kode dan parameter yang identik** (pemilihan
mode akuisisi dan jumlah *thread* otomatis). Sistem di-*reboot* di antara kedua
pengukuran.

**Pipeline lengkap (webcam → deteksi):**

| Model | 2.4 GHz | 2.7 GHz | Kenaikan |
|---|---|---|---|
| FP32 640 px | 6.7 | 7.1 | +6.0% |
| INT8 640 px | 16.3 | 17.2 | +5.5% |
| INT8 416 px | 37.0 | 40.3 | +8.9% |
| INT8 320 px | 58.2 | **60.2** | +3.4% |

**Inferensi terisolasi (tanpa kamera), median 40 iterasi:**

| Model | *Threads* | 2.4 GHz | 2.7 GHz | Kenaikan |
|---|---|---|---|---|
| FP32 640 px | 4 | 136.84 ms | 127.22 ms | +7.6% |
| INT8 640 px | 4 | 50.86 ms | 47.54 ms | +7.0% |
| INT8 416 px | 4 | 21.32 ms | 19.59 ms | +8.8% |
| INT8 320 px | 4 | 13.47 ms | 12.17 ms | +10.7% |
| INT8 320 px | 3 | 14.85 ms | 13.27 ms | +11.9% |

**Interpretasi.** Pada inferensi terisolasi, peningkatan mendekati kenaikan clock
teoretis (+12.5%) — terbesar pada model kecil (+10.7%) yang seluruhnya terbatas
komputasi, dan mengecil pada model besar (+7.6%) yang mulai terbatas *memory
bandwidth*. Bandwidth memori tidak ikut naik saat CPU di-*overclock*.

Pada pipeline lengkap, peningkatan lebih kecil lagi. Model produksi (320 px) hanya
memperoleh **+3.4%** meskipun inferensinya sendiri membaik +10.7%, karena akuisisi
kamera (62.4 FPS) menjadi penghambat: pipeline sudah beroperasi pada 93% kapasitas
akuisisi pada 2.4 GHz, sehingga hanya tersisa sedikit ruang untuk dimanfaatkan.

Uji ketahanan 3000 frame berturut-turut (~50 detik beban penuh) mempertahankan
60.1 FPS dengan `throttled=0x0` dan clock tetap 2.7 GHz, sehingga konfigurasi ini
aman untuk durasi pertandingan. Suhu 67.0 °C dengan pendingin aktif (`pwmfan`).

Variasi antar-*run* pada konfigurasi produksi terukur 58–60 FPS, dipengaruhi beban
latar sistem. Angka 60.2 FPS merupakan median dari rangkaian pengukuran 250 frame.

**Kesimpulan:** *overclock* +12.5% menghasilkan +3.4% pada konfigurasi produksi.
Manfaatnya nyata namun kecil, dan diperoleh dengan biaya kenaikan suhu ~5 °C serta
risiko kegagalan *boot*. Sebagai pembanding, optimasi jumlah *thread* (Bagian 6.2)
memberi +11% tanpa risiko dan tanpa biaya termal.

Konfigurasi dapat dikembalikan dengan:
```bash
sudo cp /boot/firmware/config.txt.bak-preoc /boot/firmware/config.txt && sudo reboot
```

---

## 7. Analisis Penghambat Akhir

Analisis ini berubah dua kali selama eksperimen, sehingga didokumentasikan bertahap.

**Tahap A — setelah kuantisasi INT8, sebelum perbaikan kamera.** Ketiga ukuran model
menghasilkan 15.0 FPS identik meskipun inferensi berbeda 51/21/14 ms. Penghambat:
akuisisi 68 ms.

**Tahap B — setelah `exposure_dynamic_framerate=0`, akuisisi 32 ms.** Model 416 px dan
320 px sama-sama mencapai ~30 FPS. Pada tahap ini disimpulkan bahwa 416 px lebih baik
karena akurasinya lebih tinggi tanpa biaya kecepatan. **Kesimpulan ini menjadi tidak
berlaku pada Tahap C.**

**Tahap C — setelah akuisisi mencapai 62 FPS, sebelum optimasi *thread*.** Penghambat
kembali ke model, dan seluruh 4 inti masih dialokasikan ke inferensi:

| Model | Mode | FPS |
|---|---|---|
| FP32 640 px | serial | 6.7 |
| INT8 640 px | serial | 15.9 |
| INT8 416 px | threaded | 35.6 |
| INT8 320 px | threaded | **52.8** |

**Tahap D — setelah optimasi jumlah *thread* dan *overclock* 2.7 GHz.** Konfigurasi
final, dengan pemilihan mode akuisisi dan jumlah *thread* otomatis:

| Model | Mode | *Thr* | FPS |
|---|---|---|---|
| FP32 640 px | serial | 4 | 7.1 |
| INT8 640 px | serial | 4 | 17.2 |
| INT8 416 px | threaded | 3 | 40.3 |
| INT8 320 px | threaded | 3 | **60.2** |

Penghambat pada Tahap D kembali berpindah ke kamera:

| Komponen | Kapasitas terisolasi (2.7 GHz) |
|---|---|
| Inferensi 320 px, 3 *thread* | 75.4 FPS |
| Inferensi 320 px, 4 *thread* | 82.1 FPS |
| Akuisisi kamera | **62.4 FPS** ← penentu |
| Pipeline gabungan | 60.2 FPS |

Konsekuensi:

- Pemilihan ukuran model kembali menjadi trade-off nyata. Pada Tahap B, 320 px tidak
  memberi keuntungan; pada Tahap D, 320 px memberi **+49% FPS** dibanding 416 px.
- Peningkatan lebih lanjut **tidak dapat** dicapai melalui optimasi komputasi:
  pipeline telah beroperasi pada 96% kapasitas akuisisi (60.2 dari 62.4 FPS).
  Diperlukan kamera dengan laju lebih tinggi.
- *Overclock* memberi kontribusi kecil (+3.4%) justru karena penghambat telah
  berpindah — lihat Bagian 6.3.

### Mencapai 60 FPS: urutan pemanggilan API, bukan keterbatasan bandwidth

Logitech C922 mendukung mode **MJPG 1280×720 @ 60 FPS**, terverifikasi pada
`v4l2-ctl --list-formats-ext`. Upaya awal mencapainya selalu berhenti di ~30 FPS:

| Metode | Hasil |
|---|---|
| `cv2.CAP_PROP_FPS = 60` | 31.2 FPS (driver melaporkan kembali ke 30) |
| `v4l2-ctl --set-parm=60` sebelum membuka | 31.2 FPS |
| `v4l2-ctl --set-parm=60` setelah membuka | 31.2 FPS |
| Streaming v4l2 murni tanpa OpenCV, perintah terpisah | 29.8 FPS |
| Manual exposure 10 ms / 5 ms | 29.8 FPS |

#### Hipotesis yang terbukti salah: keterbatasan USB 2.0

Pemeriksaan topologi USB menunjukkan kamera tersambung pada bus 480 Mbps (USB 2.0),
sehingga sempat disimpulkan bahwa 720p60 memerlukan USB 3.0.

**Kesimpulan tersebut keliru.** MJPG merupakan format terkompresi: 720p60 MJPG
membutuhkan sekitar 20–30 Mbps, jauh di bawah kapasitas praktis USB 2.0 (~480 Mbps).
Bandwidth tidak pernah menjadi kendala. Kesalahan ini terjadi karena kesimpulan ditarik
dari korelasi (kamera memang di USB 2.0) tanpa menghitung kebutuhan bandwidth
sebenarnya — kekeliruan metodologis yang sama dengan dua hipotesis pada Bagian 4.3.

#### Penyebab sebenarnya

Menjalankan pengaturan format dan frame rate dalam **satu perintah** menghasilkan:

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1280,height=720,pixelformat=MJPG \
  --set-parm=60 --stream-mmap --stream-count=250
# -> 59.43 fps
```

**59.4 FPS pada USB 2.0.** Penyebab kegagalan sebelumnya: `--set-fmt-video`
menegosiasi ulang stream dan **mereset frame interval kembali ke 30**. Bila format dan
frame rate diatur dalam perintah terpisah, pengaturan 60 FPS selalu terhapus oleh
pengaturan format yang menyusul.

Perilaku yang sama berlaku pada OpenCV: `cap.set(CAP_PROP_FOURCC/WIDTH/HEIGHT)`
memicu negosiasi ulang. Sehingga terdapat **dua batasan urutan pemanggilan**:

1. `CAP_PROP_FPS` harus diset **setelah** fourcc dan resolusi.
2. `exposure_dynamic_framerate=0` harus diterapkan **setelah** `VideoCapture` terbuka —
   OpenCV mereset kontrol tersebut selama negosiasi.

Dengan keduanya diterapkan:

| Konfigurasi | Latensi | FPS |
|---|---|---|
| Bawaan | 67.9 ms | 14.7 |
| `dynamic_framerate=0` saja | 32.0 ms | 31.2 |
| `dynamic_framerate=0` + urutan `CAP_PROP_FPS` benar | **16.1 ms** | **62.1** |

Peningkatan total pada tahap akuisisi: **4.2× tanpa perubahan perangkat keras apa pun.**

---

## 8. Verifikasi Kebenaran Kode Pengukuran

Dua kesalahan pada kode pengukuran ditemukan dan diperbaiki. Keduanya berpotensi
menghasilkan kesimpulan yang salah total, sehingga didokumentasikan:

**Kesalahan 1 — kesalahan argumen.** Path model diteruskan ke parameter direktori
kalibrasi, menyebabkan kalibrasi gagal.

**Kesalahan 2 — format anotasi.** Dataset menggunakan format **poligon segmentasi**
(puluhan koordinat per baris), bukan *bounding box* 4-nilai. Parser awal membaca empat
angka pertama sebagai `cx cy bw bh`, menghasilkan kotak ground-truth yang keliru.
Akibatnya model FP32 — yang terbukti berfungsi baik — terukur dengan
**precision 0.000**. Tanpa koreksi ini, seluruh perbandingan INT8 tidak bermakna.

Perbaikan: poligon dikonversi ke *bounding box* melalui min/max koordinat, dengan
dukungan kedua format sekaligus.

Kode pengukuran kini disertai **13 uji otomatis** yang dijalankan di perangkat
(7 pada `test_detect.py`, 6 pada `test_bench.py`):

- `test_bench.py` — parsing anotasi (format deteksi, poligon, baris nyata dari dataset,
  baris rusak, berkas hilang) dan perhitungan IoU
- `test_detect.py` — konsistensi letterbox↔postprocess (*roundtrip*), *class-aware* NMS,
  supresi kelas sama, *clipping* batas frame, penyaringan confidence, dan perilaku
  *thread* kamera (frame selalu baru, tidak menggantung saat kamera mati)

Selain itu `quantize.py` kini **menolak menyimpan model yang skor kelasnya kolaps**
(< 50% dari referensi FP32), sehingga kegagalan Bagian 4 tidak dapat lolos ke tahap
deployment secara diam-diam.

### 8.1 Catatan integritas berkas selama eksperimen

Dua insiden operasional tercatat saat rangkaian pengujian *overclock*, keduanya
relevan sebagai catatan reproduksibilitas:

**Insiden 1 — perintah gabungan gagal sebagian.** Perintah pengembalian konfigurasi
dan *reboot* dijalankan dalam satu baris; koneksi terputus sebelum penyalinan berkas
tersimpan ke disk, sehingga sistem melakukan *reboot* tanpa perubahan konfigurasi.
Terdeteksi karena verifikasi pasca-*reboot* menunjukkan clock masih 2.7 GHz.
Perbaikan: penyalinan, `sync`, dan verifikasi dilakukan terpisah sebelum *reboot*.

**Insiden 2 — berkas rusak akibat transfer terpotong.** `detect.py` di perangkat
mengandung *null byte* akibat `scp` yang terpotong saat sistem *reboot*, menyebabkan
`SyntaxError: source code cannot contain null bytes`. Terdeteksi karena seluruh
pengukuran mendadak tidak menghasilkan keluaran apa pun. Perbaikan: berkas dikirim
ulang dan integritasnya diverifikasi dengan perbandingan MD5 terhadap sumber.

Kedua insiden menegaskan perlunya verifikasi eksplisit setelah operasi yang melibatkan
*reboot* atau transfer berkas, karena keduanya gagal secara senyap — sistem tetap
berjalan normal namun tidak pada kondisi yang diasumsikan.

---

## 9. Konfigurasi Akhir

**Model produksi:** `best_320_int8.onnx` (3.4 MB) — **60.2 FPS**, F1 0.990.

Alasan pemilihan 320 px: setelah akuisisi mencapai 62 FPS, ukuran model kembali
menentukan kecepatan. 320 px memberi **+49% FPS** dibanding 416 px (60.2 vs 40.3)
dengan biaya 2 deteksi tambahan yang terlewat dari 417 (F1 0.990 vs 0.993).

**Parameter runtime produksi:**

| Parameter | Nilai | Alasan |
|---|---|---|
| `arm_freq` | 2700 | +12.5% clock; kontribusi +3.4% (Bagian 6.3) |
| `--threads` | auto → 3 | menyisakan satu inti untuk akuisisi + dekode MJPG |
| `--capture` | auto → threaded | inferensi 12 ms < interval frame 17 ms |
| Resolusi kamera | 1280×720 MJPG @ 60 | satu-satunya mode 60 FPS pada C922 |

Kedua parameter `auto` menyesuaikan diri terhadap model yang dimuat; pada model 640 px
keduanya otomatis beralih ke `serial` dan 4 *thread*.

**Rekomendasi alternatif:**
- `best_416_int8.onnx` — 40.3 FPS, F1 0.993. Dipilih bila ketelitian deteksi korban
  lebih diutamakan daripada laju frame.
- `best_int8.onnx` (640 px) — 17.2 FPS, F1 0.999. Untuk evaluasi akurasi maksimum atau
  pemrosesan luring (*offline*).

**Struktur berkas** (`~/M`, 402 MB termasuk venv):

```
run.sh                launcher (menyiapkan environment Wayland/X untuk preview)
detect.py             deteksi real-time + antarmuka preview
quantize.py           kuantisasi INT8 + verifikasi anti-kolaps
bench.py              pengukuran kecepatan dan akurasi
build_sizes.sh        otomasi ekspor + kuantisasi multi-resolusi
test_detect.py        7 uji: geometri, NMS, thread kamera
test_bench.py         6 uji: parsing anotasi, IoU
best_320_int8.onnx    model produksi (320 px)
best_416_int8.onnx    varian akurasi menengah
best_int8.onnx        varian akurasi tinggi (640 px)
best_op13.onnx        sumber FP32 opset 13 untuk kuantisasi ulang
best.pt               bobot PyTorch asli
calib/                171 citra + anotasi untuk kalibrasi
EKSPERIMEN.md         dokumen ini
.venv/                onnxruntime, opencv, numpy, onnx (368 MB, tanpa PyTorch)
```

Lingkungan runtime **tidak memuat PyTorch** (368 MB dibanding ~3 GB), karena inferensi
sepenuhnya menggunakan onnxruntime. PyTorch hanya diperlukan saat ekspor dan dihapus
setelahnya.

---

## 10. Panduan Operasional

### 10.1 Menjalankan

Dari terminal pada sesi VNC atau desktop:

```bash
cd ~/M && ./run.sh
```

Keluar dari program: tekan **`q`** atau **`Esc`** pada window preview.

`run.sh` menyiapkan variabel lingkungan Wayland/X secara otomatis, sehingga dapat
dijalankan baik dari terminal desktop maupun dari SSH (window tetap muncul pada
sesi desktop yang sedang berjalan).

### 10.2 Mengganti model

```bash
./run.sh --model best_416_int8.onnx     # ~40 FPS, F1 0.993
./run.sh --model best_int8.onnx         # ~17 FPS, F1 0.999
./run.sh --model best_320_int8.onnx     # ~60 FPS, F1 0.990 (bawaan)
```

Tidak diperlukan penyesuaian parameter lain: resolusi masukan dibaca dari metadata
model, sedangkan mode akuisisi dan jumlah *thread* ditentukan otomatis (Bagian 6).

### 10.3 Opsi runtime

| Opsi | Fungsi |
|---|---|
| `--conf 0.5` | ambang deteksi lebih ketat (mengurangi *false positive*) |
| `--conf 0.25` | lebih sensitif (menangkap objek samar) |
| `--headless` | tanpa window; FPS penuh, untuk pengukuran |
| `--frames N` | berhenti setelah N frame (0 = tanpa batas) |
| `--source 1` | bila kamera berpindah ke `/dev/video1` |
| `--threads N` | menimpa pemilihan otomatis jumlah *thread* |
| `--capture threaded\|serial` | menimpa pemilihan otomatis mode akuisisi |
| `--help` | seluruh opsi |

Opsi dapat digabung, misal:
`./run.sh --model best_416_int8.onnx --conf 0.5`

Untuk mengubah model bawaan secara permanen, sunting nilai `default=` pada argumen
`--model` di `detect.py`.

### 10.4 Menyiapkan model baru dari hasil pelatihan berikutnya

```bash
# 1. kirim bobot baru dari komputer pelatihan
scp best.pt bima@<ip-raspi>:~/M/

# 2. pasang ultralytics (diperlukan hanya saat ekspor, ~2 GB)
cd ~/M
python3 -m venv .venv-export
.venv-export/bin/pip install ultralytics onnx onnxslim

# 3. ekspor + kuantisasi otomatis untuk kedua resolusi
./build_sizes.sh 320 416

# 4. verifikasi akurasi terhadap dataset validasi
.venv/bin/python bench.py best_320_int8.onnx best_416_int8.onnx --limit 171

# 5. hapus kembali untuk menghemat ruang
rm -rf .venv-export
```

`build_sizes.sh` menjalankan rangkaian ekspor → konversi opset 13 → kuantisasi INT8,
dan **menolak menyimpan model yang skor kelasnya kolaps** (Bagian 4.5), sehingga
kegagalan kuantisasi tidak dapat lolos ke tahap deployment.

### 10.5 Pemeriksaan bila terjadi masalah

```bash
.venv/bin/python test_detect.py     # 7 uji kebenaran kode deteksi
.venv/bin/python test_bench.py      # 6 uji kode pengukuran
v4l2-ctl --list-devices             # memastikan kamera terdeteksi
./run.sh --headless --frames 60     # menjalankan tanpa GUI
```

**Gejala umum:** bila FPS turun drastis ke ~15, penyebab paling mungkin adalah
kontrol `exposure_dynamic_framerate` yang kembali ke nilai bawaan setelah kamera
dicabut-pasang (Bagian 5). `detect.py` menyetel ulang kontrol tersebut pada setiap
inisialisasi, sehingga cukup menjalankan ulang program.

### 10.6 Mengembalikan konfigurasi *overclock*

```bash
sudo cp /boot/firmware/config.txt.bak-preoc /boot/firmware/config.txt
sudo reboot
```

Sistem akan kembali beroperasi pada 2.4 GHz. Berdasarkan Bagian 6.3, dampaknya
terhadap konfigurasi produksi adalah penurunan dari 60.2 ke 58.2 FPS.

---

## 11. Kontribusi yang Dapat Dilaporkan

Untuk keperluan proposal/skripsi, temuan yang memiliki nilai metodologis:

1. **Kegagalan kuantisasi INT8 pada tensor keluaran gabungan.** YOLOv8 menggabungkan
   koordinat kotak dan skor kelas dalam satu tensor keluaran dengan rentang dinamis
   yang berbeda ~640×. Kuantisasi tensor tersebut menghancurkan skor kelas secara total
   sementara model tetap "berjalan normal". Solusinya — mempertahankan ekor graf dalam
   FP32 — berbiaya komputasi dapat diabaikan.

2. **Verifikasi kuantisasi tidak cukup dengan uji pemuatan model.** Empat dari lima
   percobaan menghasilkan model yang lolos `InferenceSession()` namun tidak menghasilkan
   satu pun deteksi. Verifikasi wajib membandingkan keluaran numerik terhadap model
   referensi pada masukan nyata.

3. **Penghambat pipeline dapat berpindah, dan kesimpulan optimasi ikut berubah.**
   Optimasi model 3.7× (51→14 ms) awalnya menghasilkan **nol** peningkatan FPS karena
   penghambat berpindah ke akuisisi. Setelah akuisisi diperbaiki 4.2×, penghambat
   kembali ke model dan ukuran 320 px yang semula tidak berguna menjadi pilihan
   terbaik (+55% FPS). Kesimpulan optimasi hanya sahih untuk konfigurasi saat
   pengukuran dilakukan.

4. **Parameter dan urutan pemanggilan API kamera dapat lebih menentukan daripada
   optimasi model.** Dua perbaikan non-komputasional — satu kontrol V4L2
   (`exposure_dynamic_framerate`) dan urutan pemanggilan `CAP_PROP_FPS` — memberi
   peningkatan 4.2× pada tahap akuisisi, melampaui hasil kuantisasi INT8 (2.7×).
   Keduanya tidak akan ditemukan tanpa mengukur tahap akuisisi secara terisolasi.

5. **Kebenaran kode pengukuran harus diverifikasi sebelum kesimpulan ditarik.**
   Kesalahan parsing anotasi menghasilkan precision 0.000 pada model yang terbukti
   berfungsi. Kesimpulan apa pun yang ditarik sebelum koreksi tersebut akan salah.

6. **Kesimpulan berbasis korelasi tanpa verifikasi kuantitatif berisiko keliru.**
   Tiga hipotesis dalam eksperimen ini terbantah: granularitas kuantisasi (4.3),
   rentang logit negatif (4.3), dan keterbatasan bandwidth USB 2.0 (7). Ketiganya
   masuk akal secara kualitatif namun gugur ketika diuji langsung. Khusus hipotesis
   USB, perhitungan sederhana (MJPG 720p60 ≈ 20–30 Mbps versus kapasitas 480 Mbps)
   sudah cukup membantahnya tanpa perlu eksperimen tambahan.

7. **Paralelisasi memiliki titik balik yang bergantung pada beban.** Memindahkan
   akuisisi ke *thread* terpisah memberi +13% pada model cepat namun **−22% pada model
   lambat**, karena *thread* yang membuang sebagian besar frame tetap mengonsumsi CPU
   dari inferensi. Pada perangkat berinti sedikit, keputusan paralelisasi sebaiknya
   diambil secara adaptif berdasarkan pengukuran runtime, bukan ditetapkan statis.

8. **Perbandingan antar-konfigurasi harus diukur ulang pada kondisi yang sama.**
   Baseline FP32 640 px menghasilkan angka nyaris identik (6.6 vs 6.7 FPS) sebelum dan
   sesudah seluruh optimasi pipeline, meskipun faktor pembatasnya berbeda sama sekali.
   Membandingkan angka yang diukur pada tahap optimasi berbeda dapat menghasilkan
   kesimpulan yang keliru mengenai besar kontribusi tiap perbaikan.

9. **Optimasi perangkat lunak mengungguli peningkatan perangkat keras ketika
   penghambat bukan komputasi.** Pada sistem ini, mengurangi jumlah *thread* inferensi
   dari 4 ke 3 memberi **+11%**, sedangkan *overclock* CPU sebesar +12.5% hanya memberi
   **+3.4%** pada konfigurasi produksi. Keduanya diuji pada perangkat yang sama dengan
   *reboot* di antaranya. Temuan ini menegaskan pentingnya identifikasi penghambat
   sebelum mengalokasikan sumber daya pada peningkatan perangkat keras.

10. **Efektivitas *overclock* bergantung pada jenis keterbatasan beban.** Pada
    inferensi terisolasi, kenaikan clock 12.5% menghasilkan +10.7% pada model 320 px
    (terbatas komputasi) namun hanya +7.6% pada model 640 px — selisih yang
    mengindikasikan keterbatasan *memory bandwidth*, yang tidak ikut meningkat saat
    CPU di-*overclock*. Pada pipeline lengkap, angka tersebut menyusut lagi menjadi
    +3.4% akibat keterbatasan akuisisi. Manfaat *overclock* karena itu harus diukur
    pada beban sebenarnya, bukan diekstrapolasi dari kenaikan frekuensi.

11. **Konfigurasi optimal dapat berbalik antar-beban.** Jumlah *thread* inferensi
    terbaik adalah 4 untuk model lambat (mode akuisisi serial) namun 3 untuk model
    cepat (mode *threaded*) — pembalikan yang disebabkan kebutuhan inti bagi *thread*
    akuisisi. Parameter yang ditetapkan statis berdasarkan pengujian satu konfigurasi
    dapat merugikan konfigurasi lain hingga 14%.
