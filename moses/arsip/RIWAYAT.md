# Riwayat versi — perangkat lunak sisi Raspberry Pi (R2C)

Berkas ini mendokumentasikan **semua versi buatan kita sendiri**, dari yang
pertama sampai yang terbaru, beserta apa yang berubah di tiap rilis dan
**kenapa** berubah.

## Jujur dulu soal isi folder `arsip/`

Selama pengembangan, tiap versi baru **menimpa** versi lama — di PC maupun di
Pi (`scp` ke `~/M`), dan tidak ada git di folder ini. Jadi berkas mentah dari
r1–r8 **sudah tidak ada** dan tidak bisa dikarang ulang tanpa berbohong.

Yang ada di `arsip/`:

- `2026-09-06_r9/` — snapshot **lengkap dan asli** dari rilis terbaru.
- `RIWAYAT.md` — berkas ini: catatan lengkap semua rilis, r1 sampai r9.
- `../simpan_arsip.sh` — jalankan **sebelum** mengubah apa pun mulai sekarang,
  supaya mulai r10 tiap versi punya snapshot sungguhan.

Penomoran r1–r9 dibuat sekarang untuk keperluan catatan ini; waktu
pengembangannya tidak pernah ada label versi.

---

## r1 — HUD misi pertama

**Berkas baru:** `mission_hud.py`, `run_hud.sh`, `upload_to_pi.sh`

- Panel web di `:5000` memakai `http.server` pustaka standar — sengaja tanpa
  Flask, tanpa VNC, tanpa desktop, supaya tidak menambah beban Pi.
- Tabel `MISI`: 19 misi HOME→FINISH sebagai **satu-satunya sumber kebenaran**
  rute. Sudah membawa peringatan bahwa pasangan K-4/SZ-4/K-5/SZ-5 wajib
  dicocokkan ulang dengan guidebook halaman 21.
- Sub-FSM per ruang: `IDLE → TUNGGU → STANDOFF → CENTER → LIHAT → SLOT →
  PUTUS → DEKATI → … → BERES`, plus `GAGAL`.
- Vision: YOLOv8n ONNX INT8, gerbang geometris (ROI sudut + pita tinggi bbox),
  voting k-of-N, prior eliminasi "tepat satu korban asli per ruang".
- **Tanpa autoload dan tanpa autosave kalibrasi** — permintaan eksplisit,
  supaya nilai tidak berubah diam-diam antar percobaan.
- `mission_hud.py` **mengimpor** `load_session`, `letterbox`, `postprocess`,
  `open_camera`, `CameraThread` dari `detect.py` — tidak menyalinnya, supaya
  angka benchmark di `EKSPERIMEN.md` tetap sah.
- Stream MJPEG + `/state` JSON supaya halaman bisa dipoll tanpa reload.

## r2 — nyala sendiri saat boot

**Berkas baru:** `install_service.sh`

- Unit systemd `r2c-hud`, `Restart=always`, otomatis menambahkan user ke grup
  `dialout` dan `video`.
- Bisa dibuka di `http://terra-core:5000/` tanpa SSH sama sekali.
- **Perbaikan:** `StartLimitIntervalSec` dipindah dari `[Service]` ke `[Unit]`.
  Salah tempat hanya menghasilkan "Unknown key name … ignoring" di log, lalu
  systemd diam-diam memakai batas bawaan dan layanan berhenti sendiri sesudah
  beberapa kali restart cepat.

## r3 — bisa dipakai dari Windows, dan tahan banting

**Berkas baru:** `upload_to_pi.bat`, `upload_teensy.bat` (CRLF, untuk CMD)

- Semua skrip `.sh` tidak bisa dipakai dari CMD. Diganti `.bat` yang juga
  melakukan `chmod +x` lewat ssh — `scp` tidak membawa bit eksekusi, itu
  penyebab `Permission denied` pada `./run_hud.sh`.
- `upload_teensy.bat`: pasang index PJRC, core Teensy, pustaka VL53L1X,
  kompilasi ke `build_log.txt`, dan **mendeteksi port sendiri** dari
  `arduino-cli board list` (tanpa `-p`, upload gagal "no upload port provided").
- **Kamera tahan banting:** `/dev/video19..35` ternyata node ISP Pi, bukan
  kamera. `daftar_video()` mengutamakan 0–9, `buka_kamera()` menuntut satu
  frame sungguhan, dan HUD **tidak lagi mati** kalau kamera belum tercolok —
  halamannya tetap terbuka lengkap dengan sebabnya.
- `rotate180 = True`: kamera di robot ini terpasang terbalik.

## r4 — uji ambil korban di meja

- Tombol **Vision saja / 1 slot + gerak / 3 slot + gerak** — lompat langsung ke
  ruang korban, melewati seluruh navigasi.
- Mode `tanpa_gerak`: menilai vision tanpa satu pun perintah gerak dikirim.
- **Tombol Berdiri (`b`).** Ini menutup lubang desain: firmware boot dengan
  servo **lemas**, dan `w` saja tidak menyalakan PWM. Sebelum ini tidak ada
  satu pun jalur di HUD yang mengirim `b` atau `m1`, jadi robot memang tidak
  akan pernah bergerak.

## r5 — mode JEJAK

- State `JEJAK`: boneka boleh digeser ke mana saja, robot memutar badan
  mengikutinya, lalu **berhenti menunggu perintah operator**. Tidak pernah
  pindah state sendiri dan sengaja tanpa batas waktu.
- Tombol pendamping: **Maju ke korban** (LiDAR menutup jarak ke `standoff_cm`),
  **Capit BUKA/TUTUP** (`g100`/`g0`), **Lengan OFF** (`n`).
- **Perbaikan besar — "kalau dekat mau, kalau agak jauh diam".** Pita tinggi
  bbox ketat 0,35–0,90 membuang sasaran yang masih jauh. Ditambah
  `jejak_h_min/max = 0,06/0,98`. Ternyata `CENTER` **masih** memakai gerbang
  ketat karena ada satu baris `lolos = juri.saring(dets, w, h)` tanpa syarat
  yang tertinggal. Sekarang gerbang dipilih menurut tugas state dan dihitung
  sekali saja.

## r6 — UI tab, tooltip, restart, manual penuh — dan bug terparah

- Halaman dipecah jadi lima tab supaya tidak perlu scroll panjang.
- **Tooltip** di tiap tombol dan kotak: menjelaskan arti "slot", arti "gerak",
  dan perintah firmware apa yang sebenarnya dikirim.
- Tombol **Restart program** (`os.execv`, PID tetap, systemd tidak menganggap
  crash) — bisa restart tanpa SSH.
- **Mode manual penuh**: semua perintah lewat tombol atau kotak teks bebas,
  dengan **Kirim** (lewat whitelist) dan **Kirim PAKSA** (menembus daftar
  hitam, pakai konfirmasi, tercatat `[PAKSA]`).
- **BUG: seluruh HUD kosong padahal `curl /state` mengembalikan JSON yang
  benar.** Penyebabnya `\n` di dalam string Python **biasa** (`"""`) berubah
  jadi baris baru sungguhan di tengah string JavaScript pada tooltip Kirim
  PAKSA → `SyntaxError` → **seluruh** `<script>` mati → semua kolom tinggal
  nilai default `-`. Videonya tetap jalan karena `<img src="/stream.mjpg">`
  tidak butuh JS sama sekali. **Perbaikan:** `HALAMAN = r"""…"""`, ditambah
  **uji 18** yang mengekstrak blok `<script>` dan menjalankan `node --check`
  tiap kali uji dijalankan.
  Pelajarannya: `curl` sudah menunjuk ke sisi browser sejak awal — jam-jam
  berikutnya terbuang menggeledah sisi serial.

## r7 — warna kontras & diagnosis serial

**Berkas baru:** `cek_serial.py`

- Kotak korban vs dummy diberi warna yang benar-benar kontras; RAGU kuning,
  bukan merah. Sebelumnya tidak terbaca yang mana korban.
- `cek_serial.py`: enam pemeriksaan berurutan **memakai `cari_port()` milik
  HUD sendiri**, jadi yang diuji jalur yang sungguh dipakai — interpreter/venv,
  pyserial, glob port, izin & grup, pencuri port (`ModemManager`, `brltty`,
  layanan HUD sendiri), lalu benar-benar membuka port dan mengirim `m`.
- Penghitung `tx`/`rx` di strip status: memisahkan "tidak terkirim" dari
  "terkirim tapi tidak dibalas".

## r8 — daya

**Berkas baru:** `hemat_daya.sh`

- Gejalanya: Pi **mati mendadak** begitu masuk mode JEJAK, padahal multimeter
  menunjukkan 5,03 V. Multimeter merata-rata; dia tidak bisa melihat lekukan
  10 ms. Penyebabnya transien inferensi 640 px di 3 thread.
- `jejak_hz = 8.0` — batas inferensi per detik saat menjejak.
- Kelas `Kesehatan`: membaca `thermal_zone0` dan `vcgencmd get_throttled`,
  ditampilkan di strip status; merah kalau under-voltage **sedang** terjadi.
- JPEG dibatasi ~15 Hz **dan** hanya saat ada yang menonton.
- `hemat_daya.sh` (`--status` / terapkan / `--balik`): menonaktifkan
  `arm_freq`/`over_voltage`/`gpu_freq`/`force_turbo`, memasang
  `arm_freq_max=1800`, menambah `--threads 2 --width 640 --height 480 --fps 30`
  ke ExecStart layanan, dan memindahkan target boot ke `multi-user.target`.
  `config.txt` dicadangkan ke `config.txt.sebelum-hemat`.
- **Insiden:** rilis pertama `hemat_daya.sh` ternyata berisi byte
  `mission_hud.py` (salah commit dari saya) — gejalanya
  `ModuleNotFoundError: onnxruntime` "di baris 56" pada skrip bash. Sudah
  diperiksa ulang di perangkat (4950 byte, shebang bash, `bash -n` bersih).

### Riwayat perangkat keras yang menempel di rilis ini

- **Badai over-current di semua port USB** → didiagnosis sebagai back-feed pad
  VUSB–VIN Teensy 4.1. Tim elektrik mengikir pad-nya, over-current hilang.
  **Konsekuensi permanen:** sesudah dipotong, USB saja **tidak lagi**
  menyalakan Teensy — VIN robot harus hidup supaya papannya muncul dan bisa
  di-upload.
- **"REM DARURAT" tiap kali Enter ditekan** dari terminal Windows: Enter
  mengirim CR+LF dan parser firmware mengeksekusi pada `'\r'` **dan** `'\n'`,
  jadi baris kosong memicu rem. HUD tidak terkena karena mengirim `cmd + "\n"`.
  Perbaikan 3 baris sudah diusulkan ke Vincent.

## r9 — rantai AMBIL KORBAN otomatis  ← rilis ini

- Lima state baru, berurutan, masing-masing dengan batas waktu sendiri:

  | State | Isi | Batas |
  |---|---|---|
  | `AMBIL_TENGAH` | tengahkan badan ke korban dengan kamera | 30 s |
  | `AMBIL_MAJU` | maju bertahap sampai LiDAR = `capit_cm` | 30 s |
  | `AMBIL_SIAP` | `g100` buka capit + `a<r> <h>` posisikan lengan | 10 s |
  | `AMBIL_JEPIT` | `g0` tutup capit | 8 s |
  | `AMBIL_ANGKAT` | `a<r> <h+angkat>` lalu `BERES` | 10 s |

  Dipecah lima, bukan satu blok, supaya kalau berhenti kolom `sebab` menyebut
  **langkah mana** yang gagal.
- Maju dibatasi **maksimal 6 cm sekali jalan** lalu ukur ulang — odometri gait
  meleset beberapa cm per langkah, jadi satu lompatan besar berisiko menabrak.
- `AMBIL_MAJU` **gagal bersih** kalau sudah kelewat dekat: firmware belum punya
  perintah mundur. Ini batas nyata, bukan bug, dan sengaja tidak dipura-purakan.
- `ambil_hanya_korban` (default **True**): menolak menjalankan rantai kalau
  kelas yang terbaca `dummy`.
- Tombol **Mulai AMBIL otomatis** (dengan konfirmasi) dan **Batalkan AMBIL**
  (kosongkan antrean + `s` + kembali `IDLE`) di tab Robot, plus tiga baris
  pemantau: langkah, jarak depan, target capit.
- Semua lewat antrean `Aksi` yang non-blokir — kamera dan halaman web tetap
  hidup selama rantai berjalan.

**Yang belum bisa dan harus diukur di robot sungguhan:** `capit_cm = 12.0`
adalah **tebakan aman**, bukan hasil ukur. Lengan hanya menjangkau ~44 mm dari
bahu yang duduk 50 mm di depan pusat badan, jadi angka ini hampir pasti perlu
diubah. Juga tidak ada sensor cengkeraman — HUD **tidak bisa** tahu korbannya
benar-benar terpegang; langkah terakhir hanya menulis "PERIKSA MATA".

## r9b — crosscheck v1.7 langsung ke sumbernya

Diperiksa berkas per berkas terhadap v1.61, bukan dari ingatan.

- Tombol v1.7 di tab Manual: `T`/`T0`–`T3` (profil medan), `N`/`N0`–`N3` (PD vs
  fuzzy x sumber turunan), `Z`/`Z1`/`Z0` (kemudi lateral), `Y`/`Y0` (sudut
  dinding & catat bias), `i`/`i1`/`i0` (abaikan sensor depan). Tooltipnya
  dibaca dari `Hexapod_Unlimited.ino`, tidak ditebak.
- **`ambil_maks_cm = 60.0` (baru).** v1.7 menaikkan `LIDAR_MAX_CM` 70 -> 130,
  jadi dinding sejauh 1 meter kini terbaca sebagai ANGKA, bukan `jauh`. Tanpa
  pagar ini rantai AMBIL akan berjalan menyeberangi ruangan 6 cm sekali jalan
  menuju dinding yang disangkanya korban. Dulu mustahil: bacaan sejauh itu
  selalu jatuh ke `jauh` dan langsung GAGAL.
- Dipastikan HUD tidak perlu diubah untuk sisanya: `LidarArray.cpp`
  byte-identik (parser tabel `l` aman), `.ino` hanya menambah case,
  `Mission::status()` tetap format `  kunci : nilai`, lengan & capit tidak
  tersentuh.
- **Jebakan yang ditemukan:** `CALIB_VERSION` tetap 9 dan 25 parameternya
  urutannya sama, jadi blob EEPROM lama tetap dipakai dan default baru
  `wall.setpoint` 19->17 serta `wall.min` 15->13 **tidak berlaku sendiri**.
  Harus `Qwall.setpoint 17`, `Qwall.min 13`, lalu `W`. Angka itu hasil ukur
  lebar arena sungguhan: **45 cm**, bukan 60 cm.
- **Fuzzy mati secara bawaan** (`_wallSamar = false`, RAM saja, bukan blob
  Calib) -- flash v1.7 saja masih PD. Perlu `N1`/`N3`.
- **Koreksi klaim "fuzzy dengan 3 LiDAR".** Fuzzy hanya mengganti rumus PD
  untuk error dinding di SATU sisi, dari sepasang sensor sisi yang sama.
  Sensor DEPAN tetap pada tugas lama (berhenti/belok di `FRONT_STOP_CM`).
  Yang benar-benar baru: `N2`/`N3` menghitung turunan dari SUDUT badan
  `atan2(selisih pasangan, WALL_BASE_CM)`, bukan dari selisih waktu.
- `README.md` Vincent byte-identik dengan v1.61 -- tidak ikut diperbarui.

112 uji lulus.
