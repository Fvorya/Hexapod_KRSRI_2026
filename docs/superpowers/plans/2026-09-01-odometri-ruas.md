# Odometri Per-Ruas Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robot bisa menjawab "sudah berapa cm sejak terakhir dinolkan" di mode gerak apa pun, dan bisa disuruh berhenti pada jarak tertentu, supaya slip gait bisa diukur dengan meteran di arena.

**Architecture:** Akumulator jarak duduk di `HexaGait`, diintegrasikan dari `Δfase` dan vektor gerak yang sudah di-slew **dan** sudah dinormalisasi — bukan dari perintah yang diminta. `Hexapod` meneruskannya sebagai fasad. Rem jarak duduk di `Navigation`, diperiksa **sebelum** `navUpdate()` keluar pada `NAV_DIAM`, sehingga ia mencakup mode navigasi maupun `w` manual, dan bisa memanggil `navBerhenti()` + `robot.stop()` dari satu tempat.

**Tech Stack:** C++17, Arduino/Teensy 4.1, simulasi PC lewat `test-pc/build.sh` (g++).

## Global Constraints

- **Spec:** `docs/superpowers/specs/2026-09-01-odometri-ruas-design.md`. Baca sebelum mulai.
- **Jangan menaikkan `CALIB_VERSION`.** Menambah parameter `Calib` membuang blob EEPROM dan menghapus gain dinding yang sudah disetel. Faktor skala hidup di RAM.
- **Jangan menambah dependensi atau file sumber firmware baru.** Semua perubahan firmware adalah modifikasi file yang sudah ada.
- **`build.sh` mengkompilasi seluruh sketsa dengan `-Wall -Wextra` dan `set -e`.** Peringatan apa pun menggagalkan build. Variabel yang tidak terpakai termasuk.
- **Komentar ditulis dalam bahasa Indonesia**, mengikuti gaya berkas sekitarnya: jelaskan *kenapa*, bukan *apa*.
- **`g++` ada tapi TIDAK di PATH bawaan shell.** MSYS2 memasangnya di `/c/msys64/ucrt64/bin`. Awali tiap perintah build dengan:
  `export PATH="/c/msys64/ucrt64/bin:$PATH"` — tanpa itu `build.sh` berhenti di `g++: command not found`.
- **Akar repositori adalah folder `Hexapod/`**, bukan `Hexapod_Unlimited/`. Semua jalur di rencana ini relatif terhadap akar itu. `legacy-2026/` di-ignore dan bukan urusan rencana ini.
- **Kondisi awal sudah hijau.** `./build.sh` lulus seluruhnya di commit `738d9e7` (19 sim). Kalau ada sim yang gagal sesudah suntingan Anda, itu suntingan Anda.

## Deviasi dari spec, disengaja

Spec §5.3 menaruh pemeriksaan rem jarak di `loop()` pada `.ino`. Rencana ini memindahkannya ke `Navigation::navUpdate()`, **di atas** baris `if (_mode == NAV_DIAM) return;`.

Alasannya: `Navigation` sudah memegang referensi `_robot` **dan** `navBerhenti()`, jadi kedua jalur berhenti yang dibutuhkan ada di satu tempat; `navUpdate()` tetap dipanggil tiap loop tanpa memandang mode, jadi cakupan `w` manual tidak hilang; dan yang menentukan — logika di `.ino` tidak bisa diuji oleh simulasi mana pun tanpa ikut mengkompilasi seluruh sketsa, sedangkan di `Navigation` ia diuji langsung seperti `sim_pivot` menguji pivot.

Sifat yang dijaga spec tetap utuh: rem **hanya menolkan** dan tidak pernah menulis vektor gerak.

Spec §8 uji 1 meminta perbandingan terhadap angka hafalan 10,7 cm/detik dari `sim_laju`. Rencana ini **mengukur ulang** kebenaran acuannya dari `legTargets` dengan metode yang sama seperti `sim_laju`, bukan menuliskan 10,7 sebagai konstanta. Alasannya: 10,7 hanya berlaku untuk `stepLength` 60 mm dan `cycleTime` 900 ms, jadi menuliskannya membuat uji ini gagal palsu begitu ada orang menyetel gait. Mengukur ulang menguji hal yang sama tanpa titik rapuh itu.

## File Structure

| Berkas | Tanggung jawab |
|---|---|
| `Hexapod_Unlimited/HexaGait.h` | Deklarasi `_jarakMm`, `_skalaOdo`, dan empat antarmukanya |
| `Hexapod_Unlimited/HexaGait.cpp` | Integrasi jarak di dalam `update()`, sesudah normalisasi |
| `Hexapod_Unlimited/Hexapod.h` | Fasad: `jarakCm()`, `jarakNol()`, `setSkalaOdo()`, `skalaOdo()` |
| `Hexapod_Unlimited/Navigation.h` | Deklarasi rem jarak |
| `Hexapod_Unlimited/Navigation.cpp` | Pemeriksaan rem di awal `navUpdate()` |
| `Hexapod_Unlimited/Hexapod_Unlimited.ino` | Perintah serial `D` dan barisnya di bantuan `h` |
| `test-pc/sim/sim_odo.cpp` | **Baru.** Enam uji: silang terhadap gait asli, normalisasi, skala pada penambahan, clamp skala, rem, penolakan sasaran <= 0 |
| `test-pc/build.sh` | Daftarkan `sim_odo` |

---

### Task 1: Akumulator jarak di HexaGait

**Files:**
- Create: `test-pc/sim/sim_odo.cpp`
- Modify: `test-pc/build.sh:23`
- Modify: `Hexapod_Unlimited/HexaGait.h` (blok publik sesudah `profile()`, blok privat sesudah `_phase`)
- Modify: `Hexapod_Unlimited/HexaGait.cpp` (dua blok di dalam `update()`: akumulasi fase, dan normalisasi langkah)
- Modify: `Hexapod_Unlimited/Hexapod.h` (sesudah `setGaitProfile`)

**Catatan nomor baris:** Step 5 mengubah dua blok di `update()`. Suntingan pertama menambah satu baris, jadi nomor baris blok kedua bergeser. Cari berdasarkan **isi** yang dikutip di tiap step, bukan nomor baris.

**Interfaces:**
- Consumes: `HexaGait::update()`, `setMoveVector()`, `setProfile()`, `legTargets[6]`, dan `GaitProfile` — semuanya sudah ada.
- Produces:
  - `float HexaGait::jarakMm() const` — jarak bertanda sejak nol terakhir. Mundur mengurangi.
  - `void HexaGait::jarakNol()`
  - `void HexaGait::setSkalaOdo(float s)` — di-clamp ke 0,5..1,5
  - `float HexaGait::skalaOdo() const`
  - `float Hexapod::jarakCm() const`, `void Hexapod::jarakNol()`, `void Hexapod::setSkalaOdo(float)`, `float Hexapod::skalaOdo() const`

- [ ] **Step 1: Tulis uji yang gagal**

Buat `test-pc/sim/sim_odo.cpp`:

```cpp
// ODOMETRI GAIT: apakah jarak yang dilaporkan sama dengan jarak yang
// BENAR-BENAR ditempuh badan menurut generator gait yang asli?
//
// Kebenaran acuannya bukan angka hafalan, melainkan diukur ulang dari
// legTargets dengan cara yang sama seperti sim_laju: badan bergerak
// berlawanan dengan pergeseran kaki yang sedang MENAPAK, jadi
// laju = -jumlah(delta y kaki stance) / waktu. Kalau odometer dan gait
// berbeda, salah satunya salah -- dan keduanya membaca stepLength serta
// cycleTime yang sama, jadi selisih di luar toleransi berarti rumus
// odometernya yang keliru.
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdio>
#include <cmath>

#include "Calib.h"
#include "HexaGait.h"

static HexaGait gait;

static const float STEP_LEN   = 60.0f;    // mm
static const float CYCLE_MS   = 900.0f;
static const float TICK_MS    = 10;

static void siapkan(float cmdMaju, float cmdPutar) {
    gParam[K_GAIT_STEP_LENGTH] = STEP_LEN;
    gParam[K_GAIT_CYCLE_TIME]  = CYCLE_MS;
    gParam[K_GAIT_DUTY]        = 0.5f;
    gait = HexaGait();
    gait.begin();
    gait.setProfile({ GAIT_STEP_HEIGHT, STEP_LEN, CYCLE_MS, STAND_HEIGHT, STAND_RADIUS });
    gait.setMoveVector(0.0f, cmdMaju, cmdPutar);
    // Biarkan slew vektor gerak dan ramp profil selesai (2 detik) supaya
    // yang diukur adalah keadaan tunak, bukan percepatan.
    for (int i = 0; i < 200; i++) { __nowMs += (uint32_t)TICK_MS; gait.update(); }
    gait.jarakNol();
}

// Jarak tempuh badan menurut legTargets -- kebenaran acuan.
// Hanya sah untuk maju MURNI (tanpa putar), karena dengan yaw tiap kaki
// bergeser dengan besaran berbeda.
static float ukurAcuanMm(int tick) {
    const float zHome = -STAND_HEIGHT;
    float z0 = gait.legTargets[0].z, z1 = gait.legTargets[1].z;
    float y0 = gait.legTargets[0].y, y1 = gait.legTargets[1].y;
    float maju = 0.0f;
    for (int i = 0; i < tick; i++) {
        __nowMs += (uint32_t)TICK_MS;
        gait.update();
        float nz0 = gait.legTargets[0].z, nz1 = gait.legTargets[1].z;
        float ny0 = gait.legTargets[0].y, ny1 = gait.legTargets[1].y;
        bool st0 = (fabsf(nz0 - zHome) < 0.001f) && (fabsf(z0 - zHome) < 0.001f);
        bool st1 = (fabsf(nz1 - zHome) < 0.001f) && (fabsf(z1 - zHome) < 0.001f);
        if      (st0) maju += -(ny0 - y0);
        else if (st1) maju += -(ny1 - y1);
        z0 = nz0; z1 = nz1; y0 = ny0; y1 = ny1;
    }
    return maju;
}

static int jalanSaja(int tick) {
    for (int i = 0; i < tick; i++) { __nowMs += (uint32_t)TICK_MS; gait.update(); }
    return tick;
}

int main() {
    Calib::load();

    printf("\n== UJI 1: odometer vs jarak nyata menurut gait (maju 1.0, 10 siklus) ==\n");
    {
        siapkan(1.0f, 0.0f);
        int tick = (int)(10.0f * CYCLE_MS / TICK_MS);
        float acuan = ukurAcuanMm(tick);
        float odo   = gait.jarakMm();
        float galat = (acuan > 0.001f) ? fabsf(odo - acuan) / acuan * 100.0f : 999.0f;
        printf("  acuan gait %.1f mm, odometer %.1f mm, galat %.2f%%\n", acuan, odo, galat);
        if (galat > 2.0f) { printf("  GAGAL: galat di luar 2%%\n"); return 1; }
        printf("  OK\n");
    }

    printf("\n== UJI 2: normalisasi langkah ikut terhitung ==\n");
    // Maju sambil berputar membuat gait memangkas panjang langkah
    // (HexaGait.cpp: magMax > stepLength -> f < 1). Kalau odometer
    // mengabaikan f, kedua angka di bawah akan IDENTIK.
    {
        siapkan(0.8f, 0.0f);
        jalanSaja(500);
        float lurus = gait.jarakMm();

        siapkan(0.8f, 0.5f);
        jalanSaja(500);
        float belok = gait.jarakMm();

        printf("  lurus %.1f mm, sambil putar %.1f mm\n", lurus, belok);
        if (belok >= lurus * 0.95f) {
            printf("  GAGAL: jarak saat berputar tidak dipangkas -- faktor f tidak dipakai\n");
            return 1;
        }
        printf("  OK\n");
    }

    printf("\n== UJI 3: skala berlaku pada PENAMBAHAN, bukan pada pembacaan ==\n");
    {
        siapkan(0.8f, 0.0f);
        gait.setSkalaOdo(1.0f);
        gait.jarakNol();
        jalanSaja(500);
        float a = gait.jarakMm();

        gait.setSkalaOdo(0.5f);
        float sesudahSetel = gait.jarakMm();
        if (fabsf(sesudahSetel - a) > 0.001f) {
            printf("  GAGAL: menyetel skala menulis ulang jarak yang sudah terkumpul "
                   "(%.3f -> %.3f)\n", a, sesudahSetel);
            return 1;
        }

        jalanSaja(500);
        float tambahan = gait.jarakMm() - a;
        printf("  tambahan pertama %.1f mm, tambahan kedua %.1f mm (harus ~separuh)\n",
               a, tambahan);
        if (fabsf(tambahan - a * 0.5f) > a * 0.05f) {
            printf("  GAGAL: skala tidak dipakai pada penambahan\n"); return 1;
        }
        gait.setSkalaOdo(1.0f);
        printf("  OK\n");
    }

    printf("\n== UJI 4: skala di-clamp ke 0,5 .. 1,5 ==\n");
    {
        gait.setSkalaOdo(9.0f);
        if (fabsf(gait.skalaOdo() - 1.5f) > 0.001f) {
            printf("  GAGAL: 9.0 tidak di-clamp ke 1.5 (dapat %.3f)\n", gait.skalaOdo());
            return 1;
        }
        gait.setSkalaOdo(0.0f);
        if (fabsf(gait.skalaOdo() - 0.5f) > 0.001f) {
            printf("  GAGAL: 0.0 tidak di-clamp ke 0.5 (dapat %.3f)\n", gait.skalaOdo());
            return 1;
        }
        gait.setSkalaOdo(1.0f);
        printf("  OK\n");
    }

    printf("\nSEMUA UJI LULUS\n");
    return 0;
}
```

- [ ] **Step 2: Daftarkan sim_odo di build.sh**

Di `test-pc/build.sh:23`, tambahkan `sim_odo` di ujung daftar:

```bash
DAFTAR="${1:-sim_pivot sim_body sim_lidar sim_open sim_reinit sim_peta sim_hantu sim_jejak sim_isolasi sim_param sim_goyang sim_wall sim_laju sim_servolaju sim_dinding sim_yaw sim_depan sim_boot sim_misi sim_odo}"
```

- [ ] **Step 3: Jalankan uji, pastikan GAGAL**

Run: `cd test-pc && ./build.sh sim_odo`
Expected: gagal saat kompilasi — `'class HexaGait' has no member named 'jarakNol'`.

- [ ] **Step 4: Deklarasikan antarmuka di HexaGait.h**

Di blok publik, sesudah `GaitProfile profile() const { ... }`:

```cpp
    // ODOMETRI. Jarak bertanda yang sudah ditempuh badan sejak jarakNol();
    // mundur mengurangi. Dihitung dari fase gait dan vektor gerak yang sudah
    // di-slew DAN dinormalisasi, jadi ia mengukur apa yang benar-benar
    // dilakukan kaki -- bukan apa yang diperintahkan.
    float jarakMm() const { return _jarakMm; }
    void  jarakNol()      { _jarakMm = 0.0f; }

    // Faktor slip. Dikalikan pada tiap PENAMBAHAN, bukan saat dibaca, supaya
    // menyetelnya di tengah jalan tidak menulis ulang jarak yang sudah
    // terkumpul. Hanya di RAM: menambah parameter Calib akan menaikkan
    // CALIB_VERSION dan membuang seluruh gain yang sudah disetel.
    void  setSkalaOdo(float s) { _skalaOdo = clampf(s, 0.5f, 1.5f); }
    float skalaOdo() const     { return _skalaOdo; }
```

Di blok privat, sesudah `float _phase;`:

```cpp
    float _jarakMm  = 0.0f;   // odometri, mm, bertanda
    float _skalaOdo = 1.0f;   // faktor slip, disetel 'Ds'
```

- [ ] **Step 5: Integrasikan jarak di HexaGait.cpp**

Di `update()`, ganti blok akumulasi fase (sekarang `HexaGait.cpp:87-92`) supaya `Δfase` disimpan:

```cpp
    // 3. Fase Diakumulasi (kebal terhadap transisi cycleTime)
    if (_prof.cycleTime < 100.0f) _prof.cycleTime = 100.0f;
    float dPhase = dt * 1000.0f / _prof.cycleTime;
    _phase += dPhase;
    while (_phase >= 1.0f) {
        _phase -= 1.0f;
    }
```

Lalu ganti blok normalisasi (sekarang `HexaGait.cpp:107-113`) supaya `f` hidup di luar `if`, dan tambahkan akumulasi jarak sesudahnya:

```cpp
    // Pangkas (Normalisasi) bersama-sama jika ada kaki yang melampaui stepLength
    float f = 1.0f;
    if (magMax > _prof.stepLength && magMax > 0.001f) {
        f = _prof.stepLength / magMax;
        for (int leg = 0; leg < 6; leg++) {
            sxa[leg] *= f;
            sya[leg] *= f;
        }
    }

    // ODOMETRI. Dihitung DI SINI, sesudah normalisasi, karena hanya di sini
    // 'f' diketahui -- dan tanpa 'f' jarak saat menyusuri dinding akan
    // terhitung lebih jauh daripada saat lurus karena bug rumus, lalu
    // disalahartikan sebagai slip mekanis.
    //
    // Suku yaw batal sendiri saat dirata-rata enam kaki (rx simetris
    // kiri-kanan), jadi cukup _curY; tidak perlu merata-rata sya[].
    // Memakai dPhase, bukan dt, dengan alasan yang sama seperti turunan PD
    // dinding memakai stempel sampel LiDAR: kebal terhadap loop tersendat.
    _jarakMm += _skalaOdo * 2.0f * _curY * _prof.stepLength * f * dPhase;
```

- [ ] **Step 6: Teruskan lewat fasad Hexapod**

Di `Hexapod.h`, sesudah `void setGaitProfile(const GaitProfile& p) { ... }`:

```cpp
    // Odometri gait. Lihat HexaGait untuk arti dan batasnya.
    float jarakCm() const      { return _gait.jarakMm() * 0.1f; }
    void  jarakNol()           { _gait.jarakNol(); }
    void  setSkalaOdo(float s) { _gait.setSkalaOdo(s); }
    float skalaOdo() const     { return _gait.skalaOdo(); }
```

`jarakMm()` dan `skalaOdo()` di `HexaGait` harus `const` supaya bisa dipanggil dari metode `const` ini; deklarasi di Step 4 sudah begitu.

- [ ] **Step 7: Jalankan uji, pastikan LULUS**

Run: `cd test-pc && ./build.sh sim_odo`
Expected: `SEMUA UJI LULUS`, tanpa peringatan kompilasi.

- [ ] **Step 8: Jalankan seluruh simulasi, pastikan tidak ada yang rusak**

Run: `cd test-pc && ./build.sh`
Expected: semua sim lulus. `sim_laju` sangat penting di sini — ia membaca gait yang sama dan akan menangkap kalau restrukturisasi blok normalisasi mengubah perilaku gait.

- [ ] **Step 9: Commit**

```bash
git add Hexapod_Unlimited/HexaGait.h Hexapod_Unlimited/HexaGait.cpp Hexapod_Unlimited/Hexapod.h test-pc/sim/sim_odo.cpp test-pc/build.sh
git commit -m "feat(odometri): jarak tempuh dari fase gait, hormati normalisasi langkah"
```

---

### Task 2: Rem jarak dan perintah serial `D`

**Files:**
- Modify: `Hexapod_Unlimited/Navigation.h` (blok publik dekat `navUpdate`, dan blok privat)
- Modify: `Hexapod_Unlimited/Navigation.cpp` (awal `navUpdate()`, sekarang baris 472-473)
- Modify: `Hexapod_Unlimited/Hexapod_Unlimited.ino` (parser perintah, dan bantuan `h`)
- Modify: `test-pc/sim/sim_odo.cpp` (tambah UJI 5)

**Interfaces:**
- Consumes dari Task 1: `Hexapod::jarakCm()`, `Hexapod::jarakNol()`, `Hexapod::setSkalaOdo(float)`, `Hexapod::skalaOdo()`.
- Produces:
  - `void Navigation::remJarakPasang(float cm)` — nolkan jarak lalu pasang rem; `cm <= 0` menolak
  - `void Navigation::remJarakLepas()`
  - `bool Navigation::remJarakAda() const`
  - `float Navigation::remJarakSasaran() const` — cm; 0 bila tidak terpasang

- [ ] **Step 1: Tulis uji yang gagal**

Tambahkan di `test-pc/sim/sim_odo.cpp`, tepat sebelum `printf("\nSEMUA UJI LULUS\n");`.

Berkas ini sekarang perlu `Navigation` beserta ketergantungannya, jadi tambahkan juga di blok `#include` di atas:

```cpp
#include <Wire.h>
#include <VL53L1X.h>
#include "Imu.h"
#include "Hexapod.h"
#include "LidarArray.h"
#include "Navigation.h"
```

dan di bawah `static HexaGait gait;`:

```cpp
// UJI 5 memakai Hexapod/Navigation yang asli. Objeknya berdiri sendiri dari
// 'gait' di atas -- uji 1..4 menguji generator gait langsung, uji 5 menguji
// rem lewat fasad yang benar-benar dipakai firmware.
static Imu        imu;
static Hexapod    robot;
static LidarArray lidar;
static Navigation nav(imu, robot, lidar);
```

Uji itu sendiri:

```cpp
    printf("\n== UJI 5: rem jarak menghentikan robot yang berjalan MANUAL ==\n");
    // Jalur 'w' manual: robot.walk() dipanggil langsung, mode navigasi tetap
    // NAV_DIAM. Inilah kasus yang hilang kalau rem ditaruh sesudah
    // 'if (_mode == NAV_DIAM) return;' -- dan justru inilah cara mengukur
    // slip jalan lurus di lantai terbuka tanpa dinding.
    {
        robot.begin();
        robot.arm();
        robot.setSkalaOdo(1.0f);

        nav.remJarakPasang(80.0f);
        if (!nav.remJarakAda()) { printf("  GAGAL: rem tidak terpasang\n"); return 1; }
        if (fabsf(robot.jarakCm()) > 0.001f) {
            printf("  GAGAL: memasang rem tidak menolkan jarak\n"); return 1;
        }

        robot.walk(1.0f, 0.0f, 0.0f);
        int n = 0;
        while (nav.remJarakAda() && n < 20000) {
            __nowMs += (uint32_t)TICK_MS;
            nav.navUpdate();
            robot.update();
            n++;
        }
        float akhir = robot.jarakCm();
        printf("  berhenti pada %.2f cm sesudah %d ms\n", akhir, n * (int)TICK_MS);

        if (nav.remJarakAda()) { printf("  GAGAL: rem tidak pernah menyala\n"); return 1; }
        if (akhir < 80.0f || akhir > 81.0f) {
            printf("  GAGAL: berhenti di luar 80..81 cm\n"); return 1;
        }

        // Rem TIDAK menghentikan robot seketika, dan itu disengaja.
        // robot.stop() hanya menaruh vektor gerak target di nol; HexaGait
        // menurunkannya dengan laju GAIT_SLEW_RATE (3,0/detik) supaya kaki
        // tidak menyentak di tengah langkah. Dari maju penuh itu ~330 ms
        // perlambatan, dan odometer ikut menghitung selama itu -- robot
        // meluncur sekitar 2 cm melewati sasaran.
        //
        // Untuk pengukuran slip ini tidak merugikan: angka odometri akhir
        // DAN jarak meteran sama-sama sudah termasuk luncuran itu, jadi
        // perbandingannya tetap setara. Yang diuji di sini dua hal:
        // luncurannya terbatas, lalu jaraknya BENAR-BENAR beku -- rem tidak
        // menyala lagi dan navigasi tidak menghidupkan robot kembali.
        for (int i = 0; i < 100; i++) {          // 1 detik: biarkan ramp habis
            __nowMs += (uint32_t)TICK_MS;
            nav.navUpdate();
            robot.update();
        }
        float sesudahRamp = robot.jarakCm();
        printf("  sesudah ramp perlambatan: %.2f cm (meluncur %.2f cm)\n",
               sesudahRamp, sesudahRamp - akhir);
        if (sesudahRamp - akhir > 3.0f) {
            printf("  GAGAL: meluncur lebih jauh dari yang bisa dijelaskan ramp\n");
            return 1;
        }

        for (int i = 0; i < 200; i++) {
            __nowMs += (uint32_t)TICK_MS;
            nav.navUpdate();
            robot.update();
        }
        if (fabsf(robot.jarakCm() - sesudahRamp) > 0.1f) {
            printf("  GAGAL: masih maju sesudah ramp habis (%.2f -> %.2f cm)\n",
                   sesudahRamp, robot.jarakCm());
            return 1;
        }
        printf("  OK -- rem menyala di %.2f cm, meluncur %.2f cm, lalu beku.\n",
               akhir, sesudahRamp - akhir);
    }

    printf("\n== UJI 6: rem menolak sasaran <= 0 ==\n");
    {
        nav.remJarakLepas();
        nav.remJarakPasang(0.0f);
        if (nav.remJarakAda()) { printf("  GAGAL: rem 0 cm diterima\n"); return 1; }
        nav.remJarakPasang(-5.0f);
        if (nav.remJarakAda()) { printf("  GAGAL: rem negatif diterima\n"); return 1; }
        printf("  OK\n");
    }
```

- [ ] **Step 2: Jalankan uji, pastikan GAGAL**

Run: `cd test-pc && ./build.sh sim_odo`
Expected: gagal saat kompilasi — `'class Navigation' has no member named 'remJarakPasang'`.

- [ ] **Step 3: Deklarasikan rem di Navigation.h**

Di blok publik, sesudah `void navUpdate();`:

```cpp
    // REM JARAK. Menghentikan robot sesudah menempuh jarak tertentu, di mode
    // gerak APA PUN -- termasuk 'w' manual, karena pemeriksaannya duduk di
    // atas jalan keluar NAV_DIAM di navUpdate(). Dipakai untuk mengukur slip
    // gait: jalankan sejauh N cm menurut odometri, lalu ukur dengan meteran.
    //
    // Rem HANYA menolkan; ia tidak pernah menulis vektor gerak, jadi doktrin
    // satu-penulis yang dijaga navBerhenti() tetap utuh.
    void  remJarakPasang(float cm);   // nolkan jarak lalu pasang; cm <= 0 ditolak
    void  remJarakLepas();
    bool  remJarakAda() const     { return _remJarakCm > 0.0f; }
    float remJarakSasaran() const { return _remJarakCm; }
```

Di blok privat, dekat `_diamSejak`:

```cpp
    float _remJarakCm = 0.0f;   // 0 = rem tidak terpasang
```

- [ ] **Step 4: Terapkan rem di Navigation.cpp**

Tambahkan kedua fungsi tepat sebelum `void Navigation::navUpdate() {`:

```cpp
void Navigation::remJarakPasang(float cm) {
    if (cm <= 0.0f) {
        Serial.println("Rem jarak: sasaran harus lebih dari 0 cm. Tidak dipasang.");
        return;
    }
    _robot.jarakNol();
    _remJarakCm = cm;
    Serial.print("Rem jarak DIPASANG di "); Serial.print(cm, 1);
    Serial.println(" cm. Jarak dinolkan.");
    Serial.println("  Berlaku di mode gerak apa pun, termasuk 'w' manual.");
}

void Navigation::remJarakLepas() {
    if (_remJarakCm <= 0.0f) return;    // tidak terpasang -> jangan mencetak apa-apa
    _remJarakCm = 0.0f;
    Serial.println("Rem jarak DILEPAS.");
}
```

Lalu di awal `navUpdate()`, **di atas** `if (_mode == NAV_DIAM) return;`:

```cpp
void Navigation::navUpdate() {
    // REM JARAK diperiksa SEBELUM jalan keluar NAV_DIAM di bawah. Kalau
    // ditaruh sesudahnya, 'w' manual tidak akan pernah terkena rem -- dan
    // justru jalan manual itulah satu-satunya cara berjalan lurus tanpa
    // dinding, yaitu pengukuran slip yang paling bersih.
    if (_remJarakCm > 0.0f && _robot.jarakCm() >= _remJarakCm) {
        _remJarakCm = 0.0f;              // sekali pakai; jangan menyala lagi nanti
        navBerhenti("rem jarak tercapai.");   // mengurus mode navigasi
        _robot.stop();                        // mengurus 'w' manual
        Serial.print("Rem jarak: berhenti di "); Serial.print(_robot.jarakCm(), 1);
        Serial.println(" cm.");
    }

    if (_mode == NAV_DIAM) return;
```

- [ ] **Step 5: Jalankan uji, pastikan LULUS**

Run: `cd test-pc && ./build.sh sim_odo`
Expected: `SEMUA UJI LULUS`.

- [ ] **Step 6: Tambahkan perintah `D` di .ino**

Di `switch` parser perintah, sesudah blok `case 'v':`, tambahkan:

```cpp
        case 'D': {   // odometri: D=cetak, D<cm>=pasang rem, D0=lepas+nolkan
            if (s[1] == 's') {
                // argFloats() membaca mulai s+1, jadi s+1 di sini menaruh
                // titik baca tepat sesudah huruf 's'.
                float p[1] = {0};
                if (argFloats(s + 1, p, 1) >= 1) {
                    robot.setSkalaOdo(p[0]);
                    Serial.print("Skala odometri -> "); Serial.print(robot.skalaOdo(), 3);
                    if (fabsf(robot.skalaOdo() - p[0]) > 1e-3f)
                        Serial.print("  (diminta belum sah, DI-CLAMP ke 0,5 .. 1,5)");
                    Serial.println();
                    Serial.println("  Hanya di RAM. Kalau sudah pasti, tulis ke config.h.");
                } else {
                    Serial.println("Format: Ds<faktor>, misal Ds1.05");
                }
                break;
            }
            float p[1] = {0};
            if (argFloats(s, p, 1) < 1) {                 // 'D' polos
                Serial.print("Jarak tempuh : "); Serial.print(robot.jarakCm(), 1);
                Serial.println(" cm sejak terakhir dinolkan");
                Serial.print("  rem       : ");
                if (nav.remJarakAda()) { Serial.print(nav.remJarakSasaran(), 1);
                                         Serial.println(" cm"); }
                else Serial.println("tidak terpasang");
                Serial.print("  skala     : "); Serial.println(robot.skalaOdo(), 3);
                break;
            }
            if (p[0] <= 0.0f) {                            // 'D0'
                nav.remJarakLepas();
                robot.jarakNol();
                Serial.println("Jarak dinolkan, rem dilepas.");
                break;
            }
            nav.remJarakPasang(p[0]);
            break;
        }
```

- [ ] **Step 7: Lepas rem di jalur berhenti yang sudah ada**

Rem yang tertinggal terpasang bisa menyala di perjalanan berikutnya. `remJarakLepas()` sudah `return` diam-diam kalau rem tidak terpasang, jadi tidak ada baris tambahan yang tercetak saat rem memang tidak ada.

Tiga tempat. `case 's'`:

```cpp
        case 's': // Stop
            misi.batal("dihentikan pengguna.");
            nav.navBerhenti("dihentikan pengguna.");   // WAJIB: kalau tidak,
            robot.stop();                              // navUpdate() menyalakannya lagi
            nav.remJarakLepas();                       // jangan menyala di perjalanan berikutnya
            Serial.println("Robot berhenti.");
            break;
```

`case 'x'` — sisipkan sesudah `nav.navBerhenti(...)`:

```cpp
        case 'x': // Lemas darurat: PWM mati, servo bebas
            misi.batal("servo dilemaskan.");   // WAJIB sebelum nav: kalau tidak,
            nav.navBerhenti("servo dilemaskan.");  // misi menyalakan navigasi lagi
            nav.remJarakLepas();
```

Cabang Enter kosong (rem darurat) — sisipkan sesudah `robot.stop();`:

```cpp
            } else {
                // Fitur Keselamatan: Tekan Enter kosong untuk rem darurat
                misi.batal("rem darurat.");
                nav.navBerhenti("rem darurat.");
                if (demoOn) demoStop("rem darurat.");
                if (goyangOn) goyangStop("rem darurat.");
                robot.stop();
                nav.remJarakLepas();
                Serial.println("!! REM DARURAT (Vektor = 0) !!");
            }
```

`m0` sengaja **tidak** disentuh: `Mission::batal()` sudah memanggil `_nav.navBerhenti()`, dan orang yang membatalkan misi belum tentu ingin rem ukurnya ikut lepas.

- [ ] **Step 8: Tambahkan baris bantuan**

Di teks bantuan `case 'h'`, sesudah baris `v : Status navigasi + jarak sekitar`:

```cpp
            Serial.println("  D      : Jarak tempuh, keadaan rem, dan skala odometri");
            Serial.println("  D<cm>  : Nolkan jarak lalu pasang rem di <cm> (misal D80)");
            Serial.println("  D0     : Nolkan jarak dan lepas rem");
            Serial.println("  Ds<f>  : Faktor slip odometri, RAM saja (misal Ds1.05)");
```

- [ ] **Step 9: Jalankan seluruh simulasi**

Run: `cd test-pc && ./build.sh`
Expected: semua lulus. Bagian pertama build (`-fsyntax-only -Wall -Wextra` atas seluruh sketsa) yang memeriksa perintah `D` baru; `sim_param` memeriksa parser tidak rusak.

- [ ] **Step 10: Commit**

```bash
git add Hexapod_Unlimited/Navigation.h Hexapod_Unlimited/Navigation.cpp Hexapod_Unlimited/Hexapod_Unlimited.ino test-pc/sim/sim_odo.cpp
git commit -m "feat(odometri): rem jarak dan perintah serial D"
```

---

## Verifikasi di robot

Sesudah kedua task lulus dan firmware di-flash — inilah gunanya seluruh pekerjaan ini:

1. `b` — robot berdiri.
2. `Ds1.0` — pastikan skala netral.
3. Tandai posisi kaki tengah di lantai.
4. `D80` lalu `w` — robot maju dan berhenti sendiri.
5. Ukur jarak sebenarnya dengan meteran.
6. `skala = jarak_meteran / 80`. Masukkan dengan `Ds<hasil>`, ulangi langkah 3-5 untuk memeriksa.
7. Ulangi seluruhnya dengan `F` (menyusuri dinding) alih-alih `w`, untuk melihat apakah slipnya berbeda.

Kalau kedua skala itu berbeda jauh, itu temuan yang menentukan spec berikutnya — bukan sesuatu yang ditambal di sini.
