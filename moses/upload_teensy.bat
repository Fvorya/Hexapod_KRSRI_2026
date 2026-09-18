@echo off
REM ===================================================================
REM  upload_teensy.bat -- kompilasi & upload firmware hexapod ke Teensy 4.1
REM
REM    upload_teensy.bat            kompilasi lalu upload
REM    upload_teensy.bat --cek      kompilasi saja, TIDAK upload
REM    upload_teensy.bat --lib      periksa/pasang pustaka saja
REM
REM  Port lain:  set PORT=COM7  lalu jalankan lagi.
REM  Seluruh keluaran kompilasi disimpan ke build_log.txt.
REM
REM  PENTING sesudah pad VUSB-VIN dipotong:
REM    Teensy TIDAK lagi mengambil daya dari USB. Supaya bisa di-upload,
REM    daya robot (VIN) HARUS menyala. Kalau baterai mati, Teensy tidak
REM    muncul sama sekali dan upload gagal tanpa sebab yang jelas.
REM
REM    Aman dengan servo terpasang: v1.61/v1.7/v1.8 semua boot LEMAS (PWM
REM    mati) dan DEMO_BOOT sudah 0 -- tidak ada kaki yang bergerak sendiri.
REM ===================================================================
REM enabledelayedexpansion dipakai untuk menomori daftar sketch di bawah.
REM Efek sampingnya: tanda seru di dalam path akan hilang. Path firmware
REM tidak pernah punya itu, jadi aman di sini.
setlocal enabledelayedexpansion
cd /d "%~dp0"
REM SRC = folder induk yang berisi versi-versi sketch Vincent.
REM Di laptop ini: ...\Desktop\Vincent\Hexapod\  (dua tingkat di atas moses\moses).
REM Laptop lain tinggal:  set SRC=D:\jalur\ke\vincent  lalu jalankan lagi.
if "%SRC%"=="" set "SRC=%~dp0..\.."

REM VERSI BAKU -- yang dipakai kalau kamu cuma menekan Enter di menu.
REM Di laptop ini SRC menunjuk ke C:\Users\R2C\Desktop\Vincent\Hexapod, jadi
REM baris di bawah ini adalah POHON KERJA -- folder yang sedang disunting, dan
REM sumber yang sama persis yang dikirim kirim_teensy.bat ke Raspi. Kedua jalur
REM flash memakai sumber yang sama; itu memang yang dituju.
REM
REM Sampai 19 Sep 2026 baku ini menunjuk ke Hexapod_Unlimited_v1.18\ dan itu
REM SALAH sejak perapian tata letak 18 Sep: pohon kerja dipindah ke
REM Hexapod_Unlimited\, dan yang tertinggal di folder v1.18 cuma berkas .bak --
REM tidak ada Hexapod_Unlimited.ino di sana. Menekan Enter jatuh ke
REM :sketch_hilang. Folder v1.17 di sebelahnya salinan referensi OLED dan tidak
REM berisi perbaikan 16 Sep 2026; jangan di-flash tanpa sengaja.
REM
REM Menu TIDAK dihapus: versi lain tetap bisa dipilih dengan nomornya, dan
REM tanggal tiap folder tetap dicetak. Yang berubah cuma apa yang terjadi
REM saat Enter -- dulu batal, sekarang pakai yang baku. Membatalkan
REM sekarang: ketik 0.
REM
REM Versi baku lain:  set BAKU=%SRC%\<folder>\Hexapod_Unlimited
if "%BAKU%"=="" set "BAKU=%SRC%\Hexapod_Unlimited"

REM PILIH sketch dari daftar. TIDAK ditebak lagi.
REM
REM Versi sebelumnya menebak "yang paling baru" lewat 'dir /s /o-d'. Itu SALAH,
REM dan salahnya diam-diam: 'dir /s' menelusuri folder urut NAMA, sedangkan
REM '/o-d' hanya mengurutkan DI DALAM tiap folder. Karena tiap folder cuma
REM berisi satu Hexapod_Unlimited.ino, pengurutan tanggalnya tidak pernah
REM berlaku sama sekali -- keluarnya urut alfabet, dan 'v1.61' kebetulan paling
REM depan. Jadi skripnya menawarkan v1.61 sambil terlihat yakin.
REM
REM Tanggal berkas juga bukan penentu yang jujur: itu waktu folder DISALIN,
REM bukan versi firmwarenya. Jadi tanggal cuma DITAMPILKAN sebagai bahan
REM pertimbangan, dan yang memutuskan tetap kamu.
REM
REM Melewati menu ini:  set SKETCH=%SRC%\<folder>\Hexapod_Unlimited
echo.
echo === PILIH FIRMWARE ===
if not "%SKETCH%"=="" goto :sketch_manual
set "N=0"
for /f "delims=" %%f in ('dir /b /s "%SRC%\Hexapod_Unlimited.ino" 2^>nul') do (
  set /a N+=1
  set "S!N!=%%~dpf"
  echo     !N!^) %%~tf   %%~dpf
)
if "%N%"=="0" goto :sketch_hilang
echo.
echo       Enter = pakai versi baku:
echo         %BAKU%
echo.
set /p "PILIH=  Nomor sketch (1-%N%), Enter = baku, 0 = batal: "
if "%PILIH%"=="0" goto :batal
if "%PILIH%"=="" set "SKETCH=%BAKU%" & goto :sketch_baku
set "SKETCH=!S%PILIH%!"
if "%SKETCH%"=="" goto :pilihan_salah
if "%SKETCH:~-1%"=="\" set "SKETCH=%SKETCH:~0,-1%"
:sketch_baku
:sketch_manual
REM Baca CALIB_VERSION dari sketch YANG DIPILIH, bukan angka mati di skrip ini.
REM Versi sebelumnya menuliskan "CALIB_VERSION tetap 9" -- benar untuk v1.7/v1.8,
REM SALAH untuk v1.9/v1.10 yang menaikkannya ke 10. Nasihat yang basi diam-diam
REM lebih berbahaya daripada tidak ada nasihat.
set "CV=?"
for /f "tokens=3" %%v in ('findstr /r /c:"define CALIB_VERSION" "%SKETCH%\Calib.cpp" 2^>nul') do set "CV=%%v"

if "%FQBN%"=="" set "FQBN=teensy:avr:teensy41"
set "IDX=https://www.pjrc.com/teensy/package_teensy_index.json"
set "LOG=build_log.txt"

echo.
echo === FIRMWARE HEXAPOD -^> TEENSY 4.1 ===
echo   sketch : %SKETCH%
echo   board  : %FQBN%
echo   log    : %CD%\%LOG%
echo.

where arduino-cli >nul 2>&1
REM Tidak ada di PATH? Arduino IDE 2 membawa arduino-cli sendiri di dalam
REM folder pemasangannya. Pakai itu daripada menyuruh orang memasang lagi.
if errorlevel 1 (
  set "IDECLI=%ProgramFiles%\Arduino IDE\resources\app\lib\backend\resources"
  if exist "!IDECLI!\arduino-cli.exe" set "PATH=!IDECLI!;%PATH%"
)
where arduino-cli >nul 2>&1
if errorlevel 1 goto :tanpa_cli
if not exist "%SKETCH%\Hexapod_Unlimited.ino" goto :sketch_hilang

echo [1/5] indeks board PJRC...
arduino-cli config add board_manager.additional_urls %IDX% >nul 2>&1
if errorlevel 1 (
  arduino-cli config init >nul 2>&1
  arduino-cli config add board_manager.additional_urls %IDX% >nul 2>&1
)
arduino-cli core update-index >nul 2>&1

echo [2/5] core Teensy...
arduino-cli core install teensy:avr >nul 2>&1

echo [3/5] pustaka VL53L1X...
REM WAJIB. LidarArray.cpp ada DI DALAM folder sketch, jadi tanpa pustaka ini
REM SELURUH sketch gagal dikompilasi -- bukan cuma bagian LiDAR-nya.
REM Yang dibutuhkan punya POLOLU. Ada beberapa pustaka bernama mirip
REM (SparkFun, STM32duino) dan itu TIDAK cocok -- API-nya berbeda.
arduino-cli lib install "VL53L1X" >nul 2>&1
REM Adafruit PWM Servo Driver -- dipakai HexaServos.cpp. Dulu TIDAK ikut
REM dipasang skrip ini; kebetulan lolos karena sudah ada di komputer ini dari
REM pemasangan manual. Di komputer lain kompilasi akan gagal "No such file".
arduino-cli lib install "Adafruit PWM Servo Driver Library" >nul 2>&1
REM OLED skor (Tampilan.cpp). SSD1306 menarik Adafruit BusIO sendiri;
REM GFX tidak, jadi keduanya dipasang di sini.
arduino-cli lib install "Adafruit GFX Library" >nul 2>&1
arduino-cli lib install "Adafruit SSD1306" >nul 2>&1
echo       terpasang sekarang:
arduino-cli lib list 2>nul | findstr /i "vl53 adafruit_pwm adafruit pwm"

echo.
echo       pustaka pihak ketiga yang dibutuhkan sketch ini:
REM Kalau Vincent menambah pustaka baru, ia muncul di sini SEBELUM kompilasi
REM gagal dengan pesan yang membingungkan.
findstr /r /c:"#include <" "%SKETCH%\*.h" "%SKETCH%\*.cpp" "%SKETCH%\*.ino" 2>nul | findstr /v /i "Arduino.h EEPROM.h Wire.h math.h stdint.h stddef.h string.h stdlib.h stdio.h" | findstr /r /c:"#include <" >"%TEMP%\inc.txt"
for /f "tokens=2 delims=<>" %%i in ('type "%TEMP%\inc.txt"') do echo          %%i
del "%TEMP%\inc.txt" >nul 2>&1
if errorlevel 1 (
  echo       [!] TIDAK ADA pustaka VL53 yang terpasang.
  echo           Cari nama persisnya dengan:
  echo             arduino-cli lib search vl53l1x
  echo           lalu pasang yang AUTHOR-nya Pololu.
)
if /i "%~1"=="--lib" goto :selesai

echo.
echo [4/5] kompilasi... (keluaran lengkap -^> %LOG%)
arduino-cli compile --fqbn %FQBN% "%SKETCH%" > "%LOG%" 2>&1
set "RC=%ERRORLEVEL%"
type "%LOG%"
if not "%RC%"=="0" goto :gagal_kompilasi
echo.
echo       KOMPILASI BERSIH.

if /i "%~1"=="--cek" goto :cek_saja

echo.
echo [5/5] mencari papan Teensy...
REM arduino-cli TIDAK menebak port sendiri untuk Teensy -- kalau tidak diberi
REM -p dia langsung menyerah dengan "no upload port provided". Jadi kita cari
REM sendiri dari daftar papannya.
if not "%PORT%"=="" goto :punya_port
for /f "tokens=1" %%p in ('arduino-cli board list 2^>nul ^| findstr /i "teensy"') do set "PORT=%%p"
if "%PORT%"=="" goto :tanpa_port

:punya_port
echo       ketemu di %PORT%
echo.
echo       AKAN DI-FLASH: %SKETCH%
echo       CALIB_VERSION : %CV%
echo       Pastikan itu versi yang kamu maksud -- salah versi baru ketahuan
echo       di arena, dan di situ sudah terlambat.
set /p "OK=      Lanjut upload? [y/N] "
if /i not "%OK%"=="y" goto :batal
echo.
echo       upload...
arduino-cli upload --fqbn %FQBN% -p %PORT% "%SKETCH%"
if errorlevel 1 goto :gagal_upload

echo.
echo ============================================================
echo  SELESAI. Robot boot LEMAS -- tidak ada yang bergerak sampai
echo  kamu mengetik 'b'.
echo.
echo  CALIB_VERSION sketch ini : %CV%
echo.
echo  Kalau angka itu naik, blob di EEPROM 0 dibuang dan 25 parameter kembali
echo  ke default yang ada DI DALAM kode. Itu memang DISENGAJA: Vincent
echo  menaikkan angkanya justru supaya default baru yang lebih benar berlaku,
echo  menggantikan nilai lama yang tersimpan.
echo.
echo  Yang TIDAK terbuang -- masing-masing blob sendiri, versinya sendiri:
echo    EEPROM 1024  ServoMap : invert ^& trim servo. MENANG atas Calib,
echo                            karena itu data yang benar-benar diukur.
echo    EEPROM 1792  kompas arena     -- 'k' lihat, 'E' muat ulang
echo    EEPROM 2048  kalibrasi pivot  -- 'K' lihat
echo.
echo  Jadi arah servo, kompas dan pivot AMAN. Yang perlu dimasukkan ulang cuma
echo  parameter yang PERNAH kamu ubah sendiri dengan 'Q' lalu 'W'. Kalau tidak
echo  pernah, tidak ada yang perlu dilakukan.
echo.
echo  Periksa sesudah flash:
echo    q            25 parameter
echo    k  dan  K    kompas arena dan kalibrasi pivot
echo    l            VERIFIKASI arah ch0 dan ch2
echo ============================================================
goto :selesai

:cek_saja
echo.
echo  Mode --cek: kompilasi lulus, TIDAK ada yang di-upload.
goto :selesai

:pilihan_salah
echo.
echo  [!] Nomor "%PILIH%" tidak ada di daftar. Tidak ada yang di-flash.
goto :selesai

:batal
echo.
echo  Dibatalkan. Tidak ada yang di-flash.
echo  Enter (tanpa mengetik apa pun) memakai versi baku:
echo      %BAKU%
echo  Untuk memilih versi lain:
echo      set SKETCH=%SRC%\^<folder^>\Hexapod_Unlimited
echo      upload_teensy.bat
goto :selesai

:gagal_kompilasi
echo.
echo ============================================================
echo  [!] KOMPILASI GAGAL -- tidak ada yang di-upload.
echo      Keluaran lengkap ada di: %CD%\%LOG%
echo.
echo  Baris error yang penting:
echo ------------------------------------------------------------
findstr /i /c:"fatal error" /c:"No such file" /c:"error:" /c:"undefined reference" "%LOG%"
echo ------------------------------------------------------------
echo.
echo  Cara membacanya:
echo    "VL53L1X.h: No such file"  -^> pustaka Pololu belum terpasang.
echo                                  arduino-cli lib search vl53l1x
echo    error di dalam VL53L1X     -^> terpasang pustaka yang SALAH
echo                                  (SparkFun/STM32duino). Copot, pasang Pololu.
echo    error di file .cpp sketch  -^> jarang: kode ini sudah diuji kompilasi
echo                                  bersih dengan g++ -Wall -Wextra.
echo    "platform not installed"   -^> arduino-cli core install teensy:avr
echo ============================================================
goto :selesai

:tanpa_cli
echo [!] arduino-cli tidak ada di PATH.
echo.
echo     Pilihan 1 -- pasang arduino-cli (sekali saja):
echo       https://arduino.github.io/arduino-cli/latest/installation/
echo.
echo     Pilihan 2 -- Arduino IDE + Teensyduino:
echo       buka SELURUH FOLDER sketch:
echo         %SKETCH%
echo       Tools -^> Board -^> Teensy 4.1, lalu Upload.
echo       Pasang "VL53L1X by Pololu" lewat Library Manager.
goto :selesai

:sketch_hilang
echo [!] Sketch tidak ketemu di:
echo       %SKETCH%
echo     Jalankan .bat ini dari folder moses, atau set SKETCH=... sendiri.
echo     Dicari otomatis di: %SRC%\**\Hexapod_Unlimited.ino
goto :selesai

:tanpa_port
echo.
echo ============================================================
echo  [!] Papan Teensy TIDAK TERLIHAT dari komputer ini.
echo.
echo  Port yang terdeteksi sekarang:
echo ------------------------------------------------------------
arduino-cli board list
echo ------------------------------------------------------------
echo.
echo  Tiga sebab, urut dari yang paling sering:
echo.
echo   1. Kabel USB Teensy masih tercolok ke RASPBERRY PI, bukan ke
echo      laptop ini. Cabut dari Pi, colok ke laptop.
echo.
echo   2. Daya robot (VIN) mati. Sesudah pad VUSB-VIN dikikir, Teensy
echo      TIDAK lagi hidup dari USB -- colok USB saja tidak menyalakan
echo      papannya, jadi tidak ada port yang muncul. Nyalakan baterai.
echo      (Servo tetap aman: v1.61/v1.7/v1.8 semua boot LEMAS.)
echo.
echo   3. Teensy sedang dipegang program lain, atau perlu dipaksa masuk
echo      mode program: tekan tombol PROGRAM di papan lalu ulangi.
echo.
echo  Kalau kamu sudah tahu portnya, sebut saja langsung:
echo      set PORT=COM5
echo      upload_teensy.bat
echo ============================================================
goto :selesai

:gagal_upload
echo.
echo [!] UPLOAD GAGAL. Urutan periksa:
echo     1. Daya robot (VIN) menyala? Sesudah pad VUSB-VIN dipotong,
echo        Teensy TIDAK hidup dari USB saja.
echo     2. Teensy terlihat?  arduino-cli board list
echo     3. Port dipegang program lain? Tutup Serial Monitor / HUD.
echo        Di Pi:  sudo systemctl stop r2c-hud
echo     4. Masih bandel? tekan tombol PROGRAM di papan Teensy lalu ulangi.

:selesai
echo.
pause
endlocal
