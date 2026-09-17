@echo off
setlocal enabledelayedexpansion
REM ====================================================================
REM kirim_teensy.bat -- kirim SATU folder sketch dari laptop ke Pi.
REM Yang MEM-FLASH itu Pi, bukan laptop ini. Lihat flash_teensy.py di sana.
REM
REM KENAPA TIDAK ADA MENU LAGI:
REM   Versi sebelumnya mendaftar setiap folder di bawah Hexapod\ dan menyuruh
REM   memilih nomor. Itu berguna waktu ada banyak versi yang hidup berdampingan.
REM   Sekarang yang disunting cuma satu, dan menu yang isinya satu baris
REM   hanyalah satu kesempatan lagi untuk salah tekan.
REM
REM   Folder lain: set SUMBER=D:\jalur\ke\sketch  lalu jalankan lagi.
REM   Nama di Pi : set NAMA=v1.18                 lalu jalankan lagi.
REM
REM KENAPA -p PENTING DAN BUKAN HIASAN:
REM   scp tanpa -p menulis ulang SEMUA stempel waktu dengan waktu sekarang.
REM   Sesudah dua kali kirim, seluruh versi di Pi jadi seumur, dan
REM   flash_teensy.py -- yang mengurutkan menurut waktu justru karena nomor
REM   versi v1.61 berbohong -- kehilangan satu-satunya petunjuk yang benar.
REM   Jadi -p bukan kerapian, ia yang menjaga urutannya tetap berarti.
REM ====================================================================
cd /d "%~dp0"

set PI=bima@terra-core
set TUJUAN=~/teensy

REM AKAR versi firmware. Relatif terhadap letak .bat ini, jadi ikut ke mana
REM pun folder moses disalin: moses\moses -> ..\.. = folder Hexapod, yaitu
REM C:\Users\R2C\Desktop\Vincent\Hexapod di laptop ini. Tempat yang sama
REM yang dipakai SRC di upload_teensy.bat.
if "%AKAR%"=="" set "AKAR=%~dp0..\.."

REM Sketch BAKU di bawah akar itu. Versi lain: set SUMBER=<jalur> lalu ulangi.
if "%SUMBER%"=="" set "SUMBER=%AKAR%\Hexapod_Unlimited_v1.18\Hexapod_Unlimited"
if "%NAMA%"==""   set "NAMA=Hexapod_Unlimited_v1.18"

echo == kirim_teensy ==
echo   dari : %SUMBER%
echo   ke   : %PI%:%TUJUAN%/%NAMA%/
echo.

if not exist "%SUMBER%\Hexapod_Unlimited.ino" (
  echo !! Bukan folder sketch: %SUMBER%
  echo    Hexapod_Unlimited.ino tidak ada di sana.
  goto :selesai
)

REM HANYA SUMBER SKETCH. Bukan 'scp -r' seluruh folder: di sana ada belasan
REM berkas .bak dari penyuntingan, README, MISI.md dan satu PNG. arduino-cli
REM memang mengabaikannya, tapi semuanya ikut menyeberang tiap kali dikirim.
REM
REM Berkas .bak TIDAK dikompilasi arduino-cli -- ekstensinya bukan .cpp, jadi
REM ia lewat begitu saja. Alasan membuangnya bukan tabrakan kompilasi,
REM melainkan bahwa ~/teensy jadi berisi salinan setengah jadi yang suatu saat
REM dinamai ulang seseorang, dan sejak itu ia BENAR-BENAR ikut dikompilasi.
echo [1/2] menyiapkan folder di Pi...
ssh %PI% "mkdir -p %TUJUAN%/%NAMA%"
if errorlevel 1 goto :gagal_ssh

echo [2/2] menyalin .ino .cpp .h ...
REM scp tidak memekarkan wildcard sendiri, dan cmd.exe tidak memekarkannya
REM untuk argumen program luar. Yang bisa: 'for %%f in (...)'. Daftarnya
REM dibangun di sini lalu diserahkan ke scp sekali jalan -- satu sambungan,
REM bukan satu per berkas.
set "DAFTAR="
for %%f in ("%SUMBER%\*.ino" "%SUMBER%\*.cpp" "%SUMBER%\*.h") do set "DAFTAR=!DAFTAR! "%%f""
if "!DAFTAR!"=="" (
  echo !! Tidak ada .ino/.cpp/.h di %SUMBER%
  goto :selesai
)
scp -p !DAFTAR! %PI%:%TUJUAN%/%NAMA%/
if errorlevel 1 goto :gagal_scp

echo.
echo Terkirim. Sekarang PERIKSA DULU di Pi, jangan langsung flash:
echo.
echo    ssh %PI% "python3 ~/M/flash_teensy.py --coba --terbaru"
echo.
echo Kalau daftarnya benar dan pustakanya lengkap, baru:
echo.
echo    ssh %PI% -t "python3 ~/M/flash_teensy.py --terbaru"
echo.
echo Pakai -t supaya kalau Teensy minta tombol PROGRAM ditekan, pesannya
echo benar-benar muncul di layarmu, bukan tertahan di buffer.
goto :selesai

:gagal_ssh
echo.
echo !! SSH GAGAL. Tidak ada yang dikirim.
echo    Uji sambungan : ssh %PI% echo ok
echo    Tailscale nyala di PC dan di Raspi?
goto :selesai

:gagal_scp
echo.
echo !! SCP GAGAL. Tidak ada yang di-flash.
echo    Folder tujuan sudah dibuat, isinya mungkin separuh -- kirim ulang
echo    sebelum flash, jangan flash yang sekarang ada di sana.

:selesai
echo.
pause
