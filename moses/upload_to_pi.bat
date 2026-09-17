@echo off
REM ===================================================================
REM  upload_to_pi.bat -- unggah HUD ke Raspberry Pi dari CMD Windows.
REM
REM    upload_to_pi.bat            kirim berkas HUD
REM    upload_to_pi.bat --all      + detect.py
REM
REM  Host/folder lain:  set PI=bima@raspi5  lalu jalankan lagi.
REM  Butuh OpenSSH bawaan Windows 10/11 (scp & ssh sudah ada di PATH).
REM ===================================================================
setlocal
cd /d "%~dp0"

if "%PI%"=="" set "PI=bima@terra-core"
if "%DIR%"=="" set "DIR=M"

REM sidik_imu.py & flash_teensy.py IKUT DI SINI, bukan dikirim terpisah.
REM 11 Sep 2026: test_mission_hud.py memuat sidik_imu.py untuk mengadu
REM parser WIT-nya dengan firmware. Berkasnya tidak ada di daftar ini,
REM jadi di Pi tesnya mati dengan FileNotFoundError -- dan 600 tes lain
REM yang tidak ada hubungannya ikut tidak pernah jalan. Tesnya sudah
REM dibuat melewati berkas yang hilang; ini sisi satunya, supaya tidak
REM ada yang perlu dilewati sejak awal.
set "BERKAS=mission_hud.py test_mission_hud.py cek_serial.py sidik_firmware.py sidik_imu.py flash_teensy.py run_hud.sh siapkan_teensy_pi.sh install_service.sh hemat_daya.sh simpan_arsip.sh cek_salinan.sh"
if /i "%~1"=="--all" set "BERKAS=%BERKAS% detect.py"

echo.
echo [upload] tujuan : %PI%:~/%DIR%/
echo [upload] berkas : %BERKAS%
echo.

REM HENTIKAN layanan DULU, baru menyalin.
REM
REM Ini bukan kehati-hatian berlebihan. Menyalin ke atas berkas yang sedang
REM dipegang proses berjalan pernah menghasilkan mission_hud.py yang terpotong
REM di tengah komentar lalu diisi NULL sampai ujung -- Python menolaknya dengan
REM "source code cannot contain null bytes", layanan gagal start, systemd
REM mengulanginya 122 kali, dan dari luar gejalanya cuma "site cannot be
REM reached" tanpa petunjuk apa pun.
echo [upload] menghentikan layanan dulu (supaya berkasnya tidak dipegang)...
ssh %PI% "sudo systemctl stop r2c-hud 2>/dev/null; true"

echo [upload] menyalin...
scp %BERKAS% %PI%:~/%DIR%/
if errorlevel 1 goto :gagal_scp

echo.
echo [upload] MEMERIKSA hasil salinan di Pi...
REM Verifikasi, bukan asumsi: scp bisa "berhasil" tapi menulis berkas rusak.
REM Kalau ada null byte atau gagal dikompilasi, BERHENTI di sini -- jangan
REM menyalakan layanan yang sudah pasti gagal.
ssh %PI% "cd ~/%DIR% && chmod +x run_hud.sh install_service.sh hemat_daya.sh simpan_arsip.sh && bash cek_salinan.sh"
if errorlevel 1 goto :gagal_verifikasi

echo.
echo [upload] menjalankan uji di Pi...
ssh %PI% "cd ~/%DIR% && .venv/bin/python test_mission_hud.py | tail -3"
if errorlevel 1 goto :gagal_ssh

echo.
echo [upload] menyalakan layanan lagi...
ssh %PI% "sudo systemctl start r2c-hud 2>/dev/null; sleep 3; systemctl is-active r2c-hud"

echo.
echo ============================================================
echo  SELESAI.
echo.
echo  Jalankan HUD sekarang (manual):
echo     ssh %PI% -t "cd ~/%DIR% ^&^& ./run_hud.sh"
echo.
echo  Atau pasang sekali supaya nyala sendiri tiap Raspi boot:
echo     ssh %PI% -t "cd ~/%DIR% ^&^& ./install_service.sh"
echo     lalu reboot Raspi sekali.
echo.
echo  Sesudah itu cukup buka di browser:
echo     http://terra-core:5000/
echo ============================================================
goto :selesai

:gagal_verifikasi
echo.
echo ============================================================
echo  [!] BERKAS SAMPAI TAPI RUSAK di Pi.
echo.
echo  Ini yang terjadi kalau salinan terputus di tengah jalan: sisanya
echo  terisi NULL, dan Python menolaknya dengan
echo      SyntaxError: source code cannot contain null bytes
echo.
echo  Layanan SENGAJA tidak dinyalakan lagi -- kalau dinyalakan, systemd
echo  mengulanginya ratusan kali dan gejalanya cuma "site cannot be
echo  reached", tanpa petunjuk sama sekali.
echo.
echo  Ulangi upload_to_pi.bat sekali lagi. Kalau tetap rusak, jaringannya
echo  yang bermasalah -- coba lewat kabel, bukan Tailscale.
echo ============================================================
goto :selesai

:gagal_scp
echo.
echo [upload] GAGAL menyalin berkas.
echo          - pastikan kamu menjalankan .bat ini DARI folder moses
echo          - uji sambungan  : ssh %PI% echo ok
echo          - Tailscale nyala di PC dan di Raspi?
goto :selesai

:gagal_ssh
echo.
echo [upload] Berkas TERKIRIM, tapi perintah di Pi gagal.
echo          Masuk manual lalu jalankan:
echo             ssh %PI%
echo             cd ~/%DIR% ^&^& chmod +x run_hud.sh install_service.sh hemat_daya.sh simpan_arsip.sh

:selesai
echo.
pause
endlocal
