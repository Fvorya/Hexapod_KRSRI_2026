@echo off
setlocal EnableExtensions
title R2C -- siapkan kirim tanpa password

REM ====================================================================
REM  siapkan_kunci.bat -- sekali jalan, sesudah itu scp/ssh ke Pi tidak
REM  pernah meminta password lagi.
REM
REM    siapkan_kunci.bat                 host bawaan bima@terra-core
REM    siapkan_kunci.bat bima@raspi5     host lain
REM
REM  KENAPA KUNCI, BUKAN PASSWORD DI DALAM SKRIP.
REM  Password yang ditulis di berkas ikut masuk git, ikut ter-backup, dan
REM  ikut terbaca siapa pun yang membuka folder ini. Ia juga tidak membuat
REM  scp berhenti bertanya -- OpenSSH menolak membaca password dari pipa.
REM  Kunci publik menyelesaikan keduanya sekaligus.
REM
REM  Anda mengetik password Pi PERSIS SEKALI, di langkah 2, langsung ke
REM  prompt OpenSSH. Berkas ini tidak pernah melihatnya dan tidak
REM  menyimpannya di mana pun.
REM ====================================================================

set "PI=%~1"
if "%PI%"=="" set "PI=bima@terra-core"

set "KUNCI=%USERPROFILE%\.ssh\id_ed25519"

echo.
echo  ================================================================
echo   Menyiapkan kirim tanpa password ke %PI%
echo  ================================================================
echo.

REM --- 1. Kunci ------------------------------------------------------
if exist "%KUNCI%" (
    echo [1/4] Kunci sudah ada: %KUNCI%
) else (
    echo [1/4] Membuat kunci baru ed25519...
    echo       Passphrase DIKOSONGKAN dengan sengaja: skrip kirim berjalan
    echo       tanpa penunggu, dan passphrase membuatnya bertanya lagi.
    ssh-keygen -t ed25519 -N "" -C "r2c-laptop" -f "%KUNCI%"
    if errorlevel 1 (
        echo   GAGAL membuat kunci. OpenSSH terpasang? Coba: ssh-keygen -V
        pause
        exit /b 1
    )
)

REM --- 2. Kirim kunci publik ke Pi -----------------------------------
REM Windows tidak punya ssh-copy-id, jadi dituang lewat pipa. Ini
REM SATU-SATUNYA langkah yang meminta password Pi Anda.
echo.
echo [2/4] Menyalin kunci publik ke %PI%
echo       KETIK PASSWORD PI ANDA saat diminta. Sekali ini saja.
echo.
type "%KUNCI%.pub" | ssh %PI% "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && sort -u -o ~/.ssh/authorized_keys ~/.ssh/authorized_keys"
if errorlevel 1 (
    echo.
    echo   GAGAL menyalin kunci.
    echo   Periksa: ssh %PI% echo ok
    echo            nama host benar, Pi menyala, satu jaringan.
    pause
    exit /b 1
)

REM --- 3. Bukti, bukan asumsi ----------------------------------------
REM BatchMode=yes membuat OpenSSH MENOLAK bertanya password. Jadi kalau
REM baris ini lolos, ia lolos memakai kunci -- bukan karena Anda kebetulan
REM mengetik password sekali lagi tanpa sadar.
echo.
echo [3/4] Menguji sambungan tanpa password...
ssh -o BatchMode=yes -o ConnectTimeout=8 %PI% "echo ok"
if errorlevel 1 (
    echo.
    echo   Kunci tersalin tapi login tanpa password BELUM jalan.
    echo   Tersangka yang paling sering, urut:
    echo     - izin folder di Pi   : chmod 700 ~/.ssh ^&^& chmod 600 ~/.ssh/authorized_keys
    echo     - home Pi bisa ditulis grup lain (sshd menolak) : chmod 750 ~
    echo     - PubkeyAuthentication no di /etc/ssh/sshd_config
    pause
    exit /b 1
)

REM --- 4. sudo tanpa password, HANYA untuk layanan HUD ----------------
REM upload_to_pi.bat menghentikan dan menjalankan layanan r2c-hud lewat
REM sudo. Kunci SSH tidak menyentuh sudo sama sekali -- keduanya sistem
REM yang berbeda -- jadi tanpa langkah ini scp lancar tapi skripnya tetap
REM berhenti menunggu password sudo.
echo.
echo [4/4] Memeriksa sudo tanpa password untuk layanan r2c-hud...
ssh -o BatchMode=yes %PI% "sudo -n systemctl is-active r2c-hud >nul 2>&1 || exit 7"
if errorlevel 7 (
    echo.
    echo   sudo MASIH meminta password. Jalankan satu perintah ini di Pi,
    echo   ketik password Pi sekali lagi, lalu selesai:
    echo.
    echo     echo "$USER ALL=(ALL) NOPASSWD: /bin/systemctl start r2c-hud, /bin/systemctl stop r2c-hud, /bin/systemctl restart r2c-hud, /bin/systemctl is-active r2c-hud" ^| sudo tee /etc/sudoers.d/r2c-hud
    echo     sudo chmod 440 /etc/sudoers.d/r2c-hud
    echo.
    echo   SENGAJA DIBATASI ke empat perintah itu. NOPASSWD untuk SELURUH
    echo   sudo berarti siapa pun yang memegang laptop ini memegang root di
    echo   Pi, dan yang dibutuhkan cuma menyalakan layanan.
) else (
    echo       sudo layanan sudah bebas password.
)

echo.
echo  ================================================================
echo   SELESAI. Sekarang jalankan:
echo       upload_to_pi.bat
echo   dan ia tidak akan bertanya apa pun.
echo  ================================================================
echo.
pause
