@echo off
setlocal EnableExtensions EnableDelayedExpansion
title Hexapod KRSRI -- konsol serial

rem ====================================================================
rem  Konsol serial ke robot. Flash dulu lewat Arduino IDE, lalu jalankan
rem  berkas ini -- ia mencari sendiri port Teensy-nya dan menyambung di
rem  115200 baud.
rem
rem  Pakai:   jalankan.bat            (cari port sendiri)
rem           jalankan.bat COM5       (paksa port tertentu)
rem
rem  Memakai monitor bawaan arduino-cli yang sudah ikut Arduino IDE, jadi
rem  tidak ada yang perlu dipasang. Baris yang diketik dikirim apa adanya
rem  saat Enter; Enter kosong = REM DARURAT di firmware.
rem
rem  CATATAN: Arduino IDE dan berkas ini tidak bisa memegang port yang sama
rem  pada saat bersamaan. TUTUP Serial Monitor Arduino IDE sebelum menjalankan
rem  ini, dan tutup jendela ini sebelum meng-upload lagi.
rem ====================================================================

rem --- 1. Cari arduino-cli bawaan Arduino IDE -------------------------
set "CLI="
set "C1=%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "C2=%ProgramFiles%\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "C3=%ProgramFiles(x86)%\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
if exist "%C1%" set "CLI=%C1%"
if not defined CLI if exist "%C2%" set "CLI=%C2%"
if not defined CLI if exist "%C3%" set "CLI=%C3%"
if not defined CLI for %%X in (arduino-cli.exe) do if not defined CLI set "CLI=%%~$PATH:X"

if not defined CLI (
    echo.
    echo   TIDAK MENEMUKAN arduino-cli.
    echo   Ia ikut terpasang bersama Arduino IDE. Yang dicari:
    echo     %C1%
    echo   Kalau Arduino IDE ada di tempat lain, sunting CLI di berkas ini.
    echo.
    pause
    exit /b 1
)

rem --- 2. Tentukan port ------------------------------------------------
set "PORT=%~1"

rem Keluarannya lewat berkas sementara, bukan pipa: jalur arduino-cli
rem mengandung spasi, dan tanda kutip di dalam for /f membuat cmd memotongnya
rem di spasi pertama ("...\Programs\Arduino is not recognized").
if not defined PORT (
    echo Mencari port robot...
    "%CLI%" board list > "%TEMP%\hexa_port.txt" 2>nul
    for /f "tokens=1" %%P in ('findstr /r /c:"^COM[0-9]" "%TEMP%\hexa_port.txt"') do (
        if not defined PORT set "PORT=%%P"
    )
    del "%TEMP%\hexa_port.txt" >nul 2>&1
)

rem Teensy tidak selalu muncul di 'board list' -- ia punya penemu sendiri.
rem Kalau begitu, ambil saja port serial pertama yang ada di Windows.
if not defined PORT (
    for /f "usebackq tokens=*" %%P in (`powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object | Select-Object -First 1" 2^>nul`) do (
        if not defined PORT set "PORT=%%P"
    )
)

if not defined PORT (
    echo.
    echo   TIDAK ADA PORT SERIAL yang terbaca.
    echo   Periksa: kabel USB tersambung, robot menyala, dan Serial Monitor
    echo   Arduino IDE sudah DITUTUP ^(ia mengunci port^).
    echo   Kalau tahu nomornya, sebutkan langsung:   jalankan.bat COM5
    echo.
    pause
    exit /b 1
)

rem --- 3. Pengingat perintah pertama -----------------------------------
echo.
echo  ================================================================
echo   HEXAPOD KRSRI -- %PORT% @ 115200
echo  ================================================================
echo   Robot menyala dalam keadaan LEMAS. Topang dulu, baru ketik 'b'.
echo.
echo     h            daftar seluruh perintah
echo     b            berdiri ^(servo hidup^)
echo     l            tabel LiDAR -- keenam channel harus memberi angka
echo     m4           tabel lintasan misi
echo     m1           mulai misi
echo.
echo     Enter kosong REM DARURAT      s  stop      x  LEMAS
echo.
echo   Ctrl+C untuk menutup konsol ini.
echo  ================================================================
echo.

rem --- 4. Sambung ------------------------------------------------------
"%CLI%" monitor -p %PORT% -c baudrate=115200

if errorlevel 1 (
    echo.
    echo   arduino-cli menolak port %PORT%. Lihat daftar port yang benar:
    echo       "%CLI%" board list
)

echo.
echo   Sambungan tertutup. Ini normal kalau robot di-reset atau di-upload
echo   ulang -- Teensy memutus USB-nya sendiri saat itu.
echo   Jalankan lagi berkas ini untuk menyambung kembali.
echo.
pause
