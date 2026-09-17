@echo off
rem Serial monitor ke Teensy, 115200 baud. Dipakai MISI.md bagian 4.
rem
rem   jalankan.bat          cari sendiri port Teensy-nya
rem   jalankan.bat COM5     kalau nomornya sudah tahu
rem
rem Port dicari lewat hwgrep pyserial, bukan ditebak: 16C0:0483 adalah
rem VID:PID USB serial Teensy. Satu baris, tanpa kode deteksi sendiri.
rem
rem Enter mengirim LF saja. Firmware menerima CR/LF/CRLF sebagai satu akhir
rem baris, tapi LF yang paling bersih -- sebelum firmware diperbaiki, terminal
rem ber-CRLF membuat tiap perintah langsung disusul rem darurat.
rem
rem KELUAR: Ctrl+]   (bukan Ctrl+C -- itu dikirim ke robot)
rem
rem Arduino IDE dan skrip ini TIDAK BISA memegang port yang sama. Tutup
rem Serial Monitor IDE dulu, dan tutup jendela ini sebelum upload berikutnya.
setlocal
set PORT=%1
if "%PORT%"=="" set PORT=hwgrep://16C0:0483
python -m serial.tools.miniterm --raw --eol LF %PORT% 115200
