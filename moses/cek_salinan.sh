#!/usr/bin/env bash
# Periksa berkas Python yang baru disalin ke Pi BENAR-BENAR utuh.
#
# Dipanggil upload_to_pi.bat sesudah scp, sebelum layanan dinyalakan lagi.
# Ada sebabnya: scp pernah "berhasil" tapi menulis mission_hud.py yang
# terpotong di tengah komentar lalu diisi NULL sampai ujung. Python menolaknya
#   SyntaxError: source code cannot contain null bytes
# layanan gagal start, systemd mengulanginya 122 kali, dan dari browser
# gejalanya cuma "This site cannot be reached" -- tanpa satu pun petunjuk.
#
# Dua pemeriksaan, karena satu saja tidak cukup:
#   1. null byte   -- menangkap salinan terpotong
#   2. py_compile  -- menangkap berkas utuh tapi rusak isinya
set -u
PY=.venv/bin/python
[ -x "$PY" ] || PY=python3
rc=0
for f in mission_hud.py test_mission_hud.py cek_serial.py sidik_firmware.py; do
  [ -f "$f" ] || { echo "  [HILANG] $f"; rc=9; continue; }
  n=$(tr -dc '\0' < "$f" | wc -c)
  if [ "$n" != "0" ]; then
    echo "  [RUSAK]  $f berisi $n null byte -- salinan terpotong"
    rc=9
    continue
  fi
  if ! "$PY" -m py_compile "$f" 2>/dev/null; then
    echo "  [RUSAK]  $f gagal dikompilasi"
    "$PY" -m py_compile "$f" 2>&1 | tail -3 | sed 's/^/           /'
    rc=9
    continue
  fi
  echo "  OK       $f  $(stat -c%s "$f") byte"
done
exit $rc
