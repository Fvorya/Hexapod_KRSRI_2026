#!/usr/bin/env bash
# Bangun & jalankan seluruh simulasi. Perlu g++ (WSL, MinGW, atau Linux).
#   ./build.sh          -> bangun semua, jalankan semua
#   ./build.sh sim_wall -> satu saja
set -e
HX="$(cd "$(dirname "$0")/../Hexapod_Unlimited" && pwd)"
ST="$(cd "$(dirname "$0")/stub" && pwd)"
SM="$(cd "$(dirname "$0")/sim" && pwd)"
OUT="$(dirname "$0")/bin"; mkdir -p "$OUT"

SRC="$ST/stubdefs.cpp $HX/Navigation.cpp $HX/Hexapod.cpp $HX/HexaGait.cpp \
     $HX/HexaServos.cpp $HX/HexaArm.cpp $HX/Imu.cpp $HX/LidarArray.cpp \
     $HX/Calib.cpp $HX/ArmInverse.cpp $HX/LegInverseKinematics.cpp \
     $HX/Mission.cpp"
FLAGS="-std=gnu++17 -O1 -I$ST -I$HX"

echo "=== 1. Pemeriksaan sintaks seluruh sketsa ==="
cp "$HX/Hexapod_Unlimited.ino" "$OUT/_sketch.cpp"
g++ $FLAGS -fsyntax-only -Wall -Wextra -Wno-unused-parameter "$HX"/*.cpp "$OUT/_sketch.cpp"
rm -f "$OUT/_sketch.cpp"
echo "    OK -- tidak ada peringatan."

DAFTAR="${1:-sim_pivot sim_body sim_lidar sim_open sim_reinit sim_peta sim_hantu sim_jejak sim_isolasi sim_param sim_goyang sim_wall sim_laju sim_servolaju sim_dinding sim_yaw sim_depan sim_boot sim_misi}"
for t in $DAFTAR; do
    echo; echo "=== $t ==="
    # sim_param butuh parser serial dari .ino, jadi sketsanya ikut dikompilasi
    EKSTRA=""
    if [ "$t" = "sim_param" ] || [ "$t" = "sim_goyang" ] || [ "$t" = "sim_boot" ]; then
        cp "$HX/Hexapod_Unlimited.ino" "$OUT/_ino.cpp"; EKSTRA="$OUT/_ino.cpp"
    fi
    g++ $FLAGS -o "$OUT/$t" "$SM/$t.cpp" $EKSTRA $SRC
    "$OUT/$t"
done
