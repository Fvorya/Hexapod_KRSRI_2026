// Membuktikan 'j' memisahkan hantu MENETAP dari hantu BERUBAH-UBAH.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "LidarArray.h"
static LidarArray lidar;
static uint32_t rs = 7;
static int acak(int lo, int hi){ rs = rs*1103515245u+12345u; return lo + (int)((rs>>16)%(hi-lo+1)); }

static void jalan(int detik, void(*adegan)()) {
    for (int i = 0; i < detik*1000/2; i++) { __nowMs += 2; if ((i%12)==0) adegan(); lidar.update(); }
}
static void adeganMenetap() {   // crosstalk: 50 mm nyaris tanpa sebaran
    for (int c=0;c<8;c++){ __simMm[c]=(uint16_t)acak(49,51); __simStatus[c]=VL53L1X::RangeValid; }
}
static void adeganKosong() {    // ruang benar-benar kosong
    for (int c=0;c<8;c++){ __simMm[c]=3000; __simStatus[c]=VL53L1X::SignalFail; }
}
static void adeganBerubah() {   // pantulan tak menentu
    for (int c=0;c<8;c++){ __simMm[c]=(uint16_t)acak(40,400); __simStatus[c]=VL53L1X::RangeValid; }
}

int main(){
  Calib::load(); __serialDiam = true;
  for (int i=0;i<8;i++) __simStatus[i]=VL53L1X::RangeValid;
  lidar.begin(); __serialDiam = false;

  printf("\n########## A. Hantu MENETAP di 5 cm (crosstalk / benda terpasang) ##########\n");
  lidar.jejakMulai(LIDAR_FRONT, 3); jalan(4, adeganMenetap);

  printf("\n########## B. Ruang benar-benar kosong ##########\n");
  lidar.jejakMulai(LIDAR_FRONT, 3); jalan(4, adeganKosong);

  printf("\n########## C. Pantulan BERUBAH-UBAH ##########\n");
  lidar.jejakMulai(LIDAR_FRONT, 3); jalan(4, adeganBerubah);
  return 0;
}
