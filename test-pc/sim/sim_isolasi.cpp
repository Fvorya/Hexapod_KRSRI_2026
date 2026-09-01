// Uji isolasi harus memisahkan DUA sebab yang di 'j' terlihat identik:
// crosstalk antar-sensor vs pantulan yang melekat pada sensor itu sendiri.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "LidarArray.h"
static LidarArray lidar;
static void jalan(int detik){ for (int i=0;i<detik*1000/2;i++){ __nowMs+=2; lidar.update(); } }

int main(){
  Calib::load(); __serialDiam = true;
  for (int i=0;i<8;i++){ __simStatus[i]=VL53L1X::RangeValid; __simMm[i]=3000; }
  lidar.begin(); __serialDiam = false;

  printf("\n########## KASUS 1: crosstalk ANTAR-SENSOR ##########\n");
  printf("(hantu 40 mm hanya muncul selama ada tetangga yang memancar)\n");
  __simCrosstalkAntar = true;
  for (int i=0;i<8;i++){ __simMm[i]=3000; __simStatus[i]=VL53L1X::SignalFail; __simHantuMm[i]=40; }
  lidar.isolasiMulai((int8_t)LIDAR_FRONT, 3); jalan(8);

  printf("\n########## KASUS 2: pantulan MELEKAT pada sensor ##########\n");
  printf("(hantu 40 mm ada terus, tak peduli tetangga memancar atau tidak)\n");
  __simCrosstalkAntar = false;
  for (int i=0;i<8;i++){ __simMm[i]=40; __simStatus[i]=VL53L1X::RangeValid; }
  lidar.isolasiMulai((int8_t)LIDAR_FRONT, 3); jalan(8);

  printf("\n########## KASUS 3: sapuan OTOMATIS seluruh sensor ##########\n");
  printf("(ch0 bersih & melihat dinding 75 cm; ch1..ch5 kena crosstalk antar-sensor)\n");
  __simCrosstalkAntar = true;
  for (int i=0;i<8;i++){ __simMm[i]=3000; __simStatus[i]=VL53L1X::SignalFail; __simHantuMm[i]=(uint16_t)(25+i*4); }
  __simMm[0]=750; __simStatus[0]=VL53L1X::RangeValid; __simHantuMm[0]=750;  // ch0 target nyata
  lidar.isolasiMulai(-1, 2); jalan(30);
  return 0;
}
