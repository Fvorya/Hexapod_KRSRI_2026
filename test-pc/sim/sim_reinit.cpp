// Skenario persis gejala di robot: ch1,ch2 gagal init saat boot; ch0,ch4 init
// OK tapi tak pernah "data siap". Lalu 'I' ditekan -- tanpa reset papan.
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "LidarArray.h"
static LidarArray lidar;
static void putar(int ms){ for (int i=0;i<ms;i+=2){ __nowMs+=2; lidar.update(); } }

int main(){
  Calib::load(); __serialDiam = false;
  __initGagal[1] = __initGagal[2] = true;
  __bisu[0] = __bisu[4] = true;
  for (int i=0;i<6;i++) __simMm[i] = 600;
  __simMm[3] = 70; __simMm[5] = 320;

  printf("\n##### BOOT (gejala Anda) #####\n");
  lidar.begin(); putar(600); lidar.cetakTabel();

  printf("\n##### 'I' SEBELUM apa pun dibetulkan #####\n");
  lidar.pindaiI2C();

  printf("\n##### ch0,ch4 dicolok ulang; kabel ch1,ch2 dibetulkan #####");
  __initGagal[1] = __initGagal[2] = false; __bisu[0] = __bisu[4] = false;
  printf("\n##### 'I' lagi -- TANPA reset papan #####\n");
  lidar.pindaiI2C(); putar(600); lidar.cetakTabel();

  printf("\n##### 'I' ditekan lagi saat semua sehat (tidak boleh init ulang) #####\n");
  lidar.pindaiI2C();
  return 0;
}
