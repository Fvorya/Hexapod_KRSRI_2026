// Di lorong terbuka (dinding samping hilang), apakah navigasi MENCARI dinding
// atau malah berhenti karena mengira sensornya putus?
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <cstdio>
#include "Calib.h"
#include "Imu.h"
#include "Hexapod.h"
#include "Navigation.h"
#include "LidarArray.h"
static Imu imu; static Hexapod robot; static LidarArray lidar;
static Navigation nav(imu, robot, lidar);
static void suapYaw(){ uint8_t f[11]={0x55,0x53,0,0,0,0,0,0,0,0,0};
  uint8_t s=0; for(int i=0;i<10;i++) s+=f[i]; f[10]=s;
  for(int i=0;i<11;i++) Serial2.rx.push_back(f[i]); }

// Di VL53L1X, "tak ada target" adalah STATUS, bukan angka jarak. Helper ini
// menerjemahkan penanda 8190 yang dipakai simulasi jadi status yang benar.
static void set1X(int ch, uint16_t mm) {
    __simMm[ch]     = mm;
    __simStatus[ch] = (mm >= 8000) ? VL53L1X::SignalFail : VL53L1X::RangeValid;
}
int main(){
  Calib::load(); robot.begin(); lidar.begin(); robot.arm();
  for(int i=0;i<5;i++){ suapYaw(); imu.update(); __nowMs+=10; }
  // depan lapang, dinding kiri ADA di 20 cm
  for(int i=0;i<8;i++) set1X(i, 8190);
  set1X(LIDAR_KIRI_D, 200);
  for(int i=0;i<400;i++){ __nowMs+=2; lidar.update(); }
  nav.navMulai(NAV_DINDING_KIRI);
  for(int i=0;i<200;i++){ __nowMs+=5; suapYaw(); imu.update(); lidar.update(); nav.navUpdate(); robot.update(); }
  printf("\n  dinding kiri 20 cm, depan lapang -> mode %s, maju %.2f putar %+.2f\n",
         nav.navMode()==NAV_DIAM?"BERHENTI":"jalan", nav.majuKini(), nav.turnKini());
  // sekarang dinding kiri HILANG (tikungan / mulut lorong)
  set1X(LIDAR_KIRI_D, 8190);
  for(int i=0;i<400;i++){ __nowMs+=5; suapYaw(); imu.update(); lidar.update(); nav.navUpdate(); robot.update(); }
  printf("  dinding kiri HILANG          -> mode %s, maju %.2f putar %+.2f  (NAV_CARI_CMD %.2f)\n",
         nav.navMode()==NAV_DIAM?"BERHENTI":"jalan", nav.majuKini(), nav.turnKini(), NAV_CARI_CMD);
  // dan sekarang sensornya benar-benar putus
  __bisu[LIDAR_KIRI_D] = true;   // sensor benar-benar berhenti menjawab
  for(int i=0;i<200;i++){ __nowMs+=5; suapYaw(); imu.update(); lidar.update(); nav.navUpdate(); robot.update(); }
  printf("  sensor kiri PUTUS            -> mode %s\n\n",
         nav.navMode()==NAV_DIAM?"BERHENTI (benar)":"jalan (SALAH)");
  return 0;
}
