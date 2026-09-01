// Robot Anda apa adanya: ch0 (kiri depan) dan ch2 (belakang) rusak fisik,
// empat lainnya sehat. Apakah 'F' bisa jalan dan 'f' ditolak dengan jelas?
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
  Calib::load(); __serialDiam = false;
  __initGagal[0] = __initGagal[2] = true;      // dua sensor rusak fisik
  for (int i=0;i<6;i++) set1X(i, 8190);
  set1X(LIDAR_FRONT, 1200);   // depan lapang 120 cm
  set1X(LIDAR_KANAN_D, 200);   // dinding kanan 20 cm
  set1X(LIDAR_KANAN_B, 400);
  set1X(LIDAR_KIRI_B, 8190);

  printf("\n##### BOOT #####\n");
  robot.begin(); lidar.begin(); robot.arm();
  for(int i=0;i<5;i++){ suapYaw(); imu.update(); __nowMs+=10; }
  for(int i=0;i<400;i++){ __nowMs+=2; lidar.update(); }
  lidar.cetakTabel();

  printf("\n##### 'f' (ikut dinding KIRI -- sensornya rusak) #####\n");
  nav.navMulai(NAV_DINDING_KIRI);
  printf("  -> mode sesudahnya: %s\n", nav.navMode()==NAV_DIAM ? "DIAM (ditolak, benar)" : "JALAN (SALAH!)");

  printf("\n##### 'F' (ikut dinding KANAN -- sensornya sehat) #####\n");
  nav.navMulai(NAV_DINDING_KANAN);
  for(int i=0;i<200;i++){ __nowMs+=5; suapYaw(); imu.update(); lidar.update(); nav.navUpdate(); robot.update(); }
  printf("  -> mode %s, maju %.2f putar %+.2f\n",
         nav.navMode()==NAV_DIAM?"DIAM (SALAH!)":"JALAN", nav.majuKini(), nav.turnKini());
  return 0;
}
