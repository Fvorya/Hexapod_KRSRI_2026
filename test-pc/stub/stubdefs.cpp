#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "config.h"
#include <VL53L1X.h>
uint32_t __nowMs = 0;
bool __serialDiam = true;
int  __muxCh = 0;
bool __muxAda = true;
uint16_t __simMm[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
bool __initGagal[8] = {false};
bool __bisu[8] = {false};
uint8_t __simStatus[8] = {0,0,0,0,0,0,0,0};   // 0 = RangeValid
uint32_t __simPeriodMs = LIDAR_PERIOD_MS;
uint32_t __simLastMeas[8] = {0,0,0,0,0,0,0,0};
bool __simJalan[8] = {false};
bool __simCrosstalkAntar = false;
uint16_t __simHantuMm[8] = {40,40,40,40,40,40,40,40};
__SerialStub Serial, Serial2;
uint8_t __EE[8192];
__EEStub EEPROM;
__WireStub Wire, Wire1, Wire2;

uint16_t __servoUs[2][16] = {{0}};
