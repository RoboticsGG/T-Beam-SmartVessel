#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "utilities.h"

#ifdef HAS_SDCARD
#include <SD.h>
#include <FS.h>
#endif

#ifdef HAS_DISPLAY
#include <U8g2lib.h>

#ifndef DISPLAY_MODEL
#define DISPLAY_MODEL U8G2_SSD1306_128X64_NONAME_F_HW_I2C
#endif

#endif

#ifndef OLED_WIRE_PORT
#define OLED_WIRE_PORT Wire
#endif

#if defined(HAS_PMU) //

#include <axp20x.h>
#include "XPowersLib.h"

#ifndef PMU_WIRE_PORT
#define PMU_WIRE_PORT Wire
#endif

void setPmuFlag();
bool initPMU();
void disablePeripherals();
void initBoard();

#endif // HAS_PMU