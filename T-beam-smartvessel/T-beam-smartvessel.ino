/*
  This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
  Base on TinyGPSPlus //https://github.com/mikalhart/TinyGPSPlus
*/

#include <TinyGPSPlus.h>
#include "loramac.h"
#include "boards.h"

#define DEBUG_SERIAL_PRINT

#define GPS_EXT_LED 2  // GPIO 02
#define TX_EXT_LED  13 // GPIO 13

#define NODE_INFO "A6"

TinyGPSPlus gps;
TinyGPSCustom snr[4];

// Import from loramac.cpp
extern unsigned char lora_data[128];

// Shared with loramac.cpp
unsigned int lora_data_size;
unsigned int lora_tx_interval = 20; // in seconds

// RoboticsGG: Get average signal level
static unsigned int GetAvgSigLevel()
{
  int sum = 0;

  for (int i = 0; i < 4; ++i) {
    sum += atoi(snr[i].value());
  }

  return sum / 4;
}

// RoboticsGG: Update GPS external LED state
static void UpdateGPSExtLed()
{
  bool bGpsOk = (gps.date.isValid() && gps.time.isValid() &&
                 gps.location.isValid() && gps.altitude.isValid() &&
                 gps.satellites.isValid());

  digitalWrite(GPS_EXT_LED, bGpsOk ? HIGH : LOW);
}

// RoboticsGG: Blink LoRa Tx external LED
static void BlinkTxExtLed()
{
  unsigned long startTime;

  digitalWrite(TX_EXT_LED, HIGH);

  // Delay 1 second
  startTime = millis();
  while (millis() - startTime < 1000);

  digitalWrite(TX_EXT_LED, LOW);
}

// RoboticsGG: Send location data via LoRa
static void SendLoRaPacket()
{
  if (gps.date.isValid() && gps.time.isValid() &&
      gps.location.isValid() && gps.altitude.isValid() &&
      gps.satellites.isValid())
  {
    char txPacket[128];
    unsigned int dataSize;

    /* Data order = NodeType, NodeID, YYYY-MM-DD, HH:MM:SS, numSat, sigLevel, Latitude, Longitude, Altitude */
    snprintf(txPacket, sizeof(txPacket), "%s%04d%02d%02d%02d%02d%02d%03d%03dLat%.6lfLon%.6lfAlt%.6lf",
             NODE_INFO,
             gps.date.year(),
             gps.date.month(),
             gps.date.day(),
             gps.time.hour(),
             gps.time.minute(),
             gps.time.second(),
             gps.satellites.value(),
             GetAvgSigLevel(),
             gps.location.lat(),
             gps.location.lng(),
             gps.altitude.meters());

    dataSize = strlen(txPacket);

    // Send data to LoRa
    memcpy(lora_data, txPacket, dataSize);
    lora_data_size = dataSize;
    loopLMIC();
    BlinkTxExtLed();

    Serial.println(txPacket);
  }
}

static void displayInfo()
{
  Serial.print("Location: ");
  if (gps.location.isValid() && gps.altitude.isValid() && gps.satellites.isValid()) {
    Serial.print("Lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print("Lon=");
    Serial.print(gps.location.lng(), 6);
    Serial.print(",");
    Serial.print("Alt=");
    Serial.print(gps.altitude.meters(), 6);
    Serial.print(",");
    Serial.print("nSat=");
    Serial.print(gps.satellites.value());
    Serial.print(",");
    Serial.print("sigLev=");
    Serial.print(GetAvgSigLevel());
  } else {
    Serial.print("INVALID");
  }

  Serial.print("  Date/Time: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.year());
  } else {
    Serial.print("INVALID");
  }

  Serial.print(" ");
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print("0");
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print("0");
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print("0");
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print("0");
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print("INVALID");
  }

  Serial.println();
}

void setup()
{
  initBoard();
  // When the power is turned on, a delay is required.
  delay(2000);

  // Initialize external LEDs
  pinMode(GPS_EXT_LED, OUTPUT);
  pinMode(TX_EXT_LED, OUTPUT);
  digitalWrite(GPS_EXT_LED, LOW);
  digitalWrite(TX_EXT_LED, LOW);

  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i = 0; i < 4; ++i) {
    snr[i].begin(gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }

  // Initialize LoRa
  Serial.println("Setup LoRa");
  memset(lora_data, 0, sizeof(lora_data));
  lora_data_size = 0;
  setupLMIC();
}

void loop()
{
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      //displayInfo();
      UpdateGPSExtLed();
      SendLoRaPacket();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    while (true);
  }
}
