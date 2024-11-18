/*
  This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
  Base on TinyGPSPlus //https://github.com/mikalhart/TinyGPSPlus
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <lmic.h>
#include <hal/hal.h>
#include "boards.h"

#define DEBUG_SERIAL_PRINT

#define GPS_EXT_LED 2  // GPIO 02
#define TX_EXT_LED 13  // GPIO 13

#define NODE_INFO "A4"

TinyGPSPlus gps;
TinyGPSCustom snr[4];

// OTAA EUI Keys

// LSB APPEUI
static const u1_t PROGMEM APPEUI[8] = { 0x01, 0x0A, 0xAF, 0xF5, 0x60, 0x5D, 0x4A, 0x17 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// LSB DEVEUI
static const u1_t PROGMEM DEVEUI[8] = { 0x75, 0x27, 0x80, 0xE0, 0xE7, 0x95, 0xB1, 0x8E };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// MSB APPKEY
static const u1_t PROGMEM APPKEY[16] = { 0x1B, 0x3A, 0x39, 0xB9, 0x9B, 0x29, 0x6C, 0x51, 0x05, 0x5F, 0xBC, 0xCE, 0x2B, 0x57, 0xA9, 0xC1 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

const lmic_pinmap lmic_pins = {
  .nss = RADIO_CS_PIN,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RST_PIN,
  .dio = { RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_BUSY_PIN }
};

// Payload bytes
unsigned char lora_data[128];

static osjob_t sendjob;
static int joinStatus = EV_JOINING;
static String lora_msg = "";


// Specific for Smartvessel
unsigned int lora_data_size;
unsigned int lora_tx_interval = 30;  // in seconds

// RoboticsGG: Get average signal level
static unsigned int GetAvgSigLevel() {
  int sum = 0;
  for (int i = 0; i < 4; ++i) {
    sum += atoi(snr[i].value());
  }
  return sum / 4;
}

// RoboticsGG: Update GPS external LED state
static void UpdateGPSExtLed() {
  bool bGpsOk = (gps.date.isValid() && gps.time.isValid() && gps.location.isValid() && gps.altitude.isValid() && gps.satellites.isValid());

  digitalWrite(GPS_EXT_LED, bGpsOk ? HIGH : LOW);
}

// RoboticsGG: Blink LoRa Tx external LED
static void BlinkTxExtLed() {
  unsigned long startTime;

  digitalWrite(TX_EXT_LED, HIGH);

  // Delay 1 second
  startTime = millis();
  while (millis() - startTime < 1000)
    ;

  digitalWrite(TX_EXT_LED, LOW);
}

// RoboticsGG: Send location data via LoRa
static void SendLoRaPacket() {
  if (gps.date.isValid() && gps.time.isValid() && gps.location.isValid() && gps.altitude.isValid() && gps.satellites.isValid()) {
    char txPacket[128];
    unsigned int dataSize;

    /* Data order = NodeType, NodeID, YYYY-MM-DD, HH:MM:SS, numSat, sigLevel, Latitude, Longitude, Altitude */
    snprintf(txPacket, sizeof(txPacket), "%s%04d%02d%02d%02d%02d%02d%03d%03dLat%.6lfLon%.6lfAlt%.6lfMsgi...T-Beam1.1...",
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
    //loopLMIC();
    os_runloop_once();

    //BlinkTxExtLed();

    //Serial.println(txPacket);
  }
}

// for console logging
static void displayInfo() {

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

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");

          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");

          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
      /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
        This event is defined but not used in the code. 
        No point in wasting codespace on it.
        
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;

    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

// Tx
void do_send_old(osjob_t* j) {

  if (joinStatus == EV_JOINING) {
    Serial.println(F("Not joined yet"));
    // Check if there is not a current TX/RX job running
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);

  } else if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    Serial.println(F("OP_TXRXPEND,sending ..."));
    // example injected string A020231001090241013021Lat7.007241Lon100.501968Alt62.516026
    //static uint8_t mydata[] = "A631001090241013021La7.007241Lo100.501968Al62.51"; // working string

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lora_data, lora_data_size, 0);
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);
  }
}

// Tx
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lora_data, lora_data_size, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
  Serial.begin(115200);
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
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);  // offsets 7, 11, 15, 19
  }

  // Initialize LoRa
  Serial.println("Setup LoRa");
  memset(lora_data, 0, sizeof(lora_data));
  lora_data_size = 0;

  //
  // setupLMIC();
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      //displayInfo();
      //UpdateGPSExtLed();
      SendLoRaPacket();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    while (true)
      ;
  }
}
