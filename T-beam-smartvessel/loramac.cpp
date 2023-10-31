#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include "boards.h"

// OTAA Keys

// LSB mode
static const u1_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x7A, 0x31 };

// LSB mode
static const u1_t PROGMEM DEVEUI[8] = { 0x66, 0x88, 0x88, 0x00, 0x00, 0x33, 0x32, 0x24 };

// MSB mode
static const u1_t PROGMEM APPKEY[16] = { 0x97, 0x19, 0x9F, 0x8F, 0x82, 0x1B, 0xA4, 0x84, 0xD8, 0x1F, 0xE9, 0x89, 0x27, 0x27, 0xFA, 0x6B };

const lmic_pinmap lmic_pins = {
  .nss = RADIO_CS_PIN,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RST_PIN,
  .dio = { RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_BUSY_PIN }
};

static osjob_t sendjob;
static int joinStatus = EV_JOINING;
static String lora_msg = "";

// Shared with T-beam-smartvessel.ino
unsigned char lora_data[128];

// Import from T-beam-smartvessel.ino
extern unsigned int lora_data_size;
extern unsigned int lora_tx_interval;

void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

void do_send(osjob_t *j) {
  if (joinStatus == EV_JOINING) {
    Serial.println(F("Not joined yet"));
    // Check if there is not a current TX/RX job running
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);

  } else if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    Serial.println(F("OP_TXRXPEND,sending ..."));
    // example string A020231001090241013021Lat7.007241Lon100.501968Alt62.516026
    //static uint8_t mydata[] = "A631001090241013021La7.007241Lo100.501968Al62.51"; // working string

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lora_data, lora_data_size, 0);
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);

#ifdef HAS_DISPLAY
    if (u8g2) {
      char buf[256];
      u8g2->clearBuffer();
      snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
      u8g2->drawStr(0, 12, buf);
      u8g2->sendBuffer();
    }
#endif
  }
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
        lora_msg = "Received ACK.";
      }

      lora_msg = "rssi:" + String(LMIC.rssi) + " snr: " + String(LMIC.snr);

      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        // Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        // Serial.println();
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(lora_tx_interval), do_send);
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING: -> Joining..."));
      lora_msg = "OTAA joining....";
      joinStatus = EV_JOINING;
#ifdef HAS_DISPLAY
      if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "OTAA joining....");
        u8g2->sendBuffer();
      }
#endif
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
      lora_msg = "OTAA Joining failed";
#ifdef HAS_DISPLAY
      if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "OTAA joining failed");
        u8g2->sendBuffer();
      }
#endif
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      lora_msg = "Joined!";
      joinStatus = EV_JOINED;

#ifdef HAS_DISPLAY
      if (u8g2) {
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "Joined TTN!");
        u8g2->sendBuffer();
      }
#endif
      delay(3);
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);

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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setupLMIC(void) {
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // turn on adaptive datarate
  LMIC_setAdrMode(1); 
  LMIC_setLinkCheckMode(1); // default 0
  LMIC_setDrTxpow(DR_SF9, 14); 

  // TTN uses SF9 for its RX2 window.
   // LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(spreadFactor, 14); // DR_SF10
  // initial datarate and power
  // LMIC_setDrTxpow(DR_SF10, 14); //

  // Start job
  LMIC_startJoining();
  // Send lora_data
  do_send(&sendjob);  // Will fire up also the join
}

void loopLMIC(void) {
  os_runloop_once();
}
