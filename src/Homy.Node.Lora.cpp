/*
  Homy Node

  Homy Node sketch - monitors humidity and temperature and publishes data using LoRaWAN

  Copyright (c) 2017 Dusan Stojkovic
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Streaming.h>
#include <LowPower.h>
#include "DHT.h"

#define DEBUG true
#define LOG if(DEBUG)Serial

#define SENSOR_VALID_INTERVAL 5 // seconds
#define DATA_INTERVAL 60 // minutes

unsigned int loops = 0;

int batteryLevelWarnings=0;

double humidity;
double humidityCummulative=0;

double temperature;
double temperatureCummulative=0;

double batteryLevel;
double batteryLevelCummulative=0;

int countCummulative=0;

int TX_COMPLETE = 0;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x06, 0x4B, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 }; // Thingy
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

byte data_buffer[64];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

DHT dht(10, DHT22);

void ReadData(bool cummulative)
{
  LOG << F("[") << millis() << F("] Acquiring Data - ");

  // battery level
  int batteryVoltage = 0;
  for (int i=0; i<4; i++) batteryVoltage += analogRead(A0);

  // humidity & temperature
  SPCR &= ~_BV(SPE); // use SS for DHT22
  bool ok = dht.read(true);
  SPCR |= _BV(SPE); // use SS for SPI

  if (ok)
  {
    humidityCummulative += dht.readHumidity();
  	temperatureCummulative += dht.readTemperature();
  	batteryLevelCummulative += 1.0 * batteryVoltage / 4096;

  	countCummulative++;
  	humidity = 1.0 * humidityCummulative / countCummulative;
  	temperature = 1.0 * temperatureCummulative / countCummulative;
  	batteryLevel = 6.666 * batteryLevelCummulative / countCummulative;

    LOG << humidity << F("% - ") << temperature << F("C - ") << batteryLevel << F("V")<< endl;
  }
  else
  {
    LOG << F("ERROR") << endl;
  }

	if (!cummulative)
	{
		countCummulative=0;
		humidityCummulative=0;
		temperatureCummulative=0;
		batteryLevelCummulative=0;
	}

  if (batteryLevel < 3.6)
		batteryLevelWarnings++;
	else
		batteryLevelWarnings = 0;
}

void onEvent (ev_t ev) {
    LOG << F("[") << millis() << F("] ");
    switch(ev) {
        case EV_JOINING:
            LOG << F("EV_JOINING") << endl;
            break;
        case EV_JOINED:
            LOG << F("EV_JOINED") << endl;
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            LOG << F("EV_JOIN_FAILED") << endl;
            break;
        case EV_REJOIN_FAILED:
            LOG << F("EV_REJOIN_FAILED") << endl;
            break;
        case EV_TXCOMPLETE:
            LOG << F("EV_TXCOMPLETE") << endl;
            TX_COMPLETE = 1;
            if (LMIC.txrxFlags & TXRX_ACK)
            {
              LOG << F("[") << millis() << F("] ACKed");
            }
            if (LMIC.dataLen) {
              LOG << F("[") << millis() << F("] Received ") << LMIC.dataLen << F(" bytes of payload - ");
              LOG.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
            }
            LOG << endl;
            break;
        case EV_LINK_DEAD:
            LOG << F("EV_LINK_DEAD") << endl;
            break;
        case EV_LINK_ALIVE:
            LOG << F("EV_LINK_ALIVE") << endl;
            break;
    }
}

void setup() {
  // Setup console
  Serial.begin(115200);
  delay(10);
  Serial << endl << endl  << F("Homy.Room.Lora") << " - " << __DATE__ << " " << __TIME__ << endl << "#" << endl << endl;

  // Init DHT sensor
  dht.begin();

  // Init LoRa
  os_init();
  LMIC_reset();
}

void loop()
{
  unsigned long now = millis();
	bool sendData = (loops % 4 == 0);

  LOG << F("[") << now << F("] Loop ") << loops << endl;

  // read data
  ReadData(!sendData);

  if (sendData)
  {
    // sending data
    String message = String("{")+
      "\"t\":" + String(temperature,1) + String(",") +
      "\"h\":"+ String(humidity,1) + String(",") +
      "\"b\":" + String(batteryLevel,2) + String(",") +
      "\"l\":" + (batteryLevelWarnings > 0 ? String("1") : String("0")) +
      "}";
    message.getBytes(data_buffer, message.length()+1);
    LOG << F("[") << millis() << F("] Sending - ") << message;
    int res = LMIC_setTxData2(1, (uint8_t*) data_buffer, message.length(), 1);
    LOG << ((res == 0) ? F(" - OK") : F(" - ERROR")) << endl;

    while (TX_COMPLETE == 0)
        os_runloop_once();
    TX_COMPLETE = 0;

    // blink
    LOG.flush();
    SPCR &= ~_BV(SPE); // use SS for LED
    digitalWrite(13,HIGH); LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);digitalWrite(13,LOW);
    SPCR |= _BV(SPE); // use SS for SPI
  }
  else
  {
    os_runloop_once();
  }

  // empty battery - shutdown
  if (batteryLevelWarnings > 8)
  {
    LOG << F("[") << millis() << F("] Going to deep sleep! Change the battery!") << endl;
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    return;
  }

  // low battery blinking
  if (batteryLevelWarnings>0)
  {
    LOG << F("[") << millis() << F("] Battery level low (") << batteryLevelWarnings << F(") ! ") << batteryLevel << F("V") << endl;
    LOG.flush();
    // blink LED if low battery - 1280ms
    SPCR &= ~_BV(SPE); // use SS for LED
    digitalWrite(13,HIGH); LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    digitalWrite(13,LOW); LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    digitalWrite(13,HIGH); LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    digitalWrite(13,LOW); LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    digitalWrite(13,HIGH); LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    digitalWrite(13,LOW);
    SPCR |= _BV(SPE); // use SS for SPI
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
  }

  LOG << F("[") << millis() << F("] zz.. ");
  LOG.flush();
  for (int s = 0; s < DATA_INTERVAL / 4 * 7; s++)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LOG << F(" ..zz [") << millis() << F("]") << endl << endl;
  loops++;
}
