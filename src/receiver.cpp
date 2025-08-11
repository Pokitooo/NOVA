#include <Arduino.h>
#include <RadioLib.h>
#include "nova_pin_def.h"
#include "SPI.h"

SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);
SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

constexpr struct {
  float   center_freq      = 920.800'000f;  // MHz
  float   bandwidth        = 125.f;         // kHz
  uint8_t spreading_factor = 9;             // 6..12
  uint8_t coding_rate      = 8;             // 5..8
  uint8_t sync_word        = 0x12;          // Private
  int8_t  power            = 22;            // dBm
  uint16_t preamble_length = 16;
} params;

constexpr uint32_t TX_INTERVAL_MS = 1000;   // TX interval
volatile bool txFlag = false;               // TX trigger flag

static uint32_t lastTxMs = 0;
static uint32_t txCount  = 0;

void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("Serial begin");

  spi1.begin();
  int16_t rc = lora.begin(params.center_freq,
                          params.bandwidth,
                          params.spreading_factor,
                          params.coding_rate,
                          params.sync_word,
                          params.power,
                          params.preamble_length,
                          0,
                          false);
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.print("Initialization failed! Error: ");
    Serial.println(rc);
    while (true) { delay(10); }
  }

  lora.explicitHeader();
  lora.setCRC(true);
  lora.autoLDRO();

  Serial.print(F("[SX1262] Starting to listen ... "));
  rc = lora.startReceive();
  if (rc == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(rc);
    while (true) { delay(10); }
  }
}

static void handleDownlink() {
  int packetSize = lora.getPacketLength();
  if (packetSize <= 0) return;

  String str;
  int16_t rc = lora.readData(str);

  if (rc == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1262] Received packet!"));
    Serial.print(F("[SX1262] Data:\t\t"));
    Serial.println(str);

    Serial.print(F("[SX1262] RSSI:\t\t"));
    Serial.print(lora.getRSSI());
    Serial.println(F(" dBm"));

    Serial.print(F("[SX1262] SNR:\t\t"));
    Serial.print(lora.getSNR());
    Serial.println(F(" dB"));

    Serial.print(F("[SX1262] Frequency error:\t"));
    Serial.print(lora.getFrequencyError());
    Serial.println(F(" Hz"));

    // Example: set txFlag based on command
    if (str == "cmd tx") {
      txFlag = true;
      Serial.println(F("[SX1262] TX flag set!"));
    }
  } else if (rc == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("CRC error!"));
  } else {
    Serial.print(F("readData failed, code "));
    Serial.println(rc);
  }
}

static void doUplinkIfDue() {
  uint32_t now = millis();
  if (!txFlag) return;  // Only send if flag is set
  if (now - lastTxMs < TX_INTERVAL_MS) return;

  String payload = "ping " + String(txCount++);
  int16_t rc = lora.transmit(payload);
  if (rc == RADIOLIB_ERR_NONE) {
    Serial.print(F("[SX1262] Uplink sent: "));
    Serial.println(payload);
  } else {
    Serial.print(F("[SX1262] TX failed, code "));
    Serial.println(rc);
  }

  rc = lora.startReceive();
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.print(F("[SX1262] startReceive after TX failed, code "));
    Serial.println(rc);
  }

  lastTxMs = now;
  txFlag = false; // Reset flag after sending
}

void loop() {
  handleDownlink();
  doUplinkIfDue();
}
