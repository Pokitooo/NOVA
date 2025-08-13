#include <Arduino.h>
#include <RadioLib.h>
#include "nova_pin_def.h"
#include "SPI.h"

SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);
SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

constexpr struct
{
  float center_freq = 920.800'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 7;     // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 10;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

volatile bool receivedFlag = false;

void setFlag()
{
  receivedFlag = true;
}

String rx_buf;

void setup()
{
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
  if (rc != RADIOLIB_ERR_NONE)
  {
    Serial.print("Initialization failed! Error: ");
    Serial.println(rc);
    while (true)
    {
      delay(10);
    }
  }

  lora.explicitHeader();
  lora.setCRC(true);
  lora.autoLDRO();
  lora.setPacketReceivedAction(setFlag);

  Serial.print(F("[SX1262] Starting to listen ... "));
  rc = lora.startReceive();
  if (rc == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(rc);
    while (true)
      ;
  }
  rx_buf.reserve(300);
}

void loop()
{
  // check if the flag is set
  if (receivedFlag)
  {
    receivedFlag = false;
    rx_buf = "";
    const int state = lora.readData(rx_buf);

    if (state == RADIOLIB_ERR_NONE)
    {
      Serial.printf("SIZE=%d RSSI=%.2f SNR=%.2f DATA=",
                    lora.getPacketLength(), lora.getRSSI(), lora.getSNR());
      Serial.print(rx_buf);
      Serial.print(F("\n[END OF PACKET]\n"));
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      Serial.println(F("CRC error!"));
    }
    else
    {
      Serial.print(F("Failed, code "));
      Serial.println(state);
    }
  }
  else
  {
    if (!Serial.available())
      return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      return;

    // Blocking TX to keep state clean
    int16_t rc = lora.transmit(line);
    if (rc == RADIOLIB_ERR_NONE) {
      Serial.println(F("Transmission successful!"));
    }
    else {
      Serial.print(F("TX failed, code "));
      Serial.println(rc);
    }

    // ALWAYS return to RX after TX
    lora.startReceive();
  }
}
