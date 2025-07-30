#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>

#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include "ICM42688.h"
#include <Adafruit_BME280.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "Arduino_Extended.h"

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// Pins defination
#define PIN_SPI_MOSI1 PA7
#define PIN_SPI_MISO1 PA6
#define PIN_SPI_SCK1 PA5

// NSS
#define PIN_NSS_ICM PA15
#define PIN_NSS_SD PB9

// lORA PIN DEF
#define LORA_DIO1 PB0
#define LORA_NSS PA4
#define LORA_BUSY PB1
#define LORA_NRST PB2

// i2c
TwoWire i2c3(PB8, PA8);
SFE_UBLOX_GNSS m10q;
Adafruit_BME280 bme;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(25), &spi1);
SdExFat sd = {};
ExFile file = {};

// LoRa
volatile bool tx_flag = false;
String s;

SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
  float center_freq = 920.800'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 12;    // SF: 6 to 12
  uint8_t coding_rate = 8;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 22;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 16;
} params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// ICM
ICM42688 icm(spi1, PA15);

struct Data
{
  // 40 bits
  uint32_t timestamp;
  uint8_t counter;

  // 160 bits
  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  // 96 bits
  float temp;
  float humid;
  float press;

  // 192 bits
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

Data data;
SemaphoreHandle_t i2cMutex;

// Communication data
String constructed_data;

// Buzzer
const unsigned long threeHours = 10800000; // 3 hours in milliseconds
unsigned long startTime;
#define buzzerPin PA0

extern void read_m10q(void *);

extern void read_bme(void *);

extern void read_icm(void *);

extern void construct_data(void *);

extern void send_data(void *);

extern void save_data(void *);

extern void print_data(void *);

extern void buzz(void *);

extern void set_tx_flag();

void setup()
{
  Serial.begin(460800);
  delay(2000);

  i2c3.begin();
  i2c3.setClock(300000u);

  spi1.begin();

  i2cMutex = xSemaphoreCreateMutex();

  // GPIO
  pinMode(buzzerPin, OUTPUT); // BUZZER
  digitalWrite(buzzerPin, 1);
  delay(100);
  digitalWrite(buzzerPin, 0);
  pinMode(PB5, OUTPUT); // LED

  // variable
  static bool state;

  // SD
  state = sd.begin(sd_config);
  Serial.printf("SD CARD: %s\n", state ? "SUCCESS" : "FAILED");
  if (!state)
    while (true)
      ;
  if (file = sd.open("data.csv", O_RDWR | O_CREAT | O_AT_END | O_APPEND))
  {
    Serial.println("File open Success!");
  }
  else
  {
    Serial.println("File open failed!");
  }

  // LoRa
  int16_t ls = lora.begin(params.center_freq,
                          params.bandwidth,
                          params.spreading_factor,
                          params.coding_rate,
                          params.sync_word,
                          params.power,
                          params.preamble_length,
                          0,
                          false);
  state = state || lora.explicitHeader();
  state = state || lora.setCRC(true);
  state = state || lora.autoLDRO();
  attachInterrupt(LORA_DIO1, set_tx_flag, CHANGE);

  if (ls == RADIOLIB_ERR_NONE)
  {
    Serial.println("SX1262 initialized successfully!");
  }
  else
  {
    Serial.print("Initialization failed! Error: ");
    Serial.println(ls);
    while (true)
      ;
  }

  s.reserve(256);

  // icm42688
  uint8_t status = icm.begin();
  if (status > 0)
  {
    icm.setAccelFS(ICM42688::gpm8);
    icm.setGyroFS(ICM42688::dps500);
    icm.setAccelODR(ICM42688::odr12_5);
    icm.setGyroODR(ICM42688::odr12_5);
    Serial.println("ICM Success");
  }
  Serial.print("ICM Status: ");
  Serial.println(status);

  // m10q (0x42)
  if (m10q.begin(i2c3))
  {
    m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    Serial.println("gps Success");
  }

  // bme280(0x76)
  if (!bme.begin(0x76, &i2c3))
  {
    Serial.println("Could not find a valid BME280 sensor");
  }

  xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(construct_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(send_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(save_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(print_data, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(buzz, "", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();
}

void buzz(void *)
{
  for (;;)
  {
    unsigned long currentTime = millis();

    if (currentTime - startTime >= threeHours)
    {
      digitalToggle(buzzerPin);
      digitalToggle(PB5);
      DELAY(500);
    }
  }
}

void read_m10q(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      if (m10q.getPVT())
      {
        data.timestamp = m10q.getUnixEpoch(UBLOX_CUSTOM_MAX_WAIT);
        data.gps_latitude = static_cast<double>(m10q.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_longitude = static_cast<double>(m10q.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_altitude = static_cast<float>(m10q.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
      }
      xSemaphoreGive(i2cMutex);
    }
    DELAY(500);
  }
}

void read_bme(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      data.temp = bme.readTemperature();
      data.humid = bme.readHumidity();
      data.press = bme.readPressure() / 100.0F;
      xSemaphoreGive(i2cMutex);
    }
    DELAY(200);
  }
}

void read_icm(void *)
{
  for (;;)
  {
    icm.getAGT();
    // acceleration
    data.acc_x = icm.accX();
    data.acc_y = icm.accY();
    data.acc_z = icm.accZ();

    // gyroscope
    data.gyro_x = icm.gyrX();
    data.gyro_y = icm.gyrY();
    data.gyro_z = icm.gyrZ();
    DELAY(1000);
  }
}

void construct_data(void *)
{
  for (;;)
  {
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        << "<1>"
        << data.counter
        << data.timestamp
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.gps_altitude, 4)
        << data.temp
        << data.humid
        << data.press
        << data.acc_x
        << data.acc_y
        << data.acc_z
        << data.gyro_x
        << data.gyro_y
        << data.gyro_z;
    DELAY(1000);
  }
}

void send_data(void *)
{
  for (;;)
  {
    static int16_t state;
    static uint32_t t0, t;

    if (tx_flag)
    {
      tx_flag = false;
      t = millis();

      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.println("Transmission successful!");
        Serial.printf("Used %d ms\n", t - t0);
      }
      else
      {
        Serial.print("Transmission failed! Error: ");
        Serial.println(state);
      }
    }
    else
    {
      lora.startTransmit(constructed_data.c_str());
      t0 = millis();
    }
    DELAY(2000);
    ++data.counter;
  }
}

void save_data(void *)
{
  for (;;)
  {
    file.println(constructed_data);
    file.flush();
    Serial.println("Data written and flushed.");
    DELAY(1000);
  }
}

void print_data(void *)
{
  for (;;)
  {
    Serial.println("===== Data Output =====");
    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);

    Serial.print("GPS Latitude: ");
    Serial.println(data.gps_latitude, 6);
    Serial.print("GPS Longitude: ");
    Serial.println(data.gps_longitude, 6);
    Serial.print("GPS Altitude: ");
    Serial.println(data.gps_altitude);

    Serial.print("Env Temp: ");
    Serial.println(data.temp);
    Serial.print("Humidity: ");
    Serial.println(data.humid);
    Serial.print("Pressure: ");
    Serial.println(data.press);

    Serial.print("Acc X: ");
    Serial.println(data.acc_x);
    Serial.print("Acc Y: ");
    Serial.println(data.acc_y);
    Serial.print("Acc Z: ");
    Serial.println(data.acc_z);
    Serial.print("Gyro X: ");
    Serial.println(data.gyro_x);
    Serial.print("Gyro Y: ");
    Serial.println(data.gyro_y);
    Serial.print("Gyro Z: ");
    Serial.println(data.gyro_z);
    Serial.print(constructed_data);
    DELAY(500);
  }
}

void set_tx_flag()
{
  tx_flag = true;
  digitalToggle(PB5);
}

void loop()
{
  DELAY(1);
}
