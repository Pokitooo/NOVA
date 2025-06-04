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

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))
#define SEALEVELPRESSURE_HPA (1013.25)

// Pins defination
/*
constexpr PinName PIN_SPI_MOSI1 =
constexpr PinName PIN_SPI_MISO1 =
constexpr PinName PIN_SPI_SCLK1 =
constexpr PinName PIN_SPI_CS_SD_INT =
*/

// Devices
TwoWire i2c3(PB8, PA8);

SFE_UBLOX_GNSS m10q;
Adafruit_BME280 bme;
ICM42688 icm(SPI, PA15);
// SX1262 lora = new Module(PA4, PB1, PB2, PB0);
//  Create radio instance: NSS, DIO1, RESET, BUSY

volatile bool dataReady = false;

// SD
// SPIClass SPI_SD(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCLK1);
//SdSpiConfig sd0_cfg(PIN_SPI_CS_SD_INT, DEDICATED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD);
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;

struct Data
{
  uint32_t timestamp;

  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  float temp;
  float humid;
  float press;
  float press_altitude;

  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

Data data;
SemaphoreHandle_t i2cMutex;

extern void read_m10q(void *);

extern void read_icm(void *);

extern void read_bme(void *);

extern void print_data(void *);

extern void buzz(void *);

void setup()
{
  Serial.begin(460800);

  Wire.begin();
  i2c3.begin();
  Wire.setClock(300000u);

  i2cMutex = xSemaphoreCreateMutex();

  pinMode(PA0, OUTPUT);

  // m10q (0x42)
  if (m10q.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
  {
    m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
  }

  // icm42688
  uint8_t status = icm.begin();
  if (status > 0)
  {
    icm.setAccelFS(ICM42688::gpm8);
    icm.setGyroFS(ICM42688::dps500);
    icm.setAccelODR(ICM42688::odr12_5);
    icm.setGyroODR(ICM42688::odr12_5);
  }

  if (!bme.begin(0x76, &Wire))
  {
    Serial.println("Could not find a valid BME280 sensor");
  }

  
  // valid.sd0 = sd0.begin(SdSpiConfig(PIN_SPI_CS_SD_INT, DEDICATED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
  // if (valid.sd0)
  // {
  //   make_new_filename(sd0, sd0_filename, F_NAME, F_EXT);
  //   open_for_append(sd0, sd0_file, sd0_filename);
  // }
  

  xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(print_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(buzz, "", 2048, nullptr, 2, nullptr);
  vTaskStartScheduler();
}

void buzz(void *)
{
  for (;;)
  {
    Serial.println("Hello");
    digitalToggle(PA0);
    DELAY(500);
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
      data.press_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
      xSemaphoreGive(i2cMutex);
    }
    DELAY(200);
  }
}

void read_icm(void *)
{
  for (;;)
  {
    dataReady = false;

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

void uplink(void *)
{
  for (;;)
  {
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
    Serial.print("Pressure Altitude: ");
    Serial.println(data.press_altitude);

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
  }
}

void loop()
{
  DELAY(1);
}
