#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>

#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <MPU6050_WE.h>
#include "SdFat.h"

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// Devices
SFE_UBLOX_GNSS m10q;
MPU6500_WE mpu6500 = MPU6500_WE(0x68);

// SD
// SPIClass SPI_SD(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCLK1);
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;

struct Data
{
  uint32_t timestamp;

  float temp;

  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float resultantG;
};

Data data;
SemaphoreHandle_t i2cMutex;

extern void read_m10q(void *);

extern void read_mpu6500(void *);

void setup()
{
  Serial.begin(460800);

  Wire.begin();
  Wire.setClock(300000u);

  i2cMutex = xSemaphoreCreateMutex();

  // m10q (0x42)
  if (m10q.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
  {
    m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10q.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
  }

  // mpu6500
  if (mpu6500.init())
  {
    mpu6500.autoOffsets();
    mpu6500.enableGyrDLPF();
    mpu6500.setGyrDLPF(MPU6500_DLPF_6);
    mpu6500.setSampleRateDivider(5);
    mpu6500.setAccRange(MPU6500_ACC_RANGE_4G);
    mpu6500.enableAccDLPF(true);
    mpu6500.setAccDLPF(MPU6500_DLPF_6);
    delayMicroseconds(200);
  }

  /*
  valid.sd0 = sd0.begin(SdSpiConfig(PIN_SPI_CS_SD_INT, DEDICATED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_SD));
  if (valid.sd0)
  {
    make_new_filename(sd0, sd0_filename, F_NAME, F_EXT);
    open_for_append(sd0, sd0_file, sd0_filename);
  }
  */

  xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_mpu6500, "", 2048, nullptr, 2, nullptr);
  vTaskStartScheduler();
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

void read_mpu6500(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      xyzFloat gValue = mpu6500.getGValues();
      xyzFloat gyr = mpu6500.getGyrValues();

      // acceleration
      data.acc_x = gValue.x;
      data.acc_y = gValue.y;
      data.acc_z = gValue.z;

      // gyroscope
      data.gyro_x = gyr.x;
      data.gyro_y = gyr.y;
      data.gyro_z = gyr.z;

      data.temp = mpu6500.getTemperature();
      data.resultantG = mpu6500.getResultantG(gValue);

      xSemaphoreGive(i2cMutex);
    }
    DELAY(100);
  }
}

void loop() {}