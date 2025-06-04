#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "vt_bme280"
#include <SparkFun_u-blox_GNSS_v3.h>

#include <SPI.h>
#include "ICM42688.h"
#include "SdFat.h"
#include "psg_4_sd_tools.h"

using namespace vt;

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

// Pheripherals
#define SEALEVELPRESSURE_HPA (1013.25)

// Pin definitions
constexpr PinName PIN_SPI_MOSI1 = PA_7;
constexpr PinName PIN_SPI_MISO1 = PA_6;
constexpr PinName PIN_SPI_SCK1 = PA_5;
constexpr PinName PIN_NSS_ICM = PA_15;
constexpr PinName PIN_SPI_CS_SD = PB_9;

// i2c devices
TwoWire i2c3(PB8, PA8);
bme280_t bme;
SFE_UBLOX_GNSS m10q;

// SPI
SPIClass SPI_1(pinNametoDigitalPin(PIN_SPI_MOSI1), pinNametoDigitalPin(PIN_SPI_MISO1), pinNametoDigitalPin(PIN_SPI_SCK1));
ICM42688 icm(SPI_1, pinNametoDigitalPin(PIN_NSS_ICM));
SdSpiConfig sd0_cfg(PIN_SPI_CS_SD, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_1);

//Sd
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;

//Variables
uint16_t sat;
bool validsd0;

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

SemaphoreHandle_t i2cMutex;
Data data;

void buzz(void *)
{
    for (;;)
    {
        Serial.println("Hello");
        digitalToggle(PA0);
        DELAY(500);
    }
}

void read_bme(void *)
{
    for (;;)
    {
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
        {
            data.temp = bme.read_temperature_c();
            data.humid = bme.read_humidity();
            data.press = bme.read_pressure();
            xSemaphoreGive(i2cMutex);
        }
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
                sat = m10q.getSIV(UBLOX_CUSTOM_MAX_WAIT);
                data.gps_latitude = static_cast<double>(m10q.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
                data.gps_longitude = static_cast<double>(m10q.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
                data.gps_altitude = static_cast<float>(m10q.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
            }
            xSemaphoreGive(i2cMutex);
        }
        DELAY(1000);
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

        Serial.print("Sat: ");
        Serial.println(sat);

        DELAY(1000);
    }
}

void setup()
{
    Serial.begin(460800);
    delay(2000);

    i2c3.begin();

    pinMode(PA0, OUTPUT);

    i2cMutex = xSemaphoreCreateMutex();

    // m10q (0x42)
    if (m10q.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
    {
        m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    }

    // bme(0x76)
    if (!bme.begin(0x76, &i2c3))
    {
        Serial.println("Could not find a valid BME280 sensor");
    }

    // icm42688
    delay(500);
    uint16_t status = icm.begin();
    if (status > 0)
    {
        icm.setAccelFS(ICM42688::gpm8);
        icm.setGyroFS(ICM42688::dps500);
        icm.setAccelODR(ICM42688::odr12_5);
        icm.setGyroODR(ICM42688::odr12_5);
    }
    Serial.print("Status: ");
    Serial.println(status);

    validsd0 = sd0.begin(SdSpiConfig(PIN_SPI_CS_SD, SHARED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_1));
    if (validsd0)
    {
    //   make_new_filename(sd0, sd0_filename, F_NAME, F_EXT);
      open_for_append(sd0, sd0_file, sd0_filename);
    }
    
    Serial.println("hello");

    xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
    xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
    //xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
    xTaskCreate(print_data, "", 2048, nullptr, 2, nullptr);
    xTaskCreate(buzz, "", 2048, nullptr, 2, nullptr);
    vTaskStartScheduler();
}

void loop()
{
    DELAY(1);
}