#include <Arduino_Extended.h>
#include <Wire.h>
#include <SPI.h>
#include <vt_bme280>
#include <ICM42688.h>

#define USE_FREERTOS 0

#if defined(USE_FREERTOS) && USE_FREERTOS
#define DELAY_MS(MS) vTaskDelay(pdMS_TO_TICKS(MS))
#else
#define DELAY_MS(MS) delay(MS)
#endif

// NSS
constexpr uint32_t NSS_ICM42688 = PA15;
constexpr uint32_t NSS_SD = PB9;

// SPI
SPIClass spi1(PA7, PA6, PA5);

// I2C
TwoWire i2c3(PB8, PA8);

// Sensors
vt::bme280_t bme280;
ICM42688 imu(spi1, NSS_ICM42688, 24'000'000);

// Status
int status;

void setup() {
    Serial.begin();
    delay(1000);
    Serial.println("Hello!");
    status = bme280.begin(0x76, &i2c3);
    Serial.println(status);

    status = imu.begin();
    Serial.println(status);
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1) {
        }
    }
    imu.setAccelFS(ICM42688::gpm16);
    imu.setGyroFS(ICM42688::dps2000);
    imu.setAccelODR(ICM42688::odr12_5);
    imu.setGyroODR(ICM42688::odr12_5);
    Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() {
    // read the sensor
    imu.getAGT();

    // display the data
    Serial.print(imu.accX(), 6);
    Serial.print("\t");
    Serial.print(imu.accY(), 6);
    Serial.print("\t");
    Serial.print(imu.accZ(), 6);
    Serial.print("\t");
    Serial.print(imu.gyrX(), 6);
    Serial.print("\t");
    Serial.print(imu.gyrY(), 6);
    Serial.print("\t");
    Serial.print(imu.gyrZ(), 6);
    Serial.print("\t");
    Serial.println(imu.temp(), 6);

    float temp = bme280.read_temperature_c();
    float hum = bme280.read_humidity();
    float pres = bme280.read_pressure();
    Serial.println(temp);
    Serial.println(hum);
    Serial.println(pres);
    Serial.println("--- --- ---");
    delay(1000);
}