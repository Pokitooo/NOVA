#include <Arduino.h>
#include <SPI.h>
#include "ICM42688.h"

constexpr PinName PIN_SPI_MOSI1 = PA_7;
constexpr PinName PIN_SPI_MISO1 = PA_6;
constexpr PinName PIN_SPI_SCK1 = PA_5;
constexpr PinName PIN_NSS_ICM = PA_15;

SPIClass SPI_1(pinNametoDigitalPin(PIN_SPI_MOSI1), pinNametoDigitalPin(PIN_SPI_MISO1), pinNametoDigitalPin(PIN_SPI_SCK1));
ICM42688 icm(SPI_1, pinNametoDigitalPin(PIN_NSS_ICM));

void setup()
{
    Serial.begin(460800);
    delay(2000);

    delay(500);
    uint16_t status = icm.begin();
    if (status > 0)
    {
        icm.setAccelFS(ICM42688::gpm8);
        icm.setGyroFS(ICM42688::dps2000);
        icm.setAccelODR(ICM42688::odr12_5);
        icm.setGyroODR(ICM42688::odr12_5);
        Serial.println("ICM Success");
    }
    Serial.print("Status: ");
    Serial.println(status);
}

void loop()
{
    icm.getAGT();

    // acceleration
    float ax = icm.accX();
    float ay = icm.accY();
    float az = -icm.accZ();

    // gyroscope
    float gyro_x = icm.gyrX();
    float gyro_y = icm.gyrY();
    float gyro_z = icm.gyrZ();

    float mag = sqrt(ax * ax + ay * ay + az * az);

    Serial.print("X: ");
    Serial.print(ax);
    Serial.print("  Y: ");
    Serial.print(ay);
    Serial.print("  Z: ");
    Serial.print(az);
    Serial.print("  | Magnitude: ");
    Serial.println(mag);

    Serial.print(" || Gyro [dps]: X = ");
    Serial.print(gyro_x, 3);
    Serial.print(" | Y = ");
    Serial.print(gyro_y, 3);
    Serial.print(" | Z = ");
    Serial.println(gyro_z, 3);

    delay(1000);
}