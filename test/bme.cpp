#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// i2c
constexpr uint16_t PIN_SDA = PB8;
constexpr uint16_t PIN_SCL = PA8;

Adafruit_BME280 bme;
TwoWire i2c3(PIN_SDA, PIN_SCL);

#define BME280_I2C_ADDRESS 0x76  // Change to 0x77 if needed
#define SEA_LEVEL_PRESSURE_HPA 1013.25  // Adjust for your local sea-level pressure

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bme.begin(BME280_I2C_ADDRESS, &i2c3)) {
    Serial.println("BME280 not found. Check wiring!");
    while (1);
  }
}

void loop() {
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA); // meters

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.println("------------------");

  delay(1000);
}
