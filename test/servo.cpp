/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/
#include <Arduino.h>
#include <Servo.h>
#include "Arduino_Extended.h"
#include "lib_xcore"
#include "File_Utility.h"

#include <Wire.h>
#include <SPI.h>

#include <TinyGPS++.h>
#include <ICM42688.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "nova_peripheral_def.h"
#include "nova_pin_def.h"
#include "nova_state_def.h"

// // i2c
TwoWire i2c3(PIN_SDA, PIN_SCL);
Adafruit_BME280 bme1;

// // UARTS
HardwareSerial gnssSerial(PIN_RX, PIN_TX);
TinyGPSPlus lc86;

// // SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// // SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);
using sd_t = SdFat32;
using file_t = File32;
FsUtil<sd_t, file_t> sd_util;

// // LoRa
// SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

// SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// ICM
// ICM42688 icm(spi1, PIN_NSS_ICM);

// Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

struct SERVO
{
    uint32_t pin;
    SERVO(uint32_t pin) : pin(pin)
    {
        pinMode(pin, OUTPUT);
    }

    void write(int angle)
    {
        const int cpw = map(angle, 0, 180, 500, 2500);
        for (size_t i = 0; i < max(5, (angle + 17) / 18); ++i)
        {
            digitalWrite(pin, HIGH);
            delayMicroseconds(cpw);
            digitalWrite(pin, LOW);
            delayMicroseconds(20000);
        }
    }
};

SERVO servo_a(servoPinA);

void setup()
{
}

void loop()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n'); // read line until Enter key
        input.trim();                                // remove spaces/newline

        if (input.length() > 0)
        {
            int newAngle = input.toInt(); // convert to integer

            if (newAngle >= 0 && newAngle <= 180)
            {
                servo_a.write(newAngle);
                Serial.print("Servo moved to: ");
                Serial.println(newAngle);
            }
            else
            {
                Serial.println("Invalid! Enter 0 - 180.");
            }
        }
    }
    // digitalWrite(servoPinB, 1);
    // delay(5000);
    // digitalWrite(servoPinB, 0);
    // delay(5000);
    // servo_a.write(90);
    // delay(2000);
    // servo_a.write(0);
    // delay(2000);
}
// servo_a.write(180);
// delay(2000);
// servo_a.write(0);
// delay(2000);
// delay(1000);.
// servo_a.write(180);
// delay(1000);
// servo_a.write(90);
// delay(1000);
// while(Serial.)
// }
