#ifndef NOVA_PIN_DEF_H
#define NOVA_PIN_DEF_H

#include <Arduino.h>
#include <Arduino_Extended.h>
// Pins defination

// SPI
constexpr uint16_t PIN_SPI_MOSI1 = PA7;
constexpr uint16_t PIN_SPI_MISO1 = PA6;
constexpr uint16_t PIN_SPI_SCK1 = PA5;

// NSS
constexpr uint16_t PIN_NSS_ICM = PA15;
constexpr uint16_t PIN_NSS_SD = PB9;

// LORA
constexpr uint16_t LORA_DIO1 = PB0;
constexpr uint16_t LORA_NSS = PA4;
constexpr uint16_t LORA_BUSY = PB1;
constexpr uint16_t LORA_NRST = PB2;

// UARTS
constexpr uint16_t PIN_RX = PB7;
constexpr uint16_t PIN_TX = PB6;

// i2c
constexpr uint16_t PIN_SDA = PB8;
constexpr uint16_t PIN_SCL = PA8;

// GPIO
constexpr PinName buzzerPin = PA_0;
constexpr PinName ledPin = PB_5;

constexpr uint16_t servoPin = PA1;
constexpr PinName pyroB = PC_13;

// CURRENT ADC
constexpr uint16_t VOUT_EXT = PB1;
constexpr uint16_t VOUT_Servo = PA3;

constexpr auto PINS_OFF = []
{
    gpio_write << io_function::pull_low(ledPin)
               << io_function::pull_low(buzzerPin);

};
#endif