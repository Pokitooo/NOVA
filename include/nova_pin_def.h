#ifndef NOVA_PIN_DEF_H
#define NOVA_PIN_DEF_H

#include <Arduino.h>
// Pins defination

// SPI
#define PIN_SPI_MOSI1 PA7
#define PIN_SPI_MISO1 PA6
#define PIN_SPI_SCK1 PA5

// NSS
#define PIN_NSS_ICM PA15
#define PIN_NSS_SD PB9

// LORA
#define LORA_DIO1 PB0
#define LORA_NSS PA4
#define LORA_BUSY PB1
#define LORA_NRST PB2

// UARTS
constexpr uint16_t PIN_RX = PB7;
constexpr uint16_t PIN_TX = PB6;

// i2c
constexpr uint16_t PIN_SDA = PB8;
constexpr uint16_t PIN_SCL = PA8;

// GPIO
constexpr uint16_t buzzerPin = PA0;
constexpr uint16_t ledPin = PB5;

// CURRENT ADC
constexpr uint16_t VOUT_EXT = PB1;
constexpr uint16_t VOUT_Servo = PA3;

#endif