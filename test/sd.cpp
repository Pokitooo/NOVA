#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "psg_4_sd_tools.h"

#define SPI_SPEED_SD_MHZ (2)

constexpr PinName PIN_SPI_MOSI1 = PA_7;
constexpr PinName PIN_SPI_MISO1 = PA_6;
constexpr PinName PIN_SPI_SCK1 = PA_5;
constexpr PinName PIN_SPI_CS_SD = PB_9;

SPIClass SPI_1(pinNametoDigitalPin(PIN_SPI_MOSI1), pinNametoDigitalPin(PIN_SPI_MISO1), pinNametoDigitalPin(PIN_SPI_SCK1));
SdSpiConfig sd0_cfg(pinNametoDigitalPin(PIN_SPI_CS_SD), DEDICATED_SPI, SD_SCK_MHZ(SPI_SPEED_SD_MHZ), &SPI_1);
SdFat32 sd0;
File32 sd0_file;
String sd0_filename;

void setup() {
    Serial.begin(460800);
    delay(2000);

    retry:
    delay(500);
    Serial.println("Begin SD!");
    SPI_1.begin();
    bool status = sd0.begin(sd0_cfg);

    if (status)
        Serial.println("Sucess!");
    else {
        Serial.println("Failed!");
        goto retry;
    }

    make_new_filename(sd0, sd0_filename, "mcu0_data_test_", ".csv");
    list_files(sd0);
    open_for_append(sd0, sd0_file, sd0_filename);
    sd0_file.println("DATATADTATDASDHJAGSFHJGHJFGKJASHFAJSFH");
    sd0_file.close();
    read_file_to_stream(sd0, sd0_filename);
}

void loop() {
    Serial.println("Loop!");
    delay(5000);
}