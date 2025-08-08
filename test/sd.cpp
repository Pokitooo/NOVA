#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include "File_Utility.h"
#define SPI_SPEED_SD_MHZ (2)

#define THIS_FILE_PREFIX "NOVA_LOGGER_"
#define THIS_FILE_EXTENSION "CSV"

// SPI
#define PIN_SPI_MOSI1 PA7
#define PIN_SPI_MISO1 PA6
#define PIN_SPI_SCK1 PA5

// NSS
#define PIN_NSS_SD PB9

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);

// SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);
using sd_t = SdFat32;
using file_t = File32;
FsUtil<sd_t, file_t> sd_util;

bool sdstate;

String constructed_data;
uint64_t data = 1;

template <typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance)
{
    sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
    sd_util_instance.template open_one<FsMode::WRITE>();
}

void setup()
{
    Serial.begin(460800);
    delay(2000);

    sdstate = sd_util.sd().begin(sd_config);
    if (sdstate)
    {
        init_storage(sd_util);
    }
    Serial.printf("SD CARD: %s\n", sdstate ? "SUCCESS" : "FAILED");
    if (!sdstate)
        while (true)
            ;
}

void loop()
{
    sd_util.file().println(data);
    sd_util.flush_one();
    Serial.println("Data written and flushed.");
    Serial.println(data);
    ++data;
    delay(1000);

}