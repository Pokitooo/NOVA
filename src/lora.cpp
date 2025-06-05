#include <Arduino_Extended.h>
#include <SdFat.h>
#include <RadioLib.h>

#define USE_FREERTOS 1
#if defined(USE_FREERTOS) && USE_FREERTOS
#define DELAY_MS(MS) vTaskDelay(pdMS_TO_TICKS(MS))
#else
#define DELAY_MS(MS) delay(MS)
#endif

// NSS
constexpr uint32_t NSS_SD = PB9;

// SPI
SPIClass spi1(PA7, PA6, PA5);

// SD Config
SdSpiConfig sd_config(NSS_SD, SHARED_SPI,SD_SCK_MHZ(25), &spi1);

// SD
SdExFat sd = {};
ExFile file = {};

// LoRa data
volatile bool tx_flag = false;
String s;

// LoRa
SPISettings lora_spi_settings(18'000'000, MSBFIRST, SPI_MODE0);

// LoRa Parameters
constexpr struct {
    float center_freq = 922.500'000f; // MHz
    float bandwidth = 125.f; // kHz
    uint8_t spreading_factor = 12; // SF: 6 to 12
    uint8_t coding_rate = 8; // CR: 5 to 8
    uint8_t sync_word = 0x12; // Private SX1262
    int8_t power = 22; // up to 22 dBm for SX1262
    uint16_t preamble_length = 16;
} params;

// SX1262 pin connections (adjust pins for your board)
#define LORA_DIO1 PB0
#define LORA_NSS  PA4
#define LORA_BUSY PB1
#define LORA_NRST  PB2

// Initialize SX1262 module
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

void set_tx_flag() {
    tx_flag = true;
    digitalToggle(PB5);
}

void setup() {
    static bool state;
    pinMode(PB5, OUTPUT);
    Serial.begin();
    spi1.begin();

    delay(2000);
    Serial.println("Hi!");

    state = sd.begin(sd_config);
    Serial.printf("SD CARD: %s\n", state ? "SUCCESS" : "FAILED");
    if (!state) while (true);

    int16_t ls = lora.begin(params.center_freq,
                            params.bandwidth,
                            params.spreading_factor,
                            params.coding_rate,
                            params.sync_word,
                            params.power,
                            params.preamble_length,
                            0,
                            false);
    state = state || lora.explicitHeader();
    state = state || lora.setCRC(true);
    state = state || lora.autoLDRO();
    attachInterrupt(LORA_DIO1, set_tx_flag, CHANGE);

    if (ls == RADIOLIB_ERR_NONE) {
        Serial.println("SX1262 initialized successfully!");
    } else {
        Serial.print("Initialization failed! Error: ");
        Serial.println(ls);
        while (true);
    }

    s.reserve(256);
}

void loop() {
    // ส่งทุกๆ 5 วิ ขั้นต่ำ, recommend 10 วิ
    static xcore::nonblocking_delay<uint32_t> nb(5'000, millis);
    static int16_t state;
    static uint32_t t0, t;

    if (tx_flag) {
        tx_flag = false;
        t = millis();

        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Transmission successful!");
            Serial.printf("Used %d ms\n", t - t0);
        } else {
            Serial.print("Transmission failed! Error: ");
            Serial.println(state);
        }

        lora.finishTransmit();
    } else if (nb) {
        Serial.println("Transmitting packet...");
        s = "NOVA,13.000000,100.000000,25490,13.000000,100.000000,25490\n";
        state = lora.startTransmit(s.c_str());
        t0 = millis();
    }
}