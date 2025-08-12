#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>
#include "vt_linalg"
#include "vt_kalman"
#include "Arduino_Extended.h"
#include <STM32LowPower.h>
#include "File_Utility.h"

#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_v3.h>
#include <TinyGPS++.h>
#include <ICM42688.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

#include "SdFat.h"
#include "RadioLib.h"

#include "nova_peripheral_def.h"
#include "nova_pin_def.h"
#include "nova_state_def.h"

// Device specific
#define THIS_FILE_PREFIX "NOVA_LOGGER_"
#define THIS_FILE_EXTENSION "CSV"

using time_type = uint32_t;
using smart_delay = vt::smart_delay<time_type>;
using on_off_timer = vt::on_off_timer<time_type>;
using task_type = vt::task_t<vt::smart_delay, time_type>;

template <size_t N>
using dispatcher_type = vt::task_dispatcher<N, vt::smart_delay, time_type>;

on_off_timer::interval_params buzzer_intervals(nova::config::BUZZER_ON_INTERVAL,
                                               nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL));
bool enable_buzzer = true;

// i2c
TwoWire i2c3(PIN_SDA, PIN_SCL);
Adafruit_BME280 bme1;
SFE_UBLOX_GNSS m10q;
SemaphoreHandle_t i2cMutex;

// UARTS
HardwareSerial gnssSerial(PIN_RX, PIN_TX);
TinyGPSPlus lc86;

// SPI
SPIClass spi1(PIN_SPI_MOSI1, PIN_SPI_MISO1, PIN_SPI_SCK1);
SemaphoreHandle_t spiMutex;

// SD
SdSpiConfig sd_config(PIN_NSS_SD, SHARED_SPI, SD_SCK_MHZ(2), &spi1);
using sd_t = SdFat32;
using file_t = File32;
FsUtil<sd_t, file_t> sd_util;

// LoRa
volatile bool tx_flag = false;
volatile bool rx_flag = false;
String s;

SPISettings lora_spi_settings(4'000'000, MSBFIRST, SPI_MODE0);

constexpr struct
{
  float center_freq = 920.800'000f; // MHz
  float bandwidth = 125.f;          // kHz
  uint8_t spreading_factor = 9;     // SF: 6 to 12
  uint8_t coding_rate = 6;          // CR: 5 to 8
  uint8_t sync_word = 0x12;         // Private SX1262
  int8_t power = 10;                // up to 22 dBm for SX1262
  uint16_t preamble_length = 10;
} params;

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

// ICM
ICM42688 icm(spi1, PIN_NSS_ICM);

// ADC
constexpr size_t ADC_BITS(12);
constexpr float ADC_DIVIDER = ((1 << ADC_BITS) - 1);
constexpr float VREF = 3300;
float voltageServo, volatgeEXT = 0.F;

// Servo
Servo servo; // Create servo object

// DATA
struct Data
{
  // 40 bits
  uint32_t timestamp;
  uint8_t counter;

  nova::state_t ps;
  nova::pyro_state_t pyro_a{};
  nova::pyro_state_t pyro_b{};

  // 160 bits
  double gps_latitude;
  double gps_longitude;
  float gps_altitude;

  // 96 bits
  float altitude;
  float temp;
  float press;

  // 384 bits
  struct
  {
    vec3_u<double> acc;
    vec3_u<double> gyro;
  } imu;

  uint8_t last_ack{};
  uint8_t last_nack{};

} data;

// State
bool sdstate;

// Software control
dispatcher_type<32> dispatcher;
bool launch_override = false;
struct
{
  float altitude_offset{};
} ground_truth;

// Communication data
String constructed_data;
time_type tx_interval = nova::config::TX_IDLE_INTERVAL;
time_type log_interval = nova::config::LOG_IDLE_INTERVAL;

HardwareTimer timer_buz(TIM3);

// variables
volatile bool wake_flag = false;

extern void construct_data(void *);

extern void send_data(void *);

extern void save_data(void *);

extern void accept_command(void *);

extern void print_data(void *);

// extern void fsm_eval(void *);

// extern void buzz(void *);

// extern void pyro(void *);

extern void set_rxflag();

extern void handle_command(String rx_message);

void setup()
{
  Serial.begin(460800);
  delay(2000);

  i2c3.begin();
  i2c3.setClock(300000u);

  spi1.begin();

  // GPIO
  pinMode(to_digital(buzzerPin), OUTPUT); // BUZZER
  digitalWrite(buzzerPin, 1);
  delay(100);
  digitalWrite(buzzerPin, 0);

  timer_buz.setOverflow(nova::config::BUZZER_ON_INTERVAL * 1000, MICROSEC_FORMAT);
  timer_buz.attachInterrupt([]
                            {
  gpio_write << io_function::pull_low(buzzerPin);
  timer_buz.pause(); });
  timer_buz.pause();
  pinMode(to_digital(ledPin), OUTPUT); // LED

  servo.attach(servoPin); // Servo

  // variable
  static bool state;

  // SPI
  spiMutex = xSemaphoreCreateMutex();

  // LoRa
  uint16_t lora_state = lora.begin(params.center_freq,
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
  lora.setPacketReceivedAction(set_rxflag);
  lora.startReceive();

  if (lora_state == RADIOLIB_ERR_NONE)
  {
    Serial.println("SX1262 initialized successfully!");
  }
  else
  {
    Serial.print("Initialization failed! Error: ");
    Serial.println(lora_state);
    while (true)
      ;
  }

  s.reserve(256);

  // icm42688
  uint16_t status = icm.begin();
  if (status > 0)
  {
    Serial.println("ICM Success");
  }
  Serial.print("ICM Status: ");
  Serial.println(status);

  // lc86g UARTS
  gnssSerial.begin(115200);

  // i2c
  i2cMutex = xSemaphoreCreateMutex();

  // m10q (0x42)
  if (m10q.begin(i2c3))
  {
    m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, nova::config::UBLOX_CUSTOM_MAX_WAIT);
    m10q.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, nova::config::UBLOX_CUSTOM_MAX_WAIT);
    m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, nova::config::UBLOX_CUSTOM_MAX_WAIT);
    m10q.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, nova::config::UBLOX_CUSTOM_MAX_WAIT);
    Serial.println("gps Success");
  }


  // ADC
  analogReadResolution(ADC_BITS);

  // Scheduler
  // xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(synchronize_kf, "", 1024, nullptr, 2, nullptr);
  // xTaskCreate(read_current, "", 2048, nullptr, 2, nullptr);

  xTaskCreate(construct_data, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(send_data, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(save_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(accept_command, "", 2048, nullptr, 2, nullptr);
  // xTaskCreate(print_data, "", 1024, nullptr, 2, nullptr);

  vTaskStartScheduler();

  // Low power mode
  LowPower.begin();
}

void loop() { DELAY(1); }

void construct_data(void *)
{
  for (;;)
  {
    constructed_data = "";
    csv_stream_crlf(constructed_data)
        << "<1>"
        << data.counter
        << data.timestamp

        << nova::state_string(data.ps)
        << String(data.gps_latitude, 6)
        << String(data.gps_longitude, 6)
        << String(data.altitude, 4)

        << nova::pyro_state_string(data.pyro_a)
        << nova::pyro_state_string(data.pyro_b)

        << data.temp
        << data.press

        << data.imu.acc.x << data.imu.acc.y << data.imu.acc.z
        << data.imu.gyro.x << data.imu.gyro.y << data.imu.gyro.z

        << data.last_ack
        << data.last_nack;
    DELAY(25);
  }
}

void send_data(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      lora.transmit(constructed_data.c_str());
      xSemaphoreGive(spiMutex);
    }
    ++data.counter;
    DELAY(tx_interval);
  }
}

void accept_command(void *)
{
  // Arm RX once before the loop
  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
  {
    lora.startReceive();
    xSemaphoreGive(spiMutex);
  }

  for (;;)
  {
    // Wait (without holding the mutex) for ISR to set the flag
    if (!rx_flag)
    {
      DELAY(10);
      continue;
    }

    // Clear flag first to avoid missing a quick second packet
    rx_flag = false;

    // Now do the SPI/RadioLib work quickly under mutex
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      String rx_message;
      rx_message.reserve(64);

      int state = lora.readData(rx_message); // packet already arrived
      // Re-arm receiver for the next packet
      lora.startReceive();

      xSemaphoreGive(spiMutex);

      if (state == RADIOLIB_ERR_NONE)
      {
        handle_command(rx_message);
      }
      else
      {
        // optional: handle errors
      }
    }
  }
}

void handle_command(String rx_message)
{
  Serial.print("RX: ");
  Serial.println(rx_message);

  if (rx_message.substring(0, 4) != "cmd ")
  {
    // Return if cmd header is invalid
    ++data.last_nack;
    return;
  }

  String command = rx_message.substring(4);
  command.trim();

  ++data.last_ack;

  if (command == "ping" || command == "wake" || command == "on")
  {
    // <--- Maybe a wakeup command --->
  }
  else if (command == "arm")
  {
    // <--- Arming the rocket --->
    data.ps = nova::state_t::ARMED;
    tx_interval = nova::config::TX_ARMED_INTERVAL;
    log_interval = nova::config::LOG_ARMED_INTERVAL;
    data.pyro_a = nova::pyro_state_t::ARMED;
    data.pyro_b = nova::pyro_state_t::ARMED;
  }
  else if (command == "disarm")
  {
    data.ps = nova::state_t::IDLE_SAFE;
    tx_interval = nova::config::TX_IDLE_INTERVAL;
    log_interval = nova::config::LOG_IDLE_INTERVAL;
    data.pyro_a = nova::pyro_state_t::DISARMED;
    data.pyro_b = nova::pyro_state_t::DISARMED;
  }
  else if (command == "pad")
  {
    // <--- Prelaunch operation --->
    // Must be armed first!
    if (data.ps == nova::state_t::ARMED)
    {
      data.ps = nova::state_t::PAD_PREOP;
      tx_interval = nova::config::TX_PAD_PREOP_INTERVAL;
      log_interval = nova::config::LOG_PAD_PREOP_INTERVAL;
    }
  }
  else if (command == "manual-trigger-a")
  {
    if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
    {
      servo.write(180);
      data.pyro_a = nova::pyro_state_t::FIRING;
    }
  }
  else if (command == "manual-trigger-b")
  {
    if (data.ps != nova::state_t::IDLE_SAFE && data.ps != nova::state_t::RECOVERED_SAFE)
    {
      gpio_write << io_function::pull_high(pyroB);
      data.pyro_b = nova::pyro_state_t::FIRING;
    }
  }
  else if (command == "launch-override")
  {
    launch_override = true;
  }
  else if (command == "recover")
  {
    // <--- Rocket landing confirmed --->
    data.ps = nova::state_t::RECOVERED_SAFE;
    if (data.pyro_a != nova::pyro_state_t::FIRED)
      data.pyro_a = nova::pyro_state_t::DISARMED;
    if (data.pyro_b != nova::pyro_state_t::FIRED)
      data.pyro_b = nova::pyro_state_t::DISARMED;
  }
  else if (command == "zero")
  {
    // <--- Zero barometric altitude --->

    ground_truth.altitude_offset = data.press;
  }
  else if (command == "sleep")
  {
    // <--- Put the device into deep sleep mode (power saving) --->
    PINS_OFF;
    LowPower.deepSleep();
  }
  else if (command == "shutdown")
  {
    // <--- Shutdown the device --->
    PINS_OFF;

    if (sdstate)
    {
      sd_util.close_one();
    }

    LowPower.deepSleep();

    __NVIC_SystemReset();
  }
  else if (command == "reboot" || command == "restart")
  {
    // <--- Reboot/reset the device --->
    if (sdstate)
    {
      sd_util.close_one();
    }
    __NVIC_SystemReset();
  }
  else
  {
    // <--- Unknown command: send back nack --->
    ++data.last_nack;
    --data.last_ack;
  }
}


void print_data(void *)
{
  for (;;)
  {
    Serial.println("====== DATA ======");

    Serial.print("Timestamp: ");
    Serial.println(data.timestamp);

    Serial.print("Counter: ");
    Serial.println(data.counter);

    Serial.println("---- STATES ----");
    Serial.print("PS: ");
    Serial.println(nova::state_string(data.ps));
    Serial.print("Pyro A: ");
    Serial.println(nova::pyro_state_string(data.pyro_a));
    Serial.print("Pyro B: ");
    Serial.println(nova::pyro_state_string(data.pyro_b));

    Serial.println("---- GPS ----");
    Serial.print("Latitude: ");
    Serial.println(data.gps_latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(data.gps_longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(data.gps_altitude, 4);

    Serial.println("---- ENV ----");
    Serial.print("Temperature: ");
    Serial.println(data.temp, 2);
    Serial.print("Altitude: ");
    Serial.println(data.altitude, 2);
    Serial.print("Pressure: ");
    Serial.println(data.press, 2);

    Serial.println("---- IMU ACC ----");
    Serial.print("X: ");
    Serial.print(data.imu.acc.x, 3);
    Serial.print("  Y: ");
    Serial.print(data.imu.acc.y, 3);
    Serial.print("  Z: ");
    Serial.println(data.imu.acc.z, 3);

    Serial.println("---- IMU GYRO ----");
    Serial.print("X: ");
    Serial.print(data.imu.gyro.x, 3);
    Serial.print("  Y: ");
    Serial.print(data.imu.gyro.y, 3);
    Serial.print("  Z: ");
    Serial.println(data.imu.gyro.z, 3);

    Serial.println("---- COMM ----");
    Serial.print("Last ACK: ");
    Serial.println(data.last_ack);
    Serial.print("Last NACK: ");
    Serial.println(data.last_nack);

    Serial.println("==================\n");

    DELAY(1000);
  }
}

void set_rxflag()
{
  rx_flag = true;
}
