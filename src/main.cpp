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

// Software filters
constexpr size_t FILTER_ORDER = 4;
constexpr double dt_base = 0.1;
constexpr double covariance = 0.01;
constexpr double alpha = 0.;
constexpr double beta = 0.;
constexpr double G = 9.81;

struct software_filters
{
  vt::kf_pos<FILTER_ORDER> bme_pres{dt_base, covariance, alpha, beta};

  struct
  {
    vec3_u<vt::kf_pos<4>> acc{dt_base, covariance, alpha, beta};
    vec3_u<vt::kf_pos<4>> gyro{dt_base, covariance, alpha, beta};
  } imu_1;

  vt::kf_pos<FILTER_ORDER> altitude{dt_base, covariance, alpha, beta};
  vt::kf_acc<FILTER_ORDER> acceleration{dt_base, covariance, alpha, beta};
} filters;

struct bme_ref_t
{
  Adafruit_BME280 &bmef;
  float &temp;
  float &pres;
  float &alt;
  vt::kf_pos<FILTER_ORDER> &kf;

  bme_ref_t(Adafruit_BME280 &t_bmef, float &t_temp, float &t_pres, float &t_alt, vt::kf_pos<FILTER_ORDER> &t_kf)
      : bmef{t_bmef}, temp{t_temp}, pres{t_pres}, alt{t_alt}, kf{t_kf} {}
};

bme_ref_t bme_ref = {bme1, data.temp, data.press, data.altitude, filters.bme_pres};

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

extern void read_m10q(void *);

extern void read_gnss(void *);

extern void readBme(bme_ref_t *bme);

extern void read_bme(void *);

extern void read_icm(void *);

extern void synchronize_kf(void *);

extern void read_current(void *);

extern void construct_data(void *);

extern void send_data(void *);

extern void save_data(void *);

extern void accept_command(void *);

extern void print_data(void *);

extern void fsm_eval(void *);

extern void buzz(void *);

extern void pyro(void *);

extern void set_rxflag();

extern void buzzer_control(on_off_timer::interval_params *intervals_ms);

template <typename SdType, typename FileType>
extern void init_storage(FsUtil<SdType, FileType> &sd_util_instance);

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

  // SD
  sdstate = sd_util.sd().begin(sd_config);
  if (sdstate)
  {
    init_storage(sd_util);
  }
  Serial.printf("SD CARD: %s\n", sdstate ? "SUCCESS" : "FAILED");
  if (!sdstate)
    while (true)
      ;

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

  // bme280(0x76)
  float gnd = 0.f;

  if (bme1.begin(0x76, &i2c3))
  {
    bme1.SAMPLING_X16;
    for (size_t i = 0; i < 20; ++i)
    {
      readBme(&bme_ref);
    }
    gnd += data.altitude;
  }
  ground_truth.altitude_offset = gnd;

  // ADC
  analogReadResolution(ADC_BITS);

  // Scheduler
  xTaskCreate(read_m10q, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_bme, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(read_icm, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(synchronize_kf, "", 1024, nullptr, 2, nullptr);
  // xTaskCreate(read_current, "", 2048, nullptr, 2, nullptr);

  xTaskCreate(construct_data, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(send_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(save_data, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(accept_command, "", 2048, nullptr, 2, nullptr);
  xTaskCreate(print_data, "", 1024, nullptr, 2, nullptr);

  xTaskCreate(fsm_eval, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(pyro, "", 1024, nullptr, 2, nullptr);
  xTaskCreate(buzz, "", 1024, nullptr, 2, nullptr);
  vTaskStartScheduler();

  // Low power mode
  LowPower.begin();
}

void loop() { DELAY(1); }

void pyro(void *)
{
  auto pyro_cutoff = []
  {
    static smart_delay sd_a(nova::config::PYRO_ACTIVATE_INTERVAL, millis);
    static smart_delay sd_b(nova::config::PYRO_ACTIVATE_INTERVAL, millis);
    static bool flag_a = false, flag_b = false;

    if (data.pyro_a == nova::pyro_state_t::FIRING)
    {
      if (!flag_a)
      {
        sd_a.reset();
        flag_a = true;
      }
      else
      {
        sd_a([]
             {
          servo.write(180);
          data.pyro_a = nova::pyro_state_t::FIRED;
          flag_a = false; });
      }
    }

    if (data.pyro_b == nova::pyro_state_t::FIRING)
    {
      if (!flag_b)
      {
        sd_b.reset();
        flag_b = true;
      }
      else
      {
        sd_b([]
             {
          gpio_write << io_function::pull_low(pyroB);
          data.pyro_b = nova::pyro_state_t::FIRED;
          flag_b = false; });
      }
    }
  };

  dispatcher << task_type(pyro_cutoff, 0);
  dispatcher.reset();

  // run dispatcher forever
  for (;;)
  {
    dispatcher();
    DELAY(1); // yield ~1 tick; adjust if you want tighter timing
  }
}

void buzz(void *)
{
  for (;;)
  {
    buzzer_control(&buzzer_intervals);
  }
}

void buzzer_control(on_off_timer::interval_params *intervals_ms)
{
  for (;;)
  {
    static time_type prev_on = intervals_ms->t_on;
    static time_type prev_off = intervals_ms->t_off;
    static nova::state_t prev_state = nova::state_t::STARTUP;

    // On-off timer
    static on_off_timer timer(intervals_ms->t_on, intervals_ms->t_off, millis);
    // digitalToggle(buzzerPin);
    digitalToggle(ledPin);
    DELAY(500);
  }
}

void read_m10q(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      if (m10q.getPVT())
      {
        data.timestamp = m10q.getUnixEpoch(nova::config::UBLOX_CUSTOM_MAX_WAIT);
        data.gps_latitude = static_cast<double>(m10q.getLatitude(nova::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_longitude = static_cast<double>(m10q.getLongitude(nova::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_altitude = static_cast<float>(m10q.getAltitudeMSL(nova::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
      }
      xSemaphoreGive(i2cMutex);
    }
    DELAY(500);
  }
}

void read_gnss(void *)
{
  for (;;)
  {
    while (gnssSerial.available())
    {
      lc86.encode(gnssSerial.read());
    }

    if (lc86.location.isUpdated())
    {
      data.timestamp = lc86.time.value();
      data.gps_latitude = lc86.location.lat();
      data.gps_longitude = lc86.location.lng();
      data.gps_altitude = lc86.altitude.meters();
    }
    DELAY(500);
  }
}

void read_bme(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
      readBme(&bme_ref);
      xSemaphoreGive(i2cMutex);
    }
    DELAY(200);
  }
}

void readBme(bme_ref_t *bme)
{
  static uint32_t t_prev = millis();

  if (const float t = bme1.readTemperature(); t > 0.)
  {
    bme->temp = t;
  }

  if (const float p = bme1.readPressure() / 100.0F; p > 0.)
  {
    bme->kf.update_dt(millis() - t_prev);
    bme->kf.kf.predict().update(p);
    bme->pres = static_cast<float>(bme->kf.kf.state);
  }

  bme->alt = pressure_altitude(data.press);

  t_prev = millis();
}

void read_icm(void *)
{
  for (;;)
  {
    static uint32_t t_prev = millis();

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      icm.getAGT();
      xSemaphoreGive(spiMutex);
    }

    data.imu.acc.x = icm.accX() * G;
    data.imu.acc.y = icm.accY() * G;
    data.imu.acc.z = icm.accZ() * G;
    data.imu.gyro.x = icm.gyrX();
    data.imu.gyro.y = icm.gyrY();
    data.imu.gyro.z = icm.gyrZ();

    for (size_t i = 0; i < 3; ++i)
    {
      const uint32_t dt = millis() - t_prev;
      filters.imu_1.acc.values[i].update_dt(dt);
      filters.imu_1.gyro.values[i].update_dt(dt);
      filters.imu_1.acc.values[i].kf.predict().update(data.imu.acc.values[i]);
      filters.imu_1.gyro.values[i].kf.predict().update(data.imu.gyro.values[i]);
      data.imu.acc.values[i] = filters.imu_1.acc.values[i].kf.state;
      data.imu.gyro.values[i] = filters.imu_1.gyro.values[i].kf.state;
    }

    t_prev = millis();
    DELAY(200);
  }
}

void synchronize_kf(void *)
{
  for (;;)
  {
    static uint32_t t_prev = millis();

    const double total_acc = algorithm::root_sum_square(data.imu.acc.x, data.imu.acc.y, data.imu.acc.z);

    filters.altitude.update_dt(millis() - t_prev);
    filters.acceleration.update_dt(millis() - t_prev);
    filters.altitude.kf.predict().update(data.altitude);
    filters.acceleration.kf.predict().update(total_acc - G);

    t_prev = millis();
    DELAY(25);
  }
}

void read_current(void *)
{
  for (;;)
  {
    voltageServo = analogRead(digitalPinToAnalogInput(VOUT_Servo)) / ADC_DIVIDER * VREF;
  }
}

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

template <typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance)
{
  sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
  sd_util_instance.template open_one<FsMode::WRITE>();
}

void save_data(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      sd_util.file() << constructed_data;
      sd_util.flush_one();
      Serial.println("Data written and flushed.");
      xSemaphoreGive(spiMutex);
    }
    DELAY(log_interval);
  }
}

void accept_command(void *)
{
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  lora.startReceive();
  xSemaphoreGive(spiMutex);

  for (;;)
  {
    if (rx_flag)
    {                  // only run when TRUE
      rx_flag = false; // clear first (avoid missing quick second packet)

      if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
      {
        String rx_message;
        rx_message.reserve(64);
        int state = lora.readData(rx_message);
        lora.startReceive(); // re-arm
        xSemaphoreGive(spiMutex);

        if (state == RADIOLIB_ERR_NONE)
        {
          handle_command(rx_message);
        }
      }
    }
    else
    { DELAY(100); }  
    
    // // Serial
    // if (Serial.available())
    // {
    //   String serial_input = Serial.readStringUntil('\n'); // Read serial input line
    //   if (serial_input.length() > 0)
    //   {
    //     handle_command(serial_input); // Handle command from Serial input
    //     digitalToggle(ledPin);
    //   }
    // }
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
    readBme(&bme_ref);
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

void fsm_eval(void *)
{
  for (;;)
  {
    static bool state_satisfaction = false;
    int32_t static launched_time = 0;
    static algorithm::Sampler sampler[2];

    const double alt_x = filters.altitude.kf.state_vector[0] - ground_truth.altitude_offset;
    const double vel_x = filters.altitude.kf.state_vector[1];
    const double acc = filters.acceleration.kf.state_vector[2];

    switch (data.ps)
    {
    case nova::state_t::STARTUP:
    {
      // Next: always transfer
      data.ps = nova::state_t::IDLE_SAFE;
      break;
    }
    case nova::state_t::IDLE_SAFE:
    {
      //  <--- Next: wait for uplink --->
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL);
      break;
    }
    case nova::state_t::ARMED:
    {
      // <--- Next: wait for uplink --->
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ARMED_INTERVAL);

      if (launch_override)
      {
        data.ps = nova::state_t::PAD_PREOP;
        tx_interval = nova::config::TX_PAD_PREOP_INTERVAL;
        log_interval = nova::config::LOG_PAD_PREOP_INTERVAL;
      }

      break;
    }
    case nova::state_t::PAD_PREOP:
    {
      // !!!!! Next: DETECT launch !!!!!
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_PAD_PREOP_INTERVAL);

      static on_off_timer tim(nova::config::alg::LAUNCH_TON / 2, nova::config::alg::LAUNCH_TON / 2, millis);

      if (!state_satisfaction)
      {
        sampler[0].add(acc >= nova::config::alg::LAUNCH_ACC);
        sampler[1].add(acc >= nova::config::alg::LAUNCH_ACC);

        tim.on_rising([&]
                      {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

        tim.on_falling([&]
                       {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
      }

      state_satisfaction |= launch_override;

      if (state_satisfaction)
      {
        launched_time = millis();
        data.ps = nova::state_t::POWERED;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval = nova::config::TX_ASCEND_INTERVAL;
        log_interval = nova::config::LOG_ASCEND_INTERVAL;
      }

      break;
    }
    case nova::state_t::POWERED:
    {
      // !!!!! Next: DETECT motor burnout !!!!!
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ASCEND_INTERVAL);

      static on_off_timer tim(nova::config::alg::BURNOUT_TON / 2, nova::config::alg::BURNOUT_TON / 2, millis);

      if (!state_satisfaction)
      {
        sampler[0].add(acc < nova::config::alg::LAUNCH_ACC);
        sampler[1].add(acc < nova::config::alg::LAUNCH_ACC);

        tim.on_rising([&]
                      {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[0].reset(); });

        tim.on_falling([&]
                       {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MIN;
          }
          sampler[1].reset(); });
      }

      state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_BURNOUT_MAX;

      if (state_satisfaction)
      {
        data.ps = nova::state_t::COASTING;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
      }

      break;
    }
    case nova::state_t::COASTING:
    {
      // !!!!! Next: DETECT apogee !!!!!
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_ASCEND_INTERVAL);

      static on_off_timer tim(nova::config::alg::APOGEE_SLOW_TON / 2, nova::config::alg::APOGEE_SLOW_TON / 2, millis);

      if (!state_satisfaction)
      {
        sampler[0].add(vel_x <= nova::config::alg::APOGEE_VEL);
        sampler[1].add(vel_x <= nova::config::alg::APOGEE_VEL);

        tim.on_rising([&]
                      {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MIN;
          }
          sampler[0].reset(); });

        tim.on_falling([&]
                       {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MIN;
          }
          sampler[1].reset(); });

        state_satisfaction |= millis() - launched_time >= nova::config::TIME_TO_APOGEE_MAX;
      }

      if (state_satisfaction)
      {
        data.ps = nova::state_t::DROGUE_DEPLOY;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
      }

      break;
    }
    case nova::state_t::DROGUE_DEPLOY:
    {
      // Next: activate and always transfer
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

      static bool fired = false;

      if (!fired)
      {
        servo.write(180); // Deploy
        data.pyro_a = nova::pyro_state_t::FIRING;
        fired = true;
      }
      else if (data.pyro_a == nova::pyro_state_t::FIRED)
      {
        data.ps = nova::state_t::DROGUE_DESCEND;
      }

      break;
    }
    case nova::state_t::DROGUE_DESCEND:
    {
      // !!!!! Next: DETECT main deployment altitude !!!!!
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

      static on_off_timer tim(nova::config::alg::MAIN_DEPLOYMENT_TON / 2, nova::config::alg::MAIN_DEPLOYMENT_TON / 2, millis);

      if (!state_satisfaction)
      {
        sampler[0].add(alt_x <= nova::config::alg::MAIN_ALTITUDE);
        sampler[1].add(alt_x <= nova::config::alg::MAIN_ALTITUDE);

        tim.on_rising([&]
                      {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

        tim.on_falling([&]
                       {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
      }

      if (state_satisfaction)
      {
        data.ps = nova::state_t::MAIN_DEPLOY;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval = nova::config::TX_DESCEND_INTERVAL;
        log_interval = nova::config::LOG_DESCEND_INTERVAL;
      }

      break;
    }
    case nova::state_t::MAIN_DEPLOY:
    {
      // Next: activate and always transfer
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

      static bool fired = false;

      if (!fired)
      {
        gpio_write << io_function::pull_high(pyroB); // Change to solenoid
        data.pyro_b = nova::pyro_state_t::FIRING;
        fired = true;
      }
      else if (data.pyro_b == nova::pyro_state_t::FIRED)
      {
        data.ps = nova::state_t::MAIN_DESCEND;
      }

      break;
    }
    case nova::state_t::MAIN_DESCEND:
    {
      // !!!!! Next: DETECT landing !!!!!
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

      static on_off_timer tim(nova::config::alg::LANDING_TON / 2, nova::config::alg::LANDING_TON / 2, millis);

      if (!state_satisfaction)
      {
        const bool stable = algorithm::is_zero(vel_x, 0.5);
        sampler[0].add(stable);
        sampler[1].add(stable);

        tim.on_rising([&]
                      {
          if (sampler[0].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[0].reset(); });

        tim.on_falling([&]
                       {
          if (sampler[1].vote<1, 1>()) {
            state_satisfaction |= true;
          }
          sampler[1].reset(); });
      }

      if (state_satisfaction)
      {
        data.ps = nova::state_t::LANDED;
        state_satisfaction = false;
        sampler[0].reset();
        sampler[1].reset();
        tx_interval = nova::config::TX_IDLE_INTERVAL;
        log_interval = nova::config::LOG_IDLE_INTERVAL;
      }

      break;
    }
    case nova::state_t::LANDED:
    {
      // <--- Next: wait for uplink --->
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_DESCEND_INTERVAL);

      break;
    }
    case nova::state_t::RECOVERED_SAFE:
    {
      // Sink state (requires reboot)
      buzzer_intervals.t_off = nova::config::BUZZER_OFF_INTERVAL(nova::config::BUZZER_IDLE_INTERVAL);

      do_nothing();
      break;
    }
    default:
      __builtin_unreachable();
    }
    DELAY(25);
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
