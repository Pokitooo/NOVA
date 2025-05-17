#include <RadioLib.h>

// Create radio instance: NSS, DIO1, RESET, BUSY
SX1262 lora = new Module(PA4, PB1, PB2, PB0);

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Initializing LoRa...");

  int state = lora.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Init failed: ");
    Serial.println(state);
    while (true);
  }

  // Set frequency (adjust to your region: 868.0 or 915.0 MHz)
  lora.setFrequency(915.0);

  // Set spreading factor (6–12): Higher = longer range, lower speed
  lora.setSpreadingFactor(10);  // SF10 = good balance

  // Set bandwidth (kHz): 62.5, 125, 250, 500
  lora.setBandwidth(125.0);  // Most commonly used

  // Set coding rate: 5 = 4/5, 8 = 4/8
  lora.setCodingRate(5);  // CR 4/5

  // Set transmit power (dBm): max depends on module (SX1262 = ~22 dBm)
  lora.setOutputPower(20);  // Max reliable without violating regs

  // Optional: Set preamble length
  lora.setPreambleLength(8);  // Default is 8

  // Finished config
  Serial.println("LoRa configuration complete!");
}

void loop() {
  String message = "Hello from E22!";
  Serial.print("Sending: ");
  Serial.println(message);

  int state = lora.transmit(message);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Transmission successful!");
  } else {
    Serial.print("Transmission failed, code ");
    Serial.println(state);
  }

  delay(2000);
}
