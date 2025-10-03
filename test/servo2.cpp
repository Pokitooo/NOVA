// Only-servo sweep using STM32ServoList
// Change SERVO_A_PIN / SERVO_B_PIN to your servo signal pins (PWM-capable).

#include <Arduino.h>
#include <STM32Servo.h>
#include "nova_pin_def.h"



// Servo pulse range (µs). Use 1000–2000 for most servos; widen if your servo supports it.
static const int SERVO_MIN_US = 1000;
static const int SERVO_MAX_US = 2000;

// Sweep settings
static const int SWEEP_MIN_DEG = 0;
static const int SWEEP_MAX_DEG = 180;
static const int SWEEP_STEP    = 2;
static const int STEP_DELAY_MS = 15;

// ====== OBJECTS ======
// Both servos on the same HW timer via STM32ServoList.
STM32ServoList servoA(TIMER_SERVO);
STM32ServoList servoB(TIMER_SERVO);

void setup() {
  Serial.begin(115200);
  delay(50);

  // Attach each channel to its pin with pulse limits.
  // Index [0] corresponds to the first channel you attach on this list.
  servoA.attach(servoPinA, SERVO_MIN_US, SERVO_MAX_US, (SERVO_MIN_US + SERVO_MAX_US) / 2);
  servoB.attach(servoPinB, SERVO_MIN_US, SERVO_MAX_US, (SERVO_MIN_US + SERVO_MAX_US) / 2);

  Serial.println("STM32ServoList sweep starting…");
}

void loop() {
  // Sweep up
  for (int angle = SWEEP_MIN_DEG; angle <= SWEEP_MAX_DEG; angle += SWEEP_STEP) {
    // Example: move B in opposite direction; set both the same if you prefer
    servoA[0].write(angle);
    servoB[0].write(SWEEP_MAX_DEG - angle);
    delay(STEP_DELAY_MS);
  }

  // Sweep down
  for (int angle = SWEEP_MAX_DEG; angle >= SWEEP_MIN_DEG; angle -= SWEEP_STEP) {
    servoA[0].write(angle);
    servoB[0].write(SWEEP_MAX_DEG - angle);
    delay(STEP_DELAY_MS);
  }
}
