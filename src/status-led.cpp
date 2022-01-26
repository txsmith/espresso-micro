#include <Arduino.h>
#include "shared.h"
// #include <jled.h>
// #include <status-led.h>

// led_state currentState;

// JLed led = JLed(STATUS_LED).Off().LowActive();

// void ledLoop(void *parameter) {
//   led.Update();
//   led_state lastActedState;
//   while (1) {
//     if (lastActedState != currentState) {
//       lastActedState = currentState;
//       if (currentState == breathing) {
//         led = JLed(STATUS_LED).DelayAfter(1500).Breathe(1500).Forever().LowActive();
//       } else if (currentState == led_on) {
//         led = JLed(STATUS_LED).FadeOn(1500).LowActive();
//       } else if (currentState == led_off) {
//         led = JLed(STATUS_LED).FadeOff(1500).LowActive();
//       } else if (currentState == fast_blink) {
//         led = JLed(STATUS_LED).Blink(50, 50).Forever().LowActive();
//       } else if (currentState == slow_blink) {
//         led = JLed(STATUS_LED).Blink(100, 1000).Forever().LowActive();
//       } else if (currentState == very_slow_blink) {
//         led = JLed(STATUS_LED).Blink(100, 30000).Forever().LowActive();
//       }
//     }
//     led.Update();
//   }
// }
// void startLEDTask() {
//   xTaskCreate(ledLoop, "LED status", 1000, NULL, 1, NULL);
// }
// void setStatusLED(led_state newState) {
//   currentState = newState;
// }

void blinkStatusLED(int millis, int flashesPerSec = 10) {
  int millisPerFlash = 1000 / flashesPerSec;
  int flashes = millis / millisPerFlash;
  for (int i = 0; i < flashes; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    vTaskDelay(millisPerFlash / portTICK_RATE_MS);
    digitalWrite(STATUS_LED_PIN, LOW);
    vTaskDelay(millisPerFlash / portTICK_RATE_MS);
  }
}
