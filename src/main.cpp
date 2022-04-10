#include <Arduino.h>
#include "shared.h"
#include "pid.h"
#include "heater.h"
#include "ble.h"
#include "credentials.h"
#include "ota.h"

// JLed led = JLed(22).Off().LowActive();
void setup() {
  pinMode(HEATER_SSR_PIN, OUTPUT);
  digitalWrite(HEATER_SSR_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.begin(115200);

  setupOTA("espressomicro", WIFI_SSID, WIFI_PASSWORD);

  EspressoEnv *env = new EspressoEnv();
  env->bleQueue = xQueueCreate(32, sizeof(EspressoNotification));
  env->pidQueue = xQueueCreate(32, sizeof(EspressoNotification));
  env->heaterQueue = xQueueCreate(32, sizeof(EspressoNotification));

  xTaskCreatePinnedToCore(heaterTask, "heater", 4096, env, 2, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "pid", 4096, env, 1, NULL, 1);
  xTaskCreatePinnedToCore(bleTask, "ble", 20000, env, 1, NULL, 1);


  // setStatusLED(led_off);
  // startLEDTask();
  // startWebUI();
}

void loop() {
  // vTaskSuspend(NULL);
  vTaskDelay(1000 / portTICK_RATE_MS);
  // delay(1500);
  // if (currentWebUIState() == connecting) {
  //   setStatusLED(breathing);
  // } else if (currentWebUIState() == connection_setup) {
  //   setStatusLED(slow_blink);
  // } else if (currentWebUIState() == error) {
  //   setStatusLED(fast_blink);
  // } else {
  //   setStatusLED(led_on);
  // }
}
