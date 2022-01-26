#include <Arduino.h>
#include "shared.h"

const TickType_t CYCLE_LENGTH_TICKS = 200;

void heaterOn() {
  digitalWrite(HEATER_SSR_PIN, HIGH);
}
void heaterOff() {
  digitalWrite(HEATER_SSR_PIN, LOW);
}
void heatCycle(double onPercentage) {
  TickType_t onTime = onPercentage * CYCLE_LENGTH_TICKS;
  if (onTime > 0) {
    heaterOn();
  }
  vTaskDelay(onTime / portTICK_RATE_MS);
  heaterOff();
  vTaskDelay((CYCLE_LENGTH_TICKS - onTime) / portTICK_RATE_MS);
}

void heaterTask(void *parameters) {
  Serial.println("Starting heater task");
  EspressoEnv *env = static_cast<EspressoEnv *>(parameters);
  heaterOff();
  double onPercentage = 0.0; // percentage of on-time [0, 1]
  EspressoNotification notification;
  while (true) {
    if (xQueueReceive(env->heaterQueue, &notification, 0) == pdPASS) {
      if (notification.type == Notify_HeaterPower) {
        onPercentage = (*static_cast<double *>(notification.value) / 10000);
        // Serial.printf("Heat power: %.3f\n", onPercentage);
      }
      delete static_cast<double *>(notification.value);
    }
    heatCycle(onPercentage);
  }
  heaterOff();
  vTaskDelete(NULL);
}
