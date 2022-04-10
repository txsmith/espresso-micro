#include <Arduino.h>
const uint8_t HEATER_SSR_PIN = 26;
const uint8_t STATUS_LED_PIN = 25;
const uint8_t ZERO_CROSS_PIN = 0;
const uint8_t PUMP_PSM_PIN = 14;

enum EspressoNotificationType { Notify_TemperatureMeasurement,
                                Notify_HeaterPower,
                                Notify_Pump_Pwr,
                                Notify_Current_Settings,
                                Notify_Changed_Setpoint_Temp,
                                Notify_Changed_P,
                                Notify_Changed_I,
                                Notify_Changed_D };
struct EspressoNotification {
  EspressoNotificationType type;
  void *value;
};

struct PIDConfig {
  double setpointTemp, kP, kI, kD;
};

struct EspressoEnv {
  QueueHandle_t bleQueue;
  QueueHandle_t pidQueue;
  QueueHandle_t heaterQueue;
};
