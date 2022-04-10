#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <Preferences.h>
#include "shared.h"
#include "pid.h"
#include "status-led.h"

#define Default_P 0.2
#define Default_I 0
#define Default_D 0.001
#define Default_Temp 91.0
#define PID_Compute_Interval 200

double currentTemp, output;

Preferences preferences;

#define RREF 429.0
#define RNOMINAL 100.0

Adafruit_MAX31865 setupSensor() {
  Adafruit_MAX31865 thermo = Adafruit_MAX31865(SCL, MOSI, MISO, SCK);
  thermo.begin(MAX31865_2WIRE);
  return thermo;
}
Adafruit_MAX31865 thermo = setupSensor();

double getCurrentTemperature() {
  uint16_t rtd = thermo.readRTD();

  // Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  // Serial.print("Ratio = "); Serial.println(ratio,8);
  // Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  double temperature = thermo.temperature(RNOMINAL, RREF);
  // Serial.print("Temperature = "); Serial.println(temperature);

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    temperature = 999;
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  return temperature;
}

PIDConfig readPIDConfig() {
  preferences.begin("pid", false);
  double setpointTemp = preferences.getDouble("temp", 0.0);
  double kP = preferences.getDouble("p");
  double kI = preferences.getDouble("i");
  double kD = preferences.getDouble("d");
  // Initialize defaults if nothing is set
  if (setpointTemp < 1.0) {
    Serial.println("Initializing PID preferences");
    preferences.putDouble("temp", Default_Temp);
    preferences.putDouble("p", Default_P);
    preferences.putDouble("i", Default_I);
    preferences.putDouble("d", Default_D);
    setpointTemp = Default_Temp;
    kP = Default_P;
    kI = Default_I;
    kD = Default_D;
  }
  PIDConfig config = {setpointTemp, kP, kI, kD};
  Serial.printf("Retrieved settings: temp  = %.3f, p = %.3f, i = %.3f, d = %.3f\n", setpointTemp, kP, kI, kD);
  preferences.end();
  return config;
}

void setTargetTemperature(PIDConfig *conf, PID *pid, double tempCelsius) {
  preferences.begin("pid", false);
  preferences.putDouble("temp", tempCelsius);
  preferences.end();
  conf->setpointTemp = tempCelsius;
  pid->SetTunings(conf->kP, conf->kI, conf->kD);
}
void setKp(PIDConfig *conf, PID *pid, double k) {
  preferences.begin("pid", false);
  preferences.putDouble("p", k);
  preferences.end();
  conf->kP = k;
  pid->SetTunings(conf->kP, conf->kI, conf->kD);
}
void setKi(PIDConfig *conf, PID *pid, double k) {
  preferences.begin("pid", false);
  preferences.putDouble("i", k);
  preferences.end();
  conf->kI = k;
  pid->SetTunings(conf->kP, conf->kI, conf->kD);
}
void setKd(PIDConfig *conf, PID *pid, double k) {
  preferences.begin("pid", false);
  preferences.putDouble("d", k);
  preferences.end();
  conf->kD = k;
  pid->SetTunings(conf->kP, conf->kI, conf->kD);
}

EspressoNotification notification;
void handleConfigChangeMsg(EspressoEnv *env, PIDConfig *conf, PID *pid) {
  if (xQueueReceive(env->pidQueue, &notification, 0) == pdPASS) {
    if (notification.type == Notify_Changed_Setpoint_Temp) {
      double value = *static_cast<double *>(notification.value);
      Serial.printf("Setting temperature to %.3f\n", value);
      setTargetTemperature(conf, pid, value);
    } else if (notification.type == Notify_Changed_P) {
      double value = *static_cast<double *>(notification.value);
      Serial.printf("Setting kP to %.3f\n", value);
      setKp(conf, pid, value);
    } else if (notification.type == Notify_Changed_I) {
      double value = *static_cast<double *>(notification.value);
      Serial.printf("Setting kI to %.3f\n", value);
      setKi(conf, pid, value);
    } else if (notification.type == Notify_Changed_D) {
      double value = *static_cast<double *>(notification.value);
      Serial.printf("Setting kD to %.3f\n", value);
      setKd(conf, pid, value);
    }
    delete static_cast<double *>(notification.value);
  }
}


double lastTemp = 0.0;
double lastHeatPwr = 0.0;
void handleTemperatureChange(EspressoEnv *env, PIDConfig *conf, PID *pid) {
  currentTemp = getCurrentTemperature();
  if (currentTemp > 0 && currentTemp < 150) {
    if (lastTemp != round(currentTemp * 100) / 100) {
      EspressoNotification tempNotification = {Notify_TemperatureMeasurement, new double(currentTemp)};
      xQueueSendToBack(env->bleQueue, &tempNotification, 0);
      lastTemp = round(currentTemp * 100) / 100;
    }
    if (pid->Compute()) {
      EspressoNotification heatPwrNotification = {Notify_HeaterPower, new double(output * 10000)};
      xQueueSendToBack(env->heaterQueue, &heatPwrNotification, 0);

      if (lastHeatPwr != round(output * 100) / 100) {
        EspressoNotification bleHeatNotification = {Notify_HeaterPower, new double(lastHeatPwr)};
        xQueueSendToBack(env->bleQueue, &bleHeatNotification, 0);
        lastHeatPwr = round(output * 100) / 100;
      }
    }
    Serial.printf("PID loop: %.3f / %.3f / %.3f\n", currentTemp, conf->setpointTemp, output);
  } else {
    Serial.printf("WARNING! Temperature annomaly: %.3f degrees C\n", currentTemp);
    EspressoNotification heatPwrNotification = {Notify_HeaterPower, new double(0.0)};
    xQueueSendToBack(env->heaterQueue, &heatPwrNotification, 0);
    EspressoNotification bleHeatNotification = {Notify_HeaterPower, new double(0.0)};
    xQueueSendToBack(env->bleQueue, &bleHeatNotification, 0);
  }
}

void pidTask(void *parameter) {
  blinkStatusLED(3000);
  Serial.println("Starting PID task");
  EspressoEnv *env = static_cast<EspressoEnv *>(parameter);

  PIDConfig conf = readPIDConfig();
  PID pid(&currentTemp, &output, &conf.setpointTemp, conf.kP, conf.kI, conf.kD, DIRECT);
  pid.SetOutputLimits(0, 1);
  pid.SetMode(AUTOMATIC);
  currentTemp = 100.0;

  EspressoNotification currentSettingsNotification = {
      Notify_Current_Settings,
      new PIDConfig(conf)};
  xQueueSendToBack(env->bleQueue, &currentSettingsNotification, 0);

  while (true) {
    handleConfigChangeMsg(env, &conf, &pid);
    handleTemperatureChange(env, &conf, &pid);
    vTaskDelay(PID_Compute_Interval / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}
