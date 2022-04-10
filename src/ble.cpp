#include <Arduino.h>
#include <NimBLEDevice.h>
#include "shared.h"
#include "pid.h"
#include "status-led.h"

static const char *pidServiceUUID = "00c0ffee-add1-c5ed-0000-000000000000";
static const char *temperatureMeasurementUUID = "00c0ffee-add1-c5ed-0000-000000000001";
static const char *setTemperatureUUID = "00c0ffee-add1-c5ed-0000-000000000002";
static const char *setPUUID = "00c0ffee-add1-c5ed-0000-000000000003";
static const char *setIUUID = "00c0ffee-add1-c5ed-0000-000000000004";
static const char *setDUUID = "00c0ffee-add1-c5ed-0000-000000000005";
static const char *heatPwrUUID = "00c0ffee-add1-c5ed-0000-000000000006";

static NimBLEServer *pServer;
static NimBLEService *hPIDService;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    Serial.print("Client connected: ");
    Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    blinkStatusLED(300, 20);
    NimBLEDevice::stopAdvertising();
  };
  void onDisconnect(NimBLEServer *pServer) {
    Serial.println("Client disconnected - start advertising");
    blinkStatusLED(300, 20);
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};

template <typename T>
class CharacteristicWriteCallback : public NimBLECharacteristicCallbacks {
private:
  QueueHandle_t notifyQueue;
  EspressoNotificationType notificationType;

public:
  CharacteristicWriteCallback(EspressoNotificationType type, QueueHandle_t writeEventQueue) {
    notifyQueue = writeEventQueue;
    notificationType = type;
  }
  void onWrite(NimBLECharacteristic *pCharacteristic) {
    Serial.printf("New value written on %s\n", pCharacteristic->getUUID().toString().c_str());
    EspressoNotification notification = {
        notificationType,
        new T(pCharacteristic->getValue().getValue<T>())};
    xQueueSendToBack(notifyQueue, &notification, 0);
    blinkStatusLED(500, 20);
  };
};

NimBLECharacteristic *createTemperatureCharacteristic() {
  NimBLECharacteristic *characteristic =
      hPIDService->createCharacteristic(
          temperatureMeasurementUUID,
          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);

  NimBLE2904 *pTemperature2904 = (NimBLE2904 *)characteristic->createDescriptor("2904");
  uint16_t TEMP_CELCIUS_UNIT = 0x272F;
  pTemperature2904->setUnit(TEMP_CELCIUS_UNIT);
  pTemperature2904->setFormat(NimBLE2904::FORMAT_FLOAT64);
  return characteristic;
}

NimBLECharacteristic *createWritableCharacteristic(const char *uuid, EspressoNotificationType eventType, QueueHandle_t writeQueue) {
  NimBLECharacteristic *characteristic =
      hPIDService->createCharacteristic(
          uuid,
          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
  characteristic->setCallbacks(new CharacteristicWriteCallback<double>(
      eventType,
      writeQueue));
  return characteristic;
}

void setupPidService(EspressoEnv *env) {
  hPIDService = pServer->createService(pidServiceUUID);
  createTemperatureCharacteristic();
  createWritableCharacteristic(setTemperatureUUID, Notify_Changed_Setpoint_Temp, env->pidQueue);
  createWritableCharacteristic(setPUUID, Notify_Changed_P, env->pidQueue);
  createWritableCharacteristic(setIUUID, Notify_Changed_I, env->pidQueue);
  createWritableCharacteristic(setDUUID, Notify_Changed_D, env->pidQueue);
  hPIDService->start();
}

String getMacAddress() {
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[13] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

template <typename T>
void setCharacteristicValue(const char *uuid, const T &value) {
  if (hPIDService) {
    NimBLECharacteristic *pChr = hPIDService->getCharacteristic(uuid);
    if (pChr) {
      pChr->setValue(value);
      if (pServer->getConnectedCount()) {
        if (pChr->getProperties() & NIMBLE_PROPERTY::NOTIFY) {
          pChr->notify(true);
        }
      }
    }
  }
}

void bleTask(void *parameter) {
  Serial.println("Starting NimBLE Server");
  EspressoEnv *env = static_cast<EspressoEnv *>(parameter);

  NimBLEDevice::init(("Vibiemme-" + getMacAddress()).c_str());
  // NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  setupPidService(env);

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(pidServiceUUID);
  pAdvertising->start();
  Serial.println("Advertising Started");
  EspressoNotification notification;

  double lastTemp = 0.0;
  PIDConfig pidConfig;
  while (true) {
    if (xQueueReceive(env->bleQueue, &notification, 1000 / portTICK_RATE_MS) == pdPASS) {
      if (notification.type == Notify_TemperatureMeasurement) {
        lastTemp = *static_cast<double *>(notification.value);
        setCharacteristicValue(temperatureMeasurementUUID, lastTemp);
      } else if (notification.type == Notify_Current_Settings) {
        pidConfig = *static_cast<PIDConfig *>(notification.value);
        Serial.printf("Received Notify_Current_Settings: %.3f\n", pidConfig.setpointTemp);
        setCharacteristicValue(setTemperatureUUID, pidConfig.setpointTemp);
        setCharacteristicValue(setPUUID, pidConfig.kP);
        setCharacteristicValue(setIUUID, pidConfig.kI);
        setCharacteristicValue(setDUUID, pidConfig.kD);
      }
      delete static_cast<double *>(notification.value);
    } else {
      setCharacteristicValue(temperatureMeasurementUUID, lastTemp);
    }
  }
  Serial.println("BLE task stopped");
  vTaskDelete(NULL);
}
