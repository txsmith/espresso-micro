#include <RBDdimmer.h>
#include "shared.h"
#include "pump.h"

dimmerLamp dimmer(PUMP_PSM_PIN, ZERO_CROSS_PIN); 
void setupPump() {
  pinMode(ZERO_CROSS_PIN, INPUT); // Zero-cross
  pinMode(PUMP_PSM_PIN, OUTPUT); // PSM
  dimmer.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 
  dimmer.setPower(0);
}
void setPumpPwr(int pwr) {

}