#include "PluggableUSBMSD.h"

using namespace std;


void setup() {
  Serial.begin(115200);
  MassStorage.begin();
}

void loop() {
  delay(1000);
}