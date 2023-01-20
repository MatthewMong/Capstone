#include "PluggableUSBMSD.h"
#include <fstream>

using namespace std;


void setup() {
  Serial.begin(115200);
  MassStorage.begin();
  // fstream f;
  // write a file with some data
  // a+ means append, so every time the board is rebooted the file will grow by a new line
  // f.open("/fs/data.txt", fstream::in | fstream::out | fstream::trunc);
  // String hello = "Hello from Nano33BLE Filesystem\n";
  // f << hello;
  // f.close();
}

void loop() {
  delay(1000);
}