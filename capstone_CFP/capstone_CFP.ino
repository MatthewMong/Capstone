#include <mbed.h>
#include <mbed_events.h>
#include <rtos.h>
#include <stdio.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <FlashIAPBlockDevice.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include "LittleFileSystem.h"
#include "HeapBlockDevice.h"
#include "BlockDevice.h"
#include <SPI.h>
#include <PinNames.h>
#include <fstream>
#include <vector>
#include <sstream>

using namespace rtos;
using namespace events;
using namespace std;

// Flags
const bool useBLE = true;

// Constants
#define RED 22
#define BLUE 24
#define GREEN 23
#define BLE_DELAY 25
#define RX_BUFFER_SIZE 256
const char* nameOfPeripheral = "testDevice";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
const PinName buttonPin = PinName::AIN0;  // the number of the pushbutton pin (Analog 0)
const PinName pauseButtonPin = PinName::AIN2;
const int pushDelay = 3000000;
volatile bool isLogging = false;  // volatile bool for logging flag
uint32_t fallTime;
float gx, gy, gz, ax, ay, az, pr;  // data values
FILE* f = NULL;
const char* fileName = "/fs/data.txt";
mbed::BlockDevice *bd = mbed::HeapBlockDevice::get_default_instance();
// BlockDevice *bd = new HeapBlockDevice(2048, 1, 1, 512);

static mbed::LittleFileSystem fs("fs");

// BLE service
BLEService sendService(uuidOfService);
// BLEIndicate is much slower but ensures proper data transfer, BLENotify is faster but slight data loss
BLEStringCharacteristic datachar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, RX_BUFFER_SIZE);

BLEDevice peripheral;

EventQueue queue(64 * EVENTS_EVENT_SIZE);

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

mbed::InterruptIn button(buttonPin);
mbed::InterruptIn pauseButton(pauseButtonPin);

Thread t1;

void checkGyro(void) {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
}

void checkAccel(void) {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
}

void checkBarom(void) {
  pr = lps35hw.readPressure();
}

void handleFall(void) {
  us_ticker_init();
  fallTime = us_ticker_read();
}

void handleRise(void) {
  fflush(f);
  if (us_ticker_read() - fallTime > pushDelay) {
    transferData();
  } else {
    startStopLogging();
  }
}

void startStopLogging(void) {
  isLogging = !isLogging;
  if (isLogging) {
    digitalWrite(GREEN, LOW);
  } else {
    digitalWrite(GREEN, HIGH);
  }
}

void startBLEAdvertise() {
  BLE.advertise();
}

void stopBLEAdvertise() {
  BLE.stopAdvertise();
}

void transferData() {
  // startBLEAdvertise();
  if (useBLE) {
    digitalWrite(BLUE, LOW);
    if (!BLE.begin()) {
      Serial.println("* Starting Bluetooth® Low Energy module failed!");
      while (1)
        ;
    }
    Serial.println("Arduino Nano 33 BLE Sense (Central Device)");

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sendService);
    sendService.addCharacteristic(datachar);
    BLE.addService(sendService);
    BLE.advertise();
    Serial.print("Peripheral device MAC: ");
    Serial.println(BLE.address());
    fflush(f);
    fclose(f);
    f = fopen(fileName, "r");
    Serial.println(!f ? "Fail" : "OK");
    char line[RX_BUFFER_SIZE];
    while (true) {
      BLEDevice central = BLE.central();
      if (central.connected()) {
        break;
      }
    }
    Serial.println("beginning file transfer");
    while (fgets(line, sizeof(line), f)) {
      Serial.println(line);
      BLEDevice central = BLE.central();
      if (central.connected()) {
        ThisThread::sleep_for(BLE_DELAY);
        datachar.writeValue(line);
      } else {
        Serial.println("disconnect");
        break;
      }
    }
    datachar.writeValue("0");
    ThisThread::sleep_for(BLE_DELAY);
    BLEDevice central = BLE.central();
    while (central.connected()) {
      central = BLE.central();
    }
  }
  digitalWrite(BLUE, HIGH);
  digitalWrite(RED, LOW);
  fflush(f);
  fclose(f);
  fs.reformat(bd);
  f = fopen(fileName, "a+");
  digitalWrite(RED, HIGH);
}

void printData(FILE* f) {
  if (isLogging) {
    String value = (String)ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + pr + "," + us_ticker_read() + "\n";
    // fwrite(value.c_str(), value.length(), 1, f);
    fprintf(f, "%s", value.c_str());
  } else {
    Serial.println((String)ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + pr + "," + us_ticker_read());
  }
}

void setup() {
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  digitalWrite(BLUE, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, HIGH);
  Serial.begin(115200);
  Serial.println("Started");
  bd->init();
  fs.mount(bd);
  // fs.reformat(bd);
  // disabling barometer for now
  // if (!IMU.begin() || !lps35hw.begin_I2C()) {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize sensors!");
    while (1)
      ;
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  f = fopen(fileName, "a+");
  Serial.println("Init file system");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pauseButtonPin, INPUT_PULLUP);

  int accelID = queue.call_every(10, checkAccel);
  int gyroID = queue.call_every(100, checkGyro);
  int baromID = queue.call_every(10, checkBarom);
  int printID = queue.call_every(10, printData, f);
  button.fall(queue.event(handleFall));
  button.rise(queue.event(handleRise));
  Serial.println("setup complete");
  t1.start(callback(&queue, &EventQueue::dispatch_forever));
}

void loop() {}
