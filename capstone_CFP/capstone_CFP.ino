#include <mbed.h>
#include <mbed_events.h>
#include <rtos.h>
#include <stdio.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <FlashIAPBlockDevice.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include "FATFileSystem.h"
#include "BlockDevice.h"
#include <SPI.h>
#include <PinNames.h>
#include <fstream>
#include <deque>
#include <sstream>

using namespace rtos;
using namespace events;
using namespace std;
using namespace std::chrono_literals;
using namespace std::chrono;
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
uint64_t fallTime;
float gx, gy, gz, ax, ay, az, pr;  // data values
typedef struct dataPoint {
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float pr;
  uint64_t time;
};
deque<dataPoint> buffer;
FILE* f = NULL;
const char* fileName = "/fs/data.txt";
FlashIAPBlockDevice bd(0x80000, 0x80000);
// BlockDevice *bd = new HeapBlockDevice(2048, 1, 1, 512);
Semaphore one_slot(1);

static mbed::FATFileSystem fs("fs");
// BLE service
BLEService sendService(uuidOfService);
// BLEIndicate is much slower but ensures proper data transfer, BLENotify is faster but slight data loss
BLEStringCharacteristic datachar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, RX_BUFFER_SIZE);

BLEDevice peripheral;

EventQueue readQueue(64 * EVENTS_EVENT_SIZE);
EventQueue writeQueue(32 * EVENTS_EVENT_SIZE);

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

mbed::InterruptIn button(buttonPin);
mbed::InterruptIn pauseButton(pauseButtonPin);

Thread signalThread;
Thread writeThread;

mbed::Timer timer;

void (*resetFunc)(void) = 0;

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
  fallTime = timer.elapsed_time().count();
}

void handleRise(void) {
  if (timer.elapsed_time().count() - fallTime > pushDelay) {
    transferData();
  } else {
    startStopLogging();
  }
}

void startStopLogging(void) {
  isLogging = !isLogging;
  if (isLogging) {
    digitalWrite(GREEN, LOW);
    timer.reset();
  } else {
    digitalWrite(GREEN, HIGH);
  }
}

void addToBuffer(void) {
  if (isLogging) {
    Serial.println("buffer push");
    dataPoint p;
    p.ax = ax;
    p.ay = ay;
    p.az = az;
    p.gx = gx;
    p.gy = gy;
    p.gz = gz;
    p.pr = pr;
    p.time = timer.elapsed_time().count();
    one_slot.acquire();
    buffer.push_back(p);
    one_slot.release();
  }
}

void startBLEAdvertise() {
  BLE.advertise();
}

void stopBLEAdvertise() {
  BLE.stopAdvertise();
}

void transferData() {
  one_slot.acquire();
  // startBLEAdvertise();
  digitalWrite(BLUE, LOW);
  while (!buffer.empty()) {
    dataPoint p = buffer.front();
    buffer.pop_front();
    std::string value = std::to_string(p.ax) + "," + std::to_string(p.ay) + "," + std::to_string(p.az) + "," + std::to_string(p.gx) + "," + std::to_string(p.gy) + "," + std::to_string(p.gz) + "," + std::to_string(p.pr) + "," + std::to_string(p.time) + "\n";
    Serial.println(value.c_str());
    fwrite(value.c_str(), value.length(), 1, f);
    fflush(f);
  }
  if (useBLE) {
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
  fs.format(&bd);
  f = fopen(fileName, "a+");
  digitalWrite(RED, HIGH);
  one_slot.release();
}

void printData(FILE* f) {
  if (isLogging && !buffer.empty()) {
    one_slot.acquire();
    dataPoint p = buffer.front();
    buffer.pop_front();
    one_slot.release();
    std::string value = std::to_string(p.ax) + "," + std::to_string(p.ay) + "," + std::to_string(p.az) + "," + std::to_string(p.gx) + "," + std::to_string(p.gy) + "," + std::to_string(p.gz) + "," + std::to_string(p.pr) + "," + std::to_string(p.time) + "\n";
    Serial.println(value.c_str());
    fwrite(value.c_str(), value.length(), 1, f);
    fflush(f);
  } else {
    std:string value = std::to_string(ax) + "," + std::to_string(ay) + "," + std::to_string(az) + "," + std::to_string(gx) + "," + std::to_string(gy) + "," + std::to_string(gz) + "," + std::to_string(pr) + "," + std::to_string(timer.elapsed_time().count());
    Serial.println(value.c_str());
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
  int err = fs.mount(&bd);
  if (err) {
    Serial.println("Error with File System");
    fs.reformat(&bd);
    resetFunc();
  }
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

  timer.start();

  int accelID = readQueue.call_every(10ms, checkAccel);
  int gyroID = readQueue.call_every(10ms, checkGyro);
  int baromID = readQueue.call_every(100ms, checkBarom);
  int printID = writeQueue.call_every(10ms, printData, f);
  int writeToBufferID = readQueue.call_every(10ms, addToBuffer);
  button.fall(readQueue.event(handleFall));
  button.rise(readQueue.event(handleRise));
  Serial.println("setup complete");
  signalThread.start(callback(&readQueue, &EventQueue::dispatch_forever));
  writeThread.start(callback(&writeQueue, &EventQueue::dispatch_forever));
}

void loop() {}
