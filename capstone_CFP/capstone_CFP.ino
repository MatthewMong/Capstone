#include <mbed.h>
#include <mbed_events.h>
#include <rtos.h>
#include <stdio.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include "FATFileSystem.h"
#include "BlockDevice.h"
#include <SPI.h>
#include <memory>
#include <string>
#include <stdexcept>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
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
const bool DEBUG = false;

// Constants
#define RED 22
#define BLUE 24
#define GREEN 23
#define BLE_DELAY 25
#define RX_BUFFER_SIZE 256
#define FILE_NAME "data3.txt"
Adafruit_FlashTransport_SPI flashTransport(D5, &SPI);
Adafruit_SPIFlash flash(&flashTransport);
const int flashDevices = 1;
static const SPIFlash_Device_t my_flash_devices[] = {
  W25Q128JV_SQ,
};
const char* nameOfPeripheral = "testDevice";
const int SPI_SPEED = 32;
const char* formatString = "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
const PinName buttonPin = PinName::AIN0;  // the number of the pushbutton pin (Analog 0)
const PinName pauseButtonPin = PinName::AIN2;
const int pushDelay = 2000000;
const int resetDelay = 6000000;
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

// Global Variables
uint64_t fallTime;
volatile bool isLogging = false;   // volatile bool for logging flag
float gx, gy, gz, ax, ay, az, pr;  // data values
deque<dataPoint> buffer;
// FILE* f = NULL;

// mbed::QSPIFBlockDevice bd(0x80000, 0x80000);
// mbed::BlockDevice *bd = new mbed::SPIFBlockDevice(2048, 1, 1, 512);
Semaphore one_slot(1);

// static mbed::FATFileSystem fs("fs");
FatVolume fatfs;

// BLE service
BLEService sendService(uuidOfService);
// BLEIndicate is much slower but ensures proper data transfer, BLENotify is faster but slight data loss
BLEStringCharacteristic datachar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, RX_BUFFER_SIZE);

BLEDevice peripheral;

EventQueue readQueue(32 * EVENTS_EVENT_SIZE);
EventQueue writeQueue(32 * EVENTS_EVENT_SIZE);

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

mbed::InterruptIn button(buttonPin);
mbed::InterruptIn pauseButton(pauseButtonPin);

Thread signalThread;
Thread writeThread;

mbed::Timer timer;

template<typename... Args>
std::string string_format(const std::string& format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

void reset(File32& f) {
  digitalWrite(RED, LOW);
  isLogging = false;
  buffer.clear();
  f.close();
  fatfs.remove(FILE_NAME);
  f = fatfs.open(FILE_NAME, FILE_WRITE);
  digitalWrite(RED, HIGH);
}

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
  fallTime = us_ticker_read();
}

void handleRise(File32& f) {
  if (us_ticker_read() - fallTime < pushDelay) {
    startStopLogging();
  } else if (us_ticker_read() - fallTime < resetDelay) {
    isLogging = false;
    writeQueue.call(transferData, f);
  } else {
    writeQueue.call(reset, f);
  }
}

void startStopLogging(void) {
  isLogging = !isLogging;
  if (isLogging) {
    digitalWrite(GREEN, LOW);
    timer.reset();
    timer.start();
  } else {
    digitalWrite(GREEN, HIGH);
    timer.stop();
  }
}

void addToBuffer(void) {
  checkGyro();
  checkAccel();
  checkBarom();

  if (isLogging) {
    if (DEBUG) {
      Serial.println(timer.elapsed_time().count());
    }
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

void transferData(File32& f) {
  one_slot.acquire();
  // startBLEAdvertise();
  digitalWrite(BLUE, LOW);
  bool complete = true;
  digitalWrite(GREEN, LOW);
  while (!buffer.empty()) {
    dataPoint p = buffer.front();
    buffer.pop_front();
    if (DEBUG) {
      Serial.println(string_format(formatString, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.pr, p.time).c_str());
    }
    f.println(string_format(formatString, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.pr, p.time).c_str());
  }
  digitalWrite(GREEN, HIGH);
  if (useBLE) {
    if (!BLE.begin()) {
      if (DEBUG) {
        Serial.println("* Starting Bluetooth® Low Energy module failed!");
      }
      complete = false;
      while (1)
        ;
    }
    if (DEBUG) {
      Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
    }

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sendService);
    sendService.addCharacteristic(datachar);
    BLE.addService(sendService);
    BLE.advertise();
    if (DEBUG) {
      Serial.print("Peripheral device MAC: ");
      Serial.println(BLE.address());
    }
    f.close();
    f = fatfs.open(FILE_NAME, FILE_READ);
    if (DEBUG) {
      Serial.println(!f ? "Fail" : "OK");
      while (!f) {
        digitalWrite(RED, LOW);
        digitalWrite(RED, HIGH);
      }
    }
    char line[RX_BUFFER_SIZE];
    while (true) {
      BLEDevice central = BLE.central();
      if (central.connected()) {
        break;
      }
    }
    if (DEBUG) {
      Serial.println("beginning file transfer");
    }
    while (f.available()) {
      BLEDevice central = BLE.central();
      String line = f.readStringUntil('\n');
      if (DEBUG) {
        Serial.println(line);
      }
      if (central.connected()) {
        ThisThread::sleep_for(BLE_DELAY);
        datachar.writeValue(line);
      } else {
        if (DEBUG) {
          Serial.println("disconnect");
        }
        complete = false;
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
  f.close();
  if (complete) {
    fatfs.remove(FILE_NAME);
  }
  f = fatfs.open(FILE_NAME, FILE_WRITE);
  digitalWrite(RED, HIGH);
  one_slot.release();
}

void printData(File32& f) {
  if (isLogging && !buffer.empty() && f.available()) {
    one_slot.acquire();
    dataPoint p = buffer.front();
    buffer.pop_front();
    one_slot.release();
    f.println(string_format(formatString, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.pr, p.time).c_str());
  } else if (DEBUG) {
std:
    string value = std::to_string(ax) + "," + std::to_string(ay) + "," + std::to_string(az) + "," + std::to_string(gx) + "," + std::to_string(gy) + "," + std::to_string(gz) + "," + std::to_string(pr) + "," + std::to_string(timer.elapsed_time().count());
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
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial) delay(100);  // wait for native usb
    Serial.println("Started");
  }
  // flashTransport.setClockSpeed(SPI_SPEED * 1000000, SPI_SPEED * 1000000);
  // int err = fs.mount(&bd);
  if (!flash.begin(my_flash_devices, flashDevices)) {
    if (DEBUG) {
      Serial.println("Error with Flash System");
    }
  }
  if (!fatfs.begin(&flash)) {
    if (DEBUG) {
      Serial.println("Error with File System");
    }
  }
  // disabling barometer for now
  // if (!IMU.begin() || !lps35hw.begin_I2C()) {
  if (!IMU.begin()) {
    if (DEBUG) {
      Serial.println("Failed to initialize sensors!");
    }
    while (1)
      ;
  }
  if (DEBUG) {
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");
  }
  File32 f = fatfs.open(FILE_NAME, FILE_WRITE);
  while (!f) {
    fatfs.remove(FILE_NAME);
    f = fatfs.open(FILE_NAME, FILE_WRITE);
  }
  if (DEBUG) {
    Serial.println("Init file system");
  }

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pauseButtonPin, INPUT_PULLUP);


  int printID = writeQueue.call_every(10ms, printData, f);
  int writeToBufferID = readQueue.call_every(10ms, addToBuffer);
  button.fall(readQueue.event(handleFall));
  button.rise(readQueue.event(handleRise, f));
  if (DEBUG) {
    Serial.println("setup complete");
  }
  signalThread.start(callback(&readQueue, &EventQueue::dispatch_forever));
  writeThread.start(callback(&writeQueue, &EventQueue::dispatch_forever));
}

void loop() {}
