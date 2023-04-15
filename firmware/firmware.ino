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
const bool DEBUG = false;

// Constants
#define RED 22
#define BLUE 24
#define GREEN 23
#define BLE_DELAY 25
#define RX_BUFFER_SIZE 256
const char* formatString = "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%d\n"; // formatting string for logging
// type for our data points
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

// BLE Constants
const char* nameOfPeripheral = "testDevice";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb"; // UUID for BLE, allows client to identify incoming data
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
// BLE service
BLEService sendService(uuidOfService);
// BLEIndicate is much slower but ensures proper data transfer, BLENotify is faster but slight data loss
BLEStringCharacteristic datachar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, RX_BUFFER_SIZE);

// Button Constants
const PinName buttonPin = PinName::AIN0;  // the number of the pushbutton pin (Analog 0)
const int pushDelay = 2000000; // button delays
const int resetDelay = 6000000;

// Global Variables
volatile bool isLogging = false;  // volatile bool for logging flag
uint64_t fallTime;
float gx, gy, gz, ax, ay, az, pr;  // data values
deque<dataPoint> buffer;
FILE* f = NULL;
const char* fileName = "/fs/data.txt";
FlashIAPBlockDevice bd(0x80000, 0x80000);
// Alternatives to using internal flash storage
// SPIFBlockDevice bd(PTE2, PTE4, PTE1, PTE5);
// QSPIFBlockDevice bd(QSPI_FLASH1_IO0, QSPI_FLASH1_IO1, QSPI_FLASH1_IO2, QSPI_FLASH1_IO3, QSPI_FLASH1_SCK, QSPI_FLASH1_CSN, QSPIF_POLARITY_MODE_0, MBED_CONF_QSPIF_QSPI_FREQ);

// semaphore used to prevent multiple threads from accessing buffer at same time
Semaphore one_slot(1);
// file system, can be changed to little FS as well
static mbed::FATFileSystem fs("fs");
EventQueue readQueue(32 * EVENTS_EVENT_SIZE);
EventQueue writeQueue(32 * EVENTS_EVENT_SIZE);
// barometer
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();
BLEDevice peripheral;
mbed::InterruptIn button(buttonPin);
Thread signalThread;
Thread writeThread;
// timer used for tracking time when logging, disables sleep when active
mbed::Timer timer;

// Reset function, resets all variables and flash chip
void reset(FILE* f) {
  digitalWrite(RED, LOW);
  isLogging = false;
  buffer.clear();
  fclose(f);
  fs.format(&bd);
  f = fopen(fileName, "a+");
  digitalWrite(RED, HIGH);
}

// Gyro function, reads from IMU
void checkGyro(void) {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
}

// Accelerometer function, reads from IMU
void checkAccel(void) {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
}

// Barometer function, reads from baromter
void checkBarom(void) {
  pr = lps35hw.readPressure();
}

// Event for button push
void handleFall(void) {
  fallTime = us_ticker_read();
}

// Event for button release, adds events for outcome to event queues
void handleRise(void) {
  if (us_ticker_read() - fallTime < pushDelay) {
    startStopLogging();
  } else if (us_ticker_read() - fallTime < resetDelay) {
    writeQueue.call(transferData);
  } else {
    writeQueue.call(reset, f);
  }
}

// Function for changing flags for writing to flash and reset LED/timer
void startStopLogging(void) {
  isLogging = !isLogging;
  if (isLogging) {
    digitalWrite(GREEN, LOW);
    timer.start();
    timer.reset();
  } else {
    digitalWrite(GREEN, HIGH);
    timer.reset();
  }
}

// Adds data to buffer
void addToBuffer(void) {
  checkGyro();
  checkAccel();
  checkBarom();

  if (isLogging) {
    if (DEBUG) {
      Serial.println("buffer push");
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

// Transfer data via BLE for offload
void transferData() {
  one_slot.acquire();
  digitalWrite(BLUE, LOW);
  bool complete = true;
  digitalWrite(GREEN, LOW);
  // Process any remaining data in the buffer
  while (!buffer.empty()) {
    dataPoint p = buffer.front();
    buffer.pop_front();
    fprintf(f, formatString, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.pr, p.time);
    fflush(f);
  }
  digitalWrite(GREEN, HIGH);
  // check BLE flag, can be disabled
  if (useBLE) {
    // Init Bluetooth module and advertise device
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
    // Close file buffer to ensure everything is in flash storage
    fflush(f);
    fclose(f);
    f = fopen(fileName, "r");
    if (DEBUG) {
      Serial.println(!f ? "Fail" : "OK");
      while (true) {
        digitalWrite(RED, LOW);
        digitalWrite(RED, HIGH);
      }
    }
    char line[RX_BUFFER_SIZE];
    // wait for connection
    while (true) {
      BLEDevice central = BLE.central();
      if (central.connected()) {
        break;
      }
    }
    if (DEBUG) {
      Serial.println("beginning file transfer");
    }
    // iterate over each line in file and set BLE GATT char to line
    while (fgets(line, sizeof(line), f)) {
      BLEDevice central = BLE.central();
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
    // write 0 to signal finish
    datachar.writeValue("0");
    ThisThread::sleep_for(BLE_DELAY);
    BLEDevice central = BLE.central();
    // wait for disconnect
    while (central.connected()) {
      central = BLE.central();
    }
  }
  digitalWrite(BLUE, HIGH);
  digitalWrite(RED, LOW);
  // purge buffers and reformat flash
  fflush(f);
  fclose(f);
  if (complete) {
    fs.format(&bd);
  }
  f = fopen(fileName, "a+");
  digitalWrite(RED, HIGH);
  one_slot.release();
}

// Print data, will send to serial if DEBUG is true otherwise log to flash storage
void printData(FILE* f) {
  if (isLogging && !buffer.empty()) {
    one_slot.acquire();
    dataPoint p = buffer.front();
    buffer.pop_front();
    one_slot.release();
    fprintf(f, formatString, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.pr, p.time);
    fflush(f);
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
    Serial.println("Started");
  }
  // ensure file system is mounted, if not reformat
  int err = fs.mount(&bd);
  if (err) {
    if (DEBUG) {
      Serial.println("Error with File System");
    }
    fs.format(&bd);
  }
  // Check sensors to ensure we can read values
  if (!IMU.begin() || !lps35hw.begin_I2C()) {
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
  // Open data file as io stream, if fail reformat
  f = fopen(fileName, "a+");
  while (!f) {
    fs.format(&bd);
    f = fopen(fileName, "a+");
  }
  if (DEBUG) {
    Serial.println("Init file system");
  }
// set internal pullup resistor
  pinMode(buttonPin, INPUT_PULLUP);
// add events to their respective queues
  int printID = writeQueue.call_every(10ms, printData, f);
  int writeToBufferID = readQueue.call_every(10ms, addToBuffer);
  // set button behaviour
  button.fall(readQueue.event(handleFall));
  button.rise(readQueue.event(handleRise));
  if (DEBUG) {
    Serial.println("setup complete");
  }
  // start threads
  signalThread.start(callback(&readQueue, &EventQueue::dispatch_forever));
  writeThread.start(callback(&writeQueue, &EventQueue::dispatch_forever));
}

void loop() {}