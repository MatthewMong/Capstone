#include <mbed.h>
#include <mbed_events.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <FlashIAPBlockDevice.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include <SPI.h>
#include <PluggableUSBMSD.h>
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
const int BLE_DELAY = 25;
const int RX_BUFFER_SIZE = 256;
bool RX_BUFFER_FIXED_LENGTH = false;
const char* nameOfPeripheral = "testDevice";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
const PinName buttonPin = PinName::AIN0;          // the number of the pushbutton pin (Analog 0)
const PinName transferButtonPin = PinName::AIN1;  // the number of the pushbutton pin (Analog 1)
const PinName pauseButtonPin = PinName::AIN2;
volatile bool isLogging = false;   // volatile bool for logging flag
float gx, gy, gz, ax, ay, az, pr;  // data values
FILE* f = NULL;
const char* fileName = "/fs/data.txt";
static FlashIAPBlockDevice bd(0x80000, 0x80000);
static mbed::FATFileSystem fs("fs");
// BLE service
BLEService sendService(uuidOfService);
// BLEIndicate is much slower but ensures proper data transfer, BLENotify is faster but slight data loss
BLEStringCharacteristic datachar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast, RX_BUFFER_SIZE);

BLEDevice peripheral;

EventQueue queue(64 * EVENTS_EVENT_SIZE);

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

mbed::InterruptIn button(buttonPin);
mbed::InterruptIn transferButton(transferButtonPin);
mbed::InterruptIn pauseButton(pauseButtonPin);

Thread t1;

void checkGyro(void) {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
}

void pauseThread(void) {
  if (fs.unmount() == 0) {
    Serial.println("new MSD");
    t1.terminate();
    MassStorage.begin();
    while (MassStorage.connect()) {
      MassStorage.process();
    }
    t1.start(callback(&queue, &EventQueue::dispatch_forever));
  };
}

void checkAccel(void) {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
}

void checkBarom(void) {
  pr = lps35hw.readPressure();
}

void startStopLogging(void) {
  isLogging = !isLogging;
}

void writeFileBuffer(void) {
  // could probably replace this with just a flush, I originally had it reopen with different permissions
  fclose(f);
  f = fopen(fileName, "a+");
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
    Serial.println("beginning file transfer");
    fclose(f);
    f = fopen(fileName, "r");
    char line[RX_BUFFER_SIZE];
    while (true) {
      BLEDevice central = BLE.central();
      if (central.connected()) {
        break;
      }
    }
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
      ;
    }
  }
  fclose(f);
  f = fopen(fileName, "w+");
}

void printData(FILE* f) {
  if (isLogging) {
    digitalWrite(24, HIGH);

    String value = (String)ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + pr + "," + us_ticker_read() + "\n";
    fwrite(value.c_str(), value.length(), 1, f);
    fflush(f);
    Serial.println("writing to flash");
    digitalWrite(24, LOW);

  } else {
    Serial.println((String)ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + pr + "," + us_ticker_read());
  }
}

void setup() {
  pinMode(24, OUTPUT);
  digitalWrite(24, LOW);
  Serial.begin(115200);
  Serial.println("Started");
  fs.mount(&bd);
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
  pinMode(transferButtonPin, INPUT_PULLUP);
  pinMode(pauseButtonPin, INPUT_PULLUP);

  int accelID = queue.call_every(1, checkAccel);
  int gyroID = queue.call_every(1, checkGyro);
  int baromID = queue.call_every(100, checkBarom);
  int printID = queue.call_every(1, printData, f);
  button.fall(queue.event(startStopLogging));
  button.rise(queue.event(writeFileBuffer));
  transferButton.fall(queue.event(transferData));
  pauseButton.fall(queue.event(pauseThread));
  Serial.println("setup complete");
  t1.start(callback(&queue, &EventQueue::dispatch_forever));
}

void loop() {}
