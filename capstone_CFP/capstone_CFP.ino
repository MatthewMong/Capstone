#include <mbed.h>
#include <rtos.h>
#include <EventQueue.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include <SPI.h>

const bool useBLE = false;
using namespace rtos;
using namespace events;

const int RX_BUFFER_SIZE = 256;
bool RX_BUFFER_FIXED_LENGTH = false;
const char* nameOfPeripheral = "testDevice";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
BLEService sendService(uuidOfService);
BLEFloatCharacteristic axChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic ayChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic azChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);

EventQueue queue(64 * EVENTS_EVENT_SIZE);

BLEDevice peripheral;

float gx, gy, gz, ax, ay, az, pr;

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

Thread t1;
Thread t3;
Thread t4;
Thread t5;


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

void printData(void) {
  if (useBLE) {
    BLEDevice central = BLE.central();
    if (central.connected()) {
      axChar.writeValue(ax);
      ayChar.writeValue(ay);
      azChar.writeValue(az);
    }
  } else {
    Serial.println((String)ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + "," + pr + "," + us_ticker_read());
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Started");

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
  if (useBLE) {
    if (!BLE.begin()) {
      Serial.println("* Starting Bluetooth® Low Energy module failed!");
      while (1)
        ;
    }
    Serial.println("Arduino Nano 33 BLE Sense (Central Device)");

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sendService);
    sendService.addCharacteristic(axChar);
    sendService.addCharacteristic(ayChar);
    sendService.addCharacteristic(azChar);

    BLE.addService(sendService);
    // rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);
    BLE.advertise();
  }
  queue.call_every(1, checkAccel);
  queue.call_every(1, checkGyro);
  queue.call_every(100, checkBarom);
  queue.call_every(1, printData);
  t1.start(callback(&queue, &EventQueue::dispatch_forever));
  // t3.start(mbed::callback(checkAccel));
  // t4.start(mbed::callback(checkBarom));
  // t5.start(mbed::callback(printData));
}

void loop() {}
