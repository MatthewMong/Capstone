#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>
#include <SPI.h>


const bool useBLE = false;
using namespace rtos;

const int RX_BUFFER_SIZE = 256;
bool RX_BUFFER_FIXED_LENGTH = false;
const char* nameOfPeripheral = "testDevice";
const char* uuidOfService = "0000181a-0000-1000-8000-00805f9b34fb";
const char* uuidOfTxChar = "00002a59-0000-1000-8000-00805f9b34fb";
BLEService sendService(uuidOfService);
BLEFloatCharacteristic axChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic ayChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);
BLEFloatCharacteristic azChar(uuidOfTxChar, BLERead | BLENotify | BLEBroadcast);

BLEDevice peripheral;

Semaphore s1(0);
Semaphore s2(1);
Semaphore s3(1);
Semaphore s4(1);

float gx, gy, gz, ax, ay, az, pr;

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

Thread t2;
Thread t3;
Thread t4;
Thread t5;


void checkGyro(void) {
  while (true) {
    s1.acquire();
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
    }
    s2.release();
  }
}

void checkAccel(void) {
  while (true) {
    s2.acquire();
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
    }
    s3.release();
  }
}

void checkBarom(void) {
  while (true) {
    s3.acquire();
    pr = lps35hw.readPressure();
    s4.release();
  }
}

void printData(void) {
  while (true) {
    s4.acquire();
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
    s1.release();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin() || !lps35hw.begin_I2C()) {
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
  t2.start(mbed::callback(checkGyro));
  t3.start(mbed::callback(checkAccel));
  t4.start(mbed::callback(checkBarom));
  t5.start(mbed::callback(printData));
}

void loop() {}
