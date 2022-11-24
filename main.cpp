#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_LPS35HW.h>

using namespace rtos;

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
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(pr);
    Serial.print(",");
    Serial.println(us_ticker_read());
    wait_us(10000);
    s1.release();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin() || !lps35hw.begin_I2C()) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  t2.start(mbed::callback(checkGyro));
  t3.start(mbed::callback(checkAccel));
  t4.start(mbed::callback(checkBarom));
  t5.start(mbed::callback(printData));
}

void loop() {}
