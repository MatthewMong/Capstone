#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
#include <Arduino_LSM9DS1.h>

using namespace rtos;

Semaphore s1(1);
Semaphore s2(0);
float gx, gy, gz, ax, ay, az;

Thread t2;
Thread t3;
Thread t4;


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
    s1.acquire();
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
    }
    s2.release();
  }
}

void printData(void) {
  while (true) {
    s2.acquire();
    Serial.println("Accel data: ");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.println(az);

    Serial.println("gyro data: ");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.println(gz);

    Serial.println((String)"Time is: " + us_ticker_read());
    wait_us(100000);
    s1.release();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
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
  t4.start(mbed::callback(printData));
}

void loop() {}
