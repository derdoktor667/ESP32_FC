
#include <Arduino.h>
#include <ESP32_MPU6050.h>
#include <FlyskyIBUS.h>
#include <DShotRMT.h>

// Define IBUS RX Pin for Serial2 - YOU NEED TO ADJUST THIS TO YOUR SETUP
// Default RX pin for Serial2 on ESP32 is GPIO_NUM_16
const gpio_num_t IBUS_RX_PIN = GPIO_NUM_16;

ESP32_MPU6050 mpu;
FlyskyIBUS ibus(Serial2, IBUS_RX_PIN);

// Define ESC pins - YOU NEED TO ADJUST THESE TO YOUR SETUP
const gpio_num_t ESC_PIN_1 = GPIO_NUM_27;
const gpio_num_t ESC_PIN_2 = GPIO_NUM_25;
const gpio_num_t ESC_PIN_3 = GPIO_NUM_26;
const gpio_num_t ESC_PIN_4 = GPIO_NUM_33;

DShotRMT motor1(ESC_PIN_1, DSHOT300, false);
DShotRMT motor2(ESC_PIN_2, DSHOT300, false);
DShotRMT motor3(ESC_PIN_3, DSHOT300, false);
DShotRMT motor4(ESC_PIN_4, DSHOT300, false);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect. Needed for native USB

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized successfully!");

  Serial.println("Initializing FlyskyIBUS...");
  ibus.begin(); // No arguments needed for begin()
  Serial.println("FlyskyIBUS initialized successfully!");

  Serial.println("Initializing DShotRMT...");
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  Serial.println("DShotRMT initialized successfully!");

  // Send zero throttle to all motors to arm them (DShot requires this)
  motor1.sendThrottle(0);
  motor2.sendThrottle(0);
  motor3.sendThrottle(0);
  motor4.sendThrottle(0);
  delay(100);

}

//
void loop() {
  // Read MPU6050 data
  mpu.update();
  // Access data like: mpu.readings.accelerometer.x, mpu.readings.gyroscope.y

  // Read IBUS data
  // The IBUS library handles reading in the background. Just get the channel values.
  // Example: Print channel 0 value
  // Serial.print("Channel 0: ");
  // Serial.println(ibus.getChannel(0));

  // Example: Map IBUS channel 0 (throttle) to DShot throttle
  // int throttle = map(ibus.getChannel(0), 1000, 2000, 0, 1000); // Map 1000-2000 to 0-1000 (DShot throttle range)
  // motor1.sendThrottle(throttle);
  // motor2.sendThrottle(throttle);
  // motor3.sendThrottle(throttle);
  // motor4.sendThrottle(throttle);

  // put your main code here, to run repeatedly:

}
