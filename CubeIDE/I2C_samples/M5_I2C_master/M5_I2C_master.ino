#include <M5Stack.h>
#include <Wire.h>

// 7bit slave address
#define NUCLEO_ADDRESS 0x01

void setup() {
  M5.begin();
  M5.Speaker.begin();
  M5.Speaker.mute();
  Wire.begin();
}

void loop() {
  // Send random data
  Wire.beginTransmission(NUCLEO_ADDRESS);
  uint8_t req = (uint8_t)random(0, 256);
  Wire.write(req);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("Error is detected in Wire.endTransmission()");
  }

  // Receive data
  uint8_t res = 0;
  Wire.requestFrom(NUCLEO_ADDRESS, 1);
  if (Wire.available() > 0) {
    res = Wire.read();
  }
  else {
    Serial.println("No data has come...");
  }

  // check data
  // The slave is loop back, so if I2C succeeds, the result will be 0.
  if (req == res) {
    Serial.println("I2C succeeds.");
    Serial.print("req: ");
    Serial.println(req);
    Serial.print("res: ");
    Serial.println(res);
  }
  else {
    Serial.println("Data came from slave device, but the value is wrong...");
  }
  delay(100);
}
