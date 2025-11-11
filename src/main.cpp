#include <Arduino.h>
#include "BodenSensor.h"
#include <config.h>
#include <Wire.h>

void requestEvent() {
  Wire.beginTransmission(TARGET_ADDRESS);
  Wire.write(reinterpret_cast<byte*>(&BodenSensor::line.progress), sizeof(float));
  Wire.write(reinterpret_cast<byte*>(&BodenSensor::line.rot), sizeof(float));
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADRESS);
  Wire.onRequest(requestEvent);

  BodenSensor::initSensor();

  // some random blinking to show that the program is running
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
  digitalWrite(2, HIGH);
}

void loop() {
  BodenSensor::updateLine();

  delay(1);
}
